#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Phone GPS: the iPhone's GPS fixes, which an app on the phone shares as NMEA 0183 over the phone's Wi-Fi hotspot, published
as gpsLocation for navigation. Where the comma's own GPS hears little in the car, navigation then follows the phone, and the
u-blox again whenever the phone's fixes stop (latest_gps_source).
"""
import calendar
import datetime
import errno
import math
import re
import select
import socket
import struct
import time
from dataclasses import dataclass
from functools import reduce
from ipaddress import IPv4Address
from operator import xor

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.navd.navweb import WIFI_INTERFACE, wifi_address

DEFAULT_PORT = 11123  # GPS2IP's socket mode
ROUTE_FILE = '/proc/net/route'
RTF_UP, RTF_GATEWAY = 0x1, 0x2  # a route's flags there

# Data is taken only from the phone: the Wi-Fi's default gateway, which on the iPhone's hotspot is the phone itself, or
# PhoneGpsHost. Another device on the same network can't feed a position.
LOOKUP_PERIOD = 2.  # s between reads of the params, the Wi-Fi's gateway and the comma's address on it, which change as Wi-Fi reconnects
CONNECT_TIMEOUT = 3.  # s
RETRY_DELAYS = (1., 2., 4., 8.)  # s before each new attempt to connect after a failure, the last one repeating
STABLE_TIME = 10.  # s connected, after which a lost connection is retried from the first delay again
SILENCE_TIMEOUT = 10.  # s connected without data, after which the connection is taken for lost
MAX_LINE = 256  # bytes of a line still without its end: NMEA's are at most 82, so a longer one is garbage
MAX_READ = 65536  # bytes read from each source per step, so that a flood can't hold up the loop
MAX_DATAGRAM = 2048
FAILURES = {errno.ECONNREFUSED: 'refused', errno.ENETUNREACH: 'unreachable', errno.EHOSTUNREACH: 'unreachable', errno.EHOSTDOWN: 'unreachable',
            errno.ETIMEDOUT: 'timeout', errno.ECONNRESET: 'closed', errno.EPIPE: 'closed'}

KNOTS = 1852 / 3600  # m/s
NUMBER = re.compile(r'-?(\d+(\.\d*)?|\.\d+)')
TIME_OF_DAY = re.compile(r'(\d{2})(\d{2})(\d{2}(\.\d+)?)')
DATE = re.compile(r'(\d{2})(\d{2})(\d{2})')
NO_FIX_MODES = ('N', 'E', 'M', 'S')  # RMC's and VTG's mode: not valid, dead reckoning, manual input, simulator
FIX_QUALITIES = (1, 2, 3, 4, 5)  # GGA's: GPS, differential, PPS, RTK and float RTK. 0 is none, 6-8 dead reckoning, manual, simulator
NOMINAL_ERROR = 5.  # m of horizontal error per unit of HDOP: a consumer receiver's range error
MIN_ACCURACY = 3.  # m; no phone in a car does better
VDOP_RATIO = 1.5  # GGA has only the HDOP, and the VDOP is usually about 1.5 times it
# A phone that lost the sky still reports a position, from Wi-Fi or cell towers, tens or hundreds of meters off: never one
# to turn by. Its HDOP grows with it.
MAX_HDOP = 10.  # 50 m and worse
# NMEA carries no accuracy for the speed or the course, so a phone's is assumed. A course comes from the velocity, so its
# error grows as the car slows: at 2 m/s it's about 20 degrees, which navd's course rules widen their thresholds for
# (bearingAccuracyDeg). It is never 0 or 180, which navd takes for a course the receiver doesn't rate, taken at its word.
SPEED_ERROR = .7  # m/s
MAX_SPEED = 90.  # m/s; faster is no car
COURSE_MIN_SPEED = 1.  # m/s; slower, the course is noise, and the moving car's last one is kept: a stopped car still heads that way
GGA_MEMORY = 3.  # s a GGA's HDOP, satellites and altitude stand for an RMC whose own GGA hasn't come
VTG_WINDOW = .5  # s between an RMC and the VTG that gives its missing speed or course
FLAG_FIX, FLAG_GGA = 1, 2  # gpsLocation flags: a fix (as the u-blox's gnssFixOK), and the HDOP, satellites and altitude its own epoch's

# The phone's clock and the comma's can differ, by any amount when the comma has had no network time yet. So the fix's own
# time is used only to tell old data, held up and sent at once, from new, comparing it with when it arrived, on the
# comma's monotonic clock, and never to say how old a fix is: a fix is as old as it is since it arrived, as navd counts from
# the message's logMonoTime. A clock difference never moves a position along the route.
CLOCK_AGREEMENT = 1.  # s; the fix's own time goes out as unixTimestampMillis only this close to the comma's clock, else the arrival's
# A fix that arrives later than the phone's fastest lately was held up on the way: where it was taken, the car no longer is.
# The phone sends a fix about every second, and when it sends on a timer, a fix is up to a second old as it leaves.
MAX_LATE = 1.  # s
DRIFT = 1e-3  # s per s that the fastest lately may rise by, as the two clocks drift apart (at most about 1e-4)
LATE_RESTART = 10  # late fixes in a row, arriving as fixes do: the phone's clock went back, and the fixes start over
RESTART_TIME = 5.  # s without a published fix, after which two fixes in a row must agree before one goes out again
PAIR_GAP = 2.5  # s between those two fixes, at most
PAIR_JITTER = .5  # s by which they may arrive closer together or farther apart than they were taken
CLOCK_STEP = 5.  # s the phone's time may go back by before its fixes start over
JUMP_SPEED = 1.5  # times the faster of two fixes' speeds, that the car may have driven between them
JUMP_MARGIN = 30.  # m besides, for the positions' own errors
EARTH_RADIUS = 6371000.  # m
STATUS_WINDOW = 3.  # s of what came in that the status tells
STATUS_PERIOD = 1.  # s between writes of the status, at least
# What came in, the most telling first: the status names the first heard within STATUS_WINDOW
DATA_STATES = ('fixes', 'starting', 'inaccurate', 'late', 'jump', 'incomplete', 'no_fix', 'no_rmc', 'unreadable')


@dataclass
class Fix:
  """A position from the phone: its RMC, joined by the GGA and VTG of its epoch."""
  time: float  # s since 1970, UTC: the phone's time of the fix
  latitude: float  # deg
  longitude: float  # deg
  speed: float | None  # m/s
  course: float | None  # deg, where reported
  received: float = 0.  # monotonic s it arrived
  wall: float = 0.  # s since 1970 on the comma's clock as it arrived
  hdop: float | None = None
  satellites: int | None = None
  altitude: float | None = None  # m above the WGS 84 ellipsoid
  joined: bool = False  # its own epoch's GGA joined it


@dataclass
class Gga:
  time: float  # s of the day, UTC
  fix: bool
  satellites: int | None = None
  hdop: float | None = None
  altitude: float | None = None  # m above the WGS 84 ellipsoid


def sentence(line: bytes) -> tuple[str, list[str]] | None:
  """An NMEA 0183 sentence's type, such as RMC, and its fields, from any talker (GP, GN, GL...): None for a line that isn't
  one with its checksum right. A line may begin with the tail of a sentence cut off, so the last '$' starts it."""
  start = line.rfind(b'$')
  if start < 0 or len(line) > MAX_LINE:
    return None
  body, star, check = line[start + 1:].partition(b'*')
  if not star or re.fullmatch(rb'[0-9A-Fa-f]{2}', check.strip()) is None or reduce(xor, body, 0) != int(check.strip(), 16):
    return None
  try:
    text = body.decode('ascii')
  except UnicodeDecodeError:
    return None
  fields = text.split(',')
  address = fields[0]
  # the talker, then the type; a proprietary sentence starts with P
  if not text.isprintable() or len(address) != 5 or not (address.isalpha() and address.isupper()) or address.startswith('P'):
    return None
  return address[2:], fields[1:]


def number(text: str) -> float | None:
  value = float(text) if NUMBER.fullmatch(text) else None
  return value if value is not None and math.isfinite(value) else None


def time_of_day(text: str) -> float | None:
  """hhmmss.ss as seconds of the day, else None."""
  match = TIME_OF_DAY.fullmatch(text)
  if match is None or int(match[1]) > 23 or int(match[2]) > 59 or float(match[3]) >= 61:  # 60: a leap second
    return None
  return int(match[1]) * 3600 + int(match[2]) * 60 + float(match[3])


def date(text: str) -> float | None:
  """ddmmyy as the seconds since 1970 of its midnight, UTC, else None."""
  match = DATE.fullmatch(text)
  try:
    return None if match is None else calendar.timegm(datetime.date(2000 + int(match[3]), int(match[2]), int(match[1])).timetuple())
  except ValueError:  # no such day
    return None


def coordinate(text: str, hemisphere: str, hemispheres: str, limit: float) -> float | None:
  """An NMEA latitude (ddmm.mmmm, N or S) or longitude (dddmm.mmmm, E or W) in signed degrees, else None."""
  value = number(text)
  if value is None or value < 0 or len(hemisphere) != 1 or hemisphere not in hemispheres:
    return None
  degrees = math.floor(value / 100)
  minutes = value - 100 * degrees
  angle = degrees + minutes / 60
  if minutes >= 60 or angle > limit:
    return None
  return angle if hemisphere == hemispheres[0] else -angle


def position(fields: list[str]) -> tuple[float, float] | str:
  """(latitude, longitude) from the four fields of an RMC or a GGA, else why not: no_fix for none, unreadable for a malformed one."""
  if not any(fields):
    return 'no_fix'
  latitude, longitude = coordinate(fields[0], fields[1], 'NS', 90.), coordinate(fields[2], fields[3], 'EW', 180.)
  if latitude is None or longitude is None:
    return 'unreadable'
  return 'no_fix' if (latitude, longitude) == (0., 0.) else (latitude, longitude)  # what some apps send without a fix


def speed_and_course(knots: str, course: str) -> tuple[float | None, float | None] | str:
  """Speed (m/s) and course (deg) where reported, from knots and degrees true, else unreadable for an impossible one. A
  negative value is the phone's own for none."""
  speed, heading = number(knots), number(course)
  if speed is not None and speed * KNOTS > MAX_SPEED:
    return 'unreadable'
  speed = None if speed is None or speed < 0 else speed * KNOTS
  heading = None if heading is None or not 0 <= heading <= 360 else heading % 360
  return speed, heading


def parse_rmc(fields: list[str]) -> Fix | str:
  """$--RMC,time,status,lat,N/S,lon,E/W,knots,course,date,variation,E/W[,mode[,status]]: a fix, else why not."""
  if len(fields) < 9:
    return 'unreadable'
  if fields[1] != 'A' or (len(fields) > 11 and fields[11] in NO_FIX_MODES):
    return 'no_fix'
  day, midnight, where, motion = time_of_day(fields[0]), date(fields[8]), position(fields[2:6]), speed_and_course(fields[6], fields[7])
  if day is None or midnight is None or isinstance(motion, str):
    return 'unreadable'
  if isinstance(where, str):
    return where
  return Fix(midnight + day, *where, *motion)


def parse_gga(fields: list[str]) -> Gga | str:
  """$--GGA,time,lat,N/S,lon,E/W,quality,satellites,HDOP,altitude,M,geoid separation,M,...: the epoch's quality, satellites,
  HDOP and altitude, else unreadable."""
  if len(fields) < 8:
    return 'unreadable'
  day, quality = time_of_day(fields[0]), number(fields[5])
  if day is None or quality is None:
    return 'unreadable'
  if quality not in FIX_QUALITIES or isinstance(position(fields[1:5]), str):
    return Gga(day, False)
  satellites, hdop = number(fields[6]), number(fields[7])
  # above mean sea level, and the geoid above the ellipsoid
  altitude, separation = number(fields[8]) if len(fields) > 8 else None, (number(fields[10]) if len(fields) > 10 else None) or 0.
  return Gga(day, True, int(satellites) if satellites is not None and satellites.is_integer() and 0 <= satellites < 100 else None,
             hdop if hdop is not None and 0 < hdop < 100 else None,
             altitude + separation if altitude is not None and -1000 < altitude + separation < 10000 else None)


def parse_vtg(fields: list[str]) -> tuple[float | None, float | None] | None:
  """$--VTG,course,T,course,M,knots,N,km/h,K[,mode]: speed (m/s) and course (deg) where reported, else None."""
  if len(fields) < 8 or (len(fields) > 8 and fields[8] in NO_FIX_MODES):
    return None
  motion = speed_and_course(fields[4], fields[0])
  if isinstance(motion, str):
    return None
  speed, course = motion
  kph = number(fields[6])
  if speed is None and kph is not None and 0 <= kph / 3.6 <= MAX_SPEED:
    speed = kph / 3.6
  return speed, course


def distance(a: Fix, b: Fix) -> float:
  """m between two fixes, close together."""
  east = (b.longitude - a.longitude + 180) % 360 - 180
  return EARTH_RADIUS * math.hypot(math.radians(b.latitude - a.latitude), math.radians(east) * math.cos(math.radians((a.latitude + b.latitude) / 2)))


def jumped(a: Fix, b: Fix) -> bool:
  """Whether fix b is farther from fix a than the car could have driven between them."""
  assert a.speed is not None and b.speed is not None
  return distance(a, b) > JUMP_SPEED * max(a.speed, b.speed) * max(b.time - a.time, 0.) + JUMP_MARGIN


def consistent(a: Fix, b: Fix) -> bool:
  """Whether fix b followed fix a as fixes do as they come: soon after, arriving as much later as it was taken, and no farther
  than the car could have driven. Data held up and sent at once arrives all together."""
  taken = b.time - a.time
  return 0 < taken <= PAIR_GAP and abs(b.received - a.received - taken) <= PAIR_JITTER and not jumped(a, b)


def default_gateway(route: str, interface: str = WIFI_INTERFACE) -> str | None:
  """The IPv4 address of an interface's default gateway, from the text of /proc/net/route, else None. On the iPhone's
  hotspot, that is the phone."""
  best: tuple[int, int] | None = None  # (metric, gateway)
  for line in route.splitlines()[1:]:
    fields = line.split()
    if len(fields) < 8 or fields[0] != interface:
      continue
    try:
      destination, gateway, flags, metric, mask = int(fields[1], 16), int(fields[2], 16), int(fields[3], 16), int(fields[6]), int(fields[7], 16)
    except ValueError:
      continue
    if destination == 0 and mask == 0 and gateway != 0 and flags & RTF_UP and flags & RTF_GATEWAY and (best is None or metric < best[0]):
      best = (metric, gateway)
  # the kernel prints the address as the number it keeps in network byte order: its bytes, in the machine's own order
  return None if best is None else socket.inet_ntoa(struct.pack('=I', best[1] & 0xFFFFFFFF))


def port_param(params) -> int:
  try:
    port = int(params.get("PhoneGpsPort", return_default=True))
  except (TypeError, ValueError):
    return DEFAULT_PORT
  return port if 0 < port < 65536 else DEFAULT_PORT


def host_param(params) -> str | None:
  """PhoneGpsHost, an IPv4 address in place of the Wi-Fi's gateway, else None, also for what isn't one."""
  try:
    return str(IPv4Address(str(params.get("PhoneGpsHost") or '').strip()))
  except ValueError:
    return None


class PhoneFixes:
  """Fixes from the phone's NMEA lines, each one checked before it goes out: an epoch's sentences joined, a fix only once,
  never one older than the last, nor one held up on the way, jumping farther than the car could drive, or without the
  speed and course that navd's timing and course rules need."""

  def __init__(self):
    self.pending: Fix | None = None  # the newest RMC with a fix since the last take
    self.ggas: list[tuple[Gga, float]] = []  # the last few seconds' GGAs, with when each arrived
    self.vtg: tuple[float | None, float | None, float] | None = None  # (speed, course, when it arrived) of the last VTG with a fix
    self.published: Fix | None = None
    self.candidate: Fix | None = None  # the first of the two fixes that must agree as the fixes start
    self.baseline: float | None = None  # s; the least by which a fix lately arrived after it was taken, between the two clocks
    self.baseline_time = 0.
    self.late = 0  # late fixes in a row
    self.course: float | None = None  # deg; the moving car's last
    self.heard: dict[str, float] = {}  # when each of DATA_STATES, and any RMC or other sentence, last came in

  def add(self, line: bytes, received: float, wall: float) -> None:
    """A line that arrived at monotonic time received, which was wall on the comma's clock."""
    line = line.strip()
    if not line:
      return
    parsed = sentence(line)
    if parsed is None:
      self.heard['unreadable'] = received
      return
    kind, fields = parsed
    if kind == 'RMC':
      self.heard['rmc'] = received
      fix = parse_rmc(fields)
      if isinstance(fix, str):
        self.heard[fix] = received
        if fix == 'no_fix':  # the phone lost its fix since
          self.pending = None
      elif self.pending is None or fix.time > self.pending.time:
        fix.received, fix.wall = received, wall
        self.pending = fix
    elif kind == 'GGA':
      gga = parse_gga(fields)
      if isinstance(gga, str):
        self.heard[gga] = received
        return
      self.heard['nmea' if gga.fix else 'no_fix'] = received
      self.ggas = [(g, at) for g, at in self.ggas if received - at < GGA_MEMORY] + [(gga, received)]
    elif kind == 'VTG':
      self.heard['nmea'] = received
      vtg = parse_vtg(fields)
      self.vtg = None if vtg is None else (*vtg, received)
    else:  # GSA, GSV, GLL and the like say nothing that RMC and GGA don't
      self.heard['nmea'] = received

  def take(self, now: float):
    """The gpsLocation message of the newest fix since the last take, if it passes its checks, else None."""
    fix, self.pending = self.pending, None
    if fix is None:
      return None
    reason = self.join(fix) or self.check(fix)
    if reason is not None:
      if reason != 'old':  # a repeat is no news
        self.heard[reason] = now
      return None
    self.heard['fixes'] = now
    return self.message(fix)

  def join(self, fix: Fix) -> str | None:
    """Joins its epoch's GGA, else the last recent one, and its VTG to a fix: None for one to check further, else why not."""
    day = fix.time % 86400
    own = [g for g, _ in self.ggas if abs(g.time - day) < .005]
    if own and not own[-1].fix:
      return 'no_fix'
    recent = [g for g, at in self.ggas if g.fix and fix.received - at < GGA_MEMORY]
    gga = own[-1] if own else recent[-1] if recent else None
    if gga is not None:
      fix.hdop, fix.satellites, fix.altitude, fix.joined = gga.hdop, gga.satellites, gga.altitude, bool(own)
    if self.vtg is not None and abs(fix.received - self.vtg[2]) <= VTG_WINDOW:
      fix.speed = self.vtg[0] if fix.speed is None else fix.speed
      fix.course = self.vtg[1] if fix.course is None else fix.course
    # navd counts the car's travel since a fix by its speed, and its course rules need the course of a moving car
    if fix.speed is None or (fix.course is None and fix.speed >= COURSE_MIN_SPEED):
      return 'incomplete'
    if fix.hdop is not None and fix.hdop > MAX_HDOP:
      return 'inaccurate'
    return None

  def restart(self) -> None:
    self.published = self.candidate = self.baseline = None
    self.late = 0

  def check(self, fix: Fix) -> str | None:
    """None for a fix to publish, else why not: old, late, jump, or starting while the first two fixes don't agree yet."""
    last = self.published
    if last is not None and fix.time <= last.time:
      if fix.time >= last.time - CLOCK_STEP:
        return 'old'
      self.restart()  # the phone's clock went back: its fixes start over
      last = None
    elif last is not None and fix.time - last.time > fix.received - last.received + MAX_LATE:
      self.restart()  # its time ran ahead of the time it took to come: the phone's clock jumped, or this time is wrong
      last = None
    offset = fix.received - fix.time
    if self.baseline is not None:
      self.baseline, self.baseline_time = min(self.baseline + DRIFT * (fix.received - self.baseline_time), offset), fix.received
      if offset - self.baseline > MAX_LATE:
        self.late += 1
        if self.late >= LATE_RESTART:
          self.restart()
        return 'late'
    self.late = 0
    if last is None or fix.received - last.received > RESTART_TIME:
      # The first fix after a while may be one the phone kept from before, or one of several held up on the way, so it goes
      # out only once a second one agrees with it
      previous, self.candidate = self.candidate, fix
      if previous is None or not consistent(previous, fix):
        return 'starting'
      if self.baseline is None:
        self.baseline, self.baseline_time = min(previous.received - previous.time, offset), fix.received
    elif jumped(last, fix):
      return 'jump'
    self.published, self.candidate = fix, None
    return None

  def message(self, fix: Fix):
    assert fix.speed is not None
    if fix.course is not None and fix.speed >= COURSE_MIN_SPEED:
      self.course = fix.course
    course = self.course if self.course is not None else fix.course or 0.
    accuracy = max(NOMINAL_ERROR * fix.hdop, MIN_ACCURACY) if fix.hdop is not None else 0.  # 0: not known
    msg = messaging.new_message('gpsLocation', valid=True)
    gps = msg.gpsLocation
    gps.source = log.GpsLocationData.SensorSource.iOS
    gps.hasFix = True
    gps.flags = FLAG_FIX | (FLAG_GGA if fix.joined else 0)
    gps.latitude, gps.longitude, gps.altitude = fix.latitude, fix.longitude, fix.altitude or 0.
    gps.speed, gps.speedAccuracy = fix.speed, SPEED_ERROR
    gps.bearingDeg, gps.bearingAccuracyDeg = course, math.degrees(math.atan2(SPEED_ERROR, fix.speed))
    gps.horizontalAccuracy, gps.verticalAccuracy = accuracy, VDOP_RATIO * accuracy
    gps.satelliteCount = fix.satellites or 0
    gps.vNED = [fix.speed * math.cos(math.radians(course)), fix.speed * math.sin(math.radians(course)), 0.]
    gps.unixTimestampMillis = round(1000 * (fix.time if abs(fix.time - fix.wall) <= CLOCK_AGREEMENT else fix.wall))
    return msg

  def data_state(self, now: float) -> str | None:
    """What came in lately, as one of DATA_STATES, else None for nothing."""
    recent = {kind for kind, at in self.heard.items() if now - at < STATUS_WINDOW}
    if 'rmc' not in recent and 'nmea' in recent:  # NMEA without RMC, the only sentence with both the speed and the date
      recent.add('no_rmc')
    return next((state for state in DATA_STATES if state in recent), None)


class PhoneGps:
  """phonegpsd: a TCP connection to the phone and a UDP socket on the comma's Wi-Fi address, both on PhoneGpsPort, taking
  data only from the phone, and publishing each fix that passes as it arrives. Its state goes to PhoneGpsStatus for the
  navigation page."""

  def __init__(self, params, pm, route_file: str = ROUTE_FILE, wifi=wifi_address):
    self.params, self.pm, self.route_file, self.wifi = params, pm, route_file, wifi
    self.fixes = PhoneFixes()
    self.phone: str | None = None  # the only address data is taken from
    self.address: str | None = None  # the comma's own on Wi-Fi
    self.port = DEFAULT_PORT
    self.lookup_time = -math.inf
    self.tcp: socket.socket | None = None
    self.connected = False
    self.connect_deadline = 0.
    self.connected_time = 0.
    self.tcp_heard = 0.  # when data last came over the connection
    self.buffer = b''  # the start of a line, until its end comes
    self.retry_time = 0.
    self.retries = 0  # failed attempts to connect, in a row
    self.problem: str | None = None  # why the last attempt failed, or the last connection ended
    self.udp: socket.socket | None = None
    self.udp_heard = -math.inf
    self.ignored = False  # a datagram from another address was logged
    self.udp_failed: tuple[str, int] | None = None  # (address, port) a UDP socket couldn't be bound to, logged once
    self.status: dict | None = None
    self.status_time = -math.inf

  def lookup(self, now: float) -> None:
    """Where the phone is: PhoneGpsHost, else the Wi-Fi's default gateway, while the comma has an address on Wi-Fi."""
    self.lookup_time = now + LOOKUP_PERIOD
    port, host = port_param(self.params), host_param(self.params)
    try:
      with open(self.route_file) as f:
        route = f.read()
    except OSError:
      route = ''
    address = self.wifi()
    phone = None if address is None else host or default_gateway(route)
    if self.udp is not None and (address, port) != (self.address, self.port):
      self.udp.close()
      self.udp = None
    if self.udp is None and address is not None:
      self.udp = self.open_udp(address, port)
    if (phone, address, port) != (self.phone, self.address, self.port):
      cloudlog.warning(f"phonegpsd: the phone is {phone} port {port}, the comma {address}")
      self.close_tcp()
      self.phone, self.address, self.port = phone, address, port
      self.retry_time, self.retries, self.problem = now, 0, None

  def open_udp(self, address: str, port: int) -> socket.socket | None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
      sock.setblocking(False)
      sock.bind((address, port))  # the Wi-Fi's address only: never the cellular link's
    except OSError as e:
      if self.udp_failed != (address, port):
        cloudlog.warning(f"phonegpsd: no UDP on {address} port {port}: {e}")
        self.udp_failed = (address, port)
      sock.close()
      return None
    return sock

  def close_tcp(self) -> None:
    if self.tcp is not None:
      self.tcp.close()
    self.tcp, self.connected, self.buffer = None, False, b''

  def connect(self, now: float) -> None:
    assert self.phone is not None and self.address is not None
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      sock.setblocking(False)
      sock.bind((self.address, 0))  # from the Wi-Fi's address, never over the cellular link
      error = sock.connect_ex((self.phone, self.port))
    except OSError as e:
      error = e.errno
    if error in (0, errno.EINPROGRESS, errno.EAGAIN, errno.EALREADY):
      self.tcp, self.connected, self.connect_deadline = sock, False, now + CONNECT_TIMEOUT
    else:
      sock.close()
      self.failed(now, FAILURES.get(error, 'failed'))

  def failed(self, now: float, problem: str) -> None:
    """Schedules the next attempt to connect, later after each failure in a row."""
    self.problem = problem
    self.retry_time = now + RETRY_DELAYS[min(self.retries, len(RETRY_DELAYS) - 1)]
    self.retries += 1

  def lost(self, now: float, problem: str) -> None:
    if now - self.connected_time >= STABLE_TIME:
      self.retries = 0
    cloudlog.warning(f"phonegpsd: connection to the phone lost: {problem}")
    self.close_tcp()
    self.failed(now, problem)

  def read_tcp(self, now: float, wall: float) -> None:
    assert self.tcp is not None
    total = 0
    while total < MAX_READ:
      try:
        data = self.tcp.recv(4096)
      except (BlockingIOError, InterruptedError):
        return
      except OSError as e:
        self.lost(now, FAILURES.get(e.errno or 0, 'closed'))
        return
      if not data:
        self.lost(now, 'closed')
        return
      total += len(data)
      self.tcp_heard = now
      # a line may come in pieces: its start waits in the buffer for the rest
      *lines, self.buffer = re.split(rb'[\r\n]', self.buffer + data)
      if len(self.buffer) > MAX_LINE:
        self.buffer = b''
        self.fixes.heard['unreadable'] = now
      for line in lines:
        self.fixes.add(line, now, wall)

  def read_udp(self, now: float, wall: float) -> None:
    assert self.udp is not None
    for _ in range(MAX_READ // MAX_DATAGRAM):
      try:
        data, sender = self.udp.recvfrom(MAX_DATAGRAM)
      except OSError:  # nothing more to read, also BlockingIOError
        return
      if sender[0] != self.phone:  # only the phone's: another device on the network can't feed a position
        if not self.ignored:
          cloudlog.warning(f"phonegpsd: ignoring UDP from {sender[0]}, not the phone {self.phone}")
          self.ignored = True
        continue
      self.udp_heard = now
      for line in re.split(rb'[\r\n]', data):  # each datagram stands alone: a line cut off in it ends there
        self.fixes.add(line, now, wall)

  def link(self, now: float) -> str:
    """How the comma is linked to the phone, for the status."""
    if self.connected:
      return 'connected'
    if now - self.udp_heard < STATUS_WINDOW:
      return 'udp'
    if self.address is None:
      return 'no_wifi'
    if self.phone is None:
      return 'no_gateway'
    return self.problem or 'connecting'

  def write_status(self, now: float) -> None:
    status = {'host': self.phone, 'port': self.port, 'link': self.link(now), 'data': self.fixes.data_state(now)}
    if status != self.status and now - self.status_time >= STATUS_PERIOD:
      self.status, self.status_time = status, now
      try:
        self.params.put("PhoneGpsStatus", status)
      except Exception:
        cloudlog.exception("phonegpsd.status_failed")

  def step(self, timeout: float = 1.) -> None:
    """Waits for data up to timeout, reads it, and publishes the fix it brings, if any."""
    now = time.monotonic()
    if now >= self.lookup_time:
      self.lookup(now)
    if self.tcp is None and self.phone is not None and now >= self.retry_time:
      self.connect(now)
    connecting = self.tcp is not None and not self.connected
    deadlines = [self.lookup_time, self.connect_deadline if connecting else math.inf,
                 self.retry_time if self.tcp is None and self.phone is not None else math.inf]
    readers = [sock for sock in (self.tcp if self.connected else None, self.udp) if sock is not None]
    readable, writable, _ = select.select(readers, [self.tcp] if connecting else [], [], min(timeout, max(min(deadlines) - now, 0.)))
    now, wall = time.monotonic(), time.time()  # noqa: TID251  the comma's clock, only to tell whether the phone's agrees
    if connecting:
      assert self.tcp is not None
      error = self.tcp.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR) if writable else None
      if error == 0:
        cloudlog.warning(f"phonegpsd: connected to the phone at {self.phone} port {self.port}")
        self.connected, self.connected_time, self.tcp_heard, self.problem = True, now, now, None
      elif error is not None or now >= self.connect_deadline:
        self.close_tcp()
        self.failed(now, FAILURES.get(error, 'failed') if error is not None else 'timeout')
    elif self.connected:
      if self.tcp in readable:
        self.read_tcp(now, wall)
      elif now - self.tcp_heard > SILENCE_TIMEOUT:  # gone without closing, as when the phone leaves
        self.lost(now, 'silent')
    if self.udp is not None and self.udp in readable:
      self.read_udp(now, wall)
    message = self.fixes.take(now)
    if message is not None:
      self.pm.send('gpsLocation', message)
    self.write_status(now)

  def close(self) -> None:
    self.close_tcp()
    if self.udp is not None:
      self.udp.close()
      self.udp = None


def main() -> None:
  params = Params()
  phone = PhoneGps(params, messaging.PubMaster(['gpsLocation']))
  try:
    while True:
      phone.step()
  finally:
    phone.close()
    params.remove("PhoneGpsStatus")


if __name__ == "__main__":
  main()
