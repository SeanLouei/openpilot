#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A page on the device's Wi-Fi for setting the navigation destination and Mapbox token from a phone.
"""
import fcntl
import http.client
import io
import ipaddress
import json
import math
import re
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import NamedTuple
from urllib.parse import SplitResult, parse_qs, unquote_plus, urljoin, urlsplit

import requests

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source

PORT = 8082
WIFI_INTERFACE = 'wlan0'  # the comma's Wi-Fi, also its hotspot's; its cellular link is another interface
SIOCGIFADDR = 0x8915  # Linux ioctl: an interface's IPv4 address
LOOPBACK = '127.0.0.1'
MAX_BODY = 4096
MAX_HEADERS = 16384  # bytes of request headers: a browser sends about 1 KB, and the stdlib alone allows 100 lines of 64 KB
MAX_CONNECTIONS = 12  # open at once: a phone's browser opens a few
MAX_CONNECTION_TIME = 15.  # s from accept to close, even while the client keeps sending
GEOCODE_URL = "https://api.mapbox.com/search/geocode/v6/forward"
SHORT_LINK_HOSTS = ('maps.app.goo.gl', 'maps.apple')  # shared map links that redirect to one with the place in it
MAX_REDIRECTS = 5

LINK = re.compile(r'https?://\S+')
COORDINATES = re.compile(r'^\s*(-?\d{1,2}\.\d+)\s*,\s*(-?\d{1,3}\.\d+)\s*$')
GOOGLE_PLACE = re.compile(r'!3d(-?\d{1,2}\.\d+)!4d(-?\d{1,3}\.\d+)')  # the place itself, not the map's viewport
GOOGLE_STOP = re.compile(r'!1d(-?\d{1,3}\.\d+)!2d(-?\d{1,2}\.\d+)')  # a directions stop, longitude first
GOOGLE_VIEWPORT = re.compile(r'@(-?\d{1,2}\.\d+),(-?\d{1,3}\.\d+)')
COORDINATE_KEYS = ('destination', 'daddr', 'q', 'query', 'coordinate', 'll')  # best first: Google's ll is the map's center
TEXT_KEYS = ('destination', 'daddr', 'q', 'query', 'address')

OFFLINE = "The device is offline: connect it to the internet and try again"
LINK_FAILED = "Couldn't open that link: copy it from your maps app again, or paste the place's coordinates"
MAPBOX_ERRORS = {
  401: "The Mapbox token is missing or invalid: save a valid one below",
  403: "The Mapbox token is missing or invalid: save a valid one below",
  422: "Mapbox couldn't read that address: shorten it and try again",
  429: "Too many searches: try again shortly",
}
MAPBOX_FAILED = "Mapbox isn't answering right now: try again shortly"

GPS_SERVICES = ('gpsLocation', 'gpsLocationExternal')
SERVICES = [*GPS_SERVICES, 'navInstruction', 'managerState', 'ubloxGnss']
# Comma devices with a u-blox chip on /dev/ttyHS0 run pigeond, which talks to the chip, and ubloxd, which decodes what it
# sends; the others run qcomgpsd, for the GPS in the Quectel modem. The manager runs them only while the car is on.
RECEIVERS = {'u-blox': (('pigeond', 'ubloxd'), 'gpsLocationExternal'), 'Quectel modem': (('qcomgpsd',), 'gpsLocation')}
GPS_PROCESSES = tuple(name for names, _ in RECEIVERS.values() for name in names)
ANTENNA = {'ok': 'antenna OK', 'short': 'antenna short-circuited', 'open': 'antenna not connected',  # what the u-blox's antenna check says
           'init': 'antenna check starting', 'dontknow': "antenna check not available on this chip"}
ANTENNA_POWER = {'on': 'antenna powered', 'off': 'antenna not powered', 'dontknow': 'antenna power unknown'}
AGC_FULL_SCALE = 8191  # the u-blox's gain control count at full scale: the weaker the signal from the antenna, the higher


class Place(NamedTuple):
  coordinates: tuple[float, float] | None
  name: str = ''  # what the link calls the place, to show
  query: str = ''  # what to look up when the link has no coordinates


class LinkError(Exception):
  """A share link that didn't lead to a map link."""


def valid_coordinates(latitude: float, longitude: float) -> bool:
  return -90 <= latitude <= 90 and -180 <= longitude <= 180 and (latitude, longitude) != (0, 0)


def coordinates_in(text: str) -> tuple[float, float] | None:
  """(latitude, longitude) from 'lat, lon', else None."""
  match = COORDINATES.match(text)
  if match and valid_coordinates(float(match[1]), float(match[2])):
    return float(match[1]), float(match[2])
  return None


def short_link(url: str) -> bool:
  """Whether url is a share link to expand: https on exactly a short-link host, with nothing that URL parsers read differently."""
  # requests ends the host at a backslash and urlsplit doesn't, so https://evil.example\@maps.app.goo.gl/x would pass a
  # hostname check here and then connect to evil.example. Userinfo, a port or whitespace can hide a host too.
  if not (url.isascii() and url.isprintable()) or any(c in ' \\@' for c in url):
    return False
  scheme, _, rest = url.partition('://')
  return scheme == 'https' and re.split('[/?#]', rest, maxsplit=1)[0] in SHORT_LINK_HOSTS


def resolve_link(url: str) -> str:
  """The map link behind a share link, which only carries an ID, else url itself. Only short-link hosts are ever fetched."""
  for _ in range(MAX_REDIRECTS):
    if not short_link(url):
      return url
    # one redirect at a time, so that each URL is checked before it's fetched
    with requests.get(url, allow_redirects=False, timeout=5, stream=True) as response:
      location = response.headers.get('Location') if response.is_redirect else None
    if not location:
      raise LinkError("the share link didn't redirect")
    url = urljoin(url, location)
  if short_link(url):
    raise LinkError("too many redirects")
  return url


def interstitial(url: SplitResult) -> bool:
  """Whether url is Google's cookie consent or unusual traffic page, which has the page it's in front of as continue=."""
  host = url.hostname or ''
  return host.startswith('consent.google.') or (url.path.startswith('/sorry') and re.match(r'(www\.)?google\.', host) is not None)


def read_link(link: str) -> Place:
  """The place a Google or Apple Maps link points to: its coordinates when the link has them, and its name or address."""
  url = urlsplit(link)
  if interstitial(url):  # the link it continues to, never the page itself
    url = urlsplit(parse_qs(url.query).get('continue', [''])[0])
    if interstitial(url):
      return Place(None)
  query = {key: values[0].strip() for key, values in parse_qs(url.query).items()}
  segments = [unquote_plus(segment).strip() for segment in url.path.split('/')]
  path = '/'.join(segments)

  # Google: /maps/place/<name>, /maps/search/<text>, /maps/dir/<stop>/.../<stop> or /maps/@<viewport>, then the viewport and data
  after = segments[segments.index('maps') + 1:] if 'maps' in segments else []
  kind = after[0] if after else ''
  stops = [s for s in after[1:] if s and not s.startswith('@') and '=' not in s]
  if kind == 'dir':
    stop = stops[-1] if stops else ''  # the destination: the first stop is where the directions start
  else:
    stop = stops[0] if stops and kind in ('place', 'search') else ''

  pairs = COORDINATES.findall(stop)
  if kind == 'dir':
    # stops have their pairs in order, but only when they have one: with fewer pairs than stops the last pair may be an
    # earlier stop's, even the start, so it's only the destination's when every stop has one
    stop_pairs = GOOGLE_STOP.findall(path)
    if len(stop_pairs) >= len(stops):
      pairs += [(latitude, longitude) for longitude, latitude in stop_pairs[-1:]]
  pairs += GOOGLE_PLACE.findall(path)[-1:]
  for key in COORDINATE_KEYS:
    pairs += COORDINATES.findall(query.get(key, ''))
  if kind == 'place' or kind.startswith('@'):  # the map's center: the place on a place link, and all a link to the map has
    pairs += GOOGLE_VIEWPORT.findall(path)[:1]
  candidates = [(float(latitude), float(longitude)) for latitude, longitude in pairs]
  coordinates = next((c for c in candidates if valid_coordinates(*c)), None)

  texts = [text for text in (stop, *(query.get(key, '') for key in TEXT_KEYS)) if text and not COORDINATES.match(text)]
  text = texts[0] if texts else ''
  return Place(coordinates, ' '.join((query.get('name') or text).split())[:200], text)


def place_from_text(text: str) -> Place:
  """The place in pasted text: from the first map link in it, expanding a share link (which fetches it), else from 'lat, lon'."""
  link = LINK.search(text)
  return read_link(resolve_link(link[0])) if link else Place(coordinates_in(text))


def search_text(text: str) -> str:
  """text as Mapbox takes it: one line without semicolons, of at most 20 words and numbers and 256 characters."""
  text = ' '.join(text.replace(';', ' ').split())
  words = list(re.finditer(r'\w+', text))
  if len(words) > 20:
    text = text[:words[19].end()]
  return text[:256].rstrip()


def geocode(query: str, token: str, near: tuple[float, float] | None) -> list[dict]:
  """Places matching an address search, nearest first when the position is known."""
  query = search_text(query)
  if not query:
    return []
  options = {'q': query, 'access_token': token, 'limit': 5, 'autocomplete': 'false'}
  if near is not None:
    options['proximity'] = f'{near[1]},{near[0]}'
  response = requests.get(GEOCODE_URL, params=options, timeout=10)
  response.raise_for_status()
  places = []
  for feature in response.json().get('features', []):
    properties = feature.get('properties') or {}
    coordinates = properties.get('coordinates') or {}
    # a routable point is the spot on the road for the place, not the middle of its building
    point = next(iter(coordinates.get('routable_points') or []), coordinates)
    if 'latitude' not in point:
      longitude, latitude = feature['geometry']['coordinates']
      point = {'latitude': latitude, 'longitude': longitude}
    places.append({'place_name': properties.get('full_address') or properties.get('name') or query,
                   'latitude': float(point['latitude']), 'longitude': float(point['longitude'])})
  return places


def mapbox_error(e: requests.RequestException) -> str:
  """What to tell the user about a failed Mapbox request: never the error itself, whose URL has the access token."""
  if isinstance(e, requests.ConnectionError):
    return OFFLINE
  if e.response is not None and e.response.status_code in MAPBOX_ERRORS:
    return MAPBOX_ERRORS[e.response.status_code]
  return MAPBOX_FAILED


def last_gps_position(params: Params) -> tuple[float, float] | None:
  """(latitude, longitude) of the last GPS fix saved, else None."""
  data = params.get("LastGPSPosition")
  if not data:
    return None
  try:
    if data.startswith(b'{'):
      # the JSON that osm_map_data writes for mapd to the memory params, which on macOS are these
      position = json.loads(data)
      latitude, longitude = float(position['latitude']), float(position['longitude'])
    else:
      # the gpsLocationExternal event ubloxd saves about once a minute with a fix
      with log.Event.from_bytes(data) as event:
        latitude, longitude = event.gpsLocationExternal.latitude, event.gpsLocationExternal.longitude
  except Exception:  # unreadable: searches just aren't biased
    return None
  return (latitude, longitude) if valid_coordinates(latitude, longitude) else None


def wifi_address() -> str | None:
  """The Wi-Fi interface's IPv4 address now, else None."""
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      ifreq = fcntl.ioctl(s.fileno(), SIOCGIFADDR, struct.pack('256s', WIFI_INTERFACE.encode()))
  except OSError:  # no address: not connected
    return None
  return socket.inet_ntoa(ifreq[20:24])


def serves(local_address: str) -> bool:
  """Whether to answer a connection that came in at this address of the device: from the device itself or over its Wi-Fi,
  never over its cellular link, where anyone on the carrier's network may reach it. The page has no password. A machine
  without Wi-Fi, such as a PC, answers on every address."""
  if local_address == LOOPBACK:
    return True
  try:
    socket.if_nametoindex(WIFI_INTERFACE)
  except OSError:
    return True
  return local_address == wifi_address()


def device_host(host: str) -> bool:
  """Whether a Host header names the device by an IP address or as localhost, which a rebound DNS name never does."""
  name, _, port = host.rpartition(':')
  if not (port.isascii() and port.isdigit()):  # no port to strip
    name = host
  if name.startswith('[') and name.endswith(']'):  # an IPv6 address
    name = name[1:-1]
  if name.lower() == 'localhost':
    return True
  try:
    ipaddress.ip_address(name)
  except ValueError:
    return False
  return True


def field(struct, name: str):
  """struct.name, else None: for a field this build's schema lacks, a union member that isn't set, or no struct at all."""
  try:
    return getattr(struct, name)
  except Exception:  # AttributeError, or capnp's KjException for a union member that isn't set
    return None


def entry(table, key: str):
  """table[key], else None: for a table, or a SubMaster, without it."""
  try:
    return table[key]
  except Exception:  # KeyError, or TypeError for no table at all
    return None


def known(value, kind: type) -> bool | int | float | None:
  """value as kind (bool, int or float) when it's one, else None, which the page shows as unknown. Never NaN or infinity,
  which aren't JSON: the page couldn't read its state."""
  if kind is bool or isinstance(value, bool):
    return value if kind is bool and isinstance(value, bool) else None
  if not isinstance(value, (int, float)) or (isinstance(value, float) and not math.isfinite(value)):
    return None
  try:
    return kind(value)
  except OverflowError:  # an integer too big for a float
    return None


def name_of(read) -> str | None:
  """The name read() gives, such as a capnp enum's value or the member of a union that's set, else None."""
  try:
    value = read()
    return None if value is None else str(value)
  except Exception:  # a field this build's schema lacks, a union member that isn't set, or a value it doesn't name
    return None


def received(sm, service: str) -> float | None:
  """When sm last received a message of service (monotonic seconds), else None: also before it ever has."""
  return known(entry(field(sm, 'recv_time'), service), float) if entry(field(sm, 'seen'), service) is True else None


def location(sm, service: str, used: bool) -> dict:
  """The last message of a location service as the page shows it, used: whether latest_gps_source took the position from it."""
  seen = known(entry(field(sm, 'seen'), service), bool)
  message = entry(sm, service) if seen else None  # before the first message, a SubMaster has a placeholder
  return {'seen': seen, 'valid': known(entry(field(sm, 'valid'), service), bool) if seen else None,
          'hasFix': known(field(message, 'hasFix'), bool), 'satelliteCount': known(field(message, 'satelliteCount'), int),
          'horizontalAccuracy': known(field(message, 'horizontalAccuracy'), float),
          'verticalAccuracy': known(field(message, 'verticalAccuracy'), float), 'speed': known(field(message, 'speed'), float), 'used': used}


def gps_processes(manager_state) -> dict[str, dict | None]:
  """Each GPS process as the manager last reported it, None for one it didn't list."""
  processes: dict[str, dict | None] = dict.fromkeys(GPS_PROCESSES)
  try:
    listed = list(field(manager_state, 'processes') or [])
  except TypeError:  # not a list
    listed = []
  for process in listed:
    name = field(process, 'name')
    if isinstance(name, str) and name in processes:
      processes[name] = {'running': known(field(process, 'running'), bool), 'shouldBeRunning': known(field(process, 'shouldBeRunning'), bool),
                         'exitCode': known(field(process, 'exitCode'), int)}
  return processes


def age(now: float, then: float | None) -> float | None:
  return None if then is None else round(now - then, 2)


class GpsDetails:
  """What the page shows of the GPS, to tell apart why there's no fix: the receiver's processes not running, running without
  sending anything, messages without a fix, or a fix that latest_gps_source rejects. It only reads, and never raises: what
  it can't read is unknown, None."""

  def __init__(self):
    self.locations = {service: (None, location(None, service, False)) for service in GPS_SERVICES}  # (when received, the message)
    self.processes: dict[str, dict | None] | None = None  # from the manager's last managerState
    self.processes_time: float | None = None
    self.ublox_time: float | None = None  # ubloxd sends only what it decodes from the chip's data
    self.satellites: int | None = None  # in the last measurement report: those the chip receives
    self.satellites_time: float | None = None
    self.antenna: str | None = None
    self.hardware: dict | None = None  # the chip's last hardware report: antenna check and power, noise, gain and interference
    self.hardware_time: float | None = None

  def update(self, sm, source: str | None) -> None:
    """After each update of sm, with the service latest_gps_source took the position from."""
    for service in GPS_SERVICES:
      self.locations[service] = received(sm, service), location(sm, service, source == service)
    if entry(field(sm, 'updated'), 'managerState') is True:
      self.processes, self.processes_time = gps_processes(entry(sm, 'managerState')), received(sm, 'managerState')
    if entry(field(sm, 'updated'), 'ubloxGnss') is True:
      # ubloxGnss carries reports of several kinds, and the SubMaster only keeps the last message: each of these is from
      # the last report of its kind
      gnss = entry(sm, 'ubloxGnss')
      self.ublox_time = received(sm, 'ubloxGnss')
      kind = name_of(lambda: gnss.which())
      if kind == 'measurementReport':
        self.satellites, self.satellites_time = known(field(field(gnss, kind), 'numMeas'), int), self.ublox_time
      elif kind == 'hwStatus':
        hardware = field(gnss, kind)
        self.antenna = name_of(lambda: gnss.hwStatus.aStatus)
        self.hardware = {'antenna': self.antenna, 'antennaPower': name_of(lambda: gnss.hwStatus.aPower),
                         'noise': known(field(hardware, 'noisePerMS'), int), 'gain': known(field(hardware, 'agcCnt'), int),
                         'interference': known(field(hardware, 'jamInd'), int)}
        self.hardware_time = self.ublox_time

  def state(self, started: bool | None, ublox_available: bool | None) -> dict:
    now = time.monotonic()
    details = {'started': started, 'ubloxAvailable': ublox_available, 'processes': self.processes, 'processesAge': age(now, self.processes_time),
               **{service: {**message, 'age': age(now, then)} for service, (then, message) in self.locations.items()},
               'ubloxGnss': {'seen': self.ublox_time is not None, 'age': age(now, self.ublox_time), 'satellites': self.satellites,
                             'satellitesAge': age(now, self.satellites_time), 'antenna': self.antenna,
                             'hardware': self.hardware, 'hardwareAge': age(now, self.hardware_time)}}
    return {**details, 'summary': gps_summary(details)}


def duration(seconds: float) -> str:
  """seconds in words: 0.4 s, 25 s, 12 min or 3 h."""
  if seconds < 10:
    return f'{seconds:.1f} s'
  if seconds < 120:
    return f'{seconds:.0f} s'
  if seconds < 7200:
    return f'{seconds / 60:.0f} min'
  return f'{seconds / 3600:.0f} h'


def length(meters: float) -> str:
  return f'{meters:.0f} m' if meters < 1000 else f'{meters / 1000:.0f} km'


def plural(count: int, noun: str) -> str:
  return f'{count} {noun}' + ('' if count == 1 else 's')


def stopped(name: str, process: dict | None) -> str:
  """Why a GPS process that should run doesn't, in words."""
  if process is None:
    return f"{name} isn't in the process list"
  code = process['exitCode']
  if code is None:
    return f'{name} stopped'
  # multiprocessing gives a process that a signal ended the signal's number, negated
  return f'{name} was killed by signal {-code}' if code < 0 else f'{name} stopped with exit code {code}'


def processes_words(processes: dict[str, dict | None] | None, names: tuple[str, ...]) -> str:
  """Whether the manager runs these processes, in words."""
  if processes is None:  # no managerState yet
    return 'process state unknown'
  if all(processes[name] is not None and processes[name]['running'] for name in names):
    return 'running'
  # the manager never restarts a process that ended: until the car is turned off, it's one that should run and doesn't
  ended = [stopped(name, process) for name in names if (process := processes[name]) is None or (process['shouldBeRunning'] and not process['running'])]
  return ', '.join(['not running', *ended])


def receiver_words(details: dict, receiver: str) -> list[str]:
  """What a GPS receiver is doing, in words: whether its processes run, when its last message came, and what it said."""
  names, service = RECEIVERS[receiver]
  message, gnss = details[service], details['ubloxGnss']
  ublox = receiver == 'u-blox'  # only ubloxd reports satellites and the antenna: qcomgpsd's satelliteCount is always 0
  words = [processes_words(details['processes'], names)]
  if message['age'] is not None:
    words.append(f"last message {duration(message['age'])} ago")
  elif ublox and gnss['age'] is not None:  # ubloxd decodes data from the chip, but no position
    words.append(f"no position messages yet, but data {duration(gnss['age'])} ago")
  else:
    words.append('no messages yet')

  if message['hasFix'] is False:
    words.append('no fix yet')
  elif message['hasFix'] and message['used']:
    words.append('has a fix')
  elif message['hasFix']:  # latest_gps_source takes a fix only from a valid message under 2 s old
    words.append("has a fix, but it's marked invalid" if message['valid'] is False else "has a fix, but it's too old to use")

  if ublox and message['satelliteCount'] is not None:
    words.append(f"{plural(message['satelliteCount'], 'satellite')} used" + ('' if gnss['satellites'] is None else f", {gnss['satellites']} heard"))
  elif ublox and gnss['satellites'] is not None:
    words.append(f"{plural(gnss['satellites'], 'satellite')} heard")
  if (message['horizontalAccuracy'] or 0) > 0:
    words.append(f"accuracy {length(message['horizontalAccuracy'])}")
  elif (message['verticalAccuracy'] or 0) > 0:  # all that qcomgpsd reports
    words.append(f"vertical accuracy {length(message['verticalAccuracy'])}")
  return words


def hardware_words(gnss: dict) -> str | None:
  """The u-blox's own hardware report in words, to tell a chip that hears no satellites because of its antenna (not
  connected, short-circuited, not powered: a weak signal also turns its gain all the way up) from one that is drowned out
  by interference, or simply has no sky: its antenna check and power, interference, noise level and gain."""
  hardware = gnss['hardware']
  if hardware is None:
    return None
  words = [ANTENNA.get(hardware['antenna'], 'antenna check unknown')]
  if hardware['antennaPower'] in ANTENNA_POWER:
    words.append(ANTENNA_POWER[hardware['antennaPower']])
  if hardware['interference'] is not None:
    level = 'low' if hardware['interference'] < 50 else 'medium' if hardware['interference'] < 150 else 'high'
    words.append(f"interference {level} ({hardware['interference']} of 255)")
  if hardware['noise'] is not None:
    words.append(f"noise level {hardware['noise']}")
  if hardware['gain'] is not None:
    words.append(f"gain {round(100 * hardware['gain'] / AGC_FULL_SCALE)}%")
  if gnss['hardwareAge'] is not None:
    words.append(f"reported {duration(gnss['hardwareAge'])} ago")
  return 'GPS chip: ' + ' · '.join(words)


def heard_from(details: dict, receiver: str) -> bool:
  """Whether a GPS receiver's processes run or should, or it has sent anything."""
  names, service = RECEIVERS[receiver]
  processes = details['processes'] or {}
  runs = any(processes.get(name) and (processes[name]['running'] or processes[name]['shouldBeRunning']) for name in names)
  return bool(runs or details[service]['seen'] or (receiver == 'u-blox' and details['ubloxGnss']['seen']))


def gps_summary(details: dict) -> list[str]:
  """The GPS details in plain words, a line each: whether the car is on, then what the GPS receiver is doing."""
  lines = []
  if details['started'] is not None:
    lines.append('Car on' if details['started'] else 'Car off: the GPS only runs while the car is on')
  if details['ubloxAvailable'] is not None:  # which receiver the manager found on the device
    receivers = ['u-blox' if details['ubloxAvailable'] else 'Quectel modem']
  else:
    receivers = [receiver for receiver in RECEIVERS if heard_from(details, receiver)]
  lines += [f'GPS receiver ({receiver}): ' + ' · '.join(receiver_words(details, receiver)) for receiver in receivers]
  if 'u-blox' in receivers and (hardware := hardware_words(details['ubloxGnss'])) is not None:
    lines.append(hardware)
  if not receivers:
    lines.append(f"GPS receiver: {processes_words(details['processes'], GPS_PROCESSES)} · no messages yet")
  return lines


class NavWeb:
  def __init__(self, params: Params | None = None):
    self.params = params if params is not None else Params()
    self.lock = threading.Lock()
    self.position: tuple[float, float] | None = None  # (latitude, longitude)
    self.instruction: dict | None = None
    self.gps = GpsDetails()

  def update(self, sm) -> None:
    source = latest_gps_source(sm)
    instruction = sm['navInstruction'] if sm.alive['navInstruction'] and sm.valid['navInstruction'] else None
    with self.lock:
      self.position = (sm[source].latitude, sm[source].longitude) if source is not None else None
      self.instruction = None if instruction is None else {
        'text': instruction.maneuverPrimaryText, 'distance': instruction.maneuverDistance,
        'distanceRemaining': instruction.distanceRemaining, 'timeRemaining': instruction.timeRemaining,
      }
      self.gps.update(sm, source)

  def token(self) -> str:
    return (self.params.get("MapboxToken") or '').strip()

  def flag(self, key: str) -> bool | None:
    """A boolean param, else None: unset or unreadable."""
    try:
      return known(self.params.get(key), bool)
    except Exception:  # a key this build doesn't have
      return None

  def near(self) -> tuple[float, float] | None:
    """Where to search near: the live GPS fix, else the last one saved, so that searches offroad are local too."""
    with self.lock:
      position = self.position
    return position if position is not None else last_gps_position(self.params)

  def state(self) -> dict:
    token = self.token()
    # the manager writes both: IsOffroad when the car turns on or off, and UbloxAvailable for whether the device has a u-blox
    offroad, ublox_available = self.flag("IsOffroad"), self.flag("UbloxAvailable")
    with self.lock:
      return {'destination': self.params.get("NavDestination"), 'navigationEnabled': self.params.get_bool("NavigationEnabled"),
              'isMetric': self.params.get_bool("IsMetric"), 'tokenHint': f'…{token[-4:]}' if token else '',
              'gps': self.position is not None, 'instruction': self.instruction,
              'gpsDetails': self.gps.state(None if offroad is None else not offroad, ublox_available)}

  def search(self, body: dict) -> tuple[int, dict]:
    query = str(body.get('query', '')).strip()
    if not query:
      return 400, {'error': 'Type an address to search for'}
    if not self.token():
      return 400, {'error': 'Add a Mapbox token to search, or paste a map link or coordinates instead'}
    return 200, {'places': geocode(query, self.token(), self.near())}

  def look_up(self, text: str) -> tuple[int, dict]:
    """Places to pick from for a link that names its place without coordinates. The destination stays as it is."""
    token = self.token()
    if not token:
      return 400, {'error': 'A Mapbox token is needed to look up this link: save one below, or paste the coordinates instead'}
    near = self.near()
    places = geocode(text, token, near)
    address = text.partition(',')[2].strip()
    if not places and address:
      # Mapbox has addresses but not businesses, so "Costco Wholesale, 1600 El Camino Real, ..." is found by its address
      places = geocode(address, token, near)
    return 200, {'places': places}

  def set_destination(self, body: dict) -> tuple[int, dict]:
    if 'text' in body:
      try:
        place = place_from_text(str(body['text']))
      except (requests.RequestException, LinkError) as e:
        cloudlog.error(f"navweb: could not open a link: {type(e).__name__}")
        return 502, {'error': OFFLINE if isinstance(e, requests.ConnectionError) else LINK_FAILED}
      if place.coordinates is None:
        if place.query:
          return self.look_up(place.query)
        return 400, {'error': 'No coordinates found: paste "latitude, longitude" or a Google or Apple Maps link'}
      latitude, longitude = place.coordinates
      place_name = place.name or f'{latitude:.5f}, {longitude:.5f}'
    else:
      latitude, longitude, place_name = float(body['latitude']), float(body['longitude']), str(body.get('place_name', ''))[:200]
    if not valid_coordinates(latitude, longitude):
      return 400, {'error': 'Invalid coordinates'}
    self.params.put("NavDestination", {'latitude': latitude, 'longitude': longitude, 'place_name': place_name}, block=True)
    return 200, self.state()

  def clear_destination(self, _body: dict) -> tuple[int, dict]:
    self.params.remove("NavDestination")
    return 200, self.state()

  def set_token(self, body: dict) -> tuple[int, dict]:
    token = body.get('token')
    if not isinstance(token, str):  # a request without a token never removes the saved one
      return 400, {'error': 'Missing token'}
    if token.strip():
      self.params.put("MapboxToken", token.strip(), block=True)
    else:
      self.params.remove("MapboxToken")
    return 200, self.state()


class HeaderReader(io.BufferedIOBase):
  """A request's file for the stdlib to read its headers from: raises once they pass MAX_HEADERS bytes, answered with 431."""

  def __init__(self, rfile):
    self.rfile = rfile
    self.left = MAX_HEADERS

  def readline(self, size: int | None = -1, /) -> bytes:
    line = self.rfile.readline(self.left + 1 if size is None or size < 0 else min(size, self.left + 1))
    self.left -= len(line)
    if self.left < 0:
      raise http.client.HTTPException(f'request headers over {MAX_HEADERS} bytes')
    return line


def make_handler(navweb: NavWeb):
  routes = {'/api/search': navweb.search, '/api/destination': navweb.set_destination,
            '/api/clear': navweb.clear_destination, '/api/token': navweb.set_token}

  class Handler(BaseHTTPRequestHandler):
    timeout = 10  # seconds a read or write may stall, so that a stalled client can't hold its thread forever

    def parse_request(self):
      # the stdlib reads the headers in here, before do_GET or do_POST could refuse them
      rfile, self.rfile = self.rfile, HeaderReader(self.rfile)
      try:
        return super().parse_request()
      finally:
        self.rfile = rfile

    def send(self, status: int, body: bytes, content_type: str) -> None:
      self.send_response(status)
      self.send_header('Content-Type', content_type)
      self.send_header('Content-Length', str(len(body)))
      self.send_header('Cache-Control', 'no-store')
      self.end_headers()
      self.wfile.write(body)

    def send_json(self, status: int, payload: dict) -> None:
      self.send(status, json.dumps(payload).encode(), 'application/json')

    def refusal(self) -> tuple[int, dict] | None:
      """The answer for a request to refuse before reading it, else None."""
      host = self.headers.get('Host', '')
      # a page on another site can rebind its DNS name to this device to read and post as the same origin, but the
      # browser still sends that name as the Host
      if not device_host(host):
        return 421, {'error': "Open this page at the device's IP address"}
      origin = self.headers.get('Origin')
      if origin is not None and origin.partition('://')[2].lower() != host.lower():
        return 403, {'error': 'Requests from other sites are not allowed'}
      if 'Transfer-Encoding' in self.headers:  # the body is never decoded from chunks here, so it would be misread
        return 501, {'error': 'Chunked requests are not supported'}
      return None

    def do_GET(self):
      if (refusal := self.refusal()) is not None:
        return self.send_json(*refusal)
      if self.path == '/':
        self.send(200, PAGE.encode(), 'text/html; charset=utf-8')
      elif self.path == '/api/state':
        self.send_json(200, navweb.state())
      else:
        self.send_json(404, {'error': 'Not found'})

    def do_POST(self):
      if (refusal := self.refusal()) is not None:
        return self.send_json(*refusal)
      # Requiring JSON makes browsers preflight requests from other sites, which this server never allows
      if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
        return self.send_json(415, {'error': 'Expected JSON'})
      length = self.headers.get('Content-Length')
      if length is None:
        return self.send_json(411, {'error': 'Content-Length required'})
      if not (length.isascii() and length.isdigit()):  # isdigit() alone passes digits like '²', which int() rejects
        return self.send_json(400, {'error': 'Invalid Content-Length'})
      if len(length) > 9 or int(length) > MAX_BODY:  # int() also rejects thousands of digits
        return self.send_json(413, {'error': 'Request too large'})
      if self.path not in routes:
        return self.send_json(404, {'error': 'Not found'})
      try:
        body = json.loads(self.rfile.read(int(length)) or b'{}')
        status, payload = routes[self.path](body if isinstance(body, dict) else {})
      except requests.RequestException as e:
        # requests exceptions contain the URL, and with it the access token; keep it out of logs
        cloudlog.error(f"navweb: request failed: {type(e).__name__}")
        status, payload = 502, {'error': mapbox_error(e)}
      except (ValueError, KeyError, TypeError, OverflowError):  # OverflowError: float() of a JSON integer too big for a float
        status, payload = 400, {'error': 'Invalid request'}
      self.send_json(status, payload)

    def log_message(self, format, *args):  # noqa: A002  # stdlib override
      pass

  return Handler


class Server(ThreadingHTTPServer):
  """Serves the page over Wi-Fi and to the device itself, with at most MAX_CONNECTIONS open at once, each for at most
  MAX_CONNECTION_TIME.

  Each connection holds a thread and its buffers, and navweb runs onroad beside the driving processes.
  """

  def __init__(self, *args, **kwargs):
    self.lock = threading.Lock()
    self.deadlines: dict[socket.socket, float] = {}  # each open connection: when it's cut off
    super().__init__(*args, **kwargs)

  def verify_request(self, request, client_address) -> bool:
    # before anything is kept for it, so that a refused connection costs nothing
    try:
      local_address = request.getsockname()[0]
    except OSError:  # the client left already
      return False
    if not serves(local_address):
      return False  # serve_forever closes it at once
    with self.lock:
      if len(self.deadlines) >= MAX_CONNECTIONS:
        return False  # serve_forever closes it at once
      self.deadlines[request] = time.monotonic() + MAX_CONNECTION_TIME
      return True

  def shutdown_request(self, request) -> None:
    with self.lock:  # forgotten before it's closed, so that service_actions never shuts down a reused descriptor
      self.deadlines.pop(request, None)
    super().shutdown_request(request)

  def service_actions(self) -> None:
    # serve_forever calls this at least every 0.5 s. The handler's timeout restarts with every byte it reads, so alone it
    # never drops a client that keeps sending a byte at a time.
    now = time.monotonic()
    with self.lock:
      for connection, deadline in self.deadlines.items():
        if now > deadline:
          try:
            connection.shutdown(socket.SHUT_RDWR)  # its handler's reads end and its writes fail
          except OSError:  # not connected: the client left, or it was cut off already
            pass

  def handle_error(self, request, client_address) -> None:
    if not isinstance(sys.exception(), ConnectionError):  # a client that left or was cut off: nothing to report
      super().handle_error(request, client_address)


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>sunnypilot navigation</title>
<style>
  :root { color-scheme: dark; --bg: #0b1014; --card: #151c22; --line: #26313a; --text: #e8eef2; --muted: #93a1ad; --accent: #48eb84; --danger: #ff6b6b; }
  * { box-sizing: border-box; }
  body { margin: 0; padding: 16px; background: var(--bg); color: var(--text); font: 16px/1.4 -apple-system, system-ui, sans-serif; }
  main { max-width: 560px; margin: 0 auto; display: grid; gap: 14px; }
  h1 { font-size: 20px; margin: 4px 0; }
  section { background: var(--card); border: 1px solid var(--line); border-radius: 14px; padding: 14px; display: grid; gap: 10px; }
  h2 { font-size: 13px; text-transform: uppercase; letter-spacing: .06em; color: var(--muted); margin: 0; }
  .row { display: flex; gap: 8px; }
  input { flex: 1; min-width: 0; padding: 12px; border-radius: 10px; border: 1px solid var(--line); font-size: 16px;
          background: var(--bg); color: var(--text); }
  button { padding: 12px 14px; border-radius: 10px; border: 0; background: var(--accent); color: #04140a; font-weight: 600; font-size: 15px; }
  button.secondary { background: var(--line); color: var(--text); }
  ul { list-style: none; margin: 0; padding: 0; display: grid; gap: 6px; }
  li button { width: 100%; text-align: left; background: var(--bg); color: var(--text); border: 1px solid var(--line); font-weight: 500; }
  .muted { color: var(--muted); font-size: 14px; }
  .error { position: fixed; left: 16px; right: 16px; bottom: 16px; max-width: 528px; margin: 0 auto; padding: 12px 14px; border-radius: 10px;
           background: #3a1416; border: 1px solid var(--danger); color: #ffd6d6; font-size: 14px; }
  .error:empty { display: none; }
  .value { font-size: 18px; font-weight: 600; word-break: break-word; }
  .gps { padding-left: 10px; border-left: 2px solid var(--line); color: var(--muted); font-size: 13px; word-break: break-word; }
  .gps strong { display: block; color: var(--text); font-weight: 600; }
</style>
</head>
<body>
<main>
  <h1>sunnypilot navigation</h1>
  <section>
    <h2>Destination</h2>
    <div class="value" id="destination">None</div>
    <div class="muted" id="status"></div>
    <div class="gps"><strong>GPS details</strong><div id="gps-details"></div></div>
    <div class="row"><button class="secondary" id="clear">Clear destination</button></div>
  </section>
  <section>
    <h2>Search an address</h2>
    <form class="row" id="search-form"><input id="query" placeholder="123 Main St, City" autocomplete="off"><button>Search</button></form>
    <ul id="results"></ul>
  </section>
  <section>
    <h2>Or paste a map link or coordinates</h2>
    <form class="row" id="link-form"><input id="link" placeholder="Google/Apple Maps link, or 37.77, -122.42" autocomplete="off"><button>Go</button></form>
    <div class="muted">For businesses and other places: share the place from your maps app, copy the link, and paste it here.</div>
  </section>
  <section>
    <h2>Mapbox token</h2>
    <div class="muted" id="token-state"></div>
    <form class="row" id="token-form"><input id="token" placeholder="pk.…" autocomplete="off" autocapitalize="off"><button>Save</button></form>
  </section>
  <div class="error" id="error"></div>
</main>
<script>
const $ = (id) => document.getElementById(id);
let errorTimer;
const showError = (message) => {
  $('error').textContent = message || '';
  clearTimeout(errorTimer);
  if (message) errorTimer = setTimeout(() => { $('error').textContent = ''; }, 6000);
};

async function call(path, body) {
  const response = await fetch(path, body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || 'Request failed');
  return payload;
}

let metric = false;
function formatDistance(meters) {
  if (metric) return meters >= 1000 ? `${(meters / 1000).toFixed(1)} km` : `${Math.round(meters / 10) * 10} m`;
  const miles = meters / 1609.34;
  return miles >= 0.2 ? `${miles.toFixed(1)} mi` : `${Math.round(meters * 3.28084 / 50) * 50} ft`;
}

function render(state) {
  metric = state.isMetric;
  const destination = state.destination;
  $('destination').textContent = destination ? (destination.place_name || `${destination.latitude.toFixed(5)}, ${destination.longitude.toFixed(5)}`) : 'None';
  const parts = [];
  if (!state.navigationEnabled) parts.push('Navigation is off: turn on "navigation (alpha)" in the device\\'s developer settings.');
  parts.push(state.gps ? 'GPS fix: yes' : 'GPS fix: waiting (the car must be on)');
  if (state.instruction) {
    const i = state.instruction;
    parts.push(`Next: ${i.text || 'continue'} in ${formatDistance(Math.max(0, i.distance))} · ${Math.max(1, Math.round(i.timeRemaining / 60))} min to go`);
  } else if (destination) {
    parts.push('Route: waiting (needs GPS, internet, and a token or comma routing)');
  }
  $('status').textContent = parts.join(' · ');
  // a line each, as text: never read as HTML
  $('gps-details').replaceChildren(...(state.gpsDetails?.summary ?? []).map((line) => {
    const item = document.createElement('div');
    item.textContent = line;
    return item;
  }));
  $('token-state').textContent = state.tokenHint
    ? `Saved (${state.tokenHint}). Paste a new one to replace it, or save empty to remove it.`
    : 'Not set. Create a free access token at mapbox.com.';
}

async function refresh() {
  try { render(await call('/api/state')); } catch (e) { showError(e.message); }
}

function showPlaces(places) {
  showError(places.length ? '' : 'No matches');
  $('results').replaceChildren(...places.map((place) => {
    const item = document.createElement('li');
    const button = document.createElement('button');
    button.textContent = place.place_name;
    button.addEventListener('click', () => setDestination(place));
    item.append(button);
    return item;
  }));
}

async function setDestination(body) {
  try {
    const payload = await call('/api/destination', body);
    // a link that names its place without coordinates gives places to pick from, like a search
    if (payload.places) return showPlaces(payload.places);
    render(payload);
    $('results').replaceChildren();
    $('link').value = '';
    showError('');
  } catch (e) { showError(e.message); }
}

$('search-form').addEventListener('submit', async (event) => {
  event.preventDefault();
  try { showPlaces((await call('/api/search', {query: $('query').value})).places); } catch (e) { showError(e.message); }
});

$('link-form').addEventListener('submit', (event) => { event.preventDefault(); setDestination({text: $('link').value}); });
$('clear').addEventListener('click', async () => { try { render(await call('/api/clear', {})); } catch (e) { showError(e.message); } });
$('token-form').addEventListener('submit', async (event) => {
  event.preventDefault();
  try { render(await call('/api/token', {token: $('token').value})); $('token').value = ''; showError(''); } catch (e) { showError(e.message); }
});

refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
"""


def main():
  navweb = NavWeb()
  server = Server(('0.0.0.0', PORT), make_handler(navweb))
  threading.Thread(target=server.serve_forever, daemon=True).start()
  cloudlog.warning(f"navweb serving on port {PORT}")

  sm = messaging.SubMaster(SERVICES)
  while True:
    sm.update(1000)
    navweb.update(sm)


if __name__ == "__main__":
  main()
