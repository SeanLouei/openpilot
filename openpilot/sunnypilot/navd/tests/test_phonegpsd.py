"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import ast
import calendar
import datetime
import itertools
import json
import math
import re
import socket
import tempfile
import time
from functools import reduce
from operator import xor
from pathlib import Path
from unittest.mock import MagicMock, patch

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.navigation.helpers import PHONE_MAX_DELAY, latest_gps_source
from openpilot.selfdrive.navigation.navd import MAX_FIX_AGE, PHONE_MAX_FIX_AGE
from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE
from openpilot.sunnypilot.navd import phonegpsd
from openpilot.sunnypilot.navd.navweb import NavWeb
from openpilot.sunnypilot.navd.phonegpsd import (CLOCK_AGREEMENT, CLOCK_DIFFERENCE, KNOTS, LATE_RESTART, MIN_ACCURACY, NOMINAL_ERROR, SPEED_ERROR,
                                                 VDOP_RATIO, Gga, PhoneFixes, PhoneGps, default_gateway, host_param, parse_gga, parse_rmc, parse_vtg,
                                                 port_param, sentence)
from openpilot.sunnypilot.navd.tests.test_navd import ROUTE, route_engine
from openpilot.sunnypilot.navd.tests.test_navweb import RUNNING, T, FakeParams, manager_state, page_state, position, receive, submaster
from openpilot.sunnypilot.sunnylink.tools.compile_settings_ui import DEFAULT_OUT
from openpilot.system.manager import process_config

PHONE = log.GpsLocationData.SensorSource.iOS
T0 = calendar.timegm((2026, 11, 4, 18, 30, 0))  # s since 1970: the phone's time of its first fix in these tests
R0 = 5000.  # monotonic s: around when that fix arrives
# /proc/net/route on the comma 4 on an iPhone's hotspot (172.20.10.1), with the cellular link's routes too
ROUTE_TABLE = """Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT
rmnet_data0\t00000000\t0100000A\t0003\t0\t0\t1000\t00000000\t0\t0\t0
wlan0\t00000000\t010A14AC\t0003\t0\t0\t600\t00000000\t0\t0\t0
wlan0\t000A14AC\t00000000\t0001\t0\t0\t600\tF0FFFFFF\t0\t0\t0
"""


def nmea(body: str) -> bytes:
  """A sentence as the phone sends it, with its checksum and line end."""
  return f'${body}*{reduce(xor, body.encode(), 0):02X}\r\n'.encode()


def angle(value: float, width: int, hemispheres: str) -> tuple[str, str]:
  """Signed degrees as NMEA writes them: (d)ddmm.mmmmmm and the hemisphere."""
  whole = int(abs(value))
  return f'{whole:0{width}d}{(abs(value) - whole) * 60:09.6f}', hemispheres[value < 0]


def epoch(t: float, north: float = 100., east: float = 0., speed: float | None = 10., course: float | None = 0., hdop: str = '0.9',
          satellites: str = '09', quality: str = '1', status: str = 'A', talker: str = 'GP') -> list[bytes]:
  """The GGA and RMC the phone sends for its fix at time t (s since 1970), at planar meters north and east of (0, 0) on the
  equator, where navd's tests lay their routes, at speed (m/s) on course (deg), None for an empty field."""
  moment = datetime.datetime.fromtimestamp(round(t, 2), datetime.UTC)
  hms, dmy = f'{moment:%H%M%S}.{moment.microsecond // 10000:02d}', f'{moment:%d%m%y}'
  (lat, ns), (lon, ew) = angle(north / METERS_PER_DEGREE, 2, 'NS'), angle(east / METERS_PER_DEGREE, 3, 'EW')
  knots, heading = '' if speed is None else f'{speed / KNOTS:.4f}', '' if course is None else f'{course:.2f}'
  return [nmea(f'{talker}GGA,{hms},{lat},{ns},{lon},{ew},{quality},{satellites},{hdop},12.5,M,-31.2,M,,'),
          nmea(f'{talker}RMC,{hms},{status},{lat},{ns},{lon},{ew},{knots},{heading},{dmy},,,A')]


def take(fixes: PhoneFixes, lines: list[bytes], received: float, wall: float):
  """The message published once these lines arrived together at monotonic time received, wall on the comma's clock."""
  for line in lines:
    fixes.add(line, received, wall)
  with patch('time.monotonic', return_value=received):
    return fixes.take(received)


def drive(fixes: PhoneFixes, count: int, start: int = 0, latency: float = .3, offset: float = 0., speed: float = 10., **kwargs) -> list:
  """The phone's fixes start to start + count - 1, a second apart, driving north from 100 m at speed, each arriving latency
  after it was taken, on a comma clock offset from the phone's: the messages published."""
  published = []
  for i in range(start, start + count):
    message = take(fixes, epoch(T0 + i, north=100 + speed * i, speed=speed, **kwargs), R0 + i + latency, T0 + i + latency + offset)
    if message is not None:
      published.append(message)
  return published


def phone_clock() -> float:
  """The phone's clock in the socket tests: the comma's own, as when both have network time."""
  return time.time()  # noqa: TID251


def gps_submaster():
  with patch.object(messaging, 'sub_sock'):
    sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal'])
  sm.update = lambda timeout=100: None  # the test hands it its messages
  return sm


def gps_message(service: str, mono: float, source: str, valid: bool = True, fix: bool = True):
  msg = messaging.new_message(service, valid=valid, logMonoTime=int(mono * 1e9))
  setattr(msg, service, {'hasFix': fix, 'source': source, 'latitude': 37., 'longitude': -122.})
  return messaging.log_from_bytes(msg.to_bytes())


def source(module: str) -> ast.Module:
  return ast.parse(Path(BASEDIR, *module.split('.')).with_suffix('.py').read_text())


def assigned(tree: ast.Module, target: str) -> ast.expr:
  return next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == target)


def method(tree: ast.Module, cls: str, name: str) -> ast.FunctionDef:
  owner = next(node for node in ast.walk(tree) if isinstance(node, ast.ClassDef) and node.name == cls)
  return next(node for node in owner.body if isinstance(node, ast.FunctionDef) and node.name == name)


class TestNmea(OpenpilotTestCase):
  def test_sentences(self):
    rmc = nmea('GPRMC,183000.00,A,0000.053955,N,00000.000000,E,19.4384,0.00,041126,,,A').strip()
    fields = ['183000.00', 'A', '0000.053955', 'N', '00000.000000', 'E', '19.4384', '0.00', '041126', '', '', 'A']
    cases = [
      (rmc, ('RMC', fields)),
      (rmc.replace(b'$GP', b'$GN')[:-2] + f"{int(rmc[-2:], 16) ^ ord('P') ^ ord('N'):02x}".encode(), ('RMC', fields)),  # any talker, lowercase hex
      (b'1.2,M,,*47' + rmc, ('RMC', fields)),  # after the tail of a sentence cut off
      (rmc[:-1] + (b'0' if rmc[-1:] != b'0' else b'1'), None),  # a wrong checksum
      (rmc.split(b'*')[0], None),  # none
      (rmc + b'0', None),  # three digits
      (rmc[:30], None),  # cut off
      (nmea('PASHR,183000.00,90.0,T,0.1,0.2,,0.1,0.1,0.5,1,0').strip(), None),  # proprietary
      (nmea('GPRMC,183000.00,A,0000.05é955,N').strip(), None),  # not ASCII
      (nmea('GPRMC,' + 'x' * 300).strip(), None),  # too long for NMEA
      (b'hello', None),
    ]
    for line, expected in cases:
      assert sentence(line) == expected, line

  def test_rmc(self):
    def rmc(**changes):
      fields = {'time': '183000.00', 'status': 'A', 'lat': '3746.6340', 'ns': 'N', 'lon': '12226.3260', 'ew': 'W', 'knots': '19.4384',
                'course': '84.40', 'date': '041126', 'mode': 'A', **changes}
      return parse_rmc([fields['time'], fields['status'], fields['lat'], fields['ns'], fields['lon'], fields['ew'], fields['knots'],
                        fields['course'], fields['date'], '', '', fields['mode']])
    fix = rmc()
    assert fix.time == T0 and abs(fix.latitude - (37 + 46.634 / 60)) < 1e-9 and abs(fix.longitude + (122 + 26.326 / 60)) < 1e-9
    assert abs(fix.speed - 10.) < 1e-4 and fix.course == 84.4
    assert rmc(ns='S', ew='E').latitude < 0 < rmc(ns='S', ew='E').longitude
    assert rmc(course='360').course == 0 and rmc(course='').course is None and rmc(course='-1.0').course is None
    assert rmc(knots='').speed is None and rmc(knots='-1.9').speed is None  # the phone's own for no speed
    assert rmc(knots='0.0').speed == 0.
    for changes, reason in [
      ({'status': 'V'}, 'no_fix'), ({'mode': 'N'}, 'no_fix'), ({'mode': 'E'}, 'no_fix'), ({'mode': 'S'}, 'no_fix'),
      ({'lat': '', 'ns': '', 'lon': '', 'ew': ''}, 'no_fix'), ({'lat': '0000.0000', 'lon': '00000.0000'}, 'no_fix'),
      ({'lat': '3760.0000'}, 'unreadable'), ({'lat': '9100.0000'}, 'unreadable'), ({'lon': '18100.0000'}, 'unreadable'),
      ({'ns': 'E'}, 'unreadable'), ({'ns': ''}, 'unreadable'), ({'lat': '-3746.6340'}, 'unreadable'), ({'lat': 'nan'}, 'unreadable'),
      ({'time': '63000 PM'}, 'unreadable'), ({'time': '18:30:00'}, 'unreadable'), ({'time': '246000'}, 'unreadable'),
      ({'date': '310226'}, 'unreadable'), ({'date': ''}, 'unreadable'), ({'knots': '500'}, 'unreadable'),
    ]:
      assert rmc(**changes) == reason, changes
    assert parse_rmc(['183000.00', 'A', '3746.6340', 'N']) == 'unreadable'
    # without the mode, from before NMEA 2.3
    assert parse_rmc(['183000.00', 'A', '3746.6340', 'N', '12226.3260', 'W', '19.4384', '84.40', '041126']).time == T0

  def test_gga(self):
    def gga(quality='1', satellites='09', hdop='0.9', altitude='12.5', separation='-31.2', time='183000.00'):
      return parse_gga([time, '3746.6340', 'N', '12226.3260', 'W', quality, satellites, hdop, altitude, 'M', separation, 'M', '', ''])
    assert gga() == Gga(66600., True, 9, .9, 12.5 - 31.2)  # above the ellipsoid
    assert gga(separation='') == Gga(66600., True, 9, .9, 12.5)
    assert gga(quality='2', satellites='', hdop='', altitude='') == Gga(66600., True)
    assert gga(satellites='120', hdop='0') == Gga(66600., True, None, None, 12.5 - 31.2)  # impossible values are unknown
    assert gga(altitude='20000') == Gga(66600., True, 9, .9, None)
    for quality in ('0', '6', '7', '8'):  # none, dead reckoning, manual, simulator
      assert gga(quality=quality) == Gga(66600., False)
    assert gga(quality='') == 'unreadable' and gga(time='1830') == 'unreadable'
    assert parse_gga(['183000.00', '', '', '', '', '1', '00', '']) == Gga(66600., False)  # no position
    assert parse_gga(['183000.00', '3746.6340', 'N', '12226.3260', 'W', '1', '09', '0.9']) == Gga(66600., True, 9, .9, None)

  def test_vtg(self):
    assert parse_vtg(['84.4', 'T', '', 'M', '19.4384', 'N', '36.0', 'K', 'A'])[1] == 84.4
    speed, course = parse_vtg(['', 'T', '', 'M', '', 'N', '36.0', 'K', 'A'])
    assert abs(speed - 10.) < 1e-9 and course is None
    assert parse_vtg(['84.4', 'T', '', 'M', '19.4384', 'N', '36.0', 'K', 'N']) is None
    assert parse_vtg(['84.4', 'T']) is None and parse_vtg(['84.4', 'T', '', 'M', '500', 'N', '', 'K']) is None


class TestPublishing(OpenpilotTestCase):
  def test_fields(self):
    fixes = PhoneFixes()
    assert drive(fixes, 1) == [] and fixes.data_state(R0 + .3) == 'starting'  # the first fix waits for a second to agree with it
    [message] = drive(fixes, 1, start=1, speed=10., course=30.)
    assert message.valid and message.logMonoTime == int((R0 + 1.3) * 1e9)  # as it arrived: navd counts its age from there
    gps = message.gpsLocation
    assert gps.source == PHONE and gps.hasFix and gps.flags == 3  # a fix, with its own epoch's GGA
    assert abs(gps.latitude * METERS_PER_DEGREE - 110.) < .01 and gps.longitude == 0.
    assert abs(gps.altitude - (12.5 - 31.2)) < 1e-6 and abs(gps.speed - 10.) < 1e-3 and abs(gps.bearingDeg - 30.) < 1e-4
    assert abs(gps.bearingAccuracyDeg - math.degrees(math.atan2(SPEED_ERROR, 10.))) < 1e-3 and abs(gps.speedAccuracy - SPEED_ERROR) < 1e-6
    assert abs(gps.horizontalAccuracy - NOMINAL_ERROR * .9) < 1e-4 and abs(gps.verticalAccuracy - VDOP_RATIO * NOMINAL_ERROR * .9) < 1e-4
    assert gps.satelliteCount == 9 and gps.unixTimestampMillis == (T0 + 1) * 1000
    assert [round(v, 3) for v in gps.vNED] == [round(10. * math.cos(math.radians(30)), 3), round(10. * math.sin(math.radians(30)), 3), 0.]
    assert fixes.data_state(R0 + 1.3) == 'fixes'
    # a good HDOP still means some meters
    [message] = drive(fixes, 1, start=2, hdop='0.3')
    assert message.gpsLocation.horizontalAccuracy == MIN_ACCURACY

  def test_once_per_fix(self):
    fixes = PhoneFixes()
    assert len(drive(fixes, 3)) == 2
    # the same fix again, or an older one, never goes out again; nothing new, nothing goes out
    assert take(fixes, epoch(T0 + 2, north=120), R0 + 2.5, T0 + 2.5) is None
    assert take(fixes, epoch(T0 + 1.5, north=115), R0 + 2.6, T0 + 2.6) is None
    assert take(fixes, [], R0 + 2.7, T0 + 2.7) is None
    # several fixes at once: only the newest goes out
    lines = [line for i in (3, 4, 5) for line in epoch(T0 + i, north=100 + 10 * i)]
    message = take(fixes, lines, R0 + 5.3, T0 + 5.3)
    assert message.gpsLocation.unixTimestampMillis == (T0 + 5) * 1000 and take(fixes, [], R0 + 5.4, T0 + 5.4) is None
    # once the phone stops, nothing goes out: consumers tell a stale fix by its age
    for now in (6.3, 7.3, 9.):
      assert take(fixes, [], R0 + now, T0 + now) is None
    assert fixes.data_state(R0 + 9.) is None

  def test_any_talker(self):
    for talker in ('GP', 'GN', 'GL', 'GA', 'BD'):
      assert len(drive(PhoneFixes(), 2, talker=talker)) == 1, talker

  def test_epochs_join(self):
    fixes = PhoneFixes()
    drive(fixes, 2)
    # RMC before its GGA, both in one read
    gga, rmc = epoch(T0 + 2, north=120, hdop='1.4')
    assert take(fixes, [rmc, gga], R0 + 2.3, T0 + 2.3).gpsLocation.flags == 3
    # the GGA only in the next read: the last one's HDOP and satellites stand for it, from up to 3 s before
    gga, rmc = epoch(T0 + 3, north=130, hdop='2.0', satellites='07')
    message = take(fixes, [rmc], R0 + 3.3, T0 + 3.3)
    assert message.gpsLocation.flags == 1 and abs(message.gpsLocation.horizontalAccuracy - 7.) < 1e-4
    assert take(fixes, [gga], R0 + 3.31, T0 + 3.31) is None
    # its own epoch's GGA without a fix: none
    no_fix, rmc = epoch(T0 + 4, north=140, quality='0')
    assert take(fixes, [no_fix, rmc], R0 + 4.3, T0 + 4.3) is None and fixes.heard['no_fix'] == R0 + 4.3
    # a phone that sends no GGA: accuracy and satellites unknown
    fixes = PhoneFixes()
    for i in range(2):
      message = take(fixes, epoch(T0 + i, north=100 + 10 * i)[1:], R0 + i + .3, T0 + i + .3)
    assert message.gpsLocation.flags == 1 and message.gpsLocation.horizontalAccuracy == 0 and message.gpsLocation.satelliteCount == 0
    # a VTG gives an RMC's missing speed and course
    _, rmc = epoch(T0 + 2, north=120, speed=None, course=None)
    vtg = nmea('GPVTG,45.0,T,,M,,N,36.0,K,A')
    message = take(fixes, [rmc, vtg], R0 + 2.3, T0 + 2.3)
    assert abs(message.gpsLocation.speed - 10.) < 1e-6 and message.gpsLocation.bearingDeg == 45.

  def test_rejected(self):
    cases = [
      ({'hdop': '12.0'}, 'inaccurate'),  # a position from Wi-Fi or cell towers
      ({'speed': None}, 'incomplete'),  # navd counts the car's travel since the fix by its speed
      ({'course': None}, 'incomplete'),  # and judges a moving car's course
      ({'status': 'V'}, 'no_fix'),
      ({'quality': '0'}, 'no_fix'),
      ({'north': 400.}, 'jump'),  # 280 m in a second at 10 m/s
    ]
    for changes, reason in cases:
      fixes = PhoneFixes()
      assert len(drive(fixes, 2)) == 1
      assert take(fixes, epoch(T0 + 2, **{'north': 120., **changes}), R0 + 2.3, T0 + 2.3) is None, changes
      assert fixes.heard[reason] == R0 + 2.3 and fixes.data_state(R0 + 4.4) == reason, changes
      # the next good fix goes out again
      assert len(drive(fixes, 1, start=3)) == 1, changes
    # what comes in when there are no fixes to tell
    for lines, state in [([b'$GPRMC,garbage*00', b'\x00\xff'], 'unreadable'), (epoch(T0)[:1], 'no_rmc'),
                         (epoch(T0)[:1] + [nmea('GPGSV,1,1,00')], 'no_rmc'), (epoch(T0, quality='0')[:1], 'no_fix')]:
      fixes = PhoneFixes()
      assert take(fixes, lines, R0, T0) is None and fixes.data_state(R0) == state, lines

  def test_slow_course(self):
    # A course comes from the velocity: navd widens its course thresholds by its accuracy (never 0 or 180, which it takes
    # at its word), and below 1 m/s the moving car's last course is kept
    fixes = PhoneFixes()
    drive(fixes, 2, course=20.)
    accuracies = {}
    for i, (speed, course) in enumerate([(2., 25.), (.5, 200.), (0., None), (0., 270.)], start=2):
      [message] = drive(fixes, 1, start=i, speed=speed, course=course)
      gps = message.gpsLocation
      accuracies[speed] = gps.bearingAccuracyDeg
      assert gps.bearingDeg == 25. and 0 < gps.bearingAccuracyDeg < 180, (speed, course)
    assert abs(accuracies[2.] - math.degrees(math.atan2(SPEED_ERROR, 2.))) < 1e-3 and accuracies[0.] == 90.

  def test_late_fixes(self):
    # Fixes held up on the way, as when the link stalls, arrive late: where they were taken, the car no longer is. With the
    # two clocks differing, only the phone's fastest lately tells: one more than MAX_LATE (1.5 s) later than it is dropped
    fixes = PhoneFixes()
    assert len(drive(fixes, 2, offset=60.)) == 1
    published = []
    for k, i in enumerate((2, 3, 4, 5)):  # held up, arriving one after another once the link is back
      published.append(take(fixes, epoch(T0 + i, north=100 + 10 * i), R0 + 5.3 + .05 * k, T0 + 60. + 5.3 + .05 * k))
    assert published[:2] == [None, None] and fixes.heard['late'] == R0 + 5.35  # 3 and 2.05 s later than the fastest
    assert None not in published[2:]  # 1.1 s later, then on time again
    assert published[3].gpsLocation.unixTimestampMillis == round((T0 + 60. + 5.45) * 1000)  # the arrival's: the clocks differ
    # a fix the phone kept from before, first after a while, never goes out: two in a row must agree first
    fixes = PhoneFixes()
    assert take(fixes, epoch(T0 - 300, north=50), R0, T0) is None
    assert drive(fixes, 1) == []
    assert len(drive(fixes, 1, start=1)) == 1
    # nor do two held up and arriving together: with the clocks differing as they don't agree, else as too old
    for offset, reason in ((60., 'starting'), (0., 'delayed')):
      fixes = PhoneFixes()
      assert take(fixes, epoch(T0 - 10, north=0.5), R0, T0 + offset) is None
      assert take(fixes, epoch(T0 - 9, north=10.5), R0 + .05, T0 + .05 + offset) is None
      assert fixes.heard[reason] == R0 + .05 and fixes.published is None, offset

  def test_timer_sending(self):
    # An app that sends the newest fix on its own 1 s timer, the phone taking its fixes about when the timer fires: a fix
    # taken just after it fired goes out a second later, the network's jitter on top, the next one taken just before at
    # once, and one is sent twice or never. None of them is late, though a second later than the fastest lately.
    fixes = PhoneFixes()
    sent = [(-.02, .03), (.98, 1.03), (.98, 2.03), (2.02, 3.1), (3.98, 4.03), (3.98, 5.03), (5.02, 6.12), (6.98, 7.03)]  # (taken, arrived) s
    published = [take(fixes, epoch(T0 + taken, north=100 + 10 * taken), R0 + arrived, T0 + arrived) is not None for taken, arrived in sent]
    assert published == [False, True, False, True, True, False, True, True] and 'late' not in fixes.heard

  def test_delayed_fixes(self):
    # While the two clocks agree, a fix's own time tells how long it took to arrive: one taken longer ago than navd allows
    # for is never used, however steadily they all come so late, as from an app iOS holds back in the background
    for latency in (PHONE_MAX_DELAY + .05, 3., 5., CLOCK_DIFFERENCE - .05):
      fixes = PhoneFixes()
      assert drive(fixes, 20, latency=latency) == [] and fixes.data_state(R0 + 19 + latency) == 'delayed', latency
    # up to it, they go out with their own time, which navd counts their age from
    fixes = PhoneFixes()
    published = drive(fixes, 20, latency=PHONE_MAX_DELAY - .05)
    assert len(published) == 19 and published[-1].gpsLocation.unixTimestampMillis == (T0 + 19) * 1000 and fixes.clocks_agree
    # one held up past it among them is dropped, and the next goes out
    assert take(fixes, epoch(T0 + 20, north=300), R0 + 22.1, T0 + 22.1) is None and fixes.heard['delayed'] == R0 + 22.1
    assert len(drive(fixes, 1, start=21, latency=PHONE_MAX_DELAY - .05)) == 1

  def test_phone_clock_goes_back(self):
    # its fixes are then old, then late, until enough came on time to start over
    fixes = PhoneFixes()
    assert len(drive(fixes, 3)) == 2
    published = []
    for i in range(3, 3 + 3 + LATE_RESTART + 2):
      t = T0 + i - 3
      message = take(fixes, epoch(t, north=100 + 10 * i), R0 + i + .3, t + .3)
      published.append(message is not None)
    assert published == [False] * (3 + LATE_RESTART + 1) + [True]

  def test_phone_clock_runs_ahead(self):
    # a fix whose time ran ahead of when it came starts the fixes over: one wrong time never goes out, nor changes what
    # counts as late after it
    for jump, published in ((30., [False, True, True]), (-0., [True, True, True])):
      fixes = PhoneFixes()
      assert len(drive(fixes, 3)) == 2
      outcome = [take(fixes, epoch(T0 + i + jump, north=100 + 10 * i), R0 + i + .3, T0 + i + .3) is not None for i in (3, 4, 5)]
      assert outcome == published, jump
    fixes = PhoneFixes()
    assert len(drive(fixes, 3)) == 2
    assert take(fixes, epoch(T0 + 103, north=130), R0 + 3.3, T0 + 3.3) is None  # one wrong time
    assert [take(fixes, epoch(T0 + i, north=100 + 10 * i), R0 + i + .3, T0 + i + .3) is not None for i in (4, 5, 6)] == [False, True, True]


class TestClockDifference(OpenpilotTestCase):
  """A phone fix reaches the comma a while after it was taken, while the car drives on: navd's distance to the turn must
  allow for that while the comma's clock and the phone's agree, and not change with their difference otherwise, also a
  large one before the comma has had network time."""

  def instruction(self, offset: float, delay: float, latency: float = .3, speed: float = 10., received: float = 0.,
                  ublox: bool = False) -> tuple[float, float, int]:
    """navd's distance to the route's first turn delay s after the phone's third fix arrived, the fixes arriving latency s
    after they were taken, with the comma's clock offset s from the phone's, and navd receiving that fix received s after it
    arrived; the car's own distance then, driving on at speed (m/s); and that fix's unixTimestampMillis. ublox: the same fix
    comes from the u-blox instead, on gpsLocationExternal."""
    fixes = PhoneFixes()
    *_, message = drive(fixes, 3, latency=latency, offset=offset, speed=speed)
    fix = message.gpsLocation
    if ublox:
      message = messaging.new_message('gpsLocationExternal', valid=True, logMonoTime=message.logMonoTime)
      message.gpsLocationExternal = fix
      message.gpsLocationExternal.source = 'ublox'
    arrived = R0 + 2 + latency
    sm, pm = gps_submaster(), MagicMock()
    sm.update_msgs(arrived + received, [messaging.log_from_bytes(message.to_bytes())])
    engine = route_engine({'NavDestination': {'latitude': 500 / METERS_PER_DEGREE, 'longitude': 200 / METERS_PER_DEGREE}}, sm, pm)
    response = MagicMock(status_code=200)
    response.json.return_value = {'routes': [{'legs': [{'steps': ROUTE}]}]}  # 300 m north to a right turn
    with patch('requests.get', return_value=response), patch('time.monotonic', return_value=arrived + delay), \
         patch('time.time', return_value=T0 + 2 + latency + offset + delay):
      engine.update()
    instruction = next(call.args[1] for call in reversed(pm.send.call_args_list) if call.args[0] == 'navInstruction')
    assert instruction.valid
    return instruction.navInstruction.maneuverDistance, 300. - (100. + speed * (2 + latency + delay)), fix.unixTimestampMillis

  def test_delay_allowed_for(self):
    # with the clocks agreeing, navd counts a fix's age from when it was taken: its distance is the car's, however long the
    # fix took to arrive, up to PHONE_MAX_DELAY, where it went out late by speed times that before
    for latency in (.05, .3, 1., PHONE_MAX_DELAY - .05):
      for speed in (8., 15., 25.):
        distance, true, _ = self.instruction(0., .6, latency, speed)
        assert abs(distance - true) < .01, (latency, speed, distance, true)
    # a difference within the agreement moves the distance by no more than the car drives in it
    for offset in (-.5, .5):
      distance, true, _ = self.instruction(offset, .6)
      assert abs(distance - true) <= 10. * abs(offset) + .01, (offset, distance, true)

  def test_phone_fixes_stop(self):
    # the phone's fixes stop after one navd received at its next update, up to a second after it was published. navd goes
    # on with it for 2 s after that (latest_gps_source): a second later and, as its updates jitter, just under 2 s later, up
    # to 3 s after it was published. All of that counts, where 1.5 s did and the distance came out up to 35 m long.
    for received, delay in ((.6, 1.6), (.6, 2.55), (.95, 1.95), (.95, 2.9)):
      for speed in (8., 15., 25.):
        distance, true, _ = self.instruction(0., delay, speed=speed, received=received)
        assert abs(distance - true) < .5, (received, delay, speed, distance, true)

  def test_ublox_age_unchanged(self):
    # the same fix from the u-blox, read as late: its age counts up to MAX_FIX_AGE, as before
    for received, delay in ((0., .6), (.6, 1.6), (.6, 2.55), (.95, 1.95), (.95, 2.9)):
      for speed in (8., 15., 25.):
        distance, true, _ = self.instruction(0., delay, 0., speed, received, ublox=True)
        assert abs(distance - (true + speed * max(delay - MAX_FIX_AGE, 0.))) < .01, (received, delay, speed, distance, true)

  def test_comma_clock_off(self):
    # clocks that differ more: a fix counts from its arrival, the same whatever the difference
    for delay in (.6, 1.9):  # the age navd counts the car's travel for, up to PHONE_MAX_FIX_AGE
      distances = {offset: self.instruction(offset, delay)[0] for offset in (CLOCK_DIFFERENCE + .5, 60., -5., -60.)}
      assert len(set(distances.values())) == 1, distances
      assert abs(distances[60.] - (180. - 10. * min(delay, PHONE_MAX_FIX_AGE))) < .01, distances
    # a difference that could as well be a delay (test_delayed_fixes): no fix goes out
    fixes = PhoneFixes()
    assert drive(fixes, 3, offset=5.) == [] and fixes.data_state(R0 + 2.3) == 'delayed'

  def test_fix_time(self):
    # the phone's own time goes out only while the comma's clock agrees with it, else the arrival's on the comma's clock
    for offset in (0., CLOCK_AGREEMENT - .4, -CLOCK_AGREEMENT + .4, PHONE_MAX_DELAY - .4):
      assert self.instruction(offset, .6)[2] == (T0 + 2) * 1000, offset
    for offset in (CLOCK_DIFFERENCE + .5, 60., -5., -60.):
      assert self.instruction(offset, .6)[2] == round((T0 + 2.3 + offset) * 1000), offset


class TestGpsSource(OpenpilotTestCase):
  def choose(self, sm, now: float) -> str | None:
    with patch('time.monotonic', return_value=now):
      return latest_gps_source(sm)

  def test_phone_first(self):
    sm = gps_submaster()
    sm.update_msgs(T, [gps_message('gpsLocation', T - .5, 'iOS'), gps_message('gpsLocationExternal', T, 'ublox')])
    assert self.choose(sm, T) == 'gpsLocation'  # though the u-blox's is newer
    assert self.choose(sm, T + 1.99) == 'gpsLocation'
    assert self.choose(sm, T + 2.) is None  # both stale
    for now, changes in ((T + 3, {'valid': False}), (T + 3.1, {'fix': False})):
      sm.update_msgs(now, [gps_message('gpsLocation', now, 'iOS', **changes), gps_message('gpsLocationExternal', now, 'ublox')])
      assert self.choose(sm, now) == 'gpsLocationExternal', changes
    sm.update_msgs(T + 4, [gps_message('gpsLocation', T + 4, 'iOS')])
    sm.update_msgs(T + 6.5, [gps_message('gpsLocationExternal', T + 6.5, 'ublox')])
    assert self.choose(sm, T + 6.5) == 'gpsLocationExternal'  # the phone's stopped

  def test_no_flapping(self):
    # the u-blox sends ten fixes a second, the phone one: while the phone's last is fresh, it's the one
    sm, chosen = gps_submaster(), []
    for tick in range(150):
      now = T + tick / 10
      messages = [gps_message('gpsLocationExternal', now, 'ublox')]
      if tick % 10 == 3 and tick < 100:  # the phone stops after its fix at 9.3 s
        messages.append(gps_message('gpsLocation', now, 'iOS'))
      sm.update_msgs(now, messages)
      chosen.append(self.choose(sm, now))
    assert set(chosen[:3]) == {'gpsLocationExternal'} and set(chosen[3:112]) == {'gpsLocation'} and set(chosen[114:]) == {'gpsLocationExternal'}

  def test_others_unchanged(self):
    # qcomgpsd's gpsLocation, on devices without a u-blox, and fixes without a source: the newest wins, as before
    for other in ('qcomdiag', 'android'):
      sm = gps_submaster()
      sm.update_msgs(T, [gps_message('gpsLocation', T - .5, other), gps_message('gpsLocationExternal', T, 'ublox')])
      assert self.choose(sm, T) == 'gpsLocationExternal'
      sm.update_msgs(T + .1, [gps_message('gpsLocation', T + .1, other)])
      assert self.choose(sm, T + .1) == 'gpsLocation'


class TestGateway(OpenpilotTestCase):
  def test_default_gateway(self):
    assert default_gateway(ROUTE_TABLE) == '172.20.10.1'
    assert default_gateway(ROUTE_TABLE, 'rmnet_data0') == '10.0.0.1'
    assert default_gateway(ROUTE_TABLE.replace('wlan0\t00000000\t010A14AC\t0003', 'wlan0\t00000000\t010A14AC\t0001')) is None  # no gateway flag
    assert default_gateway(ROUTE_TABLE.replace('wlan0', 'wlan1')) is None
    assert default_gateway('') is None and default_gateway('Iface\nwlan0\tzz\n') is None
    # the lower metric wins
    better = ROUTE_TABLE + 'wlan0\t00000000\t0101A8C0\t0003\t0\t0\t100\t00000000\t0\t0\t0\n'
    assert default_gateway(better) == '192.168.1.1'

  def test_params(self):
    header = (Path(BASEDIR) / 'openpilot/common/params_keys.h').read_text()
    entries = {name: [part.strip() for part in body.split(',')] for name, body in re.findall(r'\{"(PhoneGps\w*)",\s*\{([^{}]*)\}\}', header)}
    assert entries == {'PhoneGps': ['PERSISTENT | BACKUP', 'BOOL', '"0"'], 'PhoneGpsHost': ['PERSISTENT', 'STRING'],
                       'PhoneGpsPort': ['PERSISTENT', 'INT', '"11123"'],
                       'PhoneGpsStatus': ['CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION', 'JSON']}
    params = Params()
    assert not params.get_bool('PhoneGps') and port_param(params) == 11123 and host_param(params) is None
    for port, expected in ((2947, 2947), (0, 11123), (70000, 11123)):
      params.put('PhoneGpsPort', port)
      assert port_param(params) == expected
    for host, expected in ((' 172.20.10.5 ', '172.20.10.5'), ('phone.local', None), ('::1', None), ('', None)):
      params.put('PhoneGpsHost', host)
      assert host_param(params) == expected

  def test_process(self):
    process = next(p for p in process_config.procs if p.name == 'phonegpsd')
    assert process.module == 'openpilot.sunnypilot.navd.phonegpsd'
    params = Params()
    for started, navigation, phone, ublox in itertools.product((False, True), repeat=4):
      params.put_bool('NavigationEnabled', navigation)
      params.put_bool('PhoneGps', phone)
      with patch.object(process_config, 'ublox_available', return_value=ublox):
        assert process.should_run(started, params, None) == (started and navigation and phone and ublox)


class TestSources(OpenpilotTestCase):
  """phonegpsd's sockets, on the loopback: the phone is 127.0.0.1 unless the route says otherwise."""

  def phone_gps(self, port: int, gateway: str = '0100007F') -> PhoneGps:
    directory = tempfile.TemporaryDirectory()
    self.addCleanup(directory.cleanup)
    self.route = Path(directory.name) / 'route'
    self.route.write_text(ROUTE_TABLE.replace('010A14AC', gateway))
    params = Params()
    params.put('PhoneGpsPort', port)
    gps = PhoneGps(params, MagicMock(), route_file=str(self.route), wifi=lambda: '127.0.0.1')
    self.addCleanup(gps.close)
    return gps

  def server(self) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.addCleanup(server.close)
    server.bind(('127.0.0.1', 0))
    server.listen(4)
    server.settimeout(5)
    return server

  def run_until(self, gps: PhoneGps, done, timeout: float = 5.) -> None:
    deadline = time.monotonic() + timeout
    while not done():
      assert time.monotonic() < deadline, 'timed out'
      gps.step(.02)

  def published(self, gps: PhoneGps) -> list:
    return [call.args[1] for call in gps.pm.send.call_args_list if call.args[0] == 'gpsLocation']

  def send_fixes(self, gps: PhoneGps, send, count: int = 2) -> None:
    """count fixes, each taken as it's sent, 0.2 s apart, every one read before the next"""
    for i in range(count):
      before = gps.fixes.heard.get('rmc')
      send(b''.join(epoch(phone_clock(), north=100 + 2 * i)))
      self.run_until(gps, lambda before=before: gps.fixes.heard.get('rmc') != before)
      time.sleep(.2)

  def test_tcp(self):
    server = self.server()
    gps = self.phone_gps(server.getsockname()[1])
    self.run_until(gps, lambda: gps.connected)
    connection, peer = server.accept()
    self.addCleanup(connection.close)
    assert peer[0] == '127.0.0.1' and gps.phone == '127.0.0.1'
    self.send_fixes(gps, connection.sendall)
    assert len(self.published(gps)) == 1 and self.published(gps)[0].gpsLocation.source == PHONE
    status = {'host': '127.0.0.1', 'port': gps.port, 'link': 'connected', 'data': 'fixes', 'clock': True}
    self.run_until(gps, lambda: gps.params.get('PhoneGpsStatus') == status)
    # what the navigation page then says, reading the fix 0.3 s after it was taken on the comma's clock
    message = self.published(gps)[0]
    navweb, sm = NavWeb(FakeParams(IsOffroad=False, UbloxAvailable=True, PhoneGps=True, PhoneGpsStatus=status)), submaster()
    with patch('time.time', return_value=message.gpsLocation.unixTimestampMillis / 1000 + .3):
      receive(navweb, sm, message.logMonoTime * 1e-9, manager_state(pigeond=RUNNING, ubloxd=RUNNING, qcomgpsd={}, phonegpsd=RUNNING), message)
    assert page_state(navweb, message.logMonoTime * 1e-9 + .2)['gpsDetails']['summary'][-2:] == [
      'Phone GPS: connected to the phone at 127.0.0.1 · last fix 0.2 s ago · taken 0.3 s before it arrived · accuracy 4 m · the app reports 9 satellites',
      "Navigation uses the phone's GPS"]
    # the tail of a line cut off, then the next fix a byte at a time
    connection.sendall(b'0.9,M,,*47\r\n')
    data = b''.join(epoch(phone_clock(), north=104))
    for i in range(len(data)):
      connection.sendall(data[i:i + 1])
      gps.step(.002)
    self.run_until(gps, lambda: len(self.published(gps)) == 2)
    assert gps.fixes.heard['unreadable'] > 0
    # the Wi-Fi's gateway changes: the connection to the old one ends
    self.route.write_text(ROUTE_TABLE.replace('010A14AC', '01FFFF0A'))  # 10.255.255.1
    gps.lookup_time = 0.
    gps.step(.01)
    assert gps.phone == '10.255.255.1' and not gps.connected
    connection.settimeout(5)
    assert connection.recv(100) == b''

  def test_tcp_only_to_the_gateway(self):
    # never listening for a connection: the phone is only ever the address phonegpsd connects to
    tree = source('openpilot.sunnypilot.navd.phonegpsd')
    assert not [node for node in ast.walk(tree) if isinstance(node, ast.Attribute) and node.attr in ('listen', 'accept')]
    server = self.server()
    gps = self.phone_gps(server.getsockname()[1], gateway='01FFFF0A')  # 10.255.255.1, not the server's address
    deadline = time.monotonic() + 1.
    while time.monotonic() < deadline:
      gps.step(.02)
    server.settimeout(.1)
    with self.assertRaises(TimeoutError):
      server.accept()
    assert gps.phone == '10.255.255.1' and not gps.connected and not self.published(gps)

  def test_reconnects(self):
    server = self.server()
    gps = self.phone_gps(server.getsockname()[1])
    with patch.object(phonegpsd, 'RETRY_DELAYS', (.05, .1)):
      for _ in range(3):  # the phone ends each connection: phonegpsd connects again
        self.run_until(gps, lambda: gps.connected)
        connection, _ = server.accept()
        connection.close()
        self.run_until(gps, lambda: not gps.connected)
        assert gps.problem == 'closed'
      self.run_until(gps, lambda: gps.connected)
    # nobody on the port: later after each failure in a row, from the first delay again after a connection that lasted
    gps = self.phone_gps(server.getsockname()[1])
    now = time.monotonic()
    delays = []
    for _ in range(6):
      gps.failed(now, 'refused')
      delays.append(gps.retry_time - now)
    assert delays == [1., 2., 4., 8., 8., 8.]
    gps.connected_time = now - 11.
    gps.lost(now, 'closed')
    assert gps.retry_time - now == 1.
    # a real refusal
    server.close()
    self.run_until(gps, lambda: gps.problem == 'refused', timeout=15.)

  def test_no_udp(self):
    # Any device on the hotspot can fake a datagram's sender with a single packet: phonegpsd takes no UDP, not even from the
    # phone's address, and holds no UDP socket
    tree = source('openpilot.sunnypilot.navd.phonegpsd')
    assert not {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)} & {'SOCK_DGRAM', 'recvfrom', 'recvmsg', 'recvfrom_into'}
    server = self.server()
    port = server.getsockname()[1]
    gps = self.phone_gps(port)  # the phone is 127.0.0.1, where the datagrams come from too
    self.run_until(gps, lambda: gps.connected)
    connection, _ = server.accept()
    self.addCleanup(connection.close)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.addCleanup(sender.close)
    sender.bind(('127.0.0.1', 0))
    for i in range(3):
      sender.sendto(b''.join(epoch(phone_clock(), north=100 + 2 * i)), ('127.0.0.1', port))
      deadline = time.monotonic() + .3
      while time.monotonic() < deadline:
        gps.step(.02)
    assert gps.connected and not self.published(gps) and not gps.fixes.heard
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.addCleanup(probe.close)
    probe.bind(('127.0.0.1', port))  # free: phonegpsd has no UDP socket on its port


class TestNavweb(OpenpilotTestCase):
  STATUS = {'host': '172.20.10.1', 'port': 11123, 'link': 'connected', 'data': 'fixes'}

  def summary(self, status=STATUS, processes=None, phone_fix=True, ublox_fix=True, now=T + .5, **params) -> list[str]:
    params = {'IsOffroad': False, 'UbloxAvailable': True, 'PhoneGps': True, 'PhoneGpsStatus': status, **params}
    navweb, sm = NavWeb(FakeParams(**params)), submaster()
    receive(navweb, sm, T, manager_state(**(processes or {'pigeond': RUNNING, 'ubloxd': RUNNING, 'qcomgpsd': {}, 'phonegpsd': RUNNING})))
    messages = []
    if phone_fix:
      messages.append(position('gpsLocation', hasFix=True, latitude=37.6559, longitude=-122.4077, horizontalAccuracy=6., satelliteCount=9, source='iOS'))
    if ublox_fix:
      messages.append(position('gpsLocationExternal', hasFix=True, latitude=37.6558, longitude=-122.4077, horizontalAccuracy=12.,
                               satelliteCount=5, source='ublox'))
    receive(navweb, sm, T + .1, *messages)
    if now > T + 1.1:
      receive(navweb, sm, now)  # nothing since
    return page_state(navweb, now)['gpsDetails']['summary']

  def test_phone_line(self):
    summary = self.summary()
    assert summary[-2:] == ['Phone GPS: connected to the phone at 172.20.10.1 · last fix 0.4 s ago · accuracy 6 m · the app reports 9 satellites',
                            "Navigation uses the phone's GPS"]
    # the u-blox's fresh fix isn't too old: navigation takes the phone's while it has one
    assert summary[1] == "GPS receiver (u-blox): running · last message 0.4 s ago · has a fix, but navigation uses the phone's · 5 satellites used · " + \
      'accuracy 12 m'
    # stale: navigation goes back to the comma's own GPS
    assert self.summary(now=T + 2.5, ublox_fix=False)[-2:] == ['Phone GPS: connected to the phone at 172.20.10.1 · last fix 2.4 s ago · ' +
                                                               'accuracy 6 m · the app reports 9 satellites', 'Navigation has no GPS fix right now']
    assert self.summary({**self.STATUS, 'data': None}, phone_fix=False)[-2:] == [
      'Phone GPS: connected to the phone at 172.20.10.1 · no data from the phone: is the app running?', "Navigation uses the comma's own GPS"]

  def test_links_and_data(self):
    cases = [
      ({'link': 'refused', 'data': None}, 'not connected: the phone at 172.20.10.1 refuses port 11123: is the app sharing in socket mode on that port? · ' +
       'no data from the phone'),
      ({'link': 'unreachable', 'data': None}, "not connected: can't reach the phone at 172.20.10.1: is the comma on the iPhone's hotspot? · " +
       'no data from the phone'),
      ({'link': 'no_wifi', 'host': None, 'data': None}, "not connected: the comma isn't on Wi-Fi: join the iPhone's hotspot · no data from the phone"),
      ({'link': 'connecting', 'data': None}, 'connecting to the phone at 172.20.10.1, port 11123 · no data from the phone'),
      ({'data': 'no_fix'}, 'connected to the phone at 172.20.10.1 · the phone has no GPS fix'),
      ({'data': 'unreadable'}, "connected to the phone at 172.20.10.1 · its data can't be read as NMEA (with GPS2IP, set the iPhone to 24-hour time)"),
      ({'data': 'no_rmc'}, "connected to the phone at 172.20.10.1 · no RMC sentences, which have the speed and date: turn on RMC in the app's " +
       'NMEA settings'),
      ({'data': 'late'}, 'connected to the phone at 172.20.10.1 · its fixes arrive late, held up on the way: not used'),
      ({'data': 'inaccurate'}, 'connected to the phone at 172.20.10.1 · its fixes are too inaccurate to use'),
      ({'data': 'delayed'}, "connected to the phone at 172.20.10.1 · its fixes arrive over 2 s after they were taken: not used (is the iPhone's " +
       'time set automatically?)'),
    ]
    for changes, words in cases:
      line, source = self.summary({**self.STATUS, **changes}, phone_fix=False)[-2:]
      assert line == f'Phone GPS: {words}' and source == "Navigation uses the comma's own GPS", changes
    # what phonegpsd wrote, never shown unless it's words the page knows
    for status in ({**self.STATUS, 'link': '<b>'}, {**self.STATUS, 'link': 'udp'}, {**self.STATUS, 'data': 'other'}, 'connected', None):
      assert self.summary(status, phone_fix=False)[-2] == 'Phone GPS: no fix from the phone yet'
    assert self.summary({**self.STATUS, 'host': '<script>'}, phone_fix=False)[-2].startswith('Phone GPS: connected to the phone at unknown')

  def delay_line(self, clock: bool | None, delay: float) -> str:
    """The Phone GPS line 0.4 s after a phone fix arrived, taken delay s before that on the comma's clock (as navd
    counts it), with what phonegpsd last wrote of the two clocks: whether they agree."""
    navweb, sm = NavWeb(FakeParams(IsOffroad=False, UbloxAvailable=True, PhoneGps=True, PhoneGpsStatus={**self.STATUS, 'clock': clock})), submaster()
    receive(navweb, sm, T, manager_state(pigeond=RUNNING, ubloxd=RUNNING, qcomgpsd={}, phonegpsd=RUNNING))
    fix = messaging.new_message('gpsLocation', valid=True, logMonoTime=int((T + .1) * 1e9))
    fix.gpsLocation = {'hasFix': True, 'latitude': 37.6559, 'longitude': -122.4077, 'horizontalAccuracy': 6., 'source': 'iOS',
                       'unixTimestampMillis': round((T0 - delay) * 1000)}
    with patch('time.time', return_value=T0):  # the comma's clock as it arrives
      receive(navweb, sm, T + .1, fix)
    return page_state(navweb, T + .5)['gpsDetails']['summary'][-2]

  def test_delay(self):
    # how long before it arrived the phone's last fix was taken, which navigation allows for, while the two clocks agree
    assert self.delay_line(True, 1.04) == 'Phone GPS: connected to the phone at 172.20.10.1 · last fix 0.4 s ago · taken 1.0 s before it arrived · ' + \
      'accuracy 6 m'
    assert self.delay_line(True, .02) == 'Phone GPS: connected to the phone at 172.20.10.1 · last fix 0.4 s ago · accuracy 6 m'
    # else the fix went out with its arrival's time: the delay isn't told, nor allowed for
    assert self.delay_line(False, 0.) == "Phone GPS: connected to the phone at 172.20.10.1 · last fix 0.4 s ago · the phone's clock differs from " + \
      "the comma's, so the time its fixes take to arrive isn't allowed for · accuracy 6 m"
    assert self.delay_line(None, 1.) == 'Phone GPS: connected to the phone at 172.20.10.1 · last fix 0.4 s ago · accuracy 6 m'

  def test_not_running(self):
    stopped = {'pigeond': RUNNING, 'ubloxd': RUNNING, 'qcomgpsd': {}, 'phonegpsd': {'shouldBeRunning': True, 'exitCode': 1}}
    assert self.summary(processes=stopped, phone_fix=False)[-2] == 'Phone GPS: not running, phonegpsd stopped with exit code 1'
    assert self.summary(IsOffroad=True, phone_fix=False, ublox_fix=False)[-2:] == ['Phone GPS: off while the car is off',
                                                                                   'Navigation has no GPS fix right now']
    assert self.summary(UbloxAvailable=False, phone_fix=False, ublox_fix=False)[-2] == "Phone GPS: not on this device: it needs the comma 4's u-blox GPS"

  def test_off_as_before(self):
    # with phone GPS off, which no phone fix comes without, the details are as they were, whatever is listed or saved
    assert self.summary(PhoneGps=False, phone_fix=False) == ['Car on', 'GPS receiver (u-blox): running · last message 0.4 s ago · has a fix · ' +
                                                             '5 satellites used · accuracy 12 m']
    navweb, sm = NavWeb(FakeParams(IsOffroad=False, UbloxAvailable=True, PhoneGpsStatus=self.STATUS)), submaster()
    receive(navweb, sm, T, manager_state(pigeond=RUNNING, ubloxd=RUNNING, qcomgpsd={}, phonegpsd=RUNNING))
    assert 'phone' not in json.loads(json.dumps(navweb.state()))['gpsDetails']


class TestSettings(OpenpilotTestCase):
  def test_comma_4_toggle(self):
    tree = source('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models')
    toggle = assigned(tree, 'self.phone_gps_toggle')
    assert getattr(toggle.func, 'id', None) == 'BigParamControl'
    assert ast.unparse(toggle.args[0]) == "tr('phone GPS (alpha)')" and ast.literal_eval(toggle.args[1]) == 'PhoneGps'
    description = next(keyword.value for keyword in toggle.keywords if keyword.arg == 'description')
    text = eval(compile(ast.Expression(description.args[0]), 'description', 'eval'), {})
    for words in ('iPhone app', 'GPS2IP', 'socket mode on port 11123', 'background', "the comma on the iPhone's hotspot"):
      assert words in text, words
    names = [ast.unparse(element) for element in assigned(tree, 'self.main_items').elts]
    assert names[names.index('self.nav_exit_toggle') + 1] == 'self.phone_gps_toggle'
    # shown as off and greyed without navigation, with the reason; the param written only by the driver's tap
    update = ast.unparse(method(tree, 'ModelsLayoutMici', '_update_state'))
    assert "self.phone_gps_toggle.set_checked(navigation and ui_state.params.get_bool('PhoneGps'))" in update
    assert 'self.phone_gps_toggle.set_enabled(navigation)' in update
    assert "phone_value = '' if navigation else tr('needs navigation')" in update and 'self.phone_gps_toggle.set_value(phone_value)' in update
    assert 'self.phone_gps_toggle.refresh()' in ast.unparse(method(tree, 'ModelsLayoutMici', 'show_event'))
    writes = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and getattr(node.func, 'attr', None) in ('put', 'put_bool', 'remove')
              and any(isinstance(arg, ast.Constant) and arg.value == 'PhoneGps' for arg in node.args)]
    assert not writes

  def test_sunnylink_item(self):
    with open(DEFAULT_OUT) as f:
      schema = json.load(f)
    section = next(section for panel in schema['panels'] if panel['id'] == 'steering' for section in panel['sections'] if section['id'] == 'navigation')
    keys = [item['key'] for item in section['items']]
    assert keys[keys.index('NavExitDesire') + 1] == 'PhoneGps'
    item = section['items'][keys.index('PhoneGps')]
    assert item['widget'] == 'toggle' and item['title'] == 'Phone GPS (Alpha)'
    assert item['enablement'] == [{'type': 'param', 'key': 'NavigationEnabled', 'equals': True}]
    for words in ('iPhone app', 'GPS2IP', 'socket mode on port 11123', 'background', "Personal Hotspot"):
      assert words in item['description'], words
