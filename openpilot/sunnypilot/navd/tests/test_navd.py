"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import os
import time
from unittest.mock import MagicMock, patch

import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.navigation.navd import MAX_FIX_AGE, RECOMPUTE_HOLD, REROUTE_COUNTER_MIN, RouteEngine, set_banner
from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE


def route_engine(values, sm=None, pm=None):
  params = MagicMock()
  params.get.side_effect = values.get
  params.remove.side_effect = lambda key: values.pop(key, None)
  with patch('openpilot.selfdrive.navigation.navd.Api') as api:
    api.return_value.get_token.return_value = 'device-token'
    engine = RouteEngine(sm or MagicMock(), pm or MagicMock(), params=params)
  engine.last_position, engine.last_bearing = (0, 0), 90
  return engine


def directions_request(engine):
  response = MagicMock(status_code=200)
  response.json.return_value = {'routes': []}
  with patch('requests.get', return_value=response) as get:
    engine.calculate_route((0.002, 0))
  return get.call_args.args[0], get.call_args.kwargs['params']['access_token']


class TestDirectionsAuth(OpenpilotTestCase):
  def test_personal_token(self):
    url, token = directions_request(route_engine({'MapboxToken': ' pk.personal '}))
    assert url.startswith('https://api.mapbox.com/directions/v5/mapbox/driving-traffic/')
    assert token == 'pk.personal'

  def test_comma_routing_without_token(self):
    url, token = directions_request(route_engine({}))
    assert url.startswith('https://api.commadotai.com/v1/nav/directions/v5/mapbox/driving-traffic/')
    assert token == 'device-token'

  def test_environment_token_wins(self):
    with patch.dict(os.environ, {'MAPBOX_TOKEN': 'pk.environment'}):
      _, token = directions_request(route_engine({'MapboxToken': 'pk.personal'}))
    assert token == 'pk.environment'

  def test_empty_environment_token_uses_comma(self):
    with patch.dict(os.environ, {'MAPBOX_TOKEN': ''}):
      url, token = directions_request(route_engine({}))
    assert url.startswith('https://api.commadotai.com/v1/nav/') and token == 'device-token'

  def test_token_stays_out_of_logs(self):
    engine = route_engine({'MapboxToken': 'pk.secret'})
    error = requests.ConnectionError("Max retries exceeded with url: /directions/v5/mapbox/driving-traffic/0,0;0.002,0?access_token=pk.secret")
    with patch('requests.get', side_effect=error), patch('openpilot.selfdrive.navigation.navd.cloudlog') as cloudlog:
      engine.calculate_route((0.002, 0))
    logged = repr(cloudlog.mock_calls)
    assert 'pk.secret' not in logged and 'ConnectionError' in logged
    assert engine.route is None


class TestLaneGuidance(OpenpilotTestCase):
  def test_lanes(self):
    banner = {'distanceAlongGeometry': 300, 'primary': {'text': 'Exit 12', 'type': 'off ramp', 'modifier': 'slight right'},
              'sub': {'components': [
                {'type': 'lane', 'directions': ['sharp left', 'left'], 'active': False},
                {'type': 'lane', 'directions': ['straight'], 'active': False},
                {'type': 'lane', 'directions': ['straight', 'slight right'], 'active': True, 'active_direction': 'slight right'},
                {'type': 'lane', 'directions': ['uturn'], 'active': False},
                {'type': 'text', 'text': 'or'},
              ]}}
    instruction = messaging.new_message('navInstruction').navInstruction
    set_banner(instruction, [banner], 200)
    lanes = [([str(d) for d in lane.directions], lane.active, str(lane.activeDirection)) for lane in instruction.lanes]
    assert lanes == [(['left', 'left'], False, 'none'), (['straight'], False, 'none'),
                     (['straight', 'slightRight'], True, 'slightRight'), (['none'], False, 'none')]

  def test_banner_without_lanes(self):
    instruction = messaging.new_message('navInstruction').navInstruction
    set_banner(instruction, [{'distanceAlongGeometry': 100, 'primary': {'text': 'Turn right', 'type': 'turn', 'modifier': 'right'}}], 50)
    assert len(instruction.lanes) == 0


def degrees(meters):
  return meters / METERS_PER_DEGREE


def gps_fix(x, y, bearing, speed, accuracy=0.):
  """A GPS fix at planar meters (x east, y north) near the equator, as navd reads it, with the course accuracy the receiver
  reports (0: none)."""
  msg = messaging.new_message('gpsLocationExternal', valid=True)
  msg.gpsLocationExternal = {'hasFix': True, 'longitude': float(degrees(x)), 'latitude': float(degrees(y)), 'bearingDeg': float(bearing % 360),
                             'speed': float(speed), 'bearingAccuracyDeg': float(accuracy)}
  return messaging.log_from_bytes(msg.to_bytes()).gpsLocationExternal


class TestGpsLoss(OpenpilotTestCase):
  def test_instruction_is_invalid_without_a_fresh_fix(self):
    # without a fix, navd's distance to the maneuver is frozen, which must never look like a valid one to steer on
    sm, pm = MagicMock(), MagicMock()
    sm.valid = {'gpsLocation': True, 'gpsLocationExternal': False}
    sm.logMonoTime = {'gpsLocation': 1, 'gpsLocationExternal': 0}
    sm.recv_time = {'gpsLocation': time.monotonic(), 'gpsLocationExternal': 0.}
    sm.frame = 1
    sm.__getitem__.return_value = gps_fix(0., 0., 90., 0.)
    engine = route_engine({'NavDestination': {'latitude': 0., 'longitude': 0.002}}, sm, pm)
    response = MagicMock(status_code=200)
    response.json.return_value = {'routes': [{'legs': [{'steps': [
      {'geometry': {'coordinates': [[0, 0], [0.001, 0]]}, 'distance': 111.195, 'duration': 10,
       'bannerInstructions': [{'distanceAlongGeometry': 100, 'primary': {'text': 'Turn right', 'type': 'turn', 'modifier': 'right'}}]},
      {'geometry': {'coordinates': [[0.001, 0], [0.002, 0]]}, 'distance': 111.195, 'duration': 10, 'bannerInstructions': []}]}]}]}
    last = lambda: next(call.args[1] for call in reversed(pm.send.call_args_list) if call.args[0] == 'navInstruction')  # noqa: E731
    with patch('requests.get', return_value=response):
      engine.update()
      assert last().valid
      sm.recv_time['gpsLocation'] = time.monotonic() - 5  # the fix went stale: a tunnel, or a garage
      engine.update()
      assert not last().valid
      sm.recv_time['gpsLocation'] = time.monotonic()
      engine.update()
      assert last().valid


def step(points, banner_type, modifier):
  """A Mapbox step along planar points, its banner the maneuver at its end."""
  coordinates = [[degrees(x), degrees(y)] for x, y in points]
  distance = sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:], strict=True))
  banners = [{'distanceAlongGeometry': 200, 'primary': {'text': 'Main St', 'type': banner_type, 'modifier': modifier}}] if banner_type else []
  return {'geometry': {'coordinates': coordinates}, 'distance': distance, 'duration': 10, 'bannerInstructions': banners}


def mapbox_route(*steps):
  """Steps of (planar points, banner type, modifier), then Mapbox's empty last step at the destination."""
  end = steps[-1][0][-1]
  return [step(*s) for s in steps] + [step([end, end], None, None)]


# 300 m north to a right turn, 200 m east to a left turn, then 200 m north to the destination
ROUTE = mapbox_route(([(0, 0), (0, 300)], 'turn', 'right'), ([(0, 300), (200, 300)], 'turn', 'left'), ([(200, 300), (200, 500)], 'arrive', ''))


def load(engine, route):
  engine.route = route
  engine.route_geometry = [np.array(s['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for s in route]
  engine.step_idx = 0
  end = route[-1]['geometry']['coordinates'][-1]
  engine.nav_destination = (end[0], end[1])


class TestManeuverProgress(OpenpilotTestCase):
  def setUp(self):
    self.pm = MagicMock()
    self.engine = route_engine({'NavDestination': {'latitude': degrees(500), 'longitude': degrees(200)}}, pm=self.pm)
    load(self.engine, ROUTE)

  def at(self, x, y, bearing=0., speed=10.):
    self.engine.last_position, self.engine.last_bearing, self.engine.last_speed = (degrees(x), degrees(y)), bearing, speed
    self.engine.send_instruction()
    return self.pm.send.call_args.args[1]

  def test_taken_turn_moves_on_once_along_the_new_road(self):
    for y in (100, 250, 295):
      assert self.at(0, y).valid and self.engine.step_idx == 0
    assert self.at(5, 300, bearing=60).valid and self.engine.step_idx == 0  # turning
    # the update that moves on reports the next maneuver, the left turn, where it used to repeat the passed one
    msg = self.at(12, 300, bearing=90)
    assert msg.valid and self.engine.step_idx == 1 and str(msg.navInstruction.maneuverModifier) == 'left'
    msg = self.at(15, 300, bearing=90)
    assert math.isclose(msg.navInstruction.maneuverDistance, 185., abs_tol=0.5) and str(msg.navInstruction.maneuverModifier) == 'left'
    assert not self.engine.should_recompute()

  def test_step_waits_for_the_car_to_move(self):
    # Stopped just past a maneuver the GPS bearing is noise: the step waits, and it is not missed
    self.at(0, 295)
    assert self.at(12, 300, bearing=200, speed=1.).valid and self.engine.step_idx == 0
    assert self.at(16, 300, bearing=90).valid and self.engine.step_idx == 1
    # past a maneuver straight on, a car stopped beyond the old step's end is on the next step
    load(self.engine, mapbox_route(([(0, 0), (0, 300)], 'new name', 'straight'), ([(0, 300), (0, 600)], 'turn', 'right'),
                                   ([(0, 600), (200, 600)], 'arrive', '')))
    self.engine.reset_progress()
    self.at(0, 295)
    msg = self.at(0, 312, bearing=200, speed=1.)
    assert msg.valid and self.engine.step_idx == 0 and not self.engine.missed_maneuver
    assert self.at(0, 316).valid and self.engine.step_idx == 1

  def test_straight_past_a_turn_is_missed(self):
    assert self.at(0, 305).valid and self.engine.step_idx == 0
    msg = self.at(0, 312)  # it went past the right turn without taking it
    assert not msg.valid and self.engine.step_idx == 0  # never the left turn after it, which isn't on this road
    assert self.engine.should_recompute()

  def test_off_route_is_invalid(self):
    assert not self.at(30, 150).valid

  def test_maneuver_keeps_its_distance_to_the_destination(self):
    # how controls tell a GPS jump from the next maneuver
    for y in (0, 120, 299):
      msg = self.at(0, y).navInstruction
      assert math.isclose(msg.distanceRemaining - msg.maneuverDistance, 400., abs_tol=0.01)

  def test_progress_ignores_the_offset_from_the_road(self):
    # Parked in a lot beside the start of the route, which Mapbox snaps to the road, the car has made no progress along it;
    # behind the start it has made less than none
    for (x, y), distance in (((20, 0), 300.), ((20, 60), 240.), ((-12, 150), 150.), ((0, -10), 310.)):
      msg = self.at(x, y).navInstruction
      assert math.isclose(msg.maneuverDistance, distance, abs_tol=0.01)
      assert math.isclose(msg.distanceRemaining - msg.maneuverDistance, 400., abs_tol=0.01)

  def test_arrival(self):
    self.engine.step_idx = 2
    assert self.at(200, 505).valid and self.engine.step_idx == 2
    self.at(200, 512)
    assert self.engine.step_idx == 3
    self.at(200, 540)  # driving away from the destination clears it
    assert self.engine.route is None
    self.engine.params.remove.assert_called_with("NavDestination")

  def test_arrival_only_from_the_destination(self):
    # on the last step without having been near the destination, say after a wrong step change, navigation goes on
    self.engine.step_idx = 3
    for y in (560, 600):
      self.at(200, y)
    self.engine.params.remove.assert_not_called()
    assert self.engine.route is not None
    self.at(200, 520)
    self.at(200, 560)
    self.engine.params.remove.assert_called_with("NavDestination")


class GpsSubMaster:
  """navd's GPS inputs, with one real gpsLocationExternal fix at a time, as from the u-blox."""

  def __init__(self):
    self.frame = 0
    self.valid = {'gpsLocation': False, 'gpsLocationExternal': True}
    self.recv_time = {'gpsLocation': 0., 'gpsLocationExternal': 0.}
    self.logMonoTime = {'gpsLocation': 0, 'gpsLocationExternal': 0}
    self.fixes = {'gpsLocation': gps_fix(0., 0., 0., 0.), 'gpsLocationExternal': gps_fix(0., 0., 0., 0.)}

  def update(self, timeout=0):
    self.frame += 1

  def __getitem__(self, service):
    return self.fixes[service]

  def fix(self, x, y, bearing, speed, age=0., accuracy=0.):
    """A new fix, published age seconds ago."""
    now = time.monotonic()
    self.fixes['gpsLocationExternal'] = gps_fix(x, y, bearing, speed, accuracy)
    self.recv_time['gpsLocationExternal'] = now
    self.logMonoTime['gpsLocationExternal'] = int((now - age) * 1e9)


class Path:
  """Planar points the car drives through."""

  def __init__(self, points):
    self.points = np.array(points, dtype=np.float64)
    self.distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
    self.length = float(self.distance[-1])

  def at(self, s):
    return np.array([np.interp(s, self.distance, self.points[:, 0]), np.interp(s, self.distance, self.points[:, 1])])

  def bearing(self, s):
    """The car's heading, over the 2 m around s: it turns through a corner."""
    (x0, y0), (x1, y1) = self.at(s - 1.), self.at(s + 1.)
    return math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360


def arc(center, radius, start, end, step=5.):
  """Points on a circle from angle start to end, in degrees counterclockwise from east."""
  angles = np.radians(np.linspace(start, end, int(abs(end - start) / step) + 1))
  return [(center[0] + radius * math.cos(a), center[1] + radius * math.sin(a)) for a in angles]


class Drive:
  """The real navd loop, once a second, for a car driving through planar points at a constant speed."""

  def __init__(self, route, start, destination=None):
    """destination: planar meters of the place set on the phone, by default the route's end."""
    self.sm, self.pm = GpsSubMaster(), MagicMock()
    end = route[-1]['geometry']['coordinates'][-1]
    if destination is not None:
      end = [degrees(destination[0]), degrees(destination[1])]
    self.values = {'NavDestination': {'longitude': end[0], 'latitude': end[1]}}
    self.engine = route_engine(self.values, self.sm, self.pm)
    self.engine.last_position = (degrees(start[0]), degrees(start[1]))
    response = MagicMock(status_code=200)
    response.json.return_value = {'routes': [{'legs': [{'steps': route}]}]}
    with patch('requests.get', return_value=response):
      self.engine.calculate_route((end[0], end[1]))
    self.engine.recompute_hold = 0

  def drive(self, points, speed, phase=0.):
    """Drives through points, navd updating phase seconds after the start and then every second, until the points end or
    navd asks for a new route. Per update: where the car is, navd's step and its instruction."""
    path, records = Path(points), []
    with patch('requests.get', return_value=MagicMock(status_code=200)) as get:
      s = phase * speed
      while s <= path.length and not get.called:
        x, y = path.at(s)
        self.sm.fix(x, y, path.bearing(s), speed)
        self.pm.reset_mock()
        self.engine.update()
        msg = next(call.args[1] for call in self.pm.send.call_args_list if call.args[0] == 'navInstruction')
        records.append({'s': s, 'x': x, 'y': y, 'step': self.engine.step_idx, 'valid': msg.valid, 'rerouting': get.called,
                        'type': str(msg.navInstruction.maneuverType), 'distance': msg.navInstruction.maneuverDistance})
        s += speed
    return records

  def fixes(self, fixes, accuracy=0.):
    """Feeds fixes of (x, y, bearing, speed), once a second, a stop or a turnaround too, each reporting a course accuracy.
    Per fix: whether the instruction was valid, and how many routes navd asked for so far."""
    records = []
    with patch('requests.get', return_value=MagicMock(status_code=200)) as get:
      for x, y, bearing, speed in fixes:
        self.sm.fix(x, y, bearing, speed, accuracy=accuracy)
        self.pm.reset_mock()
        self.engine.update()
        msg = next(call.args[1] for call in self.pm.send.call_args_list if call.args[0] == 'navInstruction')
        records.append({'x': x, 'y': y, 'step': self.engine.step_idx, 'valid': msg.valid, 'type': str(msg.navInstruction.maneuverType),
                        'requests': get.call_count})
    return records


PHASES = (0., 0.25, 0.5, 0.75)


class TestStepAdvance(OpenpilotTestCase):
  """A U-turn's, a sharp turn's or a roundabout turnaround's next step runs right beside the road the car comes on: navd
  takes such a maneuver only once the car reached it and heads along the route after it."""

  def test_uturn(self):
    for median in (0., 12.):  # on one road, and across a median
      route = mapbox_route(([(0, 0), (0, 300)], 'continue', 'uturn'), ([(0, 300), (-median, 300), (-median, 0)], 'turn', 'right'),
                           ([(-median, 0), (-median - 200, 0)], 'arrive', 'right'))
      lane = 1.8  # north in the right lane, through the median opening, south in the right lane
      points = [(lane, 20.)] + arc((-median / 2, 296.), median / 2 + lane, 0., 180.) + [(-median - lane, 20.)]
      for phase in PHASES:
        records = Drive(route, points[0]).drive(points, 6., phase)
        assert all(r['valid'] for r in records)
        assert all(r['step'] == 0 for r in records if r['y'] < 296 and r['x'] > 0)  # the whole way to the U-turn
        back = [r for r in records if r['y'] < 280 and r['x'] < -median]
        assert back and all(r['step'] == 1 and r['type'] == 'turn' for r in back)  # back along the road, heading the other way
        assert all(abs(r['distance'] - r['y']) < 0.5 for r in back)  # to the right turn at y = 0

  def test_uturn_before_the_destination_across_the_median(self):
    # Creeping past the destination on the near side of a narrow median, the car has not arrived: it must U-turn first
    route = mapbox_route(([(0, 0), (0, 500)], 'continue', 'uturn'), ([(0, 500), (-6, 500), (-6, 300)], 'arrive', 'right'))
    points = [(1.8, 240.), (1.8, 450.)]
    for phase in PHASES:
      drive = Drive(route, points[0])
      records = drive.drive(points, 2.5, phase)
      assert all(r['step'] == 0 for r in records)
      drive.engine.params.remove.assert_not_called()

  def test_sharp_turns(self):
    lane = 1.8
    for theta in (30., 45., 55.):  # the new road leaves the corner theta from the road the car comes on
      a = math.radians(theta)
      direction, right = np.array([math.sin(a), -math.cos(a)]), np.array([-math.cos(a), -math.sin(a)])
      corner = np.array([0., 300.])
      elm = corner + 150. * direction
      route = mapbox_route(([(0, 0), tuple(corner)], 'turn', 'sharp right'), ([tuple(corner), tuple(elm)], 'turn', 'right'),
                           ([tuple(elm), tuple(elm + 150. * right)], 'arrive', 'right'))
      # the right lane up to where it meets the new road's right lane, then along it
      along = lane * (1 + math.cos(a)) / math.sin(a)
      turn_in = corner + along * direction + lane * right
      points = [(lane, 100.), tuple(turn_in), tuple(corner + 140. * direction + lane * right)]
      corner_s = turn_in[1] - 100.  # m the car drives to the corner
      for phase in PHASES:
        records = Drive(route, points[0]).drive(points, 5., phase)
        assert all(r['valid'] for r in records)
        assert all(r['step'] == 0 for r in records if r['s'] <= corner_s)  # up to the corner
        on_elm = [r for r in records if r['s'] > corner_s + 15.]
        assert on_elm and all(r['step'] == 1 for r in on_elm)
        for r in on_elm:  # not short by twice the approach, as when it moved on before the corner
          assert abs(r['distance'] - (150. - np.dot(np.array([r['x'], r['y']]) - corner, direction))) < 0.5

  def test_roundabout_turnaround(self):
    # In a right-hand traffic roundabout the last exit leads back beside the approach, 5 m from it
    entry, exit_x = (0., 285.), -5.13
    circle = arc((0., 300.), 15., -90., 250.)
    route = mapbox_route(([(0, 0), entry], 'roundabout', 'uturn'), (circle + [(exit_x, 100.)], 'turn', 'right'),
                         ([(exit_x, 100.), (exit_x - 200, 100.)], 'arrive', 'right'))
    points = [(1.8, 150.), (1.8, 283.)] + circle + [(exit_x - 1.8, 280.), (exit_x - 1.8, 150.)]
    for phase in PHASES:
      records = Drive(route, points[0]).drive(points, 5., phase)
      assert all(r['valid'] for r in records)
      assert all(r['step'] == 0 for r in records if r['y'] < 283 and r['x'] > 0)
      in_roundabout = [r for r in records if r['y'] > 283 or r['x'] > 2]
      assert in_roundabout[-1]['step'] == 1  # it moved on while going around
      leaving = [r for r in records if r['y'] < 270 and r['x'] < exit_x]
      assert leaving and all(r['step'] == 1 and abs(r['distance'] - (r['y'] - 100.)) < 0.5 for r in leaving)


class TestSplits(OpenpilotTestCase):
  """Past an exit the freeway stays right beside the ramp: navd moves onto the ramp only once the car is clearly on it."""

  def setUp(self):
    a = math.radians(6.)
    self.gore, self.ramp = np.array([0., 1000.]), np.array([math.sin(a), math.cos(a)])
    ramp_end = self.gore + 300. * self.ramp
    self.route = mapbox_route(([(0, 0), tuple(self.gore)], 'off ramp', 'slight right'), ([tuple(self.gore), tuple(ramp_end)], 'end of road', 'right'),
                              ([tuple(ramp_end), tuple(ramp_end + (200., 0.))], 'arrive', 'right'))

  def test_exit_taken(self):
    points = [(1.8, 600.), (1.8, 990.), tuple(self.gore + 20. * self.ramp), tuple(self.gore + 280. * self.ramp)]
    for phase in PHASES:
      records = Drive(self.route, points[0]).drive(points, 25., phase)
      assert all(r['valid'] and not r['rerouting'] for r in records)
      on_ramp = [r for r in records if r['type'] == 'end of road']
      assert on_ramp, "the ramp's maneuver is published once the car is on the ramp"
      assert on_ramp[0]['y'] > self.gore[1] + 50.  # some way past the gore, where the ramp is clearly apart from the freeway
      assert all(r['type'] == 'off ramp' for r in records if r['y'] < on_ramp[0]['y'])  # until then the passed exit, behind the car
      for r in on_ramp:
        assert abs(r['distance'] - (300. - np.dot(np.array([r['x'], r['y']]) - self.gore, self.ramp))) < 0.5

  def test_exit_not_taken(self):
    points = [(1.8, 600.), (1.8, 1600.)]
    for phase in PHASES:
      records = Drive(self.route, points[0]).drive(points, 30., phase)
      assert all(r['type'] != 'end of road' for r in records)  # the ramp's maneuver is never published
      missed = next(i for i, r in enumerate(records) if not r['valid'])
      assert records[missed]['y'] > self.gore[1] + 100.  # once the car is clearly on the freeway, not the ramp
      assert all(r['valid'] for r in records[:missed])
      assert records[-1]['rerouting'] and records[-1]['y'] - records[missed]['y'] <= 30.  # at the next update

  def test_fork_straight_on(self):
    # Just past a 25 degree exit the ramp forks, and the route keeps straight on along it: there the old road going on is the
    # route itself, so the car on it is never missed, and the ramp's own maneuver comes before its end
    a = math.radians(25.)
    gore, ramp = np.array([0., 1000.]), np.array([math.sin(a), math.cos(a)])
    fork, ramp_end = gore + 8. * ramp, gore + 408. * ramp
    route = mapbox_route(([(0, 0), tuple(gore)], 'off ramp', 'slight right'), ([tuple(gore), tuple(fork)], 'fork', 'slight left'),
                         ([tuple(fork), tuple(ramp_end)], 'end of road', 'right'), ([tuple(ramp_end), tuple(ramp_end + (300., 0.))], 'arrive', 'right'))
    points = [(0., 600.), tuple(gore), tuple(ramp_end - 5. * ramp)]
    for phase in PHASES:
      records = Drive(route, points[0]).drive(points, 30., phase)
      assert all(r['valid'] and not r['rerouting'] for r in records), phase
      assert [r for r in records if r['type'] == 'end of road'], phase

  def test_gps_outlier_past_an_exit_not_taken(self):
    # One fix off toward the ramp, from multipath say, while the car stays on the freeway. The straight line navd checks
    # between two fixes then runs through the ramp's side of the road, where the car never was: the ramp's turn must
    # never come to the car on the freeway
    for jump in (20., 30., 60.):
      for past in (70., 80., 90., 100., 110.):
        for phase in (0., 0.5):
          outlier = self.gore[1] + past
          fixes = [(1.8 + (jump if abs(y - outlier) < 15. else 0.), y, 0., 30.) for y in np.arange(600. + 30. * phase, 1400., 30.)]
          records = Drive(self.route, (1.8, 600.)).fixes(fixes)  # the receiver's course stays the car's, from its motion
          assert not [r for r in records if r['valid'] and r['type'] == 'end of road'], (jump, past, phase)
          assert records[-1]['requests'] > 0  # missed, and rerouted


class TestServiceRoad(OpenpilotTestCase):
  """The route turns onto a short connector to a service road that runs beside the main road: a car staying on the main
  road is as close to the route after the turn as one that took it, so navd moves on only once the car's course turned
  toward the service road and the car is clearly on it."""

  def route(self, side, turn=600.):
    """The service road side m beside the main road, to its own right turn at y = turn."""
    return mapbox_route(([(0, 0), (0, 300)], 'turn', 'right'), ([(0, 300), (side, 300), (side, turn)], 'turn', 'right'),
                        ([(side, turn), (side + 200, turn)], 'arrive', 'right'))

  def test_staying_on_the_main_road(self):
    for side in (12., 15.):  # m between the roads' center lines
      for speed in (8., 12., 16.):
        for phase in PHASES:
          records = Drive(self.route(side), (1.8, 100.)).drive([(1.8, 100.), (1.8, 700.)], speed, phase)
          assert all(r['step'] == 0 for r in records[:-1]), (side, speed, phase)  # never onto the service road's turn
          assert records[-1]['rerouting'] and records[-1]['y'] < 350.  # missed, and rerouted

  def test_taking_the_service_road(self):
    for side in (12., 15.):
      points = [(1.8, 100.), (1.8, 298.2), (side + 1.8, 298.2), (side + 1.8, 590.)]
      for phase in PHASES:
        records = Drive(self.route(side), points[0]).drive(points, 6., phase)
        assert all(r['valid'] and not r['rerouting'] for r in records)
        along = [r for r in records if r['y'] > 320.]
        assert along and all(r['step'] == 1 and abs(r['distance'] - (600. - r['y'])) < 0.5 for r in along), (side, phase)

  def test_taking_an_angled_connector(self):
    # a connector at 60 degrees, taken at different speeds: the course turns toward the service road, which is what counts
    side = 15.
    route = mapbox_route(([(0, 0), (0, 300)], 'turn', 'right'), ([(0, 300), (side, 300 + side / math.sqrt(3)), (side, 600)], 'turn', 'right'),
                         ([(side, 600), (side + 200, 600)], 'arrive', 'right'))
    points = [(1.8, 100.), (1.8, 297.), (side + 1.8, 297. + side / math.sqrt(3)), (side + 1.8, 590.)]
    for speed in (4., 6., 8.):
      for phase in PHASES:
        records = Drive(route, points[0]).drive(points, speed, phase)
        assert all(r['valid'] and not r['rerouting'] for r in records), (speed, phase)
        assert [r for r in records if r['y'] > 340.] and all(r['step'] == 1 for r in records if r['y'] > 340.), (speed, phase)

  def test_offset_intersection(self):
    # The road goes on 12 m to the right across an intersection: a right turn, a short step, a left turn. The car drives it
    # as one smooth S over 40 m, its course never more than 24 degrees off, which is enough to have turned toward it
    route = mapbox_route(([(0, -400), (0, 0)], 'turn', 'right'), ([(0, 0), (12, 0)], 'turn', 'left'), ([(12, 0), (12, 400)], 'turn', 'right'),
                         ([(12, 400), (212, 400)], 'arrive', 'right'))
    s_curve = [(12. * (3 * u * u - 2 * u ** 3), -15. + 40. * u) for u in np.linspace(0., 1., 21)]
    points = [(0., -200.)] + s_curve + [(12., 300.)]
    for speed in (8., 12.):
      for phase in PHASES:
        records = Drive(route, points[0]).drive(points, speed, phase)
        assert all(r['valid'] and not r['rerouting'] for r in records), (speed, phase)
        assert all(r['step'] == 2 for r in records if r['y'] > 60.), (speed, phase)

  def test_staying_in_the_curb_lane(self):
    # In the main road's curb lane the car is no farther from a service road 12-18 m beside than from its own road's line, so
    # no distance tells that it stayed: its course never turned toward the connector, so it missed it. Also where the service
    # road runs on for 800 m, past the 300 m of the main road's straight continuation that are compared.
    for side, lane in ((12., 3.6), (12., 5.4), (15., 5.4), (15., 9.), (18., 9.)):
      for turn in (600., 1100.):
        for speed in (8., 12., 16.):
          for phase in PHASES:
            records = Drive(self.route(side, turn), (lane, 100.)).drive([(lane, 100.), (lane, turn + 100.)], speed, phase)
            assert all(r['step'] == 0 for r in records[:-1]), (side, lane, turn, speed, phase)  # never onto the service road's turn
            assert records[-1]['rerouting'] and records[-1]['y'] <= 340. + 2. * speed, (side, lane, turn, speed, phase)

  def test_gps_error_toward_the_service_road(self):
    # A GPS error of 6-8 m toward the service road for seconds, from multipath say, while the car stays in the curb lane: the
    # fixes are on the service road, but the course never turned, so its turn never comes to the car on the main road
    for error in (6.5, 8.):
      for phase in (0., 0.5):
        fixes = [(5.4 + (error if 320. <= y <= 400. else 0.), y, 0., 12.) for y in np.arange(100. + 12. * phase, 700., 12.)]
        records = Drive(self.route(12.), (5.4, 100.)).fixes(fixes)
        assert not [r for r in records if r['step'] == 1], (error, phase)
        assert records[-1]['requests'] > 0

  def test_road_beside_past_the_old_roads_continuation(self):
    # A slight turn onto a road 8 m beside that runs on for 800 m: past the 300 m of the old road's straight continuation
    # that are compared, the distance to it grows on either road, which must not move a car still on the old road onto
    # the new one
    route = mapbox_route(([(0, 0), (0, 300)], 'turn', 'slight right'), ([(0, 300), (8., 300), (8., 1100)], 'turn', 'right'),
                         ([(8., 1100), (208., 1100)], 'arrive', 'right'))
    for phase in PHASES:
      records = Drive(route, (1.8, 100.)).drive([(1.8, 100.), (1.8, 1200.)], 12., phase)
      assert all(r['step'] == 0 for r in records[:-1]), phase
      assert records[-1]['rerouting'] and records[-1]['y'] < 630., phase


class TestShortSteps(OpenpilotTestCase):
  def test_forks_at_highway_speed(self):
    # Between two 1 Hz fixes at highway speed the car drives past a short step between two forks: no miss in any phase
    for length, speed in ((4., 25.), (8., 25.), (15., 30.)):
      a, b = math.radians(10.), math.radians(5.)
      fork2 = (length * math.sin(a), length * math.cos(a))
      turn = (fork2[0] + 1500. * math.sin(b), fork2[1] + 1500. * math.cos(b))
      route = mapbox_route(([(0, -2000), (0, 0)], 'fork', 'slight right'), ([(0, 0), fork2], 'fork', 'slight left'),
                           ([fork2, turn], 'end of road', 'right'), ([turn, (turn[0] + 300, turn[1])], 'arrive', 'right'))
      points = [(0., -400.), (0., 0.), fork2, (fork2[0] + 400. * math.sin(b), fork2[1] + 400. * math.cos(b))]
      for phase in np.arange(0., 1., 0.1):
        records = Drive(route, points[0]).drive(points, speed, phase)
        assert all(r['valid'] and not r['rerouting'] for r in records), (length, speed, phase)
        assert records[-1]['step'] == 2 and records[-1]['type'] == 'end of road'


class TestFixAge(OpenpilotTestCase):
  def test_late_fix_moves_the_distance_ahead(self):
    # navd reads a fix up to about a second late; the car drove on meanwhile
    for age, ahead in ((0., 0.), (0.8, 16.), (5., 20. * MAX_FIX_AGE), (-1., 0.)):
      drive = Drive(ROUTE, (0., 100.))
      drive.sm.fix(0., 100., 0., 20., age=age)
      with patch('requests.get') as get:
        drive.engine.update()
      get.assert_not_called()
      instruction = next(call.args[1] for call in drive.pm.send.call_args_list if call.args[0] == 'navInstruction').navInstruction
      assert math.isclose(instruction.maneuverDistance, 200. - ahead, abs_tol=0.05), age
      assert math.isclose(instruction.distanceRemaining - instruction.maneuverDistance, 400., abs_tol=0.01)


class TestWrongHeading(OpenpilotTestCase):
  def run_fixes(self, fixes):
    drive = Drive(ROUTE, fixes[0][:2])
    results = []
    with patch('requests.get', return_value=MagicMock(status_code=200)) as get:
      for x, y, bearing, speed in fixes:
        drive.sm.fix(x, y, bearing, speed)
        drive.engine.update()
        msg = next(call.args[1] for call in reversed(drive.pm.send.call_args_list) if call.args[0] == 'navInstruction')
        results.append((msg.valid, get.called))
    return results

  def test_heading_away_from_a_maneuver_far_ahead(self):
    # on the route's road, but heading back the way the route came: it left the route
    results = self.run_fixes([(0., 200. - 10. * i, 180., 10.) for i in range(REROUTE_COUNTER_MIN + 4)])
    assert results[0] == (True, False)  # one fix may be a bad bearing
    assert all(not valid for valid, _ in results[1:])
    # and it counts toward the reroute, like being off the route: from the second fix on, a reroute once the count passes the minimum
    assert next(i for i, (_, rerouting) in enumerate(results) if rerouting) == 1 + REROUTE_COUNTER_MIN + 1

  def test_turning_in_early_near_the_maneuver(self):
    assert all(valid for valid, _ in self.run_fixes([(0., 280. + 2. * i, 60., 5.) for i in range(4)]))

  def test_slow(self):
    assert all(valid for valid, _ in self.run_fixes([(0., 200. - 2. * i, 180., 2.) for i in range(4)]))


class TestLeavingTheRoute(OpenpilotTestCase):
  """A road that branches off at a shallow angle stays within 25 m and 45 degrees of the route for a long way: a car that
  takes it must not get the route's next turn on it, a road the route doesn't take."""
  TURN = 400.  # m north to the route's right turn

  def route(self):
    return mapbox_route(([(0, 0), (0, self.TURN)], 'turn', 'right'), ([(0, self.TURN), (300, self.TURN)], 'turn', 'left'),
                        ([(300, self.TURN), (300, self.TURN + 300)], 'arrive', ''))

  def test_shallow_branch(self):
    for angle in (15., 25., 35.):
      for before in (50., 70., 90.):  # m before the route's turn the branch leaves
        a, branch = math.radians(angle), self.TURN - before
        points = [(1.8, 150.), (1.8, branch), (1.8 + 150. * math.sin(a), branch + 150. * math.cos(a))]
        for phase in PHASES:
          records = Drive(self.route(), points[0]).drive(points, 11., phase)
          assert all(r['valid'] for r in records if r['y'] <= branch)
          on_branch = [r for r in records if r['y'] > branch]
          gone = next(i for i, r in enumerate(on_branch) if not r['valid'])
          assert on_branch[gone]['s'] - (branch - 150.) < 35., (angle, before, phase)  # m into the branch
          assert not any(r['valid'] for r in on_branch[gone:])
          assert records[-1]['rerouting']  # and the route is calculated again

  def test_branch_where_the_route_goes_straight_on(self):
    # the route goes on straight through the junction, as a new name: navd moves onto that step as the car takes the branch
    a = math.radians(15.)
    route = mapbox_route(([(0, 0), (0, 300)], 'new name', 'straight'), ([(0, 300), (0, 380)], 'turn', 'right'),
                         ([(0, 380), (200, 380)], 'arrive', 'right'))
    points = [(1.8, 100.), (1.8, 300.), (1.8 + 150. * math.sin(a), 300. + 150. * math.cos(a))]
    for phase in PHASES:
      records = Drive(route, points[0]).drive(points, 11., phase)
      assert not [r for r in records if r['valid'] and r['type'] == 'turn' and r['distance'] < 40.], phase
      assert records[-1]['rerouting']

  def test_frontage_road(self):
    # a branch that turns to run beside the route, 15 m from it
    a, branch, side = math.radians(25.), self.TURN - 120., 15.
    run = (side - 1.8) / math.sin(a)
    points = [(1.8, 150.), (1.8, branch), (side, branch + run * math.cos(a)), (side, self.TURN + 50.)]
    for phase in PHASES:
      records = Drive(self.route(), points[0]).drive(points, 11., phase)
      assert not [r for r in records if r['valid'] and r['x'] > side - 1.], phase
      assert records[-1]['rerouting']

  def test_lane_change(self):
    # a lane change heads off the step's direction and moves away from it only for a moment: the route and its turn stay
    for speed, length in ((6., 12.), (8., 15.), (11., 35.)):
      for before in (100., 150.):
        start = self.TURN - before
        points = [(1.8, 150.), (1.8, start), (5.3, start + length), (5.3, self.TURN - 20.)]
        for phase in PHASES:
          records = Drive(self.route(), points[0]).drive(points, speed, phase)
          assert not any(r['rerouting'] for r in records)
          after = [r for r in records if r['y'] > start + length + 2. * speed]  # two fixes after it
          assert after and all(r['valid'] for r in after), (speed, length, before, phase)

  def test_median_end(self):
    # Where a raised median ends, the map's one-way line along each carriageway rejoins the road's centerline over a few tens
    # of meters: under a car in the curb lane the step's line heads off the car's course and moves away from it, while the
    # car stays on the route. The lanes may shift after the median's end, before it, or not at all.
    for lanes in (2, 3):
      # with a 3 m median, m right of the road's centerline: the carriageway's line, and the curb lane while beside the median
      line, curb = 1.5 + 1.8 * lanes, 3.6 * lanes - 0.3
      for rejoin in (15., 30.):
        turn = 300. + rejoin + 70.
        route = mapbox_route(([(line, 0), (line, 300), (0, 300 + rejoin), (0, turn)], 'turn', 'right'),
                             ([(0, turn), (300, turn)], 'turn', 'left'), ([(300, turn), (300, turn + 300)], 'arrive', ''))
        for lane_shift in ([(curb, 300.), (curb - 1.5, 300. + min(3. * rejoin, 48.))], [], [(curb, 270.), (curb - 1.5, 300.)]):
          points = [(curb, 100.)] + lane_shift + [(curb - 1.5 if lane_shift else curb, turn - 10.)]
          for speed in (8., 11.):
            for phase in PHASES:
              records = Drive(route, points[0]).drive(points, speed, phase)
              assert all(r['valid'] and not r['rerouting'] for r in records), (lanes, rejoin, lane_shift, speed, phase)

  def test_boulevard(self):
    # A boulevard the map draws as one-way carriageway lines along the 100 m around each intersection, where a raised median
    # is, and as one centerline between them: the car keeps its curb lane past every one without the route being recalculated
    line, curb = 6.9, 9.  # 3+3 lanes, a 3 m median
    centerline, lanes = [(0., 0.)], [(curb, 0.)]
    for y in (100., 350., 600.):
      centerline += [(0., y - 20.), (line, y), (line, y + 100.), (0., y + 120.)]
      lanes += [(curb, y - 45.), (curb + 1.5, y), (curb + 1.5, y + 100.), (curb, y + 145.)]
    route = mapbox_route((centerline + [(0., 900.)], 'turn', 'right'), ([(0., 900.), (300., 900.)], 'turn', 'left'),
                         ([(300., 900.), (300., 1200.)], 'arrive', ''))
    for speed in (6., 9.):
      for phase in (0., 0.5):
        records = Drive(route, lanes[0]).drive(lanes + [(curb, 890.)], speed, phase)
        assert all(r['valid'] and not r['rerouting'] for r in records), (speed, phase)

  def test_branch_where_the_route_bends(self):
    # the route's road bends left at a map vertex while a branch goes on straight there: the car on it leaves the route
    for theta in (15., 25., 40.):
      a = math.radians(theta)
      turn = (-70. * math.sin(a), 300. + 70. * math.cos(a))  # the route's right turn, 70 m on along the bend
      left = (turn[0] + 300. * math.cos(a), turn[1] + 300. * math.sin(a))
      route = mapbox_route(([(0, 0), (0, 300), turn], 'turn', 'right'), ([turn, left], 'turn', 'left'),
                           ([left, (left[0] - 300. * math.sin(a), left[1] + 300. * math.cos(a))], 'arrive', ''))
      for phase in PHASES:
        records = Drive(route, (1.8, 150.)).drive([(1.8, 150.), (1.8, 450.)], 11., phase)
        past = [r for r in records if r['y'] > 300.]
        gone = next(i for i, r in enumerate(past) if not r['valid'])
        assert past[gone]['y'] - 300. < 35. and not any(r['valid'] for r in past[gone:]), (theta, phase)
        assert records[-1]['rerouting']

  def test_branch_before_the_route_bends_toward_it(self):
    # a branch leaves 40-60 m before the route's road bends right by as much, so it cuts the bend and runs beside the route
    # past it: the car on it leaves the route all the same
    for theta in (15., 30.):
      a = math.radians(theta)
      turn = (100. * math.sin(a), 300. + 100. * math.cos(a))  # the route's right turn, 100 m on along the bend
      left = (turn[0] + 300. * math.cos(a), turn[1] - 300. * math.sin(a))
      route = mapbox_route(([(0, 0), (0, 300), turn], 'turn', 'right'), ([turn, left], 'turn', 'left'),
                           ([left, (left[0] + 300. * math.sin(a), left[1] + 300. * math.cos(a))], 'arrive', ''))
      for before in (40., 60.):
        branch = 300. - before
        points = [(1.8, 150.), (1.8, branch), (1.8 + 300. * math.sin(a), branch + 300. * math.cos(a))]
        for phase in PHASES:
          records = Drive(route, points[0]).drive(points, 11., phase)
          on_branch = [r for r in records if r['y'] > branch]
          gone = next(i for i, r in enumerate(on_branch) if not r['valid'])
          assert on_branch[gone]['s'] - (branch - 150.) < 35. and not any(r['valid'] for r in on_branch[gone:]), (theta, before, phase)
          assert records[-1]['rerouting']

  def test_course_accuracy(self):
    # At a crawl, a fix's position drifts 6 m to the right over 5 s while its course reads 15 degrees right: a car taking a
    # shallow branch, or the GPS error of a receiver that rates its course to 8 degrees at that speed, as in slow traffic.
    # A course rated that poor doesn't take the car off the route; a better one does, and so does one the receiver doesn't
    # rate (0, or 180 from qcomgpsd).
    fixes = [(1.8 + 1.2 * min(max(i - 30, 0), 5), 150. + 3.5 * i, 15. if 30 < i <= 35 else 0., 3.5) for i in range(45)]
    for accuracy, leaves in ((0., True), (2., True), (4.8, True), (8., False), (20., False), (180., True)):
      records = Drive(self.route(), fixes[0][:2]).fixes(fixes, accuracy=accuracy)
      assert any(not r['valid'] for r in records) == leaves, accuracy
      assert (records[-1]['requests'] > 0) == leaves, accuracy
    # once one fix heads off by more than that course's error can, the fixes after it only need to stay off
    fixes[31] = (fixes[31][0], fixes[31][1], 22., 3.5)
    records = Drive(self.route(), fixes[0][:2]).fixes(fixes, accuracy=8.)
    assert any(not r['valid'] for r in records) and records[-1]['requests'] > 0


class TestArrival(OpenpilotTestCase):
  """Navigation ends once the car drives away from the destination, whichever way it leaves: never with a route back."""

  def arrive(self):
    """Drives ROUTE in the right lane to 5 m short of its end, where it stops for 30 s, still on: a drop-off."""
    drive = Drive(ROUTE, (1.8, 100.))
    records = drive.drive([(1.8, 100.), (1.8, 298.2), (201.8, 298.2), (201.8, 495.)], 10.)
    assert all(r['valid'] for r in records) and records[-1]['step'] == 2
    drive.fixes([(201.8, 495., 0., 0.)] * 30)
    assert drive.engine.route is not None and 'NavDestination' in drive.values
    return drive

  def test_leaving(self):
    for way, fixes in (('turning around', [(199., 497., 270., 3.)] + [(198.2, y, 180., 10.) for y in range(490, 280, -10)]),
                       ('through a lot beside it', [(x, 492., 90., 6.) for x in range(205, 330, 8)]),
                       ('on past the end', [(201.8, y, 0., 10.) for y in range(500, 600, 10)])):
      drive = self.arrive()
      records = drive.fixes(fixes)
      assert records[-1]['requests'] == 0, way
      assert 'NavDestination' not in drive.values and drive.engine.route is None, way

  def test_parked_deep_in_a_lot(self):
    # The place is set back 20 m from the road, where Mapbox ends the route. The car parks 40 m from the road for a minute,
    # still on, and leaves by the lot's back exit: off the route all along, but never with a route back to the place.
    route = mapbox_route(([(0, -400), (0, 0)], 'turn', 'right'), ([(0, 0), (90, 0)], 'arrive', 'right'))
    drive = Drive(route, (1.8, -300.), destination=(90., -20.))
    drive.drive([(1.8, -300.), (1.8, -1.8), (86., -1.8)], 10.)
    lot = [(88., -1.8 - d, 180., 4.) for d in range(4, 40, 4)] + [(88., -40., 180., 0.)] * 60
    back_exit = [(88., -40. - d, 180., 6.) for d in range(6, 80, 6)] + [(88. - d, -121.8, 270., 10.) for d in range(10, 200, 10)]
    records = drive.fixes(lot + back_exit)
    assert records[-1]['requests'] == 0
    assert 'NavDestination' not in drive.values and drive.engine.route is None

  def test_new_destination_set_before_leaving(self):
    # set on the phone as the car leaves the old one, before navd routes to it: the old one's end never removes it
    drive = self.arrive()
    new = {'longitude': degrees(-500.), 'latitude': degrees(100.)}
    drive.values['NavDestination'] = new
    drive.engine.last_position = (degrees(198.2), degrees(440.))
    drive.engine.send_instruction()
    assert drive.engine.route is None and drive.values['NavDestination'] == new
    assert drive.fixes([(198.2, 430., 180., 10.)])[0]['requests'] == 1  # and it gets its route


class TestDestinationChange(OpenpilotTestCase):
  def test_new_destination_gets_its_route_at_once(self):
    # Set on the phone right after a route came, at the start of a drive or after a reroute: the old route's turns must not
    # go on until the reroute hold ends
    drive = Drive(ROUTE, (0., 100.))
    drive.engine.recompute_hold = RECOMPUTE_HOLD
    drive.values['NavDestination'] = {'longitude': degrees(-300.), 'latitude': degrees(100.)}
    records = drive.fixes([(0., 110., 0., 10.)])
    assert records[0]['requests'] == 1 and not records[0]['valid']  # here Mapbox found no route
    # a failed request is tried again once the hold ends, not every second
    records = drive.fixes([(0., 120. + 10. * i, 0., 10.) for i in range(RECOMPUTE_HOLD)])
    assert [r['requests'] for r in records] == [0] * (RECOMPUTE_HOLD - 1) + [1]
