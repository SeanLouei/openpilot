"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import os
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.navigation.navd import RouteEngine, set_banner
from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE


def route_engine(values, sm=None, pm=None):
  params = MagicMock()
  params.get.side_effect = values.get
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


class TestGpsLoss(OpenpilotTestCase):
  def test_instruction_is_invalid_without_a_fresh_fix(self):
    # without a fix, navd's distance to the maneuver is frozen, which must never look like a valid one to steer on
    sm, pm = MagicMock(), MagicMock()
    sm.valid = {'gpsLocation': True, 'gpsLocationExternal': False}
    sm.logMonoTime = {'gpsLocation': 1, 'gpsLocationExternal': 0}
    sm.recv_time = {'gpsLocation': time.monotonic(), 'gpsLocationExternal': 0.}
    sm.frame = 1
    sm.__getitem__.return_value = SimpleNamespace(hasFix=True, latitude=0., longitude=0., bearingDeg=90.)
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


def degrees(meters):
  return meters / METERS_PER_DEGREE


def step(start, end, banner_type, modifier):
  coordinates = [[degrees(x), degrees(y)] for x, y in (start, end)]
  banners = [{'distanceAlongGeometry': 200, 'primary': {'text': 'Main St', 'type': banner_type, 'modifier': modifier}}] if banner_type else []
  return {'geometry': {'coordinates': coordinates}, 'distance': math.dist(start, end), 'duration': 10, 'bannerInstructions': banners}


# 300 m north to a right turn, 200 m east to a left turn, then 200 m north to the destination
ROUTE = [step((0, 0), (0, 300), 'turn', 'right'), step((0, 300), (200, 300), 'turn', 'left'),
         step((200, 300), (200, 500), 'arrive', ''), step((200, 500), (200, 500), None, None)]


class TestManeuverProgress(OpenpilotTestCase):
  def setUp(self):
    self.pm = MagicMock()
    self.engine = route_engine({'NavDestination': {'latitude': degrees(500), 'longitude': degrees(200)}}, pm=self.pm)
    self.engine.route = ROUTE
    self.engine.route_geometry = [np.array(s['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for s in ROUTE]
    self.engine.step_idx, self.engine.nav_destination = 0, (degrees(200), degrees(500))

  def at(self, x, y):
    self.engine.last_position = (degrees(x), degrees(y))
    self.engine.send_instruction()
    return self.pm.send.call_args.args[1]

  def test_taken_turn_moves_on_once_along_the_new_road(self):
    for y in (100, 250, 295):
      assert self.at(0, y).valid and self.engine.step_idx == 0
    assert self.at(5, 300).valid and self.engine.step_idx == 0  # turning
    msg = self.at(12, 300)
    assert msg.valid and str(msg.navInstruction.maneuverType) == 'turn' and self.engine.step_idx == 1
    msg = self.at(15, 300)
    assert math.isclose(msg.navInstruction.maneuverDistance, 185., abs_tol=0.5) and str(msg.navInstruction.maneuverModifier) == 'left'
    assert not self.engine.should_recompute()

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

  def test_arrival(self):
    self.engine.step_idx = 2
    assert self.at(200, 505).valid and self.engine.step_idx == 2
    self.at(200, 512)
    assert self.engine.step_idx == 3
    self.at(200, 540)  # driving away from the destination clears it
    assert self.engine.route is None
    self.engine.params.remove.assert_called_with("NavDestination")
