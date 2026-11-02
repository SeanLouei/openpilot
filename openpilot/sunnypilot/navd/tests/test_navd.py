"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.navigation.navd import RouteEngine, set_banner


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
