"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
from types import SimpleNamespace

from openpilot.cereal import log, custom
from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LANE_CHANGE_SPEED_MIN
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_INSTRUCTION_MAX_AGE, NavDesireController, active_lane, \
  fresh_nav_instruction, normalize_direction

TurnDirection = custom.ModelDataV2SP.TurnDirection


def car(**kwargs):
  return SimpleNamespace(**({'vEgo': 8., 'leftBlinker': False, 'rightBlinker': False, 'leftBlindspot': False, 'rightBlindspot': False,
                             'steeringPressed': False, 'steeringTorque': 0., 'standstill': False, 'brakePressed': False} | kwargs))


def nav(maneuver_type='turn', modifier='right', distance=40., lanes=()):
  return SimpleNamespace(maneuverType=maneuver_type, maneuverModifier=modifier, maneuverDistance=distance, lanes=list(lanes))


def lane(*directions, active_direction=None):
  return SimpleNamespace(directions=list(directions), active=active_direction is not None, activeDirection=active_direction or 'none')


def nudge(right=True):
  return {'steeringPressed': True, 'steeringTorque': -1. if right else 1.}


class FakeSubMaster(SimpleNamespace):
  def __getitem__(self, service):
    return self.data[service]


# a highway exit on the right: two through lanes, and the exit lane the route uses
RIGHT_EXIT_LANES = (lane('straight'), lane('straight'), lane('slightRight', active_direction='slightRight'))


def controller(turns=True, exits=True):
  c = NavDesireController()
  c.turn_enabled, c.exit_enabled = turns, exits
  return c


def desire_helper(turns=True, exits=True, lane_turns=False, lane_change=AutoLaneChangeMode.NUDGE):
  dh = DesireHelper()
  dh.nav_desire.turn_enabled, dh.nav_desire.exit_enabled = turns, exits
  dh.lane_turn_controller.enabled, dh.lane_turn_controller.lane_turn_value = lane_turns, LANE_CHANGE_SPEED_MIN
  dh.alc.lane_change_set_timer = lane_change
  # the settings above stand in for params, so skip the first param read
  dh.nav_desire.param_read_counter = dh.lane_turn_controller.param_read_counter = dh.alc.param_read_counter = 1
  return dh


def run(dh, carstate, instruction, frames=10):
  for _ in range(frames):
    dh.update(carstate, True, 1.0, False, False, instruction, 0.)
  return dh


class TestHelpers(OpenpilotTestCase):
  @parameterized.expand([
    ('slight left', 'slightLeft'), ('sharp right', 'sharpRight'), ('right', 'right'), ('straight', 'straight'),
    ('uturn', 'uturn'), ('slightRight', 'slightRight'), ('', 'none'),
  ])
  def test_normalize_direction(self, modifier, expected):
    assert normalize_direction(modifier) == expected

  def test_active_lane(self):
    assert active_lane(RIGHT_EXIT_LANES) == ('slightRight', 2)
    assert active_lane([lane('straight'), lane('slightRight', active_direction='none')]) == ('slightRight', 1)  # the only direction
    assert active_lane([lane('straight', 'slightRight', active_direction='none')]) == ('', -1)  # ambiguous without one
    assert active_lane([]) == ('', -1)

  def test_fresh_nav_instruction(self):
    instruction = nav()
    now = time.monotonic()
    sm = FakeSubMaster(seen={'navInstruction': True}, valid={'navInstruction': True}, recv_time={'navInstruction': now - 0.5},
                       data={'navInstruction': instruction})
    found, age = fresh_nav_instruction(sm)
    assert found is instruction and 0.5 <= age < 1.0
    sm.recv_time['navInstruction'] = now - NAV_INSTRUCTION_MAX_AGE - 0.1  # navd stopped publishing
    assert fresh_nav_instruction(sm) == (None, 0.)
    sm.recv_time['navInstruction'], sm.valid['navInstruction'] = now, False  # no route
    assert fresh_nav_instruction(sm) == (None, 0.)

  def test_capnp_lanes(self):
    instruction = log.NavInstruction.new_message(maneuverType='off ramp', maneuverModifier='slight right', maneuverDistance=50.)
    instruction.lanes = [{'directions': ['straight'], 'active': False, 'activeDirection': 'none'},
                         {'directions': ['slightRight'], 'active': True, 'activeDirection': 'slightRight'}]
    assert active_lane(instruction.lanes) == ('slightRight', 1)
    c = controller()
    c.update(car(vEgo=25., **nudge()), True, instruction)
    assert c.keep_desire == log.Desire.keepRight


class TestRouteTurns(OpenpilotTestCase):
  @parameterized.expand([
    (car(rightBlinker=True), TurnDirection.turnRight),
    (car(), TurnDirection.none),
    (car(leftBlinker=True), TurnDirection.none),
    (car(leftBlinker=True, rightBlinker=True), TurnDirection.none),  # hazards
    (car(rightBlinker=True, rightBlindspot=True), TurnDirection.none),
    (car(rightBlinker=True, vEgo=14.5), TurnDirection.none),  # too fast to plan the turn yet
    (car(rightBlinker=True, vEgo=0., standstill=True), TurnDirection.none),  # waits to move off, so the pulse is not spent at a stop
  ])
  def test_turn_requires_matching_blinker(self, carstate, expected):
    c = controller()
    c.update(carstate, True, nav(distance=40.))
    assert c.turn_direction == expected

  @parameterized.expand([
    (2., 34., True), (2., 37., False),  # the preview never shrinks below 35 m
    (8., 47., True), (8., 50., False),  # 6 s ahead at 8 m/s
    (13.5, 80., True), (13.5, 82., False),  # 81 m at 13.5 m/s
    (8., -1., False),  # passed the maneuver
  ])
  def test_turn_waits_until_close(self, v_ego, distance, turns):
    c = controller()
    c.update(car(rightBlinker=True, vEgo=v_ego), True, nav(distance=distance))
    assert c.turn_direction == (TurnDirection.turnRight if turns else TurnDirection.none)

  def test_route_turn_signal(self):
    c = controller()
    c.update(car(rightBlinker=True, vEgo=8.), True, nav(distance=100.))
    assert c.route_turn_signal and c.turn_direction == TurnDirection.none  # signaling for the turn, but it is too far off
    c.update(car(rightBlinker=True, vEgo=8.), True, nav(distance=120.))
    assert not c.route_turn_signal  # beyond 15 m + 12 s
    c.update(car(leftBlinker=True, vEgo=8.), True, nav(distance=100.))
    assert not c.route_turn_signal

  def test_dead_reckoning(self):
    c = controller()
    c.update(car(rightBlinker=True, vEgo=10.), True, nav(distance=70.), nav_age=0.)
    assert c.turn_direction == TurnDirection.none
    c.update(car(rightBlinker=True, vEgo=10.), True, nav(distance=70.), nav_age=1.5)  # 55 m by now
    assert c.turn_direction == TurnDirection.turnRight

  @parameterized.expand([
    ('turn', 'sharp left', TurnDirection.turnLeft),
    ('end of road', 'left', TurnDirection.turnLeft),
    ('on ramp', 'right', TurnDirection.turnRight),
    ('turn', 'slight right', TurnDirection.none),
    ('turn', 'uturn', TurnDirection.none),
    ('roundabout', 'right', TurnDirection.none),
    ('continue', 'right', TurnDirection.none),
    ('off ramp', 'right', TurnDirection.none),
  ])
  def test_maneuvers(self, maneuver_type, modifier, expected):
    c = controller()
    left = expected == TurnDirection.turnLeft
    c.update(car(leftBlinker=left, rightBlinker=not left), True, nav(maneuver_type, modifier))
    assert c.turn_direction == expected

  def test_inactive(self):
    for c, lateral_active, instruction in [(controller(turns=False), True, nav()), (controller(), False, nav()), (controller(), True, None)]:
      c.update(car(rightBlinker=True), lateral_active, instruction)
      assert c.turn_direction == TurnDirection.none and not c.route_turn_signal


class TestRouteExits(OpenpilotTestCase):
  @parameterized.expand([
    (car(vEgo=25., **nudge()), log.Desire.keepRight),
    (car(vEgo=25., rightBlinker=True, **nudge()), log.Desire.keepRight),
    (car(vEgo=25.), log.Desire.none),  # the driver confirms with a nudge
    (car(vEgo=25., **nudge(right=False)), log.Desire.none),
    (car(vEgo=25., leftBlinker=True, **nudge()), log.Desire.none),
    (car(vEgo=25., rightBlindspot=True, **nudge()), log.Desire.none),
  ])
  def test_exit_requires_confirmation(self, carstate, expected):
    c = controller()
    c.update(carstate, True, nav('off ramp', 'slight right', 100., RIGHT_EXIT_LANES))
    assert c.keep_desire == expected

  @parameterized.expand([
    (30., 155., True), (30., 165., False), (15., 85., True), (15., 95., False), (25., -1., False),
  ])
  def test_exit_waits_until_split_is_close(self, v_ego, distance, keeps):
    c = controller()
    c.update(car(vEgo=v_ego, **nudge()), True, nav('off ramp', 'slight right', distance, RIGHT_EXIT_LANES))
    assert c.keep_desire == (log.Desire.keepRight if keeps else log.Desire.none)

  def test_exit_needs_lane_guidance(self):
    c = controller()
    for lanes in [(), (lane('straight', active_direction='straight'), lane('slightRight'))]:  # none, or the route goes straight
      c.update(car(vEgo=25., **nudge()), True, nav('fork', 'slight right', 50., lanes))
      assert c.keep_desire == log.Desire.none

  def test_fork_left(self):
    c = controller()
    lanes = (lane('slightLeft', active_direction='slightLeft'), lane('straight'))
    c.update(car(vEgo=20., **nudge(right=False)), True, nav('fork', 'slight left', 50., lanes))
    assert c.keep_desire == log.Desire.keepLeft

  def test_ambiguous_split_waits_longer(self):
    # two exit lanes, one shared with the road, and only one other lane: wait until 60% of the usual distance
    lanes = (lane('straight'), lane('straight', 'slightRight', active_direction='slightRight'), lane('slightRight', active_direction='slightRight'))
    c = controller()
    c.update(car(vEgo=30., **nudge()), True, nav('off ramp', 'slight right', 120., lanes))
    assert c.keep_desire == log.Desire.none
    c.update(car(vEgo=30., **nudge()), True, nav('off ramp', 'slight right', 90., lanes))
    assert c.keep_desire == log.Desire.keepRight

  def test_edge_exit_lane_next_to_shared_lane(self):
    shared = (lane('straight'), lane('straight', 'slightRight'), lane('slightRight', active_direction='slightRight'))
    wide = (*[lane('straight')] * 4, lane('straight', 'slightRight'), lane('slightRight', active_direction='slightRight'))
    for lanes, expected in [(shared, log.Desire.none), (wide, log.Desire.keepRight)]:
      c = controller()
      c.update(car(vEgo=30., **nudge()), True, nav('off ramp', 'slight right', 50., lanes))
      assert c.keep_desire == expected

  def test_disabled(self):
    c = controller(exits=False)
    c.update(car(vEgo=25., **nudge()), True, nav('off ramp', 'slight right', 100., RIGHT_EXIT_LANES))
    assert c.keep_desire == log.Desire.none


class TestDesireHelperNavigation(OpenpilotTestCase):
  def test_route_turn_desire(self):
    dh = run(desire_helper(), car(rightBlinker=True, vEgo=8.), nav(distance=40.))
    assert dh.desire == log.Desire.turnRight
    assert dh.lane_turn_direction == TurnDirection.turnRight  # shows the turn alert

  def test_route_turn_above_lane_turn_speed(self):
    dh = run(desire_helper(lane_turns=True), car(rightBlinker=True, vEgo=12.), nav(distance=60.))
    assert dh.desire == log.Desire.turnRight

  def test_blinker_for_route_turn_waits_for_intersection(self):
    carstate = car(rightBlinker=True, vEgo=5.)
    assert run(desire_helper(lane_turns=True), carstate, None).desire == log.Desire.turnRight  # a lane turn starts right away
    dh = run(desire_helper(lane_turns=True), carstate, nav(distance=70.))
    assert dh.desire == log.Desire.none  # but a route turn 70 m ahead holds it back
    assert run(dh, carstate, nav(distance=30.)).desire == log.Desire.turnRight

  def test_blinker_for_other_turn_is_not_held(self):
    dh = run(desire_helper(lane_turns=True), car(rightBlinker=True, vEgo=5.), nav(modifier='left', distance=70.))
    assert dh.desire == log.Desire.turnRight

  def test_route_turn_blinker_needs_nudge_to_change_lanes(self):
    carstate = car(rightBlinker=True, vEgo=15.)
    dh = run(desire_helper(lane_change=AutoLaneChangeMode.NUDGELESS), carstate, nav(distance=150.))
    assert dh.lane_change_state == LaneChangeState.preLaneChange and dh.desire == log.Desire.none
    dh = run(dh, car(rightBlinker=True, vEgo=15., **nudge()), nav(distance=150.), frames=1)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting and dh.desire == log.Desire.laneChangeRight

  def test_distant_route_turn_allows_nudgeless_lane_change(self):
    dh = run(desire_helper(lane_change=AutoLaneChangeMode.NUDGELESS), car(rightBlinker=True, vEgo=15.), nav(distance=400.))
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_exit_keep_only_without_lane_change(self):
    exit_ahead = nav('off ramp', 'slight right', 100., RIGHT_EXIT_LANES)
    assert run(desire_helper(), car(vEgo=25., **nudge()), exit_ahead).desire == log.Desire.keepRight
    dh = run(desire_helper(), car(vEgo=25., rightBlinker=True), exit_ahead)  # signaling: a lane change, not a keep
    dh = run(dh, car(vEgo=25., rightBlinker=True, **nudge()), exit_ahead, frames=1)
    assert dh.desire == log.Desire.laneChangeRight

  def test_disabled_leaves_desires_unchanged(self):
    for carstate, expected in [(car(rightBlinker=True, vEgo=5.), log.Desire.turnRight), (car(vEgo=25., **nudge()), log.Desire.none)]:
      instruction = nav(distance=70.) if carstate.rightBlinker else nav('off ramp', 'slight right', 100., RIGHT_EXIT_LANES)
      assert run(desire_helper(turns=False, exits=False, lane_turns=True), carstate, instruction).desire == expected
