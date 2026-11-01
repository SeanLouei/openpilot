"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import time
from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom
from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LANE_CHANGE_SPEED_MIN
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_INSTRUCTION_MAX_AGE, NavDesireController, NavInputs, active_lane, \
  fresh_nav_instruction, nav_inputs, normalize_direction, outer_lanes

TurnDirection = custom.ModelDataV2SP.TurnDirection


def car(**kwargs):
  return SimpleNamespace(**({'vEgo': 8., 'leftBlinker': False, 'rightBlinker': False, 'leftBlindspot': False, 'rightBlindspot': False,
                             'steeringPressed': False, 'steeringTorque': 0., 'standstill': False, 'brakePressed': False} | kwargs))


def maneuver(maneuver_type='turn', modifier='right', distance=40., lanes=()):
  return SimpleNamespace(maneuverType=maneuver_type, maneuverModifier=modifier, maneuverDistance=distance, lanes=list(lanes))


def nav(maneuver_type='turn', modifier='right', distance=40., lanes=(), age=0., **inputs):
  return NavInputs(maneuver(maneuver_type, modifier, distance, lanes), age, **inputs)


def auto_nav(maneuver_type='turn', modifier='right', distance=30., lanes=(), **inputs):
  """A route turn with openpilot in control of the speed, in the rightmost lane."""
  return nav(maneuver_type, modifier, distance, lanes, **({'long_active': True, 'outer_lanes': (False, True)} | inputs))


def lane(*directions, active_direction=None):
  return SimpleNamespace(directions=list(directions), active=active_direction is not None, activeDirection=active_direction or 'none')


def nudge(right=True):
  return {'steeringPressed': True, 'steeringTorque': -1. if right else 1.}


class FakeSubMaster(SimpleNamespace):
  def __getitem__(self, service):
    return self.data[service]


def model(edge_stds=(0.2, 0.2), line_probs=(0.9, 0.9, 0.9, 0.05), edge_y=(-6., 2.5)):
  return SimpleNamespace(roadEdgeStds=list(edge_stds), laneLineProbs=list(line_probs),
                         roadEdges=[SimpleNamespace(y=[y]) for y in edge_y])


# a highway exit on the right: two through lanes, and the exit lane the route uses
RIGHT_EXIT_LANES = (lane('straight'), lane('straight'), lane('slightRight', active_direction='slightRight'))


def controller(turns=True, exits=True, auto=False):
  c = NavDesireController()
  c.turn_enabled, c.exit_enabled, c.auto_enabled = turns, exits, auto
  return c


def desire_helper(turns=True, exits=True, auto=False, lane_turns=False, lane_change=AutoLaneChangeMode.NUDGE):
  dh = DesireHelper()
  dh.nav_desire.turn_enabled, dh.nav_desire.exit_enabled, dh.nav_desire.auto_enabled = turns, exits, auto
  dh.lane_turn_controller.enabled, dh.lane_turn_controller.lane_turn_value = lane_turns, LANE_CHANGE_SPEED_MIN
  dh.alc.lane_change_set_timer = lane_change
  # the settings above stand in for params, so skip the first param read
  dh.nav_desire.param_read_counter = dh.lane_turn_controller.param_read_counter = dh.alc.param_read_counter = 1
  return dh


def run(dh, carstate, inputs, frames=10):
  for _ in range(frames):
    dh.update(carstate, True, 1.0, False, False, inputs)
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
    instruction = maneuver()
    now = time.monotonic()
    sm = FakeSubMaster(seen={'navInstruction': True}, valid={'navInstruction': True}, recv_time={'navInstruction': now - 0.5},
                       data={'navInstruction': instruction})
    found, age = fresh_nav_instruction(sm)
    assert found is instruction and 0.5 <= age < 1.0
    sm.recv_time['navInstruction'] = now - NAV_INSTRUCTION_MAX_AGE - 0.1  # navd stopped publishing
    assert fresh_nav_instruction(sm) == (None, 0.)
    sm.recv_time['navInstruction'], sm.valid['navInstruction'] = now, False  # no route
    assert fresh_nav_instruction(sm) == (None, 0.)

  def test_outer_lanes(self):
    assert outer_lanes(model()) == (False, True)  # a curb 2.5 m to the right and no lane line beyond the car's
    assert outer_lanes(model(line_probs=(0.9, 0.9, 0.9, 0.6))) == (False, False)  # a lane to the right
    assert outer_lanes(model(edge_y=(-6., 6.))) == (False, False)  # the edge is a lane's width away
    assert outer_lanes(model(edge_stds=(0.2, 0.9))) == (False, False)  # the edge is unsure
    assert outer_lanes(model(edge_y=(-2.5, 2.5), line_probs=(0.05, 0.9, 0.9, 0.05))) == (True, True)  # a one lane road
    assert outer_lanes(model(edge_y=(-6., math.nan))) == (False, False)
    assert outer_lanes(SimpleNamespace(roadEdgeStds=[], laneLineProbs=[], roadEdges=[])) == (False, False)

  def test_nav_inputs(self):
    # real messages, so a renamed or retired field fails here rather than in modeld
    instruction = messaging.new_message('navInstruction').navInstruction
    car_control = messaging.new_message('carControl').carControl
    car_control.longActive = True
    sm = FakeSubMaster(seen={'navInstruction': True}, valid={'navInstruction': True}, recv_time={'navInstruction': time.monotonic()},
                       data={'navInstruction': instruction, 'carControl': car_control,
                             'driverMonitoringState': messaging.new_message('driverMonitoringState').driverMonitoringState})
    model_msg = messaging.new_message('modelV2').modelV2
    assert nav_inputs(sm, model_msg).outer_lanes == (False, False)  # no model output yet
    model_msg.roadEdgeStds, model_msg.laneLineProbs = [0.2, 0.2], [0.9, 0.9, 0.9, 0.05]
    edges = model_msg.init('roadEdges', 2)
    edges[0].y, edges[1].y = [-6.], [2.5]
    inputs = nav_inputs(sm, model_msg)
    assert inputs.instruction is not None and inputs.long_active and inputs.right_hand_traffic
    assert inputs.outer_lanes == (False, True)

  def test_capnp_lanes(self):
    instruction = log.NavInstruction.new_message(maneuverType='off ramp', maneuverModifier='slight right', maneuverDistance=50.)
    instruction.lanes = [{'directions': ['straight'], 'active': False, 'activeDirection': 'none'},
                         {'directions': ['slightRight'], 'active': True, 'activeDirection': 'slightRight'}]
    assert active_lane(instruction.lanes) == ('slightRight', 1)
    c = controller()
    c.update(car(vEgo=25., **nudge()), True, NavInputs(instruction))
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
    c.update(car(rightBlinker=True, vEgo=10.), True, nav(distance=70.))
    assert c.turn_direction == TurnDirection.none
    c.update(car(rightBlinker=True, vEgo=10.), True, nav(distance=70., age=1.5))  # 55 m by now
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
    for c, lateral_active, inputs in [(controller(turns=False), True, nav()), (controller(), False, nav()), (controller(), True, None),
                                      (controller(), True, NavInputs())]:
      c.update(car(rightBlinker=True), lateral_active, inputs)
      assert c.turn_direction == TurnDirection.none and not c.route_turn_signal

  def test_bad_distance(self):
    for distance in (math.nan, math.inf, -math.inf):
      c = controller(auto=True)
      c.update(car(rightBlinker=True, vEgo=5.), True, auto_nav(distance=distance))
      assert c.turn_direction == TurnDirection.none and not c.route_turn_signal


class TestAutoTurns(OpenpilotTestCase):
  def test_auto_turn(self):
    c = controller(turns=False, auto=True)
    c.update(car(vEgo=5.), True, auto_nav())
    assert c.turn_direction == TurnDirection.turnRight

  @parameterized.expand([
    ('auto turns off', {}, {}, False),
    ("gas pressed, or the car's own cruise control", {}, {'long_active': False}, True),
    ('not slowed down yet', {'vEgo': 8.}, {}, True),
    ('not in the rightmost lane', {}, {'outer_lanes': (False, False)}, True),
    ('blind spot', {'rightBlindspot': True}, {}, True),
    ('the other blinker', {'leftBlinker': True}, {}, True),
    ('hazards', {'leftBlinker': True, 'rightBlinker': True}, {}, True),
    ('stopped', {'vEgo': 0., 'standstill': True}, {}, True),
  ])
  def test_auto_turn_blocked(self, _name, car_state, inputs, auto):
    c = controller(turns=False, auto=auto)
    c.update(car(**({'vEgo': 5.} | car_state)), True, auto_nav(**inputs))
    assert c.turn_direction == TurnDirection.none

  def test_far_side_turn_needs_no_lane_choice(self):
    c = controller(auto=True)
    c.update(car(vEgo=5.), True, auto_nav(modifier='left', outer_lanes=(False, False)))
    assert c.turn_direction == TurnDirection.turnLeft  # across oncoming traffic, with no lane guidance
    for lanes in [(lane('left', active_direction='left'), lane('straight'))]:
      c = controller(auto=True)
      c.update(car(vEgo=5.), True, auto_nav(modifier='left', lanes=lanes))
      assert c.turn_direction == TurnDirection.none  # a turn lane the car may not be in: waits for the blinker
      c.update(car(vEgo=5., leftBlinker=True), True, auto_nav(modifier='left', lanes=lanes))
      assert c.turn_direction == TurnDirection.turnLeft

  def test_left_hand_traffic(self):
    for modifier, outer, lanes, expected in [('left', (True, False), (), TurnDirection.turnLeft),
                                             ('left', (False, False), (), TurnDirection.none),
                                             ('right', (False, False), (), TurnDirection.turnRight),
                                             ('right', (False, False), (lane('right', active_direction='right'), lane('straight')), TurnDirection.none)]:
      c = controller(auto=True)
      c.update(car(vEgo=5.), True, auto_nav(modifier=modifier, outer_lanes=outer, lanes=lanes, right_hand_traffic=False))
      assert c.turn_direction == expected, (modifier, outer, lanes)


class TestOnePulsePerTurn(OpenpilotTestCase):
  def test_started_turn_holds(self):
    c = controller(auto=True)
    c.update(car(vEgo=5.), True, auto_nav())
    for carstate in [car(vEgo=5., rightBlindspot=True), car(vEgo=9.), car(vEgo=5., standstill=True), car(vEgo=5., rightBlinker=True)]:
      c.update(carstate, True, auto_nav(outer_lanes=(False, False)))
      assert c.turn_direction == TurnDirection.turnRight  # held to the maneuver point, so the model never gets a second pulse
    c.update(car(vEgo=5.), True, auto_nav(distance=-6.))
    assert c.turn_direction == TurnDirection.none

  def test_called_off_auto_turn_stays_off(self):
    c = controller(auto=True)
    c.update(car(vEgo=5.), True, auto_nav(distance=30.))
    for inputs in [auto_nav(distance=28., long_active=False), auto_nav(distance=26.), auto_nav(distance=24.)]:
      c.update(car(vEgo=5.), True, inputs)  # the gas called it off, and it does not come back by itself
      assert c.turn_direction == TurnDirection.none
    c.update(car(vEgo=5., rightBlinker=True), True, auto_nav(distance=22.))
    assert c.turn_direction == TurnDirection.turnRight  # signaling for it restarts it

  def test_route_gap_does_not_restart(self):
    for carstate in [car(vEgo=5.), car(vEgo=5., rightBlinker=True)]:
      c = controller(auto=True)
      c.update(carstate, True, auto_nav(distance=30.))
      assert c.turn_direction == TurnDirection.turnRight
      c.update(carstate, True, None)  # navd went quiet for a moment
      c.update(carstate, True, auto_nav(distance=25.))
      assert c.turn_direction == TurnDirection.none

  def test_blinker_off_and_on_again_restarts(self):
    c = controller()
    c.update(car(rightBlinker=True), True, nav(distance=40.))
    c.update(car(), True, nav(distance=38.))
    assert c.turn_direction == TurnDirection.none
    c.update(car(rightBlinker=True), True, nav(distance=36.))
    assert c.turn_direction == TurnDirection.turnRight

  def test_next_maneuver_starts_again(self):
    c = controller(auto=True)
    c.update(car(vEgo=5.), True, auto_nav(distance=30.))
    c.update(car(vEgo=5.), True, auto_nav(distance=-6.))
    c.update(car(vEgo=5.), True, auto_nav(modifier='left', distance=300.))  # navd moved on to the next turn
    c.update(car(vEgo=5.), True, auto_nav(modifier='left', distance=30.))
    assert c.turn_direction == TurnDirection.turnLeft
    c = controller(auto=True)
    c.update(car(vEgo=5.), True, auto_nav(distance=30.))
    c.update(car(vEgo=5.), True, auto_nav(distance=-6.))
    c.update(car(vEgo=5.), True, auto_nav(distance=200.))  # the same kind of turn, after a reroute
    c.update(car(vEgo=5.), True, auto_nav(distance=30.))
    assert c.turn_direction == TurnDirection.turnRight


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

  def test_auto_turn_desire(self):
    dh = run(desire_helper(turns=False, auto=True), car(vEgo=5.), auto_nav())
    assert dh.desire == log.Desire.turnRight and dh.lane_turn_direction == TurnDirection.turnRight

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
    for carstate, expected in [(car(rightBlinker=True, vEgo=5.), log.Desire.turnRight), (car(vEgo=25., **nudge()), log.Desire.none),
                               (car(vEgo=5.), log.Desire.none)]:
      if carstate.rightBlinker:
        inputs = nav(distance=70.)
      elif carstate.steeringPressed:
        inputs = nav('off ramp', 'slight right', 100., RIGHT_EXIT_LANES)
      else:
        inputs = auto_nav()
      assert run(desire_helper(turns=False, exits=False, lane_turns=True), carstate, inputs).desire == expected
