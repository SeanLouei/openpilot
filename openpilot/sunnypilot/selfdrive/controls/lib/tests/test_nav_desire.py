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
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_INSTRUCTION_MAX_AGE, NAV_TURN_STEER_DISTANCE, NAV_TURN_TRIGGER_TIME, \
  ManeuverTracker, NavDesireController, NavInputs, active_lane, fresh_nav_instruction, nav_inputs, normalize_direction, outer_lanes

TurnDirection = custom.ModelDataV2SP.TurnDirection
TURN_DESIRES = (log.Desire.turnLeft, log.Desire.turnRight)
TURN_DIRECTIONS = (TurnDirection.turnLeft, TurnDirection.turnRight)


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
  c.param_read_counter = 1  # the settings above stand in for params, so skip the first param read
  return c


def desire_helper(turns=True, exits=True, auto=False, lane_turns=False, lane_change=AutoLaneChangeMode.NUDGE):
  # through params, which DesireHelper reads again every 50 frames of a long approach
  params = Params()
  for key, value in [('NavTurnDesire', turns), ('NavExitDesire', exits), ('NavAutoTurn', auto), ('LaneTurnDesire', lane_turns)]:
    params.put_bool(key, value)
  params.put('LaneTurnValue', 20.)
  params.put('AutoLaneChangeTimer', int(lane_change))
  return DesireHelper()


def trigger_distance(v_ego):
  return NAV_TURN_STEER_DISTANCE + NAV_TURN_TRIGGER_TIME * v_ego


def drive(target, start, v_ego, seconds, inputs=nav, carstate=None, lane_change_prob=1.0, **kwargs):
  """Drive toward the route's maneuver from `start` meters at v_ego, with navd's distance fresh every frame. Returns each
  frame's (output, distance): the turn direction of a controller, or the desire of a DesireHelper."""
  frames, distance = [], start
  for _ in range(round(seconds / DT_MDL)):
    carstate_now = carstate if carstate is not None else car(vEgo=v_ego)
    if isinstance(target, DesireHelper):
      target.update(carstate_now, True, lane_change_prob, False, False, inputs(distance=distance, **kwargs))
      frames.append((target.desire, distance))
    else:
      target.update(carstate_now, True, inputs(distance=distance, **kwargs))
      frames.append((target.turn_direction, distance))
    distance -= v_ego * DT_MDL
  return frames


def pulses(frames, values=TURN_DIRECTIONS + TURN_DESIRES, before=None):
  """The distances where the output rises into one of the values: each one is a fresh pulse to the model.
  `before` is the output just before these frames, when they continue an earlier drive."""
  outputs = [output for output, _ in frames]
  return [distance for (output, distance), prev in zip(frames, [before, *outputs[:-1]], strict=True) if output in values and output != prev]


class TestManeuverTracker(OpenpilotTestCase):
  def test_distance_only_shrinks(self):
    t = ManeuverTracker()
    assert t.update(maneuver(distance=100.), 0., 10.)
    for _ in range(20):  # navd keeps reporting 100 m, frozen, while the car drives 10 m
      assert not t.update(maneuver(distance=100.), 0., 10.)
    assert math.isclose(t.distance, 90., abs_tol=1e-6)
    t.update(None, 0., 10.)  # and keeps going without one
    assert math.isclose(t.distance, 89.5, abs_tol=1e-6)

  def test_new_maneuver(self):
    t = ManeuverTracker()
    t.update(maneuver(distance=100.), 0., 0.)
    assert t.update(maneuver(modifier='left', distance=100.), 0., 0.)
    assert not t.update(maneuver(modifier='left', distance=120.), 0., 0.)  # noise
    assert t.update(maneuver(modifier='left', distance=160.), 0., 0.)  # a reroute: more than 30 m over navd's last distance
    assert t.distance == 160. and t.turn()

  def test_passed_stays_passed(self):
    t = ManeuverTracker()
    t.update(maneuver(distance=0.2), 0., 5.)
    t.update(maneuver(distance=0.2), 0., 5.)
    assert t.passed
    for _ in range(60):  # navd still reports the completed turn just ahead
      t.update(maneuver(distance=1.), 0., 5.)
    assert t.passed and t.distance < 0.

  def test_unreadable_distance(self):
    t = ManeuverTracker()
    t.update(maneuver(distance=50.), 0., 0.)
    for distance in (math.nan, math.inf):
      assert not t.update(maneuver(distance=distance), 0., 0.)
    assert t.distance == 50.


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
    sm.recv_time['navInstruction'], sm.valid['navInstruction'] = now, False  # no route, or no GPS fix
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


class TestRouteTurnTiming(OpenpilotTestCase):
  @parameterized.expand([(2.,), (5.,), (8.5,)])
  def test_pulses_once_3_s_before_steering(self, v_ego):
    edges = pulses(drive(controller(), 120., v_ego, 140. / v_ego, carstate=car(vEgo=v_ego, rightBlinker=True)))
    assert len(edges) == 1  # one pulse through the whole approach and turn, which ends 20 m past the maneuver point
    assert trigger_distance(v_ego) - v_ego * DT_MDL - 1e-6 <= edges[0] <= trigger_distance(v_ego)

  def test_too_fast_for_the_turn(self):
    assert not pulses(drive(controller(), 120., 9.5, 12., carstate=car(vEgo=9.5, rightBlinker=True)))

  @parameterized.expand([
    (car(vEgo=5., rightBlinker=True), 1),
    (car(vEgo=5.), 0),
    (car(vEgo=5., leftBlinker=True), 0),
    (car(vEgo=5., leftBlinker=True, rightBlinker=True), 0),  # hazards
    (car(vEgo=5., rightBlinker=True, rightBlindspot=True), 0),
  ])
  def test_requires_matching_blinker(self, carstate, expected):
    assert len(pulses(drive(controller(), 60., 5., 10., carstate=carstate))) == expected

  def test_completed_turn_never_pulses_again(self):
    c = controller()
    blinker = car(vEgo=5., rightBlinker=True)
    approach = drive(c, 60., 5., 14., carstate=blinker)  # through the turn to 10 m past it
    assert len(pulses(approach)) == 1
    # navd still reports the turn just completed for another block, a meter or two ahead, while the car drives on
    assert not pulses(drive(c, 1.5, 0., 12., carstate=blinker), before=approach[-1][0])

  def test_frozen_gps_distance_pulses_once(self):
    c = controller()
    frames = [(c.update(car(vEgo=5., rightBlinker=True), True, nav(distance=30.)), c.turn_direction)[1] for _ in range(200)]
    edges = [i for i, (prev, cur) in enumerate(zip([None, *frames[:-1]], frames, strict=True)) if cur in TURN_DIRECTIONS and cur != prev]
    assert len(edges) == 1

  def test_waiting_short_of_the_turn_pulses_again(self):
    c = controller()
    blinker = {'rightBlinker': True}
    approach = drive(c, 60., 5., 9.6, carstate=car(vEgo=5., **blinker))  # to 12 m: the pulse at 23 m
    stopped = drive(c, 12., 0., 10., carstate=car(vEgo=0., standstill=True, **blinker))
    moving_off = drive(c, 12., 3., 8., carstate=car(vEgo=3., **blinker))
    assert len(pulses(approach)) == 1
    assert len(pulses(stopped, before=approach[-1][0])) >= 2  # every 4 s, so the 5 s desire history never runs out while it waits
    assert all(distance > NAV_TURN_STEER_DISTANCE for distance in pulses(moving_off, before=stopped[-1][0]))  # never mid-turn

  def test_no_fresh_pulse_into_the_blind_spot(self):
    c = controller()
    approach = drive(c, 60., 5., 9.6, carstate=car(vEgo=5., rightBlinker=True))
    stopped = drive(c, 12., 0., 10., carstate=car(vEgo=0., standstill=True, rightBlinker=True, rightBlindspot=True))
    assert not pulses(stopped, before=approach[-1][0])

  def test_route_gap_holds_the_turn(self):
    c = controller()
    drive(c, 40., 5., 4., carstate=car(vEgo=5., rightBlinker=True))
    assert c.turn_direction == TurnDirection.turnRight
    for _ in range(40):  # navd goes quiet, or loses the GPS fix
      c.update(car(vEgo=5., rightBlinker=True), True, NavInputs())
      assert c.turn_direction == TurnDirection.turnRight

  def test_blinker_latches_as_the_turn_signal(self):
    c = controller()
    c.update(car(vEgo=10., rightBlinker=True), True, nav(distance=120.))  # within 15 m + 12 s
    assert c.route_turn_signal
    c.update(car(vEgo=3., rightBlinker=True), True, nav(distance=60.))  # slowing shrinks the distance to 51 m
    for _ in range(80):  # and navd going quiet
      c.update(car(vEgo=3., rightBlinker=True), True, NavInputs())
    assert c.route_turn_signal  # none of which makes it a lane change blinker
    c.update(car(vEgo=3.), True, nav(distance=55.))
    assert not c.route_turn_signal
    c.update(car(vEgo=8., rightBlinker=True), True, nav(distance=120.))  # beyond 15 m + 12 s: a lane change blinker
    assert not c.route_turn_signal

  @parameterized.expand([
    ('turn', 'sharp left', 1), ('end of road', 'left', 1), ('on ramp', 'right', 1), ('continue', 'right', 1),
    ('turn', 'slight right', 0), ('turn', 'uturn', 0), ('roundabout', 'right', 0), ('off ramp', 'right', 0),
  ])
  def test_maneuvers(self, maneuver_type, modifier, expected):
    left = 'left' in modifier
    carstate = car(vEgo=5., leftBlinker=left, rightBlinker=not left)
    assert len(pulses(drive(controller(), 60., 5., 10., carstate=carstate, maneuver_type=maneuver_type, modifier=modifier))) == expected

  def test_inactive(self):
    for c, lateral_active, inputs in [(controller(turns=False), True, nav(distance=20.)), (controller(), False, nav(distance=20.)),
                                      (controller(), True, None), (controller(), True, NavInputs())]:
      for _ in range(20):
        c.update(car(vEgo=5., rightBlinker=True), lateral_active, inputs)
        assert c.turn_direction == TurnDirection.none and not c.route_turn_signal

  def test_unreadable_distance(self):
    for distance in (math.nan, math.inf, -math.inf):
      c = controller(auto=True)
      for _ in range(20):
        c.update(car(rightBlinker=True, vEgo=5.), True, auto_nav(distance=distance))
        assert c.turn_direction == TurnDirection.none and not c.route_turn_signal


class TestAutoTurns(OpenpilotTestCase):
  def test_auto_turn(self):
    edges = pulses(drive(controller(turns=False, auto=True), 60., 5., 16., inputs=auto_nav))
    assert len(edges) == 1 and abs(edges[0] - trigger_distance(5.)) <= 5. * DT_MDL

  @parameterized.expand([
    ('auto turns off', {}, {}, False),
    ("gas pressed, or the car's own cruise control", {}, {'long_active': False}, True),
    ('not slowed down yet', {'vEgo': 8.}, {}, True),
    ('not in the rightmost lane', {}, {'outer_lanes': (False, False)}, True),
    ('blind spot', {'rightBlindspot': True}, {}, True),
    ('the other blinker', {'leftBlinker': True}, {}, True),
    ('hazards', {'leftBlinker': True, 'rightBlinker': True}, {}, True),
  ])
  def test_auto_turn_blocked(self, _name, car_state, inputs, auto):
    carstate = car(**({'vEgo': 5.} | car_state))
    assert not pulses(drive(controller(turns=False, auto=auto), 60., carstate.vEgo, 10., inputs=auto_nav, carstate=carstate, **inputs))

  def test_far_side_turn_needs_no_lane_choice(self):
    assert len(pulses(drive(controller(auto=True), 60., 5., 10., inputs=auto_nav, modifier='left', outer_lanes=(False, False)))) == 1
    turn_lanes = (lane('left', active_direction='left'), lane('straight'))
    c = controller(auto=True)
    waiting = drive(c, 60., 5., 9., inputs=auto_nav, modifier='left', lanes=turn_lanes)
    assert not pulses(waiting)  # a turn lane the car may not be in: waits for the blinker
    signaled = drive(c, 15., 5., 1., inputs=auto_nav, modifier='left', lanes=turn_lanes, carstate=car(vEgo=5., leftBlinker=True))
    assert len(pulses(signaled, before=waiting[-1][0])) == 1

  @parameterized.expand([
    ('left', (True, False), (), 1), ('left', (False, False), (), 0), ('right', (False, False), (), 1),
    ('right', (False, False), (lane('right', active_direction='right'), lane('straight')), 0),
  ])
  def test_left_hand_traffic(self, modifier, outer, lanes, expected):
    frames = drive(controller(auto=True), 60., 5., 10., inputs=auto_nav, modifier=modifier, outer_lanes=outer, lanes=lanes,
                   right_hand_traffic=False)
    assert len(pulses(frames)) == expected

  def test_called_off_auto_turn_stays_off(self):
    c = controller(auto=True)
    started = drive(c, 30., 5., 2., inputs=auto_nav)
    gas = drive(c, 20., 5., 0.5, inputs=auto_nav, long_active=False)  # the gas calls it off
    released = drive(c, 17.5, 5., 1., inputs=auto_nav)  # and it does not come back by itself
    signaled = drive(c, 12.5, 5., 1., inputs=auto_nav, carstate=car(vEgo=5., rightBlinker=True))  # signaling restarts it
    assert len(pulses(started)) == 1 and not pulses(gas, before=started[-1][0]) and not pulses(released, before=gas[-1][0])
    assert len(pulses(signaled, before=released[-1][0])) == 1

  def test_next_maneuver_starts_again(self):
    c = controller(auto=True)
    first = drive(c, 40., 5., 10., inputs=auto_nav)
    second = drive(c, 300., 5., 58., inputs=auto_nav, modifier='left')  # navd moved on to the next turn
    third = drive(c, 200., 5., 38., inputs=auto_nav, modifier='left')  # the same kind of turn again, after a reroute
    assert len(pulses(first)) == len(pulses(second, before=first[-1][0])) == len(pulses(third, before=second[-1][0])) == 1


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
    dh = desire_helper()
    frames = drive(dh, 60., 5., 8., carstate=car(vEgo=5., rightBlinker=True))
    assert pulses(frames) == pulses(frames, (log.Desire.turnRight,)) and len(pulses(frames)) == 1
    assert dh.lane_turn_direction == TurnDirection.turnRight  # shows the turn alert

  def test_auto_turn_desire(self):
    dh = desire_helper(turns=False, auto=True)
    assert len(pulses(drive(dh, 60., 5., 8., inputs=auto_nav))) == 1 and dh.desire == log.Desire.turnRight

  def test_lane_turn_waits_for_the_route_turn(self):
    carstate = car(vEgo=5., rightBlinker=True)
    assert run_once(desire_helper(lane_turns=True), carstate, None) == log.Desire.turnRight  # a lane turn starts right away
    edges = pulses(drive(desire_helper(lane_turns=True), 70., 5., 12., carstate=carstate))
    assert len(edges) == 1 and edges[0] <= trigger_distance(5.)  # within 15 m + 12 s of the route turn, it waits for it

  def test_held_lane_turn_survives_slowing(self):
    dh = desire_helper(lane_turns=True)
    frames = drive(dh, 70., 5., 2., carstate=car(vEgo=5., rightBlinker=True))  # latched at 70 m
    frames += drive(dh, 60., 2., 25., carstate=car(vEgo=2., rightBlinker=True))  # slowing shrinks the distance to 39 m
    edges = pulses(frames)
    assert len(edges) == 1 and edges[0] <= trigger_distance(2.)

  def test_early_lane_turn_is_not_taken_over(self):
    edges = pulses(drive(desire_helper(lane_turns=True), 100., 5., 18., carstate=car(vEgo=5., rightBlinker=True)))
    assert len(edges) == 1 and edges[0] > 95.  # the lane turn went out at the blinker, beyond 75 m, and stays the only pulse

  def test_blinker_for_other_turn_is_not_held(self):
    dh = desire_helper(lane_turns=True)
    assert run_once(dh, car(vEgo=5., rightBlinker=True), nav(modifier='left', distance=70.)) == log.Desire.turnRight

  def test_route_blinker_never_starts_nudgeless_lane_change(self):
    dh = desire_helper(lane_change=AutoLaneChangeMode.NUDGELESS)
    frames = drive(dh, 150., 12., 2., carstate=car(vEgo=12., rightBlinker=True))  # latched at 150 m
    frames += drive(dh, 126., 9.2, 3., carstate=car(vEgo=9.2, rightBlinker=True))  # slowing shrinks the distance below 126 m
    for _ in range(60):  # and navd goes quiet for 3 s
      dh.update(car(vEgo=9.2, rightBlinker=True), True, 1.0, False, False, NavInputs())
      assert dh.lane_change_state != LaneChangeState.laneChangeStarting
    assert all(desire != log.Desire.laneChangeRight for desire, _ in frames)

  def test_nudge_changes_lanes_before_the_turn(self):
    dh = desire_helper()
    drive(dh, 150., 12., 1., carstate=car(vEgo=12., rightBlinker=True))
    frames = drive(dh, 138., 12., 0.2, carstate=car(vEgo=12., rightBlinker=True, **nudge()))
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting and frames[-1][0] == log.Desire.laneChangeRight

  def test_turn_waits_for_a_lane_change(self):
    dh = desire_helper()
    drive(dh, 80., 8.95, 1., carstate=car(vEgo=8.95, rightBlinker=True))
    frames = drive(dh, 71., 8.95, 6., carstate=car(vEgo=8.95, rightBlinker=True, **nudge()))  # nudged into a turn lane
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert all(desire == log.Desire.laneChangeRight for desire, _ in frames[1:])  # through the trigger point at 35 m

  def test_no_lane_change_hides_behind_a_turn(self):
    dh = desire_helper()
    frames = drive(dh, 60., 8.95, 3.5, carstate=car(vEgo=8.95, rightBlinker=True))  # the turn starts at 35 m
    frames += drive(dh, 28.7, 8.95, 1., carstate=car(vEgo=8.95, rightBlinker=True, **nudge()))  # steering into the turn
    frames += drive(dh, 19.7, 8.95, 3., carstate=car(vEgo=8.95, rightBlinker=True, rightBlindspot=True, **nudge()))  # a cyclist
    assert len(pulses(frames, TURN_DESIRES)) == 1
    assert dh.lane_change_state == LaneChangeState.off
    assert all(desire not in (log.Desire.laneChangeLeft, log.Desire.laneChangeRight) for desire, _ in frames)

  def test_exit_keep_only_without_lane_change(self):
    exit_ahead = nav('off ramp', 'slight right', 100., RIGHT_EXIT_LANES)
    assert run_once(desire_helper(), car(vEgo=25., **nudge()), exit_ahead, frames=10) == log.Desire.keepRight
    dh = desire_helper()
    run_once(dh, car(vEgo=25., rightBlinker=True), exit_ahead, frames=10)  # signaling: a lane change, not a keep
    assert run_once(dh, car(vEgo=25., rightBlinker=True, **nudge()), exit_ahead) == log.Desire.laneChangeRight

  def test_disabled_leaves_desires_unchanged(self):
    for carstate, inputs, expected in [(car(rightBlinker=True, vEgo=5.), nav(distance=70.), log.Desire.turnRight),
                                       (car(vEgo=25., **nudge()), nav('off ramp', 'slight right', 100., RIGHT_EXIT_LANES), log.Desire.none),
                                       (car(vEgo=5.), auto_nav(distance=20.), log.Desire.none)]:
      assert run_once(desire_helper(turns=False, exits=False, lane_turns=True), carstate, inputs, frames=10) == expected

  def test_settings_from_params(self):
    Params().put_bool("NavTurnDesire", True)
    dh = DesireHelper()
    assert len(pulses(drive(dh, 60., 5., 8., carstate=car(vEgo=5., rightBlinker=True)))) == 1
    Params().put_bool("NavTurnDesire", False)
    drive(dh, 300., 5., 3., carstate=car(vEgo=5.), modifier='left')  # the next maneuver, while the setting is read again
    assert not pulses(drive(dh, 60., 5., 8., carstate=car(vEgo=5., leftBlinker=True), modifier='left'))

  def test_unreadable_route_never_stops_the_model(self):
    class Unreadable:
      maneuverType, maneuverModifier, lanes = 'turn', 'right', []

      @property
      def maneuverDistance(self):
        raise ValueError("corrupt message")

    dh = desire_helper(lane_turns=True)
    for _ in range(5):
      dh.update(car(vEgo=5., rightBlinker=True), True, 1.0, False, False, NavInputs(Unreadable()))
    assert dh.nav_desire_failed and dh.desire == log.Desire.turnRight  # the lane turn still works without the route


def run_once(dh, carstate, inputs, frames=1):
  for _ in range(frames):
    dh.update(carstate, True, 1.0, False, False, inputs)
  return dh.desire
