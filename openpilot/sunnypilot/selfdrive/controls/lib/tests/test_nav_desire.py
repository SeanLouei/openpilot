"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import dataclasses
import math
import time
from types import SimpleNamespace

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom
from openpilot.common.parameterized import parameterized
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_AGREE_DISTANCE, NAV_INSTRUCTION_GRACE, NAV_INSTRUCTION_MAX_AGE, \
  NAV_MAX_PULL_IN, NAV_NEAR_DISTANCE, NAV_PUSH_DEADBAND, NAV_RESEED_DISTANCE, NAV_TURN_CRAWL_SPEED, NAV_TURN_DECEL, NAV_TURN_HOLD, NAV_TURN_LINED_UP, \
  NAV_TURN_MARGIN, NAV_TURN_MAX_DECEL, NAV_TURN_REPULSE_TIME, NAV_TURN_SPEED, NAV_TURN_STEER_DISTANCE, NAV_TURN_STOCK_FAR_SPEED, \
  NAV_TURN_STOCK_NEAR_SPEED, NAV_TURN_TRIGGER_TIME, OUTER_LANE_TIME, ManeuverTracker, NavDesireController, NavInputs, active_lane, \
  fresh_nav_instruction, nav_inputs, normalize_direction, outer_lanes, time_to_steer, turn_speed_limit

TurnDirection = custom.ModelDataV2SP.TurnDirection
TURN_DESIRES = (log.Desire.turnLeft, log.Desire.turnRight)
TURN_DIRECTIONS = (TurnDirection.turnLeft, TurnDirection.turnRight)


def car(**kwargs):
  return SimpleNamespace(**({'vEgo': 8., 'leftBlinker': False, 'rightBlinker': False, 'leftBlindspot': False, 'rightBlindspot': False,
                             'steeringPressed': False, 'steeringTorque': 0., 'standstill': False, 'brakePressed': False,
                             'gasPressed': False} | kwargs))


def maneuver(maneuver_type='turn', modifier='right', distance=40., lanes=(), position=1000.):
  """navd's instruction for a maneuver `position` meters before the destination, `distance` meters ahead."""
  return SimpleNamespace(maneuverType=maneuver_type, maneuverModifier=modifier, maneuverDistance=distance,
                         distanceRemaining=distance + position, lanes=list(lanes))


def nav(maneuver_type='turn', modifier='right', distance=40., lanes=(), age=0., position=1000., **inputs):
  return NavInputs(maneuver(maneuver_type, modifier, distance, lanes, position), age, **inputs)


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


def drive(target, start, v_ego, seconds, inputs=nav, carstate=None, lane_change_prob=1.0, lateral_active=True, **kwargs):
  """Drive toward the route's maneuver from `start` meters at v_ego, with navd's distance fresh every frame. Returns each
  frame's (output, distance): the turn direction of a controller, or the desire of a DesireHelper."""
  frames, distance = [], start
  for _ in range(round(seconds / DT_MDL)):
    carstate_now = carstate if carstate is not None else car(vEgo=v_ego)
    if isinstance(target, DesireHelper):
      target.update(carstate_now, lateral_active, lane_change_prob, False, False, inputs(distance=distance, **kwargs))
      frames.append((target.desire, distance))
    else:
      target.update(carstate_now, lateral_active, inputs(distance=distance, **kwargs))
      frames.append((target.turn_direction, distance))
    distance -= v_ego * DT_MDL
  return frames


def drive_limits(c, start, v_ego, seconds, inputs=auto_nav, carstate=None, lateral_active=True, **kwargs):
  """Like drive, returning each frame's (speed limit, distance)."""
  frames, distance = [], start
  for _ in range(round(seconds / DT_MDL)):
    c.update(carstate if carstate is not None else car(vEgo=v_ego), lateral_active, inputs(distance=distance, **kwargs))
    frames.append((c.speed_limit, distance))
    distance -= v_ego * DT_MDL
  return frames


def pulses(frames, values=TURN_DIRECTIONS + TURN_DESIRES, before=None):
  """The distances where the output rises into one of the values: each one is a fresh pulse to the model.
  `before` is the output just before these frames, when they continue an earlier drive."""
  outputs = [frame[0] for frame in frames]
  return [frame[1] for frame, prev in zip(frames, [before, *outputs[:-1]], strict=True) if frame[0] in values and frame[0] != prev]


def slowed_trigger_distance(v_ego):
  """Where a turn pulses while openpilot slows the car for it: 3 s before the steering point at the speed the slowdown imposes."""
  return next(d / 100. for d in range(6000, 0, -1) if time_to_steer(d / 100., v_ego, True) <= NAV_TURN_TRIGGER_TIME)


def closed_loop(target, start, v_set, seconds, inputs=auto_nav, carstate=None, lateral_active=True, navd_period=1., model_speed=None,
                lane_change_prob=1.0, **kwargs):
  """Drive toward the maneuver from `start` meters at a set speed, the car slowing for the route turn's speed limit as
  plannerd does (the profile's deceleration and a correction toward it, at most NAV_TURN_MAX_DECEL, through a 0.3 s lag)
  while openpilot controls the speed, and for a driving model that wants slower (model_speed of the distance). navd
  reports once per navd_period. carstate(distance, v) gives the car's state, lane_change_prob the model's lane change
  probability, or a function of the distance. Returns each frame's (output, distance, speed, speed limit): the turn
  direction of a controller, or the desire of a DesireHelper."""
  c = target.nav_desire if isinstance(target, DesireHelper) else target
  frames, distance, v, a, a_plan = [], start, v_set, 0., 0.
  message, sent = None, 0.
  for i in range(round(seconds / DT_MDL)):
    t = i * DT_MDL
    if i % max(round(navd_period / DT_MDL), 1) == 0:
      message, sent = inputs(distance=distance, **kwargs), t
    nav_now = dataclasses.replace(message, age=t - sent)
    carstate_now = carstate(distance, v) if carstate is not None else car(vEgo=v)
    if isinstance(target, DesireHelper):
      prob = lane_change_prob(distance) if callable(lane_change_prob) else lane_change_prob
      target.update(carstate_now, lateral_active, prob, False, False, nav_now)
      frames.append((target.desire, distance, v, c.speed_limit))
    else:
      target.update(carstate_now, lateral_active, nav_now)
      frames.append((target.turn_direction, distance, v, c.speed_limit))
    wanted = [float(np.clip(v_set - v, -0.5, 1.))]
    if model_speed is not None:
      wanted.append(float(np.clip(model_speed(distance) - v, -2., 1.)))
    if nav_now.long_active and c.speed_limit > 0.:
      decel = NAV_TURN_DECEL if c.speed_limit > NAV_TURN_SPEED else 0.
      wanted.append(max(float(np.clip(c.speed_limit - v - decel, -NAV_TURN_MAX_DECEL, 1.)), a_plan - 1.5 * DT_MDL))
    a_plan = min(wanted)
    a += (a_plan - a) * DT_MDL / 0.3
    v = max(v + a * DT_MDL, 0.)
    distance -= v * DT_MDL
  return frames


def in_history(frames, at_distance):
  """The turn pulses still in the model's 5 s desire history when the car is at `at_distance`."""
  index = next(i for i, frame in enumerate(frames) if frame[1] <= at_distance)
  recent = frames[max(index - round(5. / DT_MDL) + 1, 0):index + 1]
  before = frames[index - len(recent)][0] if index >= len(recent) else None
  return pulses(recent, before=before)


def track(t, reported, v_ego, seconds, committed=False, period=1.):
  """Feed a tracker navd's distance, reported(true distance), once per period while the car drives at v_ego, from where
  the tracker's distance is. Returns each frame's (true distance, tracker distance)."""
  true, frames = t.distance, []
  for i in range(round(seconds / DT_MDL)):
    true -= v_ego * DT_MDL
    instruction = maneuver(distance=reported(true)) if i % round(period / DT_MDL) == 0 else t.instruction
    t.update(instruction, 0., v_ego, committed)
    frames.append((true, t.distance))
  return frames


class TestManeuverTracker(OpenpilotTestCase):
  def test_frozen_distance_holds_the_approach(self):
    # navd keeps reporting 100 m, frozen but fresh, while the car drives on. The dead reckoned distance used to count down
    # to a turn navd never saw coming; now navd's distance, the safe side, holds it back, so no turn comes up
    t = ManeuverTracker()
    assert t.update(maneuver(distance=100.), 0., 10.)
    frames = track(t, lambda _: 100., 10., 20., period=DT_MDL)
    assert min(distance for _, distance in frames) > 100. - NAV_RESEED_DISTANCE - 2.  # never more than a reseed short
    assert max(distance for _, distance in frames[-round(3. / DT_MDL):]) > 98.  # and back out to navd's
    # within NAV_NEAR_DISTANCE of the maneuver it holds back only slowly, and the turn waits for navd to agree instead
    t = ManeuverTracker()
    t.update(maneuver(distance=40.), 0., 5.)
    track(t, lambda _: 40., 5., 4.)
    assert t.distance < 40. - 15. and t.residual > NAV_AGREE_DISTANCE

  def test_started_turn_only_comes_in(self):
    t = ManeuverTracker()
    t.update(maneuver(distance=100.), 0., 10.)
    frames = track(t, lambda _: 100., 10., 1., committed=True, period=DT_MDL)  # the turn went out: nothing moves it back
    assert math.isclose(frames[-1][1], 90., abs_tol=1e-6)
    track(t, lambda true: true - 10., 10., 3., committed=True)  # though a shorter navd distance still pulls it in
    assert t.distance < 60. - 3.
    distance = t.distance
    t.update(None, 0., 10.)  # and it keeps going without one
    assert math.isclose(t.distance, distance - 0.5, abs_tol=1e-6)

  def test_new_maneuver(self):
    t = ManeuverTracker()
    assert t.update(maneuver(distance=100.), 0., 0.)
    assert t.update(maneuver(modifier='left', distance=100.), 0., 0.)  # another kind of maneuver
    assert not t.update(maneuver(modifier='left', distance=140.), 0., 0.)  # a GPS jump back: the car moved, not the maneuver
    assert t.distance == 100.
    assert t.update(maneuver(modifier='left', distance=160., position=1200.), 0., 0.)  # a reroute that turns somewhere else
    assert t.distance == 160. and t.turn()
    assert t.update(maneuver(modifier='left', distance=30., position=1160.), 0., 0.)  # the same kind of turn 40 m after it

  def test_gps_jump_ahead_pulls_in_slowly(self):
    t = ManeuverTracker()
    t.update(maneuver(distance=100.), 0., 0.)
    t.update(maneuver(distance=60.), 0., 0.)  # one fix 40 m ahead does nothing: the median of the last three needs two
    assert t.distance == 100.
    for _ in range(round(1. / DT_MDL) - 1):  # navd reports 40 m less for a second
      t.update(maneuver(distance=60.), 0., 0.)
    assert math.isclose(t.distance, 100. - NAV_MAX_PULL_IN * (1. - DT_MDL), abs_tol=1e-6)

  def test_gps_error_ahead_is_taken_back(self):
    # a 25 m GPS error ahead for 4 s, then correct reports: the distance comes back to the truth, so the turn is not early
    for start in (150., 80.):  # also when the error ends within NAV_NEAR_DISTANCE of the maneuver
      t = ManeuverTracker()
      t.update(maneuver(distance=start), 0., 5.)
      error = track(t, lambda true: true - 25., 5., 4.)
      assert error[-1][1] < error[-1][0] - 10.  # pulled in at NAV_MAX_PULL_IN once two reports agree
      frames = track(t, lambda true: true, 5., 6.)
      assert abs(frames[-1][1] - frames[-1][0]) < 3.5

  def test_follows_a_speed_error(self):
    # vEgo 1 % high over a 3 km step: the dead reckoned distance falls short, and navd pushes it back out
    t = ManeuverTracker()
    t.update(maneuver(distance=3000.), 0., 15.)
    true, frames = 3000., []
    for i in range(round(2990. / 15. / DT_MDL)):
      true -= 15. * DT_MDL
      t.update(maneuver(distance=true) if i % 20 == 0 else t.instruction, 0., 15. * 1.01)
      frames.append((true, t.distance))
    assert all(abs(distance - true) < 4. for true, distance in frames)

  def test_gps_error_behind_near_the_maneuver(self):
    # a 20 m GPS error behind the car for 4 s within NAV_NEAR_DISTANCE: taken outright, it would time the turn late, so
    # the distance only creeps out, and navd disagrees until the error ends
    t = ManeuverTracker()
    t.update(maneuver(distance=55.), 0., 5.)
    frames = track(t, lambda true: true + 20., 5., 4.)
    assert all(distance - true < 4. for true, distance in frames) and t.residual > NAV_AGREE_DISTANCE
    frames = track(t, lambda true: true, 5., 3.)
    assert abs(frames[-1][1] - frames[-1][0]) < 4. and t.residual < NAV_AGREE_DISTANCE
    # beyond it, a longer navd distance (the car left the route or joined it from beside it) replaces it at once
    t = ManeuverTracker()
    t.update(maneuver(distance=150.), 0., 5.)
    frames = track(t, lambda true: true + 40., 5., 3.)
    assert abs(frames[-1][1] - frames[-1][0] - 40.) < 1.

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
    for edge in (3.2, 4.1, 4.6):  # a bike lane, a parking lane, or a narrow lane between the car and the curb
      assert outer_lanes(model(edge_y=(-6., edge))) == (False, False)
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
    assert nav_inputs(sm, model_msg, True).outer_lanes == (False, False)  # no model output yet
    assert nav_inputs(sm, model_msg, True).yaw_rate == 0.
    model_msg.roadEdgeStds, model_msg.laneLineProbs = [0.2, 0.2], [0.9, 0.9, 0.9, 0.05]
    edges = model_msg.init('roadEdges', 2)
    edges[0].y, edges[1].y = [-6.], [2.5]
    model_msg.orientationRate.z = [-0.3, -0.1]  # turning left: the model's frame is forward, right, down
    inputs = nav_inputs(sm, model_msg, True)
    assert inputs.instruction is not None and inputs.long_active and inputs.right_hand_traffic
    assert inputs.outer_lanes == (False, True) and math.isclose(inputs.yaw_rate, 0.3, rel_tol=1e-6)
    assert not nav_inputs(sm, model_msg, False).long_active  # cruise button control reports longActive without openpilot's speed control

  def test_capnp_lanes(self):
    instruction = log.NavInstruction.new_message(maneuverType='off ramp', maneuverModifier='slight right', maneuverDistance=50.)
    instruction.lanes = [{'directions': ['straight'], 'active': False, 'activeDirection': 'none'},
                         {'directions': ['slightRight'], 'active': True, 'activeDirection': 'slightRight'}]
    assert active_lane(instruction.lanes) == ('slightRight', 1)
    c = controller()
    c.update(car(vEgo=25., **nudge()), True, NavInputs(instruction))
    assert c.keep_desire == log.Desire.keepRight


class TestRouteTurnTiming(OpenpilotTestCase):
  @parameterized.expand([(2.,), (4.,), (NAV_TURN_STOCK_NEAR_SPEED,)])
  def test_pulses_once_3_s_before_steering(self, v_ego):
    # the driver holds the speed, so nothing slows the car: it pulses 3 s ahead at that speed
    edges = pulses(drive(controller(), 120., v_ego, 140. / v_ego, carstate=car(vEgo=v_ego, rightBlinker=True)))
    assert len(edges) == 1  # one pulse through the whole approach and turn, which ends 20 m past the maneuver point
    assert trigger_distance(v_ego) - v_ego * DT_MDL - 1e-6 <= edges[0] <= trigger_distance(v_ego)

  @parameterized.expand([(8.9,), (13.4,), (17.9,)])
  def test_pulses_3_s_before_steering_while_slowing(self, v_set):
    # openpilot slows the car for the turn the driver signals 8 s ahead: the pulse is timed on the speed it will have, so
    # the car reaches the steering point about 3 s later, not nearly 4 s
    frames = closed_loop(controller(), 200., v_set, 40., carstate=lambda d, v: car(vEgo=v, rightBlinker=d <= 8. * v_set))
    edges = [i for i, (output, prev) in enumerate(zip([f[0] for f in frames], [None] + [f[0] for f in frames[:-1]], strict=True))
             if output in TURN_DIRECTIONS and output != prev]
    steer = next(i for i, frame in enumerate(frames) if frame[1] <= NAV_TURN_STEER_DISTANCE)
    assert len(edges) == 1 and 2.8 <= (steer - edges[0]) * DT_MDL <= 3.3

  def test_too_fast_for_the_turn(self):
    assert not pulses(drive(controller(), 120., 9.5, 12., carstate=car(vEgo=9.5, rightBlinker=True)))

  @parameterized.expand([
    ('right', 8.05, 0),  # Tesla's own cruise control cannot go below 18 mph without a car ahead
    ('right', NAV_TURN_STOCK_NEAR_SPEED, 1), ('right', NAV_TURN_STOCK_NEAR_SPEED + 0.1, 0),
    ('left', NAV_TURN_STOCK_FAR_SPEED, 1), ('left', NAV_TURN_STOCK_FAR_SPEED + 0.1, 0),  # a far side turn is wider
  ])
  def test_without_openpilot_speed_control_only_at_turning_speed(self, modifier, v_ego, expected):
    # nothing slows the car for the turn, so the blinker turn starts only at a speed it can be made at
    carstate = car(vEgo=v_ego, leftBlinker=modifier == 'left', rightBlinker=modifier == 'right')
    assert len(pulses(drive(controller(), 120., v_ego, 120. / v_ego, carstate=carstate, modifier=modifier))) == expected

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

  def test_frozen_gps_distance(self):
    # navd's distance stuck, but fresh, while the car drives on. Far from the turn it holds the approach back: navd never
    # agrees the turn is near, so none starts. Right at the turn it may still start, once
    for frozen, expected in ((60., 0), (30., 1)):
      c = controller()
      frames = [(c.update(car(vEgo=5., rightBlinker=True), True, nav(distance=frozen)), c.turn_direction)[1] for _ in range(400)]
      edges = [i for i, (prev, cur) in enumerate(zip([None, *frames[:-1]], frames, strict=True)) if cur in TURN_DIRECTIONS and cur != prev]
      assert len(edges) == expected

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
    c = controller()
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


class TestLateStarts(OpenpilotTestCase):
  """A turn starts only while the car can still make it. Whatever held its start back, a car that reaches the steering
  point still moving, or too fast to be down to turn speed by it, gets no turn desire: it goes straight."""

  @parameterized.expand([(11.2,), (13.4,), (15.6,)])
  def test_late_signal(self, v_set):
    # openpilot slows the car from the signal on. Signaled late, it either still gets down to turn speed by the steering
    # point and turns, or gets no turn desire at all: never one at 20 mph a few meters from the corner
    for signal_at in range(25, 115, 5):
      frames = closed_loop(controller(), 150., v_set, 30., carstate=lambda d, v, signal_at=signal_at: car(vEgo=v, rightBlinker=d <= signal_at))
      edges = edge_frames(frames)
      at_steering_point = next(v for _, d, v, _ in frames if d <= NAV_TURN_STEER_DISTANCE)
      assert len(edges) <= 1 and all(d > NAV_TURN_STEER_DISTANCE for _, d, _, _ in edges)
      assert not edges or at_steering_point <= NAV_TURN_SPEED + 0.5, (signal_at, edges, at_steering_point)
    for signal_at in (2.5 * v_set, 3.5 * v_set, 4.5 * v_set):  # 2.5-4.5 s ahead: too late
      assert not pulses(closed_loop(controller(), 150., v_set, 30., carstate=lambda d, v, at=signal_at: car(vEgo=v, rightBlinker=d <= at)))

  @parameterized.expand([(6., 0), (15., 1)])
  def test_blind_spot_clearing(self, clears_at, expected):
    c = controller()
    frames = []
    for i in range(round(12. / DT_MDL)):
      d = 40. - 4.5 * DT_MDL * i
      c.update(car(vEgo=4.5, rightBlinker=True, rightBlindspot=clears_at < d <= 30.), True, nav(distance=d))
      frames.append((c.turn_direction, d))
    edges = pulses(frames)
    assert len(edges) == expected and all(d > NAV_TURN_STEER_DISTANCE for d in edges)  # a cyclist cleared inside it: no right hook

  @parameterized.expand([(5., 0), (12., 1)])
  def test_re_engaging_rolling(self, at, expected):
    # the brake drops steering on this Tesla from 30 m; re-engaged rolling at 4 m/s with the blinker still on
    c = controller()
    frames = []
    for i in range(round(15. / DT_MDL)):
      d = 45. - 4. * DT_MDL * i
      c.update(car(vEgo=4., rightBlinker=True), not at < d <= 30., nav(distance=d))
      frames.append((c.turn_direction, d))
    edges = pulses(frames)
    assert len(edges) == expected and all(d > NAV_TURN_STEER_DISTANCE for d in edges)

  @parameterized.expand([(9., 0), (16., 1)])
  def test_curb_lane_seen_late(self, seen_from, expected):
    # a parking lane ends at a bulb-out: the model sees the curb lane only from there, for the second the automatic turn needs
    frames = drive(controller(turns=False, auto=True), 40., 4.5, 12., inputs=lambda distance, **kw: auto_nav(
      distance=distance, outer_lanes=(False, distance <= seen_from)))
    edges = pulses(frames)
    assert len(edges) == expected and all(d > NAV_TURN_STEER_DISTANCE for d in edges)

  @parameterized.expand([(6., 0), (25., 1)])
  def test_lane_change_ending_late(self, ends_at, expected):
    c = controller()
    frames = []
    for i in range(round(15. / DT_MDL)):
      d = 60. - 4.5 * DT_MDL * i
      c.update(car(vEgo=4.5, rightBlinker=True), True, nav(distance=d), lane_changing=ends_at < d <= 50.)
      frames.append((c.turn_direction, d))
    edges = pulses(frames)
    assert len(edges) == expected and all(d > NAV_TURN_STEER_DISTANCE for d in edges)

  @parameterized.expand([(15.,), (20.,), (25.,)])
  def test_gps_error_behind_ending_close(self, ends_at):
    # the GPS reads 25 m behind the car from 100 m out, taken while the turn is far, and right again only close to it: the
    # dead reckoned distance is then 25 m long, so the turn may only start on navd's nearer one, not late inside the corner
    for c, carstate, inputs in ((controller(), car(vEgo=5., rightBlinker=True), nav), (controller(turns=False, auto=True), car(vEgo=5.), auto_nav)):
      frames = closed_loop(c, 150., 5., 40., carstate=lambda d, v, cs=carstate: cs, inputs=lambda distance, inputs=inputs, **kw: inputs(
        distance=distance + (25. if ends_at < distance <= 100. else 0.)))
      assert all(d > NAV_TURN_STEER_DISTANCE for d in pulses(frames))

  @parameterized.expand([(55., 0), (90., 1)])
  def test_automatic_turn_published_late_at_speed(self, published_at, expected):
    # the route's turn first comes up late (a reroute, or the route set late) at 30 mph
    frames = closed_loop(controller(turns=False, auto=True), 150., 13.4, 30., inputs=lambda distance, **kw: auto_nav(
      distance=distance) if distance <= published_at else NavInputs(long_active=True, outer_lanes=(False, True)))
    assert len(pulses(frames)) == expected

  def test_stopped_at_the_line_still_pulses(self):
    # signaling while stopped at a stop line 6 m before the maneuver point: the car is lined up, and pulls away into the turn
    for c, carstate, inputs in ((controller(), car(vEgo=0., standstill=True, rightBlinker=True), nav),
                                (controller(turns=False, auto=True), car(vEgo=0., standstill=True), auto_nav)):
      assert len(pulses(drive(c, 6., 0., 2., carstate=carstate, inputs=inputs))) == 1


def edge_frames(frames, values=TURN_DIRECTIONS + TURN_DESIRES):
  """The frames where the output rises into one of the values."""
  return [frame for frame, prev in zip(frames, [None] + [f[0] for f in frames[:-1]], strict=True) if frame[0] in values and frame[0] != prev]


class TestKeepAlive(OpenpilotTestCase):
  """A started turn pulses again only to stay in the model's 5 s desire history: while the car waits lined up at a crawl,
  or while it is still at least 2 s from the steering point. Never as it rolls into the turn, nor once it has turned in."""

  @parameterized.expand([
    (False, 8.9, None), (False, 13.4, None), (True, 13.4, None), (True, 17.9, None),
    (False, 13.4, (3.5, 15.)), (True, 13.4, (4.0, 15.)), (False, 11.2, (3.0, 15.)), (True, 11.2, (2.5, 15.)),  # a model slowing more
    (False, 13.4, (2.0, 21.)), (True, 13.4, (2.0, 21.)),  # a slow car ahead from just after the pulse
  ])
  def test_one_turn_command(self, auto, v_set, slower):
    # the slowdown fed back into the car's speed: one pulse about 3 s ahead, and none as the car rolls into the turn, so
    # once the car is past the corner the model's history holds at most the one turn command
    signal_at = 8. * v_set if not auto else -100.
    model = (lambda d: slower[0] if d <= slower[1] else 99.) if slower is not None else None
    frames = closed_loop(controller(turns=not auto, auto=auto), 200., v_set, 50., model_speed=model,
                         carstate=lambda d, v: car(vEgo=v, rightBlinker=d <= signal_at))
    moving = [frame for frame in edge_frames(frames) if frame[2] > NAV_TURN_CRAWL_SPEED]
    assert len(moving) == 1 and moving[0][1] > NAV_TURN_STEER_DISTANCE
    assert all(len(in_history(frames, d)) <= 1 for d in (0., -5., -10.))

  @parameterized.expand([(2., False), (4., False), (6., False), (4., True)])
  def test_stopped_after_turning_in(self, stop_at, brake):
    # the car turns 40 deg into the turn, then stops 2-6 m short of the maneuver point (a pedestrian on the crosswalk, a car
    # stopped ahead): no new turn command, with steering held, nor after a brake (steering off) and re-engaging stopped
    c = controller()
    blinker = {'rightBlinker': True}
    frames = drive(c, 40., 4.5, 32. / 4.5, carstate=car(vEgo=4.5, **blinker))
    seconds = (8. - stop_at) / 3.
    frames += drive(c, 8., 3., seconds, carstate=car(vEgo=3., **blinker), yaw_rate=-math.radians(40.) / seconds)
    assert c.turned > NAV_TURN_LINED_UP and len(pulses(frames)) == 1
    stopped = car(vEgo=0., standstill=True, **blinker)
    if brake:
      frames += drive(c, stop_at, 0., 2., carstate=stopped, lateral_active=False)
    frames += drive(c, stop_at, 0., 10., carstate=stopped)
    frames += drive(c, stop_at, 2., 2., carstate=car(vEgo=2., **blinker), yaw_rate=-0.5)
    assert len(pulses(frames)) == 1

  def test_left_turn_gap_wait(self):
    # waiting in the intersection to turn left across traffic, lined up but for a few degrees: the turn desire is refreshed
    # every 4 s, so the model still has it when the gap comes
    c = controller()
    blinker = {'leftBlinker': True}
    frames = drive(c, 40., 4.5, 32. / 4.5, carstate=car(vEgo=4.5, **blinker), modifier='left')
    frames += drive(c, 8., 2., 1.5, carstate=car(vEgo=2., **blinker), modifier='left', yaw_rate=math.radians(10.) / 1.5)
    waiting = drive(c, 5., 0., 13., carstate=car(vEgo=0., standstill=True, **blinker), modifier='left')
    assert c.lined_up and len(pulses(frames)) == 1
    starts = [i for i, (output, prev) in enumerate(zip([w[0] for w in waiting], [frames[-1][0]] + [w[0] for w in waiting[:-1]], strict=True))
              if output in TURN_DIRECTIONS and output != prev]
    assert len(starts) >= 3 and all(abs((b - a) * DT_MDL - NAV_TURN_REPULSE_TIME) < 0.1 for a, b in zip(starts, starts[1:], strict=False))

  def test_no_refresh_off_the_route(self):
    # navd's instruction gone for longer than its grace: the turn desire holds, but no fresh pulse goes out for it
    c = controller()
    blinker = {'rightBlinker': True}
    drive(c, 60., 5., 9.6, carstate=car(vEgo=5., **blinker))  # to 12 m: the pulse at 23 m
    for _ in range(round(10. / DT_MDL)):
      c.update(car(vEgo=0., standstill=True, **blinker), True, NavInputs())
      assert c.turn_direction == TurnDirection.turnRight or c.gap <= NAV_INSTRUCTION_GRACE


class TestStaleBlinker(OpenpilotTestCase):
  """A blinker on since before the car passed the last maneuver was for that one, whatever side; one switched on after it,
  even before navd moves on to the next maneuver, is the next turn's signal."""

  @staticmethod
  def two_turns(c, blinkers, first=('turn', 'right'), second='left', speed=lambda d: 5.):
    """Turn at A, then `second` at B 90 m later; navd moves on to B once the car is 12 m past A, as it does once the car
    drives along the next step. blinkers(d) and speed(d) give the (left, right) blinkers and the speed d m before A
    (negative past it). Returns each frame's (output, distance to B)."""
    frames, d = [], 40.
    while d > -100.:
      left, right = blinkers(d)
      v = speed(d)
      if d > -12.:
        inputs = nav(*first, distance=d, position=1100.)
      else:
        inputs = nav(modifier=second, distance=90. + d, position=1010.)
      c.update(car(vEgo=v, standstill=v == 0., leftBlinker=left, rightBlinker=right), True, inputs)
      frames.append((c.turn_direction, 90. + d))
      d -= max(v, 0.05) * DT_MDL
    return frames

  def test_other_blinker_before_navd_moves_on(self):
    # the right blinker cancels at the end of the right turn, and the left one comes on 3 m later, before navd moves on
    frames = self.two_turns(controller(), lambda d: (d < -8., d > -5.))
    b = [d for d in pulses(frames) if d < 90.]
    assert len(b) == 1 and b[0] > NAV_TURN_STEER_DISTANCE

  def test_same_blinker_again_before_navd_moves_on(self):
    # the right blinker cancels after the right turn and is switched on again for the next right turn
    frames = self.two_turns(controller(), lambda d: (False, d > -5. or d < -8.), second='right')
    assert len([d for d in pulses(frames) if d < 90.]) == 1

  def test_new_blinker_while_stopped_past_the_turn(self):
    # stopped in a queue 3 m past the turn (navd has not moved on), the driver signals left for the next turn
    stop = {'t': 0.}

    def speed(d):
      if -3. >= d and stop['t'] < 8.:
        stop['t'] += DT_MDL
        return 0.
      return 5.
    frames = self.two_turns(controller(), lambda d: (d <= -3. and stop['t'] > 2., d > 0.), speed=speed)
    assert len([d for d in pulses(frames) if d < 90.]) == 1

  def test_blinker_left_on_through_an_exit(self):
    # an exit never latches the blinker, but one left on from before it is still not the next same-side turn's signal
    frames = self.two_turns(controller(), lambda d: (False, True), first=('off ramp', 'slight right'), second='right')
    assert not pulses(frames)
    # switched off and on again after the exit, it is
    frames = self.two_turns(controller(), lambda d: (False, not -20. < d <= -18.), first=('off ramp', 'slight right'), second='right')
    assert len(pulses(frames)) == 1


class TestAutoTurns(OpenpilotTestCase):
  def test_auto_turn(self):
    # openpilot slows the car for an automatic turn, so it pulses 3 s ahead at the speed the slowdown will impose
    edges = pulses(drive(controller(turns=False, auto=True), 60., 5., 16., inputs=auto_nav))
    assert len(edges) == 1 and abs(edges[0] - slowed_trigger_distance(5.)) <= 5. * DT_MDL

  def test_lane_change_into_the_curb_lane(self):
    # signaling for the turn and changing into the curb lane, the blinker off after it: the automatic turn still comes,
    # once, but only after the model has seen the curb lane for a second after the lane change (it sees it during it)
    c = controller(auto=True)
    frames, ended = [], None
    for i in range(round(12. / DT_MDL)):
      d = 50. - 4.5 * DT_MDL * i
      lane_changing = 23. < d <= 40.
      c.update(car(vEgo=4.5, rightBlinker=23. < d <= 45.), True, auto_nav(distance=d, outer_lanes=(False, d <= 30.)), lane_changing=lane_changing)
      frames.append((c.turn_direction, d))
      ended = d if lane_changing else ended
    edges = pulses(frames)
    assert len(edges) == 1 and NAV_TURN_STEER_DISTANCE < edges[0] <= ended - 4.5 * OUTER_LANE_TIME + 1e-6

  @parameterized.expand([
    ('auto turns off', {}, {}, False),
    ("gas pressed, or the car's own cruise control", {}, {'long_active': False}, True),
    ('not slowed down yet', {'vEgo': 8.}, {}, True),
    ('not in the rightmost lane', {}, {'outer_lanes': (False, False)}, True),
    ('blind spot', {'rightBlindspot': True}, {}, True),
    ('the other blinker', {'leftBlinker': True}, {}, True),
    ('the gas', {'gasPressed': True}, {}, True),
    ('hazards', {'leftBlinker': True, 'rightBlinker': True}, {}, True),
  ])
  def test_auto_turn_blocked(self, _name, car_state, inputs, auto):
    carstate = car(**({'vEgo': 5.} | car_state))
    assert not pulses(drive(controller(turns=False, auto=auto), 60., carstate.vEgo, 10., inputs=auto_nav, carstate=carstate, **inputs))

  def test_far_side_turn_waits_for_the_blinker(self):
    c = controller(auto=True)
    waiting = drive(c, 60., 5., 9., inputs=auto_nav, modifier='left', outer_lanes=(True, True))
    assert not pulses(waiting) and c.speed_limit > 0.  # it slows, but crosses oncoming traffic only once signaled
    signaled = drive(c, 15., 5., 1., inputs=auto_nav, modifier='left', carstate=car(vEgo=5., leftBlinker=True))
    assert len(pulses(signaled, before=waiting[-1][0])) == 1

  @parameterized.expand([('left', (True, False), 1), ('left', (False, False), 0), ('right', (True, True), 0)])
  def test_left_hand_traffic(self, modifier, outer, expected):
    frames = drive(controller(auto=True), 60., 5., 10., inputs=auto_nav, modifier=modifier, outer_lanes=outer, right_hand_traffic=False)
    assert len(pulses(frames)) == expected

  def test_outermost_lane_for_a_second(self):
    c = controller(auto=True)
    seen_at = trigger_distance(5.) + 3.  # the model sees the outermost lane from 0.6 s before the trigger point
    unseen = drive(c, 60., 5., (60. - seen_at) / 5., inputs=auto_nav, outer_lanes=(False, False))
    seen = drive(c, seen_at, 5., 3., inputs=auto_nav)
    edges = pulses(seen, before=unseen[-1][0])
    assert not pulses(unseen) and len(edges) == 1 and edges[0] <= seen_at - 5. * (OUTER_LANE_TIME - DT_MDL) + 1e-6

  def test_gas_calls_off_for_good(self):
    c = controller(auto=True)
    approach = drive(c, 120., 9., 2., inputs=auto_nav)  # too fast to turn yet
    gas = drive(c, 102., 9., 0.5, inputs=auto_nav, carstate=car(vEgo=9., gasPressed=True), long_active=False)
    slowed = drive(c, 97.5, 4., 20., inputs=auto_nav)  # then a lead car or a red light slows it to turning speed
    assert not pulses(approach + gas + slowed) and c.speed_limit == 0.
    signaled = drive(c, 17.5, 4., 1., inputs=auto_nav, carstate=car(vEgo=4., rightBlinker=True))  # signaling brings it back
    assert len(pulses(signaled, before=slowed[-1][0])) == 1 and c.speed_limit > 0.

  def test_other_blinker_calls_off_for_good(self):
    c = controller(auto=True)
    signal = drive(c, 120., 9., 1., inputs=auto_nav, carstate=car(vEgo=9., leftBlinker=True))  # a lane change the other way
    after = drive(c, 111., 4., 24., inputs=auto_nav)
    assert not pulses(signal + after) and c.speed_limit == 0.

  def test_steering_off_calls_off_only_the_automatic_turn(self):
    c = controller(auto=True)
    approach = drive(c, 60., 5., 2., inputs=auto_nav)
    off = drive(c, 50., 5., 0.5, inputs=auto_nav, lateral_active=False)
    on = drive(c, 47.5, 5., 6., inputs=auto_nav)
    assert not pulses(approach + off + on) and c.speed_limit > 0.  # it still slows for the turn, which the driver takes

  def test_gps_jump_does_not_restart_a_turn(self):
    c = controller(auto=True)
    frames = drive(c, 40., 5., 3.6, inputs=auto_nav)  # the turn started at 23 m
    frames += drive(c, 57., 5., 1., inputs=auto_nav)  # navd's distance jumps back 35 m for a second
    frames += drive(c, 17., 5., 3., inputs=auto_nav)
    assert len(pulses(frames)) == 1

  def test_same_kind_of_turn_right_after(self):
    c = controller(auto=True)
    first = drive(c, 40., 5., 10., inputs=auto_nav, position=1100.)  # through the turn to 10 m past it
    second = drive(c, 30., 5., 5., inputs=auto_nav, position=1060.)  # navd moves on: another right turn 40 m on
    assert len(pulses(first)) == len(pulses(second, before=first[-1][0])) == 1

  def test_last_turns_blinker_is_not_the_next_turns_signal(self):
    c = controller()
    blinker = car(vEgo=5., rightBlinker=True)
    first = drive(c, 40., 5., 10., carstate=blinker, position=1100.)
    second = drive(c, 30., 5., 3., carstate=blinker, position=1060.)  # still on from the turn just taken
    assert len(pulses(first)) == 1 and not pulses(second, before=first[-1][0]) and not c.route_turn_signal
    off = drive(c, 15., 5., 0.2, carstate=car(vEgo=5.), position=1060.)
    again = drive(c, 14., 5., 1., carstate=blinker, position=1060.)  # off and on again: the signal for this turn
    assert len(pulses(again, before=off[-1][0])) == 1

  def test_re_engaging_after_taking_the_turn_by_hand(self):
    c = controller(auto=True)
    frames = drive(c, 60., 5., 2., inputs=auto_nav)
    frames += drive(c, 50., 5., 12., inputs=auto_nav, lateral_active=False, long_active=False)  # the brake: all off, on this Tesla
    for _ in range(100):  # back on along the new road, while navd still reports the turn a meter or two ahead
      c.update(car(vEgo=5.), True, auto_nav(distance=1.5))
      frames.append((c.turn_direction, 1.5))
    assert not pulses(frames) and c.speed_limit == 0.

  def test_re_engaging_at_a_stop_with_the_blinker_on(self):
    c = controller()
    blinker = {'rightBlinker': True}
    approach = drive(c, 60., 5., 8.6, carstate=car(vEgo=5., **blinker))  # the turn started at 23 m
    braked = drive(c, 17., 0., 3., carstate=car(vEgo=0., standstill=True, **blinker), lateral_active=False)  # braked to a stop
    resumed = drive(c, 17., 0., 1., carstate=car(vEgo=0., standstill=True, **blinker))  # steering back on, still signaling
    pulling_away = drive(c, 17., 3., 2., carstate=car(vEgo=3., **blinker))
    assert len(pulses(approach)) == 1 and len(pulses(resumed + pulling_away, before=braked[-1][0])) == 1

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
    first = drive(c, 40., 5., 10., inputs=auto_nav, position=1300.)
    second = drive(c, 300., 5., 58., inputs=auto_nav, modifier='sharp right', position=1000.)  # navd moved on to the next turn
    third = drive(c, 200., 5., 38., inputs=auto_nav, modifier='sharp right', position=700.)  # the same kind of turn, after a reroute
    assert len(pulses(first)) == len(pulses(second, before=first[-1][0])) == len(pulses(third, before=second[-1][0])) == 1


class TestNavSpeedLimit(OpenpilotTestCase):
  def test_auto_slows_for_every_turn_and_holds_through_it(self):
    c = controller(turns=False, auto=True)
    frames = drive_limits(c, 200., NAV_TURN_SPEED, 230. / NAV_TURN_SPEED, modifier='left')  # waits for the blinker to turn left
    for limit, distance in frames:
      if distance > NAV_TURN_MARGIN:
        assert math.isclose(limit, turn_speed_limit(distance), rel_tol=1e-3)
      elif distance > -NAV_TURN_HOLD + 0.5:
        assert limit == NAV_TURN_SPEED
      elif distance < -NAV_TURN_HOLD - 0.5:
        assert limit == 0.

  def test_blinker_turns_slow_once_signaled(self):
    c = controller()
    assert all(limit == 0. for limit, _ in drive_limits(c, 200., 15., 1., inputs=nav))
    signaled = drive_limits(c, 185., 15., 1., inputs=nav, carstate=car(vEgo=15., rightBlinker=True))  # within 15 m + 12 s
    assert all(math.isclose(limit, turn_speed_limit(distance), rel_tol=1e-3) for limit, distance in signaled)

  def test_started_turn_keeps_its_speed(self):
    c = controller(auto=True)
    drive(c, 40., 5., 3.8, inputs=auto_nav)  # the automatic turn started at 22 m
    assert c.started
    gas = drive_limits(c, 21., 5., 1., carstate=car(vEgo=5., gasPressed=True), long_active=False)
    after = drive_limits(c, 16., 5., 5.)
    assert all(limit > 0. for limit, _ in gas + after)  # the model has the turn: the car stays at turning speed through it
    assert c.turn_direction == TurnDirection.none  # though the desire's reminders end

  def test_lane_change_keeps_the_slowdown(self):
    # a lane change with the turn's blinker goes toward the turn (the other blinker calls the turn off), so the slowdown
    # keeps going through it, rather than letting the car speed back up into a turn it can no longer slow for
    c = controller()
    blinker = car(vEgo=15., rightBlinker=True)
    drive_limits(c, 190., 15., 1., inputs=nav, carstate=blinker)
    frames = drive_limits(c, 175., 15., 4., inputs=lambda distance, **kw: nav(distance=distance), carstate=car(vEgo=15., rightBlinker=True, **nudge()))
    c.update(car(vEgo=15., rightBlinker=True), True, nav(distance=115.), lane_changing=True)
    assert all(math.isclose(limit, turn_speed_limit(distance), rel_tol=1e-3) for limit, distance in frames)
    assert math.isclose(c.speed_limit, turn_speed_limit(115.), rel_tol=1e-3)

  def test_lane_change_into_the_turn_lane(self):
    # signal 8 s ahead at 35 mph, then a nudged lane change into the turn lane over the next 50 m: the car keeps slowing
    # through it, and the turn pulses about where it would have without it
    v_set, signal_at = 15.6, 125.

    def run(lane_change):
      def carstate(d, v):
        return car(vEgo=v, rightBlinker=d <= signal_at, **(nudge() if lane_change and signal_at - 10. >= d > signal_at - 15. else {}))
      return closed_loop(desire_helper(), 200., v_set, 40., inputs=nav, carstate=carstate, long_active=True,
                         lane_change_prob=lambda d: 1. if d > signal_at - 60. else 0.)

    frames, baseline = run(True), run(False)
    edges = pulses(frames, TURN_DESIRES)
    changing = [v for desire, _, v, _ in frames if desire == log.Desire.laneChangeRight]
    assert len(changing) > round(2. / DT_MDL) and all(later <= earlier + 1e-6 for earlier, later in zip(changing, changing[1:], strict=False))
    assert len(edges) == 1 and abs(edges[0] - pulses(baseline, TURN_DESIRES)[0]) < 3.
    assert all(v <= NAV_TURN_SPEED + 0.6 for _, d, v, _ in frames if 0. <= d <= NAV_TURN_STEER_DISTANCE)

  def test_started_turn_through_a_route_gap(self):
    # the turn's pulse went out, the car stops 21 m out behind a queue, and navd goes quiet for 10 s (no fix, or a stall):
    # the turn's slowdown keeps going on the distance driven until the car is through the turn
    c = controller(auto=True)
    drive(c, 40., 5., 3.8, inputs=auto_nav)
    assert c.started

    def quiet(distance, **kwargs):
      return NavInputs(long_active=True, outer_lanes=(False, True))
    stopped = drive_limits(c, 21., 0., 10., inputs=quiet, carstate=car(vEgo=0., standstill=True))
    pulling_away = drive_limits(c, 21., 4.5, 12., inputs=quiet, carstate=car(vEgo=4.5))
    assert all(limit > 0. for limit, _ in stopped) and c.gap > NAV_INSTRUCTION_GRACE
    assert all(limit > 0. for limit, d in pulling_away if d > -NAV_TURN_HOLD + 0.5)
    assert all(limit == 0. for limit, d in pulling_away if d < -NAV_TURN_HOLD - 0.5)

  def test_navd_moving_on_early_at_a_sharp_corner(self):
    # navd moves on to the next maneuver 25 m before a sharp corner, after its turn's pulse: the pinned turn point keeps its
    # profile, and turn speed to 20 m past the corner, and no other turn starts inside it, even signaled for
    c = controller()
    approach = drive(c, 60., 6.5, 35. / 6.5, carstate=car(vEgo=6.5, leftBlinker=True), modifier='sharp left', position=1200.)
    assert len(pulses(approach)) == 1 and c.started
    corner = c.odometer + approach[-1][1]  # on the odometer
    assert corner - c.odometer > 25.
    frames = []
    for i in range(round(70. / 6.5 / DT_MDL)):
      blinker = not 0.5 < i * DT_MDL <= 0.7  # off and on again: the signal for the next turn
      c.update(car(vEgo=6.5, leftBlinker=blinker), True, nav(modifier='left', distance=60. - 6.5 * DT_MDL * i, position=1100.))
      frames.append((c.turn_direction, corner - c.odometer, c.speed_limit))
    inside = [frame for frame in frames if frame[1] >= -NAV_TURN_HOLD]
    assert not pulses(inside) and len(pulses(frames)) == 1  # the next turn starts once the car is through this one
    assert all(limit <= turn_speed_limit(d) + 1e-6 for _, d, limit in inside)
    assert all(limit == NAV_TURN_SPEED for _, d, limit in inside if d < NAV_TURN_MARGIN)

  def test_left_the_route(self):
    # the car leaves the route 100 m before the turn: navd's distance holds where it left while the car drives away. navd's
    # longer distance takes over, so the slowdown lets go and nothing starts, where dead reckoning alone would have slowed
    # the car to turn speed and turned it into the side street
    c = controller(auto=True)
    frames = []
    for i in range(round(12. / DT_MDL)):
      true = 150. - 11. * DT_MDL * i
      if i % round(1. / DT_MDL) == 0:
        message = auto_nav(distance=max(true, 100.))
      c.update(car(vEgo=11.), True, dataclasses.replace(message, age=(i % round(1. / DT_MDL)) * DT_MDL))
      frames.append((c.turn_direction, true, c.speed_limit))
    assert frames[-1][1] < 20. and not pulses(frames)
    assert all(limit >= turn_speed_limit(NAV_NEAR_DISTANCE) for _, true, limit in frames if true < 100.)

  def test_speed_reading_high_over_a_long_step(self):
    # vEgo 1 % high over a 3 km step: navd pushes the dead reckoned distance back out, so the automatic turn pulses within
    # a few meters of where it would without the error, and its turn speed holds to about 20 m past the corner
    def run(scale):
      return closed_loop(controller(turns=False, auto=True), 3000., 15., 3000. / 15. + 15., carstate=lambda d, v: car(vEgo=v * scale))
    nominal, high = run(1.), run(1.01)
    edges = pulses(high)
    assert len(edges) == 1 and abs(edges[0] - pulses(nominal)[0]) < NAV_PUSH_DEADBAND + 1.
    assert min(d for _, d, _, limit in high if limit > 0.) < -NAV_TURN_HOLD + 4.

  def test_gps_error_ahead_on_the_approach(self):
    # the GPS reads 25 m ahead for 4 s some 120 m before the turn, then right again: the blinker turn pulses where it would
    # have without the error, instead of 25 m early with its desire gone before the corner
    def run(error):
      def reported(distance, **kwargs):
        return nav(distance=distance - (25. if error and 125. >= distance > 105. else 0.))
      return closed_loop(desire_helper(), 200., 5., 45., inputs=reported, carstate=lambda d, v: car(vEgo=v, rightBlinker=d <= 70.))
    edges, nominal = pulses(run(True), TURN_DESIRES), pulses(run(False), TURN_DESIRES)
    assert len(edges) == len(nominal) == 1 and abs(edges[0] - nominal[0]) < 1.

  def test_route_gap(self):
    c = controller(auto=True)
    drive_limits(c, 200., 10., 1.)
    grace = round(NAV_INSTRUCTION_GRACE / DT_MDL)
    limits = []
    for _ in range(grace + 20):  # navd goes quiet: the slowdown keeps going on the distance driven, for a while
      c.update(car(vEgo=10.), True, NavInputs(long_active=True, outer_lanes=(False, True)))
      limits.append(c.speed_limit)
    assert all(limit > 0. for limit in limits[:grace - 1]) and all(limit == 0. for limit in limits[grace + 1:])

  def test_hold_outlasts_navd_moving_on(self):
    c = controller(auto=True)
    drive_limits(c, 30., NAV_TURN_SPEED, 40. / NAV_TURN_SPEED, position=1100.)  # to 10 m past the turn
    moved_on = drive_limits(c, 190., NAV_TURN_SPEED, 1., modifier='left', position=900.)  # navd moved on to the next maneuver
    later = drive_limits(c, 185.5, NAV_TURN_SPEED, 3., modifier='left', position=900.)  # until 20 m past the turn
    assert all(limit == NAV_TURN_SPEED for limit, _ in moved_on)
    assert math.isclose(later[-1][0], turn_speed_limit(later[-1][1]), rel_tol=1e-3)

  def test_frozen_distance_ends_the_slowdown(self):
    c = controller(auto=True)
    for _ in range(round(60. / DT_MDL)):  # navd's distance stuck at 10 m for a minute, while the car drives on
      c.update(car(vEgo=NAV_TURN_SPEED), True, auto_nav(distance=10.))
    assert c.speed_limit == 0.

  def test_disabled(self):
    c = controller(turns=False, exits=False)
    assert all(limit == 0. for limit, _ in drive_limits(c, 100., 10., 3., carstate=car(vEgo=10., rightBlinker=True)))


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
    for distance, expected in [(120., log.Desire.none), (90., log.Desire.keepRight)]:
      c = controller()
      c.update(car(vEgo=30., **nudge()), True, nav('off ramp', 'slight right', distance, lanes))
      assert c.keep_desire == expected

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
    # a turn only starts at turning speed now, so the car speeds up past 20 mph after it starts, with the wheel turned
    dh = desire_helper()
    frames = drive(dh, 40., 5., 3.6, carstate=car(vEgo=5., rightBlinker=True))  # the turn starts at 23 m
    frames += drive(dh, 22., 8.95, 1., carstate=car(vEgo=8.95, rightBlinker=True, **nudge()))  # steering into the turn
    frames += drive(dh, 13., 8.95, 3., carstate=car(vEgo=8.95, rightBlinker=True, rightBlindspot=True, **nudge()))  # a cyclist
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


  def test_no_lane_turn_either_way_during_a_route_turn(self):
    dh = desire_helper(turns=False, auto=True, lane_turns=True)
    frames = drive(dh, 40., 5., 3.8, inputs=auto_nav)  # the automatic right turn starts at 22 m
    frames += drive(dh, 21., 5., 6., inputs=auto_nav, carstate=car(vEgo=5., leftBlinker=True))  # the other blinker, too late
    assert pulses(frames) == pulses(frames, (log.Desire.turnRight,)) and len(pulses(frames)) == 1

  def test_no_pulse_at_or_past_the_corner(self):
    # a steering blip and a route gap on the approach, then navd still reporting the turn just ahead after it
    dh = desire_helper(lane_turns=True)
    blinker = car(vEgo=4., rightBlinker=True)
    frames = drive(dh, 60., 4., 11.25, carstate=blinker)  # the turn started at 20 m
    dh.update(blinker, False, 1.0, False, False, nav(distance=15.))  # steering drops for a frame
    frames.append((dh.desire, 15.))
    for i in range(70):  # navd goes quiet for 3.5 s
      dh.update(blinker, True, 1.0, False, False, NavInputs())
      frames.append((dh.desire, 15. - 4. * DT_MDL * (i + 1)))
    frames += drive(dh, 1., 4., 1., carstate=blinker)
    for _ in range(100):  # along the new road, still signaling
      dh.update(blinker, True, 1.0, False, False, nav(distance=1.5))
      frames.append((dh.desire, -10.))
    edges = pulses(frames, TURN_DESIRES)
    assert edges and all(distance > NAV_TURN_STEER_DISTANCE for distance in edges)


class TestRouteBlinkerHandOff(OpenpilotTestCase):
  """With Lane Turn Desire on, a blinker a route turn has had stays the route's until it goes off: the model gets one turn
  pulse for it, the route's, whatever navd, the steering or the controller do meanwhile. And it never starts a nudgeless
  lane change."""

  def test_steering_dropout_on_the_approach(self):
    # a brake tap turns steering off from 50 to 45 m: the route takes the blinker back and pulses at its trigger point
    dh = desire_helper(lane_turns=True)
    frames = drive(dh, 70., 5., 4., carstate=car(vEgo=5., rightBlinker=True))
    frames += drive(dh, 50., 5., 1., carstate=car(vEgo=5., rightBlinker=True), lateral_active=False)
    frames += drive(dh, 45., 5., 12., carstate=car(vEgo=5., rightBlinker=True))
    edges = pulses(frames, TURN_DESIRES)
    assert len(edges) == 1 and abs(edges[0] - trigger_distance(5.)) <= 5. * DT_MDL

  @parameterized.expand([('right', 'right'), ('left', 'left'), ('right', 'left')])
  def test_navd_moving_on_with_the_blinker_on(self, first, second):
    # the blinker still on as navd moves on to the next step, 15 m into the new road: no second pulse there
    dh = desire_helper(lane_turns=True)
    blinker = car(vEgo=5., leftBlinker=first == 'left', rightBlinker=first == 'right')
    frames = drive(dh, 60., 5., 15., carstate=blinker, modifier=first, position=1120.)  # to 15 m past the turn
    frames += drive(dh, 105., 5., 10., carstate=blinker, modifier=second, position=1000.)  # the next turn, 120 m on
    assert len(pulses(frames, TURN_DESIRES)) == 1

  def test_reroute_after_a_missed_turn(self):
    # too fast for the turn the driver signals for, the car goes straight; navd calls it missed, then reroutes
    dh = desire_helper(lane_turns=True)
    blinker = car(vEgo=8., rightBlinker=True)
    frames = drive(dh, 80., 8., 12., carstate=blinker)
    for _ in range(40):
      dh.update(blinker, True, 1.0, False, False, NavInputs())
      frames.append((dh.desire, -20.))
    frames += drive(dh, 250., 8., 5., carstate=car(vEgo=4., rightBlinker=True), position=600.)  # slowed below Lane Turn speed
    assert not pulses(frames, TURN_DESIRES)

  def test_steering_off_past_the_corner(self):
    # the brake 3 m past the corner turns steering off for a second, the blinker still on, then re-engaged
    dh = desire_helper(lane_turns=True)
    blinker = car(vEgo=5., rightBlinker=True)
    frames = drive(dh, 40., 5., 8.6, carstate=blinker)
    frames += drive(dh, -3., 2., 1., carstate=blinker, lateral_active=False)
    frames += drive(dh, -5., 4., 3., carstate=blinker)
    assert len(pulses(frames, TURN_DESIRES)) == 1

  def test_aborted_turn(self):
    # the driver takes over 15 m before the corner and goes straight on with the blinker on, re-engaging past it
    dh = desire_helper(lane_turns=True)
    blinker = car(vEgo=5., rightBlinker=True)
    frames = drive(dh, 40., 5., 5., carstate=blinker)
    frames += drive(dh, 15., 5., 5., carstate=blinker, lateral_active=False)
    frames += drive(dh, -10., 5., 3., carstate=blinker)
    assert len(pulses(frames, TURN_DESIRES)) == 1

  def test_controller_failure_after_the_pulse(self):
    class Unreadable:
      maneuverType, maneuverModifier, lanes = 'turn', 'right', []

      @property
      def maneuverDistance(self):
        raise ValueError("corrupt message")

    dh = desire_helper(lane_turns=True)
    blinker = car(vEgo=5., rightBlinker=True)
    frames = drive(dh, 40., 5., 4., carstate=blinker)
    for _ in range(60):
      dh.update(blinker, True, 1.0, False, False, NavInputs(Unreadable()))
      frames.append((dh.desire, 0.))
    assert dh.nav_desire_failed and len(pulses(frames, TURN_DESIRES)) == 1

  @parameterized.expand([(AutoLaneChangeMode.NUDGELESS,), (AutoLaneChangeMode.TWO_SECONDS,)])
  def test_no_nudgeless_lane_change_after_a_reroute(self, mode):
    # the latched blinker stays on past a missed turn at 25 mph, and navd reroutes: still only a nudge changes lanes
    dh = desire_helper(lane_change=mode)
    blinker = car(vEgo=11.2, rightBlinker=True)
    frames = drive(dh, 140., 11.2, 13., carstate=blinker)
    for _ in range(40):
      dh.update(blinker, True, 1.0, False, False, NavInputs())
      frames.append((dh.desire, -20.))
    frames += drive(dh, 250., 11.2, 4., carstate=blinker, position=600.)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert all(desire not in (log.Desire.laneChangeLeft, log.Desire.laneChangeRight) for desire, _ in frames)
    drive(dh, 205., 11.2, 0.2, carstate=car(vEgo=11.2, rightBlinker=True, **nudge()), position=600.)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_new_blinker_after_the_route_let_go(self):
    # switched off and on again, the blinker is the driver's again: a lane change blinker, nudgeless if set so
    dh = desire_helper(lane_change=AutoLaneChangeMode.NUDGELESS)
    drive(dh, 140., 11.2, 13., carstate=car(vEgo=11.2, rightBlinker=True))
    drive(dh, 250., 11.2, 1., carstate=car(vEgo=11.2), position=600.)
    drive(dh, 239., 11.2, 1., carstate=car(vEgo=11.2, rightBlinker=True), position=600.)
    assert dh.route_blinker_side is None and dh.lane_change_state == LaneChangeState.laneChangeStarting


def run_once(dh, carstate, inputs, frames=1):
  for _ in range(frames):
    dh.update(carstate, True, 1.0, False, False, inputs)
  return dh.desire
