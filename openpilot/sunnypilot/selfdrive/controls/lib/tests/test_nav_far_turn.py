"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Automatic far side route turns (left where traffic keeps right): where they start without the blinker, where they wait for
it and ask for it, and that every approach still gives the driving model at most one turn command.
"""
import dataclasses
import math
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.parameterized import parameterized
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import AUTO_FAR_LIMIT_DISTANCE, FAR_LANE_DISTANCE, FAR_PROMPT_DEBOUNCE, \
  FAR_SIGNAL_LEAD, FAR_TURN_MIN_LEAD, NAV_TURN_STEER_DISTANCE, NAV_TURN_TRIGGER_TIME, NavDesireController, NavInputs, far_lanes, \
  far_turn_lane, nav_inputs, posted_speed_limit, time_to_steer
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_desire import TURN_DESIRES, TURN_DIRECTIONS, FakeSubMaster, car_state, \
  closed_loop, in_history, nudge, pulses, slowed_trigger_distance

MPH = 0.44704  # m/s
MPH35, MPH45, MPH50 = 35 * MPH, 45 * MPH, 50 * MPH
KPH = 1 / 3.6  # m/s


def guidance(*lanes):
  """navd's lane guidance, left to right: each lane's directions, and the direction the route takes it in, if any."""
  return tuple({'directions': list(directions), 'active': active is not None, 'activeDirection': active or 'none'} for directions, active in lanes)


# a curbed left turn bay: the route's lane guidance has the leftmost lane for the turn
BAY = guidance((('left',), 'left'), (('straight',), None), (('straight', 'right'), None))


def far_instruction(maneuver_type='turn', modifier='left', distance=30., lanes=BAY, position=1000.):
  """A real navInstruction for a maneuver `position` meters before the destination, `distance` meters ahead."""
  msg = messaging.new_message('navInstruction').navInstruction
  msg.maneuverType, msg.maneuverModifier = maneuver_type, modifier
  msg.maneuverDistance, msg.distanceRemaining = distance, distance + position
  msg.lanes = list(lanes)
  return msg


def far_nav(maneuver_type='turn', modifier='left', distance=30., lanes=BAY, age=0., position=1000., **inputs):
  """A far side route turn with openpilot in control of the speed, a curb surely close beside the car on its left, on a
  road posted 35 mph."""
  inputs = {'long_active': True, 'outer_lanes': (True, False), 'far_lanes': (True, False), 'speed_limit': MPH35} | inputs
  return NavInputs(far_instruction(maneuver_type, modifier, distance, lanes, position), age, **inputs)


def far_controller(far=True, auto=True, turns=False):
  c = NavDesireController()
  c.turn_enabled, c.auto_enabled, c.far_enabled, c.exit_enabled = turns, auto, far, False
  c.param_read_counter = 1  # the settings above stand in for params, so skip the first param read
  return c


def far_helper(far=True, turns=True, lane_turns=False):
  """DesireHelper with route turns, automatic turns and, if far, automatic far side turns on, read from params."""
  params = Params()
  for key, value in [('NavTurnDesire', turns), ('NavExitDesire', False), ('NavAutoTurn', True), ('NavAutoTurnFarSide', far),
                     ('LaneTurnDesire', lane_turns)]:
    params.put_bool(key, value)
  params.put('LaneTurnValue', 20.)
  return DesireHelper()


def run(c, start, v, seconds, inputs=far_nav, carstate=None, lane_changing=None, lateral_active=True, **kwargs):
  """Drive toward the maneuver from `start` meters at v, with navd's distance fresh every frame. carstate gives the car's
  state, a real carState, or a function of the distance that does; lane_changing(distance) whether a lane change is under
  way. Returns each frame's (turn direction, distance, signal prompt, turn across, speed limit)."""
  frames, distance = [], start
  default = car_state(vEgo=v, standstill=v == 0.)
  for _ in range(round(seconds / DT_MDL)):
    carstate_now = carstate(distance) if callable(carstate) else (carstate if carstate is not None else default)
    changing = lane_changing(distance) if lane_changing is not None else False
    c.update(carstate_now, lateral_active, inputs(distance=distance, **kwargs), lane_changing=changing)
    frames.append((c.turn_direction, distance, c.signal_prompt, c.turn_across, c.speed_limit))
    distance -= v * DT_MDL
  return frames


def rises(values) -> int:
  """How many times a flag goes on."""
  return sum(1 for value, prev in zip(values, [False, *values[:-1]], strict=True) if value and not prev)


def model_message(edge_y=(-1.9, 6.), edge_stds=(0.3, 0.3), line_probs=(0.05, 0.9, 0.9, 0.05)):
  """A real modelV2 with the road edges `edge_y` meters to the car's side (negative left) and the lane line probabilities,
  left to right."""
  msg = messaging.new_message('modelV2').modelV2
  msg.roadEdgeStds, msg.laneLineProbs = list(edge_stds), list(line_probs)
  edges = msg.init('roadEdges', len(edge_y))
  for edge, y in zip(edges, edge_y, strict=True):
    edge.y = [y, y]
  return msg


def plan_message(limit=MPH35, valid=True):
  msg = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP
  msg.speedLimit.resolver.speedLimit = limit
  msg.speedLimit.resolver.speedLimitValid = valid
  return msg


def plan_submaster(plan, age=0.05, seen=True, valid=True):
  return FakeSubMaster(seen={'longitudinalPlanSP': seen}, valid={'longitudinalPlanSP': valid},
                       recv_time={'longitudinalPlanSP': time.monotonic() - age}, data={'longitudinalPlanSP': plan})


class TestFarInputs(OpenpilotTestCase):
  @parameterized.expand([
    ('a curb beside the car', {}, (True, False)),
    ('on the wrong side', {'edge_y': (1.9, 6.)}, (False, False)),
    ('under the car', {'edge_y': (-0.5, 6.)}, (False, False)),
    ('a bike lane or a wide gutter between', {'edge_y': (-2.7, 6.)}, (False, False)),
    ('a lane between', {'edge_y': (-3.9, 6.)}, (False, False)),
    ('unsure: enough to block a lane change, not to turn', {'edge_stds': (0.6, 0.3)}, (False, False)),
    ('no reading', {'edge_y': (math.nan, 6.)}, (False, False)),
    ('no uncertainty', {'edge_stds': (math.nan, 0.3)}, (False, False)),
    ('a negative uncertainty', {'edge_stds': (-0.1, 0.3)}, (False, False)),
    ('a lane line beyond the car\'s own', {'line_probs': (0.3, 0.9, 0.9, 0.05)}, (False, False)),
    ('the right side, where traffic keeps left', {'edge_y': (-6., 1.9)}, (False, True)),
    ('the right side, with a lane line beyond', {'edge_y': (-6., 1.9), 'line_probs': (0.05, 0.9, 0.9, 0.3)}, (False, False)),
    ('a one lane road', {'edge_y': (-1.8, 1.8)}, (True, True)),
  ])
  def test_far_lanes(self, _name, fields, expected):
    assert far_lanes(model_message(**fields)) == expected

  def test_far_lanes_malformed(self):
    assert far_lanes(messaging.new_message('modelV2').modelV2) == (False, False)  # no model output yet
    assert far_lanes(model_message(edge_y=(-1.9,))) == (False, False)
    assert far_lanes(model_message(line_probs=(0.05, 0.9))) == (False, False)
    msg = model_message()
    msg.roadEdges[0].y = []
    assert far_lanes(msg) == (False, False)

  @parameterized.expand([
    ('no lane guidance', (), True, True),
    ('the bay', BAY, True, True),
    ('a left lane that also goes straight', guidance((('left', 'straight'), None), (('straight',), None)), True, False),
    ('a shared lane the route uses', guidance((('left', 'straight'), 'none'), (('straight',), None)), True, True),
    ('the turn is from another lane', guidance((('straight',), None), (('left',), 'left')), True, False),
    ('a U-turn only lane on the left', guidance((('none',), None), (('left',), 'left')), True, False),
    ('a U-turn lane the route marks', guidance((('none',), 'none'), (('left',), 'left')), True, False),
    ('the route takes the left lane straight', guidance((('left', 'straight'), 'straight'), (('straight',), None)), True, False),
    ('left-hand traffic, the right lane', guidance((('straight',), None), (('right',), 'right')), False, True),
    ('a direction on a lane the route does not use', ({'directions': ['left'], 'active': False, 'activeDirection': 'left'},), True, False),
    ('left-hand traffic, the turn from another lane', guidance((('right',), 'right'), (('straight',), None)), False, False),
  ])
  def test_far_turn_lane(self, _name, lanes, left, expected):
    assert far_turn_lane(far_instruction(lanes=lanes).lanes, left) == expected

  def test_posted_speed_limit(self):
    assert math.isclose(posted_speed_limit(plan_submaster(plan_message())), MPH35, rel_tol=1e-6)
    for sm, why in [
      (plan_submaster(plan_message(), age=1.), 'stale'),
      (plan_submaster(plan_message(), age=-1.), 'from the future'),
      (plan_submaster(plan_message(), seen=False), 'never seen'),
      (plan_submaster(plan_message(), valid=False), 'an invalid message'),
      (plan_submaster(plan_message(valid=False)), 'no limit found'),
      (plan_submaster(plan_message(math.nan)), 'NaN'),
      (plan_submaster(plan_message(math.inf)), 'infinite'),
      (plan_submaster(plan_message(0.)), 'zero'),
      (plan_submaster(plan_message(-5.)), 'negative'),
      (FakeSubMaster(seen={}, valid={}, recv_time={}, data={}), 'not subscribed'),
      (FakeSubMaster(), 'no SubMaster state at all'),
    ]:
      assert posted_speed_limit(sm) == 0., why

  def test_nav_inputs(self):
    # real messages, so a renamed or retired field fails here rather than in modeld
    now = time.monotonic()
    instruction = far_instruction()
    car_control = messaging.new_message('carControl').carControl
    car_control.longActive = True
    dm = messaging.new_message('driverMonitoringState').driverMonitoringState
    data = {'navInstruction': instruction, 'carControl': car_control, 'carState': messaging.new_message('carState').carState,
            'driverMonitoringState': dm, 'longitudinalPlanSP': plan_message(MPH45)}
    sm = FakeSubMaster(seen={'navInstruction': True, 'longitudinalPlanSP': True}, valid={'navInstruction': True, 'longitudinalPlanSP': True},
                       recv_time={'navInstruction': now, 'longitudinalPlanSP': now - 0.05}, data=data)
    inputs = nav_inputs(sm, model_message(), True)
    assert inputs.far_lanes == (True, False) and not inputs.driver_alert and math.isclose(inputs.speed_limit, MPH45, rel_tol=1e-6)
    assert nav_inputs(sm, messaging.new_message('modelV2').modelV2, True).far_lanes == (False, False)  # no model output yet
    for level in ('one', 'two', 'three'):
      dm.alertLevel = level
      assert nav_inputs(sm, model_message(), True).driver_alert, level
    dm.alertLevel = 'none'
    sm.recv_time['longitudinalPlanSP'] = now - 1.  # plannerd stopped publishing: the limit is unknown
    assert nav_inputs(sm, model_message(), True).speed_limit == 0.
    for state in (sm.seen, sm.valid, sm.recv_time, sm.data):  # a SubMaster without the service never raises
      del state['longitudinalPlanSP']
    assert nav_inputs(sm, model_message(), True).speed_limit == 0.


class TestFarStarts(OpenpilotTestCase):
  @parameterized.expand([
    # a curbed left turn bay, the lane beside a raised median, the left lane of a one-way street (no lane guidance)
    ('curbed bay', {'edge_y': (-1.9, 6.)}, BAY),
    ('median lane', {'edge_y': (-1.4, 5.), 'edge_stds': (0.2, 0.3)}, guidance((('left',), 'left'), (('straight',), None))),
    ('one-way left lane', {'edge_y': (-2.3, 9.), 'edge_stds': (0.4, 0.3)}, ()),
  ])
  def test_one_pulse_from_a_confirmed_lane(self, _name, model_fields, lanes):
    curb = far_lanes(model_message(**model_fields))
    frames = run(far_controller(), 60., 4.5, 14., lanes=lanes, far_lanes=curb)
    edges = pulses(frames)
    assert curb == (True, False) and len(edges) == 1 and abs(edges[0] - slowed_trigger_distance(4.5)) <= 4.5 * DT_MDL + 1e-6
    assert not any(f[2] for f in frames)  # never a prompt from a confirmed lane, also while the curb's run builds
    assert all(f[3] for f in frames if NAV_TURN_STEER_DISTANCE < f[1] <= edges[0])  # Turning Across Traffic while it holds the turn

  @parameterized.expand([
    # (name, controller settings, car state, route inputs, whether it asks for the blinker)
    ('setting off', {'far': False}, {}, {}, False),
    ('sharp left', {}, {}, {'modifier': 'sharp left'}, True),
    ('end of road: a T junction', {}, {}, {'maneuver_type': 'end of road'}, True),
    ('posted limit unknown', {}, {}, {'speed_limit': 0.}, True),
    ('posted 50 mph', {}, {}, {'speed_limit': MPH50}, True),
    ('posted 75 km/h', {}, {}, {'speed_limit': 75 * KPH}, True),
    ('driver monitoring warning', {}, {}, {'driver_alert': True}, True),
    ('lane guidance puts the turn in another lane', {}, {}, {'lanes': guidance((('straight',), None), (('left',), 'left'))}, True),
    ('no curb: a two-way road', {}, {}, {'far_lanes': (False, False)}, True),
    ('a car in the blind spot', {}, {'leftBlindspot': True}, {}, True),
    ("the car's own cruise control: called off", {}, {}, {'long_active': False}, False),
    ('the right blinker: called off', {}, {'rightBlinker': True}, {}, False),
    ('the gas: called off', {}, {'gasPressed': True}, {'long_active': False}, False),
    ('steering off', {}, {}, {}, False),
  ])
  def test_no_automatic_start(self, name, settings, car_fields, inputs, asks):
    frames = run(far_controller(**settings), 60., 4.5, 12., carstate=car_state(**({'vEgo': 4.5} | car_fields)),
                 lateral_active=name != 'steering off', **inputs)
    assert not pulses(frames) and any(f[2] for f in frames) == asks and not any(f[3] for f in frames)

  def test_posted_72_kph_turns(self):
    assert len(pulses(run(far_controller(), 60., 4.5, 12., speed_limit=72 * KPH))) == 1

  def test_needs_automatic_turns(self):
    # NavAutoTurnFarSide alone does nothing: the controller reads it only with NavAutoTurn on
    params = Params()
    params.put_bool('NavAutoTurnFarSide', True)
    params.put_bool('NavTurnDesire', True)
    params.put_bool('NavAutoTurn', False)
    c = NavDesireController()
    assert not c.far_enabled and not pulses(run(c, 60., 4.5, 12.)) and not c.speed_limit
    params.put_bool('NavAutoTurn', True)
    c = NavDesireController()
    assert c.far_enabled and len(pulses(run(c, 60., 4.5, 12.))) == 1

  @parameterized.expand([(200., 1), (100., 0)])
  def test_posted_limit_near_the_turn(self, change_at, expected):
    # a 50 mph road until `change_at` meters before the turn, 35 mph after: only the last AUTO_FAR_LIMIT_DISTANCE counts
    c = far_controller()
    frames = run(c, 300., 8., (300. - 60.) / 8., inputs=lambda distance, **kw: far_nav(distance=distance, speed_limit=MPH50 if distance > change_at else MPH35))
    frames += run(c, 60., 4.5, 12., inputs=lambda distance, **kw: far_nav(distance=distance, speed_limit=MPH35))
    assert len(pulses(frames)) == expected and (change_at > AUTO_FAR_LIMIT_DISTANCE) == bool(expected)

  def test_posted_limit_is_each_turns_own(self):
    # a left turn off a 50 mph road, missed, then the next left turn off a 35 mph road: only that road's limit counts
    c = far_controller()
    first = run(c, 160., 8., 190. / 8., position=1300., speed_limit=MPH50)
    second = run(c, 100., 4.5, 20., position=1000., speed_limit=MPH35)
    assert not pulses(first) and len(pulses(second, before=first[-1][0])) == 1

  def test_curb_run_needs_both_time_and_distance(self):
    c = far_controller()
    run(c, 60., 4.5, 1.)  # the maneuver, its posted limit and the slowdown
    c.far_lane_time, c.far_lane_distance = [1.4, 0.], [12., 0.]  # glimpsed at speed
    assert not c._auto_turn_allowed(car_state(vEgo=4.5), far_nav(distance=21.), 4.5, True)
    c.far_lane_time, c.far_lane_distance = [3., 0.], [9.9, 0.]  # waiting beside it, or creeping
    assert not c._auto_turn_allowed(car_state(vEgo=4.5), far_nav(distance=21.), 4.5, True)
    c.far_lane_time, c.far_lane_distance = [1.5, 0.], [10., 0.]
    assert c._auto_turn_allowed(car_state(vEgo=4.5), far_nav(distance=21.), 4.5, True)

  def test_posted_limit_lost_near_the_turn(self):
    # the limit known on the approach, then lost for a while: the highest seen near the turn still counts
    frames = run(far_controller(), 60., 4.5, 12., inputs=lambda distance, **kw: far_nav(distance=distance, speed_limit=MPH35 if distance > 40. else 0.))
    assert len(pulses(frames)) == 1

  @parameterized.expand([
    ('curb seen 8 m of every 10', lambda d: (d % 10. < 8., False), None, None),
    ('a blind spot blip restarts the curb', lambda d: (True, False), lambda d: 24.8 < d <= 25., None),
    ('a car in the blind spot until 15 m out', lambda d: (True, False), lambda d: d > 15., None),
    ('a lane change ending 25 m out', lambda d: (True, False), None, lambda d: 25. < d <= 45.),
  ])
  def test_curb_run(self, _name, curb, blindspot, lane_changing):
    frames = run(far_controller(), 70., 4.5, 14., inputs=lambda distance, **kw: far_nav(distance=distance, far_lanes=curb(distance)),
                 carstate=lambda d: car_state(vEgo=4.5, leftBlindspot=bool(blindspot and blindspot(d))), lane_changing=lane_changing)
    assert not pulses(frames)

  @parameterized.expand([(40., 1), (30., 1), (17.5, 0)])
  def test_curb_for_10_m(self, seen_from, expected):
    # the curb comes up beside the car at seen_from (a bay opening, a median starting): 10 m of it before the turn
    frames = run(far_controller(), 60., 4.5, 12., inputs=lambda distance, **kw: far_nav(distance=distance, far_lanes=(distance <= seen_from, False)))
    edges = pulses(frames)
    assert len(edges) == expected and all(NAV_TURN_STEER_DISTANCE < d <= seen_from - FAR_LANE_DISTANCE + 1e-6 for d in edges)

  def test_blind_spot_restarts_the_curb(self):
    # a car beside on the left 35-30 m out: there is a lane there after all, so the curb counts again from 30 m
    frames = run(far_controller(), 60., 4.5, 12., carstate=lambda d: car_state(vEgo=4.5, leftBlindspot=30. < d <= 35.))
    edges = pulses(frames)
    assert len(edges) == 1 and NAV_TURN_STEER_DISTANCE < edges[0] <= 30. - FAR_LANE_DISTANCE + 1e-6

  @parameterized.expand([(kind,) for kind in ('blind spot', 'curb', 'lane change')])
  def test_never_late(self, kind):
    # whatever holds the start back until late (a car in the blind spot, the curb coming up, a lane change), it comes
    # FAR_TURN_MIN_LEAD or more before the steering point, or not at all
    for clear_at in range(45, 9, -1):
      def inputs(distance, at=clear_at, **kw):
        return far_nav(distance=distance, far_lanes=(kind != 'curb' or distance <= at, False))
      def carstate(d, at=clear_at):
        return car_state(vEgo=4.5, leftBlindspot=kind == 'blind spot' and d > at)
      frames = run(far_controller(), 70., 4.5, 14., inputs=inputs, carstate=carstate,
                   lane_changing=lambda d, at=clear_at: kind == 'lane change' and d > at)
      edges = pulses(frames)
      assert all(time_to_steer(d, 4.5, True) >= FAR_TURN_MIN_LEAD - 1e-6 for d in edges), (clear_at, edges)
      if clear_at >= 30:
        assert len(edges) == 1, clear_at
      elif clear_at <= 26:
        assert not edges, clear_at


class TestFarStops(OpenpilotTestCase):
  def test_stop_before_the_start(self):
    # stopped 25 m out (a light, a queue) before the turn started: the turn is the driver's. The car keeps slowing for it,
    # asks for the blinker and, without it, goes straight
    c = far_controller()
    approach = run(c, 60., 4.5, (60. - 25.) / 4.5)
    stopped = run(c, 25., 0., 6.)
    assert not pulses(approach + stopped) and c.speed_limit > 0. and stopped[-1][2] and not any(f[2] for f in approach)
    green = run(c, 25., 4.5, (25. - 5.) / 4.5)
    assert not pulses(green, before=stopped[-1][0]) and c.speed_limit > 0.
    assert all(f[2] for f in green if f[1] > NAV_TURN_STEER_DISTANCE + 2.)  # still asking while signaling can start it

  def test_signal_after_a_stop(self):
    c = far_controller()
    run(c, 60., 4.5, (60. - 25.) / 4.5)
    stopped = run(c, 25., 0., 6.)
    signaled = run(c, 25., 0., 1., carstate=car_state(vEgo=0., standstill=True, leftBlinker=True))
    pulling = run(c, 25., 4.5, 4., carstate=car_state(vEgo=4.5, leftBlinker=True))
    edges = pulses(signaled + pulling, before=stopped[-1][0])
    assert len(edges) == 1 and abs(edges[0] - slowed_trigger_distance(4.5)) <= 4.5 * DT_MDL + 1e-6 and c.turn_by_blinker
    assert not any(f[2] for f in signaled + pulling) and not any(f[3] for f in signaled + pulling)

  def test_stop_during_a_gps_gap(self):
    # stopped 25 m out while navd has no fix for 2 s, the model still seeing the curb: the stop still hands the turn to the
    # driver, who sees the prompt once the route is back
    c = far_controller()
    approach = run(c, 60., 4.5, 35. / 4.5)
    for _ in range(round(2. / DT_MDL)):
      c.update(car_state(vEgo=0., standstill=True), True, NavInputs(long_active=True, far_lanes=(True, False), speed_limit=MPH35))
      assert not c.signal_prompt  # no prompt without the route's instruction
    green = run(c, 25., 4.5, 4.)
    assert not pulses(approach + green) and any(f[2] for f in green) and c.speed_limit > 0.

  def test_creeping_in_a_queue_counts_as_a_stop(self):
    c = far_controller()
    approach = run(c, 60., 4.5, 32. / 4.5)
    creeping = run(c, 28., 1., 3.)
    green = run(c, 25., 4.5, 4.)
    assert not pulses(approach + creeping + green) and creeping[-1][2] and c.speed_limit > 0.

  def test_no_prompt_pulling_away_after_the_hand_back(self):
    # handed back at the line 12 m out, then the light turns green and the car pulls away unsignaled: past a crawl a blinker
    # could no longer start the turn there, so it stops asking, and the car goes straight
    c = far_controller()
    run(c, 40., 4.5, 28. / 4.5)
    waiting = run(c, 12., 0., 10.)
    green = run(c, 12., 4.5, 2.)
    assert waiting[-1][2] and not any(f[2] for f in green) and not pulses(green, before=waiting[-1][0])

  def test_stop_outside_the_approach(self):
    # stopped in a queue 40 m out, beyond where the car approaches the turn: it still turns by itself, and never asks
    c = far_controller()
    frames = run(c, 60., 4.5, 20. / 4.5) + run(c, 40., 0., 6.) + run(c, 40., 4.5, 8.)
    assert len(pulses(frames)) == 1 and not any(f[2] for f in frames)

  @parameterized.expand([(12.,), (9.,)])
  def test_stop_after_the_start(self, stop_at):
    # the automatic turn pulsed 21.5 m out, then the car stops at the line (the light turned red, a pedestrian): it holds
    # the turn without pulsing again, and FAR_STOP_RELEASE after the pulse hands it to the driver, asking for the blinker
    c = far_controller()
    approach = run(c, 40., 4.5, (40. - stop_at) / 4.5)
    edges = pulses(approach)
    assert len(edges) == 1 and c.started and approach[-1][3]
    waiting = run(c, stop_at, 0., 10.)
    assert not pulses(waiting, before=approach[-1][0]) and c.turning is None and waiting[-1][2] and not waiting[-1][3]
    held = sum(1 for f in waiting if f[0] in TURN_DIRECTIONS) * DT_MDL
    assert abs(held - (5. - (edges[0] - stop_at) / 4.5)) < 2 * DT_MDL, held  # 5 s after the pulse, the model's desire history
    assert all(f[3] == (f[0] in TURN_DIRECTIONS) for f in waiting)  # Turning Across Traffic as long as it holds, then the prompt
    assert c.speed_limit > 0.

  def test_signal_after_the_hand_back(self):
    # signaled at the line once handed back: at a standstill 9 m out it pulses at once; 12 m out, as the car creeps away
    for stop_at, creep in ((9., 0.), (12., 1.2)):
      c = far_controller()
      approach = run(c, 40., 4.5, (40. - stop_at) / 4.5)
      waiting = run(c, stop_at, 0., 10.)
      signaled = run(c, stop_at, creep, 2., carstate=car_state(vEgo=creep, standstill=creep == 0., leftBlinker=True))
      assert len(pulses(approach)) == 1 and len(pulses(signaled, before=waiting[-1][0])) == 1 and c.turn_by_blinker, stop_at
      assert not signaled[-1][2] and not signaled[-1][3]

  def test_brief_stop_keeps_the_turn(self):
    # the model yields for 2 s 10 m out after the automatic pulse, then goes on: still its one turn, held throughout
    c = far_controller()
    frames = run(c, 40., 4.5, 30. / 4.5) + run(c, 10., 0., 2.) + run(c, 10., 3., 2.)
    assert len(pulses(frames)) == 1 and all(f[0] in TURN_DIRECTIONS for f in frames if f[1] <= 21.)
    assert not any(f[2] for f in frames)

  def test_keep_alive_only_while_driver_monitoring_is_quiet(self):
    # after the automatic pulse the car slows to a walk behind a cyclist, still 2 s or more from the steering point when
    # the pulse is due again: it pulses again, but not while driver monitoring warns
    for alert, expected in ((False, 2), (True, 1)):
      c = far_controller()
      frames = run(c, 40., 4.5, (40. - 18.) / 4.5)
      frames += run(c, 18., 1.6, 3.4, driver_alert=alert)
      assert len(pulses(frames)) == expected, alert


class TestFarPrompt(OpenpilotTestCase):
  def test_prompt_on_a_two_way_road(self):
    # no curb: it asks FAR_SIGNAL_LEAD before the start would come, after the debounce, until the steering point, once
    v = 4.5
    frames = run(far_controller(), 60., v, 13., far_lanes=(False, False))
    prompted = [f[1] for f in frames if f[2]]
    assert prompted and not pulses(frames) and rises([f[2] for f in frames]) == 1
    lead = time_to_steer(max(prompted), v, True)
    expected = NAV_TURN_TRIGGER_TIME + FAR_SIGNAL_LEAD - FAR_PROMPT_DEBOUNCE
    assert expected - 0.1 <= lead <= expected + 0.06, lead
    assert min(prompted) > NAV_TURN_STEER_DISTANCE
    assert not any(f[2] for f in run(far_controller(False), 60., v, 13., far_lanes=(False, False)))  # setting off: as before

  def test_signal_at_the_prompt_turns_on_time(self):
    v, c = 4.5, far_controller()
    frames, first, d = [], None, 60.
    for _ in range(round(13. / DT_MDL)):
      blinker = first is not None and d <= first - v * 1.  # signaled 1 s after the prompt
      c.update(car_state(vEgo=v, leftBlinker=blinker), True, far_nav(distance=d, far_lanes=(False, False)))
      first = d if c.signal_prompt and first is None else first
      frames.append((c.turn_direction, d))
      d -= v * DT_MDL
    edges = pulses(frames)
    assert len(edges) == 1 and abs(edges[0] - slowed_trigger_distance(v)) <= v * DT_MDL + 1e-6 and c.turn_by_blinker and not c.turn_across

  @parameterized.expand([(25., False), (28., False), (30., True)])
  def test_late_curb(self, seen_from, turns):
    # the curb only from 28 m: it asks once, and then waits for the blinker even though the curb's run would be long enough
    # 2 s before the steering point; from 30 m, the debounce lets the curb's run build and it turns by itself, never having
    # asked
    frames = run(far_controller(), 60., 4.5, 13., inputs=lambda distance, **kw: far_nav(distance=distance, far_lanes=(distance <= seen_from, False)))
    assert (len(pulses(frames)) == 1) == turns and rises([f[2] for f in frames]) == (0 if turns else 1)

  @parameterized.expand([('the gas', {'gasPressed': True}, {'long_active': False}), ('the right blinker', {'rightBlinker': True}, {})])
  def test_no_prompt_once_called_off(self, _name, car_fields, inputs):
    # the driver called the turn off for a moment on the approach: it neither turns nor asks for the blinker after
    c = far_controller()
    frames = run(c, 60., 4.5, 2.) + run(c, 51., 4.5, 0.5, carstate=car_state(vEgo=4.5, **car_fields), **inputs) + run(c, 48.75, 4.5, 10.)
    assert not pulses(frames) and not any(f[2] for f in frames) and c.called_off

  def test_asks_while_a_car_is_in_the_blind_spot(self):
    # a car beside on the left through the start's window: there is a lane there, so it won't turn by itself here
    frames = run(far_controller(), 60., 4.5, 13., carstate=lambda d: car_state(vEgo=4.5, leftBlindspot=d <= 32.))
    assert not pulses(frames) and rises([f[2] for f in frames]) == 1 and max(f[1] for f in frames if f[2]) >= 28.

  def test_debounced(self):
    # a curb read on and off, 0.3 s missing out of every second: never a run long enough to turn, and no prompt until the
    # start is out of reach for good, FAR_TURN_MIN_LEAD before the steering point
    v = 4.5
    frames = run(far_controller(), 60., v, 13., inputs=lambda distance, **kw: far_nav(distance=distance, far_lanes=((60. - distance) / v % 1. < 0.7, False)))
    prompted = [f[1] for f in frames if f[2]]
    assert not pulses(frames) and rises([f[2] for f in frames]) == 1
    assert time_to_steer(max(prompted), v, True) <= FAR_TURN_MIN_LEAD - FAR_PROMPT_DEBOUNCE + 0.06

  def test_prompt_survives_a_reset(self):
    # it asked, then the controller failed for a few frames (DesireHelper resets it), then the curb shows up: it still
    # waits for the blinker
    v, c = 4.5, far_controller()
    frames = run(c, 60., v, (60. - 28.) / v, far_lanes=(False, False))
    assert frames[-1][2] and c.asked
    c.reset()
    assert not c.signal_prompt and not c.turn_across
    frames = run(c, 28., v, 6.)
    assert not pulses(frames) and frames[0][2]

  def test_no_prompt_during_a_lane_change(self):
    frames = run(far_controller(), 60., 4.5, 13., far_lanes=(False, False), lane_changing=lambda d: 26. < d <= 34.)
    assert not any(f[2] for f in frames if 26. < f[1] <= 34.) and any(f[2] for f in frames)

  def test_no_prompt_for_a_near_side_or_far_off(self):
    frames = run(far_controller(), 60., 4.5, 13., modifier='right', lanes=(), outer_lanes=(False, False), far_lanes=(False, False))
    assert not any(f[2] for f in frames)
    frames = run(far_controller(), 300., 4.5, 20., far_lanes=(False, False))
    assert not any(f[2] for f in frames)

  def test_no_prompt_mid_turn(self):
    c = far_controller()
    frames = run(c, 40., 4.5, 30. / 4.5)
    frames += run(c, 10., 3., 1.5, yaw_rate=math.radians(40.) / 1.5)  # turned in 40 deg
    frames += run(c, 5.5, 0., 8.)
    assert len(pulses(frames)) == 1 and not c.lined_up and not any(f[2] for f in frames)

  def test_no_prompt_without_a_route(self):
    c = far_controller()
    frames = run(c, 60., 4.5, 8., far_lanes=(False, False))
    assert frames[-1][2]
    for _ in range(round(5. / DT_MDL)):  # navd stops: no prompt without its instruction, nor once the route is gone
      c.update(car_state(vEgo=0., standstill=True), True, NavInputs(long_active=True))
      assert not c.signal_prompt and not c.turn_across
    c.update(car_state(vEgo=0., standstill=True), True, None)
    assert not c.signal_prompt and not c.turn_across

  def test_no_prompt_past_the_turn(self):
    # it asked, the driver didn't signal, and the car went straight: past the maneuver point it no longer asks, also once
    # stopped there
    c = far_controller()
    frames = run(c, 60., 4.5, 65. / 4.5, far_lanes=(False, False))
    past = run(c, -5., 0., 3., far_lanes=(False, False))
    assert any(f[2] for f in frames) and not any(f[2] for f in past) and c.tracker.passed


class TestFarBlinkerCancel(OpenpilotTestCase):
  @parameterized.expand([
    ('the driver switches it off, no lane change', None, 50., 0),
    ('the car cancels it as the lane change ends', (55., 45.), 45., 1),
    ('the car cancels it 1 s after the lane change', (60., 50.), 50. - 4.5, 1),
    ('it goes off during the lane change', (55., 45.), 50., 1),
    ('switched off 3 s after the lane change', (75., 65.), 65. - 3 * 4.5, 0),
  ])
  def test_cancel(self, _name, lane_change, off_at, expected):
    c = far_controller()
    frames = run(c, 90., 4.5, 18., carstate=lambda d: car_state(vEgo=4.5, leftBlinker=off_at < d <= 85.),
                 lane_changing=lambda d: lane_change is not None and lane_change[1] < d <= lane_change[0])
    assert len(pulses(frames)) == expected
    assert not c.turn_by_blinker and any(f[3] for f in frames) == bool(expected)
    prompted = [f[1] for f in frames if f[2]]
    if expected:
      assert not prompted
    else:
      assert prompted and max(prompted) >= 28.  # it asks as early as for a turn that can't start by itself

  def test_blinker_kept_on_is_a_blinker_turn(self):
    c = far_controller()
    frames = run(c, 90., 4.5, 18., carstate=lambda d: car_state(vEgo=4.5, leftBlinker=d <= 85.))
    assert len(pulses(frames)) == 1 and c.turn_by_blinker and not any(f[3] for f in frames)

  def test_early_latch_on_a_fast_road(self):
    # 47 mph on a road posted 45: the blinker latches from the slowdown's approach distance, beyond the signal distance. A
    # lane change into the bay ending as the car cancels the blinker still leaves the automatic turn; the driver switching
    # it off out there does not
    for lane_change, expected in (((262., 240.), 1), (None, 0)):
      dh = far_helper(turns=False)
      off_at = 240. if lane_change else 255.

      def carstate(d, v, off_at=off_at, lane_change=lane_change):
        return car_state(vEgo=v, leftBlinker=off_at < d <= 270., **(nudge(right=False) if lane_change and 262. >= d > 258. else {}))
      frames = closed_loop(dh, 300., 21., 40., inputs=far_nav, carstate=carstate, speed_limit=MPH45,
                           lane_change_prob=lambda d, lane_change=lane_change: 1. if lane_change and d > lane_change[1] else 0.)
      assert any(f[0] == log.Desire.laneChangeLeft for f in frames) == (lane_change is not None)
      assert len(pulses(frames, TURN_DESIRES)) == expected and not dh.nav_desire.turn_by_blinker


class TestFarUnchanged(OpenpilotTestCase):
  RIGHT = {'modifier': 'right', 'lanes': (), 'outer_lanes': (False, True), 'far_lanes': (False, True)}

  @parameterized.expand([
    # (name, controller settings, route inputs, the drive: (start, speed, seconds, car state fields or a function of the
    # distance, lane change from, to), rising edges)
    ('automatic right turn', {}, RIGHT, [(60., 4.5, 14., {}, None)], 1),
    ('blinker right turn', {'turns': True}, RIGHT | {'outer_lanes': (False, False)}, [(60., 4.5, 14., {'rightBlinker': True}, None)], 1),
    ('right turn after a stop 25 m out', {}, RIGHT, [(60., 4.5, 35. / 4.5, {}, None), (25., 0., 6., {}, None), (25., 4.5, 6., {}, None)], 1),
    # kept alive every 4 s while it waits at the line
    ('right turn waiting at the line', {}, RIGHT, [(40., 4.5, 30. / 4.5, {}, None), (10., 0., 10., {}, None), (10., 2., 6., {}, None)], 4),
    ('right turn with a blind spot blip', {}, RIGHT, [(60., 4.5, 14., lambda d: {'rightBlindspot': 30. < d <= 31.}, None)], 1),
    ('right turn after a lane change', {}, RIGHT, [(70., 4.5, 15., lambda d: {'rightBlinker': 40. < d <= 65.}, (55., 40.))], 1),
    ('right blinker on and off on the approach', {}, RIGHT, [(70., 4.5, 15., lambda d: {'rightBlinker': 45. < d <= 60.}, None)], 1),
    ('right turn called off with the gas', {}, RIGHT, [(60., 4.5, 4., {}, None), (42., 4.5, 1., {'gasPressed': True}, None),
                                                       (37.5, 4.5, 8., {}, None)], 0),
    ('left-hand traffic left turn', {}, {'modifier': 'left', 'lanes': (), 'outer_lanes': (True, False), 'far_lanes': (True, False),
                                         'right_hand_traffic': False}, [(60., 4.5, 14., {}, None)], 1),
    ('far side blinker turn', {'turns': True}, {'far_lanes': (False, False)}, [(60., 4.5, 14., {'leftBlinker': True}, None)], 1),
  ])
  def test_same_with_the_setting_on_and_off(self, _name, settings, inputs, drive_plan, edges):
    # every output, every frame, the same whether automatic far side turns are on or off
    def trace(far):
      c, frames = far_controller(far=far, **settings), []
      for start, v, seconds, fields, lane_change in drive_plan:
        def carstate(d, v=v, fields=fields):
          return car_state(vEgo=v, standstill=v == 0., **(fields(d) if callable(fields) else fields))
        frames += run(c, start, v, seconds, carstate=carstate, lane_changing=lambda d, lc=lane_change: lc is not None and lc[1] < d <= lc[0],
                      **(inputs | ({'long_active': False} if fields == {'gasPressed': True} else {})))
      return frames
    on, off = trace(True), trace(False)
    assert [f[:1] + f[2:] for f in on] == [f[:1] + f[2:] for f in off]
    assert not any(f[2] or f[3] for f in on) and len(pulses(on)) == edges

  def test_near_side_closed_loop(self):
    def trace(far):
      return closed_loop(far_controller(far=far), 200., 13.4, 40., inputs=far_nav, carstate=lambda d, v: car_state(vEgo=v), **self.RIGHT)
    assert trace(True) == trace(False) and len(pulses(trace(True))) == 1


class TestFarLeftHandTraffic(OpenpilotTestCase):
  # where traffic keeps left the far side is the right: the curb on the right, the rightmost lane in the guidance, the right
  # blind spot
  LHT = {'modifier': 'right', 'lanes': guidance((('straight',), None), (('right',), 'right')), 'right_hand_traffic': False,
         'outer_lanes': (False, True), 'far_lanes': (False, True)}

  def test_turns_from_the_right_curb(self):
    frames = run(far_controller(), 60., 4.5, 12., **self.LHT)
    assert len(pulses(frames)) == 1 and frames[-1][0] == TURN_DIRECTIONS[1] and any(f[3] for f in frames)

  @parameterized.expand([
    ('the curb on the left', {'far_lanes': (True, False), 'outer_lanes': (True, False)}, {}),
    ('a car in the right blind spot', {}, {'rightBlindspot': True}),
    ('lane guidance puts the turn in another lane', {'lanes': guidance((('right',), 'right'), (('straight',), None))}, {}),
    ('the left blinker', {}, {'leftBlinker': True}),
  ])
  def test_waits(self, _name, inputs, car_fields):
    frames = run(far_controller(), 60., 4.5, 12., carstate=car_state(**({'vEgo': 4.5} | car_fields)), **(self.LHT | inputs))
    assert not pulses(frames)

  def test_stop_before_the_start(self):
    c = far_controller()
    approach, stopped = run(c, 60., 4.5, 35. / 4.5, **self.LHT), run(c, 25., 0., 6., **self.LHT)
    green = run(c, 25., 4.5, 4., **self.LHT)
    assert not pulses(approach + stopped + green) and stopped[-1][2] and c.speed_limit > 0.


class TestFarClosedLoop(OpenpilotTestCase):
  @parameterized.expand([(20,), (30,), (40,)])
  def test_one_turn_command(self, mph):
    # the slowdown fed back into the car's speed, from 20, 30 and 40 mph on a road posted 45: one pulse about 3 s before the
    # steering point, none rolling into the turn, and past the corner the model's history holds at most the one
    frames = closed_loop(far_controller(), 250., mph * MPH, 60., inputs=far_nav, speed_limit=MPH45, carstate=lambda d, v: car_state(vEgo=v))
    edges = [f for f, p in zip(frames, [None] + [g[0] for g in frames[:-1]], strict=True) if f[0] in TURN_DIRECTIONS and f[0] != p]
    assert len(edges) == 1 and edges[0][1] > NAV_TURN_STEER_DISTANCE and edges[0][2] <= 7.
    assert time_to_steer(edges[0][1], edges[0][2], True) >= FAR_TURN_MIN_LEAD
    assert all(len(in_history(frames, d)) <= 1 for d in (0., -5., -10.))

  def test_one_turn_command_after_slowing_behind_traffic(self):
    # a slower car ahead from just after the pulse: the turn is kept alive while still 2 s or more from the steering point,
    # never as the car rolls into it
    frames = closed_loop(far_controller(), 200., 13.4, 50., inputs=far_nav, model_speed=lambda d: 2. if d <= 20. else 99.,
                         carstate=lambda d, v: car_state(vEgo=v))
    moving = [f for f, p in zip(frames, [None] + [g[0] for g in frames[:-1]], strict=True) if f[0] in TURN_DIRECTIONS and f[0] != p]
    assert moving and all(f[1] > NAV_TURN_STEER_DISTANCE for f in moving)
    assert all(len(in_history(frames, d)) <= 1 for d in (0., -5., -10.))


class TestFarDesireHelper(OpenpilotTestCase):
  def bay(self, far=True, cancel=True, curb=True, lane_turns=False):
    """Signaled for the turn 120 m out at 30 mph, nudged into the bay; the lane change ends 70 m out, where the car cancels
    the blinker, unless not cancel. The bay's curb shows beside the car from 75 m, unless not curb."""
    signal_at, nudge_at, done_at = 120., 112., 70.

    def carstate(d, v):
      blinker = d <= signal_at and (d > done_at or not cancel)
      return car_state(vEgo=v, leftBlinker=blinker, **(nudge(right=False) if nudge_at >= d > nudge_at - 4. else {}))

    def inputs(distance, **kw):
      seen = curb and distance <= done_at + 5.
      return far_nav(distance=distance, far_lanes=(seen, False), outer_lanes=(seen, False))
    dh = far_helper(far, lane_turns=lane_turns)
    prompts = []

    def lateral_active(d):  # steering is on throughout; this records the controller's prompt each frame
      prompts.append(dh.nav_desire.signal_prompt)
      return True
    frames = closed_loop(dh, 200., 13.4, 40., inputs=inputs, carstate=carstate, lateral_active=lateral_active,
                         lane_change_prob=lambda d: 1. if d > done_at else 0.)
    return dh, frames, prompts

  def test_lane_change_into_a_curbed_bay(self):
    # the car cancels the blinker as the lane change ends: the automatic turn comes, once, by itself, with its alert
    dh, frames, prompts = self.bay()
    changing = [f[1] for f in frames if f[0] == log.Desire.laneChangeLeft]
    edges = [f for f, p in zip(frames, [None] + [g[0] for g in frames[:-1]], strict=True) if f[0] in TURN_DESIRES and f[0] != p]
    assert changing and len(edges) == 1 and edges[0][0] == log.Desire.turnLeft and edges[0][1] > NAV_TURN_STEER_DISTANCE
    assert not dh.nav_desire.turn_by_blinker and not any(prompts)

  def test_painted_bay(self):
    # no curb beside the bay: no turn by itself, and it asks for the blinker
    dh, frames, prompts = self.bay(curb=False)
    assert not pulses(frames, TURN_DESIRES) and any(prompts)

  def test_setting_off(self):
    dh, frames, prompts = self.bay(far=False)
    assert not pulses(frames, TURN_DESIRES) and not any(prompts)  # as before: the blinker went off, so the turn waits for it

  def test_blinker_kept_on(self):
    dh, frames, prompts = self.bay(cancel=False)
    assert len(pulses(frames, TURN_DESIRES)) == 1 and dh.nav_desire.turn_by_blinker and not any(prompts)

  def test_with_lane_turn_desire(self):
    # Lane Turn Desire stays quiet while the route has the turns, so the blinker below 20 mph adds no turn of its own
    for cancel in (True, False):
      dh, frames, prompts = self.bay(cancel=cancel, lane_turns=True)
      edges = pulses(frames, TURN_DESIRES)
      assert len(edges) == 1 and edges[0] > NAV_TURN_STEER_DISTANCE and pulses(frames, (log.Desire.turnLeft,)) == edges, cancel

  def test_signaled_beyond_the_approach_at_a_walk(self):
    # Lane Turn Desire on, signaling left 90 m out at 10 mph, beyond the turn's approach: no lane turn at once, only the
    # route's turn at its trigger
    dh = far_helper(lane_turns=True)
    frames, d = [], 100.
    for _ in range(round(19. / DT_MDL)):
      dh.update(car_state(vEgo=4.5, leftBlinker=d <= 90.), True, 1.0, False, False, far_nav(distance=d, far_lanes=(False, False)))
      frames.append((dh.desire, d))
      d -= 4.5 * DT_MDL
    edges = pulses(frames, TURN_DESIRES)
    assert len(edges) == 1 and abs(edges[0] - slowed_trigger_distance(4.5)) <= 4.5 * DT_MDL + 1e-6

  def test_outputs_after_a_failure(self):
    # an unreadable route message mid approach: DesireHelper resets the controller, which then neither asks nor shows the
    # turn across until it reads the route again
    class Unreadable:
      maneuverType, maneuverModifier, lanes = 'turn', 'left', []

      @property
      def maneuverDistance(self):
        raise ValueError("corrupt message")

    dh = far_helper()
    for i in range(round(9. / DT_MDL)):
      d = 60. - 4.5 * DT_MDL * i
      dh.update(car_state(vEgo=4.5), True, 1.0, False, False, far_nav(distance=d, far_lanes=(False, False)))
    assert dh.nav_desire.signal_prompt
    dh.update(car_state(vEgo=4.5), True, 1.0, False, False, NavInputs(Unreadable(), long_active=True, far_lanes=(True, False), speed_limit=MPH35))
    assert dh.nav_desire_failed and not dh.nav_desire.signal_prompt and not dh.nav_desire.turn_across
    dh.update(car_state(vEgo=4.5), True, 1.0, False, False, far_nav(distance=19.))  # the route back, the curb with it
    assert dh.nav_desire.signal_prompt and dh.desire == log.Desire.none  # it asked before, so it still waits for the blinker

  def test_reset_mid_turn_at_a_stop(self):
    # the automatic turn holds at the line when the controller fails and resets: no new turn command while the car waits,
    # nor as it creeps on
    c = far_controller()
    frames = run(c, 40., 4.5, 31. / 4.5)
    frames += run(c, 9., 0., 1.)
    c.reset()
    frames += run(c, 9., 0., 8.) + run(c, 9., 0.5, 2.)
    assert len(pulses(frames)) == 1 and not any(f[3] for f in frames[-round(10. / DT_MDL):])


class TestFarNavInputsReplay(OpenpilotTestCase):
  def test_model_messages_drive_the_turn(self):
    # the curb read from real model messages as the car comes up to the bay: 6 m to the left until 45 m out (a lane
    # between), then 1.9 m (the bay)
    c = far_controller()
    frames, d = [], 70.
    for _ in range(round(14. / DT_MDL)):
      curb = far_lanes(model_message(edge_y=(-6. if d > 45. else -1.9, 6.)))
      c.update(car_state(vEgo=4.5), True, dataclasses.replace(far_nav(distance=d), far_lanes=curb))
      frames.append((c.turn_direction, d))
      d -= 4.5 * DT_MDL
    edges = pulses(frames)
    assert len(edges) == 1 and NAV_TURN_STEER_DISTANCE < edges[0] <= 45. - FAR_LANE_DISTANCE + 1e-6
