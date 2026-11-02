"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Automatic turn lanes: from the curb lane, before a near side turn, the car moves by itself into a turn lane that opens beside
it, and the automatic turn then starts from that lane; where it must not (never on the far side, where a median's end looks
just like a bay), how the driver ends it, and that every approach still gives the driving model at most one lane change and
one turn command. Closed loop through the real DesireHelper, on real model, route, car and plan messages of a synthetic road.
"""
import ast
import copy
import dataclasses
import json
import math
import random
import re
import subprocess
import sys
import textwrap
import time
import types
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from opendbc.car.structs import car
from openpilot.common.basedir import BASEDIR
from openpilot.common.parameterized import parameterized
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import EVENTS, Events
from openpilot.selfdrive.ui import soundd
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_TIME_MAX, DesireHelper, LaneChangeState
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib import nav_desire
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_TURN_DECEL, NAV_TURN_MAX_DECEL, NAV_TURN_SPEED, NAV_TURN_STEER_DISTANCE, \
  POCKET_LOOKAHEAD, POCKET_MARGIN, POCKET_MAX_DISTANCE, POCKET_MEMORY, POCKET_MOVED, POCKET_WIDTH, NavDesireController, NavInputs, \
  RoadView, curb_distance, lane_line_distance, nav_inputs, road_view, turn_lane_open
from openpilot.sunnypilot.selfdrive.controls.lib.nav_turn_speed import NavTurnSpeedController
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_desire import FakeSubMaster, car_state
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_far_turn import guidance
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_far_turn_wiring import MODELDS, calls, method
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_turn_speed import source
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, ET, EventsSP, Priority
from openpilot.sunnypilot.sunnylink.tools.compile_settings_ui import DEFAULT_OUT

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
AudibleAlert = log.SelfdriveState.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert
AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert

MPH = 0.44704  # m/s
MPH35, MPH50 = 35 * MPH, 50 * MPH
LANE = 3.4  # m between the car's lane lines
POCKET = 3.3  # m a turn lane is wide
EDGE_LINE_PROB = 0.3  # a painted edge line beside a curb, faintly seen
LANE_CHANGE_TIME = 3.  # s the driving model takes to move the car over a lane
X = ModelConstants.X_IDXS  # m ahead of the car of the model's points

LANE_CHANGE_DESIRES = (log.Desire.laneChangeLeft, log.Desire.laneChangeRight)
TURN_DESIRES = (log.Desire.turnLeft, log.Desire.turnRight)

# route lane guidance, left to right: the turn from the rightmost lane, from the leftmost (a left turn bay), from another lane
RIGHT_LANE = guidance((('straight',), None), (('straight',), None), (('right',), 'right'))
LEFT_BAY = guidance((('left',), 'left'), (('straight',), None), (('straight', 'right'), None))
RIGHT_FROM_ANOTHER_LANE = guidance((('straight',), None), (('right',), 'right'), (('straight',), None))
LEFT_FROM_ANOTHER_LANE = guidance((('straight',), None), (('left',), 'left'), (('straight', 'right'), None))
# a two-lane road's: a shared left and through lane, or a left-only lane (curbed or painted)
SHARED_LEFT = guidance((('left', 'straight'), 'left'), (('straight', 'right'), None))
LEFT_ONLY = guidance((('left',), 'left'), (('straight', 'right'), None))


class Road:
  """A straight road up to the route's turn, as the driving model sees it from the car. Lateral positions are toward the
  turn lane's side (left with left=True), from the center of the lane the car starts in: that lane is LANE wide, with the
  road edge on that side `gutter` beyond its line there, and another lane beyond its other line. A turn lane opens on that
  side: `width` wide beyond the car's lane line, full width from `opens` m before the maneuver point after a `taper`, for
  `length` m (or on past the maneuver point); its lane line `line_prob` sure, and a line beyond it `beyond_prob` sure, or
  `beyond_inside` from the turn lane itself. opens=None: no turn lane."""

  def __init__(self, left=False, opens=85., taper=15., width=POCKET, length=None, gutter=0.2, line_prob=0.9, beyond_prob=0.05,
               edge_std=0.3, beyond_inside=0.05, edge_line_prob=EDGE_LINE_PROB):
    self.left, self.opens, self.taper, self.width, self.length, self.gutter = left, opens, taper, width, length, gutter
    self.line_prob, self.beyond_prob, self.edge_std, self.beyond_inside = line_prob, beyond_prob, edge_std, beyond_inside
    self.edge_line_prob = edge_line_prob  # the painted line beside the curb

  def edge(self, d: float) -> float:
    """m to the road edge on the turn lane's side, d m before the maneuver point."""
    curb = LANE / 2 + self.gutter
    if self.opens is None:
      return curb
    end = self.opens - self.length if self.length is not None else -math.inf
    if d < end or d > self.opens + self.taper:
      return curb
    return curb + self.width * min((self.opens + self.taper - d) / self.taper, 1.)

  def open_at(self, d: float) -> bool:
    return self.edge(d) > LANE / 2 + self.gutter + 1e-6

  def message(self, d: float, offset: float):
    """A real modelV2 message d m before the maneuver point, from the car `offset` m toward the turn lane's side of the
    center of the lane it started in. The model counts the lane lines from the lane the car's center is in."""
    pocket = self.open_at(d)
    if offset > LANE / 2:  # in the turn lane
      lines = [(-LANE / 2, 0.9), (LANE / 2, self.line_prob), (LANE / 2 + self.width, self.edge_line_prob), (LANE / 2 + self.width + LANE, self.beyond_inside)]
    elif offset < -LANE / 2:  # in the lane beyond the other line
      lines = [(-2.5 * LANE, 0.05), (-1.5 * LANE, 0.9), (-LANE / 2, 0.9), (LANE / 2, self.line_prob if pocket else self.edge_line_prob)]
    else:
      lines = [(-1.5 * LANE, 0.9), (-LANE / 2, 0.9), (LANE / 2, self.line_prob if pocket else self.edge_line_prob),
               (LANE / 2 + self.width, self.beyond_prob if pocket else 0.05)]
    edges = [([-2.5 * LANE] * len(X), 0.3), ([self.edge(d - x) for x in X], self.edge_std)]
    sign = -1. if self.left else 1.  # the model's y is to the right
    lines = [(sign * (s - offset), prob) for s, prob in lines]
    edges = [([sign * (s - offset) for s in ys], std) for ys, std in edges]
    if self.left:  # the model lists them left to right
      lines.reverse()
      edges.reverse()
    msg = messaging.new_message('modelV2').modelV2
    msg.laneLineProbs = [prob for _, prob in lines]
    for line, (y, _) in zip(msg.init('laneLines', 4), lines, strict=True):
      line.x, line.y = [0., 10.], [y, y]
    msg.roadEdgeStds = [std for _, std in edges]
    for edge, (ys, _) in zip(msg.init('roadEdges', 2), edges, strict=True):
      edge.x, edge.y = X, ys
    return msg


class TwoWay(Road):
  """A two-way road where traffic keeps right, seen toward the left: the car's left lane line is the center line, one
  oncoming lane (`width`) beyond it and the far curb beyond that, with no lane line seen beyond the oncoming lane (`beyond_prob`,
  a faint far edge line at most). Beside the car from `median[0]` down to `median[1]` m before the maneuver point, a raised
  median or an island, its curb `gutter` beyond the center line, ends in a nose `nose` m long, the edge unsure while the car
  passes it; and over `crossing` (from, to) the car crosses an intersection straight, with no lane lines and unsure edges.
  To the model, the oncoming lane beyond a median's or an island's end is a turn lane opening beside the car, with every
  number a curbed bay has."""

  def __init__(self, median=(math.inf, math.inf), nose=8., crossing=None, **road):
    super().__init__(**({'left': True, 'width': LANE} | road))
    self.median, self.nose, self.crossing = median, nose, crossing

  def edge(self, d: float) -> float:
    curb = LANE / 2 + self.gutter
    start, end = self.median
    if end <= d <= start:
      return curb
    if end - self.nose < d < end:
      return curb + self.width * (end - d) / self.nose
    return curb + self.width

  def message(self, d: float, offset: float):
    msg = super().message(d, offset)
    if self.median[1] - self.nose < d < self.median[1]:
      msg.roadEdgeStds = [0.6, msg.roadEdgeStds[1]]
    if self.crossing is not None and self.crossing[1] <= d <= self.crossing[0]:
      msg.laneLineProbs, msg.roadEdgeStds = [0.05] * 4, [0.9, 0.9]
    return msg


class Crossing(Road):
  """Through an intersection, turning: no lane lines, both edges unsure."""

  def message(self, d: float, offset: float):
    msg = super().message(d, offset)
    msg.laneLineProbs, msg.roadEdgeStds = [0.05] * 4, [0.9, 0.9]
    return msg


def turn_lane_helper(lanes=True, far=True, turns=True, auto=True, lane_change=AutoLaneChangeMode.NUDGE):
  """DesireHelper with route turns, automatic turns, automatic far side turns and automatic turn lanes as set, read from
  params as the drive starts, from parked."""
  params = Params()
  for key, value in [('NavTurnDesire', turns), ('NavExitDesire', False), ('NavAutoTurn', auto), ('NavAutoTurnFarSide', far),
                     ('NavAutoTurnLane', lanes), ('LaneTurnDesire', False)]:
    params.put_bool(key, value)
  params.put('LaneTurnValue', 20.)
  params.put('AutoLaneChangeTimer', int(lane_change))
  return DesireHelper()


class Approach:
  """A drive up to the route's turn through the real DesireHelper, on the road's model messages and real route, car and
  plan messages. The car's speed follows the route turn's slowdown as plannerd has it while openpilot controls the speed
  (unless follow=False: something ahead holds the car at the set speed), and the driving model moves the car over as a
  lane change desire's rising edge asks, LANE_CHANGE_TIME to the next lane's center, its lane change probability high
  meanwhile; the driver steering against it brings the car back. The route turns `modifier`, toward the turn lane's side
  unless given, `position` m before the destination (a new position is another maneuver). driver_alert: whether driver
  monitoring warns, or a function of the drive giving it."""

  def __init__(self, road, dh=None, mph=25., start=220., modifier=None, maneuver_type='turn', lanes=(), limit=MPH35, rht=True,
               long_active=True, driver_alert=False, follow=True, navd=None, navd_error=None):
    self.road, self.dh = road, dh if dh is not None else turn_lane_helper()
    self.v_set = mph * MPH
    self.d, self.v, self.a, self.a_plan, self.offset, self.t = start, self.v_set, 0., 0., 0., 0.
    self.modifier = modifier if modifier is not None else ('left' if road.left else 'right')
    self.maneuver_type, self.lanes, self.limit, self.rht, self.position = maneuver_type, lanes, limit, rht, 1000.
    self.long_active, self.driver_alert, self.follow = long_active, driver_alert, follow
    self.navd = navd or (lambda s: True)  # whether navd has a fix and a route now
    self.navd_error = navd_error or (lambda s: 0.)  # m navd's distance to the maneuver is off by
    self.moving: tuple[float, float, float] | None = None  # the model's lane change: (start time, from, to)
    self.prev_desire = log.Desire.none
    self.frames: list[dict] = []

  def submaster(self, gas: bool):
    instruction = messaging.new_message('navInstruction').navInstruction
    instruction.maneuverType, instruction.maneuverModifier = self.maneuver_type, self.modifier
    reported = self.d + self.navd_error(self)
    instruction.maneuverDistance, instruction.distanceRemaining = reported, reported + self.position
    instruction.lanes = list(self.lanes)
    car_control = messaging.new_message('carControl').carControl
    car_control.longActive, car_control.actuators.accel = self.long_active and not gas, self.a_plan
    dm = messaging.new_message('driverMonitoringState').driverMonitoringState
    dm.isRHD, dm.alertLevel = not self.rht, 'one' if self.warning() else 'none'
    plan = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP
    plan.speedLimit.resolver.speedLimit, plan.speedLimit.resolver.speedLimitValid = self.limit, self.limit > 0.
    now = time.monotonic()
    return FakeSubMaster(seen={'navInstruction': True, 'longitudinalPlanSP': True}, valid={'navInstruction': self.navd(self), 'longitudinalPlanSP': True},
                         recv_time={'navInstruction': now, 'longitudinalPlanSP': now},
                         data={'navInstruction': instruction, 'carControl': car_control, 'carState': car_state(aEgo=self.a),
                               'driverMonitoringState': dm, 'longitudinalPlanSP': plan})

  def warning(self) -> bool:
    """Whether driver monitoring warns now."""
    return bool(self.driver_alert(self) if callable(self.driver_alert) else self.driver_alert)

  def run(self, seconds=40., car=None, lateral=None, edge_block=None, until=lambda s: s.d < -5.):
    """Drive on for `seconds`, or until until(self). car(self) gives the car's state fields, lateral(self) whether steering
    is on, edge_block(self) the road edge lane change block, (left, right). Returns this stretch's frames."""
    first = len(self.frames)
    for _ in range(round(seconds / DT_MDL)):
      if until is not None and until(self):
        break
      fields = car(self) if car is not None else {}
      gas = bool(fields.get('gasPressed', False))
      # the route message just in: its age, microseconds of the test's own time, would make no two drives alike
      nav = dataclasses.replace(nav_inputs(self.submaster(gas), self.road.message(self.d, self.offset), True), age=0.)
      carstate = car_state(**({'vEgo': self.v, 'aEgo': self.a, 'standstill': self.v < 0.01} | fields))
      blocks = edge_block(self) if edge_block is not None else (False, False)
      self.dh.update(carstate, lateral(self) if lateral is not None else True, 1. if self.moving is not None else 0., blocks[0], blocks[1], nav)

      # the driving model: a lane change desire's rising edge moves the car a lane over that way, and the driver steering
      # against it brings it back
      desire = self.dh.desire
      if desire in LANE_CHANGE_DESIRES and desire != self.prev_desire and self.moving is None:
        toward = 1. if (desire == log.Desire.laneChangeLeft) == self.road.left else -1.
        self.moving = (self.t, self.offset, self.offset + toward * (LANE / 2 + self.road.width / 2))
      if self.moving is not None and carstate.steeringPressed and self.moving[2] != 0.:
        moving_left = (self.moving[2] > self.moving[1]) == self.road.left
        if (carstate.steeringTorque > 0.) != moving_left:
          self.moving = (self.t, self.offset, 0.)
      self.prev_desire = desire
      if self.moving is not None:
        start, origin, target = self.moving
        f = min((self.t + DT_MDL - start) / LANE_CHANGE_TIME, 1.)
        self.offset = origin + (target - origin) * (1. - math.cos(math.pi * f)) / 2.
        if f >= 1.:
          self.moving = None
      c = self.dh.nav_desire
      self.frames.append({'d': self.d, 'v': self.v, 't': self.t, 'desire': desire, 'state': self.dh.lane_change_state,
                          'direction': self.dh.lane_change_direction, 'turn_lane': self.dh.turn_lane_change is not None, 'offset': self.offset,
                          'carry': c.lane_carry, 'limit': c.speed_limit, 'prompt': c.signal_prompt, 'across': c.turn_across,
                          'turn': self.dh.lane_turn_direction, 'dm': self.warning()})

      # the car's speed: the set speed, the route turn's slowdown as plannerd follows it, or the driver's gas
      wanted = [float(np.clip(self.v_set - self.v, -0.5, 1.))]
      if self.follow and self.long_active and not gas and c.speed_limit > 0.:
        decel = NAV_TURN_DECEL if c.speed_limit > NAV_TURN_SPEED else 0.
        wanted.append(max(float(np.clip(c.speed_limit - self.v - decel, -NAV_TURN_MAX_DECEL, 1.)), self.a_plan - 1.5 * DT_MDL))
      self.a_plan = min(wanted) if not gas else 0.5
      self.a += (self.a_plan - self.a) * DT_MDL / 0.3
      self.v = max(self.v + self.a * DT_MDL, 0.)
      self.d -= self.v * DT_MDL
      self.t += DT_MDL
    return self.frames[first:]


def rises(frames, values, key='desire') -> list[dict]:
  """The frames where frame[key] rises into one of the values: each one a new command to the driving model."""
  return [f for f, p in zip(frames, [None, *frames][:len(frames)], strict=True) if f[key] in values and (p is None or p[key] != f[key])]


def moves(frames) -> list[dict]:
  """The frames where the route's own lane change into a turn lane starts."""
  return [f for f, p in zip(frames, [None, *frames][:len(frames)], strict=True) if f['turn_lane'] and (p is None or not p['turn_lane'])]


def since_move(approach) -> float:
  """s since the route's lane change into a turn lane started on this drive, or -1 before."""
  start = next((f['t'] for f in moves(approach.frames)), None)
  return approach.t - start if start is not None else -1.


def outputs(frame) -> tuple:
  return (frame['desire'], frame['state'], frame['direction'], frame['turn'], round(frame['limit'], 9), frame['prompt'], frame['across'],
          frame['turn_lane'])


class TestTurnLaneView(OpenpilotTestCase):
  def view(self, road=None, d=60., offset=0.):
    return road_view((road or Road()).message(d, offset))

  def test_road_view(self):
    view = self.view()
    assert view.edge_x[1] == tuple(float(x) for x in X) and len(view.edge_y) == 2 and len(view.line_y) == 4 and len(view.line_probs) == 4
    assert math.isclose(view.edge_y[1][0], LANE / 2 + 0.2 + POCKET, rel_tol=1e-6) and math.isclose(view.line_y[2], LANE / 2, rel_tol=1e-6)
    # no model output yet, a short message, or none of it a model message at all: nothing, and never an exception
    assert road_view(messaging.new_message('modelV2').modelV2) == RoadView()
    msg = Road().message(60., 0.)
    msg.laneLineProbs = [0.9, 0.9]
    assert road_view(msg) == RoadView()
    assert road_view(None) == RoadView() and road_view(object()) == RoadView()

  @parameterized.expand([
    ('a turn lane', {}, {}, True),
    ('a bike lane', {'width': 1.5}, {}, False),
    ('a shoulder', {'width': 2.2}, {}, False),
    ('a turn lane just wide enough', {'width': POCKET_WIDTH}, {}, True),
    ('a bus bay, ending within the lookahead', {'length': 20.}, {}, False),
    ('a lane line beyond it', {'beyond_prob': 0.6}, {}, False),
    ('its lane line unsure', {'line_prob': 0.4}, {}, False),
    ('the edge unsure', {'edge_std': 0.6}, {}, False),
    ('the left side, for a left turn lane', {'left': True}, {}, True),
    ('the other side', {'left': True}, {'left': False}, False),
    ('not yet open beside the car', {}, {'d': 99.}, False),
    ('not quite as far out as the baseline needs', {}, {'baseline': 2.6}, False),
    ('no baseline', {}, {'baseline': math.nan}, False),
    ('the lookahead beyond the model', {}, {'lookahead': 200.}, False),
  ])
  def test_turn_lane_open(self, _name, road, test, expected):
    road = Road(**road)
    test = {'left': road.left, 'd': 60., 'baseline': LANE / 2 + 0.2, 'lookahead': POCKET_LOOKAHEAD} | test
    view = road_view(road.message(test['d'], 0.))
    assert turn_lane_open(view, test['left'], test['baseline'], test['lookahead']) == expected

  def test_turn_lane_open_line_distance(self):
    # the lane line between the car and the turn lane 1.0-2.4 m from the car's center: nearer, the car is not in its lane
    # (or on the line), farther, there is room for something between
    for offset, expected in ((0., True), (-0.6, True), (0.8, False), (-0.8, False)):
      view = road_view(Road().message(60., offset))
      assert turn_lane_open(view, False, LANE / 2 + 0.2 - offset, POCKET_LOOKAHEAD) == expected, offset

  def test_turn_lane_open_gap_from_the_line(self):
    # the edge out a turn lane's width beyond where it was, but the lane line farther out too (the car's lane wider where the
    # turn lane opens): what lies beyond the line is narrower than a turn lane, a bike or parking lane
    def view(edge, line):
      xs = tuple(float(x) for x in X)
      return RoadView(edge_x=(xs, xs), edge_y=((-8.5,) * len(X), (edge,) * len(X)), edge_stds=(0.3, 0.3),
                      line_y=(-5.1, -1.7, line, 7.), line_probs=(0.9, 0.9, 0.9, 0.05))
    assert turn_lane_open(view(4.6, 1.7), False, 1.5, 30.)
    assert not turn_lane_open(view(4.3, 2.2), False, 1.5, 30.)  # 2.8 m out from where it was, 2.1 m from the line

  def test_turn_lane_open_malformed(self):
    good = self.view()

    def with_(**fields):
      return RoadView(**({name: getattr(good, name) for name in ('edge_x', 'edge_y', 'edge_stds', 'line_y', 'line_probs')} | fields))
    assert turn_lane_open(with_(), False, 1.9, 30.)
    bad_y = list(good.edge_y[1])
    bad_y[25] = math.nan  # 117 m ahead: beyond the lookahead, but the message is broken
    swapped_x = list(good.edge_x[1])
    swapped_x[3], swapped_x[4] = swapped_x[4], swapped_x[3]
    for view, why in [
      (with_(edge_y=(good.edge_y[0], tuple(bad_y))), 'a point of the edge NaN'),
      (with_(edge_y=(good.edge_y[0], good.edge_y[1][:10])), 'fewer points than distances'),
      (with_(edge_x=(good.edge_x[0], tuple(reversed(good.edge_x[1])))), 'distances not increasing'),
      (with_(edge_x=(good.edge_x[0], tuple(swapped_x))), 'two distances out of order'),
      (with_(edge_x=(good.edge_x[0], (math.inf,) + good.edge_x[1][1:])), 'an infinite distance'),
      (with_(edge_stds=(0.3, math.nan)), 'the edge NaN sure'),
      (with_(edge_stds=(0.3, -0.1)), 'a negative uncertainty'),
      (with_(line_y=good.line_y[:2] + (math.nan,) + good.line_y[3:]), 'the lane line nowhere'),
      (with_(line_probs=good.line_probs[:2] + (math.nan,) + good.line_probs[3:]), 'its probability NaN'),
      (with_(line_probs=good.line_probs[:2] + (1.5,) + good.line_probs[3:]), 'its probability above 1'),
      (with_(line_probs=good.line_probs[:3] + (math.nan,)), 'the line beyond NaN'),
      (with_(line_probs=good.line_probs[:3] + (-0.5,)), 'the line beyond negative'),
      (with_(edge_x=(), edge_y=()), 'no edges'),
      (with_(line_y=(), line_probs=()), 'no lines'),
      (RoadView(), 'nothing'),
    ]:
      assert not turn_lane_open(view, False, 1.9, 30.), why
    assert not turn_lane_open(good, False, 1.9, math.nan)

  def test_curb_and_line_distances(self):
    view = self.view(Road(opens=None))
    assert math.isclose(curb_distance(view, False), LANE / 2 + 0.2, rel_tol=1e-6) and math.isnan(curb_distance(view, True))  # the other edge is far
    assert math.isclose(lane_line_distance(view, False), LANE / 2, rel_tol=1e-6) and math.isclose(lane_line_distance(view, True), LANE / 2, rel_tol=1e-6)
    assert math.isnan(curb_distance(self.view(Road(opens=None), offset=1.5), False))  # under the car
    assert math.isnan(curb_distance(self.view(Road(opens=None), offset=-1.5), False))  # a lane's width away
    left = self.view(Road(left=True, opens=None))
    assert math.isclose(curb_distance(left, True), LANE / 2 + 0.2, rel_tol=1e-6) and math.isnan(curb_distance(left, False))
    for view in (RoadView(), RoadView(edge_y=((), ())), RoadView(edge_y=((-1.9,), (math.nan,)))):
      assert math.isnan(curb_distance(view, False)) and math.isnan(lane_line_distance(view, False))
    assert math.isnan(curb_distance(RoadView(edge_y=((-6.,), (-1.9,))), False))  # the right edge read on the left

  def test_nav_inputs_carry_the_view(self):
    approach = Approach(Road())
    msg = Road().message(60., 0.)
    assert nav_inputs(approach.submaster(False), msg, True).road == road_view(msg)
    assert nav_inputs(approach.submaster(False), messaging.new_message('modelV2').modelV2, True).road == RoadView()


class TestTurnLaneMoves(OpenpilotTestCase):
  def one_move_and_turn(self, frames, left, across):
    """One lane change into the turn lane, toward the turn, with its alert while it lasts, done POCKET_MARGIN or more before
    the turn starts, and then one turn command, from the turn lane."""
    changes, turns = rises(frames, LANE_CHANGE_DESIRES), rises(frames, TURN_DESIRES)
    assert [f['desire'] for f in changes] == [log.Desire.laneChangeLeft if left else log.Desire.laneChangeRight], changes
    assert [f['desire'] for f in turns] == [log.Desire.turnLeft if left else log.Desire.turnRight], turns
    moving = [f for f in frames if f['turn_lane']]
    assert len(moves(frames)) == 1 and all(f['turn_lane'] == (f['state'] == LaneChangeState.laneChangeStarting) for f in frames)
    assert moving[0] is changes[0] and moving[-1]['d'] >= turns[0]['d'] + POCKET_MARGIN
    assert math.isclose(turns[0]['offset'], LANE / 2 + POCKET / 2, abs_tol=1e-6) and turns[0]['carry'] == left
    assert turns[0]['d'] > NAV_TURN_STEER_DISTANCE and turns[0]['v'] <= 7.
    assert not any(f['prompt'] for f in frames)
    assert all(f['across'] == (across and f['turn'] != 0) for f in frames if f['d'] > 0.)

  # a turn lane open by 70 m before the turn at 20 mph, 80 m at 25-30 mph: the move, 3.5 s at the car's speed, ends before
  # the turn's start, as the slowdown will time it, with 5 m to spare (one opening later is in TestNoTurnLaneMove)
  @parameterized.expand([(20, 70.)] + [(mph, opens) for mph in (20, 25, 30) for opens in (80., 90., 120.)])
  def test_right_turn_lane(self, mph, opens):
    frames = Approach(Road(opens=opens), mph=mph, lanes=RIGHT_LANE).run()
    self.one_move_and_turn(frames, left=False, across=False)

  def test_right_turn_lane_without_lane_guidance(self):
    self.one_move_and_turn(Approach(Road()).run(), left=False, across=False)

  def test_sharp_right(self):
    self.one_move_and_turn(Approach(Road(), modifier='sharp right').run(), left=False, across=False)

  def test_slow_traffic(self):
    # crawling up to the turn lane at 6 mph, the move ends long before the turn
    self.one_move_and_turn(Approach(Road(opens=60.), mph=6., start=120.).run(60.), left=False, across=False)

  def test_long_turn_lane_within_the_memory(self):
    # a turn lane opening 185 m out: the car moves over once within POCKET_MAX_DISTANCE of the turn, while the curb lane it
    # left at the taper is still within POCKET_MEMORY behind it
    frames = Approach(Road(opens=185.), mph=25., start=260.).run(60.)
    self.one_move_and_turn(frames, left=False, across=False)
    assert moves(frames)[0]['d'] <= POCKET_MAX_DISTANCE

  def test_blind_spot_on_the_other_side(self):
    self.one_move_and_turn(Approach(Road()).run(car=lambda s: {'leftBlindspot': True}), left=False, across=False)

  def test_turn_lane_alert_state(self):
    # navTurnLane only while the route's own lane change runs, and the car's own lane change toward the turn never raises it
    approach = Approach(Road())
    frames = approach.run()
    states = {f['state'] for f in frames if f['turn_lane']}
    assert states == {LaneChangeState.laneChangeStarting} and {f['direction'] for f in frames if f['turn_lane']} == {log.LaneChangeDirection.right}
    nudged = Approach(Road(opens=None), dh=turn_lane_helper(), mph=25.)
    frames = nudged.run(car=lambda s: {'rightBlinker': True, 'steeringPressed': s.d < 150., 'steeringTorque': -1.}, until=lambda s: s.d < 100.)
    assert any(f['state'] == LaneChangeState.laneChangeStarting for f in frames) and not any(f['turn_lane'] for f in frames)


class TestNoTurnLaneMove(OpenpilotTestCase):
  @parameterized.expand([
    # (name, road, approach, helper settings, run: car fields as a function of the drive, steering, edge block)
    ('setting off', {}, {}, {'lanes': False}, {}),
    ('automatic route turns off', {}, {}, {'auto': False}, {}),
    ('automatic lane changes off', {}, {}, {'lane_change': AutoLaneChangeMode.OFF}, {}),
    ('far side without automatic left turns', {'left': True}, {'lanes': LEFT_BAY}, {'far': False}, {}),
    ('far side without lane guidance', {'left': True}, {}, {}, {}),
    ('far side on a two-way road: no curb beside the car', {'left': True, 'gutter': LANE + 0.2}, {'lanes': LEFT_BAY}, {}, {}),
    ('far side, a painted bay: no curb beside it either', {'left': True, 'gutter': LANE + 0.2, 'opens': 100.}, {'lanes': LEFT_BAY}, {}, {}),
    ('far side, posted limit unknown', {'left': True}, {'lanes': LEFT_BAY, 'limit': 0.}, {}, {}),
    ('far side, posted 50 mph', {'left': True}, {'lanes': LEFT_BAY, 'limit': MPH50}, {}, {}),
    ('far side, driver monitoring warning', {'left': True}, {'lanes': LEFT_BAY, 'driver_alert': True}, {}, {}),
    ('far side, sharp left', {'left': True}, {'lanes': LEFT_BAY, 'modifier': 'sharp left'}, {}, {}),
    ('far side, end of road', {'left': True}, {'lanes': LEFT_BAY, 'maneuver_type': 'end of road'}, {}, {}),
    ('far side, guidance puts the turn in another lane', {'left': True}, {'lanes': LEFT_FROM_ANOTHER_LANE}, {}, {}),
    ('a bike lane', {'width': 1.5}, {}, {}, {}),
    ('a shoulder', {'width': 2.2}, {}, {}, {}),
    ('a bus bay shorter than the lookahead', {'length': 20.}, {}, {}, {}),
    ('a lane line beyond the turn lane', {'beyond_prob': 0.6}, {}, {}, {}),
    ('guidance puts the turn in another lane', {}, {'lanes': RIGHT_FROM_ANOTHER_LANE}, {}, {}),
    ('a car in that blind spot', {}, {}, {}, {'car': lambda s: {'rightBlindspot': True}}),
    ('the road edge lane change block', {}, {}, {}, {'edge_block': lambda s: (False, True)}),
    ('the blinker toward it', {}, {}, {}, {'car': lambda s: {'rightBlinker': True}}),
    ('the other blinker', {}, {}, {}, {'car': lambda s: {'leftBlinker': True}}),
    ('hazards', {}, {}, {}, {'car': lambda s: {'leftBlinker': True, 'rightBlinker': True}}),
    ('called off with the gas', {}, {}, {}, {'car': lambda s: {'gasPressed': 135. < s.d <= 140.}}),
    ("the turn's blinker switched off: its automatic turn called off", {}, {}, {}, {'car': lambda s: {'rightBlinker': 140. < s.d <= 145.}}),
    ('no route from navd as the turn lane opens', {}, {'navd': lambda s: not 70. < s.d <= 101.}, {}, {}),
    ('navd puts the turn farther', {}, {'navd_error': lambda s: 14. if s.d <= 105. else 0.}, {}, {}),
    ("the car's own cruise control", {}, {'long_active': False}, {}, {}),
    ("the car's own cruise control, at a crawl beyond the approach", {'opens': 120.}, {'long_active': False, 'mph': 6., 'start': 180., 'follow': False}, {},
     {'until': lambda s: s.d < 60.}),
    ('the blinker toward it, below 20 mph', {}, {'mph': 18.}, {}, {'car': lambda s: {'rightBlinker': True}}),
    ('steering off', {}, {}, {}, {'lateral': lambda s: False}),
    ('the turn lane opening too late', {'opens': 60.}, {}, {}, {}),
    ('the turn lane opening too late, slower', {'opens': 60.}, {'mph': 20.}, {}, {}),
    ('never confirmed in the curb lane: a gutter too wide', {'gutter': 1.2}, {}, {}, {}),
    ('just in the curb lane', {}, {'start': 105.}, {}, {}),
    # (the curb lane test passes some 3 m into the taper, so 13 m in 0.9 s)
    ('in the curb lane for 13 m, but not a second yet', {'opens': 120.}, {'start': 145., 'mph': 33., 'follow': False}, {}, {}),
    ('far side, beside the median for 8 m', {'left': True, 'opens': 110.}, {'lanes': LEFT_BAY, 'start': 133.}, {}, {}),
    ('a turn lane on the other side', {'left': True}, {'modifier': 'right'}, {}, {}),
    ('a turn the other way', {}, {'modifier': 'left'}, {'far': True}, {}),
    ('going straight on', {}, {'modifier': 'straight'}, {}, {}),
    ('an exit', {}, {'maneuver_type': 'off ramp', 'modifier': 'slight right'}, {}, {}),
    ('too far from the turn', {'opens': 400.}, {'start': 480., 'mph': 25.}, {}, {'until': lambda s: s.d < POCKET_MAX_DISTANCE + 5.}),
    # the turn lane open beside the car from 250 m out: an open turn lane does not keep the curb lane in memory, which is
    # 100 m behind the car by the time it is near enough
    ('a long turn lane: the curb lane too far behind', {'opens': 250.}, {'start': 320.}, {}, {}),
    # the move's chime is driver monitoring's first warning's: none while that warns, nor for POCKET_DM_QUIET after
    ('driver monitoring warning', {}, {'driver_alert': True}, {}, {}),
    ('driver monitoring warning until the turn lane opens, quiet too late', {}, {'driver_alert': lambda s: s.d > 100.}, {}, {}),
  ])
  def test_no_move(self, name, road, approach, settings, run):
    approach = {'mph': 25.} | approach
    frames = Approach(Road(**road), dh=turn_lane_helper(**settings), **approach).run(**({'until': lambda s: s.d < -5.} | run))
    assert not moves(frames) and not any(f['turn_lane'] for f in frames), name
    assert not rises(frames, LANE_CHANGE_DESIRES) or name == 'the blinker toward it', name
    assert len(rises(frames, TURN_DESIRES)) <= 1
    # where it does not move over, every output is the same as with the setting off: today's behavior
    if settings.get('lanes', True):
      off = Approach(Road(**road), dh=turn_lane_helper(**(settings | {'lanes': False})), **approach).run(**({'until': lambda s: s.d < -5.} | run))
      assert [outputs(f) for f in frames] == [outputs(f) for f in off], name

  def test_too_fast(self):
    # held at the set speed by something ahead: at 34 mph it moves over, at 35 mph, above POCKET_MAX_SPEED, it does not
    for mph, expected in ((33.5, 1), (35., 0)):
      frames = Approach(Road(opens=135.), mph=mph, start=260., follow=False).run(until=lambda s: s.d < 60.)
      assert len(moves(frames)) == expected, mph

  def test_too_slow(self):
    # crawling in a queue at 6 mph it moves over, at 5 mph, below POCKET_MIN_SPEED, it does not
    for mph, expected in ((6., 1), (5., 0)):
      frames = Approach(Road(opens=80.), mph=mph, start=110., follow=False).run(60., until=lambda s: s.d < 40.)
      assert len(moves(frames)) == expected, mph

  def test_pocket_must_hold(self):
    # open for less than POCKET_TIME at a time (a reading that comes and goes): no move
    class Flicker(Road):
      def message(self, d, offset):
        return super().message(d if int(d / 4.) % 2 else 200., offset)
    frames = Approach(Flicker(), mph=25.).run()
    assert not moves(frames)

  def test_lookahead_near_the_turn(self):
    # crawling, the move can still come within 40 m of the turn: there the turn lane needs to hold only to 10 m before the
    # maneuver point, here a bay ending 8 m before it
    frames = Approach(Road(opens=33., length=25.), mph=6., start=80., follow=False).run(60., until=lambda s: s.d < 15.)
    assert len(moves(frames)) == 1 and moves(frames)[0]['d'] < POCKET_LOOKAHEAD + 10.

  def test_one_move_per_turn(self):
    # a move into a first bay ended by steering against it, the car back in its lane, the curb lane seen again after the bay,
    # then a second bay for the same turn: no second move, though the second bay is open beside the confirmed curb lane, as
    # long as a move would need, and while one could still come
    class TwoBays(Road):
      def edge(self, d):
        return max(Road(opens=135., length=35.).edge(d), Road(opens=70.).edge(d))
    approach = Approach(TwoBays(), mph=6., start=170., follow=False)

    def car(s):
      return {'steeringPressed': True, 'steeringTorque': 1.} if 0.2 <= since_move(s) < 0.6 else {}
    frames = approach.run(60., car=car, until=lambda s: s.d < 60.)
    c = approach.dh.nav_desire
    assert len(moves(frames)) == 1 and c.curb_confirmed[1] and c.pocket_time[1] >= 1.
    frames += approach.run(60., car=car, until=lambda s: s.d < 30.)
    assert len(moves(frames)) == 1

  def test_turn_command_in_the_desire_history(self):
    # the last turn's command, made just before this turn's turn lane opens (two turns a block apart), still in the model's
    # desire history: no lane change command on top of it, until it is out of it, by which time the turn lane is too close
    for last_turn, expected in ((False, 1), (True, 0)):
      approach = Approach(Road(), mph=25.)
      frames = approach.run(until=lambda s: s.d < 101.)
      if last_turn:
        c = approach.dh.nav_desire
        c.pulsed_maneuver, c.pulse_frame = c.maneuver_id - 1, c.frame
      frames += approach.run()
      assert len(moves(frames)) == expected, last_turn

  def test_turn_lane_not_the_outermost(self):
    # in the turn lane, the model sees a lane line beyond it after all (a second turn lane opening there): the car is not in
    # the outermost lane, so the turn does not start by itself from there
    frames = Approach(Road(beyond_inside=0.6)).run()
    assert len(moves(frames)) == 1 and sum(f['carry'] is not None for f in frames) <= 1
    assert not rises(frames, TURN_DESIRES) and not any(f['prompt'] for f in frames)

  def test_stays_in_lane_after_a_late_turn_lane(self):
    # no move: the car keeps to its lane, and with the curb lane gone beside it, no automatic right turn from there, as today
    approach = Approach(Road(opens=60.), mph=25.)
    frames = approach.run()
    assert not rises(frames, LANE_CHANGE_DESIRES + TURN_DESIRES) and frames[-1]['offset'] == 0. and any(f['limit'] for f in frames)

  def test_far_side_prompts_without_a_move(self):
    # a left turn bay opening too late: the car stays beside it, the curb gone, and asks for the blinker, as today
    frames = Approach(Road(left=True, opens=60.), lanes=LEFT_BAY).run()
    assert not moves(frames) and any(f['prompt'] for f in frames) and not rises(frames, TURN_DESIRES)


def no_move_as_today(test, approach_args, road, run=None):
  """Drives the approach with the setting on and off: no route lane change and no lane change command at all, at most one
  turn command, and every output the same as with the setting off, frame by frame. Returns the frames."""
  run = {'seconds': 60.} | (run or {})
  frames = Approach(road(), **approach_args).run(**run)
  off = Approach(road(), dh=turn_lane_helper(lanes=False), **approach_args).run(**run)
  assert not moves(frames) and not rises(frames, LANE_CHANGE_DESIRES), test
  assert len(rises(frames, TURN_DESIRES)) <= 1, test
  assert [outputs(f) for f in frames] == [outputs(f) for f in off], test
  return frames


def two_turns(left, b_road, a_lanes=(), b_lanes=(), crossing=30., b_dist=150., car=None):
  """Turn A from the outermost lane on its side (beside a raised median for a far side turn, in the curb lane for a near
  side one), `crossing` m through its intersection, then B's street, a new maneuver `b_dist` m on, as navd reports once the
  car leaves A's intersection. Returns the drive's frames up to B's street, and from there."""
  approach = Approach(Road(left=left, opens=None), mph=25., start=200., lanes=a_lanes)
  a = approach.run(60., car=car, until=lambda s: s.d < 0.)
  approach.road, approach.position, approach.d = Crossing(left=left, opens=None), 600., b_dist + crossing
  a += approach.run(30., until=lambda s: s.d < b_dist)
  approach.road, approach.lanes = b_road, b_lanes
  return a, approach.run(60.)


class TestNoFarSideMove(OpenpilotTestCase):
  """Never a move on the far side (left where traffic keeps right): where a raised median or an island ends and the road
  goes on two-way, its oncoming lane looks to the model just like a curbed bay opening out of the median, and a lane change
  into it, and a turn from it, would be the worst there is. So the car stays beside a real bay too, as without the setting."""

  @parameterized.expand([(mph, opens) for mph in (20, 25, 30) for opens in (80., 90., 120.)])
  def test_curbed_bay(self, mph, opens):
    # a curbed bay opening out of the median, with the route's lane guidance for it: the car stays in its lane, and with the
    # median gone from beside it, the left turn asks for the blinker
    frames = no_move_as_today((mph, opens), {'mph': mph, 'lanes': LEFT_BAY}, lambda: Road(left=True, opens=opens))
    assert any(f['prompt'] for f in frames) and not rises(frames, TURN_DESIRES)

  def test_long_bay(self):
    no_move_as_today('long bay', {'mph': 25., 'start': 320., 'lanes': LEFT_BAY}, lambda: Road(left=True, opens=250.))

  @parameterized.expand([
    # (name, the road, the route's lane guidance, start, mph)
    ('a median ending 130 m out', TwoWay(median=(1e9, 130.)), SHARED_LEFT, 250., 25.),
    ('a median ending 110 m out', TwoWay(median=(1e9, 110.)), LEFT_ONLY, 230., 25.),
    ('a median ending 95 m out', TwoWay(median=(1e9, 95.)), LEFT_ONLY, 215., 25.),
    ('a median ending 110 m out, at 20 mph', TwoWay(median=(1e9, 110.)), LEFT_ONLY, 230., 20.),
    ('a median ending 120 m out, at 30 mph', TwoWay(median=(1e9, 120.)), SHARED_LEFT, 250., 30.),
    ('a median ending 110 m out, a 3 m oncoming lane, a faint far edge line', TwoWay(median=(1e9, 110.), width=3., beyond_prob=0.15),
     LEFT_ONLY, 230., 25.),
    ('a median ending 600 m out', TwoWay(median=(1e9, 600.)), LEFT_ONLY, 720., 25.),
    ('a median ending at an intersection 260 m out', TwoWay(median=(1e9, 262.), crossing=(262., 240.)), LEFT_ONLY, 330., 25.),
    ('a 25 m island 230 m out', TwoWay(median=(230., 205.), nose=2.), LEFT_ONLY, 330., 25.),
    ('a 20 m island 230 m out, at 20 mph', TwoWay(median=(230., 210.), nose=2.), LEFT_ONLY, 330., 20.),
    ('a 14 m island 230 m out, at 15 mph', TwoWay(median=(230., 216.), nose=2.), SHARED_LEFT, 330., 15.),
  ])
  def test_median_or_island_ends(self, name, road, lanes, start, mph):
    # the car beside the median or the island long enough for an automatic left turn's curb run, which then ends, the road
    # two-way from there: the double yellow line surely seen, the far curb a lane beyond it, nothing beyond that
    frames = no_move_as_today(name, {'mph': mph, 'start': start, 'lanes': lanes}, lambda: road, {'seconds': 90.})
    assert not rises(frames, TURN_DESIRES) and all(f['offset'] == 0. for f in frames)

  @parameterized.expand([('automatic', None), ('signaled', lambda s: {'leftBlinker': s.d < 60.})])
  def test_left_turn_onto_a_two_way_street(self, _name, car):
    # left turn A from beside a raised median, then left turn B 150 m along a two-way street with one oncoming lane: nothing
    # moves the car into B's oncoming lane, and B, with no curb beside the car, asks for the blinker
    a, b = two_turns(True, TwoWay(), a_lanes=LEFT_BAY, b_lanes=LEFT_ONLY, car=car)
    assert [f['desire'] for f in rises(a, TURN_DESIRES)] == [log.Desire.turnLeft]
    assert not moves(a + b) and not rises(b, LANE_CHANGE_DESIRES + TURN_DESIRES) and any(f['prompt'] for f in b)
    assert all(f['offset'] == 0. for f in b)

  def test_far_side_where_traffic_keeps_left(self):
    # where traffic keeps left the far side is the right: a right turn bay beside a median, with lane guidance for it
    lanes = guidance((('straight', 'left'), None), (('straight',), None), (('right',), 'right'))
    frames = no_move_as_today('right bay', {'rht': False, 'lanes': lanes}, Road)
    assert any(f['prompt'] for f in frames)


def gas_with_controls_late(s):
  """The gas from 1 s into the move for half a second, and controlsd giving openpilot's speed control back 0.1 s later."""
  s.long_active = not 1. <= since_move(s) < 1.6
  return {'gasPressed': 1. <= since_move(s) < 1.5}


def speed_control_off_a_moment(s):
  s.long_active = not 1. <= since_move(s) < 1.5
  return {}


class TestTurnLaneEnds(OpenpilotTestCase):
  def run_and_end(self, car=None, lateral=None, road=None, **approach):
    road = road or Road()
    approach = Approach(road, **({'mph': 25.} | approach))
    frames = approach.run(car=car, lateral=lateral)
    started = moves(frames)
    assert len(started) == 1 and len(rises(frames, LANE_CHANGE_DESIRES)) == 1  # never a second lane change command
    assert len(rises(frames, TURN_DESIRES)) <= 1
    return frames, started[0]

  def ended_early(self, frames, cause_t):
    """The route's lane change ended at the cause, never carried the outermost lane over, and no other started."""
    moving = [f for f in frames if f['turn_lane']]
    assert moving[-1]['t'] < cause_t and frames[[f['t'] for f in frames].index(moving[-1]['t']) + 1]['state'] == LaneChangeState.off
    assert not any(f['carry'] is not None for f in frames)

  @parameterized.expand([(0.3,), (0.6,)])
  def test_blind_spot_before_the_car_moved(self, after):
    # a car coming up in the turn lane while the car has not yet moved over: it ends the route's lane change, for good (the
    # driving model, which has its pulse, may still move the car over: only the driver steering against it stops that)
    def car(s):
      return {'rightBlindspot': since_move(s) >= after}
    frames, start = self.run_and_end(car)
    self.ended_early(frames, start['t'] + after + DT_MDL)
    assert all(f['offset'] < POCKET_MOVED for f in frames if f['t'] <= start['t'] + after)

  def test_blind_spot_after_the_car_moved(self):
    # once the car has moved toward the turn lane, it goes on: the lane change, and the turn from the turn lane
    frames, start = self.run_and_end(lambda s: {'rightBlindspot': 1.2 <= since_move(s) < 2.})
    assert any(f['carry'] is not None for f in frames) and len(rises(frames, TURN_DESIRES)) == 1

  @parameterized.expand([
    ('the blinker toward it', {'rightBlinker': True}),
    ('the other blinker', {'leftBlinker': True}),
    ('hazards', {'leftBlinker': True, 'rightBlinker': True}),
    ('the gas', {'gasPressed': True}),
    ('steering against it', {'steeringPressed': True, 'steeringTorque': 1.}),
  ])
  def test_driver_ends_it(self, _name, fields):
    def car(s):
      return fields if 1. <= since_move(s) < 1.5 else {}
    frames, start = self.run_and_end(car)
    self.ended_early(frames, start['t'] + 1. + DT_MDL)

  @parameterized.expand([
    ('the gas', lambda s: {'gasPressed': 1. <= since_move(s) < 1.5}),
    ('the other blinker', lambda s: {'leftBlinker': 1. <= since_move(s) < 1.5}),
    ('the other blinker, held to the corner', lambda s: {'leftBlinker': since_move(s) >= 1.}),
    ('the gas, controlsd late to give the speed control back', gas_with_controls_late),
    ('the speed control off for a moment', speed_control_off_a_moment),
  ])
  def test_calls_off_only_the_automatic_turn(self, _name, car):
    # once the route moved the car toward the turn lane, the driving model finishes that lane change unless the driver steers
    # against it: the gas, the other blinker or the speed control going off ends the route's lane change and calls off the
    # automatic turn, never the slowdown, so the car reaches the corner at turning speed, in the turn lane, and turns only
    # if the driver signals for it
    approach = Approach(Road(), mph=25.)
    frames = approach.run(car=car)
    c = approach.dh.nav_desire
    corner = min(frames, key=lambda f: abs(f['d']))
    assert len(moves(frames)) == 1 and len(rises(frames, LANE_CHANGE_DESIRES)) == 1 and not rises(frames, TURN_DESIRES)
    assert math.isclose(corner['offset'], LANE / 2 + POCKET / 2, abs_tol=1e-6) and corner['v'] <= NAV_TURN_SPEED + 0.05
    assert c.auto_off and not c.called_off and c.maneuver_id in c.pins

  def test_signaling_after_the_gas_takes_the_turn(self):
    # the gas called the automatic turn off once the car started moving over: signaling for the turn takes it, from the turn
    # lane, as any signaled turn
    approach = Approach(Road(), mph=25.)
    frames = approach.run(car=lambda s: {'gasPressed': 1. <= since_move(s) < 1.5, 'rightBlinker': s.d < 45.})
    turns = rises(frames, TURN_DESIRES)
    assert len(rises(frames, LANE_CHANGE_DESIRES)) == 1 and [f['desire'] for f in turns] == [log.Desire.turnRight]
    assert math.isclose(turns[0]['offset'], LANE / 2 + POCKET / 2, abs_tol=1e-6) and turns[0]['v'] <= 7.

  def test_ending_blinker_starts_no_lane_change(self):
    # with nudgeless lane changes, the blinker that ends the move, held on, starts no lane change: it was on before
    approach = Approach(Road(opens=120.), dh=turn_lane_helper(lane_change=AutoLaneChangeMode.NUDGELESS), mph=25., start=260.)
    frames = approach.run(car=lambda s: {'leftBlinker': True} if since_move(s) >= 1. else {}, until=lambda s: s.d < 70.)
    assert len(moves(frames)) == 1 and len(rises(frames, LANE_CHANGE_DESIRES)) == 1
    # switched off and on again, it is a new one: as without the setting, it changes lanes
    frames = approach.run(car=lambda s: {'leftBlinker': s.d < 66.}, until=lambda s: s.d < 45.)
    assert rises(frames, (log.Desire.laneChangeLeft,))

  def test_gas_before_controls_catch_up(self):
    # the gas ends the move on the frame the car reports it, before controlsd reports openpilot's speed control off
    approach = Approach(Road(), mph=25.)
    approach.run(until=lambda s: since_move(s) >= 1.)
    dh = approach.dh
    nav = dataclasses.replace(nav_inputs(approach.submaster(False), approach.road.message(approach.d, approach.offset), True), age=0.)
    assert nav.long_active and dh.turn_lane_change is not None
    dh.update(car_state(vEgo=approach.v, gasPressed=True), True, 1., False, False, nav)
    assert dh.turn_lane_change is None and dh.lane_change_state == LaneChangeState.off

  def test_steering_with_it_goes_on(self):
    frames, _ = self.run_and_end(lambda s: {'steeringPressed': True, 'steeringTorque': -1.} if 1. <= since_move(s) < 1.5 else {})
    assert any(f['carry'] is not None for f in frames)

  def test_speed_control_off(self):
    # the car's own cruise taking over, which calls the automatic turn off
    approach = Approach(Road(), mph=25.)
    frames = approach.run(until=lambda s: since_move(s) >= 1.)
    approach.long_active = False
    frames += approach.run()
    self.ended_early(frames, moves(frames)[0]['t'] + 1. + DT_MDL)

  def test_steering_off(self):
    frames, start = self.run_and_end(lateral=lambda s: not 1. <= since_move(s) < 1.5)
    self.ended_early(frames, start['t'] + 1. + DT_MDL)

  def test_navd_moves_on(self):
    # a reroute to another maneuver mid move: it ends, and nothing carries over to the new maneuver
    approach = Approach(Road(), mph=25.)
    frames = approach.run(until=lambda s: since_move(s) >= 1.)
    approach.modifier = 'left'
    frames += approach.run(until=lambda s: s.d < 20.)
    self.ended_early(frames, moves(frames)[0]['t'] + 1. + DT_MDL)
    assert not rises(frames, TURN_DESIRES)

  def test_controller_failure(self):
    # the route message unreadable mid move: DesireHelper resets the controller, the move ends, and none comes again
    class Unreadable:
      maneuverType, maneuverModifier, lanes = 'turn', 'right', []

      @property
      def maneuverDistance(self):
        raise ValueError("corrupt message")
    approach = Approach(Road(), mph=25.)
    frames = approach.run(until=lambda s: since_move(s) >= 1.)
    dh = approach.dh
    dh.update(car_state(vEgo=approach.v), True, 1., False, False, NavInputs(Unreadable(), long_active=True))
    assert dh.nav_desire_failed and dh.turn_lane_change is None and dh.lane_change_state == LaneChangeState.off
    after = approach.run()
    assert not any(f['turn_lane'] for f in after) and len(rises(frames + after, LANE_CHANGE_DESIRES)) == 1

  def test_never_a_second_move(self):
    # ended by the blinker, the turn lane still there, the blinker off again: no second move for this maneuver
    frames, _ = self.run_and_end(lambda s: {'leftBlinker': True} if 0.3 <= since_move(s) < 0.6 else {})
    assert len(moves(frames)) == 1

  def test_completion_needs_the_model(self):
    # the model never finishes the lane change, its probability high throughout: DesireHelper's own timeout ends the move,
    # LANE_CHANGE_TIME_MAX after it started, with nothing carried over and no lane change command after it
    class Stuck(Approach):
      def run(self, seconds=40., **kwargs):
        update = self.dh.update

        def stuck(carstate, lateral_active, prob, *args):
          return update(carstate, lateral_active, 1. if self.dh.turn_lane_change is not None else prob, *args)
        self.dh.update = stuck
        return super().run(seconds, **kwargs)
    frames = Stuck(Road(opens=140.), mph=25.).run()
    moving = [f for f in frames if f['turn_lane']]
    assert len(moves(frames)) == 1 and math.isclose(moving[-1]['t'] - moving[0]['t'], LANE_CHANGE_TIME_MAX, abs_tol=2 * DT_MDL)
    assert not any(f['carry'] is not None for f in frames) and len(rises(frames, LANE_CHANGE_DESIRES)) == 1


class TestTurnLaneLeftHandTraffic(OpenpilotTestCase):
  # where traffic keeps left the near side is the left: a left turn lane from the curb lane; the far side the right, where
  # it never moves (TestNoFarSideMove)
  def test_near_side_left(self):
    frames = Approach(Road(left=True), rht=False).run()
    TestTurnLaneMoves().one_move_and_turn(frames, left=True, across=False)

  @parameterized.expand([
    ('far side without automatic far side turns', {}, {'lanes': RIGHT_LANE}, {'far': False}),
    ('far side without lane guidance', {}, {}, {}),
    ('far side, guidance puts the turn in another lane', {}, {'lanes': RIGHT_FROM_ANOTHER_LANE}, {}),
    ('near side, a bike lane', {'left': True, 'width': 1.5}, {}, {}),
  ])
  def test_no_move(self, _name, road, approach, settings):
    frames = Approach(Road(**road), dh=turn_lane_helper(**settings), rht=False, **approach).run()
    assert not moves(frames) and not rises(frames, LANE_CHANGE_DESIRES)


class TestTurnLaneMemory(OpenpilotTestCase):
  def test_confirmation_needs_time_and_distance(self):
    # in the curb lane for a second but less than 10 m (creeping), then the turn lane: no move
    c = NavDesireController()
    c.turn_enabled = c.auto_enabled = c.lane_enabled = True
    c.param_read_counter = 1
    road = Road(opens=None)
    for _ in range(round(3. / DT_MDL)):
      msg = road.message(120., 0.)
      c.update(car_state(vEgo=2.), True, NavInputs(outer_lanes=nav_desire.outer_lanes(msg), road=road_view(msg)))
    assert c.outer_lane_time[1] >= 1. and c.outer_lane_distance[1] < 10. - 1e-6 and not c.curb_confirmed[1]
    for _ in range(round(3. / DT_MDL)):
      msg = road.message(120., 0.)
      c.update(car_state(vEgo=4.), True, NavInputs(outer_lanes=nav_desire.outer_lanes(msg), road=road_view(msg)))
    assert c.curb_confirmed[1] and math.isclose(c.curb_baseline[1], LANE / 2 + 0.2, rel_tol=1e-6)

  @parameterized.expand([('lane change', {'lane_changing': True}), ('steering off', {'lateral_active': False})])
  def test_forgotten(self, _name, update):
    c = NavDesireController()
    c.turn_enabled = c.auto_enabled = c.lane_enabled = True
    c.param_read_counter = 1
    road = Road(opens=None)

    def frame(lateral_active=True, lane_changing=False):
      msg = road.message(120., 0.)
      nav = NavInputs(outer_lanes=nav_desire.outer_lanes(msg), far_lanes=nav_desire.far_lanes(msg), road=road_view(msg))
      c.update(car_state(vEgo=8.), lateral_active, nav, lane_changing)
    for _ in range(round(3. / DT_MDL)):
      frame()
    assert c.curb_confirmed[1]
    frame(**update)
    assert not c.curb_confirmed[1] and math.isnan(c.curb_baseline[1])

  @parameterized.expand([(True,), (False,)])
  def test_never_on_the_far_side(self, rht):
    # beside a raised median on the far side for longer than an automatic far side turn's curb run needs: never confirmed
    # there, whichever side traffic keeps to; beside the curb on the near side, the same drive mirrored, it is
    for left, confirmed in ((rht, False), (not rht, True)):
      c = NavDesireController()
      c.turn_enabled = c.auto_enabled = c.lane_enabled = c.far_enabled = True
      c.param_read_counter = 1
      road = Road(left=left, opens=None)
      for _ in range(round(3. / DT_MDL)):
        msg = road.message(120., 0.)
        nav = NavInputs(outer_lanes=nav_desire.outer_lanes(msg), far_lanes=nav_desire.far_lanes(msg), right_hand_traffic=rht,
                        road=road_view(msg))
        c.update(car_state(vEgo=8.), True, nav)
      side = 0 if left else 1
      assert c.outer_lane_time[side] >= 1. and c.far_lane_time[side] >= 1.5 and c.curb_confirmed[side] == confirmed, (rht, left)

  def test_forgotten_once_the_turn_starts(self):
    # in the curb lane up to the turn, no turn lane: confirmed until the route's turn starts, and forgotten from then on
    approach = Approach(Road(opens=None), mph=25.)
    c = approach.dh.nav_desire
    approach.run(until=lambda s: s.dh.nav_desire.started)
    assert c.curb_confirmed[1]
    approach.run(1.)
    assert c.started and not c.curb_confirmed[1] and math.isnan(c.curb_baseline[1])

  @parameterized.expand([(3.4,), (2.7,)])
  def test_forgotten_for_the_next_maneuver(self, width):
    # right turn A from the curb lane, then right turn B 150 m along the next street, the car in that street's second lane, a
    # travel lane (3.4 m) or a marked parking lane (2.7 m) beyond its right lane line: the curb lane seen before A says
    # nothing about B's street, so the car stays in its lane there
    a, b = two_turns(False, Road(opens=1e5, taper=1., width=width))
    assert [f['desire'] for f in rises(a, TURN_DESIRES)] == [log.Desire.turnRight]
    assert not moves(a + b) and not rises(a + b, LANE_CHANGE_DESIRES) and all(f['offset'] == 0. for f in b)

  def test_forgotten_for_the_next_maneuver_after_no_turn(self):
    # in the curb lane up to a maneuver that is no turn (the road's name changes), so no turn starts to forget the curb lane
    # by, then the right turn 140 m on, the road wider from there, the car in its second lane beside a marked parking lane:
    # the curb lane seen for the maneuver before is not this one's
    approach = Approach(Road(opens=None), mph=25., start=200., maneuver_type='new name', modifier='straight')
    frames = approach.run(until=lambda s: s.d < 0.)
    assert approach.dh.nav_desire.curb_confirmed[1]
    approach.road, approach.position, approach.d = Road(opens=1e5, taper=1., width=2.7), 600., 140.
    approach.maneuver_type, approach.modifier = 'turn', 'right'
    frames += approach.run()
    assert not moves(frames) and not rises(frames, LANE_CHANGE_DESIRES) and all(f['offset'] == 0. for f in frames)

  def test_turn_lane_carried_for_its_maneuver_only(self):
    # the move done, the turn lane counts as the curb lane for that maneuver's turn; navd's next maneuver drops that
    approach = Approach(Road(), mph=25.)
    approach.run(until=lambda s: s.dh.nav_desire.lane_carry is not None)
    c = approach.dh.nav_desire
    assert c.lane_carry is False
    approach.position = 600.
    approach.run(DT_MDL)
    assert c.lane_carry is None

  def test_memory_ends(self):
    # the edge gone from beside the car for POCKET_MEMORY, with no turn lane open there: the curb lane is forgotten
    approach = Approach(Road(opens=None), mph=25., start=400.)
    approach.run(until=lambda s: s.d < 300.)
    c = approach.dh.nav_desire
    assert c.curb_confirmed[1]
    wide = Road(opens=None, gutter=2.)  # a parking lane starts: the edge out of reach, no lane line before it
    approach.road = wide
    approach.run(until=lambda s: s.d < 300. - POCKET_MEMORY + 2.)
    assert c.curb_confirmed[1]
    approach.run(until=lambda s: s.d < 300. - POCKET_MEMORY - 2.)
    assert not c.curb_confirmed[1]

  def test_baseline_through_a_taper(self):
    # the edge starting to move out as a long taper begins hardly moves where it was, so a 3 m turn lane still opens: from
    # where the edge last was before it left the curb lane test, it would not
    frames = Approach(Road(opens=100., taper=40., width=3.), mph=25., start=240.).run()
    assert len(moves(frames)) == 1

  def test_baseline_through_a_glitch(self):
    # the model reading the curb under the car for a moment, in the curb lane, here just as the taper starts: that reading
    # never counts as where the curb was
    class Glitch(Road):
      def message(self, d, offset):
        msg = super().message(d, offset)
        if 95. < d <= 98.5:
          msg.roadEdges[1].y = [0.5] * len(X)
        return msg
    assert len(moves(Approach(Glitch(), mph=25.).run())) == 1

  def test_forgotten_while_off(self):
    # confirmed in the curb lane, the setting switched off, the car a lane over by the time it is back on, at a stop: the
    # curb lane beside it now must not look like a turn lane opening there
    params = Params()
    approach = Approach(Road(opens=None, edge_line_prob=0.1), mph=25., start=400.)
    approach.run(until=lambda s: s.d < 300.)
    assert approach.dh.nav_desire.curb_confirmed[1]
    params.put_bool('NavAutoTurnLane', False)
    approach.run(until=lambda s: s.d < 270.)
    approach.offset, approach.v, approach.v_set = -LANE, 0., 0.
    params.put_bool('NavAutoTurnLane', True)
    approach.run(3.)
    assert approach.dh.nav_desire.lane_enabled
    approach.v_set = 25 * MPH
    assert not moves(approach.run())


class TestTurnLaneSetting(OpenpilotTestCase):
  def test_switched_on_only_at_a_standstill(self):
    params = Params()
    for key, value in (('NavTurnDesire', True), ('NavAutoTurn', True), ('NavAutoTurnFarSide', False), ('NavAutoTurnLane', False), ('NavExitDesire', False)):
      params.put_bool(key, value)
    dh = DesireHelper()

    def frames(seconds, v):
      for _ in range(round(seconds / DT_MDL)):
        dh.update(car_state(vEgo=v, standstill=v == 0.), True, 0., False, False, None)
    frames(3., 20.)
    params.put_bool('NavAutoTurnLane', True)
    frames(10., 20.)
    assert not dh.nav_desire.lane_enabled
    frames(3., 0.)
    assert dh.nav_desire.lane_enabled
    frames(3., 20.)
    assert dh.nav_desire.lane_enabled
    params.put_bool('NavAutoTurnLane', False)
    frames(3., 20.)
    assert not dh.nav_desire.lane_enabled
    params.put_bool('NavAutoTurnLane', True)
    frames(10., 20.)
    assert not dh.nav_desire.lane_enabled
    # as the drive starts, from parked, the settings count as they are
    assert DesireHelper().nav_desire.lane_enabled

  def test_needs_automatic_turns(self):
    params = Params()
    for key, value in (('NavTurnDesire', True), ('NavAutoTurn', False), ('NavAutoTurnLane', True)):
      params.put_bool(key, value)
    assert not NavDesireController().lane_enabled
    params.put_bool('NavAutoTurn', True)
    assert NavDesireController().lane_enabled

  def test_switched_off_while_moving(self):
    # switched off mid move: no new move starts, and the one under way, which the model already has, finishes with its
    # turn from the turn lane
    approach = Approach(Road(), mph=25.)
    frames = approach.run(until=lambda s: any(f['turn_lane'] for f in s.frames))
    Params().put_bool('NavAutoTurnLane', False)
    frames += approach.run()
    assert not approach.dh.nav_desire.lane_enabled
    TestTurnLaneMoves().one_move_and_turn(frames, left=False, across=False)


def random_drive(seed: int, seconds: float):
  """A random drive toward random route maneuvers, frame by frame: navd's message (noisy, now and then missing), the car's
  state (speed, blinkers, gas, blind spots, steering), steering and speed control dropping out, driver monitoring, posted
  limits, the road edge lane change block, the model's lane change probability, and a road with or without a turn lane on
  either side, the car moving over as a lane change desire asks. Every other drive is calm: all of that comes five times
  less often, and each turn has a turn lane, open in time, that lane guidance has it from. A generator: it yields each
  frame's (car state, steering on, lane change probability, edge block, route inputs, the model's road view), and takes
  the frame's desire back."""
  r = random.Random(seed)
  k = 0.2 if seed % 2 == 0 else 1.
  maneuvers = [('turn', 'right')] * 3 + [('turn', 'left')] * 3 + [('turn', 'sharp right'), ('end of road', 'left'), ('turn', 'straight'),
                                                                    ('off ramp', 'slight right'), ('roundabout', 'right'), ('turn', 'uturn')]
  guidances = [(), RIGHT_LANE, LEFT_BAY, RIGHT_FROM_ANOTHER_LANE, LEFT_FROM_ANOTHER_LANE]
  state = {'position': 8000., 'v': r.choice([3., 8., 11., 14.]), 'offset': 0., 'moving': None, 'msg': None, 'gap': 0, 'gas': 0}

  def new_maneuver():
    state['maneuver'], state['d'] = r.choice(maneuvers[:6] if k < 1. else maneuvers), r.uniform(120., 320.)
    state['position'] -= r.uniform(50., 400.)
    turn_left = state['maneuver'][1] in ('left', 'sharp left')
    if k < 1.:  # a turn lane for the turn, open in time, that guidance has the turn from
      state['lanes'], state['road'] = (LEFT_BAY if turn_left else RIGHT_LANE), Road(left=turn_left, opens=r.uniform(85., 140.))
    else:
      state['lanes'] = r.choice(guidances)
      state['road'] = Road(left=turn_left if r.random() < 0.8 else r.random() < 0.5, opens=r.choice([None, r.uniform(60., 160.), r.uniform(75., 140.)]),
                           width=r.choice([POCKET] * 3 + [1.5, 2.8]), length=r.choice([None] * 3 + [25.]), beyond_prob=r.choice([0.05] * 3 + [0.5]))
    state['offset'], state['moving'] = 0., None
  new_maneuver()
  state.update(v_target=state['v'], blinkers=(False, False), lat=True, long=True, bs=[0, 0], limit=MPH35, dm=False, edge=(False, False))
  for i in range(round(seconds / DT_MDL)):
    if r.random() < 0.01:
      state['v_target'] = r.choice([6., 8., 11., 13.] if k < 1. else [0., 2., 4.5, 8., 11., 11., 13., 16.])
    state['v'] = max(state['v'] + float(np.clip(state['v_target'] - state['v'], -3. * DT_MDL, 2. * DT_MDL)), 0.)
    state['d'] -= state['v'] * DT_MDL
    if state['d'] < -r.uniform(5., 30.) or r.random() < 0.002:
      new_maneuver()
    if r.random() < 0.004 * k:
      state['blinkers'] = r.choice([(False, False)] * 6 + [(True, False), (False, True), (True, True)])
    if (state['blinkers'] != (False, False)) and r.random() < 0.02 * (1. - k):
      state['blinkers'] = (False, False)
    state['gas'] = state['gas'] - 1 if state['gas'] > 0 else (r.randint(1, 30) if r.random() < 0.002 * k else 0)
    if r.random() < (0.002 * k if state['lat'] else 0.01):
      state['lat'] = not state['lat']
    if r.random() < (0.002 * k if state['long'] else 0.01):
      state['long'] = not state['long']
    for i_side in range(2):
      state['bs'][i_side] = state['bs'][i_side] - 1 if state['bs'][i_side] > 0 else (r.randint(1, 60) if r.random() < 0.003 * k else 0)
    if r.random() < 0.005 * k:
      state['limit'] = r.choice([0., 35 * MPH, 45 * MPH, 50 * MPH])
    if r.random() < 0.005:
      state['dm'] = r.random() < 0.3 * k
    if r.random() < 0.01:
      state['edge'] = (r.random() < 0.1 * k, r.random() < 0.1 * k)
    if state['gap'] > 0:
      state['gap'] -= 1
      state['msg'] = None
    elif r.random() < 0.003 * k:
      state['gap'], state['msg'] = r.randint(5, 100), None
    elif i % 20 == 0 or state['msg'] is None:
      msg = messaging.new_message('navInstruction').navInstruction
      msg.maneuverType, msg.maneuverModifier = state['maneuver']
      msg.maneuverDistance = state['d'] + r.gauss(0., 2.)
      msg.distanceRemaining = msg.maneuverDistance + state['position']
      msg.lanes = list(state['lanes'])
      state['msg'] = msg
    road = state['road']
    model = road.message(state['d'], state['offset'])
    carstate = car_state(vEgo=state['v'], standstill=state['v'] < 0.05, leftBlinker=state['blinkers'][0], rightBlinker=state['blinkers'][1],
                         gasPressed=state['gas'] > 0, leftBlindspot=state['bs'][0] > 0, rightBlindspot=state['bs'][1] > 0,
                         steeringPressed=r.random() < 0.02, steeringTorque=r.choice([-1., 1.]))
    inputs = {'instruction': state['msg'], 'long_active': state['long'] and state['gas'] == 0, 'outer_lanes': nav_desire.outer_lanes(model),
              'far_lanes': nav_desire.far_lanes(model), 'driver_alert': state['dm'], 'speed_limit': state['limit'], 'accel': r.choice([0., 0., 0.5])}
    desire = yield carstate, state['lat'], 1. if state['moving'] is not None else r.choice([0., 0.01, 0.3]), state['edge'], inputs, road_view(model)
    # the driving model moves the car a lane over as a lane change desire asks
    if desire in LANE_CHANGE_DESIRES and state['moving'] is None and desire != state.get('prev'):
      toward = 1. if (desire == log.Desire.laneChangeLeft) == road.left else -1.
      state['moving'] = (i, state['offset'], state['offset'] + toward * (LANE / 2 + road.width / 2))
    state['prev'] = desire
    if state['moving'] is not None:
      start, origin, target = state['moving']
      f = min((i + 1 - start) * DT_MDL / LANE_CHANGE_TIME, 1.)
      state['offset'] = origin + (target - origin) * (1. - math.cos(math.pi * f)) / 2.
      state['moving'] = None if f >= 1. else state['moving']


def helper_outputs(dh) -> tuple:
  c = dh.nav_desire
  return (int(dh.desire), int(dh.lane_change_state), int(dh.lane_change_direction), int(dh.lane_turn_direction), round(c.speed_limit, 9),
          c.turn_across, c.signal_prompt)


def run_random(seed, seconds, helpers, nav_classes, turn_lanes=True):
  """Drive the helpers, in step, through one random drive, the model moving the car as the first one asks; without
  turn_lanes, the model reads no road edges or lane lines for them. Returns each frame's outputs, per helper."""
  drive = random_drive(seed, seconds)
  frame = next(drive)
  traces = [[] for _ in helpers]
  while True:
    carstate, lat, prob, edge, inputs, view = frame
    view = view if turn_lanes else RoadView()
    for dh, nav_class, trace in zip(helpers, nav_classes, traces, strict=True):
      nav = nav_class(**inputs, **({'road': view} if 'road' in nav_class.__dataclass_fields__ else {}))
      dh.update(carstate, lat, prob, edge[0], edge[1], nav)
      trace.append(helper_outputs(dh) + ((dh.turn_lane_change,) if hasattr(dh, 'turn_lane_change') else (None,)))
    try:
      frame = drive.send(helpers[0].desire)
    except StopIteration:
      return traces


def load_b1ea957():
  """b1ea957's nav_desire and desire_helper, what the owner drives today, from git under their own module names, or None
  where git or that commit is not there."""
  paths = ('openpilot/sunnypilot/selfdrive/controls/lib/nav_desire.py', 'openpilot/selfdrive/controls/lib/desire_helper.py')
  try:
    sources = [subprocess.run(['git', 'show', f'b1ea957:{path}'], cwd=BASEDIR, capture_output=True, text=True, check=True, timeout=30).stdout
               for path in paths]
  except (OSError, subprocess.SubprocessError):
    return None
  modules = []
  current = sys.modules[nav_desire.__name__]
  for name, path, text in zip(('nav_desire_b1ea957', 'desire_helper_b1ea957'), paths, sources, strict=True):
    module = types.ModuleType(name)
    module.__file__ = path
    sys.modules[name] = module
    # desire_helper imports nav_desire by its package name: b1ea957's, while it loads
    sys.modules[nav_desire.__name__] = modules[0] if modules else current
    try:
      exec(compile(text, f'b1ea957:{path}', 'exec'), module.__dict__)
    finally:
      sys.modules[nav_desire.__name__] = current
    modules.append(module)
  return modules


class TestTurnLaneUnchanged(OpenpilotTestCase):
  SETTINGS = [(True, True, True), (True, True, False), (False, True, False), (True, False, False)]  # turns, auto, far

  def helper(self, cls, turns, auto, far, lanes):
    params = Params()
    for key, value in (('NavTurnDesire', turns), ('NavAutoTurn', auto), ('NavAutoTurnFarSide', far), ('NavAutoTurnLane', lanes),
                       ('NavExitDesire', False), ('LaneTurnDesire', False)):
      params.put_bool(key, value)
    params.put('AutoLaneChangeTimer', int(AutoLaneChangeMode.NUDGELESS))
    return cls()

  def test_same_as_b1ea957_with_the_setting_off(self):
    # every output, every frame, of random drives past random turn lanes: with the setting off, the same as b1ea957's, and
    # never a lane change of the route's own
    old = load_b1ea957()
    if old is None:
      pytest.skip("b1ea957 is not in this checkout's git history")
    old_nav, old_helper = old
    moved = 0
    for seed in range(8):
      for turns, auto, far in self.SETTINGS:
        new = self.helper(DesireHelper, turns, auto, far, False)
        before = self.helper(old_helper.DesireHelper, turns, auto, far, False)
        traces = run_random(seed, 50., [new, before], [NavInputs, old_nav.NavInputs])
        assert [t[:-1] for t in traces[0]] == [t[:-1] for t in traces[1]], (seed, turns, auto, far)
        assert not any(t[-1] is not None for t in traces[0])
        # the same drives with the setting on do move into turn lanes: these drives reach it
        on = run_random(seed, 50., [self.helper(DesireHelper, turns, auto, far, True)], [NavInputs])[0]
        moved += sum(1 for t, p in zip(on, [(None,)] + on[:-1], strict=True) if t[-1] is not None and p[-1] is None)
    assert moved > 0

  def test_same_without_turn_lanes(self):
    # where no turn lane opens, the setting changes nothing, also when it is on
    for seed in range(6):
      for turns, auto, far in self.SETTINGS:
        helpers = [self.helper(DesireHelper, turns, auto, far, lanes) for lanes in (True, False)]
        traces = run_random(seed, 40., helpers, [NavInputs, NavInputs], turn_lanes=False)
        assert traces[0] == traces[1], (seed, turns, auto, far)

  def test_invariants_with_the_setting_on(self):
    # random drives past random turn lanes with the setting on: the route's lane change only toward the turn's side, and only
    # for a near side turn (right: these drives keep right), at most one per maneuver, never while a blinker is on or driver
    # monitoring warns, and never a second lane change command while one is under way
    for seed in range(10):
      dh = self.helper(DesireHelper, True, True, True, True)
      drive = random_drive(seed, 60.)
      frame, prev_desire, last_move = next(drive), log.Desire.none, None
      while True:
        carstate, lat, prob, edge, inputs, view = frame
        c = dh.nav_desire
        was_moving = dh.turn_lane_change is not None
        dh.update(carstate, lat, prob, edge[0], edge[1], NavInputs(**inputs, road=view))
        if dh.turn_lane_change is not None and not was_moving:
          turn = c.tracker.turn()
          assert turn is False and dh.turn_lane_change == turn and not (carstate.leftBlinker or carstate.rightBlinker) and not inputs['driver_alert']
          assert last_move != c.maneuver_id
          last_move = c.maneuver_id
        if was_moving and dh.desire in LANE_CHANGE_DESIRES and prev_desire not in LANE_CHANGE_DESIRES:
          raise AssertionError((seed, 'a second lane change command while the route moved over'))
        prev_desire = dh.desire
        try:
          frame = drive.send(dh.desire)
        except StopIteration:
          break


class TestTurnLaneWiring(OpenpilotTestCase):
  """From the setting to the alert: the param, both UIs, modeld's output, selfdrived's alert and its sound. Processes are
  read from their source: importing them needs compiled pieces."""

  def test_param(self):
    # off by default, after automatic far side turns, and never restored from a sunnylink backup
    header = (Path(BASEDIR) / 'openpilot/common/params_keys.h').read_text()
    names = re.findall(r'^\s*\{"(\w+)",', header, re.MULTILINE)
    assert names[names.index('NavAutoTurnFarSide') + 1] == 'NavAutoTurnLane'
    entries = re.findall(r'\{"NavAutoTurnLane",\s*\{([^{}]*)\}\}', header)
    assert len(entries) == 1
    flags, kind, default = (part.strip() for part in entries[0].split(','))
    assert flags == 'PERSISTENT' and kind == 'BOOL' and default == '"0"'

  def test_comma_4_toggle(self):
    tree = source('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models')
    toggle = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == 'self.nav_turn_lane_toggle')
    assert getattr(toggle.func, 'id', None) == 'BigParamControl'
    assert ast.unparse(toggle.args[0]) == "tr('auto turn lanes (alpha)')" and ast.literal_eval(toggle.args[1]) == 'NavAutoTurnLane'
    items = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == 'self.main_items')
    names = [ast.unparse(element) for element in items.elts]
    assert names[names.index('self.nav_far_turn_toggle') + 1] == 'self.nav_turn_lane_toggle'
    # shown as off and greyed without navigation and automatic route turns, with the reason; switched on only while parked;
    # the param written only by the driver's tap
    update = ast.unparse(method(tree, 'ModelsLayoutMici', '_update_state'))
    assert "lane_checked = navigation and auto_turns and ui_state.params.get_bool('NavAutoTurnLane')" in update
    assert 'self.nav_turn_lane_toggle.set_checked(lane_checked)' in update
    assert 'self.nav_turn_lane_toggle.set_enabled(navigation and auto_turns and (ui_state.is_offroad() or lane_checked))' in update
    assert "far_value = '' if navigation and auto_turns else tr('needs navigation') if not navigation else tr('needs auto route turns')" in update
    assert 'self.nav_turn_lane_toggle.set_value(far_value)' in update
    assert 'self.nav_turn_lane_toggle.refresh()' in ast.unparse(method(tree, 'ModelsLayoutMici', 'show_event'))
    writes = [call for name in ('put', 'put_bool', 'put_nonblocking', 'remove') for call in calls(tree, name)
              if any(isinstance(arg, ast.Constant) and arg.value == 'NavAutoTurnLane' for arg in call.args)]
    assert not writes

  def test_sunnylink_item(self):
    with open(DEFAULT_OUT) as f:
      schema = json.load(f)
    section = next(section for panel in schema['panels'] if panel['id'] == 'steering' for section in panel['sections'] if section['id'] == 'navigation')
    keys = [item['key'] for item in section['items']]
    assert keys[keys.index('NavAutoTurnFarSide') + 1] == 'NavAutoTurnLane'
    item = section['items'][keys.index('NavAutoTurnLane')]
    assert item['widget'] == 'toggle' and item['title'] == 'Automatic Turn Lanes (Alpha)'
    assert item['enablement'] == [{'type': 'offroad_only'}, {'type': 'param', 'key': 'NavigationEnabled', 'equals': True},
                                  {'type': 'param', 'key': 'NavAutoTurn', 'equals': True}]

  def test_modelds_publish_it(self):
    for modeld in MODELDS:
      tree = source(modeld)
      values = [node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
                for target in node.targets if isinstance(target, ast.Attribute) and target.attr == 'navTurnLane']
      assert len(values) == 1 and ast.unparse(values[0]) == 'DH.turn_lane_change is not None', modeld
    field = custom.ModelDataV2SP.schema.fields['navTurnLane']
    assert field.proto.ordinal.explicit == 12 and field.proto.slot.type.which() == 'bool'
    assert not messaging.new_message('modelDataV2SP').modelDataV2SP.navTurnLane

  def lane_change_events(self, state, nav_turn_lane, events_sp=()):
    """Runs selfdrived's own lane change and lane turn blocks on real modelV2 and modelDataV2SP messages, with selfdrived's
    own module level names and the events already raised, returning the events they leave."""
    text = (Path(BASEDIR) / 'openpilot/selfdrive/selfdrived/selfdrived.py').read_text()
    block = text[text.index('    # Handle lane change'):text.index("    for i, pandaState in enumerate(self.sm['pandaStates']):")]
    names = {'custom': custom, 'log': log}
    for node in ast.parse(text).body:
      if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) in ('LaneChangeState', 'LaneChangeDirection', 'EventName', 'TurnDirection',
                                                                          'NavTurnAcross'):
        exec(ast.unparse(node), names)
    model = messaging.new_message('modelV2').modelV2
    model.meta.laneChangeState, model.meta.laneChangeDirection = state, 'right'
    sp = messaging.new_message('modelDataV2SP').modelDataV2SP
    sp.navTurnLane = nav_turn_lane
    selfdrived = SimpleNamespace(sm={'modelV2': model, 'modelDataV2SP': sp}, events=Events(), events_sp=EventsSP())
    for event in events_sp:
      selfdrived.events_sp.add(event)
    exec(textwrap.dedent(block), names | {'self': selfdrived, 'CS': car_state()})
    return selfdrived.events.names, selfdrived.events_sp.names

  @parameterized.expand([
    ('laneChangeStarting', True, [], [EventNameSP.navTurnLane]),  # in place of Changing Lanes
    ('laneChangeFinishing', True, [], [EventNameSP.navTurnLane]),
    ('laneChangeStarting', False, [EventName.laneChange], []),  # the driver's own lane change, as before
    ('preLaneChange', False, [EventName.preLaneChangeRight], []),
    ('off', False, [], []),
  ])
  def test_raises_its_alert(self, state, nav_turn_lane, events, events_sp):
    assert self.lane_change_events(state, nav_turn_lane) == (events, events_sp)

  def test_slowing_never_shows_over_it(self):
    # Slowing for Route Turn, which plannerd may raise while the car moves over, would show over Moving to Turn Lane as the
    # newer alert of the same priority: it is dropped while the move lasts
    assert self.lane_change_events('laneChangeStarting', True, (EventNameSP.navTurnSlowing,)) == ([], [EventNameSP.navTurnLane])
    assert self.lane_change_events('off', False, (EventNameSP.navTurnSlowing,)) == ([], [EventNameSP.navTurnSlowing])

  def test_alert(self):
    event = EventNameSP.navTurnLane
    assert event == 29 and list(EVENTS_SP[event]) == [ET.WARNING]
    alert = EVENTS_SP[event][ET.WARNING]
    assert (alert.alert_text_1, alert.alert_text_2, alert.alert_status, alert.alert_size, alert.priority, alert.visual_alert, alert.duration) == \
           ("Moving to Turn Lane", "Steer to stay in your lane", AlertStatus.userPrompt, AlertSize.mid, Priority.LOW, VisualAlert.none, int(0.1 / DT_CTRL))
    # a chime of its own: none of the other route alerts' sounds (soundd plays a sound again only when it changes), and one
    # soundd has
    others = {EVENTS_SP[getattr(EventNameSP, name)][ET.WARNING].audible_alert for name in ('navTurnSlowing', 'navTurnAcross', 'navTurnSignal')}
    assert alert.audible_alert != AudibleAlert.none and alert.audible_alert not in others and alert.audible_alert in soundd.sound_list
    events = EventsSP()
    events.add(event)
    assert [a.alert_type for a in events.create_alerts([ET.WARNING])] == [f'navTurnLane/{ET.WARNING}']

  @staticmethod
  def alerts_and_sounds(frames, pay_attention=0.1):
    """Closed loop over a drive's frames: the route's lane change and turn speed (modeld, modelDataV2SP as it publishes
    them), the slowdown and its Slowing for Route Turn (plannerd), selfdrived's alerts at 100 Hz, with driver monitoring's
    first warning, Pay Attention, while the drive's driver monitoring warns (shown for at least `pay_attention` s: 0.1 s,
    and 2 s on the comma 4), and soundd's own rule for when a sound plays. Per frame: the alert shown, the sound started
    (None for none), and whether plannerd raised Slowing for Route Turn."""
    turn_speed = NavTurnSpeedController(SimpleNamespace(openpilotLongitudinalControl=True))
    manager, sound = AlertManager(), object.__new__(soundd.Soundd)
    sound.current_alert, sound.current_sound_frame, sound.pending_stop = AudibleAlert.none, 0, False
    sound.loaded_sounds = {k: np.zeros(int(0.4 * soundd.SAMPLE_RATE), dtype=np.float32) for k in soundd.sound_list}  # only their lengths matter
    warning = copy.copy(EVENTS[EventName.driverDistracted1][ET.PERMANENT])
    warning.duration, warning.alert_type, warning.event_type = int(pay_attention / DT_CTRL), f'driverDistracted1/{ET.PERMANENT}', ET.PERMANENT
    a_prev, out = 0., []
    for i, f in enumerate(frames):
      plan = EventsSP()
      msg = messaging.new_message('modelDataV2SP').modelDataV2SP
      msg.navTurnSpeed, msg.navTurnLane = f['limit'], f['turn_lane']
      model = FakeSubMaster(seen={'modelDataV2SP': True}, recv_time={'modelDataV2SP': time.monotonic()}, data={'modelDataV2SP': msg})
      turn_speed.update(model, True, False, f['v'], a_prev, plan)
      a_prev = turn_speed.a_target if turn_speed.active else 0.
      for k in range(i * 5, i * 5 + 5):  # selfdrived at 100 Hz
        events = EventsSP()
        for name in plan.names:
          events.add(name)
        if f['turn_lane']:
          events.add(EventNameSP.navTurnLane)
          events.remove(EventNameSP.navTurnSlowing)
        manager.add_many(k, events.create_alerts([ET.WARNING], [None, None, None, False, 0, None]) + ([warning] if f['dm'] else []))
        manager.process_alerts(k, set())
      before = (sound.current_alert, sound.current_sound_frame)
      sound.update_alert(int(manager.current_alert.audible_alert))
      started = sound.current_alert if sound.current_sound_frame == 0 and before != (sound.current_alert, 0) else None
      out.append((manager.current_alert.alert_type, started, EventNameSP.navTurnSlowing in plan.names))
      sound.current_sound_frame += int(0.05 * soundd.SAMPLE_RATE)
    return out

  @parameterized.expand([(20,), (25,), (30,)])
  def test_shows_and_chimes_while_it_moves(self, mph):
    # Moving to Turn Lane shows for the whole move, over Slowing for Route Turn where that shows too, and its sound plays as
    # it starts; Slowing for Route Turn shows on the approach, with its sound
    frames = Approach(Road(), mph=mph).run()
    out = self.alerts_and_sounds(frames)
    first = next(i for i, f in enumerate(frames) if f['turn_lane'])
    assert out[first][1] == AudibleAlert.preAlert
    assert all(s == f'navTurnLane/{ET.WARNING}' for (s, _, _), f in zip(out, frames, strict=True) if f['turn_lane'])
    slowing = [i for i, (s, _, _) in enumerate(out) if s == f'navTurnSlowing/{ET.WARNING}']
    assert slowing and out[slowing[0]][1] == AudibleAlertSP.promptSingleHigh

  # where the slowdown starts while the car moves over (these turn lanes and speeds), plannerd raised its one Slowing for
  # Route Turn then, and selfdrived dropped it: it now waits for the move to end, and shows
  @parameterized.expand([(20, 85.), (25, 95.), (25, 105.), (25, 115.), (30, 135.), (30, 145.)])
  def test_slowing_shows_after_the_move(self, mph, opens):
    frames = Approach(Road(opens=opens), mph=mph, start=260.).run()
    out = self.alerts_and_sounds(frames)
    moving = [i for i, f in enumerate(frames) if f['turn_lane']]
    raised = [i for i, (_, _, slowing) in enumerate(out) if slowing]
    assert moving and len(raised) == 1 and raised[0] > moving[-1]
    assert out[raised[0]][0] == f'navTurnSlowing/{ET.WARNING}' and out[raised[0]][1] == AudibleAlertSP.promptSingleHigh
    assert frames[raised[0]]['v'] > NAV_TURN_SPEED + 0.5

  def test_slowing_waits_for_the_move(self):
    # plannerd, on real modelDataV2SP messages: no Slowing for Route Turn while the route's lane change is under way, then
    # its one alert
    c = NavTurnSpeedController(SimpleNamespace(openpilotLongitudinalControl=True))
    msg = messaging.new_message('modelDataV2SP').modelDataV2SP
    msg.navTurnSpeed, msg.navTurnLane = 6., True
    sm = FakeSubMaster(seen={'modelDataV2SP': True}, recv_time={'modelDataV2SP': time.monotonic()}, data={'modelDataV2SP': msg})
    names = []
    for _ in range(20):
      events = EventsSP()
      c.update(sm, True, False, 10., 0., events)
      names += events.names
    assert c.active and not names and not c.alerted
    msg.navTurnLane = False
    for _ in range(20):
      events = EventsSP()
      c.update(sm, True, False, 10., 0., events)
      names += events.names
    assert names == [EventNameSP.navTurnSlowing]

  @parameterized.expand([(0.1,), (2.,)])
  def test_waits_for_driver_monitoring_and_chimes(self, pay_attention):
    # driver monitoring's first warning, whose sound the move's alert shares, until the turn lane has opened: the move waits
    # until it has been quiet for 3 s, so Pay Attention is gone, also where it shows for 2 s, and the move's chime plays
    frames = Approach(Road(opens=125.), mph=25., start=280., driver_alert=lambda s: 138. < s.d <= 150.).run()
    out = self.alerts_and_sounds(frames, pay_attention)
    quiet = next(f['t'] for f in frames if f['d'] <= 138.)
    first = next(i for i, f in enumerate(frames) if f['turn_lane'])
    assert len(moves(frames)) == 1 and frames[first]['t'] >= quiet + 3. - DT_MDL
    assert out[first][1] == AudibleAlert.preAlert and out[first][0] == f'navTurnLane/{ET.WARNING}'
    # and the same drive with the warning a moment later: no move while it warns, nor for 3 s after
    frames = Approach(Road(opens=125.), mph=25., start=280., driver_alert=lambda s: 130. < s.d <= 132.).run()
    assert len(moves(frames)) == 1 and moves(frames)[0]['t'] >= next(f['t'] for f in frames if f['d'] <= 130.) + 3. - DT_MDL

  # driver monitoring's warnings show over it: its later ones whenever they come, and its first one, of the same priority,
  # when it comes during the move (the move never starts while it warns)
  @parameterized.expand([(EventName.driverDistracted2, 2.), (EventName.driverDistracted2, 0.), (EventName.driverDistracted1, 2.)])
  def test_driver_monitoring_shows_over_it(self, warning, dm_from):
    manager, events, events_sp = AlertManager(), Events(), EventsSP()
    for k in range(round(6. / DT_CTRL)):
      t = k * DT_CTRL
      events.clear()
      events_sp.clear()
      if t >= dm_from:
        events.add(warning)
      if t >= 2. - dm_from:
        events_sp.add(EventNameSP.navTurnLane)
      args = [None, None, None, False, 0, None]
      manager.add_many(k, events.create_alerts([ET.PERMANENT, ET.WARNING], args) + events_sp.create_alerts([ET.PERMANENT, ET.WARNING], args))
      manager.process_alerts(k, set())
      if t >= 2.:
        assert manager.current_alert.alert_type == f'{events.get_event_name(warning)}/{ET.PERMANENT}', (t, manager.current_alert.alert_type)
