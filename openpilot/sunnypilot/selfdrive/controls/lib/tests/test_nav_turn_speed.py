"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import time
from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.nav_turn_speed import NAV_TURN_HOLD, NAV_TURN_MAX_DECEL, NAV_TURN_SPEED, \
  NavTurnSpeedController, turn_speed_limit

EventNameSP = custom.OnroadEventSP.EventName
V_CRUISE = 30.  # m/s, the driver's set speed


class FakeSubMaster(SimpleNamespace):
  def __getitem__(self, service):
    return self.data[service]


class FakeEvents(list):
  def add(self, event):
    self.append(event)


def submaster(distance=200., modifier='right', blinkers=(False, False), route=True):
  # real messages, so a renamed or retired field fails here rather than in plannerd
  car_state = messaging.new_message('carState').carState
  car_state.leftBlinker, car_state.rightBlinker = blinkers
  instruction = messaging.new_message('navInstruction').navInstruction
  instruction.maneuverType, instruction.maneuverModifier, instruction.maneuverDistance = 'turn', modifier, distance
  return FakeSubMaster(seen={'navInstruction': route}, valid={'navInstruction': route}, recv_time={'navInstruction': time.monotonic()},
                       data={'carState': car_state, 'navInstruction': instruction})


def controller(auto=True, turns=False, openpilot_long=True):
  params = Params()
  params.put_bool("NavAutoTurn", auto)
  params.put_bool("NavTurnDesire", turns)
  return NavTurnSpeedController(SimpleNamespace(openpilotLongitudinalControl=openpilot_long))


def step(c, sm, v_ego, long_enabled=True, long_override=False, a_prev=0., v_cruise=V_CRUISE, frames=1):
  events = FakeEvents()
  for _ in range(frames):
    c.update(sm, long_enabled, long_override, v_ego, a_prev, v_cruise, events)
  return events


class TestNavTurnSpeed(OpenpilotTestCase):
  def test_speed_profile(self):
    assert turn_speed_limit(0.) == turn_speed_limit(15.) == NAV_TURN_SPEED  # down to turn speed 15 m before the turn
    assert math.isclose(turn_speed_limit(115.), math.sqrt(NAV_TURN_SPEED ** 2 + 2 * 0.8 * 100.))
    assert turn_speed_limit(50.) < turn_speed_limit(100.) < turn_speed_limit(400.)

  def test_auto_slows_for_every_turn(self):
    c = controller()
    events = step(c, submaster(distance=400.), 20.)
    assert c.active and math.isclose(c.v_target, turn_speed_limit(400.), rel_tol=1e-4) and not events  # limiting, not slowing yet
    events = step(c, submaster(distance=200.), 20., frames=40)
    assert c.active and c.v_target < 20. and events == [EventNameSP.navTurnSlowing]  # one chime per approach
    assert math.isclose(c.a_target, -NAV_TURN_MAX_DECEL)  # behind the profile: slows harder than the cruise limit, never more
    assert LongitudinalPlannerSP.nav_turn_accel(SimpleNamespace(nav_turn=c)) == c.a_target

  def test_jerk_limited(self):
    c = controller()
    step(c, submaster(distance=30.), 20.)  # a late turn
    assert math.isclose(c.a_target, -1.5 * 0.05)  # eases in from the planner's acceleration

  def test_auto_waits_while_signaling_the_other_way(self):
    c = controller()
    step(c, submaster(blinkers=(True, False)), 20.)
    assert not c.active and c.v_target == V_CRUISE_UNSET  # a blinker the other way is not for this turn
    step(c, submaster(blinkers=(False, True)), 20.)
    assert c.active

  def test_blinker_turns(self):
    for blinkers, active in [((False, False), False), ((False, True), True), ((True, False), False), ((True, True), False)]:
      c = controller(auto=False, turns=True)
      step(c, submaster(blinkers=blinkers), 20.)
      assert c.active == active, blinkers
    c = controller(auto=False, turns=True)
    step(c, submaster(distance=300., blinkers=(False, True)), 20.)  # beyond 15 m + 12 s: the blinker is for a lane change
    assert not c.active

  def test_only_with_openpilot_longitudinal(self):
    for c, long_enabled in [(controller(openpilot_long=False), True), (controller(), False)]:
      step(c, submaster(), 20., long_enabled=long_enabled)
      assert not c.active and c.v_target == V_CRUISE_UNSET

  def test_inactive_without_route(self):
    for sm in [submaster(route=False), submaster(modifier='slight right')]:
      c = controller()
      step(c, sm, 20.)
      assert not c.active

  def test_gas_calls_turn_off(self):
    c = controller()
    step(c, submaster(distance=200.), 20.)
    step(c, submaster(distance=190.), 20., long_override=True)
    assert not c.active
    step(c, submaster(distance=180.), 20.)
    assert not c.active  # stays off for this turn
    step(c, submaster(distance=170., blinkers=(False, True)), 20.)
    assert c.active  # until the driver signals for it
    c = controller()
    step(c, submaster(distance=200.), 20.)
    step(c, submaster(distance=190.), 20., long_override=True)
    step(c, submaster(distance=500., modifier='left'), 20.)
    step(c, submaster(distance=200., modifier='left'), 20.)
    assert c.active  # and the next turn slows again

  def test_holds_through_the_turn(self):
    c = controller()
    step(c, submaster(distance=10.), NAV_TURN_SPEED)
    assert c.v_target == NAV_TURN_SPEED and c.hold == NAV_TURN_HOLD
    step(c, submaster(distance=-3.), NAV_TURN_SPEED)
    assert c.v_target == NAV_TURN_SPEED
    step(c, submaster(distance=500., modifier='left'), NAV_TURN_SPEED, frames=10)  # navd moved on: still through the turn
    assert c.v_target == NAV_TURN_SPEED
    step(c, submaster(distance=500., modifier='left'), NAV_TURN_SPEED, frames=100)  # 20 m later
    assert math.isclose(c.v_target, turn_speed_limit(c.tracker.distance), rel_tol=1e-4) and c.hold == 0.

  def test_signal_latches_while_slowing(self):
    c = controller(auto=False, turns=True)
    step(c, submaster(distance=200., blinkers=(False, True)), 20.)  # within 15 m + 12 s at 20 m/s
    assert c.active
    step(c, submaster(distance=160., blinkers=(False, True)), 10., frames=20)  # slowing shrinks that distance to 135 m
    assert c.active
    step(c, submaster(distance=150.), 10.)  # until the blinker goes off
    assert not c.active

  def test_completed_turn_is_not_slowed_for_again(self):
    c = controller()
    step(c, submaster(distance=5.), NAV_TURN_SPEED, frames=40)  # through the maneuver point
    step(c, submaster(distance=1.), NAV_TURN_SPEED, frames=100)  # navd still reports it just ahead, for 22 m
    assert c.tracker.passed and c.hold == 0. and not c.active

  def test_below_cruise_only(self):
    c = controller()
    step(c, submaster(distance=200.), 10., v_cruise=10.)  # the set speed is already lower than the profile
    assert not c.active
