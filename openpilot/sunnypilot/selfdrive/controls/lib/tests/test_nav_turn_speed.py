"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import time
from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_TURN_MARGIN, NAV_TURN_SPEED, NavDesireController, NavInputs
from openpilot.sunnypilot.selfdrive.controls.lib.nav_turn_speed import NAV_TURN_ALERT_RESET, NAV_TURN_JERK, NAV_TURN_MAX_DECEL, \
  NavTurnSpeedController

EventNameSP = custom.OnroadEventSP.EventName


class FakeSubMaster(SimpleNamespace):
  def __getitem__(self, service):
    return self.data[service]


class FakeEvents(list):
  def add(self, event):
    self.append(event)


def submaster(limit=10., age=0., seen=True):
  # a real message, so a renamed or retired field fails here rather than in plannerd
  msg = messaging.new_message('modelDataV2SP').modelDataV2SP
  msg.navTurnSpeed = limit
  return FakeSubMaster(seen={'modelDataV2SP': seen}, recv_time={'modelDataV2SP': time.monotonic() - age}, data={'modelDataV2SP': msg})


def controller(openpilot_long=True):
  return NavTurnSpeedController(SimpleNamespace(openpilotLongitudinalControl=openpilot_long))


def step(c, sm, v_ego, long_enabled=True, long_override=False, a_prev=0., frames=1):
  events = FakeEvents()
  for _ in range(frames):
    c.update(sm, long_enabled, long_override, v_ego, a_prev, events)
  return events


class TestNavTurnSpeed(OpenpilotTestCase):
  def test_follows_the_published_limit(self):
    c = controller()
    step(c, submaster(10.), 12.)
    assert c.active and c.v_target == 10.
    assert LongitudinalPlannerSP.nav_turn_accel(SimpleNamespace(nav_turn=c)) == c.a_target

  def test_inactive(self):
    for c, sm, long_enabled, long_override in [
      (controller(openpilot_long=False), submaster(), True, False),
      (controller(), submaster(), False, False),
      (controller(), submaster(), True, True),  # the gas
      (controller(), submaster(0.), True, False),  # no route turn
      (controller(), submaster(math.nan), True, False),
      (controller(), submaster(age=1.), True, False),  # modeld stopped publishing
      (controller(), submaster(seen=False), True, False),
    ]:
      step(c, sm, 12., long_enabled=long_enabled, long_override=long_override)
      assert not c.active and c.v_target == V_CRUISE_UNSET
      assert LongitudinalPlannerSP.nav_turn_accel(SimpleNamespace(nav_turn=c)) is None

  def test_more_braking_is_jerk_limited_and_capped(self):
    c = controller()
    step(c, submaster(10.), 20.)
    assert math.isclose(c.a_target, -NAV_TURN_JERK * DT_MDL)  # eases in from the planner's last output
    a = 0.
    for _ in range(100):
      step(c, submaster(10.), 20., a_prev=a)
      a = c.a_target
    assert math.isclose(a, -NAV_TURN_MAX_DECEL)  # far behind the profile: never harder than its cap

  def test_never_holds_on_to_other_braking(self):
    c = controller()
    step(c, submaster(11.), 12., a_prev=-3.)  # the planner brakes hard for a lead car as the slowdown starts
    assert c.a_target == -NAV_TURN_MAX_DECEL
    step(c, submaster(15.), 12., a_prev=-3.)  # and the turn needs none
    assert c.a_target == ACCEL_MAX

  def test_alerts_once_as_it_starts_slowing(self):
    # modeld's controller and this one together, on a steady approach from cruising speed with no other limit
    nav = NavDesireController()
    nav.turn_enabled, nav.auto_enabled, nav.param_read_counter = False, True, 1
    c = controller()
    v, a, distance, events, v_at_margin = 20., 0., 400., [], None
    while distance > -30.:
      instruction = SimpleNamespace(maneuverType='turn', maneuverModifier='left', maneuverDistance=distance,
                                    distanceRemaining=distance + 1000., lanes=[])
      carstate = SimpleNamespace(vEgo=v, leftBlinker=False, rightBlinker=False, leftBlindspot=False, rightBlindspot=False,
                                 gasPressed=False, standstill=False, steeringPressed=False, steeringTorque=0.)
      nav.update(carstate, True, NavInputs(instruction, 0., True))
      events += step(c, submaster(nav.speed_limit), v, a_prev=a)
      a = min(c.a_target, 0.) if c.active else 0.  # the cruise speed is 20 m/s
      v = max(v + a * DT_MDL, 0.)
      distance -= v * DT_MDL
      if v_at_margin is None and distance <= NAV_TURN_MARGIN:
        v_at_margin = v
    assert events == [EventNameSP.navTurnSlowing]
    assert v_at_margin <= NAV_TURN_SPEED + 0.5

  def test_alerts_again_only_after_a_pause(self):
    c = controller()
    assert step(c, submaster(10.), 15.) == [EventNameSP.navTurnSlowing]
    assert not step(c, submaster(0.), 15., frames=round(3. / DT_MDL))  # a short break, like navd moving on
    assert not step(c, submaster(10.), 15.)
    step(c, submaster(0.), 15., frames=round((NAV_TURN_ALERT_RESET + 1.) / DT_MDL))
    assert step(c, submaster(10.), 15.) == [EventNameSP.navTurnSlowing]
