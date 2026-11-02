"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import time

import numpy as np

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_TURN_DECEL, NAV_TURN_SPEED

EventNameSP = custom.OnroadEventSP.EventName

NAV_TURN_LIMIT_MAX_AGE = 0.5  # s; modeld publishes the route turn's speed limit with every model frame
NAV_TURN_MAX_DECEL = 1.5  # m/s^2 at most, when the turn comes up late
NAV_TURN_JERK = 1.5  # m/s^3
NAV_TURN_ALERT_DECEL = 0.3  # m/s^2 of slowing that counts as the slowdown taking over
NAV_TURN_ALERT_MARGIN = 0.5  # m/s above turn speed
NAV_TURN_ALERT_RESET = 5.  # s without a route turn limit before the next one alerts again


class NavTurnSpeedController:
  """Slows for the route's turn while openpilot controls the speed, following the speed limit modeld's NavDesireController
  publishes, so the slowdown and the turn desire share one state: which turns it slows for, and what calls them off.
  An acceleration candidate that may slow harder than the gentle cruise limit, within a jerk limit, whatever the set
  speed, and an alert when it starts slowing the car, once per approach."""

  def __init__(self, CP):
    self.CP = CP
    self.v_target = V_CRUISE_UNSET
    self.a_target = 0.
    self.active = False  # a route turn limits the speed
    self.alerted = False
    self.inactive_time = math.inf

  @staticmethod
  def published_limit(sm) -> float:
    if not sm.seen['modelDataV2SP'] or not 0. <= time.monotonic() - sm.recv_time['modelDataV2SP'] <= NAV_TURN_LIMIT_MAX_AGE:
      return 0.
    limit = float(sm['modelDataV2SP'].navTurnSpeed)
    return limit if math.isfinite(limit) and limit > 0. else 0.

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_prev: float, events_sp) -> None:
    limit = self.published_limit(sm)
    engaged = self.CP.openpilotLongitudinalControl and long_enabled and not long_override
    self.v_target = limit if engaged and limit > 0. else V_CRUISE_UNSET
    self.active = self.v_target < V_CRUISE_UNSET
    if not self.active:
      self.inactive_time += DT_MDL
      self.alerted &= self.inactive_time < NAV_TURN_ALERT_RESET
      return
    self.inactive_time = 0.

    # follow the profile: its own deceleration plus a correction toward it. Only more braking is jerk limited, from the
    # planner's last output, so harder braking the planner had for something else never lingers here.
    profile_decel = NAV_TURN_DECEL if self.v_target > NAV_TURN_SPEED else 0.
    a_desired = float(np.clip(self.v_target - v_ego - profile_decel, -NAV_TURN_MAX_DECEL, ACCEL_MAX))
    self.a_target = max(a_desired, a_prev - NAV_TURN_JERK * DT_MDL)
    if not self.alerted and a_desired < -NAV_TURN_ALERT_DECEL and v_ego > NAV_TURN_SPEED + NAV_TURN_ALERT_MARGIN:
      events_sp.add(EventNameSP.navTurnSlowing)
      self.alerted = True
