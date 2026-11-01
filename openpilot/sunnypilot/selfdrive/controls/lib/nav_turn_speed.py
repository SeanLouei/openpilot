"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_NEW_MANEUVER_JUMP, fresh_nav_instruction, normalize_direction, \
  route_turn, signals_route_turn

EventNameSP = custom.OnroadEventSP.EventName

# 10 mph keeps a tight right turn within the lateral acceleration limits of angle-steered cars like Tesla
NAV_TURN_SPEED = 4.5  # m/s
NAV_TURN_MARGIN = 15.  # m before the maneuver point the car is down to turn speed
NAV_TURN_DECEL = 0.8  # m/s^2 the speed profile to the turn slows at
NAV_TURN_MAX_DECEL = 1.5  # m/s^2 at most, when the turn comes up late
NAV_TURN_JERK = 1.5  # m/s^3
NAV_TURN_HOLD = 20.  # m past the maneuver point the turn speed holds, through the turn
NAV_TURN_ALERT_MARGIN = 0.5  # m/s above the profile that counts as slowing for it


def turn_speed_limit(distance: float) -> float:
  """The highest speed this far before a turn from which the car slows to turn speed at the profile's deceleration."""
  return math.sqrt(NAV_TURN_SPEED ** 2 + 2 * NAV_TURN_DECEL * max(distance - NAV_TURN_MARGIN, 0.))


class NavTurnSpeedController:
  """Slows for the route's next turn while openpilot controls the speed: for every turn with automatic turns on, else
  once the driver signals for it. Holds turn speed through the turn. The gas calls the slowdown off for that turn, until
  the driver signals for it again."""

  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.frame = -1
    self.turn_enabled = False
    self.auto_enabled = False
    self.read_params()

    self.v_target = V_CRUISE_UNSET
    self.a_target = 0.
    self.active = False  # the turn limits the speed below the cruise speed
    self.alerted = False
    self.hold = 0.  # m left to drive at turn speed past the maneuver point

    self.maneuver: tuple[str, str] | None = None
    self.maneuver_distance = math.inf
    self.called_off = False
    self.prev_blinkers = (False, False)

  def read_params(self) -> None:
    self.turn_enabled = self.params.get_bool("NavTurnDesire")
    self.auto_enabled = self.params.get_bool("NavAutoTurn")

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_prev: float, v_cruise: float, events_sp) -> None:
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()

    CS = sm['carState']
    blinkers = (bool(CS.leftBlinker), bool(CS.rightBlinker))
    prev_blinkers, self.prev_blinkers = self.prev_blinkers, blinkers
    turn = self._route_turn(sm, v_ego)

    distance = None
    if turn is not None:
      left, turn_distance = turn
      if blinkers == ((True, False) if left else (False, True)) and not prev_blinkers[0 if left else 1]:
        self.called_off = False  # signaling for the turn again brings its slowdown back
      auto = self.auto_enabled and not any(blinkers)
      signaled = signals_route_turn(CS, left, turn_distance, v_ego)
      if not self.called_off and (auto or signaled):
        distance = turn_distance

    v_target = V_CRUISE_UNSET
    if not (self.CP.openpilotLongitudinalControl and long_enabled) or long_override:
      self.called_off |= long_override and (self.active or self.hold > 0.)
      self.hold = 0.
    else:
      if distance is not None and 0. <= distance <= NAV_TURN_MARGIN:
        self.hold = NAV_TURN_HOLD  # at the turn: keep turn speed until through it, even once navd moves on
      elif self.hold > 0.:
        self.hold = max(self.hold - v_ego * DT_MDL, 0.)
      if distance is not None and distance >= 0.:
        v_target = turn_speed_limit(distance)
      if self.hold > 0.:
        v_target = min(v_target, NAV_TURN_SPEED)

    active = v_target < min(v_cruise, V_CRUISE_UNSET)
    if active:
      # follow the profile: its own deceleration plus a correction toward it, within the jerk limit
      profile_decel = NAV_TURN_DECEL if distance is not None and distance > NAV_TURN_MARGIN and v_target > NAV_TURN_SPEED else 0.
      a_target = float(np.clip(v_target - v_ego - profile_decel, -NAV_TURN_MAX_DECEL, ACCEL_MAX))
      a_start = self.a_target if self.active else a_prev
      self.a_target = float(np.clip(a_target, a_start - NAV_TURN_JERK * DT_MDL, a_start + NAV_TURN_JERK * DT_MDL))
      if not self.alerted and v_ego > v_target + NAV_TURN_ALERT_MARGIN:
        events_sp.add(EventNameSP.navTurnSlowing)
        self.alerted = True
    else:
      self.alerted = False
    self.active = active
    self.v_target = v_target

  def _route_turn(self, sm, v_ego: float) -> tuple[bool, float] | None:
    """The route's next turn, as (left, distance), when route turns are on; the slowdown is called off per maneuver."""
    if not (self.turn_enabled or self.auto_enabled):
      return None
    instruction, age = fresh_nav_instruction(sm)
    if instruction is None:
      return None
    turn = route_turn(instruction, age, v_ego)
    maneuver = (str(instruction.maneuverType).strip().lower(), normalize_direction(instruction.maneuverModifier))
    distance = float(instruction.maneuverDistance)
    if maneuver != self.maneuver or not distance <= self.maneuver_distance + NAV_NEW_MANEUVER_JUMP:
      self.called_off = False
    self.maneuver, self.maneuver_distance = maneuver, distance
    return turn
