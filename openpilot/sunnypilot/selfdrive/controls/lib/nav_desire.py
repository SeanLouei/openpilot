"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The route-confirmed turn and exit logic is adapted from StarPilot's navigation desires
(https://github.com/firestar5683/StarPilot), Copyright (c) 2026, firestar5683 and StarPilot contributors,
used under the MIT License:

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
  documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
  persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
  Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
  OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
import time

import numpy as np

from openpilot.cereal import log, custom
from openpilot.common.params import Params

TurnDirection = custom.ModelDataV2SP.TurnDirection

NAV_INSTRUCTION_MAX_AGE = 2.5  # s, navd publishes once a second

# The model gets a single desire pulse and decides itself when the turn is done, so a route turn only starts
# its desire once the intersection is close: the pulse then lands on the route's intersection, not an earlier one.
NAV_TURN_MAX_SPEED = 14.0  # m/s
NAV_TURN_PREVIEW_TIME = 6.0  # s
NAV_TURN_PREVIEW_DISTANCE = (35.0, 90.0)  # m

# Drivers signal well before slowing for a turn: a matching blinker this far out is for the route turn, not a lane change
NAV_TURN_SIGNAL_TIME = 12.0  # s
NAV_TURN_SIGNAL_BASE_DISTANCE = 15.0  # m
NAV_TURN_SIGNAL_DISTANCE = (30.0, 250.0)  # m

# Exits and forks get their keep desire close to the split, closer still when the exit shares lanes with the road
NAV_KEEP_SPEED_BP = [0.0, 15.0, 30.0]  # m/s
NAV_KEEP_DISTANCE_V = [25.0, 90.0, 160.0]  # m
NAV_KEEP_AMBIGUOUS_SPLIT_SCALE = 0.6
NAV_KEEP_SMALL_SPLIT_MAX_OTHER_LANES = 2

TURN_TYPES = ('turn', 'end of road', 'on ramp')  # Mapbox maneuver types that are turns at an intersection when sided
SPLIT_TYPES = ('off ramp', 'fork')
TURN_LEFT, TURN_RIGHT = ('left', 'sharpLeft'), ('right', 'sharpRight')
LEFT_DIRECTIONS, RIGHT_DIRECTIONS = ('slightLeft', 'left', 'sharpLeft'), ('slightRight', 'right', 'sharpRight')


def normalize_direction(modifier) -> str:
  """Mapbox modifiers ('slight left', 'sharp right', ...) as navInstruction directions ('slightLeft', 'sharpRight', ...)."""
  words = str(modifier).replace('-', ' ').split()
  return words[0] + ''.join(word.capitalize() for word in words[1:]) if words else 'none'


def active_lane(lanes) -> tuple[str, int]:
  """The direction and index of the first lane the route uses, from the maneuver's lane guidance."""
  for i, lane in enumerate(lanes):
    direction = str(lane.activeDirection)
    if direction == 'none' and len(lane.directions) == 1:
      direction = str(lane.directions[0])
    if lane.active and direction != 'none':
      return direction, i
  return '', -1


def fresh_nav_instruction(sm):
  """The latest route instruction and its age in seconds, or (None, 0.) without a valid, recent one."""
  age = time.monotonic() - sm.recv_time['navInstruction']
  if sm.seen['navInstruction'] and sm.valid['navInstruction'] and 0. <= age <= NAV_INSTRUCTION_MAX_AGE:
    return sm['navInstruction'], age
  return None, 0.


class NavDesireController:
  """Desires the driver confirms against the navigation route.

  Turns: a blinker matching the route's next turn starts a turn desire once that intersection is close.
  Until then, that blinker is held back from starting a nudgeless lane change or an early lane turn desire.
  Exits and forks: a nudge toward the side the route's lane guidance takes gives a keep desire near the split.
  """

  def __init__(self):
    self.params = Params()
    self.param_read_counter = 0
    self.turn_enabled = False
    self.exit_enabled = False
    self.read_params()

    self.turn_direction = TurnDirection.none
    self.route_turn_signal = False
    self.keep_desire = log.Desire.none

  def read_params(self) -> None:
    self.turn_enabled = self.params.get_bool("NavTurnDesire")
    self.exit_enabled = self.params.get_bool("NavExitDesire")

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update(self, carstate, lateral_active: bool, nav_instruction, nav_age: float = 0.) -> None:
    self.turn_direction = TurnDirection.none
    self.route_turn_signal = False
    self.keep_desire = log.Desire.none
    if nav_instruction is None or not lateral_active:
      return

    v_ego = max(float(carstate.vEgo), 0.)
    # navd only updates once a second, so dead reckon the distance since then
    distance = float(nav_instruction.maneuverDistance) - v_ego * max(nav_age, 0.)
    maneuver_type = str(nav_instruction.maneuverType).strip().lower()
    direction = normalize_direction(nav_instruction.maneuverModifier)

    if self.turn_enabled and maneuver_type in TURN_TYPES and direction in TURN_LEFT + TURN_RIGHT:
      self._update_turn(carstate, v_ego, distance, direction in TURN_LEFT)
    elif self.exit_enabled and maneuver_type in SPLIT_TYPES and direction in LEFT_DIRECTIONS + RIGHT_DIRECTIONS:
      self._update_keep(carstate, v_ego, distance, nav_instruction.lanes)

  def _update_turn(self, carstate, v_ego: float, distance: float, left: bool) -> None:
    blinker = (carstate.leftBlinker and not carstate.rightBlinker) if left else (carstate.rightBlinker and not carstate.leftBlinker)
    signal_distance = np.clip(NAV_TURN_SIGNAL_BASE_DISTANCE + v_ego * NAV_TURN_SIGNAL_TIME, *NAV_TURN_SIGNAL_DISTANCE)
    self.route_turn_signal = bool(blinker and 0. <= distance <= signal_distance)

    preview_distance = np.clip(v_ego * NAV_TURN_PREVIEW_TIME, *NAV_TURN_PREVIEW_DISTANCE)
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    if blinker and 0. <= distance <= preview_distance and v_ego < NAV_TURN_MAX_SPEED and not carstate.standstill and not blindspot:
      self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  def _update_keep(self, carstate, v_ego: float, distance: float, lanes) -> None:
    lane_direction, lane_index = active_lane(lanes)
    if lane_direction not in LEFT_DIRECTIONS + RIGHT_DIRECTIONS:
      return  # without lane guidance toward a side, the active lane may just continue straight past the split
    left = lane_direction in LEFT_DIRECTIONS
    side = set(LEFT_DIRECTIONS if left else RIGHT_DIRECTIONS)

    lane_directions = [{str(d) for d in lane.directions} for lane in lanes]
    same_side = [directions for directions in lane_directions if directions & side]
    # with several exit lanes and few others, it is unclear which lane the car is in, so wait until the split is close
    ambiguous = len(same_side) > 1 and len(lanes) - len(same_side) <= NAV_KEEP_SMALL_SPLIT_MAX_OTHER_LANES
    keep_distance = np.interp(v_ego, NAV_KEEP_SPEED_BP, NAV_KEEP_DISTANCE_V) * (NAV_KEEP_AMBIGUOUS_SPLIT_SCALE if ambiguous else 1.)
    if not 0. <= distance <= keep_distance:
      return

    # the outermost lane of an ambiguous split that shares a lane with the road exits by itself
    at_edge = lane_index == (0 if left else len(lanes) - 1)
    if ambiguous and at_edge and any(directions - side for directions in same_side):
      return

    opposite_blinker = carstate.rightBlinker if left else carstate.leftBlinker
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    nudge = carstate.steeringPressed and (carstate.steeringTorque > 0 if left else carstate.steeringTorque < 0)
    if nudge and not opposite_blinker and not blindspot:
      self.keep_desire = log.Desire.keepLeft if left else log.Desire.keepRight
