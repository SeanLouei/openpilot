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
import math
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from openpilot.cereal import log, custom
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.relc import EDGE_CLEARANCE, EDGE_PROB, NEARSIDE_PROB, VEHICLE_EDGE_MARGIN

TurnDirection = custom.ModelDataV2SP.TurnDirection

NAV_INSTRUCTION_MAX_AGE = 2.5  # s, navd publishes once a second

# The model gets a single desire pulse and decides itself when the turn is done, so a route turn only starts
# its desire once the intersection is close: the pulse then lands on the route's intersection, not an earlier one.
NAV_TURN_MAX_SPEED = 14.0  # m/s
NAV_TURN_PREVIEW_TIME = 6.0  # s
NAV_TURN_PREVIEW_DISTANCE = (35.0, 90.0)  # m
# An automatic turn, with no blinker to confirm it, also waits until the car has slowed to turning speed
AUTO_TURN_MAX_SPEED = 7.0  # m/s
# A started turn holds until just past its maneuver point; navd moves on to the next maneuver 10 m past it
NAV_TURN_RELEASE_DISTANCE = -5.0  # m
NAV_NEW_MANEUVER_JUMP = 30.0  # m; the distance growing this much means a different maneuver, after a reroute

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


@dataclass
class NavInputs:
  instruction: Any = None  # the latest navInstruction, None without a valid, recent one
  age: float = 0.  # s since navd published it
  long_active: bool = False  # openpilot controls the speed and the driver isn't pressing the gas
  outer_lanes: tuple[bool, bool] = (False, False)  # whether the car is in the outermost lane on its (left, right)
  right_hand_traffic: bool = True


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


def outer_lanes(model) -> tuple[bool, bool]:
  """Whether the model sees a road edge close on each side with no lane beyond the car's own: the outermost lane.
  The same test as the road-edge lane change block, without its minimum speed, since turns happen slowly."""
  stds, probs, edges = model.roadEdgeStds, model.laneLineProbs, model.roadEdges
  if len(stds) < 2 or len(probs) < 4 or len(edges) < 2 or not len(edges[0].y) or not len(edges[1].y):
    return False, False

  def outermost(side: int, outer_line: int) -> bool:
    edge_close = abs(edges[side].y[0]) - VEHICLE_EDGE_MARGIN < EDGE_CLEARANCE
    return bool(1. - stds[side] > EDGE_PROB and probs[outer_line] < NEARSIDE_PROB and edge_close)

  return outermost(0, 0), outermost(1, 3)


def nav_inputs(sm, model) -> NavInputs:
  """Everything route desires need from modeld: its SubMaster, and the model message just filled for this frame."""
  instruction, age = fresh_nav_instruction(sm)
  return NavInputs(instruction, age, bool(sm['carControl'].longActive), outer_lanes(model), not sm['driverMonitoringState'].isRHD)


def route_turn(instruction, age: float, v_ego: float) -> tuple[bool, float] | None:
  """(left, distance in m) of the route's next maneuver when it is a turn at an intersection, else None.
  navd only updates once a second, so the distance is dead reckoned since then."""
  maneuver_type = str(instruction.maneuverType).strip().lower()
  direction = normalize_direction(instruction.maneuverModifier)
  distance = float(instruction.maneuverDistance) - max(v_ego, 0.) * max(age, 0.)
  if maneuver_type not in TURN_TYPES or direction not in TURN_LEFT + TURN_RIGHT or not math.isfinite(distance):
    return None
  return direction in TURN_LEFT, distance


def signals_route_turn(carstate, left: bool, distance: float, v_ego: float) -> bool:
  """Whether the driver's blinker is for the route turn this far ahead, rather than for a lane change."""
  blinker = (carstate.leftBlinker and not carstate.rightBlinker) if left else (carstate.rightBlinker and not carstate.leftBlinker)
  signal_distance = np.clip(NAV_TURN_SIGNAL_BASE_DISTANCE + max(v_ego, 0.) * NAV_TURN_SIGNAL_TIME, *NAV_TURN_SIGNAL_DISTANCE)
  return bool(blinker and 0. <= distance <= signal_distance)


class NavDesireController:
  """Desires from the navigation route.

  Turns: a blinker matching the route's next turn starts a turn desire once that intersection is close. Until then,
  that blinker is held back from starting a nudgeless lane change or an early lane turn desire. With automatic turns
  on, openpilot in control of the speed, and the car slowed to turning speed in a lane it can
  turn from, the turn desire starts without the blinker. A turn desire starts once per maneuver and holds until its
  maneuver point; after that, only switching the blinker on again restarts it.
  Exits and forks: a nudge toward the side the route's lane guidance takes gives a keep desire near the split.
  """

  def __init__(self):
    self.params = Params()
    self.param_read_counter = 0
    self.turn_enabled = False
    self.auto_enabled = False
    self.exit_enabled = False
    self.read_params()

    self.turn_direction = TurnDirection.none
    self.route_turn_signal = False
    self.keep_desire = log.Desire.none

    self.maneuver: tuple[str, str] | None = None  # (type, direction) of the route's current maneuver
    self.maneuver_distance = math.inf
    self.started = False  # a turn desire has already started for the current maneuver
    self.holding: bool | None = None  # the turn being held, as left, while its desire is on
    self.held_by_blinker = False
    self.prev_blinkers = (False, False)

  def read_params(self) -> None:
    self.turn_enabled = self.params.get_bool("NavTurnDesire")
    self.auto_enabled = self.params.get_bool("NavAutoTurn")
    self.exit_enabled = self.params.get_bool("NavExitDesire")

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update(self, carstate, lateral_active: bool, nav: NavInputs | None) -> None:
    self.turn_direction = TurnDirection.none
    self.route_turn_signal = False
    self.keep_desire = log.Desire.none
    blinkers = (bool(carstate.leftBlinker), bool(carstate.rightBlinker))
    prev_blinkers, self.prev_blinkers = self.prev_blinkers, blinkers

    instruction = nav.instruction if nav is not None else None
    if instruction is None or not lateral_active:
      self.holding = None  # the maneuver is remembered, so a short gap never restarts a turn already started
      return

    v_ego = max(float(carstate.vEgo), 0.)
    distance = float(instruction.maneuverDistance) - v_ego * max(nav.age, 0.)
    maneuver = (str(instruction.maneuverType).strip().lower(), normalize_direction(instruction.maneuverModifier))
    if maneuver != self.maneuver or not distance <= self.maneuver_distance + NAV_NEW_MANEUVER_JUMP:
      self.started, self.holding = False, None
    self.maneuver, self.maneuver_distance = maneuver, distance

    turn = route_turn(instruction, nav.age, v_ego)
    if turn is not None and (self.turn_enabled or self.auto_enabled):
      self._update_turn(carstate, nav, v_ego, *turn, blinkers, prev_blinkers, instruction.lanes)
    elif self.exit_enabled and maneuver[0] in SPLIT_TYPES and maneuver[1] in LEFT_DIRECTIONS + RIGHT_DIRECTIONS and math.isfinite(distance):
      self._update_keep(carstate, v_ego, distance, instruction.lanes)

  def _update_turn(self, carstate, nav: NavInputs, v_ego: float, left: bool, distance: float, blinkers, prev_blinkers, lanes) -> None:
    blinker = blinkers == ((True, False) if left else (False, True))
    blinker_on_again = blinker and not prev_blinkers[0 if left else 1]
    self.route_turn_signal = signals_route_turn(carstate, left, distance, v_ego)

    if self.holding is not None:
      if self.held_by_blinker:
        called_off = not blinker  # the blinker went off, or over to the other side
      else:
        called_off = (any(blinkers) and not blinker) or not nav.long_active
      if distance < NAV_TURN_RELEASE_DISTANCE or called_off:
        self.holding = None
      else:
        self.turn_direction = TurnDirection.turnLeft if self.holding else TurnDirection.turnRight
        return

    preview_distance = np.clip(v_ego * NAV_TURN_PREVIEW_TIME, *NAV_TURN_PREVIEW_DISTANCE)
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    if not 0. <= distance <= preview_distance or carstate.standstill or blindspot:
      return
    if self.started and not blinker_on_again:
      return

    if blinker and v_ego < NAV_TURN_MAX_SPEED:
      self.held_by_blinker = True
    elif self.auto_enabled and not any(blinkers) and self._auto_turn_allowed(nav, v_ego, left, lanes):
      self.held_by_blinker = False
    else:
      return
    self.started, self.holding = True, left
    self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  @staticmethod
  def _auto_turn_allowed(nav: NavInputs, v_ego: float, left: bool, lanes) -> bool:
    if not nav.long_active or v_ego > AUTO_TURN_MAX_SPEED:
      return False
    if left != nav.right_hand_traffic:
      # a near side turn (right, where traffic keeps right) must start from the outermost lane, which the model can see
      return nav.outer_lanes[0 if left else 1]
    # a far side turn crosses oncoming traffic from the innermost lane, which looks the same to the model as a lane of
    # the same direction: only when the route shows no choice of lanes for it
    return len(lanes) <= 1

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
