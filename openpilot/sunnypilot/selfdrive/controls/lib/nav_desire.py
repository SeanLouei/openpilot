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
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.relc import EDGE_CLEARANCE, EDGE_PROB, NEARSIDE_PROB, VEHICLE_EDGE_MARGIN

TurnDirection = custom.ModelDataV2SP.TurnDirection

NAV_INSTRUCTION_MAX_AGE = 2.5  # s, navd publishes once a second

# The model gets a desire as a one-frame pulse and only remembers it for the 5 s its desire history covers, so a route
# turn pulses about 3 s before the car starts steering into it, and again every 4 s while the car waits short of it
NAV_TURN_TRIGGER_TIME = 3.0  # s
NAV_TURN_STEER_DISTANCE = 8.0  # m before the maneuver point where steering into an intersection turn starts
NAV_TURN_TRIGGER_MIN_SPEED = 0.5  # m/s; a car stopped short of the turn waits, one stopped at it fires
NAV_TURN_REPULSE_TIME = 4.0  # s
NAV_TURN_MAX_SPEED = 9.0  # m/s (20 mph); openpilot's lateral acceleration cap leaves faster turns running wide
AUTO_TURN_MAX_SPEED = 7.0  # m/s; an automatic turn, with no blinker to confirm it, also waits for turning speed

# A blinker for the route's next turn within this distance is its turn signal, not a lane change, for as long as it stays on
NAV_TURN_SIGNAL_TIME = 12.0  # s
NAV_TURN_SIGNAL_BASE_DISTANCE = 15.0  # m
NAV_TURN_SIGNAL_DISTANCE = (30.0, 250.0)  # m

NAV_NEW_MANEUVER_JUMP = 30.0  # m; the distance growing this much means a different maneuver, after a reroute

# Exits and forks get their keep desire close to the split, closer still when the exit shares lanes with the road
NAV_KEEP_SPEED_BP = [0.0, 15.0, 30.0]  # m/s
NAV_KEEP_DISTANCE_V = [25.0, 90.0, 160.0]  # m
NAV_KEEP_AMBIGUOUS_SPLIT_SCALE = 0.6
NAV_KEEP_SMALL_SPLIT_MAX_OTHER_LANES = 2

# Mapbox maneuver types that are turns at an intersection with a left or right modifier; 'continue' stays on its road
TURN_TYPES = ('turn', 'end of road', 'on ramp', 'continue')
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


def signal_distance(v_ego: float) -> float:
  return float(np.clip(NAV_TURN_SIGNAL_BASE_DISTANCE + max(v_ego, 0.) * NAV_TURN_SIGNAL_TIME, *NAV_TURN_SIGNAL_DISTANCE))


def blinker_side(carstate) -> bool | None:
  """The side (left) of a single blinker, or None with neither or both (hazards)."""
  return bool(carstate.leftBlinker) if bool(carstate.leftBlinker) != bool(carstate.rightBlinker) else None


class ManeuverTracker:
  """The route's next maneuver and the distance to it, dead reckoned between navd's 1 Hz updates. For one maneuver the
  distance only ever shrinks, so a frozen or late navd distance never moves it back over a threshold, and once the car
  passes the maneuver point it stays passed until navd reports a different maneuver."""

  def __init__(self):
    self.maneuver: tuple[str, str] | None = None  # (type, direction)
    self.distance = math.inf
    self.reported = math.inf  # navd's last distance, dead reckoned to when it arrived
    self.passed = False

  def update(self, instruction, age: float, v_ego: float) -> bool:
    """Advance one frame, along the last maneuver while there is no instruction. True when a different maneuver began."""
    v_ego = max(v_ego, 0.)
    self.distance -= v_ego * DT_MDL
    new = False
    if instruction is not None:
      maneuver = (str(instruction.maneuverType).strip().lower(), normalize_direction(instruction.maneuverModifier))
      reported = float(instruction.maneuverDistance) - v_ego * max(age, 0.)
      if math.isfinite(reported):
        # a reroute jumps navd's distance up; a frozen one, or a turn navd still reports just ahead after it, does not
        new = maneuver != self.maneuver or reported > self.reported + NAV_NEW_MANEUVER_JUMP
        if new:
          self.maneuver, self.distance, self.passed = maneuver, reported, False
        else:
          self.distance = min(self.distance, reported)
        self.reported = reported
    self.passed |= self.distance < 0.
    return new

  def turn(self) -> bool | None:
    """The side (left) of the maneuver when it is a turn at an intersection, else None."""
    if self.maneuver is None or self.maneuver[0] not in TURN_TYPES or self.maneuver[1] not in TURN_LEFT + TURN_RIGHT:
      return None
    return self.maneuver[1] in TURN_LEFT


class NavDesireController:
  """Desires from the navigation route.

  Turns: a blinker for the route's next turn is that turn's signal until it goes off: it never starts a nudgeless lane
  change, and it holds back the lane turn desire. About 3 s before the car starts steering into the turn, below 20 mph,
  the turn desire starts. With automatic turns on, openpilot in control of the speed, and the car slowed to turning speed
  in a lane it can turn from, it starts without the blinker. A maneuver's turn desire starts once, holds until the
  maneuver point, and pulses again every 4 s while the car waits short of the turn; after it is called off, only the
  blinker coming on again restarts it.
  Exits and forks: a nudge toward the side the route's lane guidance takes gives a keep desire near the split.
  """

  def __init__(self):
    self.params = Params()
    self.param_read_counter = 0
    self.turn_enabled = False
    self.auto_enabled = False
    self.exit_enabled = False
    self.read_params()

    self.tracker = ManeuverTracker()
    self.frame = 0
    self.prev_blinkers = (False, False)
    self.reset()

  def reset(self) -> None:
    """Forget any route turn: no desire, no turn signal, nothing held."""
    self.turn_direction = TurnDirection.none
    self.route_turn_signal = False
    self.keep_desire = log.Desire.none
    self.route_blinker: bool | None = None  # the side (left) of a blinker latched as the route turn's signal
    self.turning: bool | None = None  # the side (left) of the turn desire being held
    self.turn_by_blinker = False
    self.started = False  # the current maneuver's turn desire has started
    self.pulse_frame = 0

  def read_params(self) -> None:
    self.turn_enabled = self.params.get_bool("NavTurnDesire")
    self.auto_enabled = self.params.get_bool("NavAutoTurn")
    self.exit_enabled = self.params.get_bool("NavExitDesire")

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update(self, carstate, lateral_active: bool, nav: NavInputs | None, lane_changing: bool = False, lane_turning: bool = False) -> None:
    """lane_changing: a lane change is under way, so no turn starts. lane_turning: the lane turn desire already went out
    for this blinker, so the route does not take the blinker over, which would end that desire and pulse again."""
    self.frame += 1
    self.turn_direction = TurnDirection.none
    self.keep_desire = log.Desire.none
    blinkers = (bool(carstate.leftBlinker), bool(carstate.rightBlinker))
    prev_blinkers, self.prev_blinkers = self.prev_blinkers, blinkers
    side = blinker_side(carstate)

    nav = nav if nav is not None else NavInputs()
    v_ego = max(float(carstate.vEgo), 0.)
    if self.tracker.update(nav.instruction, nav.age, v_ego):
      self.started, self.turning = False, None
    turn = self.tracker.turn()
    route_turns = self.turn_enabled or self.auto_enabled

    # the blinker stays the route turn's signal until it goes off or over to the other side, whatever the speed or route does
    if side is None or side != self.route_blinker or not lateral_active:
      self.route_blinker = None
    if (route_turns and lateral_active and self.route_blinker is None and side is not None and side == turn and nav.instruction is not None
        and not lane_turning and not self.tracker.passed and self.tracker.distance <= signal_distance(v_ego)):
      self.route_blinker = side
    self.route_turn_signal = self.route_blinker is not None

    if self.turning is not None:
      self._hold_turn(carstate, nav, lateral_active, side)
    elif route_turns and turn is not None and lateral_active and not lane_changing and nav.instruction is not None:
      self._start_turn(carstate, nav, v_ego, turn, side, prev_blinkers, nav.instruction.lanes)

    if (self.exit_enabled and self.turning is None and nav.instruction is not None and self.tracker.maneuver is not None and lateral_active
        and self.tracker.maneuver[0] in SPLIT_TYPES and self.tracker.maneuver[1] in LEFT_DIRECTIONS + RIGHT_DIRECTIONS):
      self._update_keep(carstate, v_ego, self.tracker.distance, nav.instruction.lanes)

  def _start_turn(self, carstate, nav: NavInputs, v_ego: float, left: bool, side: bool | None, prev_blinkers, lanes) -> None:
    if self.tracker.passed or (carstate.leftBlindspot if left else carstate.rightBlindspot):
      return
    if self.started and not (side == left and not prev_blinkers[0 if left else 1]):
      return  # this maneuver already had its turn: only the blinker coming on again restarts it
    time_to_steer = max(self.tracker.distance - NAV_TURN_STEER_DISTANCE, 0.) / max(v_ego, NAV_TURN_TRIGGER_MIN_SPEED)
    if time_to_steer > NAV_TURN_TRIGGER_TIME:
      return

    if self.route_blinker == left and v_ego < NAV_TURN_MAX_SPEED:
      self.turn_by_blinker = True
    elif self.auto_enabled and not (carstate.leftBlinker or carstate.rightBlinker) and self._auto_turn_allowed(nav, v_ego, left, lanes):
      self.turn_by_blinker = False
    else:
      return
    self.started, self.turning, self.pulse_frame = True, left, self.frame
    self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  def _hold_turn(self, carstate, nav: NavInputs, lateral_active: bool, side: bool | None) -> None:
    left = self.turning
    if self.turn_by_blinker:
      called_off = side != left  # the blinker went off, or over to the other side
    else:
      called_off = ((carstate.leftBlinker or carstate.rightBlinker) and side != left) or not nav.long_active
    if called_off or not lateral_active or self.tracker.passed:
      self.turning = None
      return

    # while the car waits short of the turn, a fresh pulse keeps it inside the model's desire history
    waiting = self.tracker.distance > NAV_TURN_STEER_DISTANCE or carstate.standstill
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    if waiting and not blindspot and (self.frame - self.pulse_frame) * DT_MDL >= NAV_TURN_REPULSE_TIME:
      self.pulse_frame = self.frame + 1  # no desire for this frame, so the next one is a new rising edge
      return
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
