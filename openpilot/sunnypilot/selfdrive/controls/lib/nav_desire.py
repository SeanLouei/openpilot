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
from openpilot.sunnypilot.selfdrive.controls.lib.relc import EDGE_PROB, NEARSIDE_PROB

TurnDirection = custom.ModelDataV2SP.TurnDirection

NAV_INSTRUCTION_MAX_AGE = 2.5  # s, navd publishes once a second
NAV_INSTRUCTION_GRACE = 3.0  # s a slowdown keeps going on the dead reckoned distance after navd's instructions stop

# The model gets a desire as a one-frame pulse and only remembers it for the 5 s its desire history covers, so a route
# turn pulses about 3 s before the car starts steering into it, and again every 4 s while the car waits short of it
NAV_TURN_TRIGGER_TIME = 3.0  # s
NAV_TURN_STEER_DISTANCE = 8.0  # m before the maneuver point where steering into an intersection turn starts
NAV_TURN_TRIGGER_MIN_SPEED = 0.5  # m/s; a car stopped short of the turn waits, one stopped at it fires
NAV_TURN_REPULSE_TIME = 4.0  # s
NAV_TURN_MAX_SPEED = 9.0  # m/s (20 mph); openpilot's lateral acceleration cap leaves faster turns running wide
AUTO_TURN_MAX_SPEED = 7.0  # m/s; an automatic turn, with no blinker to confirm it, also waits for turning speed

# An automatic turn starts only from the outermost lane: a confident road edge close beside the car with no lane line
# beyond the car's own, for a second. A bike lane, a parking lane or another lane puts the edge further out.
OUTER_EDGE_DISTANCE = 2.6  # m from the car's center
OUTER_LANE_TIME = 1.0  # s

# A blinker switched on for the route's next turn within this distance is its turn signal, not a lane change, for as long
# as it stays on
NAV_TURN_SIGNAL_TIME = 12.0  # s
NAV_TURN_SIGNAL_BASE_DISTANCE = 15.0  # m
NAV_TURN_SIGNAL_DISTANCE = (30.0, 250.0)  # m

# A maneuver's distance to the destination along the route stays put while the car approaches it, whatever a GPS error
# does to the car's position; only the next maneuver, or a reroute that turns somewhere else, moves it
NAV_SAME_MANEUVER_DISTANCE = 10.0  # m
NAV_MAX_PULL_IN = 5.0  # m/s a shorter navd distance pulls the dead reckoned one in at most, so a GPS jump ahead does little

# The slowdown: from the highest speed the car can slow to turn speed from at a gentle deceleration, to turn speed by the turn
NAV_TURN_SPEED = 4.5  # m/s; 10 mph keeps a tight right turn within openpilot's 3 m/s^2 lateral acceleration cap
NAV_TURN_MARGIN = 15.  # m before the maneuver point the car is down to turn speed
NAV_TURN_DECEL = 0.8  # m/s^2
NAV_TURN_HOLD = 20.  # m past the maneuver point the turn speed holds, through the turn

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
  long_active: bool = False  # openpilot longitudinal controls the speed and the driver isn't pressing the gas
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
  """Whether the model sees the car in the outermost lane on each side: a confident road edge close beside the car, and no
  lane line beyond the car's own."""
  stds, probs, edges = model.roadEdgeStds, model.laneLineProbs, model.roadEdges
  if len(stds) < 2 or len(probs) < 4 or len(edges) < 2 or not len(edges[0].y) or not len(edges[1].y):
    return False, False

  def outermost(side: int, outer_line: int) -> bool:
    return bool(1. - stds[side] > EDGE_PROB and probs[outer_line] < NEARSIDE_PROB and abs(edges[side].y[0]) < OUTER_EDGE_DISTANCE)

  return outermost(0, 0), outermost(1, 3)


def nav_inputs(sm, model, openpilot_longitudinal: bool) -> NavInputs:
  """Everything route desires need from modeld: its SubMaster, the model message just filled for this frame, and whether
  openpilot controls the car's speed, which carControl.longActive alone also reports for cruise button control."""
  instruction, age = fresh_nav_instruction(sm)
  long_active = openpilot_longitudinal and bool(sm['carControl'].longActive)
  return NavInputs(instruction, age, long_active, outer_lanes(model), not sm['driverMonitoringState'].isRHD)


def signal_distance(v_ego: float) -> float:
  return float(np.clip(NAV_TURN_SIGNAL_BASE_DISTANCE + max(v_ego, 0.) * NAV_TURN_SIGNAL_TIME, *NAV_TURN_SIGNAL_DISTANCE))


def turn_speed_limit(distance: float) -> float:
  """The highest speed this far before a turn from which the car slows to turn speed at the profile's deceleration."""
  return math.sqrt(NAV_TURN_SPEED ** 2 + 2 * NAV_TURN_DECEL * max(distance - NAV_TURN_MARGIN, 0.))


def approach_distance(v_ego: float) -> float:
  """How far before a route turn the driver calls it off: from where the car would signal for it, or start slowing for it."""
  return max(signal_distance(v_ego), NAV_TURN_MARGIN + max(v_ego ** 2 - NAV_TURN_SPEED ** 2, 0.) / (2 * NAV_TURN_DECEL))


def blinker_side(carstate) -> bool | None:
  """The side (left) of a single blinker, or None with neither or both (hazards)."""
  return bool(carstate.leftBlinker) if bool(carstate.leftBlinker) != bool(carstate.rightBlinker) else None


class ManeuverTracker:
  """The route's next maneuver and the distance to it, dead reckoned between navd's 1 Hz updates. A maneuver is its type,
  direction and distance to the destination. For one maneuver the distance only ever shrinks, and a shorter navd distance
  pulls it in at a limited rate, so neither a frozen or late navd distance nor a GPS jump moves it far; once the car
  passes the maneuver point it stays passed until navd reports a different maneuver."""

  def __init__(self):
    self.maneuver: tuple[str, str] | None = None  # (type, direction)
    self.position = math.nan  # m from the maneuver to the destination along the route
    self.distance = math.inf
    self.passed = False

  def update(self, instruction, age: float, v_ego: float) -> bool:
    """Advance one frame, along the last maneuver while there is no instruction. True when a different maneuver began."""
    v_ego = max(v_ego, 0.)
    self.distance -= v_ego * DT_MDL
    new = False
    if instruction is not None:
      maneuver = (str(instruction.maneuverType).strip().lower(), normalize_direction(instruction.maneuverModifier))
      reported = float(instruction.maneuverDistance) - v_ego * max(age, 0.)
      position = float(instruction.distanceRemaining) - float(instruction.maneuverDistance)
      if math.isfinite(reported) and math.isfinite(position):
        new = maneuver != self.maneuver or not abs(position - self.position) <= NAV_SAME_MANEUVER_DISTANCE
        if new:
          self.maneuver, self.position, self.distance, self.passed = maneuver, position, reported, False
        else:
          self.distance = max(min(self.distance, reported), self.distance - NAV_MAX_PULL_IN * DT_MDL)
    self.passed |= self.distance < 0.
    return new

  def turn(self) -> bool | None:
    """The side (left) of the maneuver when it is a turn at an intersection, else None."""
    if self.maneuver is None or self.maneuver[0] not in TURN_TYPES or self.maneuver[1] not in TURN_LEFT + TURN_RIGHT:
      return None
    return self.maneuver[1] in TURN_LEFT


class NavDesireController:
  """Desires and a speed limit from the navigation route, one state for modeld's desires and plannerd's slowdown.

  Turns: switching the blinker on for the route's next turn makes it that turn's signal until it goes off: it never starts
  a nudgeless lane change, and it holds back the lane turn desire. About 3 s before the car starts steering into the
  turn, below 20 mph, the turn desire starts. With automatic turns on and openpilot in control of the speed, a near side
  turn (right, where traffic keeps right) also starts without the blinker, once the car is down to turning speed in the
  outermost lane. Before the turn starts, the gas, the other blinker or disengaging calls off its automatic turn and its
  slowdown; signaling for it brings them back. Once it starts, the model has the turn: the desire holds until the
  maneuver point, pulsing again every 4 s while the car waits short of it, and the turn speed holds through the turn.
  Slowing: the speed limit slows the car to turn speed by every route turn with automatic turns on, else by the turn the
  driver signals for. plannerd follows it while openpilot controls the speed.
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
    self.odometer = 0.  # m driven
    self.gap = math.inf  # s since the last fresh route instruction
    self.outer_lane_time = [0., 0.]  # s the model has seen the car in the outermost lane on its (left, right)
    self.reset()

  def reset(self) -> None:
    """Forget any route turn: no desire, no turn signal, no speed limit, nothing held."""
    self.turn_direction = TurnDirection.none
    self.route_turn_signal = False
    self.keep_desire = log.Desire.none
    self.speed_limit = 0.  # m/s the route turn limits the speed to, 0 for none
    self.route_blinker: bool | None = None  # the side (left) of a blinker latched as the route turn's signal
    self.stale_blinker: bool | None = None  # the side (left) of a blinker on since an earlier maneuver
    self.blinker_turned = False  # the latched blinker already started its turn
    self.turning: bool | None = None  # the side (left) of the turn desire being held
    self.turn_by_blinker = False
    self.started = False  # the current maneuver's turn desire went out to the model
    self.called_off = False  # the current maneuver's automatic turn and slowdown are off, until the driver signals for it
    self.auto_off = False  # the current maneuver's automatic turn is off, until the driver signals for it
    self.pulse_frame = 0
    self.hold_end = -math.inf  # the odometer reading where the turn speed hold ends

  def read_params(self) -> None:
    self.turn_enabled = self.params.get_bool("NavTurnDesire")
    self.auto_enabled = self.params.get_bool("NavAutoTurn")
    self.exit_enabled = self.params.get_bool("NavExitDesire")

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  @property
  def owns_turn_desire(self) -> bool:
    """The route decides the turn desire: while a blinker is its turn's signal, and from its turn's start to the maneuver point."""
    return self.route_turn_signal or self.turning is not None or (self.started and not self.tracker.passed)

  def update(self, carstate, lateral_active: bool, nav: NavInputs | None, lane_changing: bool = False, lane_turning: bool = False) -> None:
    """lane_changing: a lane change is under way, so no turn starts and the slowdown waits. lane_turning: the lane turn
    desire already went out for this blinker, so the route does not take the blinker over, which would pulse again."""
    self.frame += 1
    self.turn_direction = TurnDirection.none
    self.keep_desire = log.Desire.none
    side = blinker_side(carstate)

    nav = nav if nav is not None else NavInputs()
    v_ego = max(float(carstate.vEgo), 0.)
    self.odometer += v_ego * DT_MDL
    self.gap = 0. if nav.instruction is not None else self.gap + DT_MDL
    self.outer_lane_time = [t + DT_MDL if outer else 0. for t, outer in zip(self.outer_lane_time, nav.outer_lanes, strict=True)]

    following = self.tracker.maneuver is not None
    if self.tracker.update(nav.instruction, nav.age, v_ego):
      self.started = self.called_off = self.auto_off = self.blinker_turned = False
      self.turning = self.route_blinker = None
      self.stale_blinker = side if following else None  # a blinker already on was for the maneuver before
    if side != self.stale_blinker:
      self.stale_blinker = None
    turn = self.tracker.turn()
    pending = (self.turn_enabled or self.auto_enabled) and turn is not None and not self.tracker.passed and self.gap <= NAV_INSTRUCTION_GRACE

    # a blinker switched on for the route's turn stays its signal until it goes off or over to the other side, whatever the
    # speed or route does; it brings back what the driver called off
    if side is None or side != self.route_blinker or not lateral_active:
      self.route_blinker = None
    if (pending and lateral_active and self.route_blinker is None and side == turn and side != self.stale_blinker
        and nav.instruction is not None and not lane_turning and self.tracker.distance <= signal_distance(v_ego)):
      self.route_blinker, self.blinker_turned, self.called_off, self.auto_off = side, False, False, False
    self.route_turn_signal = self.route_blinker is not None

    # before the turn starts, the driver calls off its automatic turn, and its slowdown with the gas, the other blinker or
    # disengaging the speed control
    if pending and not self.started and self.tracker.distance <= approach_distance(v_ego):
      other_blinker = side is not None and side != turn and side != self.stale_blinker
      if carstate.gasPressed or other_blinker or (self.auto_enabled and not nav.long_active):
        self.called_off = True
        self.hold_end = -math.inf
      self.auto_off |= self.called_off or not lateral_active or lane_changing

    if self.turning is not None:
      self._hold_turn(carstate, nav, lateral_active, side)
    elif pending and lateral_active and nav.instruction is not None:
      self._start_turn(carstate, nav, v_ego, turn, lane_changing)

    self._update_speed_limit(carstate, pending, turn, lane_changing)

    if (self.exit_enabled and self.turning is None and nav.instruction is not None and self.tracker.maneuver is not None and lateral_active
        and self.tracker.maneuver[0] in SPLIT_TYPES and self.tracker.maneuver[1] in LEFT_DIRECTIONS + RIGHT_DIRECTIONS):
      self._update_keep(carstate, v_ego, self.tracker.distance, nav.instruction.lanes)

  def _start_turn(self, carstate, nav: NavInputs, v_ego: float, left: bool, lane_changing: bool) -> None:
    if lane_changing or (carstate.leftBlindspot if left else carstate.rightBlindspot):
      return
    time_to_steer = max(self.tracker.distance - NAV_TURN_STEER_DISTANCE, 0.) / max(v_ego, NAV_TURN_TRIGGER_MIN_SPEED)
    if time_to_steer > NAV_TURN_TRIGGER_TIME:
      return

    # once the model has had this maneuver's turn, a new signal for it only pulses again while the car waits short of it
    waiting = self.tracker.distance > NAV_TURN_STEER_DISTANCE or carstate.standstill
    if self.route_blinker == left and v_ego < NAV_TURN_MAX_SPEED and not self.blinker_turned and (waiting or not self.started):
      self.turn_by_blinker = self.blinker_turned = True
    elif (self.auto_enabled and not (self.started or self.called_off or self.auto_off or carstate.leftBlinker or carstate.rightBlinker)
          and self._auto_turn_allowed(nav, v_ego, left)):
      self.turn_by_blinker = False
    else:
      return
    self.started, self.turning, self.pulse_frame = True, left, self.frame
    self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  def _hold_turn(self, carstate, nav: NavInputs, lateral_active: bool, side: bool | None) -> None:
    left = self.turning
    if self.route_blinker == left:
      self.turn_by_blinker = self.blinker_turned = True  # signaling for an automatic turn makes it the blinker's
    if self.turn_by_blinker:
      released = self.route_blinker != left  # the blinker went off, or over to the other side
    else:
      released = carstate.gasPressed or carstate.leftBlinker or carstate.rightBlinker or not nav.long_active
    # the model keeps its pulse, so letting go only ends the turn desire's reminders, and the alert
    if released or not lateral_active or self.tracker.passed:
      self.turning = None
      return

    # while the car waits short of the turn, a fresh pulse keeps it inside the model's desire history
    waiting = self.tracker.distance > NAV_TURN_STEER_DISTANCE or carstate.standstill
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    if waiting and not blindspot and (self.frame - self.pulse_frame) * DT_MDL >= NAV_TURN_REPULSE_TIME:
      self.pulse_frame = self.frame + 1  # no desire for this frame, so the next one is a new rising edge
      return
    self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  def _auto_turn_allowed(self, nav: NavInputs, v_ego: float, left: bool) -> bool:
    if not nav.long_active or v_ego > AUTO_TURN_MAX_SPEED:
      return False
    # only a near side turn, from the outermost lane the model can see; a far side turn crosses oncoming traffic from a
    # lane the model cannot tell apart from the others, so it waits for the blinker
    return left != nav.right_hand_traffic and self.outer_lane_time[0 if left else 1] >= OUTER_LANE_TIME - 1e-6

  def _update_speed_limit(self, carstate, pending: bool, turn: bool | None, lane_changing: bool) -> None:
    # the route turn's slowdown: a started turn's until the car is through it, else every turn's with automatic turns on,
    # or the one the driver signals for, while nothing called it off and no lane change is under way
    no_blinker = not (carstate.leftBlinker or carstate.rightBlinker)
    slowing = pending and (self.started or (not self.called_off and not lane_changing and
                                            ((self.auto_enabled and no_blinker) or self.route_blinker == turn)))
    if slowing and self.tracker.distance <= NAV_TURN_MARGIN:
      self.hold_end = self.odometer + max(self.tracker.distance, 0.) + NAV_TURN_HOLD  # through the turn, even once navd moves on
    limit = turn_speed_limit(self.tracker.distance) if slowing else math.inf
    if self.odometer < self.hold_end:
      limit = min(limit, NAV_TURN_SPEED)
    self.speed_limit = limit if math.isfinite(limit) else 0.

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
