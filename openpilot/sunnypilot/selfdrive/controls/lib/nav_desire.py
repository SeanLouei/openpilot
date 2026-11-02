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
from collections import deque
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
# turn pulses about 3 s before the car starts steering into it. A fresh pulse every 4 s only keeps it there: while the
# car waits at a crawl still lined up with the approach, or while it is at least NAV_TURN_REFRESH_LEAD from the steering point.
NAV_TURN_TRIGGER_TIME = 3.0  # s
NAV_TURN_STEER_DISTANCE = 8.0  # m before the maneuver point where steering into an intersection turn starts
NAV_TURN_TRIGGER_MIN_SPEED = 0.5  # m/s; a car stopped short of the turn waits, one stopped at it fires
NAV_TURN_REPULSE_TIME = 4.0  # s
NAV_TURN_REFRESH_LEAD = 2.0  # s a moving car is still from the steering point when its turn desire is refreshed, at least
NAV_TURN_MAX_SPEED = 9.0  # m/s (20 mph); openpilot's lateral acceleration cap leaves faster turns running wide
AUTO_TURN_MAX_SPEED = 7.0  # m/s; an automatic turn, with no blinker to confirm it, also waits for turning speed

# A turn starts only while the car can still make it, else the car goes straight: moving, before the steering point and
# slow enough to be down to turn speed by it, or at a crawl while still lined up with the approach
NAV_TURN_CRAWL_SPEED = 1.5  # m/s
NAV_TURN_REACH_DECEL = 1.2  # m/s^2, below plannerd's NAV_TURN_MAX_DECEL for its jerk ramp and the actuator lag
NAV_TURN_LAG = 0.3  # s before the braking takes hold
NAV_TURN_REACH_MARGIN = 0.5  # m/s
# Without openpilot's speed control nothing slows the car for the turn (Tesla's own cruise cannot go below 18 mph)
NAV_TURN_STOCK_NEAR_SPEED = 5.5  # m/s for a near side turn (right, where traffic keeps right), the tight one
NAV_TURN_STOCK_FAR_SPEED = 6.7  # m/s for a far side turn
# How far the car has turned toward the turn's side since it came this close to it: past NAV_TURN_LINED_UP it is in the
# turn, so a stop there gets no new turn pulse
NAV_TURN_LINEUP_DISTANCE = 40.  # m
NAV_TURN_LINED_UP = 25.  # deg

# An automatic turn starts only from the outermost lane: a confident road edge close beside the car with no lane line
# beyond the car's own, for a second, counted again after a lane change. Only the edge's distance tells a lane beside the
# car from the curb: a narrow painted bike lane may not put it beyond OUTER_EDGE_DISTANCE, and one behind a curb or posts
# looks like the edge itself.
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
# Between navd's updates the distance to the maneuver is dead reckoned, and the median of navd's last few residuals
# corrects it, so one bad fix does little
NAV_RESIDUALS = 3  # navd messages
NAV_MAX_PULL_IN = 5.0  # m/s a shorter navd distance pulls the dead reckoned one in at most
NAV_PUSH_DEADBAND = 3.0  # m a longer navd distance is beyond the dead reckoned one before it pushes it out
NAV_MAX_PUSH_OUT = 1.0  # m/s; follows a speed or map scale error of a few percent, not a GPS jump
NAV_RESEED_DISTANCE = 15.0  # m beyond which navd's distance replaces the dead reckoned one outright
# This close to the maneuver a GPS error behind the car would time the turn late, so there a much longer navd distance
# takes back at once only what navd pulled in lately (a GPS error ahead that ended), and otherwise holds the turn back: a
# turn starts only while navd agrees, and judged on the nearer of the two distances
NAV_NEAR_DISTANCE = 60.0  # m
NAV_PULL_IN_MEMORY = 20.0  # s over which a pull-in fades
NAV_AGREE_DISTANCE = 8.0  # m navd's distance may be beyond the dead reckoned one for a turn to start

# The slowdown: from the highest speed the car can slow to turn speed from at a gentle deceleration, to turn speed by the turn
NAV_TURN_SPEED = 4.5  # m/s; 10 mph keeps a tight right turn within openpilot's 3 m/s^2 lateral acceleration cap
NAV_TURN_MARGIN = 15.  # m before the maneuver point the car is down to turn speed
NAV_TURN_DECEL = 0.8  # m/s^2
NAV_TURN_MAX_DECEL = 1.5  # m/s^2 plannerd slows for a route turn at most, when the turn comes up late
NAV_TURN_HOLD = 20.  # m past the maneuver point the turn speed holds, through the turn
NAV_TURN_TIME_STEPS = 32  # steps at most of the time to the steering point along the slowdown

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
  yaw_rate: float = 0.  # rad/s the car turns at, positive left


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


def yaw_rate(model) -> float:
  """The model's yaw rate now, positive left (the model's frame is forward, right, down), 0 without one."""
  rates = getattr(getattr(model, 'orientationRate', None), 'z', ())
  rate = float(rates[0]) if len(rates) else 0.
  return -rate if math.isfinite(rate) else 0.


def nav_inputs(sm, model, openpilot_longitudinal: bool) -> NavInputs:
  """Everything route desires need from modeld: its SubMaster, the model message just filled for this frame, and whether
  openpilot controls the car's speed, which carControl.longActive alone also reports for cruise button control."""
  instruction, age = fresh_nav_instruction(sm)
  long_active = openpilot_longitudinal and bool(sm['carControl'].longActive)
  return NavInputs(instruction, age, long_active, outer_lanes(model), not sm['driverMonitoringState'].isRHD, yaw_rate(model))


def signal_distance(v_ego: float) -> float:
  return float(np.clip(NAV_TURN_SIGNAL_BASE_DISTANCE + max(v_ego, 0.) * NAV_TURN_SIGNAL_TIME, *NAV_TURN_SIGNAL_DISTANCE))


def turn_speed_limit(distance: float) -> float:
  """The highest speed this far before a turn from which the car slows to turn speed at the profile's deceleration."""
  return math.sqrt(NAV_TURN_SPEED ** 2 + 2 * NAV_TURN_DECEL * max(distance - NAV_TURN_MARGIN, 0.))


def approach_distance(v_ego: float) -> float:
  """How far before a route turn the driver calls it off: from where the car would signal for it, or start slowing for it."""
  return max(signal_distance(v_ego), NAV_TURN_MARGIN + max(v_ego ** 2 - NAV_TURN_SPEED ** 2, 0.) / (2 * NAV_TURN_DECEL))


def time_to_steer(distance: float, v_ego: float, slowing: bool) -> float:
  """Seconds until the car reaches the steering point: at its speed, or while openpilot slows it for this turn, at the
  speed the slowdown will impose, the profile reached braking at most NAV_TURN_MAX_DECEL."""
  if not math.isfinite(distance):
    return math.inf
  run = distance - NAV_TURN_STEER_DISTANCE
  if run <= 0.:
    return 0.
  if not slowing:
    return run / max(v_ego, NAV_TURN_TRIGGER_MIN_SPEED)
  steps = min(math.ceil(run), NAV_TURN_TIME_STEPS)
  ds = run / steps
  t = 0.
  for i in range(steps):
    s = NAV_TURN_STEER_DISTANCE + (i + 0.5) * ds  # m before the maneuver point
    braked = math.sqrt(max(v_ego ** 2 - 2 * NAV_TURN_MAX_DECEL * (distance - s), 0.))
    t += ds / max(min(v_ego, max(turn_speed_limit(s), braked)), NAV_TURN_TRIGGER_MIN_SPEED)
  return t


def blinker_side(carstate) -> bool | None:
  """The side (left) of a single blinker, or None with neither or both (hazards)."""
  return bool(carstate.leftBlinker) if bool(carstate.leftBlinker) != bool(carstate.rightBlinker) else None


class ManeuverTracker:
  """The route's next maneuver and the distance to it, dead reckoned between navd's updates. A maneuver is its type,
  direction and distance to the destination. Each navd message leaves a residual, its distance less the dead reckoned
  one, and the median of the last few corrects the distance: in at a limited rate, out slowly (a speed or map scale
  error), or out at once when navd's is clearly longer (the car left the route or joined it from beside it, or the GPS
  lagged), but close to the maneuver only by what navd lately pulled in. So one bad fix does little and a frozen navd
  distance holds the approach back. Once the maneuver's turn started or the car passed its point the distance only comes
  in, so nothing restarts or moves a started turn, and a passed maneuver stays passed until navd reports a different one."""

  def __init__(self):
    self.maneuver: tuple[str, str] | None = None  # (type, direction)
    self.position = math.nan  # m from the maneuver to the destination along the route
    self.distance = math.inf
    self.passed = False
    self.instruction = None  # navd's last message
    self.residuals: deque[float] = deque(maxlen=NAV_RESIDUALS)  # m navd's distance is beyond the dead reckoned one, left to correct
    self.residual = 0.  # their median
    self.pulled_in = 0.  # m navd pulled the distance in lately, fading over NAV_PULL_IN_MEMORY

  @property
  def nearest(self) -> float:
    """The nearer of the dead reckoned distance and navd's."""
    return self.distance + min(self.residual, 0.)

  def update(self, instruction, age: float, v_ego: float, committed: bool = False) -> bool:
    """Advance one frame, along the last maneuver while there is no instruction. committed: the maneuver's turn started.
    True when a different maneuver began."""
    v_ego = max(v_ego, 0.)
    self.distance -= v_ego * DT_MDL
    new = False
    if instruction is not None and instruction is not self.instruction:  # navd's next message
      self.instruction = instruction
      maneuver = (str(instruction.maneuverType).strip().lower(), normalize_direction(instruction.maneuverModifier))
      reported = float(instruction.maneuverDistance) - v_ego * max(age, 0.)
      position = float(instruction.distanceRemaining) - float(instruction.maneuverDistance)
      if math.isfinite(reported) and math.isfinite(position):
        new = maneuver != self.maneuver or not abs(position - self.position) <= NAV_SAME_MANEUVER_DISTANCE
        if new:
          # the first message counts as agreeing until others come, so it takes two to move the distance
          self.maneuver, self.position, self.distance, self.passed, self.pulled_in = maneuver, position, reported, False, 0.
          self.residuals.extend([0.] * NAV_RESIDUALS)
        else:
          self.residuals.append(reported - self.distance)

    m = float(np.median(self.residuals)) if self.residuals else 0.
    push_out = min(max(m - NAV_PUSH_DEADBAND, 0.), NAV_MAX_PUSH_OUT * DT_MDL)  # a speed or map scale error
    if m < 0. or committed or self.passed:
      step = max(min(m, 0.), -NAV_MAX_PULL_IN * DT_MDL)
    elif m <= NAV_RESEED_DISTANCE:
      step = push_out
    elif self.distance > NAV_NEAR_DISTANCE:
      step = m  # navd says the maneuver is clearly farther: the car left the route or joined it, or the GPS lagged
    else:
      step = max(min(m - NAV_PUSH_DEADBAND, self.pulled_in), push_out)  # close to it, only a GPS error ahead that ended
    self.distance += step
    for i in range(len(self.residuals)):
      self.residuals[i] -= step
    self.residual = m - step
    self.pulled_in = max(self.pulled_in * math.exp(-DT_MDL / NAV_PULL_IN_MEMORY) - step, 0.)
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
  a nudgeless lane change, and it holds back the lane turn desire. A blinker already on before the car passed the last
  maneuver is not its signal. About 3 s before the car starts steering into the turn, timed on the speed the slowdown will
  impose, the turn desire starts, but only while the car can still make the turn: before the steering point and slow
  enough to be down to turn speed by it (below 20 mph; without openpilot's speed control, which nothing then slows, at
  turning speed already), or at a crawl while still lined up with the approach. A car that cannot goes straight. With
  automatic turns on and openpilot in control of the speed, a near side turn (right, where traffic keeps right) also
  starts without the blinker, once the car has been in the outermost lane for a second. Before the turn starts, the gas,
  the other blinker or disengaging calls off its automatic turn and its slowdown; signaling for it brings them back. Once
  it starts, the model has the turn: the desire holds until the maneuver point, pulsing again every 4 s only to stay in
  the model's desire history (while the car waits at a crawl still lined up, or is still 2 s or more from the steering
  point), and no other turn starts until the car is through it.
  Slowing: the speed limit slows the car to turn speed by every route turn with automatic turns on, else by the turn the
  driver signals for, through a lane change too, and by a started turn until the car passes it. The turn's point is
  pinned on the odometer once it starts or the car is close, so the turn speed holds to 20 m past it even once navd moves
  on. plannerd follows it while openpilot controls the speed.
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
    self.blinker: bool | None = None  # the side (left) of the blinker on, and the frame it came on
    self.blinker_onset = 0
    self.passed_frame: int | None = None  # the frame the car passed the current maneuver's point
    self.maneuver_id = 0
    self.pins: dict[int, float] = {}  # the odometer reading of each maneuver's turn point, by maneuver
    self.turned = 0.  # deg the car turned toward the current turn's side since it came close to it
    self.reset()

  def reset(self) -> None:
    """Forget any route turn: no desire, no turn signal, no speed limit, nothing held. The pinned turn points stay, so a
    turn already under way keeps its slowdown once the route can be read again."""
    self.turn_direction = TurnDirection.none
    self.route_turn_signal = False
    self.keep_desire = log.Desire.none
    self.speed_limit = 0.  # m/s the route turn limits the speed to, 0 for none
    self.slowing = False  # the current maneuver's slowdown is on
    self.route_blinker: bool | None = None  # the side (left) of a blinker latched as the route turn's signal
    self.stale_blinker: bool | None = None  # the side (left) of a blinker on since before the car passed the last maneuver
    self.blinker_turned = False  # the latched blinker already started its turn
    self.turning: bool | None = None  # the side (left) of the turn desire being held
    self.turn_by_blinker = False
    self.started = False  # the current maneuver's turn desire went out to the model
    self.called_off = False  # the current maneuver's automatic turn and slowdown are off, until the driver signals for it
    self.auto_off = False  # the current maneuver's automatic turn is off, until the driver signals for it
    self.pulse_frame = 0

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

  @property
  def lined_up(self) -> bool:
    """The car still faces along the approach, not into the turn."""
    return self.turned < NAV_TURN_LINED_UP

  def update(self, carstate, lateral_active: bool, nav: NavInputs | None, lane_changing: bool = False, lane_turning: bool = False) -> None:
    """lane_changing: a lane change is under way, so no turn starts and the outermost lane counts again after it.
    lane_turning: the lane turn desire already went out for this blinker, so the route does not take the blinker over,
    which would pulse again."""
    self.frame += 1
    self.turn_direction = TurnDirection.none
    self.keep_desire = log.Desire.none
    side = blinker_side(carstate)
    if side != self.blinker:
      self.blinker, self.blinker_onset = side, self.frame

    nav = nav if nav is not None else NavInputs()
    v_ego = max(float(carstate.vEgo), 0.)
    self.odometer += v_ego * DT_MDL
    self.gap = 0. if nav.instruction is not None else self.gap + DT_MDL
    self.outer_lane_time = [t + DT_MDL if outer and not lane_changing else 0. for t, outer in zip(self.outer_lane_time, nav.outer_lanes, strict=True)]
    self.pins = {maneuver: point for maneuver, point in self.pins.items() if self.odometer <= point + NAV_TURN_HOLD}

    following = self.tracker.maneuver is not None
    passed_frame = self.passed_frame
    if self.tracker.update(nav.instruction, nav.age, v_ego, self.started):
      self.maneuver_id += 1
      self.started = self.called_off = self.auto_off = self.blinker_turned = self.slowing = False
      self.turning = self.route_blinker = None
      self.passed_frame, self.turned = None, 0.
      # a blinker on since before the car passed the maneuver before was for that one, as is any blinker on when that one
      # was never passed; one switched on after it is for this one
      self.stale_blinker = side if following and (passed_frame is None or self.blinker_onset <= passed_frame) else None
    if self.tracker.passed and self.passed_frame is None:
      self.passed_frame = self.frame
    if side != self.stale_blinker:
      self.stale_blinker = None
    turn = self.tracker.turn()
    pending = (self.turn_enabled or self.auto_enabled) and turn is not None and not self.tracker.passed and self.gap <= NAV_INSTRUCTION_GRACE

    # the heading change toward the turn's side as the car comes up to it, not at a stop, where it would drift
    if turn is not None and v_ego > NAV_TURN_TRIGGER_MIN_SPEED and self.tracker.distance <= NAV_TURN_LINEUP_DISTANCE:
      self.turned = max(self.turned + math.degrees(nav.yaw_rate if turn else -nav.yaw_rate) * DT_MDL, 0.)

    # a blinker switched on for the route's turn stays its signal until it goes off or over to the other side, steering
    # disengages or the maneuver changes, whatever the speed does; it brings back what the driver called off
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
        self.pins.pop(self.maneuver_id, None)
      self.auto_off |= self.called_off or not lateral_active

    if self.turning is not None:
      self._hold_turn(carstate, nav, lateral_active, v_ego)
    elif pending and lateral_active and nav.instruction is not None:
      self._start_turn(carstate, nav, v_ego, turn, lane_changing)

    self._update_speed_limit(carstate, pending, turn)

    if (self.exit_enabled and self.turning is None and nav.instruction is not None and self.tracker.maneuver is not None and lateral_active
        and self.tracker.maneuver[0] in SPLIT_TYPES and self.tracker.maneuver[1] in LEFT_DIRECTIONS + RIGHT_DIRECTIONS):
      self._update_keep(carstate, v_ego, self.tracker.distance, nav.instruction.lanes)

  def _time_to_steer(self, nav: NavInputs, distance: float, v_ego: float) -> float:
    return time_to_steer(distance, v_ego, nav.long_active and self.slowing)

  def _can_make_turn(self, carstate, nav: NavInputs, v_ego: float, left: bool, max_speed: float) -> bool:
    """Whether the car can still make the turn: at a crawl while lined up with the approach, or moving, before the steering
    point and slow enough to be down to turn speed by it, on the nearer of dead reckoning's and navd's distance. Never
    while navd puts the turn clearly farther: the car may have left the route, or the distance is off."""
    if self.tracker.residual > NAV_AGREE_DISTANCE:
      return False
    if carstate.standstill or v_ego <= NAV_TURN_CRAWL_SPEED:
      return self.lined_up
    distance = self.tracker.nearest
    if distance <= NAV_TURN_STEER_DISTANCE:
      return False
    if nav.long_active:
      braking = max(distance - NAV_TURN_STEER_DISTANCE - NAV_TURN_LAG * v_ego, 0.)
      max_speed = min(max_speed, math.sqrt(NAV_TURN_SPEED ** 2 + 2 * NAV_TURN_REACH_DECEL * braking) + NAV_TURN_REACH_MARGIN)
    else:
      max_speed = min(max_speed, NAV_TURN_STOCK_NEAR_SPEED if left != nav.right_hand_traffic else NAV_TURN_STOCK_FAR_SPEED)
    return v_ego <= max_speed

  def _start_turn(self, carstate, nav: NavInputs, v_ego: float, left: bool, lane_changing: bool) -> None:
    # never during a lane change, into the blind spot, or while another maneuver's turn is still under way
    if (lane_changing or (carstate.leftBlindspot if left else carstate.rightBlindspot)
        or any(maneuver != self.maneuver_id for maneuver in self.pins)):
      return
    if self._time_to_steer(nav, self.tracker.distance, v_ego) > NAV_TURN_TRIGGER_TIME:
      return

    if self.route_blinker == left and not self.blinker_turned and self._can_make_turn(carstate, nav, v_ego, left, NAV_TURN_MAX_SPEED):
      self.turn_by_blinker = self.blinker_turned = True
    elif (self.auto_enabled and not (self.started or self.called_off or self.auto_off or carstate.leftBlinker or carstate.rightBlinker)
          and self._auto_turn_allowed(nav, left) and self._can_make_turn(carstate, nav, v_ego, left, AUTO_TURN_MAX_SPEED)):
      self.turn_by_blinker = False
    else:
      return
    self.started, self.turning, self.pulse_frame = True, left, self.frame
    self._pin()
    self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  def _hold_turn(self, carstate, nav: NavInputs, lateral_active: bool, v_ego: float) -> None:
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

    # a fresh pulse only keeps the turn in the model's desire history: while the car waits at a crawl still lined up with
    # the approach, or while it is still far enough from the steering point to take it as a first pulse, never as it rolls
    # into the turn
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    if (self.frame - self.pulse_frame) * DT_MDL >= NAV_TURN_REPULSE_TIME and not blindspot and self.gap <= NAV_INSTRUCTION_GRACE:
      if carstate.standstill or v_ego <= NAV_TURN_CRAWL_SPEED:
        refresh = self.lined_up
      else:
        distance = self.tracker.nearest
        refresh = distance > NAV_TURN_STEER_DISTANCE and self._time_to_steer(nav, distance, v_ego) >= NAV_TURN_REFRESH_LEAD
      if refresh:
        self.pulse_frame = self.frame + 1  # no desire for this frame, so the next one is a new rising edge
        return
    self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  def _auto_turn_allowed(self, nav: NavInputs, left: bool) -> bool:
    # only a near side turn, from the outermost lane the model can see; a far side turn crosses oncoming traffic from a
    # lane the model cannot tell apart from the others, so it waits for the blinker
    return nav.long_active and left != nav.right_hand_traffic and self.outer_lane_time[0 if left else 1] >= OUTER_LANE_TIME - 1e-6

  def _pin(self) -> None:
    """Pin the current maneuver's point on the odometer, at the nearest it has been."""
    point = self.odometer + max(self.tracker.distance, 0.)
    self.pins[self.maneuver_id] = min(self.pins.get(self.maneuver_id, math.inf), point)

  def _update_speed_limit(self, carstate, pending: bool, turn: bool | None) -> None:
    # the route turn's slowdown: a started turn's until the car passes it, whatever the route gap, else every turn's with
    # automatic turns on, or the one the driver signals for, while nothing called it off. A lane change with the turn's
    # blinker goes toward the turn, and the other blinker calls it off, so it keeps going through a lane change.
    no_blinker = not (carstate.leftBlinker or carstate.rightBlinker)
    through = self.started and turn is not None and not self.tracker.passed
    self.slowing = through or (pending and not self.called_off and ((self.auto_enabled and no_blinker) or self.route_blinker == turn))
    if self.slowing and self.tracker.distance <= NAV_TURN_MARGIN:
      self._pin()
    # every pinned turn point keeps its profile, and turn speed to NAV_TURN_HOLD past it, even once navd moved on
    limits = [turn_speed_limit(point - self.odometer) for point in self.pins.values()]
    if self.slowing:
      limits.append(turn_speed_limit(self.tracker.distance))
    limit = min(limits, default=math.inf)
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
