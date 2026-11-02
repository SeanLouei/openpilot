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
# car, still lined up with the approach, waits at a crawl short of the turn, or is at least NAV_TURN_REFRESH_LEAD from the
# steering point.
NAV_TURN_TRIGGER_TIME = 3.0  # s
NAV_TURN_STEER_DISTANCE = 8.0  # m before the maneuver point where steering into an intersection turn starts
NAV_TURN_TRIGGER_MIN_SPEED = 0.5  # m/s; a car stopped short of the turn waits, one stopped at it fires
NAV_TURN_REPULSE_TIME = 4.0  # s
NAV_TURN_REFRESH_LEAD = 2.0  # s a moving car is still from the steering point when its turn desire is refreshed, at least
NAV_TURN_MAX_SPEED = 9.0  # m/s (20 mph); openpilot's lateral acceleration cap leaves faster turns running wide
AUTO_TURN_MAX_SPEED = 7.0  # m/s; an automatic turn, with no blinker to confirm it, also waits for turning speed

# A turn starts only while the car can still make it, else the car goes straight: lined up with the approach, and moving,
# far enough before the steering point to steer from it and slow enough to be down to turn speed by it, or at a crawl
NAV_TURN_CRAWL_SPEED = 1.5  # m/s
NAV_TURN_REACH_DECEL = 1.2  # m/s^2, below plannerd's NAV_TURN_MAX_DECEL for its jerk ramp and the actuator lag
NAV_TURN_LAG = 0.3  # s before the braking takes hold, or the steering
NAV_TURN_REACH_MARGIN = 0.5  # m/s
# Above turn speed, a car openpilot still speeds up (as it pulls away from a light, or after the gas) gets no start until
# that has stopped: plannerd's slowdown eases an acceleration out at its jerk limit, and the speed the car gains meanwhile
# is not in the reach speed
NAV_TURN_START_ACCEL = 0.3  # m/s^2
# At a crawl, a near side turn (right, where traffic keeps right) starts only short of the intersection: from closer to the
# maneuver point even the tightest turn this car steers (about 5.5 m about the rear axle, which is some 1.7 m behind the
# comma) ends in the cross street's oncoming half. A far side turn waiting in the intersection still has room.
NAV_TURN_NEAR_CRAWL_DISTANCE = 5.  # m
# Without openpilot's speed control nothing slows the car for the turn (Tesla's own cruise cannot go below 18 mph), so it
# starts only at turning speed, and never while the car's own cruise is set faster: only a slower car ahead holds it
# down, and once that one goes straight the cruise speeds back up toward its set speed, through the turn
NAV_TURN_STOCK_NEAR_SPEED = 5.5  # m/s for a near side turn (right, where traffic keeps right), the tight one
NAV_TURN_STOCK_FAR_SPEED = 6.7  # m/s for a far side turn
# How far the car has turned toward the turn's side since it came this close to it: past NAV_TURN_LINED_UP it is in the
# turn, so it gets no new turn pulse there. The heading counts whenever the car moves, inching included; only not at a
# stop, where the model's yaw rate could only drift.
NAV_TURN_LINEUP_DISTANCE = 40.  # m
NAV_TURN_LINED_UP = 25.  # deg
NAV_TURN_LINEUP_MIN_SPEED = 0.05  # m/s

# An automatic turn starts only from the outermost lane: a confident road edge close beside the car with no lane line
# beyond the car's own, for a second, counted again after a lane change. Only the edge's distance tells a lane beside the
# car from the curb: a narrow painted bike lane may not put it beyond OUTER_EDGE_DISTANCE, and one behind a curb or posts
# looks like the edge itself.
OUTER_EDGE_DISTANCE = 2.6  # m from the car's center
OUTER_LANE_TIME = 1.0  # s

# A far side turn (left where traffic keeps right) crosses oncoming traffic, so it starts without the blinker only where the
# car can confirm it is in a lane for that turn: a curb or median close beside it on the turn's side, with no lane line
# beyond the car's own, surer than for a near side turn and over the last FAR_LANE_DISTANCE and FAR_LANE_TIME with no lane
# change and no car in that blind spot, and the route's lane guidance not against that lane. On a two-way road the center
# line has oncoming lanes beyond it, not a curb, so there it waits for the blinker. A car one lane over from the curb lane
# has the curb a lane's width (2.7 m or more) beyond its own lane line, about 3.8 m or more from its center; the curb lane's
# curb is 1.5-2.4 m from it.
FAR_EDGE_MIN = 1.0  # m from the car's center; nearer, the edge would be under the car, a bad reading
FAR_EDGE_DISTANCE = 2.6  # m from the car's center to the curb or median, at most
FAR_EDGE_STD = 0.5  # m the model may be unsure of the edge's position; the near side's EDGE_PROB allows 0.65
FAR_LANE_TIME = 1.5  # s of curb beside the car, so a brief misreading of the center line never counts
FAR_LANE_DISTANCE = 10.  # m driven with the curb beside the car: slow, 1.5 s could be just a corner's or an island's nose
# With no blinker, the driver first learns of a far side turn from its alert: it starts by itself only this long or more
# before the car steers across oncoming traffic, else it waits for the blinker
FAR_TURN_MIN_LEAD = 2.0  # s
# Nor is it kept alive while the car waits: a stop close to the turn (a light, a stop sign, a queue, a gap in oncoming
# traffic) is where only the driver can tell when to cross, so once the model's desire history no longer holds the turn,
# the driver signals to go on with it
FAR_STOP_RELEASE = 5.0  # s since the last pulse, the model's desire history
# Faster oncoming traffic leaves no time to react, and such roads have protected arrows the car cannot see: no automatic far
# side turn where the posted limit over the last AUTO_FAR_LIMIT_DISTANCE is unknown or above 45 mph (72 km/h passes,
# 50 mph and 75 km/h do not)
AUTO_FAR_ROAD_SPEED = 20.5  # m/s
AUTO_FAR_LIMIT_DISTANCE = 150.  # m before the maneuver point
POSTED_LIMIT_MAX_AGE = 0.5  # s plannerd's posted limit counts for; older, it is unknown
# A far side turn that will not start by itself asks for the blinker this long before its start would come, so a blinker
# given then still starts it on time, once its automatic start has been out of reach for FAR_PROMPT_DEBOUNCE: a reading
# that comes and goes asks once, not on and off
FAR_SIGNAL_LEAD = 2.0  # s
FAR_PROMPT_DEBOUNCE = 0.5  # s
# The Tesla switches the blinker off by itself as a lane change ends; a latched blinker going off later than this after one
# is the driver calling the turn off
FAR_CANCEL_TIME = 2.0  # s

# A blinker switched on for the route's next turn within this distance is its turn signal, not a lane change, for as long
# as it stays on; on a fast road from farther, from where the slowdown starts, so it has the room to get down to turn
# speed. Only within this distance does it bring back what the driver called off: farther out it may yet be a lane change.
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
# turn starts only while navd agrees, and judged on the nearer of the two distances, the dead reckoned one without what
# navd pushed out beyond that take-back here, since the push-out alone would make navd agree in the end
NAV_NEAR_DISTANCE = 60.0  # m
NAV_PULL_IN_MEMORY = 20.0  # s over which a pull-in fades
NAV_AGREE_DISTANCE = 8.0  # m navd's distance may be beyond the dead reckoned one for a turn to start
# A car standing still cannot change its distance to the maneuver, so navd's changes then are only GPS noise
NAV_STANDSTILL_SPEED = 0.1  # m/s

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
STRAIGHT_ON_TYPES = ('new name', 'continue', 'turn')  # with a 'straight' modifier: no blinker is the signal for them


@dataclass
class NavInputs:
  instruction: Any = None  # the latest navInstruction, None without a valid, recent one
  age: float = 0.  # s since navd published it
  long_active: bool = False  # openpilot longitudinal controls the speed and the driver isn't pressing the gas
  outer_lanes: tuple[bool, bool] = (False, False)  # whether the car is in the outermost lane on its (left, right)
  right_hand_traffic: bool = True
  yaw_rate: float = 0.  # rad/s the car turns at, positive left
  accel: float = 0.  # m/s^2 the car speeds up at: the more of its measured and its commanded acceleration
  far_lanes: tuple[bool, bool] = (False, False)  # whether a curb or median is surely close beside the car on its (left, right)
  driver_alert: bool = False  # driver monitoring is warning
  speed_limit: float = 0.  # m/s the posted limit plannerd found, 0 unknown


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


def far_lanes(model) -> tuple[bool, bool]:
  """Whether the model surely sees a curb or median close beside the car on each side: on that side of the car and not
  under it, with no lane line beyond the car's own. outer_lanes, with a surer edge whose side is checked."""
  stds, probs, edges = model.roadEdgeStds, model.laneLineProbs, model.roadEdges
  if len(stds) < 2 or len(probs) < 4 or len(edges) < 2 or not len(edges[0].y) or not len(edges[1].y):
    return False, False

  def curb(side: int, outer_line: int, sign: float) -> bool:
    distance, std = sign * float(edges[side].y[0]), float(stds[side])  # NaN fails every comparison
    return bool(FAR_EDGE_MIN < distance < FAR_EDGE_DISTANCE and 0. <= std < FAR_EDGE_STD and probs[outer_line] < NEARSIDE_PROB)

  return curb(0, 0, -1.), curb(1, 3, 1.)


def far_turn_lane(lanes, left: bool) -> bool:
  """Whether the route's lane guidance lets the outermost lane on the turn's side, the one beside the curb or median, make
  the turn: active, toward the turn's side. Without lane guidance it does. Only a veto: guidance tells which lanes may
  turn, never which one the car is in."""
  if not len(lanes):
    return True
  lane = lanes[0] if left else lanes[len(lanes) - 1]  # Mapbox lists the lanes left to right
  side = LEFT_DIRECTIONS if left else RIGHT_DIRECTIONS
  direction = str(lane.activeDirection)
  if direction == 'none':  # navd also reads a U-turn as none, so a U-turn only lane vetoes
    return bool(lane.active) and any(str(d) in side for d in lane.directions)
  return bool(lane.active) and direction in side


def yaw_rate(model) -> float:
  """The model's yaw rate now, positive left (the model's frame is forward, right, down), 0 without one."""
  rates = getattr(getattr(model, 'orientationRate', None), 'z', ())
  rate = float(rates[0]) if len(rates) else 0.
  return -rate if math.isfinite(rate) else 0.


def acceleration(sm) -> float:
  """The more of the car's measured acceleration and the one controlsd commands, which leads it; 0 without either."""
  accels = (float(sm['carState'].aEgo), float(sm['carControl'].actuators.accel))
  return max((accel for accel in accels if math.isfinite(accel)), default=0.)


def posted_speed_limit(sm) -> float:
  """The posted speed limit plannerd's resolver found, from the car or the map, in m/s; 0 without a recent, valid one. It
  never raises, also when the SubMaster lacks the service: modeld calls it outside DesireHelper's guard."""
  try:
    if not sm.seen['longitudinalPlanSP'] or not sm.valid['longitudinalPlanSP']:
      return 0.
    if not 0. <= time.monotonic() - sm.recv_time['longitudinalPlanSP'] <= POSTED_LIMIT_MAX_AGE:
      return 0.
    resolver = sm['longitudinalPlanSP'].speedLimit.resolver
    limit, valid = float(resolver.speedLimit), bool(resolver.speedLimitValid)
  except Exception:
    return 0.
  return limit if valid and math.isfinite(limit) and limit > 0. else 0.


def nav_inputs(sm, model, openpilot_longitudinal: bool) -> NavInputs:
  """Everything route desires need from modeld: its SubMaster, the model message just filled for this frame, and whether
  openpilot controls the car's speed, which carControl.longActive alone also reports for cruise button control."""
  instruction, age = fresh_nav_instruction(sm)
  long_active = openpilot_longitudinal and bool(sm['carControl'].longActive)
  dm = sm['driverMonitoringState']
  return NavInputs(instruction, age, long_active, outer_lanes(model), not dm.isRHD, yaw_rate(model), acceleration(sm),
                   far_lanes=far_lanes(model), driver_alert=dm.alertLevel != log.DriverMonitoringState.AlertLevel.none,
                   speed_limit=posted_speed_limit(sm))


def signal_distance(v_ego: float) -> float:
  return float(np.clip(NAV_TURN_SIGNAL_BASE_DISTANCE + max(v_ego, 0.) * NAV_TURN_SIGNAL_TIME, *NAV_TURN_SIGNAL_DISTANCE))


def turn_speed_limit(distance: float) -> float:
  """The highest speed this far before a turn from which the car slows to turn speed at the profile's deceleration."""
  return math.sqrt(NAV_TURN_SPEED ** 2 + 2 * NAV_TURN_DECEL * max(distance - NAV_TURN_MARGIN, 0.))


def approach_distance(v_ego: float) -> float:
  """How far before a route turn the car approaches it, where the turn's blinker latches and the driver calls it off: from
  where the car would signal for it, or start slowing for it."""
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


def signal_sides(maneuver: tuple[str, str]) -> tuple[bool, ...]:
  """The blinker sides (left) that may have been the signal for a maneuver: none for one that goes straight on, either
  for a U-turn, a roundabout, or one without a direction."""
  kind, direction = maneuver
  if direction in LEFT_DIRECTIONS:
    return (True,)
  if direction in RIGHT_DIRECTIONS:
    return (False,)
  if direction == 'straight' and kind in STRAIGHT_ON_TYPES:
    return ()
  return (True, False)


class ManeuverTracker:
  """The route's next maneuver and the distance to it, dead reckoned between navd's updates. A maneuver is its type,
  direction and distance to the destination. Each navd message leaves a residual, its distance less the dead reckoned
  one, and the median of the last few corrects the distance: in at a limited rate, out slowly (a speed or map scale
  error), or out at once when navd's is clearly longer (the car left the route or joined it from beside it, or the GPS
  lagged), but close to the maneuver only by what navd lately pulled in. So one bad fix does little and a frozen navd
  distance holds the approach back. Once the maneuver's turn started or the car passed its point the distance only comes
  in, so nothing restarts or moves a started turn, and a passed maneuver stays passed until navd reports a different one.
  A car standing still takes none of navd's distances in: its own cannot change, so they are only GPS noise."""

  def __init__(self):
    self.maneuver: tuple[str, str] | None = None  # (type, direction)
    self.position = math.nan  # m from the maneuver to the destination along the route
    self.distance = math.inf
    self.passed = False
    self.instruction = None  # navd's last message
    self.residuals: deque[float] = deque(maxlen=NAV_RESIDUALS)  # m navd's distance is beyond the dead reckoned one, left to correct
    self.residual = 0.  # their median
    self.pulled_in = 0.  # m navd pulled the distance in lately, fading over NAV_PULL_IN_MEMORY
    self.pushed_near = 0.  # m navd pushed the distance out within NAV_NEAR_DISTANCE beyond taking back pull-ins, less pull-ins since

  @property
  def nearest(self) -> float:
    """The nearer of navd's distance and the dead reckoned one without what navd pushed out close to the maneuver: there a
    longer navd distance may be a GPS error behind the car, which would time a turn late."""
    return self.distance - max(self.pushed_near, -min(self.residual, 0.))

  def update(self, instruction, age: float, v_ego: float, committed: bool = False) -> bool:
    """Advance one frame, along the last maneuver while there is no instruction. committed: the maneuver's turn started.
    True when a different maneuver began."""
    v_ego = max(v_ego, 0.)
    self.distance -= v_ego * DT_MDL
    still = v_ego < NAV_STANDSTILL_SPEED
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
          self.maneuver, self.position, self.distance, self.passed = maneuver, position, reported, False
          self.residual = self.pulled_in = self.pushed_near = 0.
          self.residuals.extend([0.] * NAV_RESIDUALS)
        elif not still:
          self.residuals.append(reported - self.distance)
    if still:
      # nothing moves the distance, and a pull-in taken just before the stop can still be taken back once the car moves
      self.passed |= self.distance < 0.
      return new

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
    if step < 0.:
      self.pushed_near = max(self.pushed_near + step, 0.)
    elif self.distance <= NAV_NEAR_DISTANCE:
      self.pushed_near += max(step - self.pulled_in, 0.)
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

  Turns: switching the blinker on for the route's next turn, from where it would signal for it or start slowing for it,
  makes it that turn's signal until it goes off: it never starts a nudgeless lane change, and it holds back the lane turn
  desire. A blinker already on before the car passed the last maneuver to its side, or the one before, is not its signal.
  About 3 s before the car starts steering into the turn, timed on the speed the slowdown will impose, the turn desire
  starts, but only while the car can still make the turn: still lined up with the approach, and either moving, far enough
  before the steering point to steer from it and slow enough to be down to turn speed by it (below 20 mph, and with
  openpilot's speed control not still speeding up above turn speed; without it, which nothing then slows, at turning speed
  already and with the car's own cruise off or set no faster), or at a crawl short of the maneuver point (a near side turn
  short of the intersection). A car that cannot goes straight. With automatic turns on and openpilot in control of the
  speed, a near side turn (right, where traffic keeps right) also starts without the blinker, once the car has been in the
  outermost lane for a second. With automatic far side turns on too, so does a far side turn, across oncoming traffic, but
  only from a lane the car can confirm: moving, 2 s or more before the steering point, with a curb or median surely beside
  the car on the turn's side for the last 10 m and 1.5 s, the route's lane guidance not against that lane, no sharp turn
  or T junction, the posted limit known and 45 mph or less, and driver monitoring not warning. A stop close before it, or
  the driver switching its blinker off, leaves it to the blinker; one that will not start by itself asks for the blinker,
  once, and then waits for it. Before the turn starts, the gas, the other blinker or disengaging calls off its automatic
  turn and its slowdown; signaling for it brings them back, within the signal distance. Once it starts, the model has the
  turn: the desire holds until the maneuver point, pulsing again every 4 s only to stay in the model's desire history
  (while the car is still lined up, and waits at a crawl short of the turn, or is still 2 s or more from the steering
  point), a blinker latched again pulses it again only the same way, and no other turn starts until the car is through it.
  An automatic far side turn pulses again only while the car moves and driver monitoring is quiet, and at a stop it is
  the driver's once the model's desire history no longer holds it.
  Slowing: the speed limit slows the car to turn speed by every route turn with automatic turns on, else by the turn the
  driver signals for, through a lane change too, and by a started turn until the car passes it. The turn's point is
  pinned on the odometer once it starts or the car is close: the slowdown keeps the nearest the point has been, and turn
  speed holds to 20 m past the farthest, even once navd moves on. plannerd follows it while openpilot controls the speed.
  Exits and forks: a nudge toward the side the route's lane guidance takes gives a keep desire near the split.
  """

  def __init__(self):
    self.params = Params()
    self.param_read_counter = 0
    self.turn_enabled = False
    self.auto_enabled = False
    self.far_enabled = False
    self.exit_enabled = False
    self.read_params()

    self.tracker = ManeuverTracker()
    self.frame = 0
    self.odometer = 0.  # m driven
    self.gap = math.inf  # s since the last fresh route instruction
    self.outer_lane_time = [0., 0.]  # s the model has seen the car in the outermost lane on its (left, right)
    # s and m a curb or median has surely been close beside the car on its (left, right), with no lane change and no car in
    # that blind spot
    self.far_lane_time = [0., 0.]
    self.far_lane_distance = [0., 0.]
    self.road_limit = 0.  # m/s the highest posted limit within AUTO_FAR_LIMIT_DISTANCE of the current maneuver, 0 unknown
    self.was_lane_changing = False
    self.lane_change_end: int | None = None  # the frame the last lane change ended
    self.blinker: bool | None = None  # the side (left) of the blinker on, and the frame it came on
    self.blinker_onset = 0
    self.passed_frame: int | None = None  # the frame the car passed the current maneuver's point
    self.maneuver_id = 0
    self.pins: dict[int, tuple[float, float]] = {}  # the odometer readings of each maneuver's turn point, (nearest, farthest)
    self.turned = 0.  # deg the car turned toward the current turn's side since it came close to it
    # the maneuver whose turn desire last went out, and the frame it did: the model has that turn, whatever reset() forgets
    self.pulsed_maneuver: int | None = None
    self.pulse_frame = 0
    # the maneuver whose far side turn asked for the blinker: it waits for it, whatever reset() forgets
    self.asked_maneuver: int | None = None
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
    self.reviving = False  # the latched blinker brings back what the driver called off, once within the signal distance
    self.stale_blinker: bool | None = None  # the side (left) of a blinker on since before the car passed the last maneuver
    self.blinker_turned = False  # the latched blinker already started its turn
    self.turning: bool | None = None  # the side (left) of the turn desire being held
    self.turn_by_blinker = False
    self.started = False  # the current maneuver's turn desire went out to the model
    self.called_off = False  # the current maneuver's automatic turn and slowdown are off, until the driver signals for it
    self.auto_off = False  # the current maneuver's automatic turn is off, until the driver signals for it
    self.signal_prompt = False  # a far side turn that will not start, or go on, by itself: signal for it
    self.turn_across = False  # an automatic far side turn holds its desire
    self.prompt_time = 0.  # s the current far side turn's automatic start has been out of reach, in its prompt's window

  def read_params(self) -> None:
    self.turn_enabled = self.params.get_bool("NavTurnDesire")
    self.auto_enabled = self.params.get_bool("NavAutoTurn")
    self.far_enabled = self.auto_enabled and self.params.get_bool("NavAutoTurnFarSide")
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

  @property
  def asked(self) -> bool:
    """The current maneuver's far side turn asked for the blinker, so it never starts by itself."""
    return self.asked_maneuver == self.maneuver_id

  @property
  def route_active(self) -> bool:
    """Route turns are on and navd has a route: its instruction is fresh, or was within the grace."""
    return (self.turn_enabled or self.auto_enabled) and self.gap <= NAV_INSTRUCTION_GRACE

  def update(self, carstate, lateral_active: bool, nav: NavInputs | None, lane_changing: bool = False, lane_turning: bool = False) -> None:
    """lane_changing: a lane change is under way, so no turn starts and the outermost lane, and the curb beside the car,
    count again after it.
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
    # a car in the blind spot beside the curb means a lane there after all, so the curb counts again once it is gone
    blindspots = (carstate.leftBlindspot, carstate.rightBlindspot)
    curbs = [far and not lane_changing and not blindspot for far, blindspot in zip(nav.far_lanes, blindspots, strict=True)]
    self.far_lane_time = [t + DT_MDL if curb else 0. for t, curb in zip(self.far_lane_time, curbs, strict=True)]
    self.far_lane_distance = [d + v_ego * DT_MDL if curb else 0. for d, curb in zip(self.far_lane_distance, curbs, strict=True)]
    if self.was_lane_changing and not lane_changing:
      self.lane_change_end = self.frame
    self.was_lane_changing = lane_changing
    self.pins = {maneuver: pin for maneuver, pin in self.pins.items() if self.odometer <= pin[1] + NAV_TURN_HOLD}

    previous = self.tracker.maneuver
    passed_frame = self.passed_frame
    if self.tracker.update(nav.instruction, nav.age, v_ego, self.started):
      self.maneuver_id += 1
      self.started = self.called_off = self.auto_off = self.blinker_turned = self.slowing = False
      self.turning = self.route_blinker = None
      self.passed_frame, self.turned, self.road_limit, self.prompt_time = None, 0., 0., 0.
      # a blinker on since before the car passed the maneuver before was for that one, if that one went its way; as is any
      # blinker on when that one was never passed, or still on from one before it. One switched on after it is for this
      # one, as is one switched on before a maneuver that went straight on, or the other way.
      if previous is not None:
        was_stale = side is not None and side == self.stale_blinker
        leftover = passed_frame is None or was_stale or (self.blinker_onset <= passed_frame and side in signal_sides(previous))
        self.stale_blinker = side if leftover else None
      else:
        self.stale_blinker = None
    if self.tracker.passed and self.passed_frame is None:
      self.passed_frame = self.frame
    if self.tracker.distance <= AUTO_FAR_LIMIT_DISTANCE:
      self.road_limit = max(self.road_limit, nav.speed_limit)
    if side != self.stale_blinker:
      self.stale_blinker = None
    turn = self.tracker.turn()
    pending = (self.turn_enabled or self.auto_enabled) and turn is not None and not self.tracker.passed and self.gap <= NAV_INSTRUCTION_GRACE

    # the heading change toward the turn's side as the car comes up to it, whenever the car moves
    if turn is not None and v_ego > NAV_TURN_LINEUP_MIN_SPEED and self.tracker.distance <= NAV_TURN_LINEUP_DISTANCE:
      self.turned = max(self.turned + math.degrees(nav.yaw_rate if turn else -nav.yaw_rate) * DT_MDL, 0.)

    # a blinker switched on for the route's turn stays its signal until it goes off or over to the other side, steering
    # disengages or the maneuver changes, whatever the speed does. It brings back what the driver called off before it
    # latched, once the car is within the signal distance with it still on. The driver switching it off before a far side
    # turn starts also calls off that turn's automatic start, which only the blinker brings back; the Tesla switching it
    # off by itself as a lane change ends, into a turn bay say, is not the driver.
    car_cancel = self.lane_change_end is not None and (self.frame - self.lane_change_end) * DT_MDL <= FAR_CANCEL_TIME
    blinker_off = self.route_blinker is not None and side is None and not lane_changing and not car_cancel
    if side is None or side != self.route_blinker or not lateral_active:
      self.route_blinker, self.reviving = None, False
    if blinker_off and turn == nav.right_hand_traffic:
      self.auto_off = True
    if (pending and lateral_active and self.route_blinker is None and side == turn and side != self.stale_blinker
        and nav.instruction is not None and not lane_turning and self.tracker.distance <= approach_distance(v_ego)):
      self.route_blinker, self.blinker_turned, self.reviving = side, False, True
    if self.reviving and self.tracker.distance <= signal_distance(v_ego):
      self.called_off = self.auto_off = self.reviving = False
    self.route_turn_signal = self.route_blinker is not None

    # before the turn starts, the driver calls off its automatic turn, and its slowdown with the gas, the other blinker or
    # disengaging the speed control. A stop close before a far side turn (a light, a stop sign, a queue, a gap in oncoming
    # traffic) leaves its automatic turn to the driver, who alone can tell when to cross: the car keeps slowing for it, and
    # turns once signaled. A far side turn that asked for the blinker waits for it.
    stopped = bool(carstate.standstill) or v_ego <= NAV_TURN_CRAWL_SPEED
    if pending and not self.started and self.tracker.distance <= approach_distance(v_ego):
      other_blinker = side is not None and side != turn and side != self.stale_blinker
      if carstate.gasPressed or other_blinker or (self.auto_enabled and not nav.long_active):
        self.called_off, self.reviving = True, False
        self.pins.pop(self.maneuver_id, None)
      self.auto_off |= self.called_off or not lateral_active or (turn == nav.right_hand_traffic and stopped)
    self.auto_off |= self.asked

    if self.turning is not None:
      self._hold_turn(carstate, nav, lateral_active, v_ego)
    elif pending and lateral_active and nav.instruction is not None:
      self._start_turn(carstate, nav, v_ego, turn, lane_changing)

    # One prompt to signal, for a far side turn that will not start or go on by itself: while the car waits short of it,
    # still lined up, or while it moves, from FAR_SIGNAL_LEAD before its start would come, once its automatic start has been
    # out of reach for FAR_PROMPT_DEBOUNCE. It asks only while signaling can still start the turn, and once it asked, the
    # turn waits for the blinker: a prompt is never followed by an automatic start.
    asking = bool(self.far_enabled and pending and turn == nav.right_hand_traffic and self.turning is None and side is None
                  and lateral_active and nav.long_active and not self.called_off and not lane_changing and nav.instruction is not None)
    prompt = False
    if not asking:
      self.prompt_time = 0.
    elif stopped:
      prompt = self.lined_up and self.tracker.distance <= approach_distance(v_ego)
    elif not self.started:
      window = (self._time_to_steer(nav, self.tracker.distance, v_ego) <= NAV_TURN_TRIGGER_TIME + FAR_SIGNAL_LEAD
                and self._can_make_turn(carstate, nav, v_ego, turn, NAV_TURN_MAX_SPEED))
      out_of_reach = window and not self._far_auto_possible(carstate, nav, v_ego, turn)
      self.prompt_time = self.prompt_time + DT_MDL if out_of_reach else 0.
      prompt = window and (self.asked or self.prompt_time >= FAR_PROMPT_DEBOUNCE - 1e-6)
    if prompt:
      self.asked_maneuver = self.maneuver_id
    self.signal_prompt = prompt
    self.turn_across = self.turning is not None and not self.turn_by_blinker and self.turning == nav.right_hand_traffic

    self._update_speed_limit(carstate, pending, turn)

    if (self.exit_enabled and self.turning is None and nav.instruction is not None and self.tracker.maneuver is not None and lateral_active
        and self.tracker.maneuver[0] in SPLIT_TYPES and self.tracker.maneuver[1] in LEFT_DIRECTIONS + RIGHT_DIRECTIONS):
      self._update_keep(carstate, v_ego, self.tracker.distance, nav.instruction.lanes)

  def _time_to_steer(self, nav: NavInputs, distance: float, v_ego: float) -> float:
    return time_to_steer(distance, v_ego, nav.long_active and self.slowing)

  def _crawl_room(self, nav: NavInputs, left: bool) -> float:
    """m before the maneuver point a car at a crawl needs to start the turn from: a near side turn needs to start short of
    the intersection, a far side one only short of the point."""
    return NAV_TURN_NEAR_CRAWL_DISTANCE if left != nav.right_hand_traffic else 0.

  def _can_make_turn(self, carstate, nav: NavInputs, v_ego: float, left: bool, max_speed: float) -> bool:
    """Whether the car can still make the turn: while lined up with the approach, at a crawl short of the maneuver point
    (a near side turn short of the intersection), or moving, far enough before the steering point to steer from it and
    slow enough to be down to turn speed by it, on the nearer of dead reckoning's and navd's distance. Never while navd
    puts the turn clearly farther: the car may have left the route, or the distance is off. Nor, without openpilot's
    speed control, while the car's own cruise is set faster than turning speed."""
    stock_speed = NAV_TURN_STOCK_NEAR_SPEED if left != nav.right_hand_traffic else NAV_TURN_STOCK_FAR_SPEED
    if self.tracker.residual > NAV_AGREE_DISTANCE or not self.lined_up:
      return False
    if not nav.long_active and carstate.cruiseState.enabled and carstate.cruiseState.speed > stock_speed:
      return False
    distance = self.tracker.nearest
    if carstate.standstill or v_ego <= NAV_TURN_CRAWL_SPEED:
      return distance > self._crawl_room(nav, left)
    if distance <= NAV_TURN_STEER_DISTANCE + NAV_TURN_LAG * v_ego:  # the desire takes that long to steer the car
      return False
    if nav.long_active:
      if v_ego > NAV_TURN_SPEED + NAV_TURN_REACH_MARGIN and nav.accel > NAV_TURN_START_ACCEL:
        return False
      braking = distance - NAV_TURN_STEER_DISTANCE - NAV_TURN_LAG * v_ego
      max_speed = min(max_speed, math.sqrt(NAV_TURN_SPEED ** 2 + 2 * NAV_TURN_REACH_DECEL * braking) + NAV_TURN_REACH_MARGIN)
    else:
      max_speed = min(max_speed, stock_speed)
    return v_ego <= max_speed

  def _may_pulse_again(self, carstate, nav: NavInputs, v_ego: float, left: bool, by_blinker: bool) -> bool:
    """Whether a started turn may pulse again, only to keep it in the model's desire history: NAV_TURN_REPULSE_TIME after
    its last pulse, while the car is still lined up with the approach and waits at a crawl short of the maneuver point (a
    near side turn short of the intersection), or is still far enough from the steering point to take it as a first
    pulse. Never as it rolls into the turn, nor once it has turned in. by_blinker: the blinker has the turn, else it is
    an automatic one, and an automatic far side turn pulses again only while the car moves and driver monitoring is quiet:
    at a stop only the driver can tell when to cross, and a driver monitoring warning means nobody may be watching."""
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    stopped = carstate.standstill or v_ego <= NAV_TURN_CRAWL_SPEED
    if ((self.frame - self.pulse_frame) * DT_MDL < NAV_TURN_REPULSE_TIME or blindspot or self.gap > NAV_INSTRUCTION_GRACE
        or not self.lined_up or (self._far_automatic(nav, left, by_blinker) and (stopped or nav.driver_alert))):
      return False
    distance = self.tracker.nearest
    if stopped:
      return distance > self._crawl_room(nav, left)
    return distance > NAV_TURN_STEER_DISTANCE and self._time_to_steer(nav, distance, v_ego) >= NAV_TURN_REFRESH_LEAD

  def _far_automatic(self, nav: NavInputs, left: bool, by_blinker: bool) -> bool:
    """A turn across oncoming traffic that no blinker confirms."""
    return not by_blinker and left == nav.right_hand_traffic

  def _far_stop_released(self, carstate, nav: NavInputs, v_ego: float, left: bool, by_blinker: bool) -> bool:
    """An automatic far side turn the car waits at, once the model's desire history no longer holds it: the driver's, then,
    to signal for it."""
    stopped = carstate.standstill or v_ego <= NAV_TURN_CRAWL_SPEED
    return self._far_automatic(nav, left, by_blinker) and stopped and (self.frame - self.pulse_frame) * DT_MDL >= FAR_STOP_RELEASE

  def _start_turn(self, carstate, nav: NavInputs, v_ego: float, left: bool, lane_changing: bool) -> None:
    # never during a lane change, into the blind spot, or while another maneuver's turn is still under way
    if (lane_changing or (carstate.leftBlindspot if left else carstate.rightBlindspot)
        or any(maneuver != self.maneuver_id for maneuver in self.pins)):
      return
    if self._time_to_steer(nav, self.tracker.distance, v_ego) > NAV_TURN_TRIGGER_TIME:
      return
    # the model already has this turn: a blinker latched again, or a start after reset(), only pulses it again as a
    # keep-alive would, the blinker's if it has the turn, else an automatic one's. Else the latch stays, and with it the
    # slowdown, but the desire does not resume: that would be a new rising edge, a second turn command.
    if self.pulsed_maneuver == self.maneuver_id and not self._may_pulse_again(carstate, nav, v_ego, left, self.route_blinker == left):
      return

    if self.route_blinker == left and not self.blinker_turned and self._can_make_turn(carstate, nav, v_ego, left, NAV_TURN_MAX_SPEED):
      self.turn_by_blinker = self.blinker_turned = True
    elif (self.auto_enabled and not (self.started or self.called_off or self.auto_off or carstate.leftBlinker or carstate.rightBlinker)
          and self._auto_turn_allowed(carstate, nav, v_ego, left) and self._can_make_turn(carstate, nav, v_ego, left, AUTO_TURN_MAX_SPEED)):
      self.turn_by_blinker = False
    else:
      return
    self.started, self.turning, self.pulsed_maneuver, self.pulse_frame = True, left, self.maneuver_id, self.frame
    self._pin()
    self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  def _hold_turn(self, carstate, nav: NavInputs, lateral_active: bool, v_ego: float) -> None:
    left = self.turning
    if self.route_blinker == left:
      self.turn_by_blinker = self.blinker_turned = True  # signaling for an automatic turn makes it the blinker's
    if self.turn_by_blinker:
      released = self.route_blinker != left  # the blinker went off, or over to the other side
    else:
      released = (carstate.gasPressed or carstate.leftBlinker or carstate.rightBlinker or not nav.long_active
                  or self._far_stop_released(carstate, nav, v_ego, left, False))
    # the model keeps its pulse, so letting go only ends the turn desire's reminders, and the alert
    if released or not lateral_active or self.tracker.passed:
      self.turning = None
      return

    if self._may_pulse_again(carstate, nav, v_ego, left, self.turn_by_blinker):
      self.pulse_frame = self.frame + 1  # no desire for this frame, so the next one is a new rising edge
      return
    self.turn_direction = TurnDirection.turnLeft if left else TurnDirection.turnRight

  def _auto_turn_allowed(self, carstate, nav: NavInputs, v_ego: float, left: bool) -> bool:
    side = 0 if left else 1
    if left != nav.right_hand_traffic:  # a near side turn, from the outermost lane the model can see
      return nav.long_active and self.outer_lane_time[side] >= OUTER_LANE_TIME - 1e-6
    # a far side turn crosses oncoming traffic, so only from the lane beside a curb or median the model has surely seen
    # for FAR_LANE_DISTANCE and FAR_LANE_TIME, and only where everything else lets it
    return bool(self._far_gates(carstate, nav, v_ego, left)
                and self.far_lane_time[side] >= FAR_LANE_TIME - 1e-6 and self.far_lane_distance[side] >= FAR_LANE_DISTANCE - 1e-6)

  def _far_gates(self, carstate, nav: NavInputs, v_ego: float, left: bool) -> bool:
    """Everything an automatic far side turn needs but the curb's run: the setting and openpilot's speed control, a plain
    turn (not sharp, nor at the end of the road, a T junction that must yield both ways), the car moving and still
    FAR_TURN_MIN_LEAD or more from the steering point, lane guidance not against the outermost lane, driver monitoring
    quiet, and the posted limit known and no more than AUTO_FAR_ROAD_SPEED."""
    maneuver = self.tracker.maneuver
    return bool(self.far_enabled and nav.long_active and maneuver is not None and maneuver[0] != 'end of road'
                and maneuver[1] in ('left', 'right') and not carstate.standstill and v_ego > NAV_TURN_CRAWL_SPEED
                and self._time_to_steer(nav, self.tracker.nearest, v_ego) >= FAR_TURN_MIN_LEAD
                and nav.instruction is not None and far_turn_lane(nav.instruction.lanes, left)
                and not nav.driver_alert and 0. < self.road_limit <= AUTO_FAR_ROAD_SPEED)

  def _far_auto_possible(self, carstate, nav: NavInputs, v_ego: float, left: bool) -> bool:
    """Whether an automatic far side start can still come: nothing turned it off, the gates pass and the curb is beside the
    car, though its run may not be long enough yet."""
    blindspot = carstate.leftBlindspot if left else carstate.rightBlindspot
    return bool(not self.auto_off and not blindspot and nav.far_lanes[0 if left else 1] and self._far_gates(carstate, nav, v_ego, left))

  def _pin(self) -> None:
    """Pin the current maneuver's point on the odometer, at the nearest and the farthest it has been: a distance pulled in
    and taken back again brings the slowdown forward, but cannot end the turn speed early."""
    point = self.odometer + max(self.tracker.distance, 0.)
    nearest, farthest = self.pins.get(self.maneuver_id, (math.inf, -math.inf))
    self.pins[self.maneuver_id] = (min(nearest, point), max(farthest, point))

  def _update_speed_limit(self, carstate, pending: bool, turn: bool | None) -> None:
    # the route turn's slowdown: a started turn's until the car passes it, whatever the route gap, else every turn's with
    # automatic turns on, or the one the driver signals for, while nothing called it off. A lane change with the turn's
    # blinker goes toward the turn, and the other blinker calls it off, so it keeps going through a lane change.
    no_blinker = not (carstate.leftBlinker or carstate.rightBlinker)
    through = self.started and turn is not None and not self.tracker.passed
    self.slowing = through or (pending and not self.called_off and ((self.auto_enabled and no_blinker) or self.route_blinker == turn))
    if self.slowing and self.tracker.distance <= NAV_TURN_MARGIN:
      self._pin()
    # every pinned turn point keeps its profile from the nearest it has been, and turn speed to NAV_TURN_HOLD past the
    # farthest, even once navd moved on
    limits = [turn_speed_limit(nearest - self.odometer) for nearest, _ in self.pins.values()]
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
