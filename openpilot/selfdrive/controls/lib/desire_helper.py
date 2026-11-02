from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NavDesireController, blinker_side

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    self.nav_desire = NavDesireController()
    self.nav_desire_failed = False
    self.route_blinker_side: bool | None = None  # the side (left) of a blinker a route turn has had, until it goes off
    self.ahead_blinker_side: bool | None = None  # the side (left) of a blinker toward the route's next turn ahead, until it goes off
    self.turn_lane_change: bool | None = None  # the side (left) of the route's own lane change into a turn lane, while under way

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False, nav=None):
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    self.nav_desire.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    # Route turns: a blinker for the route's next turn gets that turn's desire shortly before it, and no other desire, and
    # from a route turn's start to its maneuver point no lane turn desire pulses either way. A blinker the route has had
    # stays the route's until it goes off, whatever navd, the steering or the controller do meanwhile, so the lane turn
    # desire never adds a turn pulse of its own for it.
    # A route message the controller cannot read must never stop the driving model, so it only drops the route turn.
    side = blinker_side(carstate)
    if side != self.route_blinker_side:
      self.route_blinker_side = None
    if side != self.ahead_blinker_side:
      self.ahead_blinker_side = None
    route_owned = self.nav_desire.owns_turn_desire or self.route_blinker_side is not None
    lane_turning = self.desire in (log.Desire.turnLeft, log.Desire.turnRight) and not route_owned
    failed = False
    try:
      self.nav_desire.update(carstate, lateral_active, nav, self.lane_change_state == LaneChangeState.laneChangeStarting, lane_turning)
    except Exception:
      if not self.nav_desire_failed:
        cloudlog.exception("desire_helper.nav_desire_failed")
      self.nav_desire_failed = failed = True
      self.nav_desire.reset()
    if self.nav_desire.owns_turn_desire and side is not None:
      self.route_blinker_side = side
    if self.nav_desire.turn_blinker_ahead and side is not None:
      self.ahead_blinker_side = side
    route_turning = self.nav_desire.turning is not None
    if self.nav_desire.owns_turn_desire or self.route_blinker_side is not None:
      self.lane_turn_direction = self.nav_desire.turn_direction
    elif self.nav_desire.route_active and not failed and not (lane_turning and TURN_DESIRES[self.lane_turn_direction] == self.desire):
      # with route turns on and a route, the route has the turns: a lane turn desire for a blinker the route cannot latch
      # yet (steering off, beyond the signal distance, the last turn not yet behind) would turn at once, then keep the route
      # from latching that blinker at all. One that went out while there was no route keeps going, and keeps the route off
      # that blinker, which would pulse again.
      self.lane_turn_direction = TurnDirection.none

    # A route turn lane: the route's own lane change toward the turn into the turn lane opening beside the car, with no
    # blinker and at any speed it allows, through the same states as any other. It ends early, and for good, as the route
    # says: the driver signaling, pressing the gas, steering against it or calling the turn off, or a car in that blind spot
    # before the car moved. That ends this lane change and its alert, not the driving model's, which already has its one
    # pulse: only the driver steering against it holds the car in its lane. The blinker that ends it never starts a lane
    # change of its own: it is no new blinker after.
    turn_lane = self.nav_desire.turn_lane
    turn_lane_completed = False

    # a route turn holds the lane change off, so no lane change waits hidden behind its turn desire
    if (not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF
        or route_turning):
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    elif self.turn_lane_change is not None and turn_lane != self.turn_lane_change:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif (self.lane_change_state == LaneChangeState.off and turn_lane is not None
            and not (left_edge_detected if turn_lane else right_edge_detected)):
        self.lane_change_state = LaneChangeState.laneChangeStarting
        self.lane_change_timer = 0.0
        self.lane_change_direction = LaneChangeDirection.left if turn_lane else LaneChangeDirection.right
        self.turn_lane_change = turn_lane
        self.nav_desire.start_turn_lane()

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        edge_detected = ((left_edge_detected and self.lane_change_direction == LaneChangeDirection.left) or
                         (right_edge_detected and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, edge_detected, carstate.brakePressed, torque_applied)

        # A blinker for the route's next turn needs a nudge to change lanes, so it never starts a nudgeless one, also once
        # the route let go of it (a reroute, or navd moving on) while it stays on, nor does one toward the turn switched on
        # before it latches, which may be the early signal the guide asks for on a fast road
        auto_lane_change_allowed = (self.alc.auto_lane_change_allowed and not self.nav_desire.route_turn_signal
                                    and self.route_blinker_side is None and self.ahead_blinker_side is None)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif (torque_applied or auto_lane_change_allowed) and not self.alc.lane_change_blocked:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          turn_lane_completed = self.turn_lane_change is not None
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    # the route's lane change into the turn lane is over: done, the model finished it with nothing ending it early, the car
    # is in the turn lane
    if self.turn_lane_change is not None and self.lane_change_state != LaneChangeState.laneChangeStarting:
      self.nav_desire.end_turn_lane(turn_lane_completed)
      self.turn_lane_change = None

    self.prev_one_blinker = one_blinker and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight
      elif self.lane_change_state == LaneChangeState.off:
        self.desire = self.nav_desire.keep_desire

    self.alc.update_state()
