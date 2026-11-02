"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import ast
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.cereal import custom
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.nav_desire import NAV_TURN_HOLD, NAV_TURN_MARGIN, NAV_TURN_SPEED, NavDesireController, \
  NavInputs, turn_speed_limit
from openpilot.sunnypilot.selfdrive.controls.lib.nav_turn_speed import NAV_TURN_ALERT_DECEL, NAV_TURN_ALERT_RESET, NAV_TURN_JERK, \
  NAV_TURN_MAX_DECEL, NavTurnSpeedController

EventNameSP = custom.OnroadEventSP.EventName
LongitudinalPlanSourceSP = custom.LongitudinalPlanSP.LongitudinalPlanSource


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


def nav_controller():
  """modeld's route state, with automatic turns on."""
  params = Params()
  params.put_bool('NavTurnDesire', False)
  params.put_bool('NavAutoTurn', True)
  return NavDesireController()


def route_limit(nav, v_ego, distance, position=1000.):
  """The speed limit modeld publishes for a right turn `distance` meters ahead and `position` meters before the
  destination, with openpilot in control of the speed, one frame on."""
  instruction = messaging.new_message('navInstruction').navInstruction
  instruction.maneuverType, instruction.maneuverModifier = 'turn', 'right'
  instruction.maneuverDistance, instruction.distanceRemaining = distance, distance + position
  carstate = messaging.new_message('carState').carState
  carstate.vEgo, carstate.standstill = v_ego, v_ego < 0.01
  nav.update(carstate, True, NavInputs(instruction=instruction, age=0., long_active=True))
  return nav.speed_limit


def plant(**kwargs):
  """Upstream's longitudinal Plant, a simple car driven by plannerd's real LongitudinalPlanner, whose MPC scons compiles."""
  try:
    from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant
  except ModuleNotFoundError as e:
    if e.name != 'casadi' and 'c_generated_code' not in str(e.name):
      raise
    raise unittest.SkipTest(f"the longitudinal MPC is not built: {e}") from None
  return Plant(**kwargs)


def source(module: str) -> ast.Module:
  # processes are read from their source: importing them needs compiled pieces
  return ast.parse(Path(BASEDIR, *module.split('.')).with_suffix('.py').read_text())


def services(tree: ast.Module, master: str) -> set[str]:
  """The services a process's SubMaster or PubMaster is made with, following a list assigned to a name."""
  assigned = {target.id: node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
              for target in node.targets if isinstance(target, ast.Name)}
  found = set()
  for node in ast.walk(tree):
    if isinstance(node, ast.Call) and master in (getattr(node.func, 'id', None), getattr(node.func, 'attr', None)) and node.args:
      arg = node.args[0]
      arg = assigned.get(arg.id, arg) if isinstance(arg, ast.Name) else arg
      found |= {n.value for n in ast.walk(arg) if isinstance(n, ast.Constant) and isinstance(n.value, str)}
  return found


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
    nav, c = nav_controller(), controller()
    v, a, distance, events, v_at_margin = 20., 0., 400., [], None
    while distance > -30.:
      events += step(c, submaster(route_limit(nav, v, distance)), v, a_prev=a)
      a = min(c.a_target, 0.) if c.active else 0.  # the cruise speed is 20 m/s
      v = max(v + a * DT_MDL, 0.)
      distance -= v * DT_MDL
      if v_at_margin is None and distance <= NAV_TURN_MARGIN:
        v_at_margin = v
    assert events == [EventNameSP.navTurnSlowing]
    assert v_at_margin <= NAV_TURN_SPEED + 0.5

  def test_alerts_once_for_each_automatic_turn(self):
    # three right turns 200 m apart at 30 mph. Between two, the next turn's limit is far above the car's speed while it
    # speeds up again; navd moves on to the next turn once the car drives along the street it turned into.
    nav, c = nav_controller(), controller()
    turns, v_set = (300., 500., 700.), 13.4
    v, a, position, alerts, v_at_margins = v_set, 0., 0., [], []
    while position < turns[-1] + NAV_TURN_HOLD:
      turn = min((t for t in turns if position < t + 10.), default=turns[-1])
      limit = route_limit(nav, v, turn - position, position=1000. - turn)
      alerts += [position] * len(step(c, submaster(limit), v, a_prev=a))
      a_cruise = float(np.clip(v_set - v, -0.5, 1.))
      a = min(c.a_target, a_cruise) if c.active else a_cruise
      v = max(v + a * DT_MDL, 0.)
      position += v * DT_MDL
      if len(v_at_margins) < len(turns) and position >= turns[len(v_at_margins)] - NAV_TURN_MARGIN:
        v_at_margins.append(v)
    assert len(alerts) == len(turns), alerts
    for alert, turn, previous in zip(alerts, turns, (-math.inf, *turns), strict=False):
      assert previous + NAV_TURN_HOLD < alert < turn - NAV_TURN_MARGIN  # as the slowdown for each turn starts
    assert max(v_at_margins) <= NAV_TURN_SPEED + 0.5

  def test_alerts_once_for_each_turn_of_a_series(self):
    # the limit binds into a turn and holds turn speed through it; then the next turn's limit lets the car speed up
    c = controller()
    events = []
    for _ in range(3):
      events += step(c, submaster(10.), 15., frames=round(2. / DT_MDL))
      events += step(c, submaster(NAV_TURN_SPEED), NAV_TURN_SPEED - NAV_TURN_ALERT_DECEL / 2, frames=round(8. / DT_MDL))
      events += step(c, submaster(16.), 8., frames=round((NAV_TURN_ALERT_RESET + 0.5) / DT_MDL))
    assert events == [EventNameSP.navTurnSlowing] * 3

  def test_holding_turn_speed_keeps_the_alert(self):
    # however long the car takes through the turn, the limit still holds it: no second alert for the same turn
    c = controller()
    assert step(c, submaster(10.), 15.) == [EventNameSP.navTurnSlowing]
    assert not step(c, submaster(NAV_TURN_SPEED), NAV_TURN_SPEED, frames=round(2 * NAV_TURN_ALERT_RESET / DT_MDL))
    assert not step(c, submaster(NAV_TURN_SPEED), NAV_TURN_SPEED + 1.5)

  def test_alerts_again_only_after_a_pause(self):
    c = controller()
    assert step(c, submaster(10.), 15.) == [EventNameSP.navTurnSlowing]
    assert not step(c, submaster(0.), 15., frames=round(3. / DT_MDL))  # a short break, like navd moving on
    assert not step(c, submaster(10.), 15.)
    assert not step(c, submaster(16.), 8., frames=round(3. / DT_MDL))  # held below the limit for a while, by a slower car
    assert not step(c, submaster(10.), 15.)
    step(c, submaster(0.), 15., frames=round((NAV_TURN_ALERT_RESET + 1.) / DT_MDL))
    assert step(c, submaster(10.), 15.) == [EventNameSP.navTurnSlowing]


class TestNavTurnPlanner(OpenpilotTestCase):
  """The published limit through plannerd's real LongitudinalPlanner, in upstream's longitudinal Plant."""

  def test_slows_by_the_turn_and_releases(self):
    p, turn = plant(speed=20.), 300.
    from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MIN
    rows, events = [], []
    while p.distance < turn + NAV_TURN_HOLD:
      out = p.step(v_cruise=20., nav_turn_speed=turn_speed_limit(turn - p.distance))
      rows.append((turn - out['distance'], out['speed'], out['acceleration'], p.planner.source))
      events += p.planner.events_sp.names
    accels = [r[2] for r in rows]
    assert -NAV_TURN_MAX_DECEL - 1e-6 <= min(accels) < A_CRUISE_MIN - 0.2  # harder than the cruise limit allows, within the cap
    assert next(v for d, v, _, _ in rows if d <= NAV_TURN_MARGIN) <= NAV_TURN_SPEED + 0.5
    assert all(abs(v - NAV_TURN_SPEED) < 0.3 and source == LongitudinalPlanSourceSP.navTurn for d, v, _, source in rows if d <= 0.)
    assert events.count(EventNameSP.navTurnSlowing) == 1

    # once modeld stops publishing a limit, back toward the set speed
    for _ in range(round(5. / DT_MDL)):
      out = p.step(v_cruise=20.)
    assert not p.planner.nav_turn.active
    assert out['speed'] > NAV_TURN_SPEED + 1.

  def test_late_turn_brakes_at_most_the_cap(self):
    p = plant(speed=15.)
    accels = [p.step(v_cruise=20., nav_turn_speed=NAV_TURN_SPEED)['acceleration'] for _ in range(round(10. / DT_MDL))]
    assert math.isclose(accels[0], -NAV_TURN_JERK * DT_MDL)  # eases in from the planner's last output
    assert math.isclose(min(accels), -NAV_TURN_MAX_DECEL, abs_tol=1e-3)
    assert abs(p.speed - NAV_TURN_SPEED) < 0.2

  def test_disengaged(self):
    # openpilot disengaged, so carControl is too: the planner leaves the limit to the driver
    p = plant(speed=15., enabled=False)
    p.step(v_cruise=20., nav_turn_speed=NAV_TURN_SPEED)
    assert not p.planner.nav_turn.active


class TestNavTurnWiring(OpenpilotTestCase):
  def test_processes_carry_the_route_turn(self):
    # modeld reads navd's instruction and publishes the route turn's limit, which plannerd reads; a service a process
    # reads without subscribing to it crashes that process
    for modeld in ('openpilot.selfdrive.modeld.modeld', 'openpilot.sunnypilot.modeld_v2.modeld'):
      tree = source(modeld)
      assert 'navInstruction' in services(tree, 'SubMaster'), modeld
      assert 'modelDataV2SP' in services(tree, 'PubMaster'), modeld
      assert any(isinstance(n, ast.Attribute) and n.attr == 'navTurnSpeed' and isinstance(n.ctx, ast.Store) for n in ast.walk(tree)), modeld
    assert 'modelDataV2SP' in services(source('openpilot.selfdrive.controls.plannerd'), 'SubMaster')
