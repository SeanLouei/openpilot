"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Automatic far side route turns, from the setting to the alert: the param, both UIs, modeld's inputs and output, plannerd's
posted limit, selfdrived's alerts and their texts. Processes are read from their source: importing them needs compiled
pieces.
"""
import ast
import json
import re
import textwrap
from pathlib import Path
from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from opendbc.car.structs import car
from openpilot.common.basedir import BASEDIR
from openpilot.common.parameterized import parameterized
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_nav_turn_speed import services, source
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, ET, Alert, EventsSP, Priority
from openpilot.sunnypilot.sunnylink.tools.compile_settings_ui import DEFAULT_OUT

EventNameSP = custom.OnroadEventSP.EventName
NavTurnAcross = custom.ModelDataV2SP.NavTurnAcross
AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert
MODELDS = ('openpilot.selfdrive.modeld.modeld', 'openpilot.sunnypilot.modeld_v2.modeld')


def calls(tree: ast.AST, name: str) -> list[ast.Call]:
  return [node for node in ast.walk(tree) if isinstance(node, ast.Call) and name in (getattr(node.func, 'id', None), getattr(node.func, 'attr', None))]


def method(tree: ast.Module, cls: str, name: str) -> ast.FunctionDef:
  owner = next(node for node in ast.walk(tree) if isinstance(node, ast.ClassDef) and node.name == cls)
  return next(node for node in owner.body if isinstance(node, ast.FunctionDef) and node.name == name)


def top_level(function: ast.FunctionDef) -> str:
  """The function's statements that always run: not under an if, a loop or a try."""
  return '\n'.join(ast.unparse(statement) for statement in function.body if not isinstance(statement, (ast.If, ast.For, ast.While, ast.Try)))


class TestModeld(OpenpilotTestCase):
  def test_reads_the_posted_limit(self):
    # a service a process reads without subscribing to it crashes that process
    for modeld in MODELDS:
      assert 'longitudinalPlanSP' in services(source(modeld), 'SubMaster'), modeld

  def test_publishes_the_turn_across(self):
    for modeld in MODELDS:
      tree = source(modeld)
      values = [node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
                for target in node.targets if isinstance(target, ast.Attribute) and target.attr == 'navTurnAcross']
      expected = "'turning' if DH.nav_desire.turn_across else 'signal' if DH.nav_desire.signal_prompt else 'none'"
      assert len(values) == 1 and ast.unparse(values[0]) == expected, modeld
    # each value it can publish is one of the enum's
    msg = messaging.new_message('modelDataV2SP').modelDataV2SP
    for turn_across, signal_prompt, published in ((True, True, 'turning'), (False, True, 'signal'), (False, False, 'none')):
      DH = SimpleNamespace(nav_desire=SimpleNamespace(turn_across=turn_across, signal_prompt=signal_prompt))
      msg.navTurnAcross = eval(expected, {'DH': DH})
      assert msg.navTurnAcross == getattr(NavTurnAcross, published)

  def test_no_service_can_invalidate_modeld(self):
    # modeld checks none of its services as a whole, so the posted limit missing or late changes nothing but the far side turn
    for modeld in MODELDS:
      tree = source(modeld)
      assert not any(calls(tree, name) for name in ('all_checks', 'all_alive', 'all_valid', 'all_freq_ok')), modeld


class TestPlannerd(OpenpilotTestCase):
  def test_publishes_the_posted_limit_every_plan(self):
    # the resolver runs and is published on every plan, whether or not speed limit assist is on
    assert 'longitudinalPlanSP' in services(source('openpilot.selfdrive.controls.plannerd'), 'PubMaster')
    tree = source('openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner')
    assert 'self.resolver.update(v_ego, sm)' in top_level(method(tree, 'LongitudinalPlannerSP', 'update_targets'))
    publish = top_level(method(tree, 'LongitudinalPlannerSP', 'publish_longitudinal_plan_sp'))
    assert 'resolver.speedLimit = float(self.resolver.speed_limit)' in publish
    assert 'resolver.speedLimitValid = self.resolver.speed_limit_valid' in publish and "pm.send('longitudinalPlanSP', plan_sp_send)" in publish
    planner = source('openpilot.selfdrive.controls.lib.longitudinal_planner')
    assert 'self.publish_longitudinal_plan_sp(sm, pm)' in top_level(method(planner, 'LongitudinalPlanner', 'publish'))


class TestSelfdrived(OpenpilotTestCase):
  def lane_turn_events(self, nav_turn_across, lane_turn_direction) -> list[int]:
    """Runs selfdrived's own lane turn block on a real modelDataV2SP message, with selfdrived's own module level names,
    returning the events it adds."""
    text = (Path(BASEDIR) / 'openpilot/selfdrive/selfdrived/selfdrived.py').read_text()
    block = text[text.index('    # Handle lane turn'):text.index("    for i, pandaState in enumerate(self.sm['pandaStates']):")]
    names = {'custom': custom}
    for node in ast.parse(text).body:
      if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) in ('TurnDirection', 'NavTurnAcross'):
        exec(ast.unparse(node), names)
    msg = messaging.new_message('modelDataV2SP').modelDataV2SP
    msg.navTurnAcross, msg.laneTurnDirection = nav_turn_across, lane_turn_direction
    selfdrived = SimpleNamespace(sm={'modelDataV2SP': msg}, events_sp=EventsSP())
    exec(textwrap.dedent(block), names | {'self': selfdrived})
    return selfdrived.events_sp.names

  @parameterized.expand([
    ('none', 'none', []),
    ('none', 'turnLeft', [EventNameSP.laneTurnLeft]),
    ('none', 'turnRight', [EventNameSP.laneTurnRight]),
    ('turning', 'turnLeft', [EventNameSP.navTurnAcross]),  # in place of Turning Left
    ('turning', 'none', [EventNameSP.navTurnAcross]),  # also in the frame between two pulses
    ('signal', 'none', [EventNameSP.navTurnSignal]),
  ])
  def test_raises_the_alerts(self, nav_turn_across, lane_turn_direction, expected):
    assert self.lane_turn_events(nav_turn_across, lane_turn_direction) == expected


class TestEvents(OpenpilotTestCase):
  @parameterized.expand([
    ('navTurnAcross', 27, "Turning Across Traffic", "Brake for oncoming traffic", AudibleAlertSP.promptSingleHigh),
    ('navTurnSignal', 28, "Signal for Route Turn", "It won't turn by itself here", AudibleAlertSP.promptSingleLow),
  ])
  def test_alert(self, name, number, text_1, text_2, sound):
    event = getattr(EventNameSP, name)
    assert event == number and list(EVENTS_SP[event]) == [ET.WARNING]
    alert = EVENTS_SP[event][ET.WARNING]
    assert (alert.alert_text_1, alert.alert_text_2, alert.alert_status, alert.alert_size, alert.priority, alert.visual_alert,
            alert.audible_alert, alert.duration) == (text_1, text_2, AlertStatus.userPrompt, AlertSize.mid, Priority.MID, VisualAlert.none,
                                                     sound, int(1. / DT_CTRL))
    events = EventsSP()
    events.add(event)
    assert [a.alert_type for a in events.create_alerts([ET.WARNING])] == [f'{name}/{ET.WARNING}']

  def test_turn_across_states(self):
    assert (NavTurnAcross.none, NavTurnAcross.turning, NavTurnAcross.signal) == (0, 1, 2)
    assert messaging.new_message('modelDataV2SP').modelDataV2SP.navTurnAcross == NavTurnAcross.none

  def test_every_event_has_an_alert(self):
    # as upstream's test_alerts does for its events. hyundaiRadarTracksConfirmed has had no entry since before this branch,
    # and nothing raises it.
    missing = [name for name, number in EventNameSP.schema.enumerants.items()
               if not name.endswith('DEPRECATED') and number not in EVENTS_SP and name != 'hyundaiRadarTracksConfirmed']
    assert not missing

  def test_alert_sanity(self):
    # upstream's test_alerts rules for the alert sizes and texts
    for alerts in EVENTS_SP.values():
      for event_type, alert in alerts.items():
        if not isinstance(alert, Alert):
          continue
        if alert.alert_size == AlertSize.none:
          assert not alert.alert_text_1 and not alert.alert_text_2
        elif alert.alert_size == AlertSize.small:
          assert alert.alert_text_1 and not alert.alert_text_2
        elif alert.alert_size == AlertSize.mid:
          assert alert.alert_text_1 and alert.alert_text_2
        else:
          assert alert.alert_text_1
        assert alert.duration >= 0. and (event_type in (ET.WARNING, ET.PERMANENT, ET.PRE_ENABLE) or alert.creation_delay == 0.)


class TestSetting(OpenpilotTestCase):
  def test_param_is_not_backed_up(self):
    # off by default, and never restored from a sunnylink backup: a restore or a reinstall leaves it off
    header = (Path(BASEDIR) / 'openpilot/common/params_keys.h').read_text()
    entries = re.findall(r'\{"NavAutoTurnFarSide",\s*\{([^{}]*)\}\}', header)
    assert len(entries) == 1
    flags, kind, default = (part.strip() for part in entries[0].split(','))
    assert flags == 'PERSISTENT' and kind == 'BOOL' and default == '"0"'

  def test_comma_4_toggle(self):
    tree = source('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models')
    far = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == 'self.nav_far_turn_toggle')
    assert getattr(far.func, 'id', None) == 'BigParamControl'
    assert ast.unparse(far.args[0]) == "tr('auto left turns (alpha)')" and ast.literal_eval(far.args[1]) == 'NavAutoTurnFarSide'
    items = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == 'self.main_items')
    names = [ast.unparse(element) for element in items.elts]
    assert names[names.index('self.nav_auto_turn_toggle') + 1] == 'self.nav_far_turn_toggle'
    # it shows the setting only with navigation and automatic route turns on, can be switched on only while parked, and
    # never writes the param itself: only the driver's tap does
    update = ast.unparse(method(tree, 'ModelsLayoutMici', '_update_state'))
    assert "far_checked = navigation and auto_turns and ui_state.params.get_bool('NavAutoTurnFarSide')" in update
    assert 'self.nav_far_turn_toggle.set_checked(far_checked)' in update
    assert 'self.nav_far_turn_toggle.set_enabled(navigation and auto_turns and (ui_state.is_offroad() or far_checked))' in update
    assert "ui_state.params.get_bool('NavigationEnabled'), ui_state.params.get_bool('NavAutoTurn')" in update
    writes = [call for name in ('put', 'put_bool', 'put_nonblocking', 'remove') for call in calls(tree, name)
              if any(isinstance(arg, ast.Constant) and arg.value == 'NavAutoTurnFarSide' for arg in call.args)]
    assert not writes

  def test_sunnylink_item(self):
    with open(DEFAULT_OUT) as f:
      schema = json.load(f)
    section = next(section for panel in schema['panels'] if panel['id'] == 'steering' for section in panel['sections'] if section['id'] == 'navigation')
    keys = [item['key'] for item in section['items']]
    assert keys[keys.index('NavAutoTurn') + 1] == 'NavAutoTurnFarSide'
    item = section['items'][keys.index('NavAutoTurnFarSide')]
    assert item['widget'] == 'toggle' and item['title'] == 'Automatic Left Turns (Alpha)'
    assert item['enablement'] == [{'type': 'offroad_only'}, {'type': 'param', 'key': 'NavigationEnabled', 'equals': True},
                                  {'type': 'param', 'key': 'NavAutoTurn', 'equals': True}]
