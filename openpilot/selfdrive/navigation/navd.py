#!/usr/bin/env python3
import os
import time
from dataclasses import dataclass

import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.api import API_HOST, Api
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.route import bearing, cumulative_distance, local_meters, nearest_segment, point_along, project_onto_path

REROUTE_DISTANCE = 25
MANEUVER_TRANSITION_THRESHOLD = 10
NEXT_STEP_DISTANCE = 15  # m from the next step's path the car drives along it, once it took the maneuver
REROUTE_COUNTER_MIN = 3
ROUTE_RESEND_PERIOD = 10  # updates (seconds); a restarted mapsd gets the route within this
RECOMPUTE_HOLD = 10  # updates (seconds) between route requests
MAX_FIX_AGE = 1.5  # s; navd may read a fix a second late, so the published distances count the car's travel since, up to this

# The next step of a U-turn, a sharp turn or a roundabout turnaround runs right beside the road the car comes on, so the car
# takes a maneuver only once it reached the maneuver point and heads along the route after it
REACHED_DISTANCE = 15.  # m before the maneuver point
REACHED_TIME = 1.5  # s at the car's speed, so that a 1 Hz fix lands within it
HEADING_MIN_SPEED = 2.  # m/s; slower, the GPS bearing is noise, so the step waits
HEADING_TOLERANCE = 45.  # deg

# At a split the old road goes on straight beside the route's branch at first: an exit, a fork, or a turn within the heading
# tolerance, which a car going straight on would pass. Past it, the car is on whichever road it is clearly closer to. Moving
# on wrongly would bring the branch's next turn to a car on the old road, while a wrong miss only reroutes.
SPLIT_TYPES = ('off ramp', 'fork')
SPLIT_ANGLE = HEADING_TOLERANCE  # deg between the old road and the route's branch
SPLIT_NEAR = 8.  # m from the branch, at most, to be on it
SPLIT_MARGIN = 10.  # m closer to the branch than to the old road; a GPS error of a few meters can last seconds
SPLIT_MISSED = 10.  # m from the branch, at least, to have missed it
SPLIT_MISSED_MARGIN = 6.  # m closer to the old road than to the branch
ROUTE_AHEAD = 300.  # m of the route after a maneuver, and of the old road's straight continuation, that are compared
DIRECTION_LENGTH = 20.  # m of road a direction at a maneuver is measured over
# A turn onto a road that soon runs beside the old one, as a short connector to a service road does, is a split too: a car
# going straight on stays as close to the route ahead as one that took the turn
BESIDE_DISTANCE = (SPLIT_MARGIN, REROUTE_DISTANCE)  # m from the old road's straight continuation
BESIDE_SPACING = 2.  # m between the spots of the route ahead that are checked

# navd samples once a second, and at highway speed a short step is driven past between two fixes
REPLAY_SPACING = 2.  # m between the spots checked along the straight line from the previous fix
REPLAY_MAX = 80.  # m; a longer line, after a GPS gap, is not replayed

# Heading away from the step's direction, with its maneuver still far ahead, the car left the route
WRONG_HEADING_SPEED = 3.  # m/s
WRONG_HEADING_DISTANCE = 25.  # m before the maneuver point; nearer, turning in early is normal
WRONG_HEADING_FIXES = 2  # consecutive

# A road that branches off at a shallow angle stays within REROUTE_DISTANCE and HEADING_TOLERANCE of the step for a long way.
# A car taking it heads off the step's direction and moves away from it fix after fix, which a lane change does only
# briefly: once it heads along the step again, about where it was, it is back on the route.
LEAVING_COURSE = 12.  # deg off the step's direction, at least, while WRONG_HEADING_SPEED and WRONG_HEADING_DISTANCE hold
LEAVING_GROWTH = 1.  # m the car moved away from the step since the previous fix, at least
LEAVING_FIXES = 2  # consecutive
BACK_COURSE = 6.  # deg off the step's direction, at most
BACK_OFFSET = 4.  # m from the offset it had before it moved away, at most: a lane change and some GPS error
BACK_FIXES = 2  # consecutive

# Mapbox lane indications as navInstruction lane directions
LANE_DIRECTIONS = {'sharp left': 'left', 'left': 'left', 'slight left': 'slightLeft', 'straight': 'straight',
                   'slight right': 'slightRight', 'right': 'right', 'sharp right': 'right'}


def set_banner(instruction, banners, distance_to_maneuver):
  """Fill the banner fields from the banner to show now: the last one still ahead of the ego, else the first."""
  if not banners:
    return
  banner = next((b for b in reversed(banners) if distance_to_maneuver < b['distanceAlongGeometry']), banners[0])
  instruction.showFull = distance_to_maneuver < banner['distanceAlongGeometry']
  primary = banner['primary']
  instruction.maneuverPrimaryText = primary.get('text') or ''
  instruction.maneuverType = primary.get('type') or ''
  instruction.maneuverModifier = primary.get('modifier') or ''
  if banner.get('secondary') is not None:
    instruction.maneuverSecondaryText = banner['secondary']['text']
  lanes = [c for c in (banner.get('sub') or {}).get('components', []) if c.get('type') == 'lane']
  instruction.lanes = [{'directions': [LANE_DIRECTIONS.get(d, 'none') for d in lane.get('directions', [])],
                        'active': bool(lane.get('active')),
                        'activeDirection': LANE_DIRECTIONS.get(lane.get('active_direction'), 'none')} for lane in lanes]


def angle_between(a: float, b: float) -> float:
  """Degrees between two bearings, 0 to 180."""
  return abs((a - b + 180) % 360 - 180)


def step_maneuver(step) -> tuple[str, str]:
  """(type, modifier) of the maneuver a step ends at, from its banners."""
  banners = step['bannerInstructions']
  primary = banners[0]['primary'] if banners else {}
  return primary.get('type') or '', primary.get('modifier') or ''


@dataclass
class RouteAhead:
  """The route after a step's maneuver, which the car drives along once it took the maneuver."""
  path: np.ndarray  # (longitude, latitude) of the following steps, ROUTE_AHEAD m of them or to the route's end
  next_length: float  # m of the next step
  split: bool  # the old road goes on straight beside the route's branch at first
  maneuver: np.ndarray  # (longitude, latitude) of the maneuver point
  continuation: np.ndarray  # east/north meters from the maneuver point to the end of the old road's straight continuation

  def continuation_distance(self, position) -> float:
    """m from a (longitude, latitude) position to the old road's straight continuation."""
    return nearest_segment(np.zeros((1, 2)), self.continuation[None], local_meters(position, self.maneuver))[2]

  def comes_back_beside(self) -> bool:
    """Whether the route ahead, past its first DIRECTION_LENGTH, runs BESIDE_DISTANCE from the old road's straight
    continuation, heading the same way within HEADING_TOLERANCE."""
    distance = cumulative_distance(self.path)
    spots = np.arange(DIRECTION_LENGTH, min(float(distance[-1]), ROUTE_AHEAD) - BESIDE_SPACING, BESIDE_SPACING)
    if not len(spots) or np.linalg.norm(self.continuation) < 1e-3:
      return False
    local = local_meters(self.path, self.maneuver)

    def at(s: np.ndarray) -> np.ndarray:
      return np.stack([np.interp(s, distance, local[:, 0]), np.interp(s, distance, local[:, 1])], axis=1)

    here, ahead = at(spots), at(spots + BESIDE_SPACING)
    c = self.continuation
    t = np.clip(here @ c / (c @ c), 0., 1.)
    beside = np.linalg.norm(here - t[:, None] * c, axis=1)
    direction = ahead - here
    length = np.linalg.norm(direction, axis=1)
    same_way = direction @ c >= np.cos(np.radians(HEADING_TOLERANCE)) * np.maximum(length, 1e-6) * np.linalg.norm(c)
    return bool(np.any((length > 1e-3) & same_way & (beside >= BESIDE_DISTANCE[0]) & (beside <= BESIDE_DISTANCE[1])))


class RouteEngine:
  def __init__(self, sm, pm, params=None):
    self.sm = sm
    self.pm = pm
    self.params = params if params is not None else Params()

    self.last_position = None  # (longitude, latitude)
    self.last_bearing = None
    self.last_speed = 0.  # m/s
    self.fix_time = None  # monotonic s the last fix was published

    self.nav_destination = None  # the destination of the route
    self.requested_destination = None  # the destination of the last route request, also when it failed
    self.step_idx = None
    self.route = None
    self.route_geometry = None  # an (n, 2) array of (longitude, latitude) per step
    self.ahead_cache: tuple[list | None, dict[int, RouteAhead]] = (None, {})

    self.recompute_hold = 0
    self.reroute_counter = 0
    self.gps_ok = True
    self.reset_progress()

    self.api = Api(self.params.get("DongleId")) if not os.environ.get("MAPBOX_TOKEN") else None

  def reset_progress(self) -> None:
    """Forget the car's progress, for a new route or none."""
    self.missed_maneuver = False  # the car went past the maneuver point without taking it
    self.reached = False  # the car came up to the current step's maneuver point
    self.previous_position = None  # the last fix the steps were followed at
    self.wrong_heading_fixes = 0  # consecutive fixes heading away from the step's direction, far from its maneuver
    self.arrived = False  # the car came near the destination on the route's last road: navigation ends once it leaves
    self.reset_leaving()

  def reset_leaving(self) -> None:
    """Forget whether the car is leaving the current step, for a new step or route."""
    self.left_route = False  # the car went off along another road, as onto a branch
    self.leaving_fixes = 0  # consecutive fixes moving away from the step, heading off its direction
    self.back_fixes = 0  # consecutive fixes back along the step, after it left
    self.previous_offset = np.nan  # m from the step at the previous fix, none yet on this step
    self.offset_before = 0.  # m from the step before the car moved away from it

  @property
  def wrong_way(self) -> bool:
    return self.wrong_heading_fixes >= WRONG_HEADING_FIXES

  def update(self):
    self.sm.update(0)

    source = latest_gps_source(self.sm)
    if source is not None:
      gps = self.sm[source]
      self.last_position, self.last_bearing, self.last_speed = (gps.longitude, gps.latitude), gps.bearingDeg, gps.speed
      self.fix_time = self.sm.logMonoTime[source] * 1e-9
    # without a fresh fix the distance to the maneuver is frozen, so the instruction is not valid for steering on
    self.gps_ok = source is not None
    route = self.route
    try:
      self.recompute_route(gps_ok=source is not None)
      self.send_instruction()
    except Exception:
      cloudlog.exception("navd.failed_to_compute")
    # mapsd dedupes repeats, so a periodic resend is how a restarted mapsd gets the route
    if self.route is not route or self.sm.frame % ROUTE_RESEND_PERIOD == 0:
      self.send_route()

  def recompute_route(self, gps_ok: bool):
    if self.last_position is None:
      return

    destination = self.params.get("NavDestination")
    if destination is None:
      self.clear_route()
      self.recompute_hold = 0
      return
    new_destination = (destination['longitude'], destination['latitude'])

    # Don't recompute when GPS drifts in tunnels
    if not gps_ok and self.step_idx is not None:
      return

    self.recompute_hold -= 1
    # Once the car arrived, navigation ends as it leaves, whichever way, never with a route back
    if self.arrived and new_destination == self.nav_destination:
      if not self.near_end():
        self.end_navigation()
      return
    # A new destination gets its route at once, rather than leave the old route's turns going until the hold ends; a
    # reroute, or another try after a failed request, waits for it
    new = new_destination not in (self.nav_destination, self.requested_destination)
    if new or (self.should_recompute() and self.recompute_hold <= 0):
      self.recompute_hold = RECOMPUTE_HOLD
      self.calculate_route(new_destination)
      self.reroute_counter = 0

  def directions_auth(self) -> tuple[str, str]:
    """(host, access token) for directions: a personal Mapbox token when one is set, else the comma API with the device's token."""
    token = os.environ.get("MAPBOX_TOKEN") or (self.params.get("MapboxToken") or '').strip()
    if token:
      return "https://api.mapbox.com", token
    return API_HOST.rstrip('/') + '/v1/nav', self.api.get_token()

  def calculate_route(self, destination):
    assert self.last_position is not None
    cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")
    self.nav_destination = self.requested_destination = destination

    lang = self.params.get('LanguageSetting')
    if lang is not None:
      lang = lang.replace('main_', '')

    host, token = self.directions_auth()

    params = {
      'access_token': token,
      'geometries': 'geojson',
      'overview': 'full',
      'steps': 'true',
      'banner_instructions': 'true',
      'alternatives': 'false',
      'language': lang,
    }
    if self.last_bearing is not None:
      params['bearings'] = f"{(self.last_bearing + 360) % 360:.0f},90;"

    coords_str = ';'.join(f'{lon},{lat}' for lon, lat in (self.last_position, destination))
    url = host + '/directions/v5/mapbox/driving-traffic/' + coords_str
    try:
      resp = requests.get(url, params=params, timeout=10)
      if resp.status_code != 200:
        cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
      resp.raise_for_status()

      r = resp.json()
      if len(r['routes']):
        self.route = r['routes'][0]['legs'][0]['steps']
        self.route_geometry = [np.array(step['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for step in self.route]
        self.step_idx = 0
        self.reset_progress()
      else:
        cloudlog.warning("Got empty route response")
        self.clear_route()

    except requests.exceptions.RequestException as e:
      # requests exceptions contain the URL, and with it the access token; keep it out of logs
      cloudlog.error(f"failed to get route: {type(e).__name__}")
      self.clear_route()

  def send_instruction(self):
    msg = messaging.new_message('navInstruction', valid=True)

    if self.step_idx is None or not self.gps_ok:
      msg.valid = False
      self.pm.send('navInstruction', msg)
      return

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    self.follow_route()
    step = self.route[self.step_idx]
    along, off_route, direction = project_onto_path(self.route_geometry[self.step_idx], self.last_position, extend=True)
    distance_to_maneuver = step['distance'] - along

    # Past the maneuver point without taking it, the car missed the maneuver: what comes after it is not ahead of the car,
    # so the instruction is invalid until the reroute, as it is off the route, heading away from the step's direction, or
    # gone off along another road. The last maneuver, the arrival, keeps the distance rule.
    road_ahead = self.step_idx + 2 < len(self.route)
    self.missed_maneuver = road_ahead and self.missed(distance_to_maneuver)
    # the course tells only while the car moves, and far from the maneuver, where turning in early is not why it is off
    course_off = angle_between(self.last_bearing, direction)
    course_counts = self.last_speed >= WRONG_HEADING_SPEED and distance_to_maneuver > WRONG_HEADING_DISTANCE
    self.wrong_heading_fixes = self.wrong_heading_fixes + 1 if course_counts and course_off > HEADING_TOLERANCE else 0
    self.follow_leaving(off_route, course_off, course_counts)
    msg.valid = not self.missed_maneuver and not self.wrong_way and not self.left_route and off_route <= REROUTE_DISTANCE

    # Banner instructions are for the following maneuver step, don't use empty last step
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    # Current instruction, for where the car is as it goes out: the fix may be a second old
    fix_age = float(np.clip(time.monotonic() - self.fix_time, 0., MAX_FIX_AGE)) if self.fix_time is not None else 0.
    maneuver_distance = distance_to_maneuver - self.last_speed * fix_age
    msg.navInstruction.maneuverDistance = maneuver_distance
    set_banner(msg.navInstruction, banner_step['bannerInstructions'], maneuver_distance)

    # Total remaining distance and time: the rest of this step, then every step after it. So the maneuver's distance to the
    # destination is the later steps' sum, whatever the GPS does, which is how controls tell a GPS jump from the next maneuver.
    future = self.route[self.step_idx + 1:]
    msg.navInstruction.distanceRemaining = maneuver_distance + sum(s['distance'] for s in future)
    remaining = max(distance_to_maneuver, 0.) / max(step['distance'], 1)
    msg.navInstruction.timeRemaining = step['duration'] * remaining + sum(s['duration'] for s in future)

    self.pm.send('navInstruction', msg)

    # Navigation ends once the car drives away from the destination, whichever way it leaves, after it came near it on the
    # route's last road. Passing near it earlier, as on the far side of a divided road before a U-turn, is not arriving.
    near = self.near_end()
    self.arrived |= near and self.step_idx >= len(self.route) - 2
    if self.arrived and not near:
      self.end_navigation()
    elif not road_ahead and distance_to_maneuver < -MANEUVER_TRANSITION_THRESHOLD and self.step_idx + 1 < len(self.route):
      self.next_step()

  def follow_leaving(self, offset: float, course_off: float, course_counts: bool) -> None:
    """Whether the car went off along another road: heading LEAVING_COURSE off the step's direction and moving away from it,
    fix after fix, while its course counts. Back along the step about where it was before, it is on the route again."""
    previous, self.previous_offset = self.previous_offset, offset
    moving_away = course_counts and course_off > LEAVING_COURSE and offset - previous >= LEAVING_GROWTH
    if moving_away and self.leaving_fixes == 0:
      self.offset_before = previous
    self.leaving_fixes = self.leaving_fixes + 1 if moving_away else 0
    if self.leaving_fixes >= LEAVING_FIXES:
      self.left_route, self.back_fixes = True, 0
    elif self.left_route:
      back = course_off <= BACK_COURSE and abs(offset - self.offset_before) <= BACK_OFFSET
      self.back_fixes = self.back_fixes + 1 if back else 0
      self.left_route = self.back_fixes < BACK_FIXES

  def near_end(self) -> bool:
    """Whether the car is within REROUTE_DISTANCE of the destination, or of the route's end, where Mapbox put it on the road."""
    assert self.route_geometry is not None and self.last_position is not None
    ends = [self.route_geometry[-1][-1]] + ([self.nav_destination] if self.nav_destination is not None else [])
    return any(float(np.linalg.norm(local_meters(self.last_position, end))) <= REROUTE_DISTANCE for end in ends)

  def end_navigation(self) -> None:
    """The car arrived and drove away: the destination is done with, unless the driver set another one meanwhile."""
    cloudlog.warning("Destination reached")
    destination = self.params.get("NavDestination")
    if destination is not None and (destination['longitude'], destination['latitude']) == self.nav_destination:
      self.params.remove("NavDestination")
    self.clear_route()

  def next_step(self) -> None:
    assert self.step_idx is not None
    self.step_idx += 1
    self.reached = False
    self.recompute_hold = 0
    self.reset_leaving()

  def follow_route(self) -> None:
    """Move on through every step the car took since the last fix. The straight line from that fix to this one is checked
    every 2 m, as if navd had sampled there, with this fix's speed and bearing: at highway speed the car drives past a short
    step between two fixes. A split is taken only at the fix itself: when one of the two fixes is off by a GPS error, the
    line between them crosses the branch's side where the car never was, and past a split the route ahead is compared far
    enough that the fix alone tells."""
    assert self.route is not None and self.step_idx is not None and self.last_position is not None
    position = np.array(self.last_position, dtype=np.float64)
    positions = [position]
    if self.previous_position is not None:
      length = float(np.linalg.norm(local_meters(position, self.previous_position)))
      if length <= REPLAY_MAX:
        line = position - self.previous_position
        positions = [self.previous_position + line * d / length for d in np.arange(REPLAY_SPACING, length, REPLAY_SPACING)] + positions
    for i, spot in enumerate(positions):
      at_fix = i == len(positions) - 1
      while self.step_idx + 2 < len(self.route):  # the last road step, to the destination, keeps the distance rule
        if not self.reached:
          along = project_onto_path(self.route_geometry[self.step_idx], spot, extend=True)[0]
          self.reached = self.route[self.step_idx]['distance'] - along <= max(REACHED_DISTANCE, self.last_speed * REACHED_TIME)
        if not at_fix and self.route_ahead(self.step_idx).split:
          break
        if not self.took_maneuver(spot):
          break
        self.next_step()
    self.previous_position = position

  def route_ahead(self, idx: int) -> RouteAhead:
    """The route after step idx's maneuver, kept until the route changes."""
    assert self.route is not None and self.route_geometry is not None
    if self.ahead_cache[0] is not self.route_geometry:
      self.ahead_cache = (self.route_geometry, {})
    cache = self.ahead_cache[1]
    if idx not in cache:
      paths, length = [], 0.
      for path in self.route_geometry[idx + 1:]:
        paths.append(path)
        length += float(cumulative_distance(path)[-1])
        if length >= ROUTE_AHEAD:
          break
      road, following = self.route_geometry[idx], self.route_geometry[idx + 1]
      maneuver, next_length = road[-1], float(cumulative_distance(following)[-1])
      old = local_meters(maneuver, point_along(road, float(cumulative_distance(road)[-1]) - DIRECTION_LENGTH))
      new = local_meters(point_along(following, DIRECTION_LENGTH), maneuver)
      kind, modifier = step_maneuver(self.route[idx])
      continuation = old / max(float(np.linalg.norm(old)), 1e-3) * ROUTE_AHEAD
      ahead = RouteAhead(np.concatenate(paths), next_length, False, maneuver, continuation)
      if min(np.linalg.norm(old), np.linalg.norm(new)) > 1e-3 and modifier != 'straight':
        # a maneuver straight on, or a merge, stays on the road: no other road goes on beside it
        turn = kind != 'merge' and modifier != ''
        slight = turn and angle_between(bearing(*old), bearing(*new)) <= SPLIT_ANGLE
        ahead.split = kind in SPLIT_TYPES or slight or (turn and ahead.comes_back_beside())
      cache[idx] = ahead
    return cache[idx]

  def along_route_ahead(self, position) -> tuple[RouteAhead, float, float, bool | None]:
    """The route after the current step's maneuver, a position's progress along it and distance from it, and whether the
    car heads along it there, None while too slow to tell. Past a short next step, that is the road the car is on now."""
    assert self.step_idx is not None and self.last_bearing is not None
    ahead = self.route_ahead(self.step_idx)
    along, lateral, direction = project_onto_path(ahead.path, position)
    heading = angle_between(self.last_bearing, direction) <= HEADING_TOLERANCE if self.last_speed >= HEADING_MIN_SPEED else None
    return ahead, along, lateral, heading

  def on_next_step(self, ahead: RouteAhead, position) -> bool:
    """Close to the next step's path and some way along it."""
    assert self.route_geometry is not None and self.step_idx is not None
    along, lateral, _ = project_onto_path(self.route_geometry[self.step_idx + 1], position)
    return lateral <= NEXT_STEP_DISTANCE and along >= min(MANEUVER_TRANSITION_THRESHOLD, ahead.next_length / 2)

  def took_maneuver(self, position) -> bool:
    """Whether the car took the current step's maneuver: it reached the maneuver point, and drives along the next step.
    Past a split it must instead be on the route and clearly closer to it than to the old road going on, measured along the
    following steps too, as the car may be past a short next step before it is clearly apart from the old road."""
    if not self.reached:
      return False
    ahead, along, lateral, heading = self.along_route_ahead(position)
    if not heading:
      return False
    if ahead.split:
      return (along >= min(MANEUVER_TRANSITION_THRESHOLD, ahead.next_length / 2) and lateral <= SPLIT_NEAR and
              ahead.continuation_distance(position) - lateral >= SPLIT_MARGIN)
    return self.on_next_step(ahead, position)

  def missed(self, distance_to_maneuver: float) -> bool:
    """Whether the car went past the current step's maneuver point without taking it."""
    ahead, _, lateral, heading = self.along_route_ahead(self.last_position)
    if ahead.split:
      # past a split, until the car is clearly on one road, the step stays: its maneuver is behind the car, so nothing
      # turns or slows for it
      return distance_to_maneuver < 0. and lateral >= SPLIT_MISSED and lateral - ahead.continuation_distance(self.last_position) >= SPLIT_MISSED_MARGIN
    # on the next step but too slow to tell its heading, it waits
    return distance_to_maneuver < -MANEUVER_TRANSITION_THRESHOLD and not (self.on_next_step(ahead, self.last_position) and heading is not False)

  def send_route(self):
    coords = np.concatenate(self.route_geometry) if self.route_geometry is not None else np.empty((0, 2))
    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = [{'longitude': lon, 'latitude': lat} for lon, lat in coords.tolist()]
    self.pm.send('navRoute', msg)

  def clear_route(self):
    self.route = None
    self.route_geometry = None
    self.step_idx = None
    self.nav_destination = None
    self.reset_progress()

  def should_recompute(self):
    if self.step_idx is None:
      return True

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    # Don't recompute in last segment, assume destination is reached
    if self.step_idx == len(self.route) - 1:
      return False
    if self.missed_maneuver:
      return True

    # off the route, heading the wrong way along it, or gone off along another road
    off_route = project_onto_path(self.route_geometry[self.step_idx], self.last_position, extend=True)[1] > REROUTE_DISTANCE
    if off_route or self.wrong_way or self.left_route:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute'])
  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
