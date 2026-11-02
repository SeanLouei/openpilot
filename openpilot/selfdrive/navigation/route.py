"""Planar geometry for routes, and route-relative map pose and progress tracking."""
import math

import numpy as np

METERS_PER_DEGREE = 2 * np.pi * 6371008.8 / 360  # mean Earth radius, as Mapbox measures distances

# Training renders the map on the route rather than at the measured position:
# the ego is projected onto the route, the frame is centered there and headed along the route's tangent.
PROJECTION_WINDOW = (-30.0, 100.0)  # meters around the previous progress to search; a loop cannot capture the ego
HEADING_HALF_WINDOW = 50.0  # meters; the heading is the chord of the route this far behind and ahead


def local_meters(points, origin) -> np.ndarray:
  """Equirectangular east/north meters of (longitude, latitude) points around an origin."""
  origin = np.asarray(origin, dtype=np.float64)
  scale = np.array([METERS_PER_DEGREE * np.cos(np.radians(origin[1])), METERS_PER_DEGREE])
  return (np.asarray(points, dtype=np.float64) - origin) * scale


def nearest_segment(starts: np.ndarray, ends: np.ndarray, point) -> tuple[int, float, float]:
  """(index, fraction along it, distance) of the closest spot on the segments, in the inputs' units."""
  ab = ends - starts
  t = np.clip(((point - starts) * ab).sum(axis=1) / np.maximum((ab * ab).sum(axis=1), 1e-12), 0, 1)
  d = np.linalg.norm(starts + t[:, None] * ab - point, axis=1)
  i = int(d.argmin())
  return i, float(t[i]), float(d[i])


def cumulative_distance(route: np.ndarray) -> np.ndarray:
  """Meters along [(longitude, latitude), ...]."""
  (lon1, lat1), (lon2, lat2) = route[:-1].T, route[1:].T
  steps = np.hypot((lon2 - lon1) * np.cos(np.radians((lat1 + lat2) / 2)), lat2 - lat1) * METERS_PER_DEGREE
  return np.r_[0.0, np.cumsum(steps)]


def bearing(east: float, north: float) -> float:
  """Degrees clockwise from north of a local east/north direction."""
  return math.degrees(math.atan2(east, north)) % 360


def project_onto_path(path: np.ndarray, point, extend: bool = False) -> tuple[float, float, float]:
  """(meters along a (longitude, latitude) path to the closest spot on it, the point's distance from that spot, and the
  path's bearing there), measured on the nearest segment, so a point beside the path gets no progress for its offset.
  With extend, the first segment reaches back before the path's start and the last one past its end: a point behind the
  path reads below 0 and one past it beyond its length, each with its distance from that line. A path of one point has
  no direction: a point is its distance past it, with a NaN bearing."""
  local, p = local_meters(path, path[0]), local_meters(point, path[0])
  distance = cumulative_distance(path)
  keep = np.r_[True, np.diff(distance) > 1e-6]  # a zero length segment has no direction
  local, distance = local[keep], distance[keep]
  if len(local) < 2:
    offset = float(np.linalg.norm(p))
    return offset if extend else 0.0, offset, math.nan
  segment, fraction, lateral = nearest_segment(local[:-1], local[1:], p)
  start, direction = local[segment], local[segment + 1] - local[segment]
  if extend:
    t = float(np.dot(p - start, direction) / np.dot(direction, direction))
    if (segment == 0 and t < 0) or (segment == len(local) - 2 and t > 1):
      fraction, lateral = t, float(np.linalg.norm(start + t * direction - p))
  along = distance[segment] + fraction * (distance[segment + 1] - distance[segment])
  return float(along), lateral, bearing(*direction)


def point_along(path: np.ndarray, distance: float) -> np.ndarray:
  """The (longitude, latitude) spot a distance along a path, clipped to its ends."""
  cumulative = cumulative_distance(path)
  return np.array([np.interp(distance, cumulative, path[:, 0]), np.interp(distance, cumulative, path[:, 1])])


class RouteTracker:
  """Progress of the ego along a route, in meters, and the map pose at that progress."""

  def __init__(self, route: np.ndarray):
    self.route = np.asarray(route, dtype=np.float64).reshape((-1, 2))
    self.path = local_meters(self.route, self.route[0])
    self.distance = cumulative_distance(self.route)
    self.progress = -np.inf  # the first fix may land anywhere on the route

  def project(self, longitude: float, latitude: float) -> float:
    """Advance to the nearest spot on the route within the window ahead of the previous progress."""
    near = np.flatnonzero((self.distance[1:] >= self.progress + PROJECTION_WINDOW[0]) & (self.distance[:-1] <= self.progress + PROJECTION_WINDOW[1]))
    if not len(near):  # only before the first fix, when the progress is still -inf
      near = np.arange(len(self.path) - 1)
    point = local_meters([longitude, latitude], self.route[0])
    segment, along, _ = nearest_segment(self.path[near], self.path[near + 1], point)
    segment = near[segment]
    self.progress = max(self.progress, self.distance[segment] + along * (self.distance[segment + 1] - self.distance[segment]))
    return self.progress

  def pose(self, progress: float) -> tuple[float, float, float]:
    """(longitude, latitude, bearing) on the route at a progress, headed along the +-50m chord."""
    ends = np.clip([progress - HEADING_HALF_WINDOW, progress, progress + HEADING_HALF_WINDOW], 0, self.distance[-1])
    (lon0, lon, lon1), (lat0, lat, lat1) = (np.interp(ends, self.distance, self.route[:, axis]) for axis in (0, 1))
    bearing = np.degrees(np.arctan2((lon1 - lon0) * np.cos(np.radians(lat0)), lat1 - lat0)) % 360
    return float(lon), float(lat), float(bearing)

  def snap(self, longitude: float, latitude: float) -> tuple[float, float, float]:
    return self.pose(self.project(longitude, latitude))
