import time

from openpilot.cereal import log

PHONE = log.GpsLocationData.SensorSource.iOS  # phonegpsd's fixes, from the iPhone, on gpsLocation


def latest_gps_source(sm) -> str | None:
  """The service with the freshest usable GPS fix, or None. A usable fix from the phone wins while it lasts: the phone sends
  one a second and the u-blox ten, so the newest would switch between the two, and their positions differ by their errors."""
  now = time.monotonic()
  sources = [s for s in ('gpsLocation', 'gpsLocationExternal')
             if sm.valid[s] and sm[s].hasFix and 0 <= now - sm.recv_time[s] < 2]
  if 'gpsLocation' in sources and getattr(sm['gpsLocation'], 'source', None) == PHONE:
    return 'gpsLocation'
  return max(sources, key=lambda s: sm.logMonoTime[s]) if sources else None
