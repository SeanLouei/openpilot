import time

from openpilot.cereal import log

PHONE = log.GpsLocationData.SensorSource.iOS  # phonegpsd's fixes, from the iPhone, on gpsLocation
# s a phone fix may take to reach the comma, on the comma's clock, and still be used: phonegpsd drops one taken longer ago
PHONE_MAX_DELAY = 2.


def latest_gps_source(sm) -> str | None:
  """The service with the freshest usable GPS fix, or None. A usable fix from the phone wins while it lasts: the phone sends
  one a second and the u-blox ten, so the newest would switch between the two at every fix, and their positions differ by
  their errors. Once the phone's last fix is 2 s old, as when phonegpsd dropped the one after it, the u-blox's is taken
  until the phone's next."""
  now = time.monotonic()
  sources = [s for s in ('gpsLocation', 'gpsLocationExternal')
             if sm.valid[s] and sm[s].hasFix and 0 <= now - sm.recv_time[s] < 2]
  if 'gpsLocation' in sources and getattr(sm['gpsLocation'], 'source', None) == PHONE:
    return 'gpsLocation'
  return max(sources, key=lambda s: sm.logMonoTime[s]) if sources else None


def fix_delay(sm, service: str) -> float:
  """s from when the service's last fix was taken to when it was published, which its age counts from too. A receiver in
  the comma publishes its fix at once, 0; the phone's comes over the network, about a second later. phonegpsd sends a phone
  fix with its own time as unixTimestampMillis while the phone's clock agrees with the comma's, so its delay is the comma's
  clock as it arrived less that time, from 0 to PHONE_MAX_DELAY; else with the time it arrived, a delay of 0."""
  message = sm[service]
  if getattr(message, 'source', None) != PHONE:
    return 0.
  # phonegpsd publishes a fix as it arrives: the comma's clock then, from the clock's lead over the monotonic one now
  arrived = time.time() - (time.monotonic() - sm.logMonoTime[service] * 1e-9)  # noqa: TID251
  return min(max(arrived - message.unixTimestampMillis * 1e-3, 0.), PHONE_MAX_DELAY)
