#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A page on the device's network for setting the navigation destination and Mapbox token from a phone.
"""
import ipaddress
import json
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import NamedTuple
from urllib.parse import SplitResult, parse_qs, unquote_plus, urljoin, urlsplit

import requests

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source

PORT = 8082
MAX_BODY = 4096
GEOCODE_URL = "https://api.mapbox.com/search/geocode/v6/forward"
SHORT_LINK_HOSTS = ('maps.app.goo.gl', 'maps.apple')  # shared map links that redirect to one with the place in it
MAX_REDIRECTS = 5

LINK = re.compile(r'https?://\S+')
COORDINATES = re.compile(r'^\s*(-?\d{1,2}\.\d+)\s*,\s*(-?\d{1,3}\.\d+)\s*$')
GOOGLE_PLACE = re.compile(r'!3d(-?\d{1,2}\.\d+)!4d(-?\d{1,3}\.\d+)')  # the place itself, not the map's viewport
GOOGLE_STOP = re.compile(r'!1d(-?\d{1,3}\.\d+)!2d(-?\d{1,2}\.\d+)')  # a directions stop, longitude first
GOOGLE_VIEWPORT = re.compile(r'@(-?\d{1,2}\.\d+),(-?\d{1,3}\.\d+)')
COORDINATE_KEYS = ('destination', 'daddr', 'q', 'query', 'coordinate', 'll')  # best first: Google's ll is the map's center
TEXT_KEYS = ('destination', 'daddr', 'q', 'query', 'address')

OFFLINE = "The device is offline: connect it to the internet and try again"
LINK_FAILED = "Couldn't open that link: copy it from your maps app again, or paste the place's coordinates"
MAPBOX_ERRORS = {
  401: "The Mapbox token is missing or invalid: save a valid one below",
  403: "The Mapbox token is missing or invalid: save a valid one below",
  422: "Mapbox couldn't read that address: shorten it and try again",
  429: "Too many searches: try again shortly",
}
MAPBOX_FAILED = "Mapbox isn't answering right now: try again shortly"


class Place(NamedTuple):
  coordinates: tuple[float, float] | None
  name: str = ''  # what the link calls the place, to show
  query: str = ''  # what to look up when the link has no coordinates


class LinkError(Exception):
  """A share link that didn't lead to a map link."""


def valid_coordinates(latitude: float, longitude: float) -> bool:
  return -90 <= latitude <= 90 and -180 <= longitude <= 180 and (latitude, longitude) != (0, 0)


def coordinates_in(text: str) -> tuple[float, float] | None:
  """(latitude, longitude) from 'lat, lon', else None."""
  match = COORDINATES.match(text)
  if match and valid_coordinates(float(match[1]), float(match[2])):
    return float(match[1]), float(match[2])
  return None


def short_link(url: str) -> bool:
  """Whether url is a share link to expand: https on exactly a short-link host, with nothing that URL parsers read differently."""
  # requests ends the host at a backslash and urlsplit doesn't, so https://evil.example\@maps.app.goo.gl/x would pass a
  # hostname check here and then connect to evil.example. Userinfo, a port or whitespace can hide a host too.
  if not (url.isascii() and url.isprintable()) or any(c in ' \\@' for c in url):
    return False
  scheme, _, rest = url.partition('://')
  return scheme == 'https' and re.split('[/?#]', rest, maxsplit=1)[0] in SHORT_LINK_HOSTS


def resolve_link(url: str) -> str:
  """The map link behind a share link, which only carries an ID, else url itself. Only short-link hosts are ever fetched."""
  for _ in range(MAX_REDIRECTS):
    if not short_link(url):
      return url
    # one redirect at a time, so that each URL is checked before it's fetched
    with requests.get(url, allow_redirects=False, timeout=5, stream=True) as response:
      location = response.headers.get('Location') if response.is_redirect else None
    if not location:
      raise LinkError("the share link didn't redirect")
    url = urljoin(url, location)
  if short_link(url):
    raise LinkError("too many redirects")
  return url


def interstitial(url: SplitResult) -> bool:
  """Whether url is Google's cookie consent or unusual traffic page, which has the page it's in front of as continue=."""
  host = url.hostname or ''
  return host.startswith('consent.google.') or (url.path.startswith('/sorry') and re.match(r'(www\.)?google\.', host) is not None)


def read_link(link: str) -> Place:
  """The place a Google or Apple Maps link points to: its coordinates when the link has them, and its name or address."""
  url = urlsplit(link)
  if interstitial(url):  # the link it continues to, never the page itself
    url = urlsplit(parse_qs(url.query).get('continue', [''])[0])
    if interstitial(url):
      return Place(None)
  query = {key: values[0].strip() for key, values in parse_qs(url.query).items()}
  segments = [unquote_plus(segment).strip() for segment in url.path.split('/')]
  path = '/'.join(segments)

  # Google: /maps/place/<name>, /maps/search/<text>, /maps/dir/<stop>/.../<stop> or /maps/@<viewport>, then the viewport and data
  after = segments[segments.index('maps') + 1:] if 'maps' in segments else []
  kind = after[0] if after else ''
  stops = [s for s in after[1:] if s and not s.startswith('@') and '=' not in s]
  if kind == 'dir':
    stop = stops[-1] if stops else ''  # the destination: the first stop is where the directions start
  else:
    stop = stops[0] if stops and kind in ('place', 'search') else ''

  pairs = COORDINATES.findall(stop)
  if kind == 'dir':
    pairs += [(latitude, longitude) for longitude, latitude in GOOGLE_STOP.findall(path)[-1:]]
  pairs += GOOGLE_PLACE.findall(path)[-1:]
  for key in COORDINATE_KEYS:
    pairs += COORDINATES.findall(query.get(key, ''))
  if kind == 'place' or kind.startswith('@'):  # the map's center: the place on a place link, and all a link to the map has
    pairs += GOOGLE_VIEWPORT.findall(path)[:1]
  candidates = [(float(latitude), float(longitude)) for latitude, longitude in pairs]
  coordinates = next((c for c in candidates if valid_coordinates(*c)), None)

  texts = [text for text in (stop, *(query.get(key, '') for key in TEXT_KEYS)) if text and not COORDINATES.match(text)]
  text = texts[0] if texts else ''
  return Place(coordinates, ' '.join((query.get('name') or text).split())[:200], text)


def place_from_text(text: str) -> Place:
  """The place in pasted text: from the first map link in it, expanding a share link (which fetches it), else from 'lat, lon'."""
  link = LINK.search(text)
  return read_link(resolve_link(link[0])) if link else Place(coordinates_in(text))


def search_text(text: str) -> str:
  """text as Mapbox takes it: one line without semicolons, of at most 20 words and numbers and 256 characters."""
  text = ' '.join(text.replace(';', ' ').split())
  words = list(re.finditer(r'\w+', text))
  if len(words) > 20:
    text = text[:words[19].end()]
  return text[:256].rstrip()


def geocode(query: str, token: str, near: tuple[float, float] | None) -> list[dict]:
  """Places matching an address search, nearest first when the position is known."""
  query = search_text(query)
  if not query:
    return []
  options = {'q': query, 'access_token': token, 'limit': 5, 'autocomplete': 'false'}
  if near is not None:
    options['proximity'] = f'{near[1]},{near[0]}'
  response = requests.get(GEOCODE_URL, params=options, timeout=10)
  response.raise_for_status()
  places = []
  for feature in response.json().get('features', []):
    properties = feature.get('properties') or {}
    coordinates = properties.get('coordinates') or {}
    # a routable point is the spot on the road for the place, not the middle of its building
    point = next(iter(coordinates.get('routable_points') or []), coordinates)
    if 'latitude' not in point:
      longitude, latitude = feature['geometry']['coordinates']
      point = {'latitude': latitude, 'longitude': longitude}
    places.append({'place_name': properties.get('full_address') or properties.get('name') or query,
                   'latitude': float(point['latitude']), 'longitude': float(point['longitude'])})
  return places


def mapbox_error(e: requests.RequestException) -> str:
  """What to tell the user about a failed Mapbox request: never the error itself, whose URL has the access token."""
  if isinstance(e, requests.ConnectionError):
    return OFFLINE
  if e.response is not None and e.response.status_code in MAPBOX_ERRORS:
    return MAPBOX_ERRORS[e.response.status_code]
  return MAPBOX_FAILED


def last_gps_position(params: Params) -> tuple[float, float] | None:
  """(latitude, longitude) of the last GPS fix saved, else None."""
  data = params.get("LastGPSPosition")
  if not data:
    return None
  try:
    if data.startswith(b'{'):
      # the JSON that osm_map_data writes for mapd to the memory params, which on macOS are these
      position = json.loads(data)
      latitude, longitude = float(position['latitude']), float(position['longitude'])
    else:
      # the gpsLocationExternal event ubloxd saves about once a minute with a fix
      with log.Event.from_bytes(data) as event:
        latitude, longitude = event.gpsLocationExternal.latitude, event.gpsLocationExternal.longitude
  except Exception:  # unreadable: searches just aren't biased
    return None
  return (latitude, longitude) if valid_coordinates(latitude, longitude) else None


def device_host(host: str) -> bool:
  """Whether a Host header names the device by an IP address or as localhost, which a rebound DNS name never does."""
  name, _, port = host.rpartition(':')
  if not (port.isascii() and port.isdigit()):  # no port to strip
    name = host
  if name.startswith('[') and name.endswith(']'):  # an IPv6 address
    name = name[1:-1]
  if name.lower() == 'localhost':
    return True
  try:
    ipaddress.ip_address(name)
  except ValueError:
    return False
  return True


class NavWeb:
  def __init__(self, params: Params | None = None):
    self.params = params if params is not None else Params()
    self.lock = threading.Lock()
    self.position: tuple[float, float] | None = None  # (latitude, longitude)
    self.instruction: dict | None = None

  def update(self, sm) -> None:
    source = latest_gps_source(sm)
    instruction = sm['navInstruction'] if sm.alive['navInstruction'] and sm.valid['navInstruction'] else None
    with self.lock:
      self.position = (sm[source].latitude, sm[source].longitude) if source is not None else None
      self.instruction = None if instruction is None else {
        'text': instruction.maneuverPrimaryText, 'distance': instruction.maneuverDistance,
        'distanceRemaining': instruction.distanceRemaining, 'timeRemaining': instruction.timeRemaining,
      }

  def token(self) -> str:
    return (self.params.get("MapboxToken") or '').strip()

  def near(self) -> tuple[float, float] | None:
    """Where to search near: the live GPS fix, else the last one saved, so that searches offroad are local too."""
    with self.lock:
      position = self.position
    return position if position is not None else last_gps_position(self.params)

  def state(self) -> dict:
    token = self.token()
    with self.lock:
      return {'destination': self.params.get("NavDestination"), 'navigationEnabled': self.params.get_bool("NavigationEnabled"),
              'isMetric': self.params.get_bool("IsMetric"), 'tokenHint': f'…{token[-4:]}' if token else '',
              'gps': self.position is not None, 'instruction': self.instruction}

  def search(self, body: dict) -> tuple[int, dict]:
    query = str(body.get('query', '')).strip()
    if not query:
      return 400, {'error': 'Type an address to search for'}
    if not self.token():
      return 400, {'error': 'Add a Mapbox token to search, or paste a map link or coordinates instead'}
    return 200, {'places': geocode(query, self.token(), self.near())}

  def look_up(self, text: str) -> tuple[int, dict]:
    """Places to pick from for a link that names its place without coordinates. The destination stays as it is."""
    token = self.token()
    if not token:
      return 400, {'error': 'A Mapbox token is needed to look up this link: save one below, or paste the coordinates instead'}
    near = self.near()
    places = geocode(text, token, near)
    address = text.partition(',')[2].strip()
    if not places and address:
      # Mapbox has addresses but not businesses, so "Costco Wholesale, 1600 El Camino Real, ..." is found by its address
      places = geocode(address, token, near)
    return 200, {'places': places}

  def set_destination(self, body: dict) -> tuple[int, dict]:
    if 'text' in body:
      try:
        place = place_from_text(str(body['text']))
      except (requests.RequestException, LinkError) as e:
        cloudlog.error(f"navweb: could not open a link: {type(e).__name__}")
        return 502, {'error': OFFLINE if isinstance(e, requests.ConnectionError) else LINK_FAILED}
      if place.coordinates is None:
        if place.query:
          return self.look_up(place.query)
        return 400, {'error': 'No coordinates found: paste "latitude, longitude" or a Google or Apple Maps link'}
      latitude, longitude = place.coordinates
      place_name = place.name or f'{latitude:.5f}, {longitude:.5f}'
    else:
      latitude, longitude, place_name = float(body['latitude']), float(body['longitude']), str(body.get('place_name', ''))[:200]
    if not valid_coordinates(latitude, longitude):
      return 400, {'error': 'Invalid coordinates'}
    self.params.put("NavDestination", {'latitude': latitude, 'longitude': longitude, 'place_name': place_name}, block=True)
    return 200, self.state()

  def clear_destination(self, _body: dict) -> tuple[int, dict]:
    self.params.remove("NavDestination")
    return 200, self.state()

  def set_token(self, body: dict) -> tuple[int, dict]:
    token = body.get('token')
    if not isinstance(token, str):  # a request without a token never removes the saved one
      return 400, {'error': 'Missing token'}
    if token.strip():
      self.params.put("MapboxToken", token.strip(), block=True)
    else:
      self.params.remove("MapboxToken")
    return 200, self.state()


def make_handler(navweb: NavWeb):
  routes = {'/api/search': navweb.search, '/api/destination': navweb.set_destination,
            '/api/clear': navweb.clear_destination, '/api/token': navweb.set_token}

  class Handler(BaseHTTPRequestHandler):
    timeout = 10  # seconds a read or write may stall, so that a stalled client can't hold its thread forever

    def send(self, status: int, body: bytes, content_type: str) -> None:
      self.send_response(status)
      self.send_header('Content-Type', content_type)
      self.send_header('Content-Length', str(len(body)))
      self.send_header('Cache-Control', 'no-store')
      self.end_headers()
      self.wfile.write(body)

    def send_json(self, status: int, payload: dict) -> None:
      self.send(status, json.dumps(payload).encode(), 'application/json')

    def refusal(self) -> tuple[int, dict] | None:
      """The answer for a request to refuse before reading it, else None."""
      host = self.headers.get('Host', '')
      # a page on another site can rebind its DNS name to this device to read and post as the same origin, but the
      # browser still sends that name as the Host
      if not device_host(host):
        return 421, {'error': "Open this page at the device's IP address"}
      origin = self.headers.get('Origin')
      if origin is not None and origin.partition('://')[2].lower() != host.lower():
        return 403, {'error': 'Requests from other sites are not allowed'}
      if 'Transfer-Encoding' in self.headers:  # the body is never decoded from chunks here, so it would be misread
        return 501, {'error': 'Chunked requests are not supported'}
      return None

    def do_GET(self):
      if (refusal := self.refusal()) is not None:
        return self.send_json(*refusal)
      if self.path == '/':
        self.send(200, PAGE.encode(), 'text/html; charset=utf-8')
      elif self.path == '/api/state':
        self.send_json(200, navweb.state())
      else:
        self.send_json(404, {'error': 'Not found'})

    def do_POST(self):
      if (refusal := self.refusal()) is not None:
        return self.send_json(*refusal)
      # Requiring JSON makes browsers preflight requests from other sites, which this server never allows
      if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
        return self.send_json(415, {'error': 'Expected JSON'})
      length = self.headers.get('Content-Length')
      if length is None:
        return self.send_json(411, {'error': 'Content-Length required'})
      if not (length.isascii() and length.isdigit()):  # isdigit() alone passes digits like '²', which int() rejects
        return self.send_json(400, {'error': 'Invalid Content-Length'})
      if len(length) > 9 or int(length) > MAX_BODY:  # int() also rejects thousands of digits
        return self.send_json(413, {'error': 'Request too large'})
      if self.path not in routes:
        return self.send_json(404, {'error': 'Not found'})
      try:
        body = json.loads(self.rfile.read(int(length)) or b'{}')
        status, payload = routes[self.path](body if isinstance(body, dict) else {})
      except requests.RequestException as e:
        # requests exceptions contain the URL, and with it the access token; keep it out of logs
        cloudlog.error(f"navweb: request failed: {type(e).__name__}")
        status, payload = 502, {'error': mapbox_error(e)}
      except (ValueError, KeyError, TypeError):
        status, payload = 400, {'error': 'Invalid request'}
      self.send_json(status, payload)

    def log_message(self, format, *args):  # noqa: A002  # stdlib override
      pass

  return Handler


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>sunnypilot navigation</title>
<style>
  :root { color-scheme: dark; --bg: #0b1014; --card: #151c22; --line: #26313a; --text: #e8eef2; --muted: #93a1ad; --accent: #48eb84; --danger: #ff6b6b; }
  * { box-sizing: border-box; }
  body { margin: 0; padding: 16px; background: var(--bg); color: var(--text); font: 16px/1.4 -apple-system, system-ui, sans-serif; }
  main { max-width: 560px; margin: 0 auto; display: grid; gap: 14px; }
  h1 { font-size: 20px; margin: 4px 0; }
  section { background: var(--card); border: 1px solid var(--line); border-radius: 14px; padding: 14px; display: grid; gap: 10px; }
  h2 { font-size: 13px; text-transform: uppercase; letter-spacing: .06em; color: var(--muted); margin: 0; }
  .row { display: flex; gap: 8px; }
  input { flex: 1; min-width: 0; padding: 12px; border-radius: 10px; border: 1px solid var(--line); font-size: 16px;
          background: var(--bg); color: var(--text); }
  button { padding: 12px 14px; border-radius: 10px; border: 0; background: var(--accent); color: #04140a; font-weight: 600; font-size: 15px; }
  button.secondary { background: var(--line); color: var(--text); }
  ul { list-style: none; margin: 0; padding: 0; display: grid; gap: 6px; }
  li button { width: 100%; text-align: left; background: var(--bg); color: var(--text); border: 1px solid var(--line); font-weight: 500; }
  .muted { color: var(--muted); font-size: 14px; }
  .error { position: fixed; left: 16px; right: 16px; bottom: 16px; max-width: 528px; margin: 0 auto; padding: 12px 14px; border-radius: 10px;
           background: #3a1416; border: 1px solid var(--danger); color: #ffd6d6; font-size: 14px; }
  .error:empty { display: none; }
  .value { font-size: 18px; font-weight: 600; word-break: break-word; }
</style>
</head>
<body>
<main>
  <h1>sunnypilot navigation</h1>
  <section>
    <h2>Destination</h2>
    <div class="value" id="destination">None</div>
    <div class="muted" id="status"></div>
    <div class="row"><button class="secondary" id="clear">Clear destination</button></div>
  </section>
  <section>
    <h2>Search an address</h2>
    <form class="row" id="search-form"><input id="query" placeholder="123 Main St, City" autocomplete="off"><button>Search</button></form>
    <ul id="results"></ul>
  </section>
  <section>
    <h2>Or paste a map link or coordinates</h2>
    <form class="row" id="link-form"><input id="link" placeholder="Google/Apple Maps link, or 37.77, -122.42" autocomplete="off"><button>Go</button></form>
    <div class="muted">For businesses and other places: share the place from your maps app, copy the link, and paste it here.</div>
  </section>
  <section>
    <h2>Mapbox token</h2>
    <div class="muted" id="token-state"></div>
    <form class="row" id="token-form"><input id="token" placeholder="pk.…" autocomplete="off" autocapitalize="off"><button>Save</button></form>
  </section>
  <div class="error" id="error"></div>
</main>
<script>
const $ = (id) => document.getElementById(id);
let errorTimer;
const showError = (message) => {
  $('error').textContent = message || '';
  clearTimeout(errorTimer);
  if (message) errorTimer = setTimeout(() => { $('error').textContent = ''; }, 6000);
};

async function call(path, body) {
  const response = await fetch(path, body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || 'Request failed');
  return payload;
}

let metric = false;
function formatDistance(meters) {
  if (metric) return meters >= 1000 ? `${(meters / 1000).toFixed(1)} km` : `${Math.round(meters / 10) * 10} m`;
  const miles = meters / 1609.34;
  return miles >= 0.2 ? `${miles.toFixed(1)} mi` : `${Math.round(meters * 3.28084 / 50) * 50} ft`;
}

function render(state) {
  metric = state.isMetric;
  const destination = state.destination;
  $('destination').textContent = destination ? (destination.place_name || `${destination.latitude.toFixed(5)}, ${destination.longitude.toFixed(5)}`) : 'None';
  const parts = [];
  if (!state.navigationEnabled) parts.push('Navigation is off: turn on "navigation (alpha)" in the device\\'s developer settings.');
  parts.push(state.gps ? 'GPS fix: yes' : 'GPS fix: waiting (the car must be on)');
  if (state.instruction) {
    const i = state.instruction;
    parts.push(`Next: ${i.text || 'continue'} in ${formatDistance(Math.max(0, i.distance))} · ${Math.max(1, Math.round(i.timeRemaining / 60))} min to go`);
  } else if (destination) {
    parts.push('Route: waiting (needs GPS, internet, and a token or comma routing)');
  }
  $('status').textContent = parts.join(' · ');
  $('token-state').textContent = state.tokenHint
    ? `Saved (${state.tokenHint}). Paste a new one to replace it, or save empty to remove it.`
    : 'Not set. Create a free access token at mapbox.com.';
}

async function refresh() {
  try { render(await call('/api/state')); } catch (e) { showError(e.message); }
}

function showPlaces(places) {
  showError(places.length ? '' : 'No matches');
  $('results').replaceChildren(...places.map((place) => {
    const item = document.createElement('li');
    const button = document.createElement('button');
    button.textContent = place.place_name;
    button.addEventListener('click', () => setDestination(place));
    item.append(button);
    return item;
  }));
}

async function setDestination(body) {
  try {
    const payload = await call('/api/destination', body);
    // a link that names its place without coordinates gives places to pick from, like a search
    if (payload.places) return showPlaces(payload.places);
    render(payload);
    $('results').replaceChildren();
    $('link').value = '';
    showError('');
  } catch (e) { showError(e.message); }
}

$('search-form').addEventListener('submit', async (event) => {
  event.preventDefault();
  try { showPlaces((await call('/api/search', {query: $('query').value})).places); } catch (e) { showError(e.message); }
});

$('link-form').addEventListener('submit', (event) => { event.preventDefault(); setDestination({text: $('link').value}); });
$('clear').addEventListener('click', async () => { try { render(await call('/api/clear', {})); } catch (e) { showError(e.message); } });
$('token-form').addEventListener('submit', async (event) => {
  event.preventDefault();
  try { render(await call('/api/token', {token: $('token').value})); $('token').value = ''; showError(''); } catch (e) { showError(e.message); }
});

refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
"""


def main():
  navweb = NavWeb()
  server = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(navweb))
  threading.Thread(target=server.serve_forever, daemon=True).start()
  cloudlog.warning(f"navweb serving on port {PORT}")

  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal', 'navInstruction'])
  while True:
    sm.update(1000)
    navweb.update(sm)


if __name__ == "__main__":
  main()
