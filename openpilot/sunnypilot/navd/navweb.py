#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A page on the device's network for setting the navigation destination and Mapbox token from a phone.
"""
import json
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source

PORT = 8082
MAX_BODY = 4096
GEOCODE_URL = "https://api.mapbox.com/search/geocode/v6/forward"
SHORT_LINK_HOSTS = ('maps.app.goo.gl', 'goo.gl', 'maps.apple')  # shared map links that redirect to one with coordinates

COORDINATES = re.compile(r'^\s*(-?\d{1,2}\.\d+)\s*,\s*(-?\d{1,3}\.\d+)\s*$')
GOOGLE_PLACE = re.compile(r'!3d(-?\d{1,2}\.\d+)!4d(-?\d{1,3}\.\d+)')  # the place itself, not the map's viewport
GOOGLE_VIEWPORT = re.compile(r'@(-?\d{1,2}\.\d+),(-?\d{1,3}\.\d+)')
LINK_QUERY_KEYS = ('ll', 'coordinate', 'q', 'query', 'destination', 'daddr')


def valid_coordinates(latitude: float, longitude: float) -> bool:
  return -90 <= latitude <= 90 and -180 <= longitude <= 180 and (latitude, longitude) != (0, 0)


def coordinates_from_text(text: str) -> tuple[float, float] | None:
  """(latitude, longitude) from 'lat, lon' or a Google or Apple Maps link, else None."""
  text = text.strip()
  url = urlparse(text)
  if url.scheme in ('http', 'https'):
    query = parse_qs(url.query)
    candidates = [v for key in LINK_QUERY_KEYS for v in query.get(key, [])]
    matches = [GOOGLE_PLACE.search(text), *(COORDINATES.match(c) for c in candidates), GOOGLE_VIEWPORT.search(unquote(url.path))]
  else:
    matches = [COORDINATES.match(text)]
  for match in matches:
    if match and valid_coordinates(float(match[1]), float(match[2])):
      return float(match[1]), float(match[2])
  return None


def resolve_link(text: str) -> str:
  """The full map link behind a shortened one, which only carries an ID."""
  url = urlparse(text.strip())
  if url.scheme == 'https' and url.hostname in SHORT_LINK_HOSTS:
    with requests.get(url.geturl(), allow_redirects=True, timeout=5, stream=True) as response:
      return response.url
  return text


def geocode(query: str, token: str, near: tuple[float, float] | None) -> list[dict]:
  """Places matching an address search, nearest first when the position is known."""
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
    with self.lock:
      near = self.position
    return 200, {'places': geocode(query, self.token(), near)}

  def set_destination(self, body: dict) -> tuple[int, dict]:
    if 'text' in body:
      coordinates = coordinates_from_text(resolve_link(str(body['text'])))
      if coordinates is None:
        return 400, {'error': 'No coordinates found: paste "latitude, longitude" or a Google or Apple Maps link'}
      latitude, longitude, place_name = *coordinates, f'{coordinates[0]:.5f}, {coordinates[1]:.5f}'
    else:
      latitude, longitude, place_name = float(body['latitude']), float(body['longitude']), str(body.get('place_name', ''))[:200]
    if not valid_coordinates(latitude, longitude):
      return 400, {'error': 'Invalid coordinates'}
    self.params.put("NavDestination", {'latitude': latitude, 'longitude': longitude, 'place_name': place_name})
    return 200, self.state()

  def clear_destination(self, _body: dict) -> tuple[int, dict]:
    self.params.remove("NavDestination")
    return 200, self.state()

  def set_token(self, body: dict) -> tuple[int, dict]:
    token = str(body.get('token', '')).strip()
    if token:
      self.params.put("MapboxToken", token)
    else:
      self.params.remove("MapboxToken")
    return 200, self.state()


def make_handler(navweb: NavWeb):
  routes = {'/api/search': navweb.search, '/api/destination': navweb.set_destination,
            '/api/clear': navweb.clear_destination, '/api/token': navweb.set_token}

  class Handler(BaseHTTPRequestHandler):
    def send(self, status: int, body: bytes, content_type: str) -> None:
      self.send_response(status)
      self.send_header('Content-Type', content_type)
      self.send_header('Content-Length', str(len(body)))
      self.send_header('Cache-Control', 'no-store')
      self.end_headers()
      self.wfile.write(body)

    def send_json(self, status: int, payload: dict) -> None:
      self.send(status, json.dumps(payload).encode(), 'application/json')

    def do_GET(self):
      if self.path == '/':
        self.send(200, PAGE.encode(), 'text/html; charset=utf-8')
      elif self.path == '/api/state':
        self.send_json(200, navweb.state())
      else:
        self.send_json(404, {'error': 'Not found'})

    def do_POST(self):
      # Requiring JSON makes browsers preflight requests from other sites, which this server never allows
      if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
        return self.send_json(415, {'error': 'Expected JSON'})
      length = self.headers.get('Content-Length') or '0'
      if not length.isdigit() or int(length) > MAX_BODY:
        return self.send_json(413, {'error': 'Request too large'})
      length = int(length)
      if self.path not in routes:
        return self.send_json(404, {'error': 'Not found'})
      try:
        body = json.loads(self.rfile.read(length) or b'{}')
        status, payload = routes[self.path](body if isinstance(body, dict) else {})
      except requests.RequestException as e:
        # requests exceptions contain the URL, and with it the access token; keep it out of logs
        cloudlog.error(f"navweb: request failed: {type(e).__name__}")
        status, payload = 502, {'error': 'Could not reach the map service. Is the device online, and is the token valid?'}
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

async function setDestination(body) {
  try {
    render(await call('/api/destination', body));
    $('results').replaceChildren();
    $('link').value = '';
    showError('');
  } catch (e) { showError(e.message); }
}

$('search-form').addEventListener('submit', async (event) => {
  event.preventDefault();
  try {
    const {places} = await call('/api/search', {query: $('query').value});
    showError(places.length ? '' : 'No matches');
    $('results').replaceChildren(...places.map((place) => {
      const item = document.createElement('li');
      const button = document.createElement('button');
      button.textContent = place.place_name;
      button.addEventListener('click', () => setDestination(place));
      item.append(button);
      return item;
    }));
  } catch (e) { showError(e.message); }
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
