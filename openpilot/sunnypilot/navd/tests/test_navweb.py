"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import http.client
import json
import threading
import time
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import requests

from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.navd.navweb import MAX_BODY, NavWeb, coordinates_from_text, geocode, make_handler, resolve_link


class FakeParams(dict):
  def get(self, key, *args, **kwargs):
    return super().get(key)

  def get_bool(self, key):
    return bool(super().get(key, False))

  def put(self, key, value):
    self[key] = value

  def remove(self, key):
    self.pop(key, None)


class FakeSubMaster(SimpleNamespace):
  def __getitem__(self, service):
    return self.data[service]


@contextmanager
def serving(navweb):
  server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(navweb))
  threading.Thread(target=server.serve_forever, daemon=True).start()
  try:
    yield server.server_address[1]
  finally:
    server.shutdown()
    server.server_close()


def request(port, method, path, body=None, content_type='application/json'):
  connection = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
  connection.request(method, path, body=body, headers={'Content-Type': content_type} if body is not None else {})
  response = connection.getresponse()
  return response.status, response.read()


class TestLinks(OpenpilotTestCase):
  @parameterized.expand([
    ('37.77490, -122.41940', (37.7749, -122.4194)),
    (' -33.8688,151.2093 ', (-33.8688, 151.2093)),
    # a Google place link carries the place, which wins over the map's viewport
    ('https://www.google.com/maps/place/Ferry+Building/@37.7952,-122.3960,17z/data=!3m1!4b1!4m6!3m5!1s0x0:0x0!8m2!3d37.7955!4d-122.3937',
     (37.7955, -122.3937)),
    ('https://www.google.com/maps/@37.7952,-122.3960,17z', (37.7952, -122.396)),
    ('https://maps.google.com/?q=37.7749,-122.4194', (37.7749, -122.4194)),
    ('https://www.google.com/maps/dir/?api=1&destination=37.7749,-122.4194', (37.7749, -122.4194)),
    ('https://maps.apple.com/?ll=37.3349,-122.0090&q=Apple%20Park', (37.3349, -122.009)),
    ('https://maps.apple.com/place?coordinate=37.3349,-122.0090&name=Apple%20Park', (37.3349, -122.009)),
  ])
  def test_coordinates(self, text, expected):
    assert coordinates_from_text(text) == expected

  @parameterized.expand([
    ('Unit 5, 12 Elm St',), ('1600 Amphitheatre Pkwy, Mountain View',), ('95.0, 10.0',), ('0.0, 0.0',),
    ('https://maps.apple.com/?q=Apple%20Park',), ('',),
  ])
  def test_not_coordinates(self, text):
    assert coordinates_from_text(text) is None

  def test_short_links(self):
    response = MagicMock(url='https://www.google.com/maps/place/X/data=!3d37.1!4d-122.1')
    response.__enter__.return_value = response
    with patch('requests.get', return_value=response) as get:
      assert resolve_link('https://maps.app.goo.gl/abc123') == response.url
      assert resolve_link('https://example.com/maps.app.goo.gl') == 'https://example.com/maps.app.goo.gl'  # other hosts are never fetched
      assert resolve_link('37.1, -122.1') == '37.1, -122.1'
    get.assert_called_once()

  def test_geocode(self):
    response = MagicMock()
    response.json.return_value = {'features': [
      {'geometry': {'coordinates': [-122.40, 37.79]},
       'properties': {'full_address': '1 Ferry Building, San Francisco', 'coordinates': {
         'latitude': 37.79, 'longitude': -122.40, 'routable_points': [{'name': 'default', 'latitude': 37.7951, 'longitude': -122.3941}]}}},
      {'geometry': {'coordinates': [-122.41, 37.78]}, 'properties': {'name': 'Market St'}},
    ]}
    with patch('requests.get', return_value=response) as get:
      places = geocode('ferry building', 'pk.test', (37.7, -122.4))
    assert get.call_args.kwargs['params']['proximity'] == '-122.4,37.7'
    assert places == [{'place_name': '1 Ferry Building, San Francisco', 'latitude': 37.7951, 'longitude': -122.3941},
                      {'place_name': 'Market St', 'latitude': 37.78, 'longitude': -122.41}]


class TestNavWeb(OpenpilotTestCase):
  def test_destination(self):
    params = FakeParams()
    navweb = NavWeb(params)
    status, _ = navweb.set_destination({'latitude': 37.7749, 'longitude': -122.4194, 'place_name': 'Home'})
    assert status == 200 and params['NavDestination'] == {'latitude': 37.7749, 'longitude': -122.4194, 'place_name': 'Home'}
    assert navweb.set_destination({'text': 'somewhere nice'})[0] == 400
    assert navweb.set_destination({'latitude': 91, 'longitude': 0})[0] == 400
    assert params['NavDestination']['place_name'] == 'Home'
    navweb.set_destination({'text': '37.1, -122.1'})
    assert (params['NavDestination']['latitude'], params['NavDestination']['longitude']) == (37.1, -122.1)
    navweb.clear_destination({})
    assert 'NavDestination' not in params

  def test_token(self):
    params = FakeParams()
    navweb = NavWeb(params)
    assert navweb.state()['tokenHint'] == ''
    assert navweb.search({'query': 'ferry building'})[0] == 400  # searching needs a token
    navweb.set_token({'token': ' pk.abcdefgh1234 '})
    assert params['MapboxToken'] == 'pk.abcdefgh1234'
    assert navweb.state()['tokenHint'] == '…1234'  # never the token itself
    navweb.set_token({'token': ''})
    assert 'MapboxToken' not in params

  def test_state(self):
    navweb = NavWeb(FakeParams(NavigationEnabled=True))
    services = ('gpsLocation', 'gpsLocationExternal', 'navInstruction')
    sm = FakeSubMaster(valid=dict.fromkeys(services, True), alive=dict.fromkeys(services, True), recv_time=dict.fromkeys(services, time.monotonic()),
                       logMonoTime={'gpsLocation': 2, 'gpsLocationExternal': 1, 'navInstruction': 1},
                       data={'gpsLocation': SimpleNamespace(hasFix=True, latitude=37., longitude=-122.),
                             'gpsLocationExternal': SimpleNamespace(hasFix=False),
                             'navInstruction': SimpleNamespace(maneuverPrimaryText='Turn right', maneuverDistance=120., distanceRemaining=900.,
                                                               timeRemaining=90.)})
    navweb.update(sm)
    state = navweb.state()
    assert state['navigationEnabled'] and state['gps'] and navweb.position == (37., -122.)
    assert state['instruction'] == {'text': 'Turn right', 'distance': 120., 'distanceRemaining': 900., 'timeRemaining': 90.}
    sm.valid['navInstruction'] = False  # no route
    navweb.update(sm)
    assert navweb.state()['instruction'] is None


class TestServer(OpenpilotTestCase):
  def test_requests(self):
    params = FakeParams()
    with serving(NavWeb(params)) as port:
      status, page = request(port, 'GET', '/')
      assert status == 200 and b'sunnypilot navigation' in page
      status, body = request(port, 'POST', '/api/destination', json.dumps({'text': '37.1, -122.1'}))
      assert status == 200 and json.loads(body)['destination']['latitude'] == 37.1
      status, body = request(port, 'GET', '/api/state')
      assert status == 200 and json.loads(body)['destination']['longitude'] == -122.1
      assert request(port, 'POST', '/api/missing', '{}')[0] == 404
      assert request(port, 'POST', '/api/destination', '[1, 2]')[0] == 400
      assert request(port, 'POST', '/api/destination', '{"latitude": "north"}')[0] == 400
      assert request(port, 'POST', '/api/clear', 'x' * (MAX_BODY + 1))[0] == 413

  def test_other_sites_cannot_post(self):
    # a form on another site can only send a simple content type, which never changes anything
    params = FakeParams()
    with serving(NavWeb(params)) as port:
      assert request(port, 'POST', '/api/token', 'token=pk.evil', content_type='application/x-www-form-urlencoded')[0] == 415
      assert request(port, 'POST', '/api/token', '{"token": "pk.evil"}', content_type='text/plain')[0] == 415
    assert 'MapboxToken' not in params

  def test_failed_search_keeps_token_out_of_logs(self):
    with serving(NavWeb(FakeParams(MapboxToken='pk.secret'))) as port, \
         patch('requests.get', side_effect=requests.ConnectionError('url: /forward?access_token=pk.secret')), \
         patch('openpilot.sunnypilot.navd.navweb.cloudlog') as cloudlog:
      status, body = request(port, 'POST', '/api/search', json.dumps({'query': 'ferry building'}))
    assert status == 502 and b'pk.secret' not in body
    assert 'pk.secret' not in repr(cloudlog.mock_calls)
