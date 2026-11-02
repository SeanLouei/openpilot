"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import http.client
import json
import select
import socket
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from urllib.parse import urljoin

import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.parameterized import parameterized
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.navd.navweb import (LINK_FAILED, MAPBOX_ERRORS, MAPBOX_FAILED, MAX_BODY, MAX_CONNECTIONS, MAX_HEADERS, MAX_REDIRECTS, OFFLINE,
                                              PORT, LinkError, NavWeb, Place, Server, geocode, main, make_handler, place_from_text, read_link,
                                              resolve_link, search_text)

COSTCO = 'Costco Wholesale, 1600 El Camino Real, South San Francisco, CA 94080'
COSTCO_ADDRESS = '1600 El Camino Real, South San Francisco, CA 94080'
COSTCO_ENCODED = 'Costco+Wholesale,+1600+El+Camino+Real,+South+San+Francisco,+CA+94080'
# what share links from the Google Maps apps expand to: a business by name, without coordinates
IPHONE_LINK = f'https://maps.google.com/?q={COSTCO_ENCODED}&ftid=0x808f79b3cbb5e0e1:0x1f2f19f4ba6c0b8d&entry=gps'
ANDROID_LINK = f'https://www.google.com/maps/place/{COSTCO_ENCODED}/data=!4m2!3m1!1s0x808f79b3cbb5e0e1:0x1f2f19f4ba6c0b8d?utm_source=mstt_1&entry=gps'
CONSENT_LINK = 'https://consent.google.com/ml?continue=https://www.google.com/maps/place/X/data%3D!3d37.1!4d-122.1&gl=SE&hl=sv'
APPLE_PLACE_LINK = 'https://maps.apple.com/place?address=1600+El+Camino+Real,+South+San+Francisco,+CA&coordinate=37.6559,-122.4077&name=Costco'


class FakeParams(dict):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.blocked: dict[str, bool] = {}  # key: whether its last write waited until it was saved

  def get(self, key, *args, **kwargs):
    return super().get(key)

  def get_bool(self, key):
    return bool(super().get(key, False))

  def put(self, key, value, block=False):
    self[key] = value
    self.blocked[key] = block

  def remove(self, key):
    self.pop(key, None)


class FakeSubMaster(SimpleNamespace):
  def __getitem__(self, service):
    return self.data[service]


def answer(status, location=None):
  """A short-link host's response: a redirect to location, or else a page."""
  response = MagicMock(status_code=status, is_redirect=location is not None, headers={} if location is None else {'Location': location})
  response.__enter__.return_value = response
  return response


def mapbox(*places):
  """A Mapbox geocoding response with these (address, latitude, longitude) places."""
  response = MagicMock()
  response.json.return_value = {'features': [{'geometry': {'coordinates': [longitude, latitude]}, 'properties': {'full_address': address}}
                                             for address, latitude, longitude in places]}
  return response


def mapbox_failure(status):
  response = requests.Response()
  response.status_code = status
  return response


def proximity(navweb):
  """The position a search is biased to, as Mapbox gets it."""
  with patch('requests.get', return_value=mapbox()) as get:
    navweb.search({'query': 'costco'})
  return get.call_args.kwargs['params'].get('proximity')


@contextmanager
def serving(navweb, **attributes):
  server = Server(('127.0.0.1', 0), type('Handler', (make_handler(navweb),), attributes))
  threading.Thread(target=server.serve_forever, daemon=True).start()
  try:
    yield server.server_address[1]
  finally:
    server.shutdown()
    server.server_close()


def request(port, method, path, body=None, content_type='application/json', headers=None):
  connection = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
  connection.request(method, path, body=body, headers={**({'Content-Type': content_type} if body is not None else {}), **(headers or {})})
  response = connection.getresponse()
  return response.status, response.read()


def answered(port):
  """Whether a request for the state gets an answer, rather than its connection closed at once."""
  try:
    return request(port, 'GET', '/api/state')[0] == 200
  except ConnectionError:
    return False


def eventually(condition, timeout=2.):
  """Whether condition() comes true within timeout seconds."""
  end = time.monotonic() + timeout
  while not condition():
    if time.monotonic() > end:
      return False
    time.sleep(0.01)
  return True


def trickle(client, limit=5.):
  """Sends a byte every 0.1 s until the server closes the connection, or for limit seconds."""
  start = time.monotonic()
  try:
    while not select.select([client], [], [], 0.1)[0] and time.monotonic() - start < limit:
      client.sendall(b'a')
  except OSError:  # reset by the server
    pass


def raw_request(port, method, path, headers, body=None):
  """A request with only these headers, where http.client's request() would add Host and framing."""
  connection = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
  connection.putrequest(method, path, skip_host=True, skip_accept_encoding=True)
  for name, value in headers.items():
    connection.putheader(name, value)
  connection.endheaders(body)
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
    assert place_from_text(text).coordinates == expected

  @parameterized.expand([
    ('Unit 5, 12 Elm St',), ('1600 Amphitheatre Pkwy, Mountain View',), ('95.0, 10.0',), ('0.0, 0.0',),
    ('https://maps.apple.com/?q=Apple%20Park',), ('',),
  ])
  def test_not_coordinates(self, text):
    assert place_from_text(text).coordinates is None

  @parameterized.expand([
    # a dropped pin, from the Google Maps app
    ('https://www.google.com/maps/search/37.6559,+-122.4077?entry=tts', (37.6559, -122.4077), ''),
    ('https://www.google.com/maps/place/37.6559,-122.4077/@37.6,-122.4,17z', (37.6559, -122.4077), ''),
    ('https://www.google.com/maps/place/Ferry+Building/@37.7952,-122.3960,17z/data=!3m1!4b1!4m6!3m5!1s0x0:0x0!8m2!3d37.7955!4d-122.3937',
     (37.7955, -122.3937), 'Ferry Building'),
    # a place link without the place's data is centered on it
    ('https://www.google.com/maps/place/Costco/@37.6559,-122.4077,17z', (37.6559, -122.4077), 'Costco'),
    # directions go to their last stop: never the first, where they start, nor the map's viewport
    ('https://www.google.com/maps/dir/37.70,-122.40/Some+Place/37.6559,-122.4077/@37.68,-122.41,13z', (37.6559, -122.4077), ''),
    ('https://www.google.com/maps/dir/Home/Costco/@37.6,-122.4,12z/data=!4m8!4m7!1m2!2m2!1d-122.3!2d37.5!1m2!2m2!1d-122.4077!2d37.6559',
     (37.6559, -122.4077), 'Costco'),
    ('https://www.google.com/maps/dir/37.70,-122.40/Costco/@37.68,-122.41,13z/data=!4m2!3d37.6559!4d-122.4077', (37.6559, -122.4077), 'Costco'),
    # Google's cookie consent page in the EU, and its unusual traffic page (whose q isn't a search), in front of the link
    (CONSENT_LINK, (37.1, -122.1), 'X'),
    ('https://www.google.com/sorry/index?continue=https://www.google.com/maps/place/Y/@37.2,-122.2,17z&q=EgQ', (37.2, -122.2), 'Y'),
    (APPLE_PLACE_LINK, (37.6559, -122.4077), 'Costco'),
    ('https://maps.apple.com/?q=Apple+Park&ll=37.3349,-122.0090', (37.3349, -122.009), 'Apple Park'),
  ])
  def test_app_links(self, link, coordinates, name):
    place = read_link(link)
    assert (place.coordinates, place.name) == (coordinates, name)

  @parameterized.expand([
    (IPHONE_LINK, COSTCO),
    (ANDROID_LINK, COSTCO),
    ('https://maps.apple.com/?q=Apple%20Park', 'Apple Park'),
    ('https://maps.apple.com/place?address=1600+El+Camino+Real,+South+San+Francisco,+CA&name=Costco', '1600 El Camino Real, South San Francisco, CA'),
    ('https://www.google.com/maps/dir/?api=1&destination=Pier+39', 'Pier 39'),
    # the viewport of a search or directions is only where the map was
    ('https://www.google.com/maps/search/costco/@37.6,-122.4,12z', 'costco'),
    ('https://www.google.com/maps/dir/37.70,-122.40/Costco+Wholesale,+South+San+Francisco/@37.68,-122.41,13z', 'Costco Wholesale, South San Francisco'),
    # only the start has a stop pair: it is never taken for the destination, which is looked up instead
    ('https://www.google.com/maps/dir/Home/Costco/@37.6,-122.4,12z/data=!4m8!4m7!1m2!2m2!1d-122.3!2d37.5!1m0', 'Costco'),
  ])
  def test_text_only_links(self, link, query):
    place = read_link(link)
    assert place.coordinates is None and place.query == query

  def test_interstitials_are_never_destinations(self):
    assert read_link('https://consent.google.com/ml?gl=SE&hl=sv') == Place(None)
    assert read_link('https://www.google.com/sorry/index?q=EgQ') == Place(None)
    assert read_link('https://consent.google.com/ml?continue=https://www.google.com/sorry/index%3Fq%3DEgQ') == Place(None)

  def test_pasted_text(self):
    # share sheets put the place's name before the link
    with patch('requests.get', return_value=answer(302, 'https://www.google.com/maps/place/Costco/data=!3d37.6559!4d-122.4077')) as get:
      place = place_from_text('Costco Wholesale\nhttps://maps.app.goo.gl/abc123')
    assert (place.coordinates, place.name) == ((37.6559, -122.4077), 'Costco')
    assert get.call_args.args[0] == 'https://maps.app.goo.gl/abc123'

  def test_short_links(self):
    place = 'https://www.google.com/maps/place/X/data=!3d37.1!4d-122.1'
    with patch('requests.get', side_effect=[answer(301, '/abc123?g_st=ic'), answer(302, place)]) as get:
      assert resolve_link('https://maps.app.goo.gl/abc123') == place
    # one redirect at a time, each checked before it's fetched: a relative one stays on the short-link host
    assert [c.args[0] for c in get.call_args_list] == ['https://maps.app.goo.gl/abc123', 'https://maps.app.goo.gl/abc123?g_st=ic']
    assert all(c.kwargs['allow_redirects'] is False and c.kwargs['timeout'] == 5 for c in get.call_args_list)
    with patch('requests.get', return_value=answer(301, 'https://maps.apple.com/place?coordinate=37.3349,-122.0090')) as get:
      assert resolve_link('https://maps.apple/p/AbC.dEf') == 'https://maps.apple.com/place?coordinate=37.3349,-122.0090'
    get.assert_called_once()
    with patch('requests.get') as get:
      assert resolve_link('https://example.com/maps.app.goo.gl') == 'https://example.com/maps.app.goo.gl'  # other hosts are never fetched
      assert resolve_link('37.1, -122.1') == '37.1, -122.1'
    get.assert_not_called()

  def test_short_link_failures(self):
    with patch('requests.get', return_value=answer(200)), self.assertRaises(LinkError):  # a page, not a redirect
      resolve_link('https://maps.app.goo.gl/abc')
    with patch('requests.get', side_effect=[answer(302, f'/hop{i}') for i in range(MAX_REDIRECTS)]) as get, self.assertRaises(LinkError):
      resolve_link('https://maps.app.goo.gl/abc')
    assert get.call_count == MAX_REDIRECTS
    hops = [answer(302, f'/hop{i}') for i in range(MAX_REDIRECTS - 1)] + [answer(302, 'https://www.google.com/maps/place/X')]
    with patch('requests.get', side_effect=hops):
      assert resolve_link('https://maps.app.goo.gl/abc') == 'https://www.google.com/maps/place/X'

  @parameterized.expand([
    ('https://evil.example\\@maps.app.goo.gl/x',),  # urlsplit finds maps.app.goo.gl in this, but requests connects to evil.example
    ('https://evil.example@maps.app.goo.gl/x',),
    ('https://maps.app.goo.gl:8443/x',),
    ('https://maps.app.goo.gl.evil.example/x',),
    ('https://maps.app.goo.gl。evil.example/x',),  # a full stop, to IDNA
    ('https://maps.app.goo.gl\t.evil.example/x',),
    ('https://maps.app.goo.gl/x y',),
    ('https://maps.app.goo.gl/x\x00',),
    ('http://maps.app.goo.gl/x',),
    ('https://goo.gl/maps/x',),
    ('https://evil.example/maps.app.goo.gl',),
    ('https://evil.example#@maps.app.goo.gl/x',),
    ('https://www.google.com/maps/place/X',),
  ])
  def test_only_short_links_are_fetched(self, link):
    # neither when pasted nor when a short link redirects to it
    with patch('requests.get', return_value=answer(302, link)) as get:
      assert resolve_link(link) == link
      get.assert_not_called()
      assert resolve_link('https://maps.app.goo.gl/abc') == urljoin('https://maps.app.goo.gl/abc', link)
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

  def test_search_text(self):
    # Mapbox rejects semicolons, and more than 20 words and numbers or 256 characters
    assert search_text(' Costco;  Wholesale\n') == 'Costco Wholesale'
    assert search_text(' '.join(str(n) for n in range(30))) == ' '.join(str(n) for n in range(20))
    assert len(search_text('a' * 300)) == 256
    with patch('requests.get', return_value=mapbox()) as get:
      geocode('1600 El Camino Real; ' * 10, 'pk.test', None)
    assert get.call_args.kwargs['params']['q'] == ' '.join(['1600 El Camino Real'] * 5)
    with patch('requests.get') as get:
      assert geocode(' ; ', 'pk.test', None) == []
    get.assert_not_called()


class TestNavWeb(OpenpilotTestCase):
  def test_destination(self):
    params = FakeParams()
    navweb = NavWeb(params)
    status, _ = navweb.set_destination({'latitude': 37.7749, 'longitude': -122.4194, 'place_name': 'Home'})
    assert status == 200 and params['NavDestination'] == {'latitude': 37.7749, 'longitude': -122.4194, 'place_name': 'Home'}
    assert params.blocked['NavDestination']  # saved before it's read back for the response
    assert navweb.set_destination({'text': 'somewhere nice'})[0] == 400
    assert navweb.set_destination({'latitude': 91, 'longitude': 0})[0] == 400
    assert params['NavDestination']['place_name'] == 'Home'
    navweb.set_destination({'text': '37.1, -122.1'})
    assert params['NavDestination'] == {'latitude': 37.1, 'longitude': -122.1, 'place_name': '37.10000, -122.10000'}
    navweb.clear_destination({})
    assert 'NavDestination' not in params

  @parameterized.expand([
    (CONSENT_LINK, 'X'),
    (APPLE_PLACE_LINK, 'Costco'),
    ('https://maps.apple.com/?q=Apple+Park&ll=37.3349,-122.0090', 'Apple Park'),
    ('https://www.google.com/maps/search/37.6559,+-122.4077?entry=tts', '37.65590, -122.40770'),
  ])
  def test_place_names(self, link, place_name):
    params = FakeParams()
    assert NavWeb(params).set_destination({'text': link})[0] == 200
    assert params['NavDestination']['place_name'] == place_name and params.blocked['NavDestination']

  def test_links_without_coordinates(self):
    home = {'latitude': 37.7749, 'longitude': -122.4194, 'place_name': 'Home'}
    params = FakeParams(MapboxToken='pk.test', NavDestination=home)
    navweb = NavWeb(params)
    # Mapbox has addresses but no businesses, so with nothing found for both, the address is looked up alone
    with patch('requests.get', side_effect=[mapbox(), mapbox((COSTCO_ADDRESS, 37.6559, -122.4077))]) as get:
      status, payload = navweb.set_destination({'text': IPHONE_LINK})
    assert status == 200 and payload == {'places': [{'place_name': COSTCO_ADDRESS, 'latitude': 37.6559, 'longitude': -122.4077}]}
    assert [c.kwargs['params']['q'] for c in get.call_args_list] == [COSTCO, COSTCO_ADDRESS]
    assert params['NavDestination'] == home  # until a place is picked
    with patch('requests.get', side_effect=[answer(302, ANDROID_LINK), mapbox((COSTCO, 37.6559, -122.4077))]) as get:
      status, payload = navweb.set_destination({'text': 'https://maps.app.goo.gl/abc123'})
    assert status == 200 and get.call_count == 2 and params['NavDestination'] == home
    navweb.set_destination(payload['places'][0])
    assert params['NavDestination'] == {'latitude': 37.6559, 'longitude': -122.4077, 'place_name': COSTCO}

  def test_link_lookups_need_a_token(self):
    params = FakeParams()
    with patch('requests.get') as get:
      status, payload = NavWeb(params).set_destination({'text': IPHONE_LINK})
    assert status == 400 and 'token' in payload['error'] and 'NavDestination' not in params
    get.assert_not_called()

  def test_link_failures(self):
    navweb = NavWeb(FakeParams(MapboxToken='pk.test'))
    with patch('requests.get', side_effect=requests.ConnectionError('https://maps.app.goo.gl/abc')), \
         patch('openpilot.sunnypilot.navd.navweb.cloudlog') as cloudlog:
      assert navweb.set_destination({'text': 'https://maps.app.goo.gl/abc'}) == (502, {'error': OFFLINE})
    assert 'maps.app.goo.gl' not in repr(cloudlog.mock_calls)
    for failing in (patch('requests.get', return_value=answer(404)), patch('requests.get', return_value=answer(302, '/again')),
                    patch('requests.get', side_effect=requests.ReadTimeout('https://maps.app.goo.gl/abc'))):
      with failing:
        assert navweb.set_destination({'text': 'https://maps.app.goo.gl/abc'}) == (502, {'error': LINK_FAILED})

  def test_token(self):
    params = FakeParams()
    navweb = NavWeb(params)
    assert navweb.state()['tokenHint'] == ''
    assert navweb.search({'query': 'ferry building'})[0] == 400  # searching needs a token
    navweb.set_token({'token': ' pk.abcdefgh1234 '})
    assert params['MapboxToken'] == 'pk.abcdefgh1234' and params.blocked['MapboxToken']
    assert navweb.state()['tokenHint'] == '…1234'  # never the token itself
    # only an empty token removes it: a request without one changes nothing
    assert navweb.set_token({})[0] == 400 and navweb.set_token({'token': None})[0] == 400
    assert params['MapboxToken'] == 'pk.abcdefgh1234'
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

  def test_searches_offroad_are_near_the_last_fix(self):
    params = Params()
    params.put("MapboxToken", "pk.test")
    navweb = NavWeb(params)
    assert proximity(navweb) is None  # never had a fix
    fix = messaging.new_message('gpsLocationExternal')
    fix.gpsLocationExternal.latitude, fix.gpsLocationExternal.longitude = 37.6559, -122.4077
    params.put("LastGPSPosition", fix.to_bytes())  # as ubloxd saves it
    assert proximity(navweb) == '-122.4077,37.6559'
    navweb.position = (37.1, -122.1)  # a live fix wins
    assert proximity(navweb) == '-122.1,37.1'
    navweb.position = None
    params.put("LastGPSPosition", json.dumps({'latitude': 37.5, 'longitude': -122.3}).encode())  # as osm_map_data writes it on macOS
    assert proximity(navweb) == '-122.3,37.5'
    for unreadable in (b'{}', b'not a position', messaging.new_message('gpsLocation').to_bytes(), messaging.new_message('gpsLocationExternal').to_bytes()):
      params.put("LastGPSPosition", unreadable)
      assert proximity(navweb) is None


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
      # float() raises OverflowError, not ValueError, for an integer too big for a float
      for huge in ({'latitude': int('9' * 400), 'longitude': 1}, {'latitude': 1, 'longitude': -int('9' * 400)}):
        status, body = request(port, 'POST', '/api/destination', json.dumps(huge))
        assert status == 400 and json.loads(body) == {'error': 'Invalid request'}
      assert request(port, 'POST', '/api/destination', '{"latitude": 1e999, "longitude": 1}')[0] == 400  # inf
      assert request(port, 'POST', '/api/clear', 'x' * (MAX_BODY + 1))[0] == 413
      assert request(port, 'POST', '/api/token', '{}')[0] == 400

  def test_other_sites_cannot_post(self):
    # a form on another site can only send a simple content type, which never changes anything
    params = FakeParams()
    with serving(NavWeb(params)) as port:
      assert request(port, 'POST', '/api/token', 'token=pk.evil', content_type='application/x-www-form-urlencoded')[0] == 415
      assert request(port, 'POST', '/api/token', '{"token": "pk.evil"}', content_type='text/plain')[0] == 415
    assert 'MapboxToken' not in params

  def test_rebound_names_are_refused(self):
    # a page on another site that rebinds its DNS name to the device is same-origin with it, but sends that name as the Host
    params = FakeParams(MapboxToken='pk.secret')
    with serving(NavWeb(params)) as port:
      for host in ('evil.example', 'evil.example:8082', '192.168.1.5.nip.io:8082', 'localhost.evil.example', '[::1].evil.example', ''):
        status, body = request(port, 'GET', '/api/state', headers={'Host': host})
        assert status == 421 and b'pk.secret' not in body and 'error' in json.loads(body)
        assert request(port, 'POST', '/api/token', '{"token": "pk.evil"}', headers={'Host': host})[0] == 421
        assert request(port, 'GET', '/', headers={'Host': host})[0] == 421
      assert raw_request(port, 'GET', '/api/state', {})[0] == 421  # no Host at all
      for host in ('192.168.43.1:8082', f'127.0.0.1:{port}', f'LOCALHOST:{port}', 'localhost', '[::1]:8082', '[fe80::1]', '10.0.0.2'):
        assert request(port, 'GET', '/api/state', headers={'Host': host})[0] == 200
    assert params['MapboxToken'] == 'pk.secret'

  def test_other_origins_are_refused(self):
    params = FakeParams()
    destination = json.dumps({'text': '37.1, -122.1'})
    with serving(NavWeb(params)) as port:
      for origin in ('http://evil.example', f'http://evil.example:{port}', 'null', f'http://127.0.0.1:{port}.evil.example', f'http://127.0.0.1:{port + 1}'):
        assert request(port, 'POST', '/api/destination', destination, headers={'Origin': origin})[0] == 403
        assert request(port, 'GET', '/api/state', headers={'Origin': origin})[0] == 403
      assert 'NavDestination' not in params
      # the page's own requests
      assert request(port, 'POST', '/api/destination', destination, headers={'Origin': f'http://127.0.0.1:{port}'})[0] == 200
    assert params['NavDestination']['latitude'] == 37.1

  def test_request_bodies(self):
    params = FakeParams(NavDestination={'latitude': 37.1, 'longitude': -122.1, 'place_name': ''})
    with serving(NavWeb(params)) as port:
      headers = {'Host': f'127.0.0.1:{port}', 'Content-Type': 'application/json'}
      assert raw_request(port, 'POST', '/api/clear', headers)[0] == 411
      # '²' passes str.isdigit(), and int() rejects it
      assert raw_request(port, 'POST', '/api/clear', {**headers, 'Content-Length': '²'}, b'{}')[0] == 400
      assert raw_request(port, 'POST', '/api/clear', {**headers, 'Content-Length': '-2'}, b'{}')[0] == 400
      assert raw_request(port, 'POST', '/api/clear', {**headers, 'Content-Length': '9' * 5000}, b'{}')[0] == 413  # past what int() reads
      # the server never decodes chunks, so it would read them as the body
      assert raw_request(port, 'POST', '/api/clear', {**headers, 'Transfer-Encoding': 'chunked'}, b'2\r\n{}\r\n0\r\n\r\n')[0] == 501
      assert raw_request(port, 'POST', '/api/clear', {**headers, 'Content-Length': '2', 'Transfer-Encoding': 'chunked'}, b'{}')[0] == 501
      assert raw_request(port, 'GET', '/api/state', {'Host': f'127.0.0.1:{port}', 'Transfer-Encoding': 'chunked'}, b'0\r\n\r\n')[0] == 501
      assert 'NavDestination' in params
      assert raw_request(port, 'POST', '/api/clear', {**headers, 'Content-Length': '2'}, b'{}')[0] == 200
    assert 'NavDestination' not in params

  def test_stalled_clients_are_dropped(self):
    assert make_handler(NavWeb(FakeParams())).timeout == 10
    # rather than holding a thread for as long as they keep the connection open
    with serving(NavWeb(FakeParams()), timeout=0.1) as port, socket.create_connection(('127.0.0.1', port), timeout=5) as client:
      client.sendall(b'GET /api/state HTTP/1.1\r\nHost: 127.0.0.1\r\n')
      assert client.recv(1024) == b''

  @parameterized.expand([
    (b'GET /api/state HT',),
    (b'GET /api/state HTTP/1.1\r\nHost: 127.0.0.1\r\nX-Slow: ',),
    (b'POST /api/clear HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Type: application/json\r\nContent-Length: 1000\r\n\r\n{',),
  ])
  def test_trickling_clients_are_cut_off(self, start):
    # the handler's timeout restarts with every byte it reads, so alone it would never drop a client sending one byte at a time
    with patch('openpilot.sunnypilot.navd.navweb.MAX_CONNECTION_TIME', 1.), patch('openpilot.sunnypilot.navd.navweb.MAX_CONNECTIONS', 1), \
         serving(NavWeb(FakeParams())) as port:
      with socket.create_connection(('127.0.0.1', port), timeout=5) as client:
        connected = time.monotonic()
        client.sendall(start)
        assert not answered(port)  # it holds the only connection
        trickle(client)
        assert 0.9 < time.monotonic() - connected < 2.5  # cut off at its deadline, long before the handler's 10 s timeout
      assert eventually(lambda: answered(port))  # which frees its connection

  def test_connections_are_capped(self):
    # each connection holds a thread and its buffers, and a phone's browser opens a few at a time
    with serving(NavWeb(FakeParams())) as port:
      idle = [socket.create_connection(('127.0.0.1', port), timeout=5) for _ in range(MAX_CONNECTIONS)]
      try:
        with socket.create_connection(('127.0.0.1', port), timeout=5) as extra:
          assert extra.recv(1) == b''  # closed at once, without waiting for a request
        assert select.select(idle, [], [], 0.2)[0] == []  # while the others stay open
        idle.pop().close()
        assert eventually(lambda: answered(port))  # one that closes makes room
      finally:
        for client in idle:
          client.close()
      # finished requests never keep theirs: with each one kept, all later ones would be refused
      assert eventually(lambda: all(answered(port) for _ in range(2 * MAX_CONNECTIONS)))

  def test_large_headers_are_refused(self):
    # the stdlib alone reads up to 100 header lines of 64 KB each, before do_GET or do_POST could refuse them
    def status_line(port, size):
      """The response's status line to a request whose headers are size bytes, the blank line that ends them included."""
      host = b'Host: 127.0.0.1\r\n'
      padding = b'X-Padding: ' + b'a' * (size - len(host) - len(b'X-Padding: \r\n\r\n')) + b'\r\n'
      with socket.create_connection(('127.0.0.1', port), timeout=5) as client:
        client.sendall(b'GET /api/state HTTP/1.1\r\n' + host + padding + b'\r\n')
        return client.makefile('rb').readline()

    with serving(NavWeb(FakeParams())) as port:
      assert status_line(port, MAX_HEADERS).startswith(b'HTTP/1.0 200 ')
      assert status_line(port, MAX_HEADERS + 1).startswith(b'HTTP/1.0 431 ')

  def test_main_serves_with_the_limits(self):
    class Stop(Exception):
      pass

    with patch('openpilot.sunnypilot.navd.navweb.Server') as server, patch.object(messaging, 'SubMaster', side_effect=Stop), self.assertRaises(Stop):
      main()
    assert server.call_args.args[0] == ('0.0.0.0', PORT)

  def test_failed_search_keeps_token_out_of_logs(self):
    with serving(NavWeb(FakeParams(MapboxToken='pk.secret'))) as port, \
         patch('requests.get', side_effect=requests.ConnectionError('url: /forward?access_token=pk.secret')), \
         patch('openpilot.sunnypilot.navd.navweb.cloudlog') as cloudlog:
      status, body = request(port, 'POST', '/api/search', json.dumps({'query': 'ferry building'}))
    assert status == 502 and b'pk.secret' not in body
    assert 'pk.secret' not in repr(cloudlog.mock_calls)

  @parameterized.expand([
    (mapbox_failure(401), MAPBOX_ERRORS[401]),
    (mapbox_failure(403), MAPBOX_ERRORS[403]),
    (mapbox_failure(422), MAPBOX_ERRORS[422]),
    (mapbox_failure(429), MAPBOX_ERRORS[429]),
    (mapbox_failure(503), MAPBOX_FAILED),
    (requests.ConnectionError('url: /forward?access_token=pk.secret'), OFFLINE),
    (requests.ReadTimeout('url: /forward?access_token=pk.secret'), MAPBOX_FAILED),
  ])
  def test_mapbox_errors(self, outcome, message):
    with serving(NavWeb(FakeParams(MapboxToken='pk.secret'))) as port, patch('requests.get', side_effect=[outcome]):
      status, body = request(port, 'POST', '/api/search', json.dumps({'query': 'costco'}))
    assert status == 502 and json.loads(body) == {'error': message}
