# Route-assisted turns and exits

This branch (`nav`) adds route-assisted turns and exits to sunnypilot. It uses sunnypilot's
built-in navigation (alpha) and adapts StarPilot's navigation desires. Every setting below is off
by default; with them off, the car drives like plain sunnypilot.

## What it does

- **Turns with your blinker** (*nav turn desires*): with a destination set, **signal for the route's next turn about
  8 seconds ahead** (about 100 m / 330 ft at 30 mph). On roads faster than about 45 mph (70 km/h), **signal earlier, about
  15 seconds or a quarter mile (400 m) ahead**, so the car has room to slow down: there, a signal 8 seconds ahead can be too
  late, and then the turn is skipped; above about 55 mph (90 km/h) there is no turn without an early signal. About
  3 seconds before the car starts steering into the turn, the car gives the driving model its "turn" desire, and the screen
  shows **Turning Left / Turning Right**. It does this only if the car can still be down to turning speed by then:
  - with alpha longitudinal, which slows the car for the turn (see below): under 20 mph (32 km/h), slow enough to get down
    to turning speed in time, and not still speeding up above about 11 mph (18 km/h), as when it pulls away from a light.
    Then the turn waits until the slowdown has taken hold, so a late signal while pulling away can mean no turn;
  - without alpha longitudinal, never while the car's own cruise control is on: it speeds back up toward its set speed
    (18 mph or more on a Tesla) as soon as the car ahead moves on, and would take the turn far too fast. With the cruise
    control off, and you driving the speed, the turn starts under about 12 mph (20 km/h) for a right turn, or 15 mph
    (24 km/h) for a left turn (the other way round where traffic keeps left).
- **A turn it can't make is skipped, and the car goes straight.** That happens when the turn hasn't started by the time the
  car reaches the point where it would start steering, about 8 m (25 ft) from the corner and a little more when faster, or
  when the car is too fast to slow to turning speed in time. A late signal, a blind-spot warning or a lane change that ends
  close to the corner can each cause it. If the car stops before the corner, at a stop line for example, while still facing
  straight ahead, the turn can start there. A right turn (left where traffic keeps left) never starts at a crawl within
  about 5 m (15 ft) of the turn point, though: if the car has stopped inside the intersection, it goes straight.
- **Waiting at a turn:** if you stop short of the turn, at a red light or waiting for a gap in traffic, the car repeats the
  desire every 4 seconds while it still faces straight ahead (for a right turn, only while it is short of the intersection),
  so the model still knows about the turn when you go. Once the car has begun turning, it doesn't repeat it. After a long
  stop in the middle of a turn, the model may have forgotten the turn, so be ready to steer the rest yourself.
- **Lane changes:** while your blinker is on for the route's next turn, openpilot won't start an *automatic* (nudgeless) lane
  change, however much you slow down, until you switch the blinker off (also after a missed turn). A nudge changes lanes only
  above 20 mph (32 km/h), as without navigation, and once you signal for a route turn with alpha longitudinal, the slowdown
  has the car below 20 mph about 50 m (165 ft) before the turn. So move into a turn lane before you signal for the turn, or
  steer into it yourself (on a Tesla, a firm steer turns openpilot off completely; see Tesla notes). The turn waits for a
  lane change to end, and the slowdown keeps going through it. If you also use *Lane Turn Desire*, it stays quiet while route
  turns are on and a route is set: the route gives the turn desires.
- **Slowing for turns:** when openpilot controls the speed (alpha longitudinal), the car slows to about 10 mph by the turn and
  holds that speed through it. The screen shows **Slowing for Route Turn** with a chime as it starts slowing for a turn. With
  blinker turns, this starts once you signal for the turn. **Slowing for Route Turn** doesn't by itself mean the turn will
  start. It brakes gently (at most about 0.15 g), so after a late signal, or for a turn right after a freeway exit, it may
  not get down to turning speed in time. Then there is no turn: the car goes straight, at turning speed to about 20 m
  (65 ft) past the corner; the gas overrides that only while you press it.
- **Automatic turns** (*auto route turns (alpha)*): the car slows for **every** route turn, and turns **right** without your
  blinker (left where traffic keeps left). This needs alpha longitudinal. With it, openpilot's driving model also brakes on
  its own and tries to stop for red lights and stop signs, though not reliably. It only turns on its own when:
  - it has slowed to turning speed (16 mph or less) and the turn is about 3 seconds ahead;
  - the blind spot on that side is clear;
  - the model has seen the car in the curb lane for a full second. After a lane change, that second starts again once the
    lane change ends.

  If any of these isn't met, it slows and waits for your blinker. Turns across oncoming traffic (left turns, where traffic
  keeps right) always wait for your blinker.

  The curb-lane check can't see a bike lane behind a curb or posts, and it may take a narrow painted bike lane for the curb
  lane, so **before every automatic turn, check for cyclists on the side you're turning to**, or signal.
- **Exits and forks (alpha):** near a route exit or fork that has lane guidance, a light steering nudge toward it gives a
  "keep left/right" desire. Above 20 mph with your blinker on, a nudge changes lanes instead, the same as without navigation.

**Calling a turn off:** do it before **Turning…** shows. How depends on the kind of turn:
- **An automatic turn** (no blinker): press the gas, switch on the other blinker, or disengage. The turn and its slowdown
  then stay off until you signal for that turn.
- **A turn you signaled for:** switch the blinker off, or over to the other side. The gas only ends the slowdown: the car
  speeds up again, and if it is still slow enough near the corner, behind slower traffic for example, the turn starts
  anyway. Disengaging calls it off only until you engage again with the blinker still on.

A signaled turn and its slowdown last only while the blinker stays on, also when the car switches it off by itself: if the
blinker stops before **Turning…** shows, there is no turn, so signal again. Within the last 15 m (50 ft) before the turn the
car already holds turning speed to about 20 m past the corner, whatever the blinker does. Once **Turning…** shows, the model
has been told to turn, and nothing takes that back, not even switching the blinker off: steer to stop it, or brake (on a
Tesla, either one turns openpilot off completely; see Tesla notes). With alpha longitudinal, while openpilot stays engaged,
the car stays at turning speed through the intersection, to about 20 m past the corner; the gas overrides that only while
you press it.

**Missed turns:** if the car goes straight past a route turn it was slowing for, it keeps turning speed to about 20 m past the
corner; the gas overrides that only while you press it. Nothing acts on the next turn from the wrong road: the route pauses
until it is recalculated. The route moves on to the next turn only once the car is on the next road and heading along it.
Past a freeway exit or fork, or a turn onto a road that soon runs right beside the one you're on (a service road, for
example), it moves on only once the car is clearly on the new road; if you stay on the old one, nothing slows or turns for
the new road, and the route is recalculated. A blinker left on from the last turn doesn't count for the next one: switch it
off and on.

**Leaving the route:** if the car leaves the route's road before a turn, onto a road that branches off at a shallow angle for
example, the route's next turn stops counting as soon as the car is clearly heading away along the other road: nothing slows
or turns for it, and the route is recalculated. A road that leaves at a very shallow angle (under about 12°), or one taken
within about 40 m (130 ft) of the turn, can still get that turn, so watch for **Turning…** and call it off before it shows.

**GPS:** route turns need a GPS fix. A turn that has started keeps its turn desire and its slowdown through a gap in the GPS
(in a tunnel, for example), going by the distance driven, but without a fix the desire isn't repeated: after a stop of more
than a few seconds, the model may have forgotten the turn, so be ready to steer it yourself. One that hasn't started waits
for the fix, and its slowdown stops a few seconds after the fix is lost.

It does not signal for you, handle U-turns or roundabouts, or change lanes to get into a turn lane. When two turns are less
than about 30 m (100 ft) apart, the second one is skipped and the car goes straight; about 30-40 m (100-130 ft) apart, it
starts late, so be ready to steer it. It does not judge gaps in oncoming traffic, and it does not watch for pedestrians,
cyclists or cross traffic: it will turn across a bike lane. The model decides exactly where to turn, so stay ready to take
over.

## Install on a comma 4

1. **Back up your settings:** comma 4 → sunnylink → *backup settings*. Uninstalling wipes the device's
   settings and downloaded models.
2. **Uninstall:** Settings → Software → *uninstall*.
3. On the setup screen, choose **Custom Software** and enter `installer.comma.ai/SeanLouei/nav`.
4. The first boot builds everything, including the driving model, which takes a while (roughly
   15–30+ minutes). Keep the device powered.
5. Pair sunnylink again, use *restore*, and download your driving model again if you use a custom one.

Later updates to the branch install from Settings → Software, like any sunnypilot update.

## One-time setup

1. Settings → Developer → turn on **navigation (alpha)**.
2. Settings → Models → turn on **nav turn desires**. Optionally also turn on **auto route turns
   (alpha)** and **nav exit desires (alpha)**. All of them are also in the sunnylink app under
   Steering → Navigation.
3. Turn on Settings → Developer → **alpha longitudinal** (read its warning: it may disable automatic
   emergency braking). The slowdown and automatic turns need it, and on a Tesla, route turns in practice
   do too: they don't start while Tesla's own cruise control is on (see Tesla notes).
4. If you use **Pause Lateral Control with Blinker** (sunnylink → Steering), turn it off. Otherwise
   steering pauses exactly when you signal for a turn.
5. **Routing:** create a free account at mapbox.com and copy your default public access token
   (`pk.…`). You'll paste it into the navigation page below. Without a token, the device tries comma's
   own routing, which may not be available without comma prime.

## Setting a destination

The comma 4 serves a small page for your phone on port **8082**, over Wi-Fi only.

1. Connect the comma 4 to your phone's hotspot. Its IP address is shown in the comma 4's Wi-Fi
   settings.
2. On your phone, open `http://<comma IP>:8082`. Paste your Mapbox token once and tap Save.
3. Search for an address, or paste a Google Maps / Apple Maps share link (best for businesses) or
   coordinates like `37.7749, -122.4194`. A new destination replaces the route right away.
4. The comma 4 shows the next maneuver and the distance to it. Navigation ends by itself once the car has
   come within about 25 m (80 ft) of the destination, or of where the route ends on the road, and then drives
   more than that away from both, whichever way it leaves. The destination also clears when you turn the car
   off. If you stop farther away, in a large parking lot for example, clear the destination on the page.

**Use a Wi-Fi network only you can join.** The page has no password: anyone on the same Wi-Fi network as the
comma 4 can open it and change your destination or your Mapbox token. The comma 4 answers it only over Wi-Fi,
never over its own cellular data. Use your phone's hotspot or your home Wi-Fi, with a password only you know,
and never public Wi-Fi. Don't use the comma 4's own hotspot, and leave its *enable tethering* setting (under
network) off while navigation is on: every comma device ships with the same hotspot password, so anyone nearby
could join it and change your destination.

## Tesla notes

- **Brake whenever you need to.** With the standard Tesla harness, sunnypilot forces "disengage steering on brake", so the
  brake turns steering off as well as speed control, in the middle of a turn too. With alpha longitudinal, the slowdown
  usually gets the car to turning speed without the brake, but not after a late signal, right after a freeway exit, or for
  anything in the road.
- **Use alpha longitudinal for route turns.** Tesla's own cruise control can't go below 18 mph (30 km/h) unless a car ahead
  slows you, and it speeds back up toward its set speed as soon as that car moves on, so a turn started behind it would be
  taken far too fast. So while Tesla's cruise control is on, route turns don't start at all. Without alpha longitudinal,
  they start only with the cruise control off, while you drive the speed yourself: below about 12 mph for a right turn or
  15 mph for a left turn.
- **Turn speed.** openpilot caps lateral acceleration at about 3 m/s², and the Tesla limits how fast the steering wheel turns.
  That allows about 10 mph for a tight right turn, about 15 mph for a left turn, and walking pace for very tight corners.
  Faster, the car runs wide, which is why the slowdown aims for 10 mph, and why a turn only starts when the car can be at
  turning speed by the corner.
- **Grip.** Rest your hands lightly on the wheel. Holding it firmly, or steering against openpilot hard enough to stop a
  turn, disengages openpilot completely: steering and speed control, with the disengage chime. openpilot then no longer
  slows the car for the turn or for traffic ahead, so you control the speed too.
- **Turn signal.** A blinker turn and its slowdown end if the blinker goes off before **Turning…** shows, also when it
  switches itself off, for example after a short signal that only flashes a few times, or when the car cancels the signal
  after a lane change. If that happens, signal again. Within the last 15 m (50 ft) before the turn, the car still holds
  turning speed to about 20 m past the corner.

## First drives

1. Start in an empty parking lot or on quiet streets, at low speed, hands on the wheel.
2. With alpha longitudinal on, begin with **blinker turns only**. Set a destination around the block,
   signal for each turn about 8 seconds ahead (about 15 seconds on roads faster than 45 mph), and check when
   **Slowing for Route Turn** and **Turning…** appear and whether the car picks the right opening.
3. Only after that, try **auto route turns**, still on quiet streets. Signal yourself anyway where you
   can: it's the law, and it warns pedestrians and cyclists.
4. Only then try normal traffic. Different driving models may handle turns differently.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Page won't load | navigation (alpha) is on; the IP is right; your phone and the comma 4 are on the same Wi-Fi network (the page doesn't answer over the comma's cellular data) |
| "Route: waiting" | the device has internet; the car is on and has a GPS fix; a Mapbox token is saved; the car is on the route's road and heading along it, not in a parking lot beside it; after a missed turn, or after leaving the route, the new route can take up to about 10 seconds |
| No "Turning…" | your blinker matches the turn, you switched it on after the last turn, and it stayed on; you signaled about 8 seconds ahead (about 15 seconds on roads faster than 45 mph), so the car could get down to turning speed before the corner; the car wasn't still speeding up, as when pulling away from a light; Tesla's cruise control is off (route turns don't start while it's on: use alpha longitudinal); no blind-spot warning; no lane change ending close to the corner; the car hadn't stopped inside the intersection; the turn isn't less than about 30 m after the last one; the car is still on the route's road; steering is engaged; nav turn desires is on |
| No slowdown | alpha longitudinal is on and engaged; with blinker turns, you signaled for the turn (a blinker on since the last turn doesn't count) and the blinker is still on; you didn't press the gas or disengage on the approach; the car is still on the route's road |
| Nudge doesn't change lanes | the car is above 20 mph: once you signal for a route turn, the slowdown has it below that about 50 m before the turn, so move over before you signal |
| Auto turn waits for the blinker | it's a left turn (those always wait); alpha longitudinal is engaged and you didn't press the gas or disengage on the approach; the car is at 16 mph or less; no blind-spot warning; the model has seen the car in the curb lane for a full second, after any lane change |
| Navigation goes on after you arrived | the car came within about 25 m (80 ft) of the destination; if you stopped farther away, clear the destination on the page |

## For developers

- `openpilot/sunnypilot/selfdrive/controls/lib/nav_desire.py`: `NavDesireController`, used by `DesireHelper`
  (`openpilot/selfdrive/controls/lib/desire_helper.py`) in both modelds, holds the one state for route turns: the turn and
  exit desires, what calls a turn off, and the route turn's speed limit, which modeld publishes as
  `modelDataV2SP.navTurnSpeed`. A turn starts only while the car can still make it and is still lined up with the approach
  (from the model's yaw rate): moving, before the steering point (8 m plus 0.3 s of travel), slow enough to reach turn
  speed by it and, under openpilot longitudinal, not still accelerating above 5 m/s; or at a crawl, a near side turn only
  more than 5 m short of the maneuver point. Without openpilot longitudinal, none starts while the car's own cruise is
  engaged. On faster roads the turn signal latches from the slowdown's approach distance. A turn pulses again only to stay
  in the model's desire history, and it pins its turn point on the odometer, so its turn speed holds to 20 m past the point
  even once navd moves on; no other turn starts until then. While route turns are on and a route is set, the lane turn
  desire stays quiet. `ManeuverTracker` dead reckons the distance to each maneuver and corrects it by the median of navd's
  last three residuals: in at up to 5 m/s, out at 1 m/s past a 3 m deadband, or out at once when navd's distance is over
  15 m longer (within 60 m of the maneuver, only by what navd pulled in lately). Once the turn started, it only comes in,
  and no turn starts while navd's distance is more than 8 m longer. A maneuver is its type, direction and distance to the
  destination (`distanceRemaining - maneuverDistance`), which a GPS jump doesn't move.
- `openpilot/sunnypilot/selfdrive/controls/lib/nav_turn_speed.py`: plannerd's side of the slowdown: it follows
  `navTurnSpeed` as a target in `LongitudinalPlannerSP` (source `navTurn`) and a deceleration candidate (at most 1.5 m/s²)
  in the longitudinal planner, and raises the alert once per approach.
- `openpilot/selfdrive/navigation/navd.py`: `MapboxToken` param, lane guidance in `navInstruction`, access token kept out of
  logs. Progress along a step is measured on its nearest segment, so the car's offset beside the road counts for nothing,
  and the published distances count the car's travel since the fix (up to 1.5 s). The step moves on once the car has come
  within 15 m (or 1.5 s) of the maneuver point and heads along the route after it, checked every 2 m between fixes so that
  short steps at speed are followed; past a split, only once the car is clearly on the branch, judged at a fix, never on
  the line between two. A split is an exit or fork, a turn within 45° of the old road, or a turn whose route comes back
  10-25 m beside the old road's straight continuation (a connector to a service road). Going past a maneuver without taking
  it, being off the route, heading against the step far from its maneuver, going off along another road (two fixes in a row
  more than 12° off the step's direction, each 1 m or more farther from it, while the maneuver is over 25 m ahead; back after
  two fixes within 6° and within 4 m of the earlier offset, or on a new step), or having no fresh GPS fix makes the
  instruction invalid, and all but no fix count toward a reroute; a missed maneuver reroutes right away. A new destination
  gets its route at once; a reroute waits 10 s after the last request. Arrival latches within 25 m of the destination or the
  route's end, on the route's last road step or past it; from then on there is no reroute, and once the car is more than
  25 m from both, navigation ends: the destination is removed, unless the driver set another one meanwhile.
- `openpilot/sunnypilot/navd/navweb.py`: the destination page (process `navweb`), with no authentication: it relies on a
  private Wi-Fi network. It answers only connections that arrive at 127.0.0.1 or at `wlan0`'s current IPv4 address, so
  never over the cellular link (a machine without `wlan0` answers on every address). At most 12 connections at a time,
  each closed 15 s after it opens, and 16 KB of headers.
- Params: `NavTurnDesire`, `NavAutoTurn`, `NavExitDesire`, `MapboxToken` (not logged). Event: `navTurnSlowing`.
