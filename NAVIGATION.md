# Route-assisted turns and exits

This branch (`nav`) adds route-assisted turns and exits to sunnypilot. It uses sunnypilot's
built-in navigation (alpha) and adapts StarPilot's navigation desires. Every setting below is off
by default; with them off, the car drives like plain sunnypilot.

## What it does

- **Turns with your blinker** (*nav turn desires*): with a destination set, **signal for the route's next turn about
  8 seconds ahead** (about 100 m / 330 ft at 30 mph). About 3 seconds before the car starts steering into the turn, the car
  gives the driving model its "turn" desire, and the screen shows **Turning Left / Turning Right**. It does this only if the
  car can still be down to turning speed by then:
  - with alpha longitudinal, which slows the car for the turn (see below): under 20 mph (32 km/h), and slow enough to get
    down to turning speed in time;
  - with the car's own cruise control, which doesn't slow for turns: under about 12 mph (20 km/h) for a right turn, or
    15 mph (24 km/h) for a left turn (the other way round where traffic keeps left).
- **A turn it can't make is skipped, and the car goes straight.** That happens when the turn hasn't started by the time the
  car is about 8 m (25 ft) from the corner, where it would start steering, or when the car is too fast to slow to turning
  speed in time. A late signal, a blind-spot warning or a lane change that ends close to the corner can each cause it. If
  the car stops before the corner, at a stop line for example, while still facing straight ahead, the turn can start there.
- **Waiting at a turn:** if you stop short of the turn, at a red light or waiting for a gap in traffic, the car repeats the
  desire every 4 seconds while it still faces straight ahead, so the model still knows about the turn when you go. Once the
  car has begun turning, it doesn't repeat it. After a long stop in the middle of a turn, the model may have forgotten the
  turn, so be ready to steer the rest yourself.
- **Lane changes:** while your blinker is on for the route's next turn, openpilot won't start an *automatic* (nudgeless) lane
  change, however much you slow down, until you switch the blinker off (also after a missed turn). A deliberate nudge still
  changes lanes, for example into a turn lane, until the turn begins. The turn then waits for the lane change, and the
  slowdown keeps going through it.
- **Slowing for turns:** when openpilot controls the speed (alpha longitudinal), the car slows to about 10 mph by the turn and
  holds that speed through it. The screen shows **Slowing for Route Turn** with a chime as it starts slowing for a turn. With
  blinker turns, this starts once you signal for the turn. It brakes gently (at most about 0.15 g), so after a late signal,
  or for a turn right after a freeway exit, it may not get down to turning speed in time. Then there is no turn: the car goes
  straight and keeps slowing through the intersection. Press the gas to go on.
- **Automatic turns** (*auto route turns (alpha)*): the car slows for **every** route turn, and turns **right** without your
  blinker (left where traffic keeps left). This needs alpha longitudinal. With it, openpilot's driving model also brakes on
  its own and tries to stop for red lights and stop signs, though not reliably. It only turns on its own when:
  - it has slowed to turning speed (16 mph or less) and the turn is about 3 seconds ahead;
  - the blind spot on that side is clear;
  - the model has seen the car in the curb lane for a full second. After a lane change, that second starts again once the
    lane change ends.

  The curb-lane check can't see a bike lane behind a curb or posts, and it may take a narrow painted bike lane for the curb
  lane, so **before every automatic turn, check for cyclists on the side you're turning to**, or signal. Otherwise it slows
  and waits for your blinker. Turns across oncoming traffic (left turns, where traffic keeps right) always wait for your
  blinker.
- **Exits and forks (alpha):** near a route exit or fork that has lane guidance, a light steering nudge toward it gives a
  "keep left/right" desire. Above 20 mph with your blinker on, a nudge changes lanes instead, the same as without navigation.

**Calling a turn off:** before **Turning…** shows, press the gas, switch on the opposite blinker, or disengage. The automatic
turn and the slowdown for that turn then stay off; signaling for the turn brings them back. A blinker turn and its slowdown
last only while the blinker stays on, also when the car switches it off by itself: if the blinker stops before **Turning…**
shows, there is no turn, so signal again. Once **Turning…** shows, the model has been told to turn, and nothing takes that
back, not even switching the blinker off: steer to stop it, or brake (on a Tesla, either one turns openpilot off completely;
see Tesla notes). With alpha longitudinal, while openpilot stays engaged, the car stays at turning speed through the
intersection.

**Missed turns:** if the car goes straight past a route turn it was slowing for, it keeps turning speed to about 20 m past the
corner unless you press the gas. Nothing acts on the next turn from the wrong road: the route pauses until it is
recalculated. The route moves on to the next turn only once the car is on the next road and heading along it. Past a freeway
exit or fork, it moves on only once the car is clearly on the exit road; if you stay on the freeway, nothing slows or turns
for the exit's road, and the route is recalculated. A blinker left on from the last turn doesn't count for the next one:
switch it off and on.

**GPS:** route turns need a GPS fix. A turn that has started keeps its turn desire and its slowdown through a gap in the GPS
(in a tunnel, for example), going by the distance driven, but without a fix the desire isn't repeated: after a stop of more
than a few seconds, the model may have forgotten the turn, so be ready to steer it yourself. One that hasn't started waits
for the fix, and its slowdown stops a few seconds after the fix is lost.

It does not signal for you, handle U-turns or roundabouts, or change lanes to get into a turn lane. When two turns are less
than about 35 m (115 ft) apart, the second one can't start until the car is about 20 m past the first, so it is usually
skipped and the car goes straight. It does not judge gaps in oncoming traffic, and it does not watch for pedestrians, cyclists
or cross traffic: it will turn across a bike lane. The model decides exactly where to turn, so stay ready to take over.

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
   do too: Tesla's own cruise control can't slow the car to turning speed (see Tesla notes).
4. If you use **Pause Lateral Control with Blinker** (sunnylink → Steering), turn it off. Otherwise
   steering pauses exactly when you signal for a turn.
5. **Routing:** create a free account at mapbox.com and copy your default public access token
   (`pk.…`). You'll paste it into the navigation page below. Without a token, the device tries comma's
   own routing, which may not be available without comma prime.

## Setting a destination

The comma 4 serves a small page for your phone on port **8082**.

1. Connect the comma 4 to your phone's hotspot. Its IP address is shown in the comma 4's Wi-Fi
   settings.
2. On your phone, open `http://<comma IP>:8082`. Paste your Mapbox token once and tap Save.
3. Search for an address, or paste a Google Maps / Apple Maps share link (best for businesses) or
   coordinates like `37.7749, -122.4194`.
4. The comma 4 shows the next maneuver and the distance to it. The destination clears when you turn the
   car off, or once you've arrived and drive away from it.

**Use a network only you can join.** The page has no password: anyone on the same network as the comma 4 can
open it and change your destination or your Mapbox token. Use your phone's hotspot or your home Wi-Fi, with a
password only you know, and never public Wi-Fi. Don't use the comma 4's own hotspot, and leave its
*enable tethering* setting (under network) off while navigation is on: every comma device ships with the same
hotspot password, so anyone nearby could join it and change your destination.

## Tesla notes

- **Brake whenever you need to.** With the standard Tesla harness, sunnypilot forces "disengage steering on brake", so the
  brake turns steering off as well as speed control, in the middle of a turn too. With alpha longitudinal, the slowdown
  usually gets the car to turning speed without the brake, but not after a late signal, right after a freeway exit, or for
  anything in the road.
- **Use alpha longitudinal for route turns.** Tesla's own cruise control can't go below 18 mph (30 km/h) unless a car ahead
  slows you, and nothing slows the car for a turn. So with it, blinker turns only start below about 12 mph for a right turn
  or 15 mph for a left turn, which in practice means only when traffic has slowed you down.
- **Turn speed.** openpilot caps lateral acceleration at about 3 m/s², and the Tesla limits how fast the steering wheel turns.
  That allows about 10 mph for a tight right turn, about 15 mph for a left turn, and walking pace for very tight corners.
  Faster, the car runs wide, which is why the slowdown aims for 10 mph, and why a turn only starts when the car can be at
  turning speed by the corner.
- **Grip.** Rest your hands lightly on the wheel. Holding it firmly, or steering against openpilot hard enough to stop a
  turn, disengages openpilot completely: steering and speed control, with the disengage chime. openpilot then no longer
  slows the car for the turn or for traffic ahead, so you control the speed too.
- **Turn signal.** A blinker turn and its slowdown end if the blinker goes off before **Turning…** shows, also when it
  switches itself off, for example after a short signal that only flashes a few times, or when the car cancels the signal
  after a lane change. If that happens, signal again.

## First drives

1. Start in an empty parking lot or on quiet streets, at low speed, hands on the wheel.
2. With alpha longitudinal on, begin with **blinker turns only**. Set a destination around the block,
   signal for each turn about 8 seconds ahead, and check when **Slowing for Route Turn** and **Turning…**
   appear and whether the car picks the right opening.
3. Only after that, try **auto route turns**, still on quiet streets. Signal yourself anyway where you
   can: it's the law, and it warns pedestrians and cyclists.
4. Only then try normal traffic. Different driving models may handle turns differently.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Page won't load | navigation (alpha) is on; the IP is right; your phone is on the same network |
| "Route: waiting" | the device has internet; the car is on and has a GPS fix; a Mapbox token is saved; the car is on the route's road and heading along it, not in a parking lot beside it; after a missed turn, the new route can take up to about 10 seconds |
| No "Turning…" | your blinker matches the turn, you switched it on after the last turn, and it stayed on; you signaled about 8 seconds ahead, so the car could get down to turning speed before the corner (with Tesla's cruise control: under about 12 mph for a right turn, 15 mph for a left); no blind-spot warning; no lane change ending close to the corner; steering is engaged; nav turn desires is on |
| No slowdown | alpha longitudinal is on and engaged; with blinker turns, you signaled for the turn (a blinker on since the last turn doesn't count) and the blinker is still on; you didn't press the gas or disengage on the approach |
| Auto turn waits for the blinker | it's a left turn (those always wait); alpha longitudinal is engaged and you didn't press the gas or disengage on the approach; the car is at 16 mph or less; no blind-spot warning; the model has seen the car in the curb lane for a full second, after any lane change |

## For developers

- `openpilot/sunnypilot/selfdrive/controls/lib/nav_desire.py`: `NavDesireController`, used by `DesireHelper`
  (`openpilot/selfdrive/controls/lib/desire_helper.py`) in both modelds, holds the one state for route turns: the turn and
  exit desires, what calls a turn off, and the route turn's speed limit, which modeld publishes as
  `modelDataV2SP.navTurnSpeed`. A turn starts only while the car can still make it: before the steering point and slow
  enough to reach turn speed by it, or at a crawl while still lined up with the approach (from the model's yaw rate). It
  pulses again only to stay in the model's desire history, and it pins its turn point on the odometer, so its turn speed
  holds to 20 m past the point even once navd moves on; no other turn starts until then. `ManeuverTracker` dead reckons the
  distance to each maneuver and corrects it by the median of navd's last three residuals: in at up to 5 m/s, out at 1 m/s
  past a 3 m deadband, or out at once when navd's distance is over 15 m longer (within 60 m of the maneuver, only by what
  navd pulled in lately). Once the turn started, it only comes in, and no turn starts while navd's distance is more than
  8 m longer. A maneuver is its type, direction and distance to the destination (`distanceRemaining - maneuverDistance`),
  which a GPS jump doesn't move.
- `openpilot/sunnypilot/selfdrive/controls/lib/nav_turn_speed.py`: plannerd's side of the slowdown: it follows
  `navTurnSpeed` as a target in `LongitudinalPlannerSP` (source `navTurn`) and a deceleration candidate (at most 1.5 m/s²)
  in the longitudinal planner, and raises the alert once per approach.
- `openpilot/selfdrive/navigation/navd.py`: `MapboxToken` param, lane guidance in `navInstruction`, access token kept out of
  logs. Progress along a step is measured on its nearest segment, so the car's offset beside the road counts for nothing,
  and the published distances count the car's travel since the fix (up to 1.5 s). The step moves on once the car has come
  within 15 m (or 1.5 s) of the maneuver point and heads along the route after it, checked every 2 m between fixes so that
  short steps at speed are followed; past an exit or fork, only once the car is clearly on the branch. Going past a
  maneuver without taking it, being off the route, heading against the step far from its maneuver, or having no fresh GPS
  fix makes the instruction invalid, and a missed maneuver reroutes right away. The destination is removed once the car has
  been near it and drives away.
- `openpilot/sunnypilot/navd/navweb.py`: the destination page (process `navweb`), with no authentication: it relies on a
  private network. At most 12 connections at a time, each closed 15 s after it opens, and 16 KB of headers.
- Params: `NavTurnDesire`, `NavAutoTurn`, `NavExitDesire`, `MapboxToken` (not logged). Event: `navTurnSlowing`.
