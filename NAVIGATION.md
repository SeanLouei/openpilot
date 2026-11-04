# Route-assisted turns and exits

This branch (`nav`) adds route-assisted turns and exits to sunnypilot. It uses sunnypilot's
built-in navigation (alpha) and adapts StarPilot's navigation desires. Every setting below is off
by default; with them off, the car drives like plain sunnypilot.

## What it does

- **Turns with your blinker** (*nav turn desires*): with a destination set, **signal for the route's next turn about
  8 seconds ahead** (about 100 m / 330 ft at 30 mph). On roads faster than about 45 mph (70 km/h), **first move into the
  lane you'll turn from, then signal earlier, about 15 seconds ahead** (about a quarter mile, 400 m, at 60 mph), so the car
  has room to slow down: there, a signal 8 seconds ahead can be too late, and then the turn is skipped. Farther out than
  about 250 m (800 ft), your blinker counts as the turn signal, and slows the car, only once the model has seen the car in
  the outermost lane on the turn's side (the curb lane, for a right turn) for a second; from another lane it's a lane
  change. So above about 55 mph (90 km/h), if **Slowing for Route Turn** doesn't show soon after you signal (from another
  lane, or where the model doesn't see the curb lane, past a wide shoulder for example), slow the car yourself (lower the
  set speed, or brake), or the turn is skipped. About 3 seconds before the car starts steering into the turn, the car gives
  the driving model its "turn" desire, and the screen shows **Turning Left / Turning Right**. It does this only if the car
  can still be down to turning speed by then:
  - with alpha longitudinal, which slows the car for the turn (see below): under 20 mph (32 km/h), and slow enough to get
    down to turning speed in time, counting the speed the car still gains while openpilot is speeding it up (pulling away
    from a light, or after the gas): openpilot takes a moment to stop speeding up. So signal before you pull away: a signal
    given while the car is already pulling away close to the turn often means no turn;
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
  stop in the middle of a turn, the model may have forgotten the turn, so be ready to steer the rest yourself. A GPS error
  can still end the repeating early while **Turning…** shows, so be ready to steer when you go. An automatic left turn is
  never repeated while the car waits: see *Automatic left turns* below.
- **Lane changes:** while your blinker is on for the route's next turn, openpilot won't start an *automatic* (nudgeless)
  lane change, however much you slow down, until you switch the blinker off (also after a missed turn). That holds for any
  blinker toward the turn from about 20 seconds before it, even before it counts as the turn signal; one the other way, or
  farther out, changes lanes as you set it to. A nudge changes lanes only above 20 mph (32 km/h), as without navigation, and
  once you signal for a route turn with alpha longitudinal, the slowdown has the car below 20 mph about 50 m (165 ft) before
  the turn. So move into a turn lane before you signal for the turn, or steer into it yourself (on a Tesla, a firm steer
  turns openpilot off completely; see Tesla notes). A lane change toward the turn within about 250 m (800 ft) of it counts
  as the turn signal, and slows the car. The turn waits for a lane change to end, and the slowdown keeps going through it.
  If you also use *Lane Turn Desire*, it stays quiet while route turns are on and a route is set: the route gives the turn
  desires.
- **Slowing for turns:** when openpilot controls the speed (alpha longitudinal), the car slows to about 10 mph by the turn
  and holds that speed through it. The screen shows **Slowing for Route Turn** with a chime as it starts slowing for a turn
  (if the car is moving into a turn lane by itself then, once that move is over). With blinker turns, this starts once you
  signal for the turn. **Slowing for Route Turn** doesn't by itself mean the turn will start. It brakes gently (at most
  about 0.15 g), so after a late signal, or for a turn right after a freeway exit, it may not get down to turning speed in
  time. Then there is no turn: the car goes straight, at turning speed to about 20 m (65 ft) past the corner. Pressing the
  gas before the corner ends that, and the car speeds up through the intersection; past the corner, the gas overrides it
  only while you press it. Once *auto turn lanes* has started moving the car into a turn lane for that turn, the gas
  overrides it only while you press it, also before the corner.
- **Automatic turns** (*auto route turns (alpha)*): the car slows for **every** route turn, and turns **right** without your
  blinker (left where traffic keeps left). This needs alpha longitudinal. With it, openpilot's driving model also brakes on
  its own and tries to stop for red lights and stop signs, though not reliably. It only turns on its own when:
  - it has slowed to turning speed (16 mph or less) and the turn is about 3 seconds ahead;
  - the blind spot on that side is clear;
  - the model has seen the car in the curb lane for a full second. After a lane change, that second starts again once the
    lane change ends; after the car moved into a turn lane by itself (*auto turn lanes*, below), it counts at once.

  If any of these isn't met, it slows and waits for your blinker. Turns across oncoming traffic (left turns, where traffic
  keeps right) wait for your blinker unless *auto left turns (alpha)* is on, and even then most of them do: see
  *Automatic left turns* below.

  The curb-lane check can't see a bike lane behind a curb or posts, and it may take a narrow painted bike lane for the curb
  lane, so **before every automatic turn, check for cyclists on the side you're turning to**, or signal.
- **Automatic turn lanes** (*auto turn lanes (alpha)*): for a right turn (left where traffic keeps left), from the curb
  lane, the car moves by itself into a turn lane that opens beside it before the route's turn, then turns from it: see
  *Automatic turn lanes* below. Never for left turns.
- **Exits and forks (alpha):** near a route exit or fork that has lane guidance, a light steering nudge toward it gives a
  "keep left/right" desire. Above 20 mph with your blinker on, a nudge changes lanes instead, the same as without navigation.

**Calling a turn off:** do it before **Turning…** shows. How depends on the kind of turn:
- **An automatic turn** (no blinker): press the gas, switch on the other blinker, or disengage. The turn and its slowdown
  then stay off until you signal for that turn. Once the car has started moving into a turn lane by itself for that turn
  (*auto turn lanes*), these call off only the automatic turn: the car may be in a turn-only lane, so it still slows to
  turning speed for the corner (hold the gas to go faster).
- **A turn you signaled for:** switch the blinker off, or over to the other side. With automatic turns on, switching it off
  also means the car won't take that turn by itself (it still slows for it): signal again if you want it. The gas only ends
  the slowdown: the car speeds up again, which usually means no turn; if it is still slow enough near the corner and no
  longer speeding up, behind slower traffic for example, the turn starts anyway. Disengaging calls it off only until you
  engage again with the blinker still on.

A signaled turn and its slowdown last only while the blinker stays on: if the blinker stops before **Turning…** shows, there
is no turn, so signal again. That includes a short signal that stops by itself after a few flashes: it counts as switching
the blinker off. Only when the car switches the blinker off by itself as an openpilot lane change ends (within about
2 seconds) is the turn not called off: with automatic turns on, the car still slows and a turn it may take by itself can still
start; with blinker turns only, there is no turn until you signal again. Within the last 15 m (50 ft) before the turn the
car already holds turning speed to about 20 m past the corner, also if the blinker goes off; switching it to the other side,
or pressing the gas, before the corner ends that hold, and the car speeds up again (not once *auto turn lanes* has started
moving the car into a turn lane for that turn: then the gas overrides it only while you press it). Once **Turning…** shows,
the model has been told to turn, and nothing takes that back, not even switching the blinker off: steer to stop it, or brake
(on a Tesla, either one turns openpilot off completely; see Tesla notes). With alpha longitudinal, while openpilot stays
engaged, the car stays at turning speed through the intersection, to about 20 m past the corner; the gas overrides that only
while you press it.

**Missed turns:** if the car goes straight past a route turn it was slowing for, it keeps turning speed to about 20 m past
the corner. Pressing the gas, or switching on the other blinker, before the corner ends that, and the car speeds up through
the intersection; past the corner, the gas overrides it only while you press it. Once *auto turn lanes* has started moving
the car into a turn lane for that turn, neither ends it: the gas overrides it only while you press it. Nothing acts on the next turn from the
wrong road: the route pauses until it is recalculated. The route moves on to the next turn only once the car is on the next
road and heading along it. Past a freeway exit or fork, or a turn onto a road that soon runs right beside the one you're on
(a service road, for example), it moves on only once the car is clearly on the new road; if you stay on the old one, nothing
slows or turns for the new road, and the route is recalculated. A blinker left on from the last turn doesn't count for the
next one: switch it off and on.

**Leaving the route:** if the car leaves the route's road before a turn, onto a road that branches off at a shallow angle
for example, the route's next turn stops counting as soon as the car is clearly heading away along the other road: nothing
slows or turns for it, and the route is recalculated. A road that leaves at a very shallow angle (under about 12°, or
somewhat more in slow traffic when the GPS reports a poor heading), or one taken within about 40 m (130 ft) of the turn, can
still get that turn, so watch for **Turning…** and call it off before it shows. Where a very wide median (about 20 m across)
ends gradually, within about 70 m (230 ft) of a turn, that turn can still be dropped and the route recalculated.

**GPS:** route turns need a GPS fix. A turn that has started keeps its turn desire and its slowdown through a gap in the GPS
(in a tunnel, for example), going by the distance driven, but without a fix the desire isn't repeated: after a stop of more
than a few seconds, the model may have forgotten the turn, so be ready to steer it yourself. One that hasn't started waits
for the fix, and its slowdown stops a few seconds after the fix is lost.

It does not signal for you or handle U-turns or roundabouts, and it changes lanes into a turn lane only with *auto turn
lanes*, and only where that section says. When two turns are less
than about 30 m (100 ft) apart, the second one is skipped and the car goes straight; about 30-40 m (100-130 ft) apart, it
starts late, so be ready to steer it. It does not judge gaps in oncoming traffic, and it does not watch for pedestrians,
cyclists or cross traffic: it will turn across a bike lane. The model decides exactly where to turn, so stay ready to take
over.

## Automatic left turns (alpha)

*auto left turns (alpha)* is a second, separate setting on top of *auto route turns (alpha)*. With both on, the car also
turns **left** by itself (right where traffic keeps left), across oncoming traffic, without your blinker, but only where it
can confirm it is in a lane for that turn. Everywhere else it slows and waits for your blinker, as without the setting,
and asks you to signal. It is off by default, does nothing without *auto route turns* and alpha longitudinal, and comes on
only while the car is parked or stopped: switched on while driving (or *auto route turns* switched on while driving with
it already set), it waits until the car next stands still. It can be switched off at any time (on the comma 4, also while
driving). It is not restored from a sunnylink backup.

**Where it turns by itself:** only from the lane right beside a raised median or curb on the left: a curbed left-turn bay,
the lane beside a raised median, or the left lane of a one-way street. All of these must hold:
- the model has surely seen that curb close beside the car, on the left, with no lane line beyond the car's own, for the
  last 10 m (33 ft) and 1.5 seconds, with no lane change and no car in the left blind spot. A lane change, or a car in the
  blind spot, starts the 10 m again;
- the route's lane guidance, where it has any, doesn't put the left turn in another lane;
- it is a plain left turn: not a sharp one, and not the end of a road (a T junction, where you yield both ways);
- the posted speed limit is known, from map data or the car, and is 45 mph (72 km/h) or less over the last 150 m (500 ft)
  before the turn; where the car's own reading (a Tesla's) is higher than the map's, the higher one counts;
- driver monitoring isn't warning;
- the car is moving, and the turn starts at least 2 seconds before the car steers across;
- everything an automatic right turn needs: alpha longitudinal engaged, slowed to turning speed, about 3 seconds ahead.

**Where it waits for your blinker:** painted turn lanes, center turn lanes and two-way streets (there is no curb beside the
turn lane), and anywhere one of the above isn't met.

**The two alerts:**
- **Turning Across Traffic** (*Brake for oncoming traffic*), with its own chime (a different sound from the other route
  alerts), as an automatic left turn starts, at least 2 seconds before the car steers across. It shows in place of
  **Turning Left**, and what this guide says about **Turning…** applies to it: once it shows, the model has been told to
  turn. It goes away once the turn is handed back to you (see *Stops*).
- **Signal for Route Turn** (*It won't turn by itself here*), with a chime, when a left turn won't start by itself: from
  about 5 seconds before the car would steer into the turn, or as soon as it can no longer start by itself after that (half
  a second after a lane change into the bay ends, for example, when the curb can't be seen for long enough before the
  turn), and while the car waits short of it. Once it has shown, that turn waits for your blinker, even if the curb shows
  up after. Signal as soon as it shows: shown about 5 seconds ahead, a blinker then still starts the turn on time; shown
  later, the turn starts later, and too late it doesn't.

Driver monitoring's **Driver Distracted** and **Driver Unresponsive** warnings show over both alerts. sunnypilot's *Quiet
Mode* silences their chimes, and those of the other route alerts, so with it on, watch the screen.

**What it can't see:** oncoming traffic, turn arrows, red lights and stop signs; it doesn't judge gaps, and doesn't watch
for pedestrians or cyclists. Frontage roads, channelized islands, curbed bus lanes and median U-turn openings can look like
a turn lane to the curb check, and on one-way streets cyclists may ride on the left. Changing the driving model can change
how curbs are seen.

**Stops:** if the car stops (about 3 mph or slower) within about 30 m (100 ft) of the turn before it starts, at a light, a
stop sign or in a queue, the turn is left to you: the car keeps slowing for it, shows **Signal for Route Turn**, and turns
once you signal. After the turn started, it isn't repeated while the car waits or creeps (about 3 mph or slower), nor while
driver monitoring warns, nor in the last 2 seconds before the car steers across. About 5 seconds after it started (or was
last repeated), when the model may have forgotten it, it is handed back to you, also if the car is moving again by then:
**Turning Across Traffic** goes away and, while your blinker can still start it again, **Signal for Route Turn** shows.
Handed back closer to the turn than that, the car goes straight.

**Calling it off:** on the approach, switching off a left blinker you had on for the turn (a short signal that stops by
itself counts too), pressing the gas, or switching on the right blinker means no automatic left turn at that intersection:
signal for it if you still want it. When the car switches the blinker off by itself as an openpilot lane change into a
turn bay ends (within about 2 seconds), the turn can still start by itself, once the curb has been beside the car for
10 m.

**Brake for oncoming traffic.** On a Tesla, braking turns steering off as well as speed control, also in the middle of the
turn: then finish the turn yourself.

**Without the blinker, a left turn it refuses or hands back means the car drives straight through the intersection** at
about 10 mph, also out of a left-turn-only lane. So when **Signal for Route Turn** shows, signal or take over.

**First use:** only after blinker turns and automatic right turns work well for you, on quiet streets, with your foot over
the brake. Turning without signaling is illegal in most places: signal yourself where you can.

## Automatic turn lanes (alpha)

*auto turn lanes (alpha)* is a third setting, on top of *auto route turns (alpha)*. With it on, when the car is in the curb
lane and a turn lane opens beside it on that side before the route's turn, the car moves into the turn lane by itself,
without the blinker, and then turns from it as it would from the curb lane. That is for right turns only (left turns where
traffic keeps left). **It never moves the car for a left turn**: where a raised median or an island ends and the road goes
on as a two-way road, the oncoming lane looks to the camera exactly like a curbed left-turn bay opening beside the car, and
nothing the car sees tells them apart. For a left turn, move into the turn lane yourself: with *auto left turns* on, the
turn can then start by itself from a curbed bay, as that section says. It is off by default, comes on only while the car is
parked or stopped (switched on while driving, it waits until the car next stands still), can be switched off at any time
(on the comma 4, also while driving), and is not restored from a sunnylink backup.

**When it moves:** all of these must hold:
- the model has seen the car in the curb lane for at least a second and 10 m (33 ft), for this turn: a lane change,
  steering turned off, or the route moving on to this turn from the one before starts that again, and it counts only for
  50 m (165 ft) after the car last had the curb beside it, whether the turn lane is open by then or not. So a turn lane
  that opens more than about 50 m before the car comes within 150 m of the turn gets no move;
- the turn lane opens right beside the car's lane: the curb steps out by at least 2.6 m (8.5 ft), the model surely sees
  the lane line between the car and the turn lane, there is no lane line beyond it, and it is that wide all the way to
  30 m (100 ft) ahead (or to 10 m before the turn), for half a second;
- the route's lane guidance, where it has any, has the turn from that outermost lane;
- everything an automatic turn needs: *auto route turns* on, alpha longitudinal and steering engaged, the turn not called
  off;
- driver monitoring isn't warning, and hasn't for the last 3 seconds (see *The alert*);
- no blinker on, no car in the blind spot on that side, *Block Lane Change: Road Edge Detection* not blocking that side,
  and *Auto Lane Change by Blinker* not set to *Off*;
- not within 5 seconds of the last turn the car made, which the driving model still remembers then;
- the car is between about 6 and 33 mph (9-54 km/h), within 150 m (490 ft) of the turn, and far enough that the move ends
  well before the turn would start: the turn lane must be open by about 70 m (230 ft) before the turn at 20 mph, and
  80 m (260 ft) at 25-30 mph. Opening later, the car stays in its lane, as without the setting: no automatic right turn
  from there.

It moves once per turn at most, and only toward the turn.

**Where it doesn't:** left turns (right where traffic keeps left), into curbed bays, painted bays or center turn lanes
alike; turn lanes that open too close to the turn, or too long before it; short bays. It is meant to refuse bike lanes and
shoulders (narrower than a turn lane) and bus bays (shorter), but the camera can misjudge them, and a parking lane, a paved
shoulder 2.6 m wide or more, or a turn lane for an earlier street or driveway can look just like the turn lane: **watch
every move**. Where a second turn lane opens beyond the first one later, the car may end up in the inner one and then not
turn by itself.

**The alert:** **Moving to Turn Lane** (*Steer to stay in your lane*) shows while the route's lane change lasts, in place
of **Changing Lanes**, with the soft *Pay Attention* chime, which none of the other route alerts use. It is the chime of
driver monitoring's first warning too, so the car doesn't start moving over while driver monitoring warns, or for 3 seconds
after. If driver monitoring's first warning comes during the move, it shows over **Moving to Turn Lane** without chiming
again; its stronger warnings show over it and chime as usual. If the slowdown starts during the move, **Slowing for Route
Turn** shows once the move is over.

**Stopping it:** once the move has started, the driving model has been told to change lanes, and only steering against it
keeps the car in its lane (or braking: on a Tesla, braking turns steering off as well as speed control). The gas, either
blinker, or a car coming up in the blind spot on that side before the car starts moving over ends the move and its alert,
but the car may still finish moving over: if a car comes up beside you in the turn lane, steer back. Whatever ends the
move, the car doesn't try again for that turn.

**After the move:** the car turns from the turn lane as from the curb lane, with the same rules, while the model still sees
the car in the outermost lane there. If it doesn't, or if you pressed the gas, switched on the other blinker or disengaged
the speed control near the turn after the move started, there is no automatic turn, but the car still slows to turning
speed for the corner in whichever lane it ends up (hold the gas to go faster): it goes straight at turning speed, out of
the turn lane, unless you signal for the turn, which then starts as any turn you signal for.

**Signal yourself where you can:** changing lanes and turning without signaling is illegal in most places. With your
blinker on, the car doesn't move by itself: you change lanes as without this setting (a nudge above 20 mph, or steer
yourself).

**First uses:** its camera thresholds were set without any real drives. Try it only after automatic right turns work well
for you, on quiet streets you know, with your hands on the wheel and full attention.

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
   (alpha)**, then, while parked, **auto left turns (alpha)** and **auto turn lanes (alpha)**, and **nav
   exit desires (alpha)**. All of them are also in the sunnylink app under Steering → Navigation.
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

## Phone GPS (alpha)

If the comma 4's own GPS hears little in your car (the navigation page's GPS details show few satellites, or no fix),
navigation can use your iPhone's GPS instead. It's off by default.

1. Install an iPhone app that shares the phone's GPS over Wi-Fi as NMEA, such as **GPS2IP**. In its settings choose
   **Socket** mode on port **11123**, keep **RMC** (and GGA) among its NMEA sentences, and turn on its background mode so it
   keeps running while you use other apps. Start it.
2. Connect the comma 4 to the iPhone's **Personal Hotspot**, as for the navigation page.
3. Settings → Models → turn on **phone GPS (alpha)** (it needs navigation (alpha); it's also in the sunnylink app under
   Steering → Navigation). While the car is on, the comma 4 connects to the phone.
4. Check it on the navigation page, under GPS details: the **Phone GPS** line says whether the comma is connected to the
   phone, how old the phone's last fix is, how accurate, and how many satellites the app reports, and the line after it
   which GPS navigation uses. If the phone refuses the port, the app isn't sharing in socket mode on 11123; if its data
   can't be read, set the iPhone to 24-hour time (GPS2IP writes a broken time otherwise).

Its limits:

- The app must keep running. If iOS stops it, or the comma leaves the hotspot, the phone's fixes stop, and within 2
  seconds navigation goes back to the comma's own GPS, then to the phone again once its fixes come back.
- The phone's fixes reach the comma about a second after they were taken, which the slowdown and turn timing allow for. A
  fix that arrives later than that, jumps farther than the car could drive, is worse than about 50 m (165 ft), or has no
  speed or course is not used.
- A phone in a pocket or a bag hears less: put it where it sees the sky, such as in a dashboard mount.
- The comma takes data only from the phone (the hotspot's gateway), never from another device on the hotspot.
- Only on a comma 4, beside its own u-blox GPS.

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
  turning speed to about 20 m past the corner. With *auto route turns*, the car keeps slowing, but a short signal that
  stops by itself counts as switching the blinker off: the car won't take that turn by itself either. Only when the car
  cancels the signal at the end of an openpilot lane change into the curb lane, or with *auto left turns* into a curbed
  bay, can the turn still start by itself (see *Automatic left turns*).

## First drives

1. Start in an empty parking lot or on quiet streets, at low speed, hands on the wheel.
2. With alpha longitudinal on, begin with **blinker turns only**. Set a destination around the block,
   signal for each turn about 8 seconds ahead (on roads faster than 45 mph, about 15 seconds ahead, from the lane
   you'll turn from), and check when **Slowing for Route Turn** and **Turning…** appear and whether the car picks the
   right opening.
3. Only after that, try **auto route turns**, still on quiet streets. Signal yourself anyway where you
   can: it's the law, and it warns pedestrians and cyclists.
4. Only after automatic right turns work well, try **auto left turns**, on quiet streets with curbed left-turn
   bays or one-way streets, with your foot over the brake.
5. Only after those work well, try **auto turn lanes**, on quiet streets you know, with right-turn lanes, hands on the
   wheel and full attention: its camera thresholds were set without real drives.
6. Only then try normal traffic. Different driving models may handle turns differently.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Page won't load | navigation (alpha) is on; the IP is right; your phone and the comma 4 are on the same Wi-Fi network (the page doesn't answer over the comma's cellular data) |
| "Route: waiting" | the device has internet; the car is on and has a GPS fix; a Mapbox token is saved; the car is on the route's road and heading along it, not in a parking lot beside it; after a missed turn, or after leaving the route, the new route can take up to about 10 seconds |
| No "Turning…" | your blinker matches the turn, you switched it on after the last turn, and it stayed on; you signaled about 8 seconds ahead (about 15 seconds on roads faster than 45 mph, from the lane you turn from), so the car could get down to turning speed before the corner; you signaled before pulling away, not while openpilot was still speeding the car up, as when pulling away from a light or after the gas; Tesla's cruise control is off (route turns don't start while it's on: use alpha longitudinal); no blind-spot warning; no lane change ending close to the corner; the car hadn't stopped inside the intersection; the turn isn't less than about 30 m after the last one; the car is still on the route's road; steering is engaged; nav turn desires is on |
| No slowdown | alpha longitudinal is on and engaged; with blinker turns, you signaled for the turn (a blinker on since the last turn doesn't count) and the blinker is still on; farther out than about 250 m (800 ft), the car is in the curb lane on the turn's side, which the model has seen for a second (else it waits until 250 m: above about 55 mph, slow down yourself first); you didn't press the gas or disengage on the approach; the car is still on the route's road |
| Nudge doesn't change lanes | the car is above 20 mph: once you signal for a route turn, the slowdown has it below that about 50 m before the turn, so move over before you signal |
| No automatic lane change toward the turn | from about 20 seconds before the route's next turn, a blinker toward it never starts an automatic (nudgeless) lane change, even before it counts as the turn signal: nudge to change lanes |
| Auto turn waits for the blinker | it's a left turn (those wait unless auto left turns is on: see the next row); alpha longitudinal is engaged and you didn't press the gas, disengage, or switch off a blinker you had on for the turn on the approach (a short signal that stops by itself counts too); the car is at 16 mph or less; no blind-spot warning; the model has seen the car in the curb lane for a full second, after any lane change |
| Signal for Route Turn / auto left turn waits | auto route turns and auto left turns are both on, and were on while the car last stood still; the car is in the lane right beside a raised median or curb on the left, not a painted turn lane, a center turn lane or a two-way street; the model has seen that curb for the last 10 m and 1.5 s, with no lane change and no car in the left blind spot since; the route's lane guidance doesn't put the turn in another lane; it's a plain left turn, not a sharp one or the end of a road; the posted limit is known and 45 mph or less over the last 150 m; driver monitoring isn't warning; the car didn't stop within about 30 m of the turn; you didn't switch off a left blinker, press the gas or signal right on the approach; no hazard lights; the turn could start at least 2 seconds before the car steers across, with the curb seen for the last 10 m by then. Once **Signal for Route Turn** has shown, that turn waits for your blinker; it also shows when a started left turn is handed back to you |
| No move into the turn lane | it's a right turn (left where traffic keeps left): it never moves for a left turn; auto turn lanes and auto route turns are on, and were on while the car last stood still; the model saw the car in the curb lane for a second and 10 m before the turn lane opened, for this turn, with no lane change and steering on since; the car came within 150 m of the turn no more than 50 m after it last had the curb beside it; the turn lane is at least 2.6 m wide, still open 30 m ahead, with a clear lane line before it and none beyond; it opened by about 70 m (20 mph) or 80 m (25-30 mph) before the turn; the car was between 6 and 33 mph; no blinker, no blind-spot warning, no road-edge block, and no driver monitoring warning in the last 3 seconds; lane guidance has the turn from the outermost lane; Auto Lane Change by Blinker isn't Off; it hasn't already moved, or been stopped, for that turn |
| Navigation goes on after you arrived | the car came within about 25 m (80 ft) of the destination; if you stopped farther away, clear the destination on the page |

## For developers

- `openpilot/sunnypilot/selfdrive/controls/lib/nav_desire.py`: `NavDesireController`, used by `DesireHelper`
  (`openpilot/selfdrive/controls/lib/desire_helper.py`) in both modelds, holds the one state for route turns: the turn and
  exit desires, what calls a turn off, and the route turn's speed limit, which modeld publishes as
  `modelDataV2SP.navTurnSpeed`. A turn starts only while the car can still make it and is still lined up with the approach
  (from the model's yaw rate): moving, before the steering point (8 m plus 0.3 s of travel), slow enough to reach turn
  speed by it and, under openpilot longitudinal, both at its speed and at the speed it still gains while plannerd eases out
  its acceleration (`v + a * (NAV_TURN_LAG + CarParams.longitudinalActuatorDelay) + a^2 / (2 * NAV_TURN_JERK)`, with
  `NavInputs.accel` the more of `aEgo` and the commanded accel), never while that is above 5 m/s and the car accelerates;
  or at a crawl, a near side turn only more than 5 m short of the maneuver point. Without openpilot longitudinal, none
  starts while the car's own cruise is engaged. Beyond the signal distance (15 m + 12 s, at most 250 m) the turn's blinker
  latches only from the outermost lane on its side (`outer_lane_time`), from the slowdown's start (with a 0.5 m/s margin on
  the speed); with automatic turns, an unlatched blinker toward the turn keeps the slowdown going. From 20 s before the
  turn (or its approach distance) a blinker toward it never starts a nudgeless lane change (`turn_blinker_ahead`, held by
  `DesireHelper.ahead_blinker_side` until it goes off). The driver switching a latched blinker off before the start (not
  within `FAR_CANCEL_TIME` of a lane change ending) sets `auto_off`, either side. A turn pulses again only to stay in the
  model's desire history (at a crawl judged with `ManeuverTracker.taken_back`, what navd lately pulled in as far as navd's
  distance now disagrees past the 3 m deadband, also for a re-latch), and it pins its turn point on the odometer, so its
  turn speed holds to 20 m past the point even once navd moves on; no other turn starts until then. While route turns are
  on and a route is set, the lane turn desire stays quiet. `ManeuverTracker` dead reckons the distance to each maneuver and
  corrects it by the median of navd's
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
  short steps at speed are followed; past a split, only once the car is clearly on the branch, judged at a fix, never on the
  line between two. A split is an exit or fork, a turn within 45° of the old road, or a turn whose route comes back 10-25 m
  beside the old road's straight continuation (a connector to a service road). Such a turn is decided by the car's course,
  which a GPS position error doesn't move: it is taken only after a fix, from 20 m before to 40 m past the maneuver point,
  whose course turned toward the new road by 15° (at most half the turn), and is missed 40 m past the point without one. Any
  other split still undecided where the old road's 300 m straight continuation ends, with the route ahead 4–25 m from it
  there, is missed. Going past a maneuver without taking it, being off the route, heading against the step far from its
  maneuver, going off along another road (two fixes in a row more than 12° off the step's direction and as far off the
  route's direction over 10–40 m behind or ahead of the car, which a map line rejoining the centerline at a median's end is
  not, each 1 m or more farther from it, while the maneuver is over 25 m ahead; back after two fixes within 6° and within
  4 m of the earlier offset, or on a new step), or having no fresh GPS fix makes the instruction invalid, and all but no fix
  count toward a reroute; a missed maneuver reroutes right away. A new destination gets its route at once; a reroute waits
  10 s after the last request. Arrival latches within 25 m of the destination or the route's end, on the route's last road
  step or past it; from then on there is no reroute, and once the car is more than 25 m from both, navigation ends: the
  destination is removed, unless the driver set another one meanwhile. Where the receiver rates its course
  (`bearingAccuracyDeg` above 0 and below 180), the first leaving fix and a split's turn-in must also exceed 2.5 times that
  accuracy.
- `openpilot/sunnypilot/navd/navweb.py`: the destination page (process `navweb`), with no authentication: it relies on a
  private Wi-Fi network. It answers only connections that arrive at 127.0.0.1 or at `wlan0`'s current IPv4 address, so
  never over the cellular link (a machine without `wlan0` answers on every address). At most 12 connections at a time,
  each closed 15 s after it opens, and 16 KB of headers.
- Automatic far side turns (`NavAutoTurnFarSide`, read only with `NavAutoTurn`): `nav_inputs` adds `far_lanes` (a
  sign-checked road edge 1.0-2.6 m beside the car with std under 0.5 m and no lane line beyond the car's own), the driver
  monitoring alert, and the posted limit from `longitudinalPlanSP.speedLimit.resolver` (`posted_speed_limit`: 0 unless seen,
  valid, at most 0.5 s old and positive; where a recent `carStateSP.speedLimit` is higher, that one, never making an unknown
  limit known; both modelds now subscribe to `longitudinalPlanSP` and `carStateSP`, and modeld checks none of its
  services). The setting comes on only at a standstill, or as modeld starts with the drive (`read_params`), and goes off at
  any time. `NavDesireController._auto_turn_allowed` keeps the near side rule as it was; a far side turn needs `_far_gates`
  (setting, openpilot longitudinal, a plain 'left' or 'right' modifier, not 'end of road', moving, 2 s or more to the
  steering point on the nearer distance, `far_turn_lane` not vetoing the outermost lane, no driver monitoring alert, the
  highest posted limit within 150 m known and at most 20.5 m/s) and the curb's run (`FAR_LANE_TIME`, `FAR_LANE_DISTANCE`,
  restarted by a lane change or that side's blind spot), through the same single start as every other turn. A stop within
  the approach distance before the start sets `auto_off`. `_may_pulse_again`, shared by keep-alives and a re-latched
  blinker, never pulses an automatic far side turn again while the car waits at a crawl or driver monitoring warns, and
  `_hold_turn` releases it `FAR_STOP_RELEASE` after its last pulse, at any speed (`handed_back`). `signal_prompt` asks from
  `FAR_SIGNAL_LEAD` before the trigger once `_far_auto_possible` (also false with a blinker or hazards on, above
  `AUTO_TURN_MAX_SPEED`, or when the rest of the curb's run, timed along the slowdown, cannot end `FAR_TURN_MIN_LEAD`
  before the steering point) has been false for `FAR_PROMPT_DEBOUNCE`, while stopped short of the turn, or after a hand-back
  on the move while the blinker could start it again as a keep-alive; once shown it latches for the maneuver (kept through
  `reset()`). modeld publishes `modelDataV2SP.navTurnAcross` (`turning` while an automatic far side turn is held, `signal`
  for the prompt), and selfdrived raises `navTurnAcross` in place of the lane turn alert, or `navTurnSignal`; both are low
  priority, under driver monitoring's, and `navTurnAcross` has its own sound (`AudibleAlertSP.prompt`).
- Automatic turn lanes (`NavAutoTurnLane`, read only with `NavAutoTurn`; like far side turns, it comes on only at a
  standstill or as modeld starts), near side turns only: on the far side a median's or an island's end, with the road going
  on two-way, gives the model the same edges and lines as a curbed bay opening, so nothing there moves the car.
  `nav_inputs` adds `road`, a `RoadView` of both road edges' points and stds and the lane lines' `y[0]` and probabilities
  (`road_view` never raises). `_watch_turn_lanes` latches, on the near side only, that the car was confirmed in the curb
  lane (`outer_lane_time` 1 s and `outer_lane_distance` 10 m) with the edge's distance averaged over 2 s as the baseline,
  for the current maneuver (a new one clears it, and `lane_carry`), forgotten on a lane change, steering off, once the
  turn has started, or 50 m after the edge was last close beside the car, whether a turn lane is open there or not.
  `turn_lane_open` needs the sign-checked edge at least 2.6 m beyond the baseline at every model point to min(30 m, the
  distance to the maneuver less 10 m), its std under 0.5, the line between at least 0.5 sure 1.0-2.4 m from the car's
  center and 2.6 m or more from the edge, the line beyond under 0.2, everything finite; held for 0.5 s (`pocket_time`).
  `_turn_lane_allowed` holds the move's gates (a near side turn, the automatic turn still possible, `far_turn_lane` as a
  veto, driver monitoring quiet for `POCKET_DM_QUIET` 3 s since the move's `preAlert` is its first warning's sound, too,
  2.5-15 m/s, within 150 m, the move, 3.5 s at the current speed, ending 5 m before the trigger by `time_to_steer`, navd
  agreeing, no turn pulse in the model's 5 s desire history) and sets `turn_lane`; with the setting off, all of it is
  forgotten.
  `DesireHelper` starts `laneChangeStarting` toward it with no blinker and below 20 mph (not against relc's edge block,
  nor with Auto Lane Change Off) and calls `start_turn_lane`; `_turn_lane_holds` ends it (either blinker, the gas,
  steering against it, openpilot longitudinal off, a new maneuver, or that blind spot before the lane line between came
  0.5 m nearer), and the blinker that ends it starts no lane change of its own. That ends only `DesireHelper`'s state and
  the alert: both modelds feed the model only the desire's rising edge, so the model finishes the lane change unless the
  driver steers against it. So once `lane_move_maneuver` is the current maneuver, the call-off block sets only `auto_off`
  for the gas, the other blinker or longitudinal off (never `called_off`, nor popping the pin), and `_update_speed_limit`
  keeps slowing whatever the blinkers. This `DesireHelper` has no `laneChangeFinishing`: the move completes as any lane
  change ends, `lane_change_prob` under 0.02 after 0.5 s, and then `end_turn_lane(True)` sets `lane_carry`, which holds
  `outer_lane_time` at its threshold while the per-frame test passes; nothing else in the start rules changes. Once per
  maneuver (`lane_move_maneuver`, kept through `reset()`). modeld publishes `modelDataV2SP.navTurnLane`, and selfdrived
  raises `navTurnLane` (sound `preAlert`) in place of `laneChange`, dropping `navTurnSlowing` meanwhile, which plannerd's
  `NavTurnSpeedController` holds back while `navTurnLane` is set, so it shows after the move.
- `openpilot/sunnypilot/navd/phonegpsd.py`: phone GPS (process `phonegpsd`), run while the car is on with navigation,
  `PhoneGps` and a u-blox (`ublox_available()`, so never beside qcomgpsd, which publishes `gpsLocation` too). It connects
  over TCP to `PhoneGpsPort` (11123) on `wlan0`'s default gateway from `/proc/net/route`, the iPhone on its hotspot, or on
  `PhoneGpsHost`, from `wlan0`'s address, retrying after 1, 2, 4 and then every 8 s, and takes UDP datagrams on `wlan0`'s
  address from that address only. It reads NMEA 0183 from any talker, checksums verified, joining each epoch's RMC
  (status, time, date, position, speed, course) with its GGA (quality, satellites, HDOP, altitude) and a VTG, and publishes
  `gpsLocation` (source `iOS`, valid) once per fix, as it arrives, never an older one: `horizontalAccuracy` 5 m per unit of
  HDOP (at least 3 m), `bearingAccuracyDeg` atan(0.7 m/s / speed), so that navd's course rules widen their thresholds for a
  slow car (never 0 or 180, which navd takes at its word), and the moving car's last course below 1 m/s. The NMEA time
  only tells late data: a fix arriving more than 1 s later than the phone's fastest lately, against the monotonic clock,
  is dropped; it goes out as `unixTimestampMillis` only within 1 s of the comma's clock, else the arrival's. navd counts a
  fix's age from `logMonoTime` (its arrival), clamped to 1.5 s, so a clock difference never moves a position. After 5 s
  without a fix, two in a row must agree in time and place before one goes out. Not used: status V or a no-fix mode,
  quality 0, an empty or (0, 0) position, HDOP over 10, no speed (or no course at 1 m/s and up), a jump beyond 1.5 times
  the faster speed's travel plus 30 m. Its link and what comes in go to `PhoneGpsStatus` for navweb's Phone GPS line.
  `latest_gps_source` takes a fresh phone fix over the u-blox's while it lasts, so the two never alternate.
- Params: `NavTurnDesire`, `NavAutoTurn`, `NavAutoTurnFarSide` and `NavAutoTurnLane` (neither backed up), `NavExitDesire`,
  `MapboxToken` (not logged), `PhoneGps` (backed up), `PhoneGpsPort`, `PhoneGpsHost`, `PhoneGpsStatus` (phonegpsd's). Events: `navTurnSlowing`, `navTurnAcross`, `navTurnSignal`, `navTurnLane`.
