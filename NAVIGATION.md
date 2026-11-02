# Route-assisted turns and exits

This branch (`nav`) adds route-assisted turns and exits to sunnypilot. It uses sunnypilot's
built-in navigation (alpha) and adapts StarPilot's navigation desires. Every setting below is off
by default; with them off, the car drives like plain sunnypilot.

## What it does

- **Turns with your blinker** (*nav turn desires*): with a destination set, **signal for the
  route's next turn**. About 3 seconds before the car starts steering into it, and only below
  20 mph (32 km/h), the car gives the driving model its "turn" desire. The screen shows
  **Turning Left / Turning Right**. If you stop short of the turn, at a red light for example, the
  car repeats the desire every few seconds so the model still knows about the turn when you go.
- **Lane changes:** while your blinker is on for the route's next turn, openpilot won't start an
  *automatic* (nudgeless) lane change, however much you slow down. A deliberate nudge still changes
  lanes, for example into a turn lane, until the turn begins; the turn then waits for the lane change.
- **Slowing for turns:** when openpilot controls the speed (alpha longitudinal), the car slows to
  about 10 mph by the turn and holds that speed through it. The screen shows **Slowing for Route
  Turn** with a chime. With blinker turns, this starts once you signal for the turn.
- **Automatic turns** (*auto route turns (alpha)*): the car slows for **every** route turn and turns
  **without your blinker**. This needs alpha longitudinal. With it, openpilot's driving model also
  brakes on its own and tries to stop for red lights and stop signs, though not reliably. It only
  turns on its own when:
  - it has slowed to turning speed (16 mph or less) and the turn is about 3 seconds ahead;
  - the blind spot on that side is clear;
  - for a right turn, the model sees the car in the rightmost lane;
  - for a left turn, the route shows no choice of turn lanes.

  Otherwise it slows and waits for your blinker.
- **Exits and forks (alpha):** near a route exit or fork that has lane guidance, a light steering
  nudge toward it gives a "keep left/right" desire. Above 20 mph with your blinker on, a nudge
  changes lanes instead, the same as without navigation.

**Calling a turn off:** press the gas, or switch on the opposite blinker. Each turn is planned
once. After you call it off, it only comes back if you switch your blinker on for it.

**GPS:** route turns need a GPS fix. In a tunnel or garage they pause until the fix is back.

It does not signal for you, handle U-turns or roundabouts, change lanes to get into a turn lane, or
judge gaps in oncoming traffic. The model decides exactly where to turn, so stay ready to take over.

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
3. For the slowdown and automatic turns, turn on Settings → Developer → **alpha longitudinal**
   (read its warning: it may disable automatic emergency braking).
4. If you use **Pause Lateral Control with Blinker** (sunnylink → Steering), turn it off. Otherwise
   steering pauses exactly when you signal for a turn.
5. **Routing:** create a free account at mapbox.com and copy your default public access token
   (`pk.…`). You'll paste it into the navigation page below. Without a token, the device tries comma's
   own routing, which may not be available without comma prime.

## Setting a destination

The comma 4 serves a small page for your phone on port **8082**.

1. Connect the comma 4 to your phone's hotspot. Its IP address is shown in the comma 4's Wi-Fi
   settings. (Your phone can also join the comma 4's own hotspot at `192.168.43.1`, but then the comma
   needs its own cellular data for routing.)
2. On your phone, open `http://<comma IP>:8082`. Paste your Mapbox token once and tap Save.
3. Search for an address, or paste a Google Maps / Apple Maps share link (best for businesses) or
   coordinates like `37.7749, -122.4194`.
4. The comma 4 shows the next maneuver and the distance to it. The destination clears when you turn the
   car off.

The page has no password: anyone on the same network as the comma 4 can open it, so only enable
navigation on your own hotspot or home Wi-Fi.

## Tesla notes

- **Brake = steering off.** With the standard Tesla harness, sunnypilot forces "disengage steering on
  brake". With alpha longitudinal, the turn slowdown means you don't need the brake for a turn. With
  Tesla's own cruise control, slow down with its speed control instead.
- **Turn speed.** openpilot caps lateral acceleration at about 3 m/s², and the Tesla limits how fast
  the steering wheel turns. That allows about 10 mph for a tight right turn, about 15 mph for a left
  turn, and walking pace for very tight corners. Faster, the car runs wide, which is why the slowdown
  aims for 10 mph and blinker turns only start below 20 mph.
- **Grip.** Holding the wheel firmly makes the Tesla's steering system drop steering control.

## First drives

1. Start in an empty parking lot or on quiet streets, at low speed, hands on the wheel.
2. Begin with **blinker turns only**. Set a destination around the block, signal for each turn, and check
   when **Turning…** appears and whether the car picks the right opening.
3. Then try the slowdown with alpha longitudinal, and only after that **auto route turns**, still on quiet
   streets. Signal yourself anyway where you can: it's the law, and it warns pedestrians.
4. Only then try normal traffic. Different driving models may handle turns differently.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Page won't load | navigation (alpha) is on; the IP is right; your phone is on the same network |
| "Route: waiting" | the device has internet; the car is on and has a GPS fix; a Mapbox token is saved |
| No "Turning…" | your blinker matches the turn; you're under 20 mph about 3 seconds before it; no blind-spot warning; steering is engaged; nav turn desires is on |
| No slowdown | alpha longitudinal is on and engaged; with blinker turns, you signaled for the turn; you didn't press the gas |
| Auto turn waits for the blinker | alpha longitudinal is engaged and you're not pressing the gas; the car is at 16 mph or less; for a right turn, the car is in the rightmost lane; for a left turn, the route shows no separate turn lanes |

## For developers

- `openpilot/sunnypilot/selfdrive/controls/lib/nav_desire.py`: route-confirmed and automatic turn
  desires and exit keep desires, used by `DesireHelper` (`openpilot/selfdrive/controls/lib/desire_helper.py`)
  in both modelds. `ManeuverTracker` keeps a distance per maneuver that only shrinks, shared with the slowdown.
- `openpilot/sunnypilot/selfdrive/controls/lib/nav_turn_speed.py`: the turn slowdown, a target in
  `LongitudinalPlannerSP` (source `navTurn`) plus a deceleration candidate in the longitudinal planner.
- `openpilot/selfdrive/navigation/navd.py`: `MapboxToken` param, lane guidance in `navInstruction`,
  access token kept out of logs, and an invalid instruction without a fresh GPS fix.
- `openpilot/sunnypilot/navd/navweb.py`: the destination page (process `navweb`).
- Params: `NavTurnDesire`, `NavAutoTurn`, `NavExitDesire`, `MapboxToken` (not logged). Event: `navTurnSlowing`.
