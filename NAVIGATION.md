# Route-assisted turns and exits

This branch (`nav`) adds route-assisted turns and exits to sunnypilot. It uses sunnypilot's
built-in navigation (alpha) and adapts StarPilot's navigation desires. Every setting below is off
by default; with them off, the car drives like plain sunnypilot.

## What it does

- **Turns with your blinker** (*nav turn desires*): with a destination set, **signal for the
  route's next turn**. The car gives the driving model its "turn" desire once that intersection is
  close: about 6 seconds ahead, between 35 and 90 m, and only below 31 mph (50 km/h). The screen
  shows **Turning Left / Turning Right**.
- **Slowing for turns:** when openpilot controls the speed (alpha longitudinal), the car slows to
  about 10 mph by the turn and holds that speed through it. The screen shows **Slowing for Route
  Turn** with a chime. With blinker turns, this starts once you signal for the turn.
- **Automatic turns** (*auto route turns (alpha)*): the car slows for **every** route turn and turns
  **without your blinker**. This needs alpha longitudinal. With it, openpilot's driving model also
  brakes on its own and tries to stop for red lights and stop signs, though not reliably. It only
  turns on its own when:
  - it has slowed to turning speed (16 mph or less) and the intersection is close;
  - the blind spot on that side is clear;
  - for a right turn, the model sees the car in the rightmost lane;
  - for a left turn, the route shows no choice of turn lanes.

  Otherwise it slows and waits for your blinker.
- **Lane changes:** while your blinker is for the upcoming route turn, it won't start a *nudgeless*
  lane change. A deliberate nudge still changes lanes, for example into a turn lane.
- **Exits and forks (alpha):** near a route exit or fork that has lane guidance, a light steering
  nudge toward it (without the opposite blinker on) gives a "keep left/right" desire.

**Calling a turn off:** press the gas, or switch on the opposite blinker. Each turn is planned
once. After you call it off, it only comes back if you signal for it.

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

1. Put your phone and the comma 4 on the same network. Either the comma 4 joins your phone's hotspot
   (its IP address is shown in the comma 4's Wi-Fi settings), or your phone joins the comma 4's
   hotspot, where the address is always `192.168.43.1`.
2. On your phone, open `http://<comma IP>:8082`. Paste your Mapbox token once and tap Save.
3. Search for an address, or paste a Google Maps / Apple Maps share link (best for businesses) or
   coordinates like `37.7749, -122.4194`.
4. The comma 4 shows the next maneuver and the distance to it. The destination clears when you turn the
   car off.

## Tesla notes

- **Brake = steering off.** With the standard Tesla harness, sunnypilot forces "disengage steering on
  brake". With alpha longitudinal, the turn slowdown means you don't need the brake for a turn. With
  Tesla's own cruise control, slow down with its speed control instead.
- **Turn speed.** Tesla's steering safety limits allow tight turns only at low speed: roughly
  ≤ 12 mph for a right turn and ≤ 17 mph for a left turn, which is why the slowdown aims for 10 mph.
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
| "Route: waiting" | the device has internet; the car is on (GPS); a Mapbox token is saved |
| No "Turning…" | your blinker matches the turn; you're under 31 mph and within ~90 m; no blind-spot warning; steering is engaged; nav turn desires is on |
| No slowdown | alpha longitudinal is on and engaged; with blinker turns, you signaled for the turn; you didn't press the gas |
| Auto turn waits for the blinker | alpha longitudinal is engaged and you're not pressing the gas; the car is at 16 mph or less; for a right turn, the car is in the rightmost lane; for a left turn, the route shows no separate turn lanes |

## For developers

- `openpilot/sunnypilot/selfdrive/controls/lib/nav_desire.py`: route-confirmed and automatic turn
  desires and exit keep desires, used by `DesireHelper` (`openpilot/selfdrive/controls/lib/desire_helper.py`)
  in both modelds.
- `openpilot/sunnypilot/selfdrive/controls/lib/nav_turn_speed.py`: the turn slowdown, a target in
  `LongitudinalPlannerSP` (source `navTurn`) plus a deceleration candidate in the longitudinal planner.
- `openpilot/selfdrive/navigation/navd.py`: `MapboxToken` param, lane guidance in `navInstruction`,
  access token kept out of logs.
- `openpilot/sunnypilot/navd/navweb.py`: the destination page (process `navweb`).
- Params: `NavTurnDesire`, `NavAutoTurn`, `NavExitDesire`, `MapboxToken`. Event: `navTurnSlowing`.
