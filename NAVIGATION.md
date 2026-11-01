# Route-assisted turns and exits

This branch (`nav`) adds route-assisted turns and exits to sunnypilot. It uses sunnypilot's
built-in navigation (alpha) and adapts StarPilot's navigation desires.

## What it does

- **Turns:** with a destination set, **signal for the route's next turn**. The car gives the driving
  model its "turn" desire once that intersection is close: about 6 seconds ahead, between 35 and 90 m,
  and only below 31 mph (50 km/h). The screen shows **Turning Left / Turning Right**.
- **Lane changes:** while your blinker is for the upcoming route turn, it won't start a *nudgeless*
  lane change. A deliberate nudge still changes lanes, for example into a turn lane.
- **Exits and forks (alpha):** near a route exit or fork that has lane guidance, a light steering
  nudge toward it (without the opposite blinker on) gives a "keep left/right" desire.

It does **not** turn without your blinker, slow the car down for turns, handle U-turns or
roundabouts, or change lanes for the route on its own. The model decides exactly where to turn,
so stay ready to steer.

## Install on a comma 4

1. **Back up your settings:** comma 4 → sunnylink → *backup settings*. Uninstalling wipes the device's
   settings and downloaded models.
2. **Uninstall:** Settings → Software → *uninstall*.
3. On the setup screen, choose **Custom Software** and enter `installer.comma.ai/SeanLouei/nav`.
4. The first boot builds everything, including the driving model, which takes a while (roughly
   15–30+ minutes). Keep the device powered.
5. Pair sunnylink again, use *restore*, and download your driving model again if you use a custom one.

## One-time setup

1. Settings → Developer → turn on **navigation (alpha)**.
2. Settings → Models → turn on **nav turn desires**. Optionally turn on **nav exit desires (alpha)**.
   Both toggles are also in the sunnylink app under Steering → Navigation.
3. If you use **Pause Lateral Control with Blinker** (sunnylink → Steering), turn it off. Otherwise
   steering pauses exactly when you signal for a turn.
4. **Routing:** create a free account at mapbox.com and copy your default public access token
   (`pk.…`). You'll paste it into the navigation page below. Without a token, the device tries comma's
   own routing, which may not be available for your device.

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
  brake". If you press the brake to slow for a turn, steering drops. Slow down with the TACC speed
  control instead, or use openpilot longitudinal (alpha) with experimental mode so the car slows itself.
- **Turn speed.** Tesla's steering safety limits allow tight turns only at low speed: roughly
  ≤ 12 mph for a right turn and ≤ 17 mph for a left turn. Faster than that, the car will run wide.
- **Grip.** Holding the wheel firmly makes the Tesla's steering system drop steering control.

## First drives

1. Start in an empty parking lot or on quiet streets, at low speed, hands on the wheel.
2. Set a destination around the block. Signal for each turn and check when **Turning…** appears, and
   whether the car picks the right opening.
3. Only then try normal traffic. Different driving models may handle turns differently.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Page won't load | navigation (alpha) is on; the IP is right; your phone is on the same network |
| "Route: waiting" | the device has internet; the car is on (GPS); a Mapbox token is saved |
| No "Turning…" | your blinker matches the turn; you're under 31 mph and within ~90 m; no blind-spot warning; steering is engaged; nav turn desires is on |

## For developers

- `openpilot/sunnypilot/selfdrive/controls/lib/nav_desire.py`: route-confirmed turn and keep desires,
  used by `DesireHelper` (`openpilot/selfdrive/controls/lib/desire_helper.py`) in both modelds.
- `openpilot/selfdrive/navigation/navd.py`: `MapboxToken` param, lane guidance in `navInstruction`,
  access token kept out of logs.
- `openpilot/sunnypilot/navd/navweb.py`: the destination page (process `navweb`).
- Params: `NavTurnDesire`, `NavExitDesire`, `MapboxToken`.
