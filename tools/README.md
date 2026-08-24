# D1 Max Console

A local operator console for the D1 Max. Python stdlib only — no `pip install`,
no npm, no build step. Runs on Linux, macOS and Windows with Python 3.9+.

It implements the ZSKJ wire protocol directly ([docs/dev/02](../docs/dev/02-wire-protocol.md))
rather than using the vendor `librobot_sdk.so`, which is why the same code will
later port to Android and iOS ([docs/dev/04](../docs/dev/04-clients-linux-android-ios.md)).

```
tools/
  d1max_proto.py     framing + JSON codec + command table   (self-testable)
  d1max_client.py    connection, heartbeat, watchdog, telemetry decode
  d1max_sim.py       a fake robot, for working without hardware
  d1max_console.py   HTTP + SSE server, serves the UI
  console_ui.html    the operator UI
```

---

## Try it with no robot

```bash
python3 tools/d1max_console.py --sim
```

Opens <http://127.0.0.1:8770>. The simulator speaks the real protocol —
handshake, heartbeat, command acks, control ownership, 1004/1102 telemetry — and
integrates your drive commands into a pose, so the position readout moves when
you drive. Do this first: it verifies your setup end to end and lets you learn
the UI before a 41 kg machine is involved.

## Connect to the real robot

1. **Close the RC app.** If it's running it holds control and the SDK is refused.
2. Join the robot's Wi-Fi: SSID `XG2WIFI_xxxxxx`, password `12345678`.
3. Run:

```bash
python3 tools/d1max_console.py --host 192.168.234.1        # Wi-Fi AP
python3 tools/d1max_console.py --host 192.168.168.168      # wired
```

4. Press **CONNECT**.

Reach the wired subnet while on Wi-Fi (lets one machine talk to both boards):

```bash
sudo ip route add 192.168.168.0/24 via 192.168.234.1       # Linux/macOS
route -p ADD 192.168.168.0 MASK 255.255.255.0 192.168.234.1  # Windows
```

### From a phone

```bash
python3 tools/d1max_console.py --host 192.168.234.1 --bind 0.0.0.0
```

Then browse to `http://<your-laptop-ip>:8770` from a phone on the same network.
The layout is responsive and the STOP / E-STOP bar pins to the bottom of the
screen. This is the quickest way to evaluate the mobile ergonomics from
[docs/dev/06](../docs/dev/06-inspection-security-ux.md) before writing any native code.

---

## Using it

**Order matters.** Issuing commands out of sequence can make the robot fall or
stop responding (`docs/source/2.8`). A safe first session:

1. **TAKE CONTROL** — check the strip turns green and reads `YOU HOLD CONTROL`.
2. **Speed: Low** — pin it. Low gives full yaw authority; high cuts it to a third.
3. **Stand** — wait for `MOTION STATUS` to settle.
4. **Mode: General**.
5. Drive with a low gain and short taps.

### Driving

`W A S D` translate · `Q E` yaw · `Space` stop · `Esc` e-stop. Or hold the
on-screen pad buttons.

**Hold to move.** Releasing stops the robot. Three independent layers enforce
this, deliberately:

| Layer | Behaviour |
|---|---|
| Browser | Key-up, pointer-up, tab-hide and window-blur all zero the setpoint |
| Python watchdog | Setpoint older than 350 ms is transmitted as zero |
| Robot | Discards any move command older than 1 s |

A crashed browser tab, a stuck key, or a dead Wi-Fi link all end with the robot
stopping. Don't add a "cruise" mode — it would defeat all three.

**Gain** scales the normalised `[-1, 1]` teleop value before it is sent. At
gain 0.20 with speed level Low you get roughly 0.2 m/s. Start there.

### The authority strip

Always visible at the top:

- **Control holder** — green when you hold it, amber when the App does. If it
  isn't green your motion commands are being ignored, and a banner says so.
- **Speed level** — highlighted amber at medium/high, because the same stick
  deflection means something very different.
- **Mode** and **motion status**.
- **Link RTT** — from the heartbeat echo. Amber over 120 ms, red over 250 ms.
- **Battery** — both packs.

### STOP vs E-STOP

- **STOP** zeroes the setpoint. Routine, use it constantly.
- **E-STOP** sends `emergency/stop`. The robot stops responding and lowers
  itself. The button becomes **CLEAR E-STOP** to recover.

The UI e-stop is a convenience. **The safety case is the physical e-stop on the
robot and the RC handset** — have one within reach.

### Frame log

Every command sent and every ack, fault and ownership change received.
Heartbeats and teleop frames are suppressed, since at 5 and 50 Hz they'd drown
everything else. This is your protocol debugger.

---

## Before you trust the axes

The protocol document contradicts itself on `lx`/`ly`
([docs/dev/02](../docs/dev/02-wire-protocol.md)): the prose calls `lx` a
left/right axis while the same sentence says positive is forward. This console
follows the reading that `lx` is forward/back and `ly` is lateral, matching the
SDK's `Move(left_right, forward_back, yaw)` signature and `example/control.cpp`.

**Verify before relying on it.** Speed Low, gain 0.10, open space, hand on the
physical e-stop, one short tap forward. If the robot strafes instead of walking
forward, swap `lx`/`ly` in `console_ui.html`'s `KEYMAP` and the pad's
`data-drive` attributes, and tell me so the docs get corrected.

---

## Verifying the install

```bash
python3 tools/d1max_proto.py --self-test     # 37 offline codec checks
python3 tools/d1max_console.py --sim         # full stack, no hardware
```

The simulator can also reproduce the preemption case, which is worth seeing
once so you recognise it in the field:

```bash
python3 tools/d1max_sim.py --port 8098 --app-holds-control
python3 tools/d1max_console.py --host 127.0.0.1 --port 8098
```

The console connects, reports `APP HAS CONTROL`, shows the amber banner, and
`TAKE CONTROL` is refused with `App holds control` — exactly as the real robot
behaves, since the SDK may never preempt the App.

---

## Known limits

- **UDP only.** The WebSocket transport on port 8081 isn't implemented; the UDP
  path on 8082 is the SDK default since v0.1.1.
- **No video.** RTSP won't play in a browser. Use
  `ffplay rtsp://192.168.234.1:8554/front`, or for low latency the GStreamer
  pipeline in `docs/source/4.1`.
- **No map or mission running.** That needs the SLAM work in
  [docs/dev/03](../docs/dev/03-slam-mapping-plan.md) and the executor in
  [docs/dev/05](../docs/dev/05-mission-system.md).
- **Single client.** One browser at a time is the intended use.
- **No auth.** Bind to `127.0.0.1` unless you're on a trusted network; `--bind
  0.0.0.0` exposes robot control to anyone who can reach the port.
