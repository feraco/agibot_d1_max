# 04 — Client Applications: Linux, Android, iOS

## The shipped library cannot go on mobile

Before planning anything, settle this, because it determines the whole
architecture. The vendor ships `librobot_sdk.so` for `x86_64` and `aarch64`. The
`aarch64` build looks like it might work on Android. It cannot. Verified against
the binary in this repository:

```
$ file librobot_sdk.so.0.1.1
ELF 64-bit LSB shared object, ARM aarch64, version 1 (GNU/Linux)

$ readelf -d librobot_sdk.so.0.1.1 | grep NEEDED
  NEEDED  libstdc++.so.6
  NEEDED  libgcc_s.so.1
  NEEDED  libc.so.6
  NEEDED  ld-linux-aarch64.so.1

$ readelf -V librobot_sdk.so.0.1.1 | grep -oE 'GLIBCXX_[0-9.]+|CXXABI_[0-9.]+' | sort -u
  CXXABI_1.3 … 1.3.13
  GLIBCXX_3.4 … 3.4.20 …

$ nm -D --defined-only librobot_sdk.so.0.1.1 | awk '$2=="T"{print $3}' | grep -vc '^_Z'
  0
```

Four independent blockers:

1. **Wrong libc.** It links `libc.so.6` and `ld-linux-aarch64.so.1` — glibc.
   Android uses bionic and `linker64`. The library will not load.
2. **Wrong C++ runtime.** GNU `libstdc++` with versioned `GLIBCXX_*`/`CXXABI_*`
   symbols. The NDK provides LLVM `libc++`.
3. **No C ABI.** *Zero* unmangled exported functions — all 142 exported symbols
   are Itanium-mangled C++. Even a JNI or FFI shim would have to be compiled
   against the exact same libstdc++ ABI, which returns you to blocker 2.
4. **No iOS build exists at all.** No Darwin object, no framework, no source.

There is no repackaging trick, no `patchelf` fix, no static-linking workaround.
The library is not source-available, so it cannot be rebuilt for either platform.

### Which is fine, because the protocol is open

[Doc 02](02-wire-protocol.md) documents the entire wire format: a 16-byte header
and a JSON body. A complete, correct client is on the order of 300 lines. The
vendor publishing `Protocol-1.2.0.pdf` as an "对外开放版本" (externally-released
version) is an explicit invitation to do exactly this.

**Decision: reimplement the protocol. Do not use `librobot_sdk.so` anywhere,
including on Linux.** Using it on desktop and a reimplementation on mobile means
maintaining two client behaviours with divergent bugs, and the desktop one would
be the opaque binary you can't debug. One implementation, everywhere.

## Architecture: one core, many shells

```
┌─────────────────────────────────────────────────────────────────┐
│                          d1max-core                             │
│                        (Rust, no_std-friendly)                  │
│                                                                 │
│  framing · JSON codec · connection state machine · heartbeat    │
│  teleop transmit loop · watchdog · ownership tracking           │
│  telemetry decode · fault decode · mission client               │
└──────┬──────────────┬──────────────┬──────────────┬─────────────┘
       │ C API        │ JNI          │ UniFFI /     │ PyO3
       │              │ (cargo-ndk)  │ swift-bridge │
       ▼              ▼              ▼              ▼
  ┌─────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐
  │  Linux  │   │ Android  │   │   iOS    │   │  Python  │
  │ GUI/CLI │   │  Kotlin  │   │  Swift   │   │  tooling │
  │ ROS 2   │   │          │   │          │   │  / ROS 2 │
  └─────────┘   └──────────┘   └──────────┘   └──────────┘
```

**Why Rust for the core.** The safety-critical parts — a 5 Hz heartbeat that must
never stall, a 50 Hz control loop, a watchdog that zeroes velocity — are
concurrency problems. Rust gives real threads and timers on all four targets with
one implementation, and its cross-compilation story to both Android and iOS is
mature. `tokio` for the async runtime, `serde_json` for the codec.

The alternative — C++ with a hand-rolled C shim — works and reuses more idiom from
the vendor examples, but you'll write the Android and iOS build plumbing yourself
and get no memory-safety guarantees in a loop that drives a 41 kg machine.

### Core responsibilities

The core owns everything that must be correct regardless of UI:

| Concern | Behaviour |
|---|---|
| Framing | Encode/decode `ZSKJ` header, `msg_id` allocation, request correlation |
| Connection | State machine per `ConnectionState`; reconnect only on unexpected loss |
| Heartbeat | 5 Hz on a dedicated timer, independent of UI thread health |
| Teleop | Fixed-rate loop, 50 Hz active / 5 Hz idle |
| **Watchdog** | If no fresh setpoint within N ms, transmit zero velocity. Non-negotiable |
| Ownership | Track `1016`/`1017` + `control_source`; expose as an observable |
| Safety guard | Reject illegal state transitions before they reach the wire |
| Telemetry | Decode `1004`/`1005`/`1100`/`1101`/`1102` into typed structs |
| Sensor config | Re-send `1008` enables after every reconnect |
| Teardown | Send `1050` on disconnect and on mobile backgrounding |

The UI layer should be incapable of putting the robot in an unsafe state, because
it never touches the socket. It publishes a desired setpoint; the core decides
what actually goes out and at what rate.

### Alternative worth considering: Flutter + `flutter_rust_bridge`

If shipping one UI across Linux, Android and iOS matters more than native feel,
Flutter over the same Rust core is a strong fit. One Dart UI covers all three
targets; the Rust core still owns the real-time work. Flutter's `RawDatagramSocket`
could even carry the protocol in pure Dart — but don't: you'd lose the shared
safety layer, which is the entire point of the core.

Recommendation: **Rust core + native shells** if the mobile apps are the primary
operator interface and you want platform-idiomatic gesture and background
behaviour. **Rust core + Flutter** if you want three platforms from one UI codebase
and can accept custom-rendered controls. Either way the core is the same, so this
decision can be deferred until after phase 1 — which is a good reason to build the
core first.

## Per-platform notes

### Linux

The fastest place to iterate, and where the ROS 2 integration lives.

- CLI first: connect, handshake, telemetry dump, keyboard teleop. This is your
  protocol conformance test-bed and it costs almost nothing.
- Then a GUI. If it needs to render maps and point clouds alongside ROS 2 data,
  RViz plugins get you a long way for free. For a purpose-built operator console,
  egui or Slint bind cleanly to a Rust core.
- Expose the core to Python via PyO3. Mission scripting, bag post-processing and
  test automation are all far pleasanter in Python, and the ROS 2 nodes from
  [doc 03](03-slam-mapping-plan.md) can share the same tested protocol code.

### Android

- Rust core via `cargo-ndk` → `.so` per ABI (`arm64-v8a` is the only one that
  matters in practice) → JNI → Kotlin.
- **Video:** ExoPlayer handles RTSP H.264. Expect latency; the low-latency
  GStreamer pipeline in `docs/source/4.1` has no direct ExoPlayer equivalent. If
  latency proves unacceptable for teleop, `libvlc-android` or a bundled GStreamer
  gives more control at the cost of app size.
- **Networking:** the phone must associate to the robot's AP (`XG2WIFI_xxxxxx`).
  Android will notice the network has no internet and may route traffic over
  cellular — bind your socket to the Wi-Fi `Network` object explicitly via
  `ConnectivityManager.bindProcessToNetwork()` or per-socket binding. This is a
  classic and very confusing failure mode: everything works on Wi-Fi-only devices
  and mysteriously fails on phones with a SIM.
- **Lifecycle:** on background, send `1050` and drop the connection. Do not run a
  robot control loop in a background service. If the operator can't see the robot,
  the robot should not be moving.

### iOS

- Rust core as a static library (`aarch64-apple-ios`, plus
  `aarch64-apple-ios-sim`), wrapped with UniFFI or swift-bridge, packaged as an
  XCFramework.
- **`NSLocalNetworkUsageDescription` is required** in `Info.plist`, and iOS 14+
  shows a local-network permission prompt. If the user declines, the app silently
  cannot reach the robot — detect and explain this rather than showing a generic
  connection failure.
- **Video:** `AVPlayer` does not support RTSP. Options: bundle `MobileVLCKit`
  (simplest, large), bundle GStreamer (most control), or run a transcoder on the
  Orin NX republishing as HLS or WebRTC. **WebRTC is the right long-term answer**
  for both mobile platforms — genuinely low latency, NAT-friendly, and one
  implementation serves Android and iOS alike. It's more work up front and worth
  scheduling deliberately rather than discovering late.
- **Background:** iOS suspends aggressively. Same rule as Android — send `1050`,
  disconnect, and require an explicit reconnect.
- Consider whether a controller app that commands a 41 kg robot is going to the
  App Store or distributed via TestFlight/enterprise. That affects how much
  latitude you have around local networking and background modes.

## The safety layer is a product feature

This is not boilerplate. A 41 kg quadruped that does 3 m/s under `Move()` hurts
people. Whatever the UI, these belong in every client:

- **E-stop as a permanent, unmissable UI element.** Not in a menu. Always visible,
  always one tap, on every screen.
- **Dead-man interaction.** Movement continues only while a control is actively
  held. Release → zero velocity. This maps naturally onto the protocol's 1-second
  command expiry, which is already a dead-man switch — don't defeat it.
- **Explicit control acquisition.** Never auto-take control on connect. Require a
  deliberate operator action, and show clearly who holds control right now.
- **Loud ownership loss.** `1016` arriving means the App preempted you. Full-screen,
  unambiguous, mission aborted.
- **Connection quality always visible.** Use the heartbeat round-trip time. An
  operator driving on a degrading link needs to know before it fails, not after.
- **Speed level always visible.** The same stick deflection means 1 m/s or 3 m/s
  depending on a mode the operator may have set ten minutes ago.
- **Fault surfacing.** `1005` faults, especially `FatalError` level, and the
  battery-low codes that precede an automatic protective stop.

A useful design test: *if this app is force-killed mid-command, what does the robot
do?* With the protocol's 1-second expiry the answer is "decelerates and stops",
which is correct. Any feature that would change that answer is a bug.

## Build order

**Phase 1 — Core + Linux CLI.** Protocol, state machine, heartbeat, watchdog,
telemetry decode. Keyboard teleop matching `example/control.cpp`. Validated
against real hardware. *Exit: you can drive the robot with zero vendor binary
code.*

**Phase 2 — Language bindings.** Python (PyO3) for tooling and the ROS 2 nodes.
Prove the JNI and Swift paths with a trivial connect-and-read app on each before
building UI on top of them.

**Phase 3 — Operator app.** Teleop, video, telemetry, faults, e-stop. This is the
first thing anyone other than you will use, and the safety layer above is most of
the requirement.

**Phase 4 — Map & mission UI.** Consumes derived products from the Orin NX — 2-D
grid, robot pose, mission state — never raw point clouds. See
[doc 05](05-mission-system.md).

**Phase 5 — Fleet.** Multiple robots, saved maps, mission libraries, run history.
Only worth designing once one robot works end to end.
