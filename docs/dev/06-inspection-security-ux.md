# 06 — UX/UI for Inspection & Security Missions

Design guidance for operator-facing software on the D1 Max, targeting two
concrete jobs: **scheduled inspection** and **autonomous security patrol**.

Everything here is constrained by the platform findings in
[doc 01](01-platform-architecture.md) and [doc 04](04-clients-linux-android-ios.md)
— preemptible control ownership, 1-second command expiry, speed-level-dependent
actuator authority, and a bandwidth budget that forbids shipping raw sensor data
to a phone.

## The central framing: these are two products

Inspection and security look similar — a robot drives a route unattended — and
teams routinely build one UI for both. That's a mistake. They differ in the thing
that actually matters.

| | **Inspection** | **Security** |
|---|---|---|
| Trigger | Scheduled, predictable | Event-driven, or randomised schedule |
| Is a human watching? | No — reviews afterwards | No — *responds* during |
| Deliverable | A dataset comparable across runs | An alert, evidence, and a response |
| Critical metric | **Pose repeatability** | **Time-to-takeover** |
| Dominant failure | Images that can't be compared | Event missed, or response too slow |
| Primary screen | Findings diff | Takeover |
| Time budget | 3 seconds per waypoint reviewed | 10 seconds from alert to hands-on |

They share the substrate — maps, missions, waypoints, telemetry, the safety layer
— and diverge completely above it. Build the substrate once; build two
foregrounds.

If you only build one first, **build inspection**. Its loop is scheduled and
forgiving, so you can get the map, mission and capture pipeline correct without
anyone needing a 3 a.m. response to work. Security is inspection plus latency
pressure plus evidence handling.

---

## What the operator actually does

Design against the real daily loop, not the demo.

**Inspection operator, most days:**
1. Morning: did the overnight runs finish?
2. Work the findings queue — perhaps 60 waypoints, 5 minutes total.
3. Escalate 2–3 items.
4. Occasionally re-record a waypoint whose framing has drifted.

They spend ~95% of their time in a **review queue**, and almost none watching a
robot drive. A UI whose centrepiece is a live map is optimised for the demo, not
the job.

**Security operator, most nights:** nothing happens. Then:
1. Alert at 03:00.
2. Phone in hand, half awake, see the clip.
3. Decide: real or not?
4. If real — take control, investigate, challenge over audio, dispatch a human.
5. Log it.

Every tap between the push notification and live video-plus-control is a design
failure. That single path is the product.

---

## Cross-cutting: the persistent safety furniture

These appear on every screen, in the same place, always. They exist because of
specific platform behaviours that cause accidents.

### The authority strip

A single always-visible bar answering four questions:

```
┌────────────────────────────────────────────────────────────┐
│ ● YOU HOLD CONTROL   SPEED: LOW   GENERAL   ⟲ 34ms   🔋 78% │
└────────────────────────────────────────────────────────────┘
```

- **Who holds control.** The App can preempt you silently at any moment
  (protocol `1016`). If the strip doesn't say you hold control, nothing you do
  reaches the robot. Colour-code hard: green you, amber someone else, red unknown.
- **Speed level.** The same stick deflection is 1 m/s or 3 m/s depending on a
  setting made ten minutes ago, and at high speed yaw authority is *cut* to a
  third. This is the most dangerous piece of hidden state on the platform.
- **Mode.** General / in-place / stair. Determines what the controls even do —
  head pan/tilt only exists in in-place mode.
- **Link quality.** Derived free from the heartbeat round-trip echo. An operator
  driving into a degrading link needs to know *before* it fails.

### E-stop vs. Abort — two buttons, never one

A distinction most robot UIs collapse, and shouldn't:

- **ABORT** — graceful. Stop the mission, decelerate, hold position. Recoverable,
  routine, no drama. This is what an operator wants 95% of the time.
- **E-STOP** — immediate. Robot stops responding and lowers to the ground.
  Disruptive and occasionally damaging to whatever it's carrying.

Make them look different — abort is a normal button, e-stop is red, larger, and
visually "armed". Put e-stop in the thumb zone, never in a menu, never behind a
confirmation. Confirmation belongs on *recovering* from e-stop, not entering it.

**The phone is never the safety case.** The physical e-stop on the robot and the
RC handset are. UI e-stop is a convenience, and the interface should not imply
otherwise.

### Hold-to-drive

The protocol expires `Move` after one second, which is already a dead-man switch.
Mirror it in the UI: virtual sticks spring to centre, driving requires continuous
contact, release means stop. **Never build a cruise or latched-velocity mode** —
it defeats the one safety property you get for free.

---

## Feature set: inspection

### Record-by-driving authoring (build this first)

The single highest-value authoring feature, and the one that justifies a mobile
app at all. The operator teleops the robot along the route and drops waypoints at
the current pose.

It wins on three counts: it needs no map-reading skill; every waypoint is
provably reachable *because the robot just reached it*; and the operator frames
each shot with their own eyes.

When a waypoint is dropped, capture the **full observation state**, not just x/y/θ:

- pose (x, y, θ) and the map it belongs to
- head pan/tilt angle
- which camera (front/rear), and head/tail orientation
- fill-light state at capture
- ambient lux reading
- speed level and mode on approach
- **the reference image itself** — shot now, by the operator, deliberately framed

That reference image is what every future run gets compared against. Everything
else exists to reproduce the conditions under which it was taken.

### Pose repeatability is a first-class UI concept

If run N+1 parks 40 cm from where run N did, the images don't compare and the
finding is noise. So:

- Capture waypoints get a **tighter tolerance** than travel waypoints — treat
  ~5–10 cm and a few degrees as the target, versus 25 cm for transit.
- Every captured image carries its **pose error** as metadata.
- The review UI **shows** that error, and visibly de-weights comparisons taken
  from a bad pose. An operator must never be asked to judge a difference that's
  actually a parallax artefact.

This one requirement propagates backwards into the localisation and controller
work in [doc 03](03-slam-mapping-plan.md). It's worth knowing early that
"inspection" means centimetre-grade repeatability, not "gets there eventually".

### The findings queue, not a gallery

The review screen is the product. Design it for **3 seconds per waypoint**.

- One waypoint at a time, full screen. Not a grid of thumbnails.
- **Blend/wipe slider between reference and current.** For comparing two photos
  of the same physical scene, a draggable wipe beats side-by-side every time —
  the eye catches changes that survive the wipe. Side-by-side forces saccades and
  misses small deltas.
- Optional delta-highlight overlay, off by default (it's noisy under changing
  light).
- Three actions only: **OK · Flag · Escalate**. Swipe-right for OK, since that's
  the overwhelming majority.
- Metadata inline and small: waypoint name, timestamp, pose error, lux, light
  state.
- History scrubber — this waypoint across the last N runs. Slow drift (corrosion,
  a settling crack, a slowly-emptying tank) is invisible run-to-run and obvious
  across ten.

### Lighting as a controlled variable

Four fill lights plus a lux sensor. For repeatable imagery, lighting must be part
of the waypoint, not ambient chance. Offer per-waypoint: `auto · on · off ·
on-at-capture-only`.

`on-at-capture-only` is the useful default for scheduled work — navigate dark,
light up briefly for the shot, go dark again. It saves power and, for security,
keeps the robot unobtrusive.

### Scheduling and endurance

- Recurring schedules, per mission.
- **Pre-flight budget check**: "this mission needs ~40 min; robot has 2 h 10 m."
  Warn or block when it doesn't fit. Endurance is 5 h unloaded, 3.5 h loaded.
- Dual batteries report separately — one pack degraded is a warning, not an
  abort. Show both.
- Return-to-dock threshold as mission policy.

---

## Feature set: security

Everything above still applies. These are the additions, and they're mostly about
latency and evidence.

### One tap from alert to hands-on

The whole product, in one requirement. Push notification → deep link → takeover
screen with live video already connecting and control acquisition already
requested. Not: notification → app → fleet list → robot → live → take control.

If the robot is mid-mission, **taking control must be a single action** that both
pauses the mission and acquires control. Two separate steps here is the difference
between a 4-second and a 20-second response.

### On-robot detection, not streamed video

This follows directly from the bandwidth analysis in [doc 03](03-slam-mapping-plan.md).
Nobody is watching a live feed at 03:00, and you can't afford to ship one
continuously anyway. Detection runs on the Orin NX — person detection, motion in a
defined zone, door state, thermal anomaly if fitted — and the client receives
**alerts with a short pre/post clip**, not streams.

The 157 TOPS on the Orin exists for exactly this. Use it.

### Randomised patrol timing

A patrol on a fixed schedule and fixed route is a patrol you can walk around. Offer:

- randomised start within a window ("between 02:00 and 03:00")
- randomised waypoint ordering, subject to route feasibility
- optional randomised dwell

Small feature, meaningful security value, and it costs almost nothing once the
mission executor exists.

### Patrol in the dark

LiDAR doesn't need light. The robot navigates perfectly well with every lamp off —
which means lights become a *tactical choice* rather than a necessity. Expose it
plainly: dark patrol by default, lights on demand, lights-at-capture-only. An
operator taking control at night should be able to arrive unlit and then choose to
light up.

### Two-way audio

Some units carry a microphone and speaker (`docs/source/1.3` lists it as
version-dependent — verify on your hardware). If present, push-to-talk from the
takeover screen is high-value: challenging a trespasser remotely resolves a large
fraction of incidents without dispatching anyone. Treat it as optional and
capability-gated, not assumed.

### Evidence handling

Security output is potentially evidentiary. Even a light-touch version pays off:

- Immutable clips and stills — write-once, never edited in place.
- Timestamps from a disciplined clock, with the source recorded.
- Location: map pose, plus RTK global coordinates when available.
- An append-only incident log: what triggered, who acknowledged, what they did.
- Export as a bundle.

### Geofencing

Draw no-go regions on the map. **Enforce them on the Orin NX**, in the executor,
not in the client — a constraint that only exists in the UI isn't a constraint.
With RTK available, geofences can be specified in real-world coordinates and shared
with other systems.

---

## Screen inventory and layouts

### Layout principle: one thing is primary

The most common robot-UI mistake is a four-pane dashboard with video, map,
telemetry and controls all at 25%, all too small to use. Decide what's primary per
screen and let the rest be a PIP the operator can promote with one tap.

- **Teleop / takeover** → video primary, map PIP.
- **Mission monitor** → map primary, video PIP.
- **Findings review** → imagery primary, everything else metadata.

### Takeover (mobile, landscape)

```
┌──────────────────────────────────────────────────────────────┐
│ ● APP HAS CONTROL    LOW   GENERAL   ⟲ 34ms   🔋 78%         │  authority strip
├──────────────────────────────────────────────────────────────┤
│ ┌────────┐                                          ┌──────┐ │
│ │ minimap│         live video — front camera        │ 120ms│ │  latency badge
│ │  ⌖     │                                          └──────┘ │
│ └────────┘                                                   │
│                                                              │
│            ┌────────────────────────────────┐                │
│            │  ▶ PAUSE MISSION & TAKE CONTROL│                │  single action
│            └────────────────────────────────┘                │
│                                                              │
│   ╭─────╮                                          ╭─────╮   │
│   │  ⊕  │   💡  📷  🎙  ⏺                          │  ⊕  │   │  hold-to-drive
│   ╰─────╯   lights cam ptt rec                     ╰─────╯   │
│  translate                                           yaw     │
│                                                              │
│              [ ABORT ]         (( E-STOP ))                  │  thumb zone
└──────────────────────────────────────────────────────────────┘
```

Landscape, two-thumb. Sticks in the bottom corners where thumbs rest. E-stop and
abort centre-bottom, reachable by either thumb, visually distinct from each other.
Latency badge sits *on* the video because that's where the operator's eyes are.

### Mission monitor (mobile, portrait)

```
┌────────────────────────────────┐
│ ● YOU  LOW  GENERAL  ⟲34ms 78% │
├────────────────────────────────┤
│                                │
│      2-D occupancy grid        │
│                                │
│   ●━━━●━━━●━━━◉╌╌╌○╌╌╌○        │  done ━ current ◉ pending ╌
│                 ⌖ robot        │
│                    ┌─────────┐ │
│                    │ video   │ │  PIP, tap to promote
│                    └─────────┘ │
├────────────────────────────────┤
│ Step 12/40 · Capture "Valve 3" │
│ ETA 18 min · battery −22% est. │  budget vs. actual
├────────────────────────────────┤
│ ◀ 11 Transit │ 12 ▶ │ 13 Look  │  step strip, scrollable
├────────────────────────────────┤
│      [ ABORT ]    (( E-STOP ))  │
└────────────────────────────────┘
```

Route coloured by status is the single most legible progress indicator — better
than a percentage bar, because it also shows *where*.

### Findings triage (mobile, portrait)

```
┌────────────────────────────────┐
│ Valve Array 3   ·  4 of 62     │
├────────────────────────────────┤
│                                │
│      ┌──────────┬─────────┐    │
│      │ reference│ current │    │
│      │          │         │    │  draggable wipe
│      └──────────┴─────────┘    │
│              ◀ ║ ▶             │
│                                │
│  ⊙ delta overlay   ⊙ history   │
├────────────────────────────────┤
│ 03:14 · pose err 4cm ✓         │  green: comparison trustworthy
│ lux 240 · lights on            │
├────────────────────────────────┤
│  [ ✓ OK ]  [ ⚑ Flag ]  [ ↑ Esc ]│
└────────────────────────────────┘
```

Swipe right = OK and advance. The pose-error line is what tells the operator
whether to trust their own eyes.

### Alert (mobile, arrives as push)

```
┌────────────────────────────────┐
│ ⚠ PERSON DETECTED              │
│ Loading Dock · 03:14:22        │
├────────────────────────────────┤
│                                │
│      [ 6-second clip ]         │
│           ▶                    │
│                                │
├────────────────────────────────┤
│  📍 map pin · zone "Dock East" │
├────────────────────────────────┤
│ ┌────────────────────────────┐ │
│ │   TAKE CONTROL NOW      ▶  │ │  one tap to takeover
│ └────────────────────────────┘ │
│  [ Acknowledge ]  [ Dismiss ]  │
└────────────────────────────────┘
```

### Desktop

Desktop earns its screen area on the things mobile can't do well:

- **Mission authoring on the map** — place, reorder, and tune waypoints; set
  per-step properties; preview the route.
- **Findings at scale** — a run's 60 waypoints in a filterable table, trend charts
  per waypoint across runs, bulk export.
- **Map management** — geofences, no-go zones, dock location, landmark
  annotation, map versioning.
- **Fleet and scheduling** — multiple robots, calendars, alert routing and
  escalation ladders.

---

## Mobile-specific constraints

- **One-handed operation.** The operator's other hand may be holding a door, a
  torch, or a radio. Keep everything critical in the bottom 40%.
- **Landscape for driving, portrait for everything else.** Don't force rotation
  for review or monitoring.
- **Gloves and rain.** IP67 robot implies outdoor work. Large touch targets, no
  precision drag gestures for anything safety-relevant.
- **Sunlight.** High-contrast mode, and don't encode critical state in colour
  alone — the authority strip needs text as well as colour.
- **Backgrounding kills the connection.** Send protocol `1050` and disconnect on
  background; require an explicit reconnect. A robot must never be driven by an
  app the operator can't see.
- **Bind the socket to the Wi-Fi network explicitly.** The robot's AP has no
  internet, and Android will happily route your traffic to cellular otherwise
  ([doc 04](04-clients-linux-android-ios.md)).

---

## Platform-specific UI affordances worth exposing

Things this robot can do that a generic AMR UI wouldn't think to offer:

- **Head/tail reversal.** The robot can swap which end leads, so it can reverse
  out of a dead-end corridor rather than three-point-turning. Expose it in mission
  authoring for passages narrower than the turning circle.
- **Stair mode.** Multi-floor inspection routes need a per-step mode hint; the UI
  should make the mode change visible, since robot behaviour changes noticeably.
- **Posture set.** `slim` for narrow gaps (50 cm minimum passage), `climb` for
  high platforms, `dsb` for kerbs, `crawl` for low clearance. These are route
  properties an author should be able to attach to a segment.
- **Look-at steps.** Head pan/tilt only works in in-place mode, so a "look" step
  implies a mode switch in and out. Handle it automatically, but *show* it — the
  robot stopping and changing stance is otherwise alarming to watch.
- **Low-confidence zones.** The ultrasonics only cover left and right at 0–4 m,
  and LiDAR famously does not see glass. Glass doors and glazed partitions are
  common in exactly the office and lobby environments security patrols cover.
  Let operators mark these regions on the map so the executor slows down and the
  reviewer knows why.

---

## What not to build

- **Full mission-graph editing on a phone.** Record-by-driving covers the mobile
  case; complex editing belongs on desktop.
- **Point clouds streamed to a client.** Two 96-line LiDARs at 10 Hz will not fit
  down a phone link. Clients get the 2-D grid and a pose.
- **A 3-D map view as the primary navigation UI.** It demos beautifully and is
  harder to read than a 2-D grid. Offer it as an inspection tool, not the default.
- **Auto-take-control on connect.** Always an explicit operator action.
- **Cruise / latched velocity.** Defeats the 1-second dead-man.
- **A unified inspection+security dashboard.** The two jobs want different
  foregrounds; forcing one screen makes both worse.

---

## Build order

1. **Safety furniture** — authority strip, e-stop, abort, hold-to-drive. Nothing
   ships without these, and everything else is built on top.
2. **Takeover screen** — video, teleop, latency. Also your protocol test-bed.
3. **Record-by-driving authoring** — unlocks real missions and needs no map editor.
4. **Mission monitor** — run and watch what you authored.
5. **Findings triage** — the inspection deliverable. Wipe slider, three actions.
6. **Alerts + one-tap takeover** — turns it into a security product.
7. **Desktop review and scheduling** — scale.
8. **Fleet** — only once one robot works end to end.

Steps 1–5 are a complete inspection product. Step 6 is most of a security one.
