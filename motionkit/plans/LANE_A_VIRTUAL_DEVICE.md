# Lane A — Virtual device: RKD6, virtual MCU, step generation (handoff)

**Goal:** the native execution path down to an actuator, fully virtual and
hardware-ready. By the end of this lane, a MotionKit plan runs this chain in
simulation: runtime → RKD6 over a simulated link → the real `no_std` device
core → a virtual stepper → a SimKit joint. It survives clock drift, link
faults and underflow with controlled stops. Adding the nucleo board later
means writing a board layer and measuring timing, nothing more.

Read first:
1. `motionkit/plans/CONTRACTS.md`: all of it, especially C1 (events), C5
   (device core boundary) and the cross-lane rules. §P0 must be on `main`.
2. `robotkit/runtime/DEVICE_PROTOCOL.md`: RKD5, the capacity math and both
   timing budgets.
3. `robotkit/device_protocol/src/runtime.rs` (the `Device` trait,
   `DeviceProtocol`), `src/device_wire.rs`, and `src/bin/pty_device.rs`.
4. `robotkit/runtime/src/device_host.cpp` and the `DeviceSerialEndpoint`
   files. Also the queue, STOP, HOLD and underflow code in
   `robotkit/runtime/src/runtime.cpp`.
5. `motionkit/IMPLEMENTATION_PLAN.md`: decisions D1–D4, Ground rules, and
   the P6–P9c log entries (how the queue, sessions and serial qualification
   behave today).
6. `robotkit/ARCHITECTURE.md`, "Deployment boundary" and the P11
   transmission design section.

Work in `../materia-lane-a` on branch `lane-a-virtual-device`, created from
`main` after §P0 has merged. Follow the ground rules and cross-lane rules
(failing test first, one commit per item, merge to `main` after each green
item). Append to the Progress log at the end of this file. Stop and log
whenever this plan turns out to be wrong.

## Lane decisions

- **LA-D1 — The device interpolates, the host doesn't stream.** The device
  receives polynomial segments with commit and replace rules, and evaluates
  them on its own clock. Host-side per-cycle setpoint streaming (P9c)
  remains, but only for *cyclic-control* endpoints (simulation today,
  EtherCAT-style drives later). It is not a device wire protocol.
- **LA-D2 — Step generation runs on the device, on a fixed timer tick.**
  - A fixed-rate step tick (default 40 kHz, configurable per board) evaluates
    the segment position, in `f32` with segment-local τ.
  - It emits a step whenever the position crosses the next step boundary
    (a DDA-style base thread, as in LinuxCNC and grblHAL stepgen).
  - Step timing jitter is at most one tick. The bound is documented and
    tested.
  - Klipper-style host step compression is a possible later optimisation for
    very high step rates. It is out of scope here.
- **LA-D3 — The in-process virtual device is the primary test vehicle.** Per
  C5, the device core plus the virtual board layer runs inside the host
  process under the SimKit clock, through a simulated link, and is fully
  deterministic. The PTY binaries remain for cross-process wire tests.
- **LA-D4 — RKD6 is a new protocol version and becomes the only one.**
  - Sync marker `RKD6`. The device protocol version is negotiated in session
    begin.
  - RKD5 keeps working only until RKD6 is proven end to end (A5–A6). No
    deployed RKD5 devices exist, only test harnesses and the nucleo stub, so
    A9 then retires RKD5 completely.
  - Simple devices use RKD6's minimal profile (A9) instead of a second
    protocol.
  - The device wire schema lives in `robotkit/schema/` next to
    `device_wire.wire.idl`, generated the same way.

---

## A1 — Board-layer split and MCU-target CI build

Problem: `Device` covers only instantaneous targets, stop, reset and read, and
the virtual devices live only in test binaries.

Do:
- **Board trait.** Extend the core with a board trait (keep `Device` as a
  compatibility alias or wrapper if that's simpler) that exposes:
  - a monotonic device clock (`now_ticks`, `tick_hz`);
  - actuator outputs (position target, velocity target, and a step+direction
    pulse per actuator);
  - digital and analog channel outputs;
  - `stop_all`.
- **Virtual board** (feature `std`, or a sibling crate
  `robotkit-device-virtual`):
  - a simulated clock with configurable tick rate, offset and drift in ppm;
  - N virtual steppers (step counter, direction, steps per unit);
  - virtual channels;
  - a recording of every output with its device-clock timestamp.
- **CI:** a script `robotkit/device_protocol/tools/check-mcu-build.sh` runs
  `cargo build --target thumbv7em-none-eabihf -p robotkit-device-protocol`
  and the nucleo board crate. Add it to the RobotKit test instructions and
  the `run-all` path if one exists.

  If the target isn't installed and there's no network access, stop and log
  it. Don't skip silently.

Tests:
- existing Rust tests and the PTY test pass unchanged;
- the virtual clock applies drift correctly (1000 ppm over 1 s gives 1 ms);
- the core compiles for the MCU target.

## A2 — RKD6 wire schema: sessions, time sync, segments, queue control

Do:
- **Schema** `robotkit/schema/device_wire6.wire.idl`, generated like RKD5
  into both `device_wire6.rs` and `device_wire6.hpp`. Messages:
  - `SESSION_BEGIN6` / `SESSION_ACK6`: adds protocol version, device tick
    rate, device queue capacity (segments and events) and step tick rate.
  - `TIME_SYNC_REQUEST(host_send_ns)` / `TIME_SYNC_REPLY(host_send_ns,
    device_rx_ticks, device_tx_ticks)`.
  - `QUEUE_BEGIN(queue_revision, replace_after_ticks, expected start
    state)`.
  - `SEGMENT`:
    - t0 in device ticks and duration in ticks;
    - degree ≤ 5 (the device may declare a lower maximum in its ack);
    - per-joint `f32` coefficients in segment-local seconds;
    - the plan id and `ends_at_rest`.
  - `COMMIT(through_ticks)`.
  - `HOLD`, `RESUME`, `ABORT`, `STOP`, `EMERGENCY_STOP`, `RESET_SAFETY`, with
    the same semantics as the runtime (P8 and follow-ups).
  - `QUEUE_STATUS` (device → host): queue revision, committed-until,
    executing plan id and segment, path clock, rate, remaining capacity,
    underflow and fault flags.
  - `STATE6`: the RKD5 state fields plus the device path clock and
    per-actuator step counts.
  - Reserve an `EVENT` message id for A7. Don't define its payload yet.
- **Framing:** CRC, maximum frame size and the capacity/timing table in a new
  `robotkit/runtime/DEVICE_PROTOCOL6.md`. Link RKD5's doc to it.

Tests:
- round-trip encode/decode vectors, shared between Rust and C++ as the RKD5
  frame vectors are (`tests/frame_vectors.rs` pattern);
- malformed-frame rejection per message.

## A3 — Clock mapping (host ↔ device)

Do:
- **Host side:** a clock estimator in C++ (`robotkit/runtime/src/`).
  - It sends `TIME_SYNC_REQUEST` periodically and keeps the replies with the
    lowest round-trip time.
  - It fits `device_ticks = offset + rate × host_ns` by least squares over a
    sliding window.
  - It exposes an uncertainty bound: half the minimum round trip plus the fit
    residual.
- **Mapping to device time.** Plan and segment times convert to device ticks
  through the mapping. The committed horizon on the host adds `link latency +
  2 × uncertainty`.
- **Loss of sync:** if the uncertainty exceeds a deployment bound, the host
  stops committing new segments and reports a fault reason `clock_sync_lost`.
  Motion already committed runs to its end.

Tests (virtual link and clock, deterministic):
- converges with a 1000 ppm drift and a 50 ms offset;
- stays within the bound under ±200 µs link jitter;
- detects a clock step (injected jump) and reports `clock_sync_lost`;
- mapped segment start times land within the bound on the device.

## A4 — Device segment queue, evaluator and on-device controlled stop

Do (Rust core, fixed capacity, no heap):
- **Segment queue:**
  - revision and replace-after rules identical to the runtime's plan
    replacement; replacement before `committed_until` is rejected;
  - evaluation on the device path clock with rate control (HOLD and RESUME
    change the path-clock rate within per-joint acceleration limits carried
    in `SESSION_BEGIN6`);
  - the P8 declared-final-knot rule;
  - underflow on a declared continuation ramps from the executed velocity and
    raises the underflow flag;
  - ABORT follows the path to a rest-declared end when that is shorter;
  - ramp setpoints are clamped to travel limits (`ramp_limit`).
- **Evaluator:** `f32`, segment-local τ, Horner form. Its outputs are the
  per-actuator position and velocity setpoints for the board layer.
- **Link loss:** if no host frame arrives within the RKD6 link-loss timeout,
  the device does a controlled stop along the path. It does not hold the last
  target. Then it latches `link_lost`. Motion never resumes automatically.
- **Shared test vectors** (`robotkit/device_protocol/tests/segment_vectors`,
  JSON or a binary fixture that both sides load):
  - segments with expected position and velocity at listed times;
  - generated from `motionkit_core` (`f64`), with the expected `f32` error
    bound;
  - the C++ runtime and the Rust device both test against them.

Tests: replace-after accepted and rejected, HOLD and RESUME limits, underflow
ramp, final-knot rule, link-loss stop, and the vector suite on both sides.

## A5 — Host device compiler and `Rkd6Endpoint`

Do:
- **Device compiler** (C++ in `robotkit/runtime/src/`): converts the
  runtime's queued polynomial segments into RKD6 segments.
  - time → device ticks via A3;
  - coefficients → `f32`, re-expanded in segment-local τ;
  - checks the device's maximum degree.
  - **Re-validates the converted trajectory** with `mk_validate` at the
    device's tick resolution and within the deployment's `target_error`
    (contract C5). A failure is a submit-time rejection, not a runtime
    surprise.
- **`Rkd6Endpoint`:** a `RobotEndpoint` that forwards plans instead of
  sampling.
  - Streams segments ahead of the device path clock, within device queue
    capacity.
  - Sends `COMMIT` as the host's committed horizon advances.
  - Maps `QUEUE_STATUS` into the runtime snapshot: session state, committed
    until, active plan, and underflow.
  - The runtime must not also sample and stream targets for this endpoint.
    Add an endpoint capability "device executes queue" and branch on it in
    the owner loop.
- **Deployment:** `deployment.json` v3 adds `protocol: "rkd5" | "rkd6"`
  (A9 later removes `"rkd5"`),
  step tick rate, link-loss timeout and the clock-sync bound. The v2 reader
  is kept.
- **Serial qualification:** extend the P9c check for RKD6. The link must
  sustain the segment rate needed to stay ahead of the path clock at the
  device queue depth. Unqualified configurations fail at construction and
  name the minimum baud or queue depth.

Tests (in-process virtual device, see A6 for the SimKit loop):
- a Ruckig plan executes on the device;
- device setpoints match `mk_trajectory_evaluate` within the `f32` bound;
- a replacement at the host committed horizon is applied on the device;
- a late one is rejected on both sides;
- snapshot fields reflect device queue status.

## A6 — In-process virtual device endpoint and SimKit loop

Do:
- Build the core plus the virtual board as a `staticlib` with a C ABI
  (`rkd_virtual_create`, `_destroy`, `_step(device_ticks)`,
  `_link_host_to_device(bytes)`, `_link_device_to_host(buffer)`,
  `_actuator_positions`, `_channel_values`).
- Wire it into the CMake build the same way the PTY binaries are built with
  cargo today.
- **`VirtualDeviceEndpoint` (C++):** owns a virtual device and a simulated
  link with configurable baud, latency, jitter, drop rate and corruption rate
  (seeded RNG). It steps the device clock from the SimKit owner clock, with
  the device's drift applied.
- **SimKit coupling:**
  - `Simulation.addRobot` gains an option to put a `VirtualDeviceEndpoint`
    between the runtime and the simulated plant.
  - The virtual steppers' positions (step count / steps-per-unit, then
    through the transmission) drive the SimKit joints kinematically.
  - `struct_size`-versioned C descriptor, with the Haxe wrapper and
    `MotionSystem` unchanged.

Tests:
- A MachineKit `LinearAxis`-compiled axis runs a Ruckig move through the
  whole chain. The SimKit joint position tracks the planned trajectory within
  one step plus the `f32` bound.
- It is deterministic: two runs give identical step logs.
- **Fault injection:**
  - 5% frame drop and ±500 µs jitter: the motion completes and the commit
    horizon absorbs it;
  - link cut mid-move: device-side controlled stop, then `link_lost`,
    observed in the snapshot;
  - clock drift of 2000 ppm: stays in sync.

**Plan C interaction (editor assembly simulation).** A separate plan
(Plan C, run from another session) simulates MachineKit assemblies in the
editor through `AssemblySimulationBridge`. It uses one link per part, and
joint → joint couplings such as lead screw → carriage live in the
`RobotModel` as `JointCoupling` (model schema v5). Once that is on `main`:
- add an A6 test that runs the virtual-device loop on the bridge's physical
  assembly model as well as on the `MachineKitRobotCompiler` model;
- the leader joint is driven through the actuator transmission, and the
  coupled follower joint must track `offset + ratio × leader` within
  tolerance.

Until then, the compiler model is enough. Don't build a parallel coupling
mechanism here: couplings belong to the model, and A7's joint → actuator
conversion stage is where the runtime applies them.

## A7 — Step generation (LA-D2) and transmissions end to end

Do:
- **Step generator** in the core: the step tick evaluates the actuator
  position, and step and direction outputs follow the crossing rule.
  - Direction setup time is configurable, in ticks.
  - A step-rate limit per actuator is derived from `maxRate` and the steps
    per unit. The device compiler rejects plans that would exceed it.
- **Transmissions (P11 design → implementation):**
  - The device layout maps channels to **actuators**, and the runtime
    converts joint targets to actuator targets through `SimpleTransmission`
    (`actuator = (joint − offset) × ratio`).
  - Update the fingerprint to cover the actuator layout.
  - The RKD6 session carries actuator count and steps per unit.
  - A dual-drive joint has two actuators. Skew is `|actuator₁/ratio₁ −
    actuator₂/ratio₂|` in joint units. Beyond a deployment bound it latches a
    `dual_drive_skew` fault.
  - Update `robotkit/ARCHITECTURE.md`'s P11 design section from "design" to
    "implemented".
  - **Couplings:** if Plan C's `JointCoupling` (model v5) is on `main` when
    this item runs, apply couplings at this same conversion stage:
    - follower joint targets are derived from their leader;
    - a plan whose follower trajectory violates a coupling is rejected at
      submit.

    If it isn't on `main` yet, leave a clearly marked extension point and
    note it in the log. Transmissions (actuator ↔ joint) and couplings
    (joint ↔ joint) stay distinct concepts.

Tests:
- For a lead-screw axis at 200 steps/rev × 16 microsteps with an 8 mm lead,
  the step count at each tick equals `floor(position × steps_per_mm)` within
  ±1.
- Step intervals never fall below the rate limit.
- A dual-drive gantry joint gets two actuators stepping in proportion.
- An injected missed step on one virtual stepper trips `dual_drive_skew`.
- The fingerprint changes when the actuator layout changes.

## A8 — Events on the RKD6 wire (after Lane B's B2 is on `main`)

Do:
- Define the `EVENT` payload: device-tick time in path time, channel index
  and value.
- The device fires an event when its **path clock** crosses the event time,
  with C1's HOLD, STOP and replacement semantics, including safe values on
  STOP and fault.
- Channel declarations go into the session and the fingerprint.
- The host compiler converts plan `TimedEvent`s alongside the segments.

Tests:
- a spray-on / spray-off event pair fires at the planned path positions
  within one step tick, with and without a HOLD in between;
- STOP sets the channel to its safe value;
- a replacement discards later events.

## A9 — Retire RKD5; minimal RKD6 device profile (after A5 and A6 are green)

Problem: two device protocols means two host code paths, two qualification
rules, two fingerprint schemes and two test suites. Nothing deployed uses
RKD5: only the PTY harnesses and the nucleo stub, which drives no pins. This
is the cheapest point to remove it, before any hardware exists.

Do:
- **Minimal profile.** The `SESSION_ACK6` device capabilities get a profile:
  - `full`: segment degree up to the device maximum, a device queue, step
    generation;
  - `minimal`: maximum degree 1, a small queue (for example 4–8 segments),
    no step generation, and outputs as position setpoints only.

  The host device compiler lowers plans to degree-1 segments at the owner
  period for `minimal` devices. A5's re-validation applies at that degree and
  resolution. Qualification (A5) covers both profiles. An unqualified
  configuration still fails at construction and names the minimum baud,
  queue depth or period.
- **Migrate:**
  - the bench deployment `robotkit/deployment/bench-nucleo-g474re/` and the
    fixtures under `robotkit/tests/fixtures/`;
  - the PTY binaries (`pty_device`, `robotd_pty_device`), the nucleo stub
    firmware (as a `minimal`-profile device driving its two virtual joints),
    and every RKD5 test.

  Deployment schema v4 drops `protocol`, since RKD6 is implied. The v3
  reader accepts only `"rkd6"`. Recompute the fingerprints.
- **Delete:**
  - the RKD5 schema (`robotkit/schema/device_wire.wire.idl` and its generated
    Rust and C++);
  - `HostLink`'s RKD5 path, and the RKD5 `DeviceProtocol` in the Rust core
    (the core keeps only RKD6);
  - the P9c serial-streaming qualification in `DeviceSerialEndpoint`, which
    becomes an RKD6-only endpoint, or is folded into `Rkd6Endpoint`.
- **Keep** the runtime's host-side sampling and per-cycle setpoint path for
  cyclic-control endpoints (the SimKit endpoint, and future EtherCAT). It is
  no longer reachable from any serial device.
- **Docs:**
  - `DEVICE_PROTOCOL6.md` becomes `DEVICE_PROTOCOL.md`; archive the old RKD5
    document in git history only.
  - Update `robotkit/README.md`, `robotkit/ARCHITECTURE.md` "Deployment
    boundary", and `motionkit/IMPLEMENTATION_PLAN.md`'s P9c note to point
    here.
- Bump the runtime ABI and the RKD protocol version as needed, then
  regenerate the bindings and fingerprints.

Tests:
- A `minimal`-profile virtual device runs a Ruckig plan with HOLD, RESUME and
  a link-loss stop.
- A `full`-profile device still passes A4–A7.
- The migrated nucleo stub builds for `thumbv7em-none-eabihf` (A1's check).
- The bench deployment loads and qualifies.
- A deployment declaring `"rkd5"` is rejected with a clear message.
- `grep -rn "RKD5\|device_wire.wire" robotkit` finds only history notes.
- `world-tcp.sh` passes in all three modes.

Commits: at least two, first adding the minimal profile and migrating, then
deleting RKD5.

## Out of scope for this lane

- The nucleo board layer (real timers, GPIO, UART DMA);
- servo, velocity or torque output modes;
- EtherCAT, CAN and FPGA;
- host-side step compression.

These are the follow-on hardware plan.

## Progress log

### A1 paused — MCU build command includes desktop binaries

Started A1 in `../materia-lane-a` on `lane-a-virtual-device` after P0 was
present on `main`. Added a failing virtual-board test, then a draft `Board`
trait and deterministic virtual clock, step counters, channel outputs and
timestamped output records. The new two tests and existing Rust tests pass.

The prescribed `cargo build --target thumbv7em-none-eabihf -p
robotkit-device-protocol` fails because Cargo also compiles the existing
`pty_device` and `robotd_pty_device` desktop binaries for the MCU target.
Those binaries use `std` and POSIX file descriptors. The target is installed;
this is a package-target selection problem, not an unavailable toolchain.
Per the Ground rules, work stopped here before committing A1. The smallest
plan correction is to build the core with `--lib` and build the nucleo board
crate separately for the same target; retain host PTY tests as a separate
check. The owner approved that correction, and A1 resumed. Both MCU builds
now pass. The Rust host tests pass, including the new virtual-board tests.

### A1 — Add the board boundary and virtual board

Added a `no_std` board trait for clock, actuator and channel outputs. The
`std` virtual board has a driven clock with offset and ppm drift, virtual
step counts, channel outputs and timestamped output records. Added the MCU
build script using the approved `--lib` correction and added host/MCU checks
and the session TCP case to RobotKit's run-all script. Commit: the commit
containing this entry.

The Rust tests (including both new virtual-board tests), MCU core and Nucleo
builds, 4,437 RobotKit Haxe assertions, 5,864 MotionKit Haxe assertions, all
12 native tests, both FFI audits and TCP integration in default, session and
lease-timeout modes passed. An initial TCP attempt used a port occupied by
another lane; all modes were rerun on port 17962. The lease-timeout case
missed its plan-start observation once under concurrent builds, then passed
unchanged on retry.

### A2 — Define RKD6 wire records and framing

Added the separately versioned RKD6 schema, generated fixed-record Rust and
C++ codecs, and a checked frame format with the `RKD6` marker, length and
CRC-32. The schema reserves message id 16 for events and defines session,
sync, queue, segment, control, status and state records. Shared frame vectors
round-trip in both languages; malformed lengths and CRCs are rejected. The
protocol document records the maximum frame and serial timing math. Tests
were written first and failed before the schema and framing were added.
Commit: the commit containing this entry.

On the lane tree, Rust tests, the MCU build, 14 native tests, 5,873 MotionKit
and 4,437 RobotKit Haxe assertions, both FFI audits and all three TCP modes
passed. The TCP tests used port 17962 to avoid other lanes' test servers.

### A3 — Estimate host to device clock mapping

Added a bounded C++ clock estimator with low-RTT sliding-window least-squares
fit, request cadence, host-time to device-tick mapping, a measured uncertainty
bound and commit-horizon margin. It latches `clock_sync_lost` when a reply
steps outside the deployment bound. Deterministic tests cover 1,000 ppm
drift, 50 ms offset, asymmetric link jitter, mapped tick error, a 10 ms
clock step and the fault latch. The test was written first. The RKD6 endpoint
in A5 will drive its request and reply methods and expose the fault reason in
the runtime snapshot. Commit: the commit containing this entry.

Rust tests, MCU build, all 16 native tests, 5,875 MotionKit and 4,437
RobotKit Haxe assertions, both FFI audits and TCP integration in default,
session and lease-timeout modes passed on the lane tree.

### A4 schema correction — Per-actuator limits in RKD6 session

A2's fixed `SessionBegin6` header has a single acceleration field, but A4
requires a limit per actuator. The schema compatibility lock correctly
rejected changing that field in place. Added a trailing `ActuatorLimit6`
record per actuator to the session frame instead; the fixed header remains
unchanged. The global field caps those records. Updated both frame decoders,
the generated codecs, tests and the protocol document in the A4 item.

### A4 — Execute a fixed-capacity device segment queue

Added a heap-free Rust scheduled core with revision and committed-horizon
replacement, expected start-state checks, committed-only execution and an
`f32` segment-local Horner evaluator. HOLD/RESUME ramp path-clock rate from
per-actuator acceleration limits. Declared final knots stop without underflow;
continuations underflow into a controlled stop. STOP and link loss ramp from
the last executed velocity with travel-limit clamps, ABORT uses a nearer
rest-declared end when possible, and emergency stop shuts outputs immediately.
The link-loss stop latches and cannot resume automatically. Shared segment
vectors were generated from `motionkit_core` and checked against both its C++
evaluator and the Rust `f32` evaluator. Tests were written before each behavior
change. Commit: the commit containing this entry.

The Rust tests, MCU compile, all 17 native tests, 5,875 MotionKit and 4,437
RobotKit Haxe assertions, both FFI audits, and TCP integration in default,
session and lease-timeout modes passed on the lane tree. A final Rust test
then caught an idle completed plan being mistaken for link loss; it was fixed
without changing the host-side behavior.

### A5 schema correction — Link-loss timeout units

The host cannot convert a deployment nanosecond timeout to device ticks before
the session ACK supplies the device tick rate. Added a trailing `SessionTiming6`
nanosecond timeout record to `SESSION_BEGIN6`; the device converts it using its
own clock. The fixed header stays locked and its old tick field is zero when
the trailing record is present. Both frame decoders and shared tests were
updated before implementing the A5 endpoint.

### A5 — Compile and forward RKD6 plans

Added a C++ device compiler that maps host plan time to device ticks, rescales
local polynomial coefficients for the mapped duration, converts them to
`f32`, runs `mk_validate` at the device step-tick resolution, and checks
sampled execution error against the deployment's `target_error`. The
`Rkd6Endpoint` negotiates a session over a complete-frame transport, sends
periodic time-sync requests, forwards queue revisions, segments and commits,
refills within the device's queue capacity, and maps device status into the
runtime snapshot. A runtime endpoint capability suppresses host per-cycle
target streaming for device-owned queues. Added v3 deployment parsing for
protocol, step-tick, link-loss and clock-sync settings while retaining v2.
Unqualified serial baud or queue depth fails at construction or submit with
the required minimum in the diagnostic. `clock_sync_lost` stops new commits
without discarding already committed motion. Tests were written first for the
compiler and v3 parser, and mock-link tests cover negotiation, replacement,
late rejection, status mapping and clock loss. Commit: the commit containing
this entry.

The plan's A5 in-process device tests depend on the static library and
SimKit loop specified in A6. A5 uses a deterministic complete-frame mock;
the full-chain device assertions will land with A6.

The Rust and MCU builds, all 19 native tests, 5,875 MotionKit and 4,442
RobotKit Haxe assertions, both FFI audits, and all three TCP integration
modes passed. The first RobotKit run caught an obsolete v3 fixture
fingerprint after the schema-lock update; the fixture was regenerated and
the suite passed on rerun.
