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
  them on its own clock. Host-side per-cycle target streaming (P9c) remains
  for RKD5 devices only.
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
- **LA-D4 — RKD6 is a new protocol version, not an edit to RKD5.**
  - Sync marker `RKD6`. The device protocol version is negotiated in session
    begin.
  - RKD5 support stays in the host, because RKD5 devices are still valid
    (the P9c streaming path).
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
- **Deployment:** `deployment.json` v3 adds `protocol: "rkd5" | "rkd6"`,
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

## Out of scope for this lane

- The nucleo board layer (real timers, GPIO, UART DMA);
- servo, velocity or torque output modes;
- EtherCAT, CAN and FPGA;
- host-side step compression.

These are the follow-on hardware plan.

## Progress log

(append entries here)
