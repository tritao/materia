# Operator HMI, pendants and control cabinet plan

**Goal.** One platform for the operator interface and electrical cabinet of every machine:
fixed HMI panel, jog/teach pendant, and control cabinet. A machine supplies a small **machine
profile** and the platform does the rest. First cases: the drill-tap VMC, the flexspline
hobber and a robot arm cell.

All numbers below are targets or estimates unless marked as read from the repo. Standards
references are to be verified against the current revisions (CC9).

## Principles

1. **Safety is hardwired and separate from our software.** E-stop, door and stop go through a
   hardwired chain that our boards and software only observe. Nothing safety-relevant is
   touch-only or software-only. **v1 is the uncertified bench chain** (see "v1 safety chain");
   the certified relay, enabling switch and locking interlock are a later drop-in upgrade of
   the same module.
2. **Everything else is a thin client of the runtime.** The runtime has one motion owner. A jog
   from the pendant is another command source (like the keyboard, a remote client or a test),
   taken as a **lease**. Loss of link or release of the enabling switch ends the lease and stops.
3. **Fast loops stay on devices.** Gearing, step/dir and encoder loops run on RKD6 nodes or
   drives on device clock, never on the HMI or the host UI. The HMI and the pendant can crash
   without affecting motion.
4. **Derive, don't hand-draw.** The cabinet (supplies, fusing, braking resistor, wire gauge,
   DIN layout, wire list, BOM) is generated from the machine's drive list.
5. **One schema, no compatibility code** for old saved profiles.

## Machine profile

A profile is the data a machine hands to the platform:

| Field | Meaning |
|---|---|
| HMI pages | Which pages exist and their order (DRO, program, alarms, gearbox, teach, ...). |
| Pendant legend | Axis selector entries, soft-key labels, LED meanings. |
| Jog permissions | Which axes the pendant may jog in which state (see hobber). |
| Guard parameters | `EnclosureGuard` configuration: setup-mode speed limit, hold-to-run, unlock rules. |
| Drive list | Drives, motors, brakes, spindle, auxiliaries; sizes the cabinet. |
| Services | Air, MQL/coolant, lubrication, mist extraction, lighting, status tower. |
| Modes | Mode key positions (Auto, Manual, Setup; T1 and T2 for robots). |

## Cases

| | Drill-tap VMC | Hobber | Robot arm cell |
|---|---|---|---|
| Machine data | 400 × 300 × 380 travel, BT30 12–15k rpm 3–4 kW, 21-tool turret, A axis, MQL first, auto door, EtherCAT | Axes C, B, X, Y, Z, A; electronic gearbox (hob master, table follower); opposed doors, 400 mm opening | Cobot or industrial arm; gripper; tending interface |
| Pendant | Wired, MPG, no screen | Wired, MPG, no screen, gearbox-aware | Wired, with screen, 6-axis puck |
| Main HMI page | DRO, program, tool table, probing | Gearbox sync status, hob shift, pass counter, error budget | Program select, cell mode, safety banner |
| Cabinet | On the machine base | Separate floor rack (already modelled) | Small controller box |
| Modes | Auto, Manual, Setup (door open, hold-to-run) | Same | Auto, T1, optional T2 |

## v1 safety chain (uncertified bench build)

For a machine only the owner operates. Not for sale, not for other operators, and not for an
arm moving near people.

- **No motion with the door open.** Door closed: jog and cut from the pendant. Door open: drives
  disabled and spindle stopped. There is no setup mode with the door open in v1, so no enabling
  switch and no setup speed-limit logic.
- **The chain.** An NC E-stop mushroom in series with a door switch (reed or limit), cutting the
  drives' STO inputs and the spindle enable or a contactor through a plain relay. NC contacts
  fail safe on a broken wire. Estimated cost €30–50 (unchecked).
- **What it is not.** No redundancy, no stuck-contact monitoring, no rating. Operating rules:
  never reach in while powered, test the E-stop regularly, nobody else uses the machine.
- **Read-back.** The cabinet controller reads the chain state (E-stop, door, drives enabled) so the
  HMI, the guard logic and the lease know it. It is observed only and never in the chain.
- **Upgrade path.** The chain is one module with a fixed interface: loop terminals plus read-back
  signals. A certified relay, 3-position enabling switch and guard-locking interlock replace it
  later without changing the pendant, the runtime or the profile. Adding the door-open setup
  mode then restores the `EnclosureGuard` setup policy.
- **The arm.** The v1 cell is kept clear of people while powered and runs with the software speed
  limits only. B1 (certified safety controller) waits until the arm is used near people.

## Long-term architecture

Every machine gets the same three parts; the profile selects pages, jog rules and I/O.

1. **Operator station.** A panel PC (CM5 on an industrial carrier) running the HMI and the host
   runtime. One shielded Ethernet cable plus 24 V to the cabinet. Touch, display and key strip
   are internal and short.
2. **Cabinet controller.** A real-time RKD6-class node that masters EtherCAT, evaluates the
   streamed segments per cycle, owns the I/O (CAN-FD to expansion modules) and talks to the host
   over that Ethernet link. The host never touches a drive directly.
3. **Pendant.** A CAN-FD node plus its hardwired safety loops, wired to the **cabinet
   controller, not the host**. The lease and heartbeat are enforced on the real-time side, so a
   host crash cannot leave motion running or stuck.
4. **Safety.** A certified safety relay or controller in the cabinet owns E-stop, enabling
   switch, guard locking and STO. Our nodes only read its state back.

| | Drill-tap | Hobber | Arm cell |
|---|---|---|---|
| Cabinet | On the base | Separate floor rack | Small controller box with a certified safety controller (B1) |
| Operator station | Swing arm | Floor stand, one Ethernet cable to the rack | Panel as above |
| Pendant | MPG on CAN-FD to the controller | MPG; B and C jog refused while the gearbox is engaged | Rugged tablet running the web client, plus a safety MCU with E-stop and enabling switch wired to the safety controller |
| Gearing / probe | Probe latch on the drives | Gearbox on device clock in DC-synchronised drives or a node (B2) | Not applicable |

### Link decisions

- **Host to cabinet controller: Ethernet, not EtherCAT.** The host streams plan-ahead segments to
  nodes that evaluate them on their own clock, so the traffic is bursty and latency-tolerant.
  UDP with sequence numbers for streams, a reliable channel for configuration, logs and firmware,
  and a heartbeat for link-loss and underflow latching (already in the controller-boundary
  design). The same protocol runs over USB on the bench.
- **EtherCAT only between the cabinet controller and the drives.** Making the Pi the master of
  the RKD6 nodes too would put the host back in the real-time loop. Sellable RKD6 modules that
  other masters can use as EtherCAT slaves are a separate product decision (ESC chip, stack,
  ETG conformance) and not part of this link.
- **Time reference.** Segments are timestamped on the node clock, so the host and node need a
  shared time base: PTP if the hardware supports it, otherwise an offset-and-drift estimate over
  the same link. PTP hardware timestamping on the Pi 5 and on the chosen MCU is unchecked.
- **Inside the cabinet:** CAN-FD to the I/O modules, key strip and pendant nodes. Not a switch.
- **Budget build.** Pi 5 in the cabinet with a Nucleo as the pendant/key node over USB, speaking
  the same cabinet protocol. The long-term controller replaces them without runtime or HMI
  changes.

## EtherCAT master spike (CC1b)

**Candidate stack: EtherCrab** (pure Rust, `std` and `no_std`, Apache-2.0 or MIT, so no GPL
issue as with SOEM or IgH). Investigated 2026-10-27 from its README, docs.rs, and issue and PR
trackers (`gh`); nothing was run.

What the trackers show:
- **Release and activity.** v0.7.1 is the latest release (2026-03-23). PRs merged through
  September 2026 and opened as recently as 2026-10-27. A mature library with an active
  maintainer, but still 0.x with breaking changes between releases.
- **Embedded.** Official `examples/embassy-stm32` targets a **Nucleo-F429**. Issue #276 (Feb 2025,
  closed) reported that example hanging on hardware, with a stack-size suspicion. Issues #89
  (Embassy) and #90 (RTIC) for multi-group examples are open. A search for "stm32" returns no
  issues, so there is little community evidence of MCU use beyond the maintainer's example.
  NOTES.md says Embassy raw sockets register one waker for TX and RX and tasks cannot be bound to
  an interrupt, so a hard-real-time MCU cycle likely needs RTIC or a hardware-timer interrupt.
- **Distributed clocks.** Open: #308 (a 32-bit DC reference clock gave spikes up to 15 ms on a
  10 ms cycle, a host timing issue), #372, #373 and #374 (topology and clock-overflow edge
  cases). The maintainer plans to disable DC by default and require an explicit `set_dc_sync` per
  subdevice. PR #322 (SYNC0 start for 32-bit devices) is merged.
- **Servo drives.** The high-level DS402 API (#233) is still open, so the README's "basic" support
  is the real state. The drive examples in the repo are disabled. Issue #49 (2023, closed) had
  chains of Omron plus ZeroErr drives time out at Init, on a very early version.
- **Error handling.** #268 (open): the bus cannot be shut down cleanly after a state-transition
  failure because the group is moved by `into_op`. This matters for fault handling.
- **Conformance.** The maintainer says the ETG conformance test applies to subdevices, not
  masters (#345). Working-counter checks are manual. ESI parsing is basic.
- **Safety and other protocols.** FSoE (#378) and SoE (PR #379) work is open; relevant to B1 only
  as a possible path, not something to rely on.

Spike steps:
1. Run EtherCrab on the Pi 5 against the real ZeroErr or Leadshine drive: CiA402 state machine,
   DC cyclic at 1 kHz, clean shutdown after a forced failure.
2. Reproduce the official `embassy-stm32` example on a Nucleo-F429ZI.
3. Port to the STM32H723 with the Ethernet DMA buffers in a non-cacheable MPU region.
4. Drive the cycle from a hardware timer at 1 kHz, toggle a GPIO each cycle, and measure jitter on
   a logic analyzer with 6–8 slaves in OP and the host link active.
5. Compare with an STM32H5 (H563/H573) only if the H723 is borderline or too costly.

MCU notes (from family knowledge, not verified in this spike): H7 (H723/H735) is the primary
candidate, since RKD6 already runs on it; H5 (M33, 250 MHz, Ethernet) is the lower-cost option;
F4/F7 are fine for leaner nodes and F429 is the reference platform; N6 is new and unchecked; G4,
U5 and L4 have **no Ethernet MAC** and cannot be a master. The RKD6 device core
(`robotkit/device_protocol`) is already `#![no_std]` Rust with a `Board` trait (position and
velocity targets, step pulses, digital/analog/process channels, `stop_all`), so an EtherCrab
master would be another `Board` implementation in the same language and crate family. Today the
host to device transport is serial/PTY carrying RKD6 frames (marker, CRC-32, wire revision 16);
the Ethernet link is a new transport for the same frames. Known gap: segment coefficients are
`f32`, which is B2. Fallback: SOEM or a commercial master on Linux (option B), or a TI
AM64x/AM243x-class SoC (option C).

## Targets

### Fixed HMI panel (all machines, one SKU)
- 15.6" 1920×1080 landscape, optically bonded, projected-capacitive with glove and wet
  rejection, anti-glare and oleophobic, IP65 at the front, 400–500 nit.
- Swing arm or fixed bezel, VESA 100, centre about 1.3–1.5 m, 10–20° tilt.
- Physical keys beside it: cycle start, feed hold, stop, door, MQL/blast, work light, mode key,
  feed / rapid / spindle override dials.
- CM5-class compute, one HDMI or eDP link, USB touch.
- Hobber: mount on the rack or a floor stand, not on the machine, so touches do not couple
  into a precision cut (judgement, not measured).

### Key strip beside the HMI (membrane panel)

A custom membrane switch with a graphic overlay under or beside the screen, for **non-safety keys
only**. The vendor candidate is JRPanel (membrane switches, graphic overlays, FPC tails, PMMA
panels). Only their home page was reviewed: lead times, MOQ, quotes and test data are unknown.

| Key | LED | Machines | Notes |
|---|---|---|---|
| Cycle start | green | all | Guard-gated: never starts a cut with the door open or unlocked. |
| Feed hold | amber | CNC | Holds feed, keeps spindle. |
| Stop / reset | red | all | Controlled stop and fault reset. Not an E-stop. |
| Door open / close | green when closed and locked | CNC | A request into `EnclosureGuard`; unlock still waits for standstill. |
| MQL / air blast | green | CNC | Manual toggle. |
| Work light | white | CNC | |
| Spindle jog (manual) | amber | CNC | Only in Manual or Setup, hold-to-run. |
| Zone reset | amber | arm cell | For a protective-zone reset after clearance, if the cell has one. |
| Auto / manual indicator | green / amber | arm cell | Indicator only; the mode itself is a key switch. |

- **Not on the strip:** E-stop, enabling switch, mode key switch, override dials and anything
  the safety chain depends on. These are bought certified parts.
- **Construction (targets, confirm with the vendor):** metal-dome tactile keys, polycarbonate
  overlay, glove-friendly actuation force (about 4–6 N), top-lit LED windows, FPC tail to a small
  RKD6-family scan node on CAN-FD or USB.
- **Legends:** ISO 7000 / IEC 60417 symbols where one exists, plus text; the language is chosen at
  order time.
- **Environment:** oil mist, cutting fluid, and cleaning solvents. Ask for tested ingress and
  chemical-resistance data for the exact overlay and adhesive, not the headline rating.
- **Prototype:** use illuminated push buttons or a stock keypad first; order the custom panel once
  the key list is stable (CC2).
- **RFQ package:** key list and legends, LED positions and colours, outline and mounting, tail
  length and connector, environment, quantity.

### Links between the host and the operator devices

The host (a Pi 5 or CM5) may sit in the cabinet with the screen on a short cable, or behind
the screen. For the cabinet case: HDMI to the monitor and USB for touch.

| Device | Link | Notes |
|---|---|---|
| Display | HDMI | Passive to about 3 m; active or optical HDMI beyond; locking or screw-lock connectors, flex-rated cable, shielded and ferrited, away from motor and brake-resistor cables. Force the mode in the boot config so a screen power-cycle recovers. |
| Touch | USB | HID device, through an isolated or active extender (USB 2.0 passive limit about 5 m). Not safety-relevant. |
| Key strip scan node | CAN-FD | Small RKD6-family node. |
| Pendant node (MPG, buttons, LEDs) | CAN-FD or Ethernet | The node reads the MPG and sends position deltas with a sequence number and heartbeat. |
| E-stop, enabling switch, door interlock | Hardwired to the safety relay | Never on a bus. The bus only reports their state. |

- **No USB hub for control inputs in the product.** The pendant lease needs a heartbeat that
  survives a noisy bus. A hub or a USB glitch looks like a lost or stuck pendant, adds a failure
  point, and takes touch, keys and jog down together.
- **Cables.** One shielded M12 cable per device: CAN_H/L, 24 V and ground. The pendant cable also
  carries its separate safety loops. Chain nodes on the bus with termination at each end.
- **The Pi 5 has no CAN.** Prefer an SPI CAN-FD HAT. A USB-CAN adapter (the BTT U2C is in the
  prototype stack) works but reintroduces a USB single point of failure, with a bus that at least
  has heartbeats and hardware filters.
- **Prototype exception.** A powered, isolated USB hub at the screen carrying touch, the key node
  and the MPG is fine for bring-up. Write the node protocol as if it were CAN, so moving off USB is
  a transport change only.
- **Separation of host and screen.** Over about 5 m between host and screen (the hobber's
  separate rack may be this far), put a panel PC next to the screen with Ethernet to the host.
  Handheld teach pendants stay web clients over Ethernet, not video cables.
- **Environment.** A bare Pi 5 is not industrial-rated; the product host is a CM5 on an
  industrial carrier (or a DIN-rail unit such as the RevPi Connect 5).

### CNC pendant (drill-tap, hobber)
- v1: an E-stop mushroom (NC contact) into the safety chain. No enabling switch, because v1
  allows no motion with the door open. The certified upgrade adds a dual-channel E-stop and a
  3-position enabling switch (IEC 60947-5-8) to a certified relay.
- MPG, 100 PPR, axis select (X, Y, Z, A, spindle), steps ×1 / ×10 / ×100 (0.001 / 0.01 /
  0.1 mm), feed override encoder, cycle start / hold / stop, mode key, a few soft keys with LEDs.
- No onboard compute: a small RKD6-family node on CAN-FD or Ethernet. 3–5 m coiled cable,
  magnetic holder, PUR jacket if oil is used.
- Wired only in v1. The loop through the connector needs a bridge plug or pendant detection.
- Detaching must not open the loop silently (see CC4).

### Arm teach pendant
- 8–10" capacitive screen, glove-capable, under about 1.2 kg, IP54, drop-rated, 5–8 m PUR cable
  with strain relief, wall holder.
- Small Linux device for the UI **plus an MCU that owns the E-stop, enabling switch and jog
  inputs**, so a UI crash cannot remove them.
- 6-axis puck, jog frames (joint, world, tool, user), MPG optional.
- Pages: safety banner (`SafetyPhase`, speed limit, stopping envelope), record / move to point,
  speed override, TCP and payload setup (feeds `LoadSafetyConfiguration`), 3-point frame teach,
  I/O view, gripper.
- Hand-guiding buttons live at the wrist or flange, not on the pendant.

### Machine guarding (reference: drill-tap)
- Design fragment energy from the largest tool. A Ø65 mm boring head at 15k rpm has about
  51 m/s rim speed. Windows are bought certified laminated glazing.
- Powered, guard-locked door, unlock only at confirmed standstill (`EnclosureGuard`).
- About 400 × 300 mm window, sloped chip floor, flood-ready splash height, filtered mist
  extraction for MQL, about 1000 lux inside, 75–80 dB(A) at the operator.

### Cabinet
- Drill-tap: on the base, 400 V 3-phase, 16 A feed (peak estimate 8–9 kW, recompute after DT8
  drive records), about 600 W × 800 H × 300 D, IP54, filtered fan or small cooler, 6 bar dry air.
- Hobber: separate rack (six drives plus supply, already modelled).
- Arm: about 500 × 400 × 200 mm box with the safety controller, supply, drives or STO wiring,
  brake release and I/O.

## Hobber-specific rules

- The gearbox lives on the device clock. At 3000 rpm and 12 gashes a gash passes every 1.7 ms.
- Pendant jog on B and C is refused while the gearbox is engaged; gearbox engage and disengage
  need standstill. The MPG drives X, Y, Z, shift and swivel.
- A re-engage procedure after any stop returns to the same tooth gap.
- Unlock delay is driven by measured spin-down of the 3000+ rpm spindle. Brake or regenerative
  load may be needed.

## Software

- **Pendant protocol and lease** in the RobotKit runtime: jog command source, enabling-switch
  state, link-loss stop, one active lease.
- **Operator HMI app**: a runtime-only UIKit profile (no editor), talking to the runtime over the
  existing protocol, locally. The web build remains the remote view.
- **Embedded Linux**: option 1 is the existing GTK backend under a single-app Wayland
  compositor (`cage`). Option 2 is a `linux-drm` NativeKit backend (DRM/KMS, GBM, EGL with the
  existing GLES3 runtime, libinput, xkbcommon, on-screen keyboard). Decide after the spike.
- **Cabinet generator** in `machinekit`, using `examples/enclosure-kit` and
  `examples/cable-harness`.
- **Guard binding**: every machine binds its door and setup mode to `processkit.guard.EnclosureGuard`.

## Phases

| Phase | Work | Depends on |
|---|---|---|
| CC0 | Operator profile data model and requirements for the three cases. **Started:** `machinekit.operator` has `OperatorProfile` (named so it does not clash with RobotKit's `RobotProfile`), a one-version codec, validation (pendant map, F-keys, jog rules, guard) and drill-tap, hobber and arm reference profiles, with `tests/haxeon.operator-profile.json`. Left: HMI page content, service wiring from the cabinet components. | none |
| CC1 | Pendant protocol and motion lease in the RobotKit runtime, headless tests (link loss, enable release, two clients). **Started:** `PendantAuthority` in `machinekit.operator` (sequence and wrap handling, 50 ms-class heartbeat lease, release-before-resume, hold-to-run, mode and jog-rule gating, HMI arbitration) with `PendantAuthorityTests`. Pendant payloads are defined in `machinekit/pendant_wire` (`.wire.idl`, lock, generated `no_std` Rust codec, byte-exact tests). The same authority rules are ported to `pendant_wire/src/authority.rs` (`no_std`, integer positions, same scenarios as the Haxe model). Left: wiring it into the RKD6 cabinet node and its link-loss enforcement, runtime integration with robotd's session lease | CC0 |
| CC1b | EtherCAT master spike (EtherCrab on Pi 5, then STM32F429 and H723; jitter measured) | CC0 |
| CC2 | Operator HMI app on the desktop GTK build, drill-tap and hobber sims | CC0, CC1 |
| CC3 | Embedded spike: aarch64 cross-build, `cage` on a Pi 5, measure boot, memory, frame time, touch; decide on `linux-drm` | CC2 |
| CC3b | `linux-drm` NativeKit backend (only if CC3 says so) | CC3 |
| CC4 | CNC pendant, one SKU, own design: see `PENDANT_PLAN.md` (small FDCAN MCU, bought MPG wheel and E-stop, 12-position axis switch, step switch, buttons, small text-only OLED, 12-pin M12 with enabling-switch pins reserved); bench on a Nucleo-G474, then a pcbkit board and a CadKit case | CC1 |
| CC5 | Cabinet generator from the drive list; drill-tap first, hobber rack second | CC0, B3 |
| CC6 | Bind drill-tap and hobber profiles to `EnclosureGuard`; hobber gearbox-aware jog rules | CC1, CC2, B2 |
| CC7 | Arm teach pendant: screen plus MCU, puck, modes, teach pages | CC1, B1 |
| CC8 | Optional tablet cradle for CNC pendants | CC4 |
| CC9 | Standards and qualification review with an outside party | CC6, CC7 |

## Blockers and open questions

- **B1. Safety-rated speed monitoring for arms.** T1 speed limits and force limits need safety-rated
  feedback (dual-channel encoders and a certified safety controller or drive). Our runtime
  limits are soft. Needs a decision on which controller to buy before any arm leaves a lab.
- **B2. RKD6 f32 segments.** Hobber gearing and long rotary motion need integer or 64-bit
  positions (known gap from the controller-platform discussion).
- **B3. Leadshine ELM2H datasheet has no Ke/R.** DT8 drive records and the drill-tap cabinet heat
  load wait on the vendor or other servos.
- **B4. Supply region.** Assumed 400 V / 50 Hz. US sale changes supply, UL 508A and parts.
- **B5. Hobber fluid.** Dry, MQL or oil decides the pendant cable jacket and HMI IP rating and
  mount.
- **B6. Aarch64 Haxeon runtime.** Does the Haxeon runtime cross-build for aarch64 Linux, and
  does it use the JIT or the C path? Checked in CC3.
- **B7. Pendant screen on CNC.** Start without; revisit after the first machine runs.
- **B8. EtherCAT master maturity.** EtherCrab is 0.x with open DC and DS402 issues and little
  evidence on MCUs; the spike (CC1b) decides it, with SOEM, a commercial master, or a TI SoC as
  fallbacks.

## Standards to verify (CC9)

ISO 16090-1 (machining centres), ISO 14119 (interlocking with guard locking), ISO 13849-1
(PL d on door and E-stop), ISO 13850 (E-stop), IEC 60947-5-8 (enabling devices), IEC 60204-1
(machine electrics), ISO 10218 (industrial robots; check the current revision), ISO/TS 15066
(collaborative), UL 508A if selling in the US.

## What exists

- `processkit.guard.EnclosureGuard` with hobber binding `HobberEnclosureControl`: locked until
  standstill, independent lock feedback, setup mode with hold-to-run and a speed limit, latched
  faults, safe reset. The limits are model parameters, not qualified values.
- `machinekit/ENCLOSURE_PLAN.md`: the enclosure kit and generator, door and guard interface (EN-D4).
- `examples/bench-mill/DRILL_TAP_PLAN.md`: drill-tap requirements, DT8 enclosure and services.
- `examples/flexspline-hobber/ENCLOSURE.md`: fitted hobber enclosure, guard checks, separate
  electrical rack.
- RobotKit safety types: `SafetyPhase`, `SafetyState`, `StoppingEnvelope`, `LoadSafetyConfiguration`.
- NativeKit: GTK3 Linux backend, a GLES3 GPU runtime, `platform-parity.md` with a "Not
  applicable" state for single-surface backends.
- Related: the controller-platform discussion (CM5 core, RKD6 nodes, bought safety).
