# Operator HMI, pendants and control cabinet plan

**Goal.** One platform for the operator interface and electrical cabinet of every machine:
fixed HMI panel, jog/teach pendant, and control cabinet. A machine supplies a small **machine
profile** and the platform does the rest. First cases: the drill-tap VMC, the flexspline
hobber and a robot arm cell.

All numbers below are targets or estimates unless marked as read from the repo. Standards
references are to be verified against the current revisions (CC9).

## Principles

1. **Safety is hardwired and bought.** E-stop, enabling switch, guard locking and stop
   categories go through a certified safety relay or controller. Our boards and software only
   observe. Nothing safety-relevant is touch-only or software-only.
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

### CNC pendant (drill-tap, hobber)
- Dual-channel E-stop mushroom and 3-position enabling switch to the safety relay.
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
| CC0 | Profile data model and requirements for the three cases | none |
| CC1 | Pendant protocol and motion lease in the RobotKit runtime, headless tests (link loss, enable release, two clients) | CC0 |
| CC2 | Operator HMI app on the desktop GTK build, drill-tap and hobber sims | CC0, CC1 |
| CC3 | Embedded spike: aarch64 cross-build, `cage` on a Pi 5, measure boot, memory, frame time, touch; decide on `linux-drm` | CC2 |
| CC3b | `linux-drm` NativeKit backend (only if CC3 says so) | CC3 |
| CC4 | CNC pendant bench prototype: Nucleo, real MPG, E-stop and enable into a safety relay, CAN-FD to the host; connector bridge or detect | CC1 |
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
