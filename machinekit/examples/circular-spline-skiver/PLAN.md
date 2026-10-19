# Circular-spline power-skiving machine plan

**Goal.** A precision power-skiving machine that cuts the internal teeth of strain-wave circular
splines. It is designed in materia as a **sibling of the flexspline hobber**: shared chassis, parts
library and software. It cuts the teeth in the editor the way the real machine would, and grades
them like a gear lab.

1. A turned circular-spline ring (from the lathe plan, `machinekit/examples/lathe/PLAN.md`) is
   clamped on a **high-speed direct-drive work spindle** (C).
2. A **skiving wheel** — a gear-shaped cutter with cutting edges — sits on a cantilevered tool
   spindle (B), at a crossed-axis angle Σ (A swivel, typically 15–30°), inside the ring.
3. Tool and work both spin fast, locked by the **electronic gearbox**:
   `C = (z_tool / z_work)·B + axial differential + programmed C`.
   The tool feeds along the work axis (Z) in several radial infeed passes (X).
4. The simulation cuts the internal teeth on the hobber's **ring stock** from the wheel's real
   cutting edges. **gearkit** metrology grades them (ISO 1328, extrapolated below module 0.5).
   An error budget names the part that limits the grade.

**Target:** circular splines matching the hobber's flexsplines: m 0.2–0.5, Z_c = Z_f + 2, sizes
14–25 class, **ISO 1328 class 5–6** (extrapolated).

**Why a separate machine.** The hobber's work table is slow (15 rpm) and tuned for sub-µm runout.
Skiving needs both spindles fast (about 1,500 rpm table and 6,000 rpm tool on the reference ring),
a larger swivel range, a cantilevered tool reaching inside the ring, more stiffness, and a much
faster gearbox loop. Making one machine do both would compromise hobbing precision. Commercial
skiving machines are dedicated machines (Liebherr, Präwema, DVS/Pittler, Mitsubishi, Gleason).
Turn-mill skiving cycles are generally medium precision.

**Prototype route outside this plan:** first circular splines can be **wire-EDM'd** or
**shaped** (and/or outsourced). This machine is for precision and volume later.

The work spans kits:
- `gearkit`: skiving tool design (crossed-axis conjugate), internal-gear metrology (exists);
- `stockkit`: the ring stock and rake sweep (reused) and a skiving driver;
- `machinekit/haxe/src/machinekit/hobbing/` (or a shared `gearcutting` package): work spindle,
  tool spindle, wide swivel, ring workholding;
- `machinekit/examples/circular-spline-skiver/`: the machine, checks, preview, project;
- `robotkit` runtime and device core: the high-speed gearbox (see PS-D4);
- `toolpathkit`, `camkit`: `SkivingJob`;
- `app`: cutting and the gear report.

## What exists

Surveyed 2026-10-19 on branch `flexspline-hobber` (9bc0a04b8). Nothing is merged to main yet; the
hobber branch must land first (Working rules).

**From the hobber (reused).**
- `gearkit`: `GearSpec` (external **and internal**), `ToothProfile`, `ConjugateRack`, metrology
  (`GearMeasurement`, internal dimensions between inward balls, `Iso1328`, `ErrorBudget`).
- `stockkit`: `RingStock` (designed to handle internal teeth unchanged); the native rake-region
  sweep (`rake.cpp`, rigid poses, exact interval subtraction, threaded, 11× fine-cut speedup);
  `HobbingAxes`/`HobbingMotion` (frame chain, exact pass timing between control ticks);
  Kienzle loads; error-source framework.
- `robotkit` runtime: `ElectronicGear` (`robotkit/runtime/ELECTRONIC_GEAR.md`): measured-master
  follow with latency extrapolation; HB7 sync tests; the physics-epoch fix.
- MachineKit hobbing parts: `RotaryTable`, `TorqueMotor`, crossed-roller and angular-contact
  bearings, ring encoder, `HobSpindle`, `SwivelHead`, mineral-cast foundations with steel
  inserts. The chassis is being revised to a GE-class **moving column** (hobber HB-D1 revision,
  2026-10-19).
- CAM: `HobbingJob` structure (per-revolution feeds, passes, overrun from geometry).

**Missing.**
- Skiving tool design (a crossed-axis conjugate cutter, rake and clearance faces).
- A skiving kinematics driver for the rake sweep.
- A high-speed precise work spindle; a cantilevered tool spindle with a slim nose; a swivel
  range to ±30°; ring workholding.
- A gearbox loop for high-speed master and slave with hardware-timestamped sampling.
- `SkivingJob`; skiving forces and chatter risk; skiving error sources.

## Decisions

**PS-D1. A sibling of the hobber, not a mode of it.**
- Same chassis family (moving column, mineral-cast bed with inserts, X/Z slides, swivel on the
  Z slide, Y on the swivel), parts library and software.
- Different where physics requires:
  - a high-speed work spindle instead of the slow table;
  - a cantilevered skiving spindle instead of the two-ended hob arbor;
  - a swivel range to ±30°;
  - a faster gearbox loop.
- **Rejected:**
  - making the hobber skive (table speed and gearbox demands work against hobbing precision);
  - a turn-mill skiving cycle on the lathe (medium precision; possibly a later coarse-gear
    option there).

**PS-D2. The skiving wheel is derived from the target tooth, never chosen from a list.**
- `SkivingToolDesign(target internal GearSpec, Σ, z_tool, tool helix, rake, clearance)`.
- The tool's cutting-edge profile is the **crossed-axis conjugate** of the target tooth, by a
  numerical envelope in gearkit, as `ConjugateRack` does for hobs.
- It works for involute targets now and for the tooth-design plan's `Sampled` circular-spline
  form later (TD-D3).
- Checks: interference and undercut regions specific to internal skiving (tool tip vs the
  opposite flank, the exit region); edge clearance angles positive along the edge; the tool
  fits the ring bore at the working Σ.

**PS-D3. The cut uses the hobber's ring stock and rake sweep.**
- Each skiving-wheel tooth's rake-face region is swept through its pass. Passes are timed
  exactly between control ticks, as HB-D4 does, from the measured B/C states.
- No new stock model is needed: internal teeth are intervals on the same circles.
- Generating marks and feed marks emerge from the edges. Formulas are used only as test oracles.

**PS-D4. The gearbox runs where the timing is: on the device, with hardware timestamps.**
- At skiving speeds, timing jitter is the critical error.
  - The reference ring's table turns at about 25.7 rev/s, so **1 µs of jitter ≈ 33″ ≈ 4.9 µm**
    at the 30 mm pitch radius. That is most of class 6's single-pitch budget from jitter
    alone.
  - Host-closed loops at 1–4 kHz (the LinuxCNC class) cannot meet it.
- The follow relation (HB-D2's measured-master law) runs in the **RKD6 device core**, with
  encoder counts **latched by hardware timers** and the slave command computed at 20–50 kHz.
  The host sets the ratio and streams X/Z/Y.
- The same Rust code runs on the virtual device in the simulation, so simulated sync error is
  the real firmware's.
- Jitter, latency, encoder resolution and interpolation error are stated parameters, and the
  budget must show they fit.
- This is the device-side gearbox already discussed for the hobber. Skiving makes it necessary.
  The hobber can adopt it too.

**PS-D5. The work spindle is a precision motor spindle, specified at speed.**
- Direct drive, rated for about 2,000 rpm (assumed).
- Non-repeatable runout **at speed** as a stated value (bearing behaviour at speed is different
  from the hobber's slow table).
- Thermal growth at speed (an HB14-style lumped model). A ring encoder or high-resolution
  spindle encoder for C.
- Ring workholding: a diaphragm chuck or face clamp with a centring bore. Clamping distortion of
  the thin-ish ring is a named error source (three-lobe after unclamping).
- Candidate bought spindle: Demat RTVS230H-030 (Ø280, 37/55 Nm, 1500/2000 rpm, ±10″/±6″,
  water-cooled; ≈ US$5,000–12,000 estimated). Runout and sync data must be requested
  (`machinekit/STRAIN_WAVE_RESEARCH.md`, Suppliers and prices). Model it with PS4's spindle
  part's bought form, carrying datasheet or measured values.

**PS-D6. Programs are multi-pass and geometry-derived.**
- `SkivingJob(target, tool, machine, ring)`:
  - Σ and tool position;
  - spindle speeds from cutting speed;
  - **several radial infeed passes** (skiving removes depth in steps), with axial feed per
    work revolution;
  - entry and exit overrun from geometry;
  - retract between passes;
  - climb or conventional direction.
- ToolpathKit ops reuse `ElectronicGear` and `ReleaseGear` and per-revolution feeds (HB-D8).
- G-code later.

**PS-D7. Forces, chatter and errors use the hobber's frameworks.**
- Kienzle loads from interval chip areas. Skiving chips vary strongly along the edge, so report
  the peak and spectrum.
- A **chatter-risk indicator**: excitation frequencies (tool tooth-passing, work rotation)
  against assumed structural modes, until FEA exists.
- Error sources on their parts:
  - work-spindle and tool-spindle runout at speed;
  - Σ setting error;
  - tool profile and runout;
  - gearbox jitter and latency;
  - ring clamping distortion;
  - thermal growth.
- `ErrorBudget` per source and combined. Use the cheaper strategy agreed for the hobber's HB11:
  analytic or coarse runs for simple once-per-revolution sources, validated once against a fine
  run, and fine simulation for coupled sources plus one combined run.

**PS-D8. Generic names in code;** vendors only as sources in docs.

## Test parts

- **Coarse demo ring:** internal involute, m 1, z 60, face 10 mm. Fast; visible marks.
- **Reference circular spline:** internal involute, m 0.3, z 202 (pairs with the hobber's z 200
  flexspline), d ≈ 60.6 mm, face 8–10 mm, ring OD ≈ 80 mm (assumed). Prototype material
  42CrMo4 Q&T about 30 HRC, soft-skived (assumed).
- **Later:** the tooth-design plan's double-circular-arc circular spline (a `Sampled` form).

## Reference numbers (assumed until checked against a datasheet, a source or a measurement)

| Quantity | Value | Source |
|---|---|---|
| Crossed-axis angle Σ | 20° (range 15–30°) | typical skiving practice, assumed |
| Skiving wheel | carbide, m 0.3, z 50 → d ≈ 15 mm | assumed; tool size limited by ring bore and chip space |
| Cutting speed | ≈ 100 m/min (soft) | assumed |
| Tool speed | ≈ 6,000 rpm | derived roughly from v_c ≈ v_tool·sin Σ; check against a skiving reference |
| Work speed | ≈ 1,500 rpm (= tool × 50/202) | derived |
| Sync jitter budget | ≲ 0.1 µs → ≲ 0.5 µm at r 30 mm | derived (PS-D4) |
| Radial passes | 4–8 | assumed, typical skiving |
| Work-spindle NRRO at speed | ≤ 1 µm | assumed |
| Class 6 limits (d 60.6, m 0.3) | fp ≈ 7.3 µm, Fp ≈ 23.5 µm | HB-D6 formulas, extrapolated |

## Steps

### Phase A: tool, kinematics, ideal cut (no machine)

**PS0. Baseline and test parts.**
- Record the gate on the starting commit.
- Internal `GearSpec`s for the demo ring and the reference circular spline.
- Tests: specs validate; the existing gearkit and stockkit suites pass.

**PS1. Skiving tool design (PS-D2).**
- `SkivingToolDesign`: crossed-axis conjugate edge profile (numerical envelope); rake and
  clearance faces; per-tooth rake regions for the sweep; interference and undercut checks;
  fit in the bore at Σ.
- Tests:
  - an involute internal target gives an involute-equivalent tool profile in the transverse
    section to 1e-6 mm (where that equivalence holds);
  - regenerating the target from the tool reproduces it within the envelope tolerance;
  - a tool too large for the bore, or an interfering Σ, is rejected, naming the parameter.

**PS2. Skiving kinematics and ideal cut (PS-D3).**
- `SkivingAxes` (frame chain incl. Σ); an `IdealSkivingJob` driver; multi-pass radial infeed.
- The existing rake sweep cuts the ring stock.
- Tests:
  - the demo ring: tooth thickness within 0.5 µm of target after the final pass;
  - feed marks match an oracle formula (pinned with a cited source);
  - pass count and depth split are reproduced;
  - every edge pass timed between control ticks.

**PS3. Internal-gear grading check.**
- Confirm `GearMeasurement` covers internal teeth end to end on ring-stock output (profile
  evaluation ranges for internal flanks, between-ball size). Extend it where missing.
- Tests: a perfect synthetic internal gear grades class ≤ 1; injected eccentricity gives
  `Fr ≈ 2e`; the ideal skived reference ring grades class ≤ 3.

### Phase B: the machine and its control

**PS4. Parts (PS-D1, PS-D5).**
- `HighSpeedWorkSpindle` (direct-drive motor spindle, speed-dependent runout and heat fields,
  encoder).
- `SkivingSpindle` (cantilevered, slim nose, shrink-fit or HSK-E interface).
- `SwivelHead` range ±30°.
- `RingChuck` (diaphragm or face clamp, centring bore, distortion field).
- Shared chassis parts from the hobber's revised moving-column layout.
- Tests: ratings, BOM, fields carried into the assembly description.

**PS5. The machine.**
- `CircularSplineSkiver` on the shared chassis.
- Checks:
  - tool-inside-ring clearance over the Σ range, infeed and axial travel;
  - insert-only precision mounts;
  - rail margins;
  - exact-solid audit.
- Project, Start-page entry ("Gear cutting"), app load check.

**PS6. High-speed device gearbox (PS-D4).**
- RKD6 device core: encoder input channels with hardware-latched timestamps; the follow mode
  with exact (integer) ratio arithmetic; a 20–50 kHz loop; sync-error reporting.
- Virtual device and simulation use the same code.
- Jitter, latency and resolution parameters.
- Tests:
  - sync error vs loop rate and jitter matches the analytic budget;
  - 1 µs of jitter at reference speeds is reported as exceeding class-6 pitch;
  - the hardware-timestamped configuration fits;
  - changing the tooth ratio needs no rebuild.

**PS7. Skiving CAM (PS-D6).**
- `SkivingJob` with multi-pass infeed, speeds, overrun and retract. ToolpathKit ops.
- Tests:
  - overrun covers full engagement (from geometry);
  - planned time matches the pass sum to 1 %;
  - a Σ or tool the bore cannot take is rejected.

**PS8. Cutting in the app.**
- A skiving stock follower from measured states; worker preview; gear report.
- Tests: the demo ring cut end to end with a grade, cycle time, sync error and no collisions.

### Phase C: physics, errors, acceptance

**PS9. Forces and chatter risk (PS-D7).**
- Loads from chip areas; peak and spectrum; spindle power and torque ratings; chatter-risk
  indicator.
- Tests: forces scale with feed and depth per pass; ratings keep the stated reserve.

**PS10. Error sources and budget (PS-D7).**
- The named sources on their parts; analytic or coarse vs fine strategy; combined run.
- Tests:
  - each source shows its signature;
  - jitter maps to pitch error as PS-D4 predicts;
  - coarse vs fine agreement is stated.

**PS11. Acceptance.**
- The reference circular spline grades class 5–6 (extrapolated).
- A cheaper specification (a host-closed 4 kHz loop, or 1 µs jitter) fails, with the gearbox
  named.
- Cycle time stated with its breakdown.
- A report of every assumed number.

### Phase D: later

- Hard skiving after heat treatment; tool wear and regrind; tool-life model.
- The tooth-design plan's double-circular-arc circular spline.
- External skiving of flexsplines as an alternative to hobbing (compare cycle time and grade).
- A coarse skiving option on the lathe (live tool) for non-critical gears.
- Gear shaping and wire EDM as modelled prototype routes.
- G-code dialect.
- Robot loading (shared with the hobber).
- FEA modal check for chatter (casting plan Phase C).

## Order

```
PS0 ── PS1 ── PS2 ── PS3 ─────────────┐
PS4 ── PS5 ── PS6 ── PS7 ─────────── PS8 ── PS9 ── PS10 ── PS11
```

- Phase A needs only gearkit and stockkit, so it can start once the hobber branch is on local
  main.
- PS4–PS5 wait for the hobber's moving-column layout (HB-D1 revision).
- PS6 is the device-side gearbox. The hobber can adopt it too. Coordinate with whoever owns
  RKD6 (`robotkit/device_protocol`).

## Working rules

- Start after the flexspline-hobber branch lands on local `main`, in its own worktree.
  Dependencies count only once they are on local `main`.
- A blocked step is marked `blocked: <what>` in Progress.
- Commit per step, e.g. `Circular-spline skiver: skiving tool design (PS1)`, with Progress
  updated in the same commit.
- Diagnose before working around. Numbers not from a datasheet, a source or a measurement are
  marked assumed.
- No legacy compatibility.
- Heavy suites go through the heavy-run wrapper, with measured memory needs.

## Progress

| Step | State | Commits |
|---|---|---|
| PS0 | planned | |
| PS1 | planned | |
| PS2 | planned | |
| PS3 | planned | |
| PS4 | planned | |
| PS5 | planned | |
| PS6 | planned | |
| PS7 | planned | |
| PS8 | planned | |
| PS9 | planned | |
| PS10 | planned | |
| PS11 | planned | |
| Phase D | later | |
