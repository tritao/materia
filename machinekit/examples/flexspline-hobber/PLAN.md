# Flexspline hobbing machine plan

**Goal.** A precision gear hobbing machine for strain-wave flexsplines, designed in materia from
real catalogue-class parts so it could later be built. In the editor it cuts the teeth of a
flexspline the way the real machine would, and grades the result like a gear lab.

1. A flexspline blank (a thin-walled cup, already turned and hardened) sits on an expanding mandrel
   on a direct-drive rotary table (C). A carbide hob on a direct-drive spindle (B) generates the
   teeth. The hob head infeeds radially (X), feeds along the work axis (Z), shifts along its own axis
   (Y) and is set to its lead angle (A).
2. The controller links table to hob through an **electronic gearbox**:
   `C = (starts/teeth)·B + differential terms + programmed C`. There is no mechanical link, so
   hob/table sync error is a real output of the simulation, not something hidden by a constraint.
3. CAM turns a target gear (a flexspline tooth) plus a hob into a hobbing program: speeds, axial
   feed per work revolution, infeed passes, hob shift, crowning.
4. The machine runs the program through the MotionKit/RobotKit stack. The simulation cuts the teeth
   on a gear-specific stock at micrometre resolution, from the hob's real cutting edges.
5. A virtual gear-measuring step reports pitch, cumulative pitch, runout, profile, helix and tooth
   thickness, and grades them against ISO 1328-1. With named error sources switched on (table
   encoder, mandrel runout, hob runout, servo sync, wall deflection), it produces an error budget
   that says which part limits the grade.

**Target:** prototype batches of our own flexsplines, about sizes 14–25, module 0.2–0.5, at
**ISO 1328 class 5–6**. Class 3 is an optional later phase (Phase E, HB13–HB16). Class 1–2
(Mitsubishi GE15FRplus class) is out of scope. Background, sources and reference machines are in
`machinekit/STRAIN_WAVE_RESEARCH.md`.

The work spans kits:
- `gearkit` (new): tooth profiles, conjugate rack, hob design, gear metrology, ISO 1328 grading;
- `stockkit`: the ring stock (`RingStock`) and the hob-edge sweep;
- `machinekit/haxe/src/machinekit/hobbing/` and `machinekit.motion`: torque motor, rotary table,
  hob spindle, bearings, mandrel, hob part;
- `machinekit/examples/flexspline-hobber/`: the `FlexsplineHobber` assembly, checks, preview,
  project;
- `robotkit` runtime and `motionkit`: the electronic gearbox (a follow relation on measured
  master positions), table limits derived through the gear;
- `toolpathkit`: `ElectronicGear` / `ReleaseGear` ops and per-revolution feeds;
- `camkit`: `HobbingJob`;
- `app`: real-time ring-stock cutting, gear report panel.

## What exists

Surveyed 2026-10-17 on local main 4a18a19b7.

**Machines and parts.**
- `AxisBuilder` / `MachineAssembly` (`machinekit/haxe/src/machinekit/assembly/`) provide
  `place/attach/slide/driveScrew/addMotor/addEncoder/addTransmission`.
- `examples/bench-mill/` is the closest template. It has:
  - cast bodies (`milling/MillCasting.hx`), HGR15 rails, SFU1605 ball screws with BK/BF supports;
  - servos with `MotorDriver`, `ShaftEncoder`, a Z brake (`ShaftBrakePart`);
  - a belt-driven spindle as a continuous revolute joint with `ProcessVelocityDrive` and
    `addAtSpeed`.
- `gantry/RotaryHead.hx` (`CHead`, `CaHead`) is the only rotary-axis precedent. It is a servo plus
  gearhead, not direct drive.
- `motion/ServoMotor.hx` (generic, Assumed), `milling/SpindleMotor.hx` (1.1 kW, 8000 rpm rated,
  Assumed), `motion/LinearScale.hx`, `motion/ShaftEncoder.hx`.
- `transmission/SpurGear.hx` gives involute geometry: 6 samples per flank, a straight radial step
  instead of a root fillet. `Rack.hx` and `GearPair.hx` provide ratios. `AssemblyIntegrity`
  checks mesh consistency.

**Couplings and motion.**
- Multi-leader couplings `follower = Σ(ratioᵢ·leaderᵢ + offsetᵢ)` run end to end (X5, CoreXY):
  - projectkit `AssemblyJointCoupling`, RobotKit `JointCoupling`;
  - the native runtime derives followers;
  - SimKit enforces them (MuJoCo `mjEQ_JOINT` / tendon);
  - MotionKit `MotionAxisCouplings`.

  They are rigid and fixed at compile time. SimKit's `joint_couple` is add-only at world build.
- `PlanCheck` checks stall, peak and RMS torque, deviation from compliance and backlash, and
  critical speed.
- `EncoderMonitor` checks following error. `EncoderReading` quantises counts (no noise, no
  latency).
- Servo gains come from `RobotRuntimeCompiler` defaults. MuJoCo applies servo-target feedforward.
- Ruckig and TOPP-RA are vendored. `DistanceDrive` slaves a joint to path distance (the extruder
  precedent).
- `ToolpathOp.MachineMove` exists for machine-axis moves. `Spindle(direction, rpm)` is a speed
  event.

**Stock and comparison.**
- StockKit is an exact tri-dexel (C++ core, 16×16 tiles, copy-on-write snapshots, deterministic).
  It provides `StockComparison` (leftover and gouge per ray) and deviation colouring.
- `app/MachiningStock` cuts the chord the simulated tool tip travelled each tick, in the stock's
  frame.
- The foam-cutter branch shows how to add a new sweep kind (`CutMotion.Wire`, `SK_MOVE_WIRE`).

**Conventions.** Plan format, working rules and registration points are as in
`examples/foam-cutter/PLAN.md`:
- `MachineKitSmoke`;
- `ExampleCatalog` + `ExampleBrowser.categoryOf` + `StartPageTests`;
- `ProjectSourceTests` + `app/haxeon.project-source.json`.

## What is missing

**Parts:** direct-drive torque motors; a rotary table; a direct-drive hob spindle; angular-contact
and crossed-roller bearings; a rotary ring encoder; HSK interfaces; a hob arbor with outboard
support; an expanding mandrel; a swivel axis with clamping; a mineral-cast bed; the hob as a tool
part.

**Control:** an electronic gearbox, meaning a follower servo tracking measured master positions
with a ratio set by the program. Also missing: the hob as a fast master axis; feed per work
revolution; servo-loop rate, latency and sync-error modelling.

**Simulation:** a stock that resolves micrometres on teeth of module 0.2–0.5. Tri-dexel at 0.1 mm
already costs 244 MB for 200×200 mm, and the app runs at 0.5 mm. Also missing: tools that are not
surfaces of revolution about +Z (StockKit phase 7 is not started); rotating-workpiece sweeps.

**Gears:** helical, internal and non-involute teeth; hob and worm geometry; conjugate (envelope)
computation; any gear metrology (pitch, cumulative pitch, profile, helix, runout, ISO grading).

**Accuracy:** error sources and budgets. Runout, thermal growth, bearing and structural stiffness
chains, and geometric errors are not modelled anywhere. SimKit fidelity levels
(`simkit/plans/FIDELITY.md`, SF0–SF6) are planned on the 3d-printing line, not on main.

## Decisions

**HB-D1. A vertical work axis, with the hob head on a column: axes C, B, X, Y, Z, A.**
- C: work table, direct-drive torque motor, crossed-roller bearing, ring encoder, full closed loop.
- B: hob spindle, direct-drive motorised spindle, velocity-commanded, its encoder is the gear
  master.
- X: radial infeed (sets tooth depth and thickness). Z: axial feed along the work axis.
  Y: tangential hob shift along the hob axis (spreads wear).
- A: hob swivel to the hob lead angle (plus helix angle for helical gears). Positioned, then
  clamped. It is not interpolated during cutting.
- This mirrors the small-module hobbers in the research notes (GE15FRplus, K 160). The cutting
  force is tens of newtons, so the design is about rotational accuracy (table runout, encoder,
  sync), not rigidity.
- **Rejected:** a horizontal work axis (worse for cup blanks on a mandrel and for loading);
  converting a 5-axis mill (no electronic gearbox; the table cannot follow a 3000 rpm master at
  servo rate).

**HB-D2. The electronic gearbox is a controller relation, not a mechanical coupling.**
- A real hobber has no mechanical link between hob and table. The table servo follows the hob
  encoder through the CNC (Fanuc EGB, Siemens EG).
- Runtime relation, set and released by the program:
  `C_target = Σ kᵢ·m̂ᵢ + C_programmed`.
  - `m̂ᵢ` is master i's **measured** position (encoder model), extrapolated over the loop latency
    by its measured velocity, as real EGB feedforward does.
  - Terms: hob B (`k = starts/teeth`), Z (helical differential, zero for spur teeth), Y (shift
    compensation).
- Loop rate, latency and encoder resolution are stated driver and controller data. Sync error,
  `C_actual − Σ kᵢ·mᵢ(true)`, is recorded as a time series.
- MotionKit plans X/Y/Z/A and the additive C. It checks C's combined speed and torque, gear term
  included, against the table's derived limits. B is a process velocity joint, as on the bench
  mill.
- **Rejected:**
  - A multi-leader `AssemblyJointCoupling`. It is rigid (sync error invisible by construction) and
    fixed at compile time, so every tooth count would need a model, blueprint and world rebuild.
  - Planning B as a position leader at 3000+ rpm.

**HB-D3. The flexspline is cut on a ring stock: circumferential dexels.**
- `RingStock` (StockKit core, C++): for each measuring circle at radius `r` and height `z` in the
  work frame, a sorted list of material intervals in θ with exact endpoints and provenance (which
  gash, which pass).
- Why circles: a tooth flank crosses a circle of constant radius almost at right angles, so flank
  positions are exact. Tooth thickness at `r` is an interval length. Pitch is the spacing of
  interval starts. Profile is a flank's θ across radii. Lead (helix) is the same across z.
  Metrology (HB4) reads it directly.
- Size: radial levels from root to tip every 5–10 µm (tooth height ≈ 2.2·m ≈ 0.7 mm), 50–150 z
  levels, ≈ 2·teeth endpoints per circle. Tens of MB at most, so µm resolution is affordable.
- It handles internal teeth (circular spline) unchanged. It also suits a later skiving machine.
- Contouring the rings gives a display mesh. A coarse tri-dexel or mesh is not needed.
- **Rejected:**
  - Tri-dexel at µm spacing (memory).
  - A radial height field `r(θ, z)`: rays graze flanks at the pressure angle, and flank position
    comes from lateral sampling.
  - B-rep booleans: 10⁵–10⁶ gash passes per part.
  - A single 2D slice: misses lead, crowning and axial feed marks. Slice mode remains as a fast
    option, meaning few z levels.

**HB-D4. The hob cuts with its real edges: one rake-face region per gash per thread.**
- Hob geometry (HB1): starts, gashes, outside diameter, lead angle, rake (zero by default), and
  the tooth profile in the rake plane from the hob's basic rack.
- For each gash pass, the rake-face region is swept through the pass and intersected with each
  measuring circle. This gives the removed θ interval, to a stated tolerance (≤ 0.1 µm).
- Enveloping-cut flats (finite gashes) and axial feed marks then emerge from the kinematics. They
  are not added as formulas; the formulas are only test oracles.
- Gash passes are evaluated at their exact times between control ticks, with joint states
  interpolated. At 3000 rpm with 12 gashes, a gash passes every 1.7 ms, which is comparable to the
  control tick.

**HB-D5. Gear geometry and metrology live in a new pure-Haxe `gearkit`.**
- Contents: tooth profiles (involute first, double-circular-arc and S-tooth later), conjugate rack
  and envelope, hob design, flexspline deformation kinematics, metrology, ISO 1328 grading.
- CamKit (`HobbingJob`), MachineKit (`Hob` part, later `SpurGear`), StockKit tests and the app
  all need it. None of them should depend on MachineKit for gear maths.
- It depends only on haxeon std. CadKit solids come through thin adapters where a solid is
  needed.
- **Rejected:** `machinekit.transmission` (CamKit would depend on MachineKit); `cadkit` (gear
  maths is not CAD).

**HB-D6. Quality is judged like a gear lab, on ISO 1328-1:2013 parameters.**
- From the ring stock:
  - single and cumulative pitch `fp`, `Fp`; runout `Fr`;
  - profile `Fα`, `ffα`, `fHα`;
  - helix `Fβ`, `ffβ`, `fHβ`;
  - tooth thickness (span or over-pins equivalent).
- Evaluation ranges follow the standard (profile evaluation range, helix evaluation range).
- Each parameter is graded separately and the overall class is the worst.
- ISO 1328-1 covers module ≥ 0.5. Below that we extrapolate its formulas and say so in the report.
  DIN 58405 (fine-pitch gears) is noted as the alternative.
- Formulas used (√2 step between classes A):
  `fp = (0.001d + 0.4m + 5)·√2^(A−5)` µm, `Fp = (0.002d + 0.55√d + 0.7m + 12)·√2^(A−5)` µm.
  The rest come from the standard. Pin each one against the text in HB4 (check).

**HB-D7. Errors are named sources with a physical home and a ValueBasis.**
- Each source sits on the part that causes it:
  - table encoder eccentricity and graduation error on the ring-encoder part;
  - table radial and axial runout on the crossed-roller bearing;
  - mandrel runout and clamping distortion on the mandrel;
  - hob radial and axial runout on the arbor and outboard support;
  - hob profile and lead errors on the hob;
  - servo loop rate, latency and gains on the driver and controller;
  - flexspline wall compliance from the blank's geometry and material.
- The sim applies geometric sources in the hob-to-work kinematics (HB3). Sync comes from the
  runtime (HB7). Deflection comes from the cutting force (HB10).
- The **error budget** runs the same job with each source alone and with all together. It reports
  each parameter's contributions and names the dominant part, carrying assumed labels as PlanCheck
  findings do.
- When SimKit fidelity levels reach main, these sources map onto the Physical level. Until then
  the hobbing sim owns them.
- **Rejected:** random noise added to final metrics. It explains nothing and cannot drive a design
  choice.

**HB-D8. A hobbing program is a machine-axis program.**
- ToolpathKit ops:
  - `MachineMove` for X/Y/Z/A/C;
  - `Spindle`;
  - new `ElectronicGear(follower, terms)` and `ReleaseGear`;
  - feeds stated per work revolution (`mm/rev`), converted by the compiler from the active gear
    and spindle speed.
- CAM: `camkit` `HobbingJob(target gear, hob, machine, stock)` gives the approach, infeed passes
  (rough then finish), climb or conventional feed, axial overrun from hob and blank geometry, the
  shift schedule, and crowning `X(Z)`.
- G-code (a Fanuc-style EGB dialect, `G81 T… L…`) is later. The IR is what the machine and the
  sim consume.

**HB-D9. The hob is derived from the tooth.**
- The hob's basic rack is the conjugate rack of the target tooth, computed by the envelope
  (gearkit). It is not chosen from a list.
- Involute teeth give a straight-sided rack. Double-circular-arc and S-tooth flexsplines plug in
  later without touching the machine or the sim.
- The target tooth for flexsplines is the undeformed tooth. Its design (from the circular spline
  and the deformation kinematics in the research notes) is a gearkit concern, separate from
  manufacturing.

**HB-D10. Generic names in code.** No trade names in identifiers or catalogue rows; vendors
appear only as sources in docs.

## Test parts

- **Coarse demo gear:** spur, m 1, z 40, α 20°, face 10 mm, solid blank. Fast; visible flats;
  used for interactive runs and smoke checks.
- **Reference flexspline:** involute, m 0.3, z 200 (for z_c = 202, ratio 100), d ≈ 60 mm, face
  8 mm, wall 0.6 mm, 40CrNiMo 38–42 HRC. Size-25 class. The acceptance part.
- **Comparison flexspline:** m 0.5, z 100, face 15 mm, the published GE15FRplus test gear, as a
  plausibility check on the sync and table numbers.
- **Later:** double-circular-arc flexspline (Liu 2025 parameters); a crowned version.

## Reference numbers (assumed until checked against a datasheet or a measurement)

The [spindle drive requirements study](../spindle-drive-study/README.md) expands
these references into a provisional 1 µm tangential control-error allocation,
timing/encoder requirements and 3000/8000 rpm calculations. It compares vector
and servo B drives while retaining HB-D1/HB-D2. The bounds require measurement;
they neither implement HB7 nor establish an ISO grade.

| Quantity | Value | Source |
|---|---|---|
| Reference part | m 0.3, z 200, d 60 mm, face 8 mm, wall 0.6 mm | derived from size-25 class; wall assumed |
| Class 6 limits at d 60, m 0.3 | fp ≈ 7.3 µm, Fp ≈ 23.5 µm | HB-D6 formulas, extrapolated below m 0.5, check |
| Class 5 limits | fp ≈ 5.2 µm, Fp ≈ 16.6 µm | same |
| Angular budget at r 30 mm | 7.3 µm ≈ 50″ (fp), 23.5 µm ≈ 160″ (Fp) | derived |
| Hob | carbide, single start, OD 32 mm, 12 gashes, length 40 mm | assumed; OD as GE15FRplus |
| Hob lead angle | ≈ 0.55° (sin γ = m·starts / d_hob,pitch) | derived |
| Cutting speed | 300 m/min → hob ≈ 3000 rpm | assumed for prototypes (GE15FRplus runs 800 m/min) |
| Table speed in gear | 3000 / 200 = 15 rpm | derived |
| Axial feed | 0.5 mm per work revolution | GE15FRplus example |
| Single-cut time | (8 + ≈ 4 overrun) mm / 0.5 → 24 rev → ≈ 96 s | derived |
| Feed-mark depth | δ ≈ f²·sin α/(4·d_a0) ≈ 0.7 µm | standard hobbing estimate, check |
| Cutting force | tens of N | GE15FRplus paper |
| Hob spindle | direct drive, 8000 rpm max, ≈ 1.5 kW | assumed |
| Table encoder | ≥ 2²³ counts/rev (0.15″), accuracy ±2″ ≈ 0.3 µm at r 30 | assumed |
| Table bearing runout | radial and axial ≤ 2 µm | assumed, crossed-roller precision class |
| Mandrel runout | 3–5 µm → Fr contribution ≈ 2e = 6–10 µm | EMAG flexspline page |
| Position loop / EGB rate | 4–8 kHz, latency one loop period | assumed (MotorDriver stores 4 kHz) |
| Linear scales X/Z | 0.1 µm resolution | assumed |
| Bed | mineral cast | EMAG uses polymer concrete; assumed |

The angular budget is the driver. 50″ for single pitch leaves room for a ±2″ encoder and µm-level
runouts, but not for a one-tick latency without feedforward: 15 rpm × 1 ms = 0.09° ≈ 47 µm at
r 30 mm. HB7 and HB11 must show this.

## Steps

### Phase A: geometry, ring stock, ideal hobbing, metrology (no machine, can start now)

**HB0. Baseline and test parts.**
- Run the gate on the starting commit and record failures that come from main, not from us.
- Scaffold `gearkit` (haxeon.json, tests).
- Tests: the gate's numbers recorded in Progress.

**HB1. Tooth, rack and hob geometry (HB-D5, HB-D9).**
- `gearkit`:
  - `ToothProfile`: involute external and internal, profile shift, tip and root fillets (a real
    trochoid fillet, not a radial step).
  - `ConjugateRack.of(profile)`: numerical envelope.
  - `HobDesign`: from the basic rack, module, starts, OD, gashes, rake. Gives lead angle, axial
    pitch and the rake-face tooth region per gash.
  - `GearSpec`: the target gear with its evaluation ranges.
- Tests:
  - the conjugate rack of an involute is straight-sided at α to 1e-9 mm;
  - the involute's own conjugate rack, re-enveloped, reproduces the profile to 1e-6 mm;
  - lead angle and axial pitch match closed form;
  - z = 200 at m 0.3 gives d = 60 mm;
  - the fillet is tangent to flank and root.

**HB2. Ring stock (HB-D3).**
- StockKit core: `sk_ring_stock` with circles on an (r, z) grid, θ intervals with exact endpoints
  and provenance, snapshots, deterministic multithreading as in the tri-dexel core.
- Haxe: `RingStock` (`annulus`, `fromGearBlank`), `cutRegion` (subtract a θ interval set per
  circle), `contour`, `compare` against a target `GearSpec`.
- Tests:
  - an analytic cut (a planar slab at known depth) gives exact interval endpoints at every circle;
  - memory and time budgets for the reference part at 5 µm × 0.1 mm: below 100 MB, below 0.5 s
    for a full-part snapshot;
  - snapshot and restore are identical;
  - bit-identical results across thread counts.

**HB3. Ideal hobbing (HB-D4).**
- `stockkit` (or `gearkit` + StockKit adapter): `HobbingSweep` takes a hob, the machine's
  hob-to-work frame chain (X, Y, Z, A and table angle as functions of time), and a gash-pass
  schedule. It subtracts each gash's swept rake region from the ring stock.
- Geometric error sources (HB-D7) enter here as frame perturbations, all zero for now.
- A pure-kinematics driver with ideal electronic gearing cuts the test gears without any machine.
- Tests:
  - the coarse demo gear: flats per flank equal `gashes × starts`, within the profile range;
  - flat depths match the enveloping-cut oracle and feed marks match `f²·sin α/(4·d_a0)` (pin the
    formulas with a cited source);
  - tooth thickness at the reference circle is within 0.2 µm of target after full-depth infeed;
  - doubling gashes cuts flat depth about 4×.

**HB4. Gear metrology and grading (HB-D6).**
- `gearkit`: `GearMeasurement.fromRings(ringStock, spec)` gives every HB-D6 parameter, with traces
  (profile and helix charts, pitch deviation per tooth).
- `Iso1328`: tolerance per parameter and class, and the grade per parameter.
- A report object the app and the tests share.
- Tests:
  - a perfect synthetic gear grades class ≤ 1 everywhere;
  - injected eccentricity e gives `Fr ≈ 2e` and a once-per-revolution sinusoid in pitch;
  - an injected profile slope gives `fHα`;
  - the HB3 ideal cut of the reference part reaches class ≤ 3 (only flats and feed marks remain);
  - tolerance values match the standard's tables at m 0.5 (check against the text).

### Phase B: the machine and its control

**HB5. Parts (HB-D1, HB-D10).**
- `machinekit.motion`:
  - `TorqueMotor`: frameless direct drive with continuous and peak torque, rotor inertia, cogging
    amplitude and pole count.
  - `AngularContactBearing`, `CrossedRollerBearing`: with precision class, runout and moment
    stiffness as catalogue rows.
  - `RotaryEncoderRing`: counts, accuracy, eccentricity, as error-source fields.
- `machinekit.hobbing`:
  - `RotaryTable`: torque motor, crossed roller, ring encoder, clamp.
  - `HobSpindle`: motorised direct drive, angular-contact pair, HSK-E interface.
  - `HobArbor` + `OutboardSupport`.
  - `Hob`: the part from a `gearkit.HobDesign`.
  - `ExpandingMandrel`: hydraulic, runout, clamping pressure.
  - `SwivelHead`: servo with worm or direct drive, plus a clamp.
  - `MineralCastBed`, `Column`.
- Catalogue rows are marked Catalog or Assumed. Error-source fields default to the reference
  numbers.
- Tests: part geometry valid; BOM rows; `jointTorque` and limits from ratings; error-source fields
  carried into the assembly description.

**HB6. The machine.**
- `FlexsplineHobber extends MachineAssembly`:
  - joints C (revolute, direct drive), B (continuous, process velocity), X, Y, Z (prismatic, ball
    screws, linear scales), A (revolute, clamped);
  - hob-to-work frame chain exported for HB3.
- `FlexsplineHobberChecks` in MachineKit smoke:
  - FK of the hob centre at travel corners;
  - the hob clears the mandrel and table at the ends of travel;
  - X reaches full depth on the smallest and largest blank;
  - derived axis limits;
  - BOM.
- `materia.project.json`, a Start-page entry in a "Gear cutting" category, `StartPageTests`, and
  `ProjectSourceTests.checkFlexsplineHobber` (loads, MuJoCo holds pose).
- Tests: smoke checks; app check.

**HB7. Electronic gearbox (HB-D2).**
- RobotKit runtime: a follow relation on a joint, `target = Σ kᵢ·m̂ᵢ + planned`, set and released
  by plan events.
  - Masters are measured (encoder model) and extrapolated over the stated latency.
  - Loop rate and latency come from driver and controller data.
  - Sync error is recorded per tick.
- ToolpathKit `ElectronicGear` / `ReleaseGear`. MotionKit plans the additive C. PlanCheck checks
  C speed and torque including the gear term (`B_rpm·starts/teeth`) against the table's limits.
- Tests:
  - changing tooth count between two programs needs no model rebuild;
  - with feedforward, sync error stays below 1″ at 15 rpm under the reference numbers;
  - without extrapolation, the error equals ω·latency (the 47 µm case);
  - a gear ratio that would overspeed the table is rejected by PlanCheck, naming the table joint.

**HB8. Hobbing CAM (HB-D8).**
- `camkit` `HobbingJob`: speeds from cutting speed and hob OD; axial feed per revolution; infeed
  passes; climb or conventional; overrun from hob and blank geometry; shift schedule; optional
  crowning `X(Z)`. It emits ToolpathKit ops.
- Tests:
  - the program for the reference part: overrun covers the full hob engagement at entry and exit
    (computed from geometry);
  - planned cycle time matches `revolutions / table rpm` to 1 %;
  - a hob too short for the shift schedule is rejected.

**HB9. Cutting in the app.**
- `app`: a `HobbingStock` (like `MachiningStock`) runs the HB3 sweep from the simulated joint
  states each tick, gash passes at exact times (HB-D4). Ring-stock preview on the worker, as
  `StockPreviewWorker`. A gear report panel after the cut, showing HB4 traces and grades.
- `ProjectSourceTests.checkFlexsplineHobber` cuts the coarse demo gear end to end. It reports
  grade, cycle time, sync error, no collisions and an allocation budget.
- Tests: app check; the ideal machine (no error sources) gives the HB3 result to 0.1 µm.

### Phase C: forces, error sources, acceptance

**HB10. Cutting forces.**
- Per gash pass, chip area from the ring stock's removed intervals, times a Kienzle specific force
  for 40CrNiMo at 38–42 HRC (kc1.1, mc assumed, marked).
- Outputs: hob torque and power, table reaction torque, radial force on the wall. These feed
  PlanCheck (spindle power, table torque) and HB11's wall deflection.
- Tests:
  - force scales with feed^(1−mc);
  - the reference cut stays in the tens-of-newtons range;
  - spindle power is below its rating with the margin stated.

**HB11. Error sources and budget (HB-D7).**
- Turn on each source from its part:
  - encoder eccentricity and graduation error;
  - table radial and axial runout;
  - mandrel runout;
  - hob radial and axial runout and profile error;
  - servo sync (HB7 runtime);
  - wall deflection (ring compliance of the blank under the HB10 radial force).
- `ErrorBudget` runs the job per source and all together, and reports contributions per
  parameter and the dominant part.
- Tests (each source alone shows its textbook signature):
  - mandrel eccentricity → `Fr ≈ 2e` and once-per-revolution `Fp`;
  - hob radial runout → profile errors repeating every hob revolution;
  - hob axial runout → `ffα`;
  - sync latency without feedforward → pitch error;
  - table encoder eccentricity → `Fp`, once per revolution;
  - the sum of sources roughly matches the all-sources run (state the tolerance, since effects are
    not strictly additive).

**HB12. Acceptance.**
- The reference flexspline with the reference parts grades **ISO 5–6** on every parameter. The
  report shows the budget.
- A cheaper specification (2¹⁸-count encoder, 10 µm mandrel, 4 kHz loop without feedforward)
  fails class 6. The report names the causes in the right order.
- The comparison part (m 0.5, z 100) with tightened parts reaches class ≤ 3, as a plausibility
  check against the published class-1 result. Cycle time for the reference part is ≤ 2 min.
- A short report of the numbers in this plan, with every assumed value listed.

### Phase D: later
- Double-circular-arc and S-tooth flexsplines; crowning and longitudinal modification.
- Thermal growth (spindle and table heat, bed); hob wear and shift strategy.
- G-code: a Fanuc-style EGB dialect in CncKit (writer and reader).
- Loading automation in a tending cell (reuse machine-tending).
- The sister machines: circular-spline power skiving (same ring stock, internal teeth) and the
  wave-generator non-round turn/grind machine.
- Buildable output: drawings, sourcing and a controller choice.

### Phase E: ISO class 3 (optional, later)

Only if the prototype target (HB12) holds and finer gears are wanted.

**Why it changes the work.** Class 3 is 2.8× tighter than class 6 (√2 per class). The limits for
the reference part are below. They come from the HB-D6 formulas, extrapolated below module 0.5,
with Fr and Fα rough; all of them need checking.

| Parameter | Class 6 | Class 3 | Table angle at r 30 mm (class 3) |
|---|---|---|---|
| fp | 7.3 µm | 2.6 µm | ≈ 18″ |
| Fp | 23.5 µm | 8.3 µm | ≈ 57″ |
| Fr (≈ 0.8–0.9·Fp) | ≈ 20 µm | ≈ 7 µm | total eccentricity ≲ 3.5 µm |
| Fα | ≈ 6–7 µm | ≈ 2–2.5 µm | — |

Class 6 is mostly a sync problem. Class 3 is a datum and rotation-accuracy problem. The 3–5 µm
mandrel runout in the reference numbers is fine for class 6 and not enough for class 3.

**HB13. Class-3 error sources.**
- Add the sources that are negligible at class 6:
  - **encoder:** sub-divisional (interpolation) error at the scale-line frequency; two read heads
    cancelling ring eccentricity;
  - **table drive:** torque-motor cogging, giving table speed ripple and from it pitch error;
    seal drag variation (contact vs non-contact seal);
  - **workholding:** clamping distortion of the thin wall; bore roundness and wall-thickness
    variation of the blank (the datum);
  - **hob:** profile error by hob grade (DIN 3968 class AA or better); hob wear against shift
    position.
- Each source lives on its part with a ValueBasis, as in HB-D7.
- Tests: each source alone shows its signature:
  - sub-divisional error gives a tooth-to-tooth pitch pattern at the beat of line and tooth
    frequency;
  - cogging gives pitch error at pole-pass frequency;
  - clamping distortion gives a two- or three-lobe runout.

**HB14. Thermal drift.**
- A lumped thermal model: spindle, table motor and column. Heat comes from motor losses and
  cutting power (HB10). Coolant is held at machine temperature or not; the room has a stated band.
- The output is X drift and C-axis tilt over a cut and over a batch.
- Tests:
  - a cold start without warm-up drifts tooth thickness by more than the class-3 band;
  - with temperature-matched coolant and warm-up, drift stays within it (numbers assumed, marked).

**HB15. Runout probe and compensation.**
- After clamping, a probe cycle measures part eccentricity on the turned diameter or the bore. The
  controller then corrects C(θ) and X(θ) during the cut.
- Program ops `ProbeRunout` and `RunoutCompensation`; HobbingJob emits them when asked.
- Tests:
  - 4 µm injected mandrel eccentricity without compensation fails class 3 Fr;
  - with compensation, the residual is set by probe uncertainty and table runout only;
  - the compensation is refused when the measured runout exceeds a stated limit.

**HB16. Class-3 acceptance.**
- A class-3 specification. Required unless the budget says otherwise:
  - a mandrel ground in place (≤ 1 µm);
  - a table with ≤ 0.5 µm non-repeatable runout, a non-contact seal and a low-cogging motor;
  - a ±1″ two-head encoder;
  - an 8 kHz feedforward gear loop;
  - an AA ground carbide hob with ≤ 1–2 µm mounting runout;
  - temperature-matched coolant;
  - runout compensation.
- The reference flexspline grades class 3 on every parameter.
- The error budget ranks the upgrades from the class-6 specification by contribution. It shows
  which can be dropped and still hold class 3.
- The report notes the real-world prerequisite: verifying class 3 on a real part needs a gear
  measuring machine with ≤ 0.5 µm uncertainty and a ≈ 0.3 mm probe.

## Order

```
HB0 ── HB1 ──┬── HB3 ── HB4 ───────────────┐
HB2 ─────────┘                              │
HB5 ── HB6 ── HB7 ── HB8 ──────────────── HB9 ── HB10 ── HB11 ── HB12
```

- Phase A (HB1–HB4) needs no machine and can run in parallel with HB5–HB6.
- HB2 (StockKit core) is independent of HB1 except for its tests against `GearSpec`.
- HB9 needs both lines. HB11 needs HB7 (sync) and HB10 (force).
- Phase E (HB13–HB16) is optional. It starts only after HB12 and runs HB13 → HB14 → HB15 → HB16.

## Working rules

- Work in a worktree `flexspline-hobber` from local `main`. Dependencies count only once they are
  on local `main`.
- A blocked step is marked `blocked: <what>` in Progress, and work goes on to the next unblocked
  step.
- Commit per step, e.g. `Flexspline hobber: ring stock (HB2)`, with Progress updated in the same
  commit.
- Diagnose before working around. Numbers not from a datasheet or a measurement are marked
  assumed.
- No legacy compatibility: one schema version, reject others.
- Heavy suites go through the heavy-run wrapper.

## Progress

| Step | State | Commits |
|---|---|---|
| HB0 | blocked: baseline waiting for heavy-run slot; scaffold tested | this commit |
| HB1 | planned | |
| HB2 | planned | |
| HB3 | planned | |
| HB4 | planned | |
| HB5 | planned | |
| HB6 | planned | |
| HB7 | planned | |
| HB8 | planned | |
| HB9 | planned | |
| HB10 | planned | |
| HB11 | planned | |
| HB12 | planned | |
| Phase D | later | |
| HB13–HB16 | optional, later (class 3) | |

### HB0 start (2026-10-17)

- Starting commit: `8c70c2190` (plan on top of local main `4a18a19b7`).
- Added dependency-free `gearkit`, its workspace test entry and nominal `GearSpec` dimensions.
  The three plan parts have pitch diameters 40, 60 and 50 mm. Detailed profile geometry and
  evaluation ranges remain HB1 work.
- `./haxeon/scripts/haxeon run --project=gearkit/tests/haxeon.json`: **18 assertions passed**.
- Baseline workspace gate: **pending**, waiting for a shared heavy-run slot. No baseline
  pass/failure counts are available yet. The starting workspace manifest is pinned under
  `build/flexspline-hb0/baseline.workspace.json`, with absolute project paths and a separate
  build directory. Log: `/tmp/flexspline-hb0-baseline.log`.
- Baseline command: `/home/joao/dev/materia-cache/tools/heavy-run --mem 16G --need 8G
  --label flexspline-hb0 -- ./haxeon/scripts/haxeon workspace test
  --workspace build/flexspline-hb0/baseline.workspace.json --jobs 2 --compilers 1`.
- Existing vendor symlinks and dirty `haxeon` submodule were present before work began and
  remain untouched; the gate therefore measures this local environment, not a pristine clone.
