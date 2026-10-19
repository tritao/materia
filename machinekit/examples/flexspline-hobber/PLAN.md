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

**HB-D1. A vertical work table with a moving column (Mitsubishi GE class): axes C, B, X, Y, Z, A.**
- Pure hobbing. Hard turning, bore datums and thin-wall turning belong to
  `machinekit/examples/lathe/PLAN.md`, a separate machine/session.
- C retains the direct-drive `RotaryTable`: torque motor, crossed roller,
  ring encoder/full closed loop and the specified non-contact seal arrangement.
- X moves the complete column on preloaded HGR25 bed guides toward the table;
  a 16×5 ground ball screw (C5 procurement target, assumed), preloaded nut and
  linear scale provide radial infeed. X retracts to 250 mm for top loading.
- Z is on the recessed column face, with 200 mm travel. A is bolted directly to
  the front of the vertical Z saddle, radial pivot through the hob-centre line.
  Its compact concentric bearing/motor/clamp drives a vertical swivel plate;
  Y uses twin rails on that plate’s front face, with a vertical spindle saddle.
  No overhead A-yoke roof or underhung horizontal Y plate remains.
  Target range: flexspline OD 25–140 mm, faces up to 20 mm.
- Target hob-centre distance from the nearest X and Z guide planes ≤ about
  150 mm. Report actual guide-plane offsets and moving masses, not an inferred
  structural accuracy. Structural/modal adequacy remains a Phase D open check.
- The outboard roller/needle support is radially stiff, nominally zero-clearance,
  and axially floating; the spindle end is the axial locator. Do not install a
  deep-groove bearing or a second axially locating preloaded pair there.
- The riser and mandrel base are integral, retaining the height needed for ±15°
  swivel clearance. Provide coolant flushing, a sloped table guard and chip tray,
  rail protection and enclosure/top-loading provisions; later robot loading
  reuses machine-tending.
- Machine radial direction is global Y and hob axis global X. Apply the explicit
  constant Rz(+90°) from the logical work frame; retain `HobbingAxes` equations
  in the work frame and regress cut geometry, synchronisation and force loads.
- Precision foundation mounts land on cast-in steel inserts using shared
  `machinekit.structural` recipes. All rail-seat inserts must carry guides.
- [MHI's GE15FRplus paper](https://www.mhi.com/technology/review/sites/g/files/jwhtju2326/files/media/pdf/e573030.pdf)
  reports DD/full-loop table control, 0.3 µm table NRRO and ISO class 1 pitch
  on its stated m0.5 example. Those are reference benchmarks, not our ratings.
- **Rejected:** the overhead X bridge (≈350 mm guide offsets, single Y rail);
  horizontal turn-and-hob for this project, since turning belongs to the lathe.
  [EMAG identifies K 160 as horizontal](https://www.emag.com/products-services/machines/gear-cutting-machines/gear-hobbing-machines/horizontal-gear-hobbing-machines/koepfer-k-160/):
  it must not be described as the vertical GE architecture. No claim is made
  that all horizontal machines have the same accuracy class.

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
- ISO 1328-1 covers module ≥ 0.5. Below that we report experimental extrapolated formula grades, never an ISO class.
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
| Single-cut feed time | (8 + 2 × 4.599 engagement overrun + 2 × 1 clearance) mm / 0.5 → 38.396 rev → 154.398 s nominal (154.408 s planned) | HB8 cylinder envelope and MotionKit S-curve; excludes positioning/settling |
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
  - the coarse demo gear: generating facets follow the active flank's roll window sampled at
    `DeltaC = 2*pi*starts/(teeth*gashes)`; more gashes refine and more starts coarsen the pattern
    (corrected during HB3; the original `gashes * starts` assertion was not kinematically valid);
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
  - the HB3 ideal cut of the reference part reaches experimental extrapolated grade ≤ 3
    (only flats and feed marks remain; module 0.3 is outside ISO scope);
  - tolerance values match independently calculated m 0.5 fixtures against the 2013 equations
    (the standard supplies formulas, not a numeric table for this fixture).

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
  - `mineral-cast` material in `projectkit` `MaterialLibrary`: density ≈ 2400 kg/m³,
    Young's modulus ≈ 35 GPa, plus loss factor (damping, ≈ 6–10× cast iron), thermal
    expansion (≈ 12–16 µm/m/K) and conductivity, all marked assumed. If `MaterialDef`
    has no damping/thermal expansion fields, add optional physical fields used only
    where a check reads them.
  - `MineralCastBed` and `MineralCastColumn`: designed bodies with thick walls and ribs,
    and cast-in steel inserts as child parts (rail seats, table mounting plate,
    threaded bushes, lifting points). Rails, the rotary table and the column mount on
    inserts, never on the casting body. Each insert carries a stated position tolerance
    and a "ground after casting" finish note. BOM: mineral casting by weight, inserts
    listed separately.
- Catalogue rows are marked Catalog or Assumed. Error-source fields default to the reference
  numbers.
- Tests: part geometry valid; BOM rows; `jointTorque` and limits from ratings; error-source fields
  carried into the assembly description; mass and inertia from volume × density;
  every precision mount lands on an insert (an `AssemblyIntegrity` style check naming
  the offending mount); insert faces are coplanar or parallel within stated tolerance.
- Out of scope here: a general casting process model (walls, draft, cores, mould design)
  and structural/modal analysis requiring an external FEA solver such as CalculiX.
  Bed/column modes versus the ≈ 600 Hz gash frequency remain an open Phase D check.
  HB14 covers bed thermal behaviour as a lumped thermal mass.

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
- Whole-machine mechanical clearance: resolve non-cutting exact-solid interferences
  at assembly pose and across the permitted axis travel. Model actual seats,
  passages and nut mounts; do not waive overlaps through as-designed collision
  exclusions. Intended hob/stock cutting engagement is reported separately.
- The reference flexspline with the reference parts grades **ISO 5–6** on every parameter. The
  report shows the budget.
- A cheaper specification (2¹⁸-count encoder, 10 µm mandrel, 4 kHz loop without feedforward)
  fails class 6. The report names the causes in the right order.
- The comparison part (m 0.5, z 100) with tightened parts reaches class ≤ 3, as a plausibility
  check against the published class-1 result.
- Total measured cycle time for the reference part is **≤ 3 min** for the prototype target.
  The HB9 report states the measured breakdown: positioning, spindle ramps, infeed passes
  and feed. HB8 gives 154.4 s for feed alone at the assumed 300 m/min, including 4.6 mm
  engagement overrun per end of the 8 mm face.
- A short report of the numbers in this plan, with every assumed value listed.

### Phase D: later

- Reference-part total cycle time **≤ 2 min** remains a later goal. Candidate levers:
  higher cutting speed with a carbide hob (e.g. up to the GE15FRplus 800 m/min class),
  a two-start hob, coarser roughing feed with a finishing pass, and shorter overrun
  from a smaller hob OD. Each must be checked against HB11's error budget; none is
  assumed to preserve accuracy for free.

- Casting process model and structural/modal analysis (external FEA), first use: hobber bed modes vs gash frequency.
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
| HB0 | blocked: baseline waiting for heavy-run slot; scaffold tested | d403e0295 |
| HB1 | done: non-undercut geometry; internal rack-limit restrictions below | 647b62f66 |
| HB2 | done: native ring stock, Haxe API, contour and GearSpec comparison | 94f246a1e |
| HB3 | done: actual rake sweep, ideal driver and slice acceptance; flat-count correction below | 2ec3bdd45 |
| HB4 | metrology implemented/tested; accelerated full-face acceptance running | Eight threads, exact inspection domains and physical face ends; completed grades pending. |
| HB5 | done: reference parts plus ribbed mineral-cast foundations and separate steel inserts | 13358ee07 + f59fcf695 |
| HB6 | mechanics/scene tested; blocked: Start-page/project-load/MuJoCo acceptance queued | 45dd31967 |
| HB7 | done: measured-master gearbox, timed events, additive C drive checks and reference sync tests | a71f86f61 |
| HB8 | done: geometry-derived hobbing CAM, revolution feeds, shifts/crown, installed limits and compiled cycle checks | this commit |
| HB9 | coarse app smoke passed; precision acceptance open | Headless cut, inspection, fixture envelope, bounded stock and coarse sync checks pass. |
| HB10 | full threaded reference force and static ratings passed | Complete small cut, interval-derived loads and spindle reserve pass. |
| HB11 | quick physical sources passed; reference budget pending | Eccentricity, hob errors and combined coarse cuts pass; full reference source studies remain open; assumed-support wall allocation passes. |
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

### HB1 geometry (2026-10-17)

Implemented in dependency-free `gearkit/haxe/gearkit/`:

- `GearSpec`: external/internal normal-system spur gears, profile shift, explicit cutter-tip
  and tooth-tip radii, and geometric evaluation windows. Defaults are a 0.38-module cutter
  radius and a 0.05-module tip radius, both **assumed design choices**. The default evaluation
  range covers the unrounded flank and full face; it makes no ISO compliance claim (HB4).
- `ToothProfile`: analytic involutes, a rounded-rack rolling root envelope, tangent circular
  tip rounding, derivatives and a sampled closed one-pitch tooth patch. Low tooth counts,
  excessive shift, pointed teeth, overlapping fillets and undercut roots are rejected.
- `ConjugateRack.of`: solves normal contact by 64-step bisection at each sampled point.
  It recovers the straight flank and rounded cutter tip from tooth coordinates and derivatives,
  removes the profile shift, and retains the cutter-root blend conjugate to the tooth-tip
  rounding. A sharp tooth tip is handled through its normal cone. Sample count is configurable;
  the sampled curved edges are not an exact B-rep or an HB3 sweep-tolerance guarantee.
- `ConjugateRack.envelopeAt`: generates a gear from the recovered rack's sampled slope, without
  using the target involute formula. It can generate a different tooth count and shift.
- `HobDesign`: right-hand, normal-module hob; starts, OD, gashes, rake; pitch diameter,
  lead angle, axial pitch and lead; closed rake-plane regions per gash/thread/axial turn.
  Gash regions include helical phasing and the rounded tip and root. The spindle axis is +Z.
  Axial length clipping, relief and sharpening allowances are not modelled in HB1.

Conventions and derivation for review:

- In a rack frame with tangent coordinate `u`, outward coordinate `v` and signed pitch radius
  `q` (negative for internal gears), rolling maps a point through
  `P = sign(q) R(phi) (u + q phi, q + v)`. For a smooth rack `u(v)`, normal contact gives
  `u + q phi = -v / (du/dv)`. The rounded cutter tip is enveloped through the same transform;
  no circular blend is substituted at the gear root.
- Internal teeth use the signed rack-limit construction. A physical pinion-shaped cutter's
  internal root depends on that cutter's tooth count and is **not** modelled here. The formal
  internal rack root also has an undercut limit; for example the unshifted 202-tooth reference
  circle is supported, while a 40-tooth internal gear with the default corner is rejected.
  Some internal tip roundings give a folded conjugate cutter root: flank conjugacy is available,
  but `HobDesign` rejects the folded cutter region. This does not assert internal hobbing feasibility.
- Normal module and the recovered cutter-tip height determine the hob pitch diameter:
  `d_p = OD - 2 h_tip`, `sin(gamma) = starts*m/d_p`,
  `p_axial = pi*m/cos(gamma)`, `lead = starts*p_axial`.
- Geometry references: Douglas Wright, [UWA machine-element notes, Appendix B](https://danotes.mech.uwa.edu.au/DANotes.pdf),
  and [Fetvaci and Senturk, rack-cutter/trochoid generation study (2024)](https://www.jstage.jst.go.jp/article/jamdsm/18/8/18_2024jamdsm0098/_article/-char/en).
  The implementation uses the coordinate conventions above and independently derived equations.

Validation:

- Haxeon host: **532,368 geometry assertions + 18 baseline dimension/validation assertions**.
- Pinned reference Haxe interpreter: same assertions and results.
- Maximum measured straight-rack error: **1.33e-14 mm** (required <= 1e-9 mm).
- Maximum measured involute re-envelope error: **2.51e-14 mm** (required <= 1e-6 mm).
- Checked root/flank/root-circle and tip/flank/tip-circle tangency, independent finite-difference
  root derivatives, external/internal and shifted gears, cross-gear rack generation, sharp tips,
  explicit evaluation windows, positive/negative rake, multi-start phase and rake-plane membership.
- `haxeon package check --project=gearkit/tests/haxeon.json`: dependency graph valid, no native deps.
- Local formatter diagnosis: directory inputs crash its HashLink file reader; with explicit
  files it splits scientific literals (`1e-12` into `1 e - 12`) and the shift operator (`>>`).
  Those rewrites were repaired and both compiler runs above were repeated successfully. No
  formatter/runtime changes were made in the existing dirty `haxeon` submodule.
- HB0's full workspace baseline remains queued through heavy-run, with no test counts available.
  No workspace pass is claimed. Existing vendor/submodule changes remain untouched.

Reproduce the focused checks:

```sh
./haxeon/scripts/haxeon run --project=gearkit/tests/haxeon.json
./haxeon/.tools/haxe/haxe -cp gearkit/haxe -cp gearkit/tests/src --run GearKitTests
./haxeon/scripts/haxeon package check --project=gearkit/tests/haxeon.json
```

### HB2 ring stock (2026-10-17)

- Added `sk_ring_stock`/snapshot handles and a C++ ring core to the existing StockKit library.
  C ABI version is now **4**; ring input structs require their exact current size. The HXI was
  regenerated by `core/tools/check-hxi.sh`, with a passing canonical ABI audit for Linux x86-64,
  Windows x86-64 and macOS x86-64/arm64.
- Cell-centred circles cover explicit annular and axial bounds. Radial and axial cell widths
  divide those bounds and do not exceed the requested spacing. Material intervals are sorted,
  disjoint, in `[0,2*pi]`, with independent gash/pass provenance at each endpoint. Original
  boundaries use `SK_SOURCE_STOCK`. The 0/2pi seam is representational, not a physical surface.
- `cutRegion` subtracts angular interval sets in input order, including wrapping/negative starts,
  overlaps, full-circle removal and zero sweeps. Recutting preserves existing boundary sources.
  The entire batch is validated before mutation. Workers own radial layers; staging and committing
  their immutable replacements also leaves geometry unchanged on allocation/worker failure.
- Snapshots share radial layers, copy a changed layer on mutation, survive their original owner,
  and restore onto any stock with identical bounds/counts. Thread count never affects circle data.
- Added Haxe `RingStock.annulus`, `fromGearBlank`, `cutRegion`, `cutSlab`, `circle`, `snapshot`,
  `restore`, `contour`, `compareStock`, and `compare(GearSpec)`, with explicit disposal and closed
  handle checks. `RingGrid`, `RingCut`, `RingInterval`, `RingSnapshot` and `RingComparison` carry
  the associated data. Lengths are **mm**, in agreement with GearKit; Cartesian Stock remains metres.
- `fromGearBlank` takes wall thickness **below the tooth roots**, not from the blank OD; that
  keeps a continuous flexspline wall under the teeth. A whole turned cup/bottom is outside this
  circumferential tooth-band model. The 0.6 mm root-wall interpretation is an assumed geometry
  choice, explicit in the API and example.
- Gear comparison solves circular crossings of the active involute and both fillets from HB1,
  mirrors/repeats the tooth about +Y, and applies the face limits. Comparison values are radians
  per circle; arc lengths are mm and volume estimates use exact annular-sector cell weights.
  It does not apply ISO evaluation ranges or claim ISO grades (HB4).
- The display contour emits only the boundary of the piecewise radial/axial cells, with angular
  chords limited by the requested angle. Position/normal/index/colour/source streams reuse
  `StockMesh`; sources are pass IDs and coordinates are mm. Display discretisation does not
  modify the stored endpoints. The rings' full endpoint provenance remains available for metrology.
- Diagnosed an inherited `Pool` resize crash: a new worker with `seen=0` could observe an old
  nonzero round and execute a null completed job. A regression reproduced SIGSEGV before the
  fix. `Pool::stop` now resets the completed round/job state before replacement workers start;
  repeated 1/4/2-thread changes and recuts pass with bit-identical data.

Validation results:

- Native ring suite: **24,224 checks passed**, through the C ABI, including independent analytic
  slab endpoints, wrapped/overlapping cuts, provenance, atomic rejection, stale handles, snapshot
  restoration, 1/2/4-thread identity, pool resizing, and contour winding/chordal annulus volume.
- Existing native StockKit suite: **20,906 / 20,906 checks passed**. CTest passed both suites;
  the ring suite was rerun after adding the independent contour-volume oracle.
- Haxe ring suite: **345 assertions passed**, including native FFI reads/streams/disposal,
  external/internal gears with shifts -0.2/0/+0.2, independent pitch-circle thickness, root/tip
  fillet crossings, face limits, leftover/gouge and annulus-volume/arc-unit oracles. It is also
  registered as a group in `StockKitTests`.
- Reference budget fixture: **20,400 circles, 2,169,600 intervals, 69,931,352 resident stock
  bytes** (69.9 MB). Tooth-band circles carry all 200 teeth; root-wall circles retain full rings.
  This is an interval-occupancy benchmark, not a simulated hobbed part (HB3).
- Thirty full-part snapshots: worst measured **3.25 microseconds** across focused runs, below
  0.5 s. Process peak RSS about **85,500 KiB**, below 100 MB. Copy-on-write mutation and restore
  of the populated reference part also passed.
- No heavy workspace success is claimed. HB0 remains waiting for a heavy-run slot. Its source
  files are now archived from `8c70c2190` at `build/flexspline-hb0/source`; pre-existing submodule
  checkouts are linked in. The queued baseline manifest points there, so later feature edits cannot
  contaminate the starting-project baseline. This still measures the existing dirty tool/vendor
  environment, not a clean submodule checkout.

Reproduce the focused native build and checks (no CAD dependency):

```sh
cmake -S stockkit/core -B build/flexspline-hb2/native -DCMAKE_BUILD_TYPE=Release
cmake --build build/flexspline-hb2/native -j 2
ctest --test-dir build/flexspline-hb2/native --output-on-failure
```

The Haxe FFI suite was compiled with the existing pinned Haxeon and run against that native
build, without rebuilding unrelated CAD libraries:

```sh
./haxeon/scripts/haxeon build --compiler-only --project=stockkit/tests/ring/haxeon.json
LD_LIBRARY_PATH="$PWD/build/flexspline-hb2/native:$PWD/haxeon/.tools/hashlink:$PWD/haxeon/out${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  ./haxeon/.tools/hashlink/hl stockkit/tests/ring/build/host/main.hl
```

### HB3 ideal hobbing (2026-10-17)

Implemented `stockkit.hobbing`, backed by the native rake-region sweep (C ABI **5**):

- `HobbingAxes`: an explicit proper rigid chain from hob coordinates through X/Y/Z/A/B into
  the inverse work-table C frame. Units are mm, radians and seconds; B/C are unwrapped.
- `GashPass`: engagement windows and exact master-phase crossing times from integer phase
  indices. Constant-speed and controller-sampled schedules include partial boundary windows.
- `HobbingMotion`/`HobbingKeyframe`: piecewise-linear measured/controller states. Scheduling
  solves the measured B crossing within ticks; actual C is interpolated independently. A
  reversing master is explicitly rejected by this scheduler. Motion knots split the sweep.
- `IdealHobbingJob`: a machine-free, positive-speed, right-hand-hob/external-spur driver. It
  sets A to the signed lead angle, C/B to starts/teeth, the target-tooth phase, radial centre
  distance/profile shift, optional axial feed and static hob-shift compensation. Different
  X/Y/Z/A/C or geometric perturbations are accepted through the generic axis sampler.
- `HobbingSweep`: obtains the real per-gash/per-thread regions from `HobDesign`, simplifies
  their polylines below tolerance/8, and prepares a native planar cutter. Axial tooth indices
  are explicit; physical tool-length end clipping and relief remain unmodelled, as in HB1.
- The native cutter ear-clips the concave planar polygons. Between rigid poses it sweeps the
  convex hull of each triangle's two endpoint positions, slices those hulls at each sampled Z,
  and intersects their halfplanes analytically with each circle. The resulting interval union
  removes material through HB2, retaining the scheduled gash/pass. Machine-ulp joins prevent
  false gaps on internal triangle boundaries; endpoints are not put on an angular raster.
  Stationary or in-plane-only face motion sweeps no volume. Input poses must be proper rigid
  transforms. Geometry is staged before committing a pass.
- Actual poses are sampled at engagement ends, the exact gash centre, controller knots and
  adaptively inserted times. No target GearSpec or feed/profile-error formula is supplied to
  the cutter. `cut` restores the initial stock, quarters the midpoint chord tolerance on
  successive refinements (roughly halving angular steps), and compares successive results. It accepts only when the maximum
  circle's angular symmetric difference, converted to arc length, is below tolerance. On an
  invalid motion/pass or failed convergence, it restores the incoming stock.
- This is **observed numerical convergence on sampled circles**, not a proof of an arbitrary
  continuous trajectory's global error or a physical machine grade. The sampler must resolve
  nonlinear motion with supplied knots/maximum time step. Defaults use a 0.05 micrometre arc
  tolerance; HB3 does not claim full-face reference-job throughput or real-time playback.

Acceptance correction and independent oracles:

- The original `flats per flank = gashes * starts` assertion was incorrect. One gash advances
  the table by `DeltaC = 2*pi*starts/(teeth*gashes)`. Facet count also depends on the evaluated
  flank roll range and phase. Doubling starts reduces generating-cut density instead of
  multiplying it. The acceptance now checks that relation and the actual source-labelled
  facets. [Gear Technology, May 2019, generative-cuts discussion/Fig.20](https://ik.imagekit.io/agmamedia/gt/issues/0519x/gt0519.pdf)
  gives the same dependence on gashes, starts and gear teeth.
- The independently derived small-angle tangent-envelope oracle is
  `delta ~= rho*DeltaC^2/8`, where the involute curvature radius at measuring radius r is
  `rho = sqrt(r*r-r_base*r_base)`. At pitch this becomes
  `pi^2*m*starts^2*sin(alpha)/(4*teeth*gashes^2)`.
  References: [Kotlyar, May/June 1994, Fig.7](https://ik.imagekit.io/agmamedia/gt/issues/0594x/gt0594.pdf),
  and [Li et al. (2023), generating-envelope definition/Fig.16](https://doi.org/10.1186/s10033-023-00930-5).
  Tests use complete interior facets; clipped end facets and radial-sample peak undersampling
  are not mistaken for an exact closed-form maximum.
- The spur-gear feed-mark oracle is `delta ~= f^2*sin(alpha)/(4*OD)`. It is Eq.1 of
  [Guba, Huesemann and Karpuschewski (2020)](https://doi.org/10.1016/j.cirpj.2020.06.005),
  [accepted manuscript, Eq.1](https://media-api.suub.uni-bremen.de/api/core/bitstreams/3b12aea7-2fd8-42ab-a6e8-dbacf6767cdb/content).
  This outside-diameter estimate is approximate: the simulation uses the actual finite hob
  flank radius. It is used only as a test oracle, never to modify the tooth surface.

Validation results (Release native core, pinned Haxeon host):

- **1,620 Haxe hobbing assertions passed**, about 187 seconds on this machine. Tests are
  registered in the main StockKit suite; individual `schedule`, `profile`, `reference` and
  `feed` cases can also be selected in the focused project.
- Coarse m1/z40 profile, 192 measuring circles, one transverse slice:
  - one start / 4 gashes: **8 facets**, 1.643 micrometre normal peak-to-valley variation;
  - one start / 8 gashes: **16 facets**, 0.433 micrometre variation;
  - two starts / 4 gashes: **4 facets**, 6.543 micrometre variation.
  Doubling gashes reduced the variation **3.80x**; doubling starts increased it **3.98x**.
  Mean measured complete-facet depth/oracle ratios were **0.979 / 0.915 / 1.067** respectively.
- Reference m0.3/z200, hob OD32/12 gashes, zero-feed full-depth pitch slice, three radial
  circles around r30: every one of the **200 teeth** is within **0.2 micrometre** of nominal
  thickness 0.471238898 mm. At this particular phase, pitch-thickness differences are at the
  floating-point floor; this is not a physical sub-nanometre accuracy claim. Two path refinements
  gave a maximum observed arc change **0.0471 micrometre** over the three circles.
- Axial-feed m1/z40 case, f2 mm/work rev: a five-Z-slice acceptance measured **11.30 micrometres**
  against **10.69 micrometres** from the outside-diameter oracle (ratio **1.057**). A preceding
  16-slice/longer-overrun investigation measured **11.51 micrometres** (ratio **1.077**). Finite
  hob geometry and sampled peak location are consistent with the small difference from the approximation.
- Controller check: **8** phase events, **7** between the 4 ms controller ticks, with exact
  phase-crossing times and independent C interpolation. Also tested varying master speed,
  invalid/reversing schedules, the frame chain, retained endpoint sources and job rollback.
- Native rake sweep: **329 checks passed**, including an independent translating-slab oracle,
  a concave L-region with an uncut notch, stationary faces, stale handles/non-rigid input and
  convergence of a rotated face to an analytic annular-sector cut.
- Existing native suites still pass: **24,224 ring checks**, **20,906 Cartesian checks**.
  CTest: **3/3 targets passed**. Existing Haxe ring suite: **345 assertions passed**.
- Canonical HXI audit passes Linux x86-64, Windows x86-64, macOS x86-64 and macOS arm64.
- HB0's full workspace baseline is still queued; no complete workspace pass is claimed.
  Existing dirty vendor/submodule state remains untouched.

Reproduce the focused checks:

```sh
cmake -S stockkit/core -B build/flexspline-hb3/native -DCMAKE_BUILD_TYPE=Release
cmake --build build/flexspline-hb3/native -j 2
ctest --test-dir build/flexspline-hb3/native --output-on-failure
./haxeon/scripts/haxeon build --compiler-only --project=stockkit/tests/hobbing/haxeon.json
LD_LIBRARY_PATH="$PWD/build/flexspline-hb3/native:$PWD/haxeon/.tools/hashlink:$PWD/haxeon/out${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  ./haxeon/.tools/hashlink/hl stockkit/tests/hobbing/build/host/main.hl
```


### HB4 metrology (2026-10-17)

- Added pure-Haxe `GearRings`, `GearInspection`, `GearMeasurement`, `GearTrace` and
  `Iso1328`. `RingStock` implements the read-only sampling interface. No StockKit or
  MachineKit dependency enters GearKit. The same report supplies application/test
  traces, all nine HB-D6 deviations, per-parameter candidate grades, circular tooth
  thickness, reconstructed virtual-ball runout radii and over/under-balls dimensions.
- Every tooth and both flanks are evaluated, preserving the specified datum axis.
  Pitch uses measurement-circle arc distance; profile/helix use the transverse base
  tangent. Residual interpolation uses roll distance/axial position. Sorted circle
  boundaries are searched by binary search, rejecting missing/ambiguous flanks.
- Explicit profile control is a designer input, since an active range needs a mating
  gear. The default geometric control is visibly assumed and cannot qualify a class.
  The inspection applies 95% roll profile evaluation, helix end exclusions,
  least-squares form/slope separation, tip/full-face slope extrapolation and end
  excess-material envelopes. Gaussian 50% cutoff filtering retains raw traces and
  discloses its linear endpoint convention. Sparse source data, unfiltered traces,
  missing face endpoints and incomplete coverage cannot earn `isoClass`.
- Pinned 2013 formulas (including unrounded total-deviation combination), class steps,
  rounding and Annex E virtual-ball construction against ISO-authored primary text.
  References, units, candidate-grade semantics and limits are in
  `gearkit/METROLOGY.md`. Tooth size has no ISO flank grade. Over/under-balls
  equivalents use reconstructed involutes rather than physical probe simulation.
  Helix trace slope signs are material-positive
  by flank; the aggregate grading uses magnitude, not a signed helix-angle claim.
- Corrected two plan assertions: ISO disallows assigning its classes outside scope;
  module 0.3 therefore gets experimental extrapolated grades and `isoClass=null`.
  Also, the 2013 standard supplies equations rather than a numeric m 0.5 fixture
  table. Tests use independently calculated decimal equation fixtures.
- Validation: 246 metrology assertions pass with both the project compiler/HashLink
  and pinned reference Haxe interpreter. Existing 532,368 geometry and 18 GearSpec
  assertions pass on the project host. Native RingStock metrology adapter: 11
  assertions, including numerical class 1 for every parameter of a perfect gear.
  Existing ring suite: 345 assertions pass. Integrated StockKit test project compiles
  (454 sources); the full CAD/workspace suite was not run here.
  Metrology covers external/internal ideal gears, actual 5 micrometre translated
  involutes (`Fr` about 10 micrometres and the pitch harmonic/amplitude), injected
  profile/helix slopes, Gaussian cutoff transmission, end excess/relief, invalid
  settings, sparse/incomplete data and out-of-scope module.
- **Still pending:** actual full-profile/full-face reference rake acceptance. Added
  `RingMetrologyTests reference`, which cuts 185 radial by 201 axial samples and
  checks every HB-D6 candidate grade <= 3. It uses actual rake geometry through
  `HobbingSweep`, not a synthetic error/surface substitution. The machine's heavy
  slots remain occupied, so this run is queued through `heavy-run`, label
  `flexspline-hb4-reference`, 8 GiB limit / 4 GiB availability requirement. Log:
  `/tmp/flexspline-hb4-reference.log`; frozen executable:
  `build/flexspline-hb4/reference-main.hl`. No full-face grade is claimed until it
  finishes. HB0 baseline is independently still queued behind the same slots.
- Validation logs: `/tmp/flexspline-hb4-gearkit-tests.log`,
  `/tmp/flexspline-hb4-reference-haxe.log`,
  `/tmp/flexspline-hb4-native-adapter-tests.log`,
  `/tmp/flexspline-hb4-ring-tests.log`, and
  `/tmp/flexspline-hb4-integrated-build.log`.


### HB5 machine parts (2026-10-17)

- Added `machinekit.motion.TorqueMotor`, `AngularContactBearing`,
  `CrossedRollerBearing`, `RotaryEncoderRing`, plus reusable `PrecisionPart` and
  `PrecisionErrors` support. Motors/encoders use the existing typed `MotorDrive`
  and `EncoderPart` interfaces. Motor curves, winding, inertia, pole count/cogging,
  bearing precision/stiffness/runout, encoder counts/accuracy/eccentricity and their
  provenance are recipe values. CAD is mm; torque/speed/inertia are SI; angular
  error is rad. Runout is TIR, eccentricity/cogging are amplitudes, not waveforms.
- Added hobbing leaves: `Hob`, `HobArbor`, `OutboardSupport`, `ExpandingMandrel`,
  `RotaryClamp`, `MineralCastBed`, `Column`, and table/spindle/swivel envelope parts.
  All are in the normal component registry with BOM identities and save/load recipes.
  Reference catalogues for the four motion parts explicitly say Assumed, Unverified,
  GenericApproximation with no verified-field claims. Catalog provenance requires a
  source and is independent for ratings and errors.
- `RotaryTable`, `HobSpindle`, `SwivelHead` are real `MachineAssembly` packages,
  preserving their individual members and BOMs. C/B are continuous, A is bounded,
  effort/speed limits originate in installed motor ratings and retain assumption
  labels. Encoders read their local joints. The spindle has two opposed instances
  of one bearing BOM item; bearing orientation is a mate, not another catalogue row.
  Static clamp holding torque is separate from drive effort. Drivers, supplies,
  clamp control and hydraulic services are HB6/HB7 integration, not claimed here.
- The spindle reference is 1.5 kW at 4000 rpm / 8000 rpm maximum, with an explicitly
  assumed winding. Default table feedback is 2^23 counts and 2 arcseconds angular
  bound; bearing radial/axial TIR is 0.002 mm; mandrel eccentricity amplitude 0.004 mm.
  Stiffnesses, envelopes and every supplied performance number remain Assumed.
  Housing bores have steps for bearing/motor seats and reject incompatible motors.
- `Hob.fromDesign` captures generating geometry and reconstructs GearKit's actual
  rack/rake regions; it rejects a mismatched generating target. Physical preview is
  a bored blank, not helical tooth/relief/sharpening geometry. HSK-E25 is a gauge
  interface identity only; no conforming taper/drawbar geometry is claimed. These
  limits and the primary interface-name references are in `PARTS.md` beside this plan.
- Error bounds live on physical component facets and round-trip through existing
  assembly recipe descriptions, without adding a second wire format. Stable sorted
  path-qualified discovery works after decode, including edited values and ValueBasis.
  ProjectKit's flattened geometry-only definition does not yet export runtime errors.
- Added assumed mineral-cast and carbide materials (2400 / 14500 kg/m^3), keeping the
  bed/column/tool BOM and mass models from silently using steel density. MachineKit
  now depends on the pure-Haxe GearKit for the tool recipe.
- Validation: `HobberPartTests` passes 931 assertions; the native-CAD `geometry` mode
  passes 61 assertions. Checks cover every leaf's recipe, registry, BOM and error
  round-trip, three package descriptions, modified error provenance, torque and speed
  limits, motor replacement, encoder quantum, spindle power/winding loss coherence,
  opposed-bearing BOM quantity, recreated rake points, invalid inputs/fit, positive
  single-solid geometry, mineral-cast density and housing-seat interference.
  The group is registered in MachineKit smoke. The integrated smoke project compiles
  (2,135 sources); the full smoke/workspace run was not repeated.
- Standalone manifest: `machinekit/tests/hobbing/haxeon.json`; no argument runs part
  checks, `geometry` runs native CAD checks. Logs: `/tmp/flexspline-hb5-build.log`,
  `/tmp/flexspline-hb5-parts.log`, `/tmp/flexspline-hb5-geometry.log`,
  `/tmp/flexspline-hb5-smoke-build.log`. Used the existing CadKit shared library at
  `/home/joao/dev/materia-cache/claude-scratch/am-quick/klipper/build/host/native/cadkit`
  with `/home/joao/dev/materia-cache/occt-f1dc4efb-release/lib`; no native source build
  or broad heavy test was needed for these checks. HB0 holds a queue slot waiting for
  memory; HB4 full-face acceptance remains queued, with no full-face grade claimed.


### Plan amendment: mineral-cast foundations (2026-10-17)

- User amendment: mineral-cast precision foundations use cast-in steel inserts with
  stated position tolerances and ground-after-casting finish. Casting bodies are not
  machined precision mounts. Added ribbed bed/column, separate insert BOMs, material
  properties and mount/face checks to HB5; generic process modelling and external FEA
  remain Phase D, with the ≈ 600 Hz gash-frequency modal question explicitly open.
- Current HB5 code already supplies density-only mineral cast, a solid bed and a
  solid `Column`. It does not yet satisfy the newly added insert/material requirements;
  HB5 status is reopened for that amendment. HB6 must use the amended foundations.
- HB4 full-face acceptance is unchanged and has not been interrupted.


### HB5 foundation amendment implemented (2026-10-17)

- Added optional physical Young modulus, loss factor, expansion, conductivity and
  assumed flag to `MaterialPhysical`; mineral cast explicitly supplies 35 GPa,
  eta 0.02 (8x an assumed iron eta 0.0025), 14e-6/K and 2 W/m/K. These inputs do
  not activate a simulation automatically. Density remains 2400 kg/m^3.
- Replaced solid foundation leaves with `MineralCastBed`/`MineralCastColumn`
  assemblies, ribbed `MineralCasting` bodies and separate `CastInsert` children.
  Removed the obsolete `Column` recipe instead of retaining a compatibility alias.
  Bodies have 20 mm end walls, 25/30 mm outer walls and a 25 mm cross rib, with
  designed cavities and insert recesses. Casting process/core feasibility is Phase D.
- Rail seats, table/column plates, threaded bushes and lifting inserts are steel
  children, with 0.01 mm stated position tolerances and ground-after-casting finish.
  Precision connectors expose steel faces. Body BOM lines include native-computed
  kg purchasing weight; `castingByWeight()` exposes the numerical weight directly.
- `FoundationIntegrity` checks insert position and nominal face parallelism against
  tolerance, plus named precision mounts on steel inserts. Negative fixtures cover
  mounting on the casting and an insert 0.03 mm outside its 0.01 mm tolerance.
- Updated HB5 tests: 951 assertions pass, including volume-density mass, volumetric
  inertia, separate BOMs, description persistence and the new assumed material inputs.
  Log `/tmp/flexspline-foundation-tests.log`; compile log
  `/tmp/flexspline-foundation-build.log`. HB4 has not been touched.


### HB6 six-axis machine (2026-10-17)

- Added `FlexsplineHobber extends MachineAssembly`, its project manifest/preview and
  a Gear cutting Start-page card. Mechanical axes use package-preserving paths
  `table/C`, `spindle/B`, `swivel/A`, and top-level `X`, `Y`, `Z`; C is continuous
  for multi-revolution hobbing. The B process-velocity drive is independent of C.
  No electronic gearbox has been substituted with a mechanical coupling.
- Three ball-screw/nut/servo drives use existing transmission/support resolution,
  couplings with two-sided engagement and separate rotating motor shafts, fixed/
  floating supports, and 0.1 micrometre linear scales. Six drivers and a rectifier
  use typed electrical ratings and existing power checks. A's separate 100 N m
  spring-applied clamp has release/feedback metadata; EGB/clamp sequencing is HB7.
- Mounted the column, table and Z rails on the revised foundation steel inserts.
  Added a paired-face coplanarity check, rejecting opposite 0.009 mm offsets that
  individually pass 0.01 mm but together exceed the pair tolerance. Foundation
  body recesses move with the table plate's forward placement at Y=-160 mm.
- Fixture dimensions and soft travel derive from the selected reference gear's
  cup/root band and tool size. Raised work shoulder and outboard bearing placement
  clear the table during end feed/tilt. Installed A is limited to +/-15 degrees;
  the generic swivel's default remains +/-45. X upper=100 mm, Y=+/-10 mm,
  Z=-10..60 mm. These travel/layout choices are assumed design values, not catalogue
  specifications. Full structural/mount/contact validation remains Phase D.
- `hobToWork(state)` returns the solved mechanical chain, with CAD/work axes rather
  than the connector-axis convention. Native checks compare all translation and
  rotation components against the independent HB3 transform at X/Y/Z/A travel
  corners with nonzero B/C. Conservative native bounding planes prove hob clearance
  from mandrel and platen there. Small/demo and comparison fixtures reach full depth.
- Fixed a real scene-export failure: screw follower defaults must equal the leader's
  initial coordinate times the resolved lead ratio. Setting X at cutting depth with
  screw angle zero failed the assembly state's coupling invariant. Startup coordinates
  now agree, and scene export/decoding succeeds with all six drives and 47 definitions.
- Validation: 243 mechanical assertions pass, plus scene generation/decoding.
  Updated part/foundation regression checks pass 955 assertions. The integrated
  MachineKit smoke project is compiler-checked; no full workspace pass is claimed.
  Logs: `/tmp/flexspline-hb6-checks.log`, `/tmp/flexspline-hb6-scene.log`,
  `/tmp/flexspline-hb6-parts.log`, `/tmp/flexspline-hb6-smoke-build.log`.
  Reused a current native CadKit library from the machine-tending worktree and the
  existing HB3 StockKit library; no native source modification was needed.
- Added StartPageTests coverage and `ProjectSourceTests.checkFlexsplineHobber`, which
  loads the actual project, checks derived rates/acceleration and process ownership,
  and performs a 200-tick zero-command MuJoCo hold. **Acceptance is pending** the
  shared heavy-run queue, not reported as passing. The HB6 app runner executes
  Start-page and the focused project-source case from an archived source commit,
  label `flexspline-hb6-app`, log `/tmp/flexspline-hb6-app.log`, with 16 GiB cap /
  8 GiB availability requirement and one native compiler at a time. Runner/source
  live under `build/flexspline-hb6/`. HB4's existing run was not interrupted; HB0
  still holds a slot waiting for memory and HB4 still waits for a slot.


### HB7 — electronic gearbox

- RobotKit's host-owned runtime now installs a programmable follow relation with
  `target = planned + Σ ratio × measured_encoder_prediction`. Masters use count
  quantisation and measured velocity, with sample age plus explicitly declared
  additional encoder latency. Driver/controller loop data is required; a runtime
  slower than the declared gear loop rejects the plan. No mechanical coupling,
  model rebuild or RobotModel wire-version change is involved.
- ToolpathKit `ElectronicGear` / `ReleaseGear` lower named joints to atomic timed
  process-event groups. `GearBinding.install` reads the installed encoders and
  driver position loops, takes controller timing and extra latency explicitly,
  and returns safe-release channel declarations for the requested independent
  followers. MotionKit continues to plan the additive coordinate. Unsupported
  CNC/printer exporters refuse gearbox operations explicitly.
- PlanCheck reads the same event groups, including both sides of changes between
  ordinary sampling instants. It includes geared velocity and acceleration in C
  speed, motor torque, RMS torque and elastic-load checks. Process masters need a
  stated acceleration in `PlanCheckOptions.processAccelerations` (zero only for
  settled speed). Excess speed names `table/C`; missing acceleration is an error.
- Per-owner-tick `sync` process records expose measured follower residual in SI
  units through the existing bounded event buffer and overflow flag. Feed hold
  stops additive feed while C follows a continuing spindle. Stop/abort/underflow
  release gearing and stop from the measured physical pose/velocity. Brake
  interlocks gate outgoing motion. Endpoint-owned device queues remain explicitly
  unsupported; a device-side EGB needs its own implementation.
- Validation: all four targeted native suites pass (electronic gear, runtime,
  blueprint validation and brakes). The native owner test covers measured rather
  than planned B, an additive C phase, per-tick records, feed hold, physical stop,
  multiple masters, cycles, incomplete packets and an underspeed owner rejection.
  With 2^23 encoder counts/rev, 4 kHz driver/owner, B at 3000 rpm, ratio 1/200 and
  1 ms extra encoder delay, maximum prediction error is 0.000309 arcsec. An assumed
  .02 kg m², 200 Hz critically damped servo stays below 1 arcsec in the steady
  15 rpm interval (0.000020 arcsec). Disabling prediction gives the expected
  47.124 µm at 30 mm radius. These assumed reference checks omit HB10/HB11 loads,
  cogging/eccentricity and structural effects; they are not measured machine accuracy.
- The standalone `machinekit/tests/gearing/haxeon.json` has 16 passing checks for
  lowering, ratio changes without rebuilding, release, combined speed, torque,
  short event intervals, trajectory masters and missing process acceleration.
  It is registered as `hobber-gearing` in the workspace. ToolpathKit's 149 core
  assertions pass; the broader ToolpathMotion project compiles. Protocol, timing
  semantics and limitations: `robotkit/runtime/ELECTRONIC_GEAR.md`.
- Logs: `/tmp/flexspline-hb7-{ctest,native,gearing,gearing-build,toolpath-core,build}.log`.
  HB4 and HB6 heavy acceptance runs remain queued and were not interrupted.

### HB8 — hobbing CAM

- `camkit.HobbingJob` produces ToolpathKit operations from GearSpec, HobDesign,
  usable hob length, settings and optional installed limits. It derives hob rpm
  from cutting speed/OD and work rpm from starts/teeth, uses axial feed per work
  revolution, validates increasing radial passes through final tooth depth,
  supports climb/conventional feed, static shifts per pass and parabolic X(Z)
  crowning. GearKit inputs are mm/radians; ToolpathKit output is SI.
- Approach/exit overrun and active hob-axis length come from the continuous
  tilted-cylinder intersection with the blank, not a fixed distance allowance.
  Concave scalar maximisation includes radial-domain endpoints and outward
  roundoff allowance. The finite hob must cover this active interval at every
  shift. The table's additive phase compensates -shift*cos(gamma)/pitch radius,
  keeping the HB3 tooth registration. Shift and phase changes happen retracted.
- New ToolpathKit `PositionJoint`, `WaitSignal` and modal `FeedPerRevolution`:
  absolute A/C positioning lowers through MotionKit's `JointPosition`; revolution
  feed is resolved from the programmed spindle rpm and active EGB ratio, including
  the axial projection of crown chords. The radial approach retains its explicit
  infeed speed. Unsupported CNC/printer exports reject these operations.
- `FlexsplineHobberCam.job` reads the HB6 tool length, travel, installed swivel
  range and spindle/table speed ratings. A startup measured-signal barrier requires
  a ready machine with referenced B=0 and released EGB. Gearing engages before B
  starts, cutting waits for spindle-at-speed, and the final radial withdrawal is
  followed by spindle-off and a bounded measured-stop barrier. The stopped B/C
  relation remains engaged to retain phase; releasing it requires rebasing additive
  C. Actual app signal production, restart/referencing and stock execution are HB9.
- The reference geometry gives **4.598932 mm engagement overrun per end**. Including
  1 mm extra clearance at each end, the axial stroke is 19.197864 mm or 38.395729
  work revolutions. At 300 m/min / OD 32, B is 2984.155 rpm and C is 14.920776 rpm.
  Nominal feed time is **154.398386 s**; the real MotionKit S-curve and the complete
  lowered program's cutting plan both give **154.408360 s**, within 0.007%.
  This corrects the earlier 96 s estimate in the reference table. It excludes
  positioning, spindle ramps and barrier waits; total measured cycle time is HB9.
- Crown chord error and MotionKit corner blending each receive half the supplied
  tolerance. X(Z) is a programmed head trajectory, not an assertion that the
  resulting tooth has that exact crown after finite cutter engagement. HB9/HB11
  inspect the physical result. Feed/revolution conversion is nominal at the settled
  programmed spindle speed; Z is not an additional encoder follower.
- Validation: **80,046 CAM assertions** pass, including independent cylinder-boundary
  samples, the 1% cycle criterion, complete lowered-program compilation from nonzero
  A/C coordinates, finite-length/shift rejection, rough/finish passes, multi-start
  and tooth-count changes, profile shift, crown chord bounds and machine-limit
  rejection. **244 mechanical/CAM adapter checks**, **149 ToolpathKit core checks**
  and **16 existing gearbox checks** pass. The broader ToolpathMotion project
  compiles; no full CAD/workspace/app run is claimed.
- The standalone manifest `camkit/tests/hobbing/haxeon.json` is registered as
  `hobbing-cam` in the workspace. Details: `camkit/HOBBING.md`. Logs:
  `/tmp/flexspline-hb8-{tests,build,machine,machine-build,core,gearing,motion-build}.log`.
  The cycle test reused the existing native MotionKit library read-only from the
  machine-tending worktree; native MotionKit sources were unchanged here.
  HB4 and HB6 acceptance remain queued and were not interrupted. CT0–CT10 remains
  a separate plan; HB8 makes no casting or insert-layout changes.


### HB9 implementation — coarse app smoke passed

- The default `cutting` entry point is a deliberately coarse m1, z40, 4 mm face
  job with four gashes. The original reference machine remains selectable as
  `hobber`. Typed scene metadata carries the job through the artifact round-trip.
- `HobbingProgramPlayer` lowers HB8 CAM against the installed machine and reads
  real B/C and XYZ/A simulation observations. Loaded joint-coordinate offsets are
  accounted for explicitly. Startup, at-speed and stopped barriers use measured
  signals; the native EGB now continues across empty queues and anchors subsequent
  plans in additive C coordinates. Native cache inputs include the runtime headers.
- `HobbingStock` streams measured poses through complete gash windows with exact
  crossing times. `RingPreviewWorker` contours immutable snapshots off-thread.
  The panel presents HB4 profile/lead traces, candidate grade and qualification
  warnings after the cut. A coarse demo result does not establish an ISO class.
- Focused validation passes: ideal streaming versus the HB3 sweep differs by
  **0 µm** (criterion 0.1 µm), with 169 windows, a peak of 15 buffered samples and
  64,024 retained stock bytes. Worker contouring, cutting artifact round-trip,
  three native runtime/gear/brake tests and the 2,664-source app compiler check pass.
- The MuJoCo `checkFlexsplineHobber` coarse end-to-end smoke now passes (see quick
  checks below). Its original archived waiter was superseded after execution
  exposed missing feedback wiring, rotary limits and physical servo configuration.
  Its collision check covers conservative cutter/mandrel/table fixture envelopes;
  a full-machine interference audit remains open. The memory criterion bounds
  retained ring stock and measured samples, not cumulative GC allocation.
- Logs: `/tmp/flexspline-hb9-{stream,artifact,native-tests,app-build,app}.log`.
  HB4/HB6 pending runs are left undisturbed. CT0–CT10 remains separate.


### HB10 implementation — full reference force and static ratings passed

- `stockkit.hobbing.HobbingForces` derives chip volume from native removed ring
  intervals after each gash; chip area is volume / peripheral cutter travel.
  Effective edge width is explicit (app: 1 mm assumed). `KienzleMaterial` marks
  40CrNiMo at 38–42 HRC kc1.1=2400 N/mm² and mc=0.25 as uncalibrated assumptions.
- Outputs are mean gash force, hob torque/power, assumed radial separating force
  and a conservative table-torque magnitude bound. Overlapping mean-load windows
  are summed with bounded retained storage. Instantaneous tooth peaks and signed
  force directions are unresolved; these limits carry into HB11.
- `HobbingStream.onPass` connects measured sweeps to the observer. The app reports
  loads and assumptions, including the previous-stock view in retained memory.
  `FlexsplineHobberLoads` reads installed continuous motor ratings with 20% reserve
  and supplies X/C joint efforts to PlanCheck; process B gets a separate scalar
  spindle check. These are load-informed recheck APIs, not applied physics loads.
- Focused tests check exact sector-volume removal, no double counting, air cuts,
  feed^(1-mc), overlapping loads and energy/torque units. The real mid-face virgin
  stock plunge is deliberately rejected: about **4.45 kN / 22.2 kW**, not a normal
  HB8 entry. This makes the trajectory distinction explicit rather than tuning
  coefficients to force a tens-of-newtons result.
- Full HB8 reference entry/feed validation passed with the threaded force runner
  (results below). HB9's archived source remains unchanged. Loaded app acceptance
  remains separate from these static process-load checks.
- Details and assumptions: `stockkit/HOBBING_FORCES.md`. Logs:
  `/tmp/flexspline-hb10-{tests,gearing,machine,stream,app-build,reference}.log`.

- Validation: **18 force checks**, **19 gearbox/PlanCheck checks** and **248 machine
  checks** pass. The measured stream still matches HB3 at **0 µm**, with callback
  chip volume equal to the independent total stock difference. The app compiler
  passes **2,667 sources**. No new native binaries were needed for these checks.


### HB11 implementation — quick physical sources passed; reference studies pending

- `HobbingErrorSource` perturbs actual work/tool frames and rake geometry before
  cutting. `FlexsplineHobberErrors` reads installed mandrel, table bearing/encoder,
  arbor, hob and outboard inputs. Runout fields explicitly mean TIR; eccentricity
  means offset amplitude. Harmonic shapes/phases are assumptions. The hob now has
  an assumed 1 µm peak profile-error field; its sample waveform is zero-rake only.
- `HobbingErrorJob` repeats fresh-blank rake sweeps and HB4 inspection per source
  and all sources. Motion knots and independent measured C are preserved.
  `gearkit.ErrorBudget` reports signed parameter deltas and dominant parts, and
  checks trace superposition (0.1 µm + 25% of summed absolute trace envelope per
  channel), rather than adding ISO grades or peak-to-peak scalar metrics.
- Sync accepts HB7 measured C-minus-baseline error. Runtime source-disable studies
  must actually regenerate the observations; diagnostic latency is not a substitute
  for a servo run. Constant speed plus constant latency produces a fixed phase
  offset, not cumulative pitch variation; changing speed produces varying error.
- `RingCompliance` derives n=2 stiffness from geometry and stated modulus, with
  explicit support stiffness in parallel. Clamping pressure alone cannot supply
  that stiffness. Local quasistatic recession uses a stated HB10 radial-load
  history; contact, higher modes, dynamics and force/compliance feedback are open.
  Unsupported-ring diagnostics do not represent the expanding mandrel installation.
- Focused model/metrology checks and the machine adapter pass. Native profile-warp
  construction is checked. The coarse physical source-cut mode is separate and
  pending; full reference-budget signatures, HB7 source studies and supported-wall
  calibration remain required before HB12. No physical ISO budget is claimed.
- Details: `stockkit/HOBBING_ERRORS.md`. Logs:
  `/tmp/flexspline-hb11-{tests,machine,stream,app-build,physical}.log`.
  HB9/HB10 acceptance artifacts are left unchanged.

- Validation: **24 focused error/metrology checks**, **251 mechanical/adapter checks**
  pass; the app compiles **2,672 sources**. The stock-stream regression still
  matches HB3 at **0 µm** (169 windows, 15 peak buffered samples). The separate
  physical source-cut acceptance now passes in its smaller quick fixture.


### Quick checks — run directly with cached native dependencies

```
machinekit/examples/flexspline-hobber/check-quick.sh
```

- Optional selector: `force`, `errors` or `app`. Only these three headless cases
  run. The script compiles Haxe only, rebuilds RobotKit's matching runtime in an
  isolated build directory, and reuses CAD/MuJoCo/other native libraries read-only.
  It uses one native compiler and memory-capped scopes, with no heavy-suite queue.
  Logs, times and native provenance are in `build/flexspline-quick/`.
- On this workspace native providers are discovered from the app build or the
  sibling machine-tending cache. Override `HOBBER_NATIVE_CACHE` (the directory
  containing `app`, `cadkit`, etc.) and `HOBBER_STOCK_NATIVE` if needed. A missing
  cache is reported; the runner does not silently launch a full dependency build.
- Force smoke: complete m0.3/z40/1 mm cut, 391 windows, **72.13 N**, **360.66 W**,
  **1.154 Nm** hob torque; 20% spindle reserve and volume conservation pass in
  about **5 s**. This is a small-part check, not the m0.3/z200 reference acceptance.
- Error smoke: m1/z20/2 mm baseline, three isolated sources and all combined;
  **Fr 7.896 µm**, **Fp 8.505 µm**, **ffalpha 3.889 µm**. Four physical signature/
  superposition checks pass in about **51–55 s**. Sampling is diagnostic, not ISO
  qualified; the lead grid covers the evaluation interval.
- App smoke: the m1/z40/4 mm demo uses a **0.1 mm radial grid, 1 µm sweep tolerance**,
  and 5 mm/s below-face air infeed. Pitch-to-tip inspection is explicitly partial.
  It completes **924 windows in 5.5535 simulated seconds**, with about **946 KiB**
  retained stock, no fixture-envelope contacts and a finite gear report (ISO class
  unqualified). Observed wall time is about **32–34 s** with generated geometry cached.
- App measured cutting sync is **0.003962 arcsec** after correcting the simulation
  command epoch (previously **402.865 arcsec**, one 4 kHz tick ahead). The session
  labels steps by their end time; commands now use the start time and feedback
  retains the end time. Explicit zero is a valid owner epoch; only omitted time
  requests next-sample prediction. Regression acceptance is now two table encoder
  quanta (about **0.309 arcsec**), with no owner-tick allowance. The coarse cut and
  three native runtime/gear/brake checks pass. This resolves the clock lead, but
  does not establish the full HB11 reference error budget or HB12 precision grade.
- Execution fixes: explicitly wire B/C encoders to their outer drives; state
  assumed A/C positioning accelerations; compile physical hold gains for braked
  servos; enable the hobbing table's assumed drive-default physical servo gains so
  velocity feedforward is used. Keep computed-torque gear holds in position mode
  instead of emitting a zero-gain servo. Exclude the virtual stock's intact convex
  collider while retaining its mass/inertia; RingStock owns removal and fixture
  checks remain active. No native cutting-force/contact-load feedback is claimed.
- HB4 is undisturbed. Full reference force validation, HB7 source-disable studies,
  supported-wall calibration and reference-grade acceptance remain separate.

- Final runner verification: **force, errors and app all pass** in one invocation
  (4.48 s, 54.12 s and 33.78 s execution respectively, plus compilation).
  Peak RSS is about 113 MiB, 126 MiB and 470 MiB. **256 machine regressions** and
  **three native runtime/gear/brake regressions** pass, including a zero-gain
  computed-torque follower draining its trajectory queue. Console summary:
  `/tmp/flexspline-quick-all.log`; per-case evidence: `build/flexspline-quick/logs/`.

### Reference-force follow-up

- `check-reference-force.sh` runs the full ideal reference stroke and checks its
  report against installed B continuous power/torque, C continuous torque and X
  continuous screw thrust, each with 20% reserve. Reports are retained on failure;
  progress is emitted every 100 windows. X conversion uses installed screw lead
  and nut efficiency. This checks static process loads; loaded dynamics, gravity
  and instantaneous tooth peaks remain separate.
- Current focused recheck: 18 force, 264 machine/adapter and 24 HB11 model assertions
  pass. The full reference cut is running directly under a 4 GiB cap, replacing
  the old queued waiter. Acceptance remains open until its report and installed
  rating checks finish. Evidence: `/tmp/flexspline-hb10-reference-mean.log` and
  `build/flexspline-reference-force/`. HB4 is undisturbed.
- HB11 now estimates support from assumed sleeve/cavity/contact design, rather
  than blocking on calibration. `MandrelRadialSupport` derives an n=2 sleeve mode
  and conforming-steel contact in series; uniform oil compression is reported
  separately, with no first-order oil restoring term for volume-preserving n=2.
  The reference estimate is **84.6 kN/mm**, with 1.5 mm sleeve wall and 16 mm end
  span over the 8 mm band. These dimensions describe an assumed design, not
  resolved sleeve CAD or measured hardware. All inputs/derivation remain in the report.
- HB3's 0.2 um value is a numerical simulation tolerance, **not** a physical
  deflection budget. Split mean deflection from load variation. Judge the mean
  thickness shift against an **assumed +/-5 um flexspline thickness tolerance**;
  report the mean displacement as compensable X infeed. Allocate 20% of class-6
  Falpha and single-pitch fp to the varying wall contribution inside HB11. These
  ISO formulas are engineering extrapolations at m0.3, not an ISO qualification.
- The reference sleeve region is explicitly 71..87 mm in mandrel coordinates.
  Assembly transforms place the toothed band at 75..83 mm, radial bore clearance
  0.025 mm. The model checks axial coverage, expansion travel and provisional
  seating pressure. Without rim support/seated contact it sets mandrel stiffness
  to zero and uses the cup wall ring stiffness. Sleeve/cavity dimensions are an
  assumed design; the current blank CAD is an annular rim, not a detailed cup.
- Keep the x0.3/x1/x3 stiffness sweep and the sufficient stiffness requirements.
  The standalone provisional assumed load case bounds mean and varying range separately; it
  does not invent a measured mean. The full-stroke force report records energy,
  a time-mean radial force (including air entry/exit), and a conservative varying
  range bound. Recompute the report from measured force when the full cut ends.
  Full combined-source HB11 acceptance remains open.

- Provisional assumed-load screen (not a safe bound on the running cut): mean thickness shift <=1.04/0.31/0.10 um
  for support x0.3/x1/x3, all inside the assumed +/-5 um tolerance. Varying profile
  bounds are <=2.86/0.86/0.29 um; x0.3 does not establish the allocated 1.9 um
  profile budget. A sufficient effective support is **38.3 kN/mm**, or nominal
  **127.6 kN/mm** across x0.3 uncertainty. Recompute with the measured full-cut
  force statistics. This conservative screen leaves wall acceptance open.
- Rechecks: **264 machine/adapter**, **18 force**, **24 HB11 model** assertions pass.
  The four small physical wall-source cuts (baseline and three stiffnesses) pass
  thickness-response checks, with changes **1.043/0.313/0.104 um**. This is an m1
  integration fixture, not the m0.3 reference inspection. Evidence:
  `/tmp/flexspline-hb11-{support-split,machine-split,wall-cuts}.log`.


### Force throughput and radial statistics follow-up

- The original fine force run was intentionally stopped by the user at 40,400
  windows; no completed report was produced. HB4 remains untouched. Tangential Fc
  and radial Fr are different components: Fr=Fc*tan(20 degrees) in this assumed
  force model. The previous 36.4 N radial case assumed Fc=100 N; it is **not** a
  measured or safe bound on the current cut. Recompute from completed reports.
- Native rake geometry now parallelizes complete pose-segment chunks (transforms,
  broad phase and circle clipping), merging arcs before deterministic union.
  Endpoints/provenance are bit-identical at 1/4/8 threads; short force cuts also
  assert identical stock and every gash load at 1/4 threads. Running libraries are
  untouched; optimized builds live in a separate directory.
- Force-only tolerance gate: two short reference-geometry fixtures compare fine
  0.00005 mm sweeps against coarser candidates. Choose only when peak/mean force
  and radial variation signatures each differ by <1%. The initial selected force
  tolerance is **0.002 mm**, 4 threads; max observed peak/mean changes are
  **0.100% / 0.050%**, per-pass RMS variation **0.461%**. The user authorized
  the completed optimized full stroke as HB10 authority after stopping the fine
  run; no fine full-stroke comparison is claimed.
- Record tolerance and thread count in every new force/source/metrology report.
  `HobbingErrorJob` defaults to 4 threads, the reference adapter to 8; all metrology
  sweep tolerances remain fine. Cached pose arithmetic preserves the old stock
  and per-pass load hashes. Full-reference HB11 per-source timing is still to be
  measured; do not extrapolate acceptance from a coarse source fixture.
- New full force runs retain per-pass radial/tangential history and time-weighted
  radial mean, peak, range, standard deviation and successive-pass max/RMS steps.
  Mean thickness/X compensation uses mean Fr, seating uses peak Fr, varying
  profile/pitch use the radial range, per-pass recession uses the saved Fr history.
  The stopped original run has only partial peak observations, no completed
  mean or per-pass history; it is not an acceptance result. Global peak/mean
  agreement alone is not pointwise history convergence.
- Offline band study uses entry, two mid-face table turns and exit from a complete
  history. It must meet the stated 1% criterion before a reduced statistics model
  is used; skipping physical preparation or changing error sources invalidates
  a reused stock checkpoint. No grading shortcut is enabled.
- Commands: `check-reference-force.sh [benchmark|run]`; comparison/band/completion
  helpers alongside it. Evidence: `build/flexspline-reference-force-fast/` and
  `/tmp/flexspline-force-{fast-full,final-validation}.log`. Target full-force wall
  time is 5–10 minutes; report measured completion time rather than claiming it
  from a short benchmark. Full HB11 source budget remains separate.

- Optimized full-stroke measurement: **92,151 windows in 360.50 s wall (6:00.50)**,
  4 threads, 0.002 mm force sweep tolerance, 0.000125 mm path tolerance, about
  **182 MiB** peak RSS. Tangential peak **46.679 N**, radial peak **16.990 N**,
  whole-stroke mean radial **3.0357 N**, material-engaged mean radial **4.9563 N**.
  Successive per-pass radial max/RMS steps are **5.8012 / 0.7285 N**. All installed
  continuous-rating checks retain the declared 20% reserve. These remain assumed
  Kienzle/window-mean process loads, not measured machine tooth-force peaks.
- X compensation/thickness use the material-engaged mean radial force; the
  whole-stroke radial mean is retained separately for energy/fine-run comparison.
  At stiffness x0.3/x1/x3, mean recession is **0.1951/0.0586/0.0195 um**, and the
  varying profile bounds are **1.3373/0.4016/0.1339 um**. All wall allocations fit
  this optimized force result; sufficient nominal support over x0.3 is **59.5
  kN/mm**, versus the assumed 84.6 kN/mm design estimate. Wall allocation acceptance **passes dependent on assumed stiffness/contact**
  and the short-cut force convergence basis; the combined HB11 budget stays open.
- The band approximation is rejected: entry + two mid-face turns + exit differs
  from full history by **20.375% in weighted mean radial force** and **3.723% in
  peak radial force**. Do not enable this shortcut.
- Regression evidence: native rake/ring/core suites pass; **27 force**, **24 error
  model**, **264 machine/adapter** and **11 ring-metrology** checks pass. Stream vs
  monolithic HB3 remains **0 um** (169 windows). The small five-cut source study
  at **0.000025 mm tolerance / 8 threads** passes in **51.92 s** (9.7–10.9 s per
  cut), while preserving its diagnostic stock grid and unqualified ISO status.
  It is not a timing claim for the complete reference error budget.
- The original force run and waiter were deliberately stopped, not failed. Its
  partial log is `build/flexspline-reference-force/force-partial-stopped.log`.
  HB4 continues unmodified. The completion helper gates the optimized full result
  on short-cut convergence, regenerates rating/wall reports and writes result.json.
  Fine full-stroke comparison is explicitly recorded as not performed.

### HB11 reference-source study runner

- `check-reference-budget.sh all` runs the complete HB8 reference feed stroke
  for the baseline, each of the **13 installed-part sources**, and all combined.
  It uses **8 threads, 0.000025 mm sweep tolerance, 0.0000015625 mm path tolerance,
  0.0015 mm radial / 0.04 mm axial spacing**. The force-only coarse tolerance is
  never used for these inspections. Every case starts from a fresh annular blank.
- Evidence is now in `build/flexspline-reference-budget-fast/`: frozen binaries and input
  hashes, settings, progress every 100 windows, per-source inspection JSON, final
  `budget.txt` and `result.json`, exit status and resource timings. Select `pilot`,
  `baseline`, an individual source id, or `combined` for a separate study.
  The runner builds its isolated native provider unless one is explicitly supplied. `all`
  retains each completed inspection before advancing; it does not silently reuse
  old source results. The entry pilot is a timing check, not tooth acceptance or
  a full-run timing prediction. **Full reference-source results remain pending.**
- The source replay uses the completed, accepted HB10 baseline radial history.
  Since its window means have discontinuous jumps, an explicitly assumed centred
  linear ramp regularizes each jump over at most 10% of either neighbouring
  interval. The reconstructed time mean and peak must agree within **0.1%**;
  currently both agree to numerical roundoff. Loads are frozen baseline values:
  changed stock, force/compliance feedback, contact dynamics and modal response
  are not solved. The previously reported stiffness x0.3/x1/x3 wall-allocation
  screen remains separate from the nominal-stiffness combined physical cut.
- HB7 sync observations are regenerated using the native `ElectronicGear` code,
  the installed 20-bit B encoder, 4 kHz sampling, assumed 1 ms transport delay and
  assumed 200 Hz critically damped unloaded servo, warmed for one second. The
  exporter independently runs prediction enabled/disabled: peak residuals are
  **0.00220 / 322.290 arcsec** at the reference speed. These are simulated plant
  observations, not loaded full-machine or hardware measurements. Constant-speed
  disabled prediction gives a phase lag; it does not establish varying pitch error.
- The SDK lowers `Array.sort` to insertion sort. The roughly one million merged
  motion knots therefore use merge sort once, then binary-search the local knots
  for each gash. This preserves exact sampling while avoiding quadratic setup and
  repeated full-trace scans. Regression against unsorted/duplicate/outside knots
  preserves every measured profile/lead trace and parameter exactly: **10 parity,
  24 error-model and 264 machine assertions pass**.
- The first batch attempt was stopped during timestamp sorting, before cutting;
  the corrected batch is running. HB4 remains untouched. Its metrology result,
  completed source signatures/superposition, coverage warnings, and the assumed
  servo/load/contact qualifications must be reviewed before HB11/HB12 acceptance.
  At module 0.3 the numerical grades remain extrapolated, never ISO certification.


### HB11 engaged-cut profiling and exact inspection sampling

- Profiling the running dense baseline showed native rake clipping, trigonometry
  and temporary-vector allocation dominate. Each worker now reuses polygon and
  interval scratch buffers, and caches radius-independent edge values lazily.
  Clipping order, arithmetic, tolerances and provenance are unchanged. The fine
  engaged fixture takes **22.61 s vs 58.36 s** for 100 windows; whole-stock
  endpoint/provenance hashes match. Two short reference fixtures also preserve
  every load and stock hash, with exact scalar equality across 1/4 threads.
- `InspectionRingStock` computes precisely the circles the current inspection
  reads: all profile radii at profile/pitch Z, and all face samples bracketing the
  measurement radius. Circle subtraction is independent under the explicitly
  frozen-load model. Both strips use the same sampled poses and complete stroke.
  Every retained radius/Z coordinate must equal the dense grid exactly; compact
  exact domains are sought by adjusting domain bounds within a few ULPs, never
  by moving a sample. If exact coordinates cannot be retained, use dense stock.
  Uncomputed-circle reads fail; overlap endpoints/provenance are checked.
- This representation supplies inspection traces, not a whole-part volume,
  force history, mesh or force/compliance feedback model. Original full radial
  and axial axes, filter coverage and every source are retained. The reference
  computes **2,102 circles including duplicate overlap vs 170,200 dense circles**.
  No target tooth boundaries or reduced force/metrology tolerances are used.
- On the same engaged reference fixture, all retained circles match the dense
  endpoints/provenance exactly: **5.17 s vs 22.30 s** with the optimized clipper,
  about **11.3x** versus the original 58.36 s. A complete small baseline and
  combined eccentricity/radial/axial/profile-warp inspection also preserves every
  parameter, profile/lead/thickness/runout trace, grade and coverage warning:
  **26 parity assertions** pass. These are measured fixture timings, not a claim
  about full-stroke or 15-cut batch duration.
- Default HB11 runner enables exact inspection sampling. Set
  `HOBBER_BUDGET_DENSE=1` to retain dense computation. Benchmark in a separate
  evidence directory with `check-reference-budget.sh engaged-cross`; it physically
  cuts dense and inspected domains, and fails on any retained endpoint mismatch.
  Each source log records actual sampling mode and computed-circle count.
- The original dense batch was intentionally stopped during its baseline and
  preserved in `build/flexspline-reference-budget/`, with `replacement.json`.
  The optimized batch is running in `build/flexspline-reference-budget-fast/`.
  Profiling/benchmark evidence: `build/flexspline-reference-budget/engaged-profile.txt`
  and `build/flexspline-clipping-speed/{old,new,cross}/engaged.json`, plus
  `validation.json`. HB4 remains on its frozen original binary/library.

### HB11 saved-result review

- `review-reference-budget.py build/flexspline-reference-budget-fast --watch`
  updates `review.json` and `review.md` as completed inspections appear, and exits
  when the batch exit status appears. It starts no cuts and changes no running
  inputs or libraries. Without `--watch`, it produces a single current review.
- The numerical summary requires all 15 inspections, the final budget, successful
  process exit, class-6 parameter limits and trace superposition. Partial writes,
  missing cases, failed processes and nonlinear interactions cannot earn a pass.
  Source rankings are provisional until all cases finish; the completed report
  uses ErrorBudget's authoritative dominant-source ids.
- Reports retain coverage warnings, sampling counts, source values/grades and
  mean thickness changes relative to baseline. Assumed support/contact and servo
  inputs qualify the result; they do not automatically fail the supported-wall
  acceptance already granted on an assumed-stiffness basis. Module 0.3 remains
  an engineering extrapolation. HB12 comparisons/cycle reporting remain separate.
- Review gates were checked with incomplete and partially written artifacts,
  successful/failed process exits, class-limit failure, superposition failure and
  qualified numerical success. The watcher is active; no completed reference
  inspection was available when it was installed, so actual acceptance is pending.

### HB11 true face-end coverage

- The running batch's cell centres stop 0.02 mm inside each face. Its warning is
  valid: unseen end excess cannot be inferred from those circles. Existing HB11
  artifacts and the running HB4 job remain unchanged.
- `FaceEndpointStock` adds independent physical rake samples exactly at Z=0 and
  Z=faceWidth, retaining every interior sample. Bounds are sampling domains only,
  not added workpiece volume. `HobbingErrorJob` has explicit endpoint sampling and
  an optional inspection domain inside the physical blank. Future reference
  baseline/source/combined runs enable the endpoints, so lead filtering includes
  actual end material rather than extending an unseen tooth surface.
- A separate `check-reference-budget.sh lead-ends` run is active in
  `build/flexspline-face-end-check/`. It repeats only the baseline and combined
  lead inspection over the complete HB8 stroke: **404 circles**, 8 threads,
  **0.000025 mm sweep / 0.0000015625 mm path tolerance**. It retains the original
  two measurement-radius samples and all 200 interior axial positions, plus both
  exact face ends. No full profile study or per-source batch is repeated.
- The supplement intentionally has no profile coverage. It grades Fbeta, ffbeta
  and fHbeta with actual ends, while fp/Fp/Fr must match the corresponding main
  inspection exactly before its result is accepted as compatible. The review
  watcher includes the separate artifacts and keeps numerical acceptance pending
  while required coverage is missing. Original source warnings remain visible;
  per-source trace-superposition at the newly sampled ends is not recomputed by
  this two-case supplement. New full endpoint-enabled batches include that check.
- Physical regression (m1/z20/face2, diagnostic stock spacing) offsets the cutter
  radially by 20 um only at one end. Interior-only lead inspection misses that
  defect; direct end sampling detects increased filtered lead error, while
  interior pitch/profile remain identical. A narrow lead/end domain also matches
  the corresponding full-domain measurements exactly: **16 assertions pass**.
  **26 existing inspection-parity, 24 error-model and 264 machine assertions**
  also pass. These small fixtures do not claim reference or ISO acceptance.

### HB4/HB11 bounded parallel rerun (2026-10-18 morning)

- Disk housekeeping by the user removed `build/flexspline-hb0`, `flexspline-hb6`
  and `flexspline-hb9`. Historical implementation checks remain recorded above,
  but fresh HB0/HB6/HB9 acceptance needs rebuilt artifacts. Recreate the pristine
  HB0 archive from `8c70c2190` if required. New runs share the 50 MB input file by
  hard link and reuse the validated native library; do not duplicate app builds.
- At the user's request the obsolete single-threaded HB4 PID 3586429 was stopped
  deliberately, without a completed result. Its small binary/metadata archive is
  retained. `build/flexspline-hb4-accelerated/` now runs the complete HB4 reference
  cut with **8 threads**, unchanged **0.00005 mm sweep convergence tolerance**,
  and the original fine grid. `HobbingSweep.cutMany` shares poses across exact
  inspection domains and checks the worst arc change on every computed circle
  at each refinement. Physical Z=0/8 samples are included. Grade <=3 acceptance
  remains pending the completed `result.json` and zero exit status.
- The completed fine HB11 baseline and mandrel inspections stay unchanged in
  `build/flexspline-reference-budget-fast/`. Its current table-radial case is
  allowed to finish. The parallel scheduler then checks the old process identity
  and stops that serial batch before further full cases; this is a deliberate
  replacement, not a failed cut.
- `build/flexspline-reference-budget-parallel/` is the replacement, restartable
  batch: **four worker slots, four native threads per cut**, a **6 GiB memory cap
  and no additional swap**. The existing table case occupies one slot until it
  completes; three other cases start immediately. Available memory is checked
  against estimated worker memory plus 3 GiB headroom. Reused cases retain their
  original thread provenance; each new cut logs its settings and execution time.
- Accepted settings are unchanged: **0.0015 mm radial / 0.04 mm axial grid,
  0.000025 mm sweep / 0.0000015625 mm path tolerance, all 92,151 windows**, and the
  same force/sync knots and histories. No coarser once-per-revolution grid has
  been accepted, and no coarse/fine agreement is claimed. Overall hours-scale
  throughput is a target pending measured engaged-case timings; short fixture
  timings must not be presented as a completed full-batch speedup.
- Every new source/combined inspection physically samples both face ends.
  `FaceEndpointStock` uses only the original measurement-radius strip for end
  planes when attached to an inspection cross; uncomputed end radii throw.
  The full reference computes **2,106 circles**, retaining every original
  inspected coordinate and adding four actual end circles.
- Completed fine profiles can be reused with a separate full-stroke lead/end cut.
  Promotion requires **exact equality** of interior fp/Fp/Fr, cumulative pitch,
  radial measurements and tooth thickness; mismatches fail rather than silently
  merge. Baseline lead comes from the already-running face-end supplement;
  mandrel and table-radial get separate narrow lead/end cuts. Original artifacts
  and their hashes are retained. The new end-inclusive traces are used for the
  complete per-source superposition check, not just baseline/combined grading.
- The Python assembler applies the same pointwise **0.1 um + 25% summed trace
  envelope** criterion and source-order scalar ranking as `gearkit.ErrorBudget`.
  It rejects inconsistent traces or missing end coverage. All case files and a
  successful assembly are needed before the review can pass. Fingerprints bind
  resumes to the frozen executable, native library, inputs and settings; a
  changed executable is rejected without changing completed acceptance status.
- Rechecks: **25 physical end-coverage/parity assertions** and **2 converged
  multi-domain parity assertions** pass. A 100-window engaged reference fixture
  gives the identical stock/provenance hash at **1, 2 and 4 threads**; every
  inspected endpoint agrees with dense stock. Cross timings were 17.05/11.99/
  12.36 s under changing concurrent load, not a controlled full-cut benchmark.
  Scheduler checks reuse all 15 saved cases across two restarts without invoking
  a cutter and reject changed binaries, interior mismatches, missing end coverage,
  inconsistent sampling; nonlinear trace superposition is flagged. HB11/HB12 acceptance
  remains open while the physical runs finish.

Resume the prepared batch (completed cases are reused):

```sh
machinekit/examples/flexspline-hobber/check-reference-budget.sh parallel
python3 machinekit/examples/flexspline-hobber/review-reference-budget.py \
  build/flexspline-reference-budget-parallel --watch
```

While the old table case is still running, set `HOBBER_BUDGET_LEGACY_PID` to its
verified PID when starting the scheduler so it can stop the superseded serial
batch after saving that case. Do not start a second batch beside the live one.

### HB11 native paired-domain speedup (2026-10-18)

- A 30 s live profile puts **90–93% of sampled CPU time in native rake workers**.
  The focused change shares transforms, triangle thickness and prism bounds
  between two stock domains inside one native call. Each domain retains its own
  clipping, interval union and provenance. `sk_ring_stock_sweep_rake_pair` validates
  both domains first and restores both on application exceptions. The existing
  single-domain API and force path are unchanged; the C ABI addition is additive.
- `HobbingSweep.cutPassMany` opts in with `STOCKKIT_RAKE_BATCH_DOMAINS=1`, captured
  when the sweep is created. It pairs domains and sweeps any odd final domain
  normally. Geometry is streamed, not retained across passes. Whole-sweep cache
  prototypes were rejected: a large engaged case exceeded their memory limits
  and fallback overhead made them slower. No acceptance cut used those caches.
- Paired, interleaved benchmark at windows **30,000–30,099**: **70.9068 s ->
  54.6342 s**, **1.29785x** speedup (about **23% less cut time**). Both paths use
  exactly **110,383 poses**, and every inspected endpoint/provenance value matches;
  the maximum symmetric-difference arc length is zero, including physical ends.
  Both begin from the same 2,400-window coarse-precut seed. This fixture tests
  geometry and throughput, not full-stroke grading or total batch runtime.
- All accepted numerical settings remain unchanged: **4 threads per new cut,
  0.0015/0.04 mm grid, 0.000025 mm sweep, 0.0000015625 mm path tolerance**, and the
  complete HB8 schedule and force/sync histories. No coarser grid or inactive-knot
  pruning has been adopted. A separate reference compatibility fixture retains
  the previous stock/provenance hash exactly. All **three native suites** pass;
  new pair tests cover different grids, 1/4/8 configured thread counts, invalid
  second handles, duplicate domains and invalid poses. The binding audit passes
  Linux, Windows and both macOS architectures.
- Evidence: `build/flexspline-shared-sweep/validation.json`,
  `paired-streaming/geometry-speed.json`, `pair-compatibility/engaged.json` and
  `engaged-profile.txt`. A hash-bound proof selects the new native library and
  frozen `main-batched.hl` for future cases. The original executable, native
  library and execution fingerprint remain archived. The benchmark's compiled
  Haxe source is retained alongside its binary; later cleanup only removes
  unused experimental benchmark branches from repository source.
- The old Python launcher alone was replaced. Its three active native cuts
  (**combined, table-axial, lead-mandrel**) retain their PIDs, original executable,
  native library and progress; the new launcher adopts their completed artifacts.
  The original table-radial case still finishes before the serial runner stops.
  HB4 and the face-end supplement continue unchanged. Adopted cuts retain their
  original library hash; where the replaced launcher cannot report an OS exit
  status, the record explicitly says so and requires the completed physical
  measurement and successful cut log.
- One additional **encoder-eccentricity** full source cut starts immediately with
  paired domains, in a separate **1,500 MiB / no-swap** scope. Its result is reused
  by the four-slot budget scheduler rather than cut again. This is an additional
  bounded full-source validation while existing cuts finish, not a fifth permanent
  worker. Each fresh worker launch reserves estimated memory, including workers
  still allocating, while leaving **3 GiB available headroom**. Existing adopted
  allocations are not counted twice.
- The fast scheduler regression uses synthetic saved results and a real short
  child process: completed cases survive a native/bytecode upgrade, an active
  child is adopted without restarting it, old execution provenance is archived,
  restarts reuse saved cases, and a slower candidate is rejected without changing
  completed acceptance status. Full-reference and HB11/HB12 acceptance remain
  pending; the short benchmark does not establish an hours-scale full-batch ETA.

### HB4 convergence diagnostics (2026-10-18)

- The accelerated HB4 reference in `build/flexspline-hb4-accelerated/` exited
  with status 1: the full multi-domain sweep did not converge after refinements
  0–3. It produced no acceptance result. Its binary and logs are preserved;
  HB4 remains open. This failure did not identify the offending circle.
- `HobbingSweep.cutMany` now accepts an optional progress callback: per-level
  path tolerance, window progress and pose counts, then every domain's maximum
  symmetric-difference arc length and its physical radius/Z. HB4 persists each
  comparison immediately in `convergence.json`, including on eventual failure.
- The replacement diagnostic run is `build/flexspline-hb4-diagnostic/`, with
  8 threads and the validated paired-streaming native provider. The full stroke,
  physical face endpoints, original inspection grid, 0.00005 mm convergence
  tolerance and refinements 0–3 are unchanged. `HB4_EVIDENCE` selects its output
  directory. The paired-domain setting is included in settings/result reports.
  No tolerance relaxation or additional refinement is silently accepted.
- The short `multi-convergence` regression passes with paired domains: exact
  stock equality against the original single-domain sweep at 1/4 threads,
  identical refinement and maximum arc change, and complete diagnostic events.
  HB11 cuts and their frozen binaries/libraries continue unchanged. Diagnose the
  full-run comparison before choosing a further refinement or geometry fix.

### Shared component migration after main merge (2026-10-18)

- All 14 remaining motion/hobbing envelope parts now extend `MachineComponent`
  directly, using its stored recipe inputs and typed parameter accessors with
  `ComponentRecipeSupport.resolve`. `ComponentEnvelopes` supplies block/annular
  geometry; optional `EngineeringMetadata` and `PrecisionErrors` facets supply
  independent provenance and error bounds. Recipe IDs, input units, ratings,
  generating hob geometry and physical mounting frames remain unchanged.
- Removed `motion.PrecisionPart` and the duplicate hobbing `MineralCasting` and
  `CastInsert` implementations. Bed/column layouts use the shared structural parts
  and the single mineral-cast material definition. `LegacyFoundationRecipes`
  reads saved HB5 IDs and translates their cores/recesses into structural recipe
  inputs; subsequent saves use the shared IDs. The hobber retains its paired-face
  and actual precision-mount attachment checks.
- Validation: **986 parts/persistence assertions**, **60 geometry assertions**,
  **264 hobber integration assertions**. The headless app coarse cut completes
  **922 windows**, **zero collisions**, **5.545250 s** total simulated cycle
  (**3.600912 s** cutting), **0.00396151 arcseconds** maximum sync error and no
  inspection failure. This is the coarse app regression, not reference grading;
  HB4's fine convergence acceptance and the running HB11 studies remain separate.
- Evidence: `build/flexspline-refactor-app/validation.json` and small test logs.
  Main's newer platform APIs require matching Haxeon packages; validation uses
  an isolated workspace manifest pointing to main's `e32024b6d8` package sources,
  leaving the dirty worktree Haxeon checkout unchanged. Fresh RobotKit, SimKit,
  trajectory and MotionKit providers were built under that evidence directory;
  old cached providers lacked new main APIs. The MuJoCo core archive was reused,
  rebuilding both sides of SimKit's changed private backend interface. No active
  HB11 binary or native provider was rebuilt or replaced.

### HB4 convergence diagnosis and deeper reference rerun (2026-10-18)

- The four-circle reproduction retains exact original sample coordinates, full
  tool extent/probes and all 2,641 original pass times. Its first comparison
  reproduces the full diagnostic's 0.006243577840651433 mm worst arc change exactly.
  All four circles retain 201 material intervals. After saving levels 0/1 it was
  deliberately stopped to avoid spending another long run on the diagnostic;
  `build/flexspline-hb4-probe/handoff.json` records this, and both snapshots remain.
- Endpoint source provenance reduces the lead and profile neighbourhoods to 103
  and 111 passes. Both retain the full original tool and motion sampler and match
  both endpoints and source passes **exactly** against full-stroke levels 0/1.
  The six-level studies take about **88 s** and **72 s**. Their recorded fixtures
  are in `stockkit/tests/metrology/fixtures/`; run the lead study with
  `check-hb4-boundary.sh`, using `HB4_BOUNDARY_INPUT`/`HB4_BOUNDARY_EVIDENCE` for
  the companion profile study.
- Cause supported by these diagnostics: the default refinement cap of three is
  insufficient for the aggregate **total-circle** symmetric-difference criterion.
  At level 3 the largest individual lead boundary shift is **0.559 nm**, and the
  profile shift is **0.408 nm**. Summing the two flank shifts and multiplying by
  200 teeth reproduces the failed full-circle changes **0.202777841 µm** and
  **0.163010766 µm** within **4.1e-12 mm**. Stable interval counts and unchanged
  source passes do not indicate a topology failure in these neighbourhoods.
- Level 4 reduces the largest local shifts to **0.00250 nm** (lead) and
  **0.00537 nm** (profile); at level 5 they are **0.00125/0.01662 nm**. These are
  numerical endpoint changes, not physical deflection budgets. The repeated-tooth
  scaling diagnoses the aggregate failure; it does **not** certify other circles.
- HB4 reference alone now permits refinements **0–5** and still stops only when
  every computed circle satisfies the same **0.00005 mm** total-arc limit. The
  original grid, face endpoints, tool and motion are unchanged. No native clipping
  arithmetic or accepted tolerance was relaxed. The new full-face rerun uses
  **8 threads**, paired domains and a frozen binary in
  `build/flexspline-hb4-refined/`. Acceptance remains open pending its successful
  convergence, grading result and zero exit status. HB11 continues on its frozen
  binaries/providers. Metrology smoke regression: **11 assertions passed**.

### HB11 mandrel case reviewed with face ends (2026-10-18 evening)

- The complete 92,151-window mandrel lead/end cut has finished. The promoted
  report is reconstructed exactly from hash-verified original fine profile and
  new physical lead/end data: original profile traces and interior pitch, runout
  and thickness arrays are unchanged, and every replaced lead value/trace matches
  the supplement. Both physical **Z=0/8 mm** planes are measured. The narrow lead
  report intentionally omits profile evaluation; the promoted report retains the
  original complete profile and has no coverage warnings.
- The individual mandrel case fits all nine extrapolated class-6 limits under the
  accepted fine study settings. Overall extrapolated grade is **4**, driven by
  **Fp=9.124673 µm** and **Fr=8.493766 µm**. Other parameters are grade 1:
  **fp=0.141554 µm**, **Falpha=0.256213 µm**, **ffalpha=0.005970 µm**,
  **fHalpha=0.262226 µm**, **Fbeta=0.478125 µm**, **ffbeta=0.487574 µm**,
  **fHbeta=0.022124 µm**. Module 0.3 remains outside the standard's scope;
  `isoClass=null` is preserved, and this is not ISO certification.
- Source-signature review uses baseline-subtracted traces. The runout's once-per-
  revolution amplitude is **3.999311 µm**, matching the **Assumed 4 µm** eccentricity
  input within **0.018%**. That harmonic explains **99.999990%** of its variance,
  with **0.000890 µm** residual RMS. Both cumulative-pitch flank traces are likewise
  dominated by one revolution (amplitudes **4.256791/4.256844 µm**). Diagnostic
  signature thresholds were stated as 1% amplitude agreement and 99.9% explained
  variance; these are not additional physical error-budget allocations.
- The eccentricity-only mean thickness shift is **0.0000908 µm** and its varying
  once-per-revolution amplitude is **2.848736 µm**. This does not close combined
  tooth-thickness acceptance or replace the separate radial-force/wall-deflection
  split and assumed support-stiffness sensitivity.
- Settings retained: **0.000025 mm sweep**, **0.0000015625 mm path**, original
  profile at **8 threads**, lead/end supplement at **4 threads**, 404 computed
  lead circles. Completion is supported by the full physical measurement and
  successful completion log. The adopted process's OS exit code is unavailable
  following launcher replacement; it is recorded as null, not fabricated as zero.
- Read-only audit took **0.4 seconds**, with no recut. Detailed values, harmonic
  fits, coverage checks and hashes: `build/flexspline-reference-budget-parallel/`
  `mandrel-case-review.json` and `.md`. **Individual mandrel review is complete**;
  combined budget/signature superposition, other source cases, HB4 fine reference
  acceptance and HB12 remain open. No running cut or frozen provider was changed.


### Whole-machine interference audit — layout acceptance reopened (2026-10-18)

- The user's viewport review exposed a gap in the earlier acceptance claims.
  The amber scene overlay is a proximity display, but its collision rules exclude
  same-rigid-body, moving-joint neighbour and as-designed contacts. It cannot
  establish that the machine's physical solids fit. Earlier HB6/HB9 checks proved
  hob clearance from the mandrel/table and process envelopes, not whole-machine
  clearance; their zero-collision result must not be presented as the latter.
- Added `FlexsplineHobberChecks interferences [reference]`, using cached exact CAD
  shapes and native common volumes for all broad-phase candidates, including
  fixed/mating parts. At the initial pose, **86 parts / 204 candidates** produce
  **80 positive-volume pairs in the coarse scene** and **82 in the reference**,
  above 0.001 mm³. The boolean audits take **3.06 / 3.69 seconds**. Raw pair lists
  and readable reports are in `build/flexspline-interferences/{coarse,reference}/`.
  Hob/blank intersection is expected cutting engagement; these raw counts are
  not an assertion that every pair is an unintended collision.
- Concrete non-cutting problems include:
  - Each Z rail overlaps its steel insert by **112,500 mm³**, its entire envelope
    volume. Rail poses use the axis datum where the mounting-base datum is needed.
  - Zplate/blockZLeft and Zplate/blockZRight each share **27,138.8 mm³**; the plate
    needs actual block interfaces rather than a box laid through the blocks.
  - Zplate also intersects an X rail, X bridge, screws, nuts and a screw support.
    The Y plate likewise intersects its rail, block, screw and nut. Synthetic
    fixed mates place these parts but do not create passages or mounting brackets.
  - Z-drive hardware intersects the spindle package; other drive hardware and
    scales intersect column/rail seats. These routes need a mechanical layout pass.
  - Adjacent servo-drive enclosures overlap by **270,000 mm³**; their current
    pitch and placement do not provide a usable electrical enclosure arrangement.
- Mechanical layout is **not accepted**. HB12 now explicitly gates on whole-machine
  solid clearance at rest and over travel, with real interfaces and separately
  identified cutting engagement. Next design work is stage/drive/interface layout
  correction followed by the same exact audit and a motion sweep. No geometry,
  scene collision filters, active simulation window, HB4/HB11 cut or frozen native
  provider was changed by this audit. Existing numerical source studies retain
  their original, narrower scope.

### Mechanical redesign — physical mounts and sampled clearance (2026-10-18)

- Replaced the generic overlapping carriage boxes with recipe-backed steel Z/X/Y
  saddles, a closed-box X bridge, A yoke and separate screw/motor/scale frames.
  Their mounting datums, load paths and remaining design assumptions are recorded
  in `MACHINE_DESIGN.md`. Z and X have four guide blocks each; Y has two. Rail
  alignment checks include block lengths, end margins and 5 mm overtravel.
- Enlarged the shared mineral-cast bed/column, moved the column rearward, and added
  a cast-in steel side insert for the Z-drive frame. Legacy HB5 casting recipes
  retain the old footprint, insert recesses and mass; they do not silently load
  the new layout. No general casting/tooling/process model was introduced.
- Added a 40 mm fixture riser and raised the cutter/stage datums with it, preserving
  cutter-to-work `HobbingAxes` transforms and mandrel support beneath the tooth band.
  The forward outboard housing now actually carries a standard 6002 bearing on a
  stepped Ø15 arbor nose. Its bearing recess and shaft passage are distinct; the
  hob seat remains 60 mm from the spindle gauge line despite the longer arbor.
- Z uses the assumed 750 W servo family, with matching shaft passage, coupling
  space and a 60 mm screw input journal. `BallScrew` persists that dimension and
  uses it in both its solid and axial-compliance calculation; old recipes retain
  their 50 mm default. X/Y remain 400 W. Electrical drives now mount on a separate
  supported backplate with 20 mm lateral gaps.
- Physical mounting contact, connected weldments, guide alignment/overtravel,
  full assembly persistence and the existing kinematic checks pass **731 assertions**.
  Parts/persistence/legacy/journal checks pass **995**, cartridge geometry **60**.
  `check-layout.sh` provides a repeatable entry point without rebuilding native
  providers or touching the active reference studies.
- Exact whole-machine audits cover **99 parts**, including fixed and moving-joint
  neighbours. There are **zero unintended common volumes >0.001 mm³** at the
  initial pose (coarse/reference: 3.94/4.43 s) and at **82 poses per configuration**
  (initial plus the 3×3×3×3 X/Y/Z/A grid; 125.02/122.27 s). Only hob/blank cutting
  engagement is allowed. Rigid-pair intersections are computed once and reused
  under their invariant rigid transform, never skipped. Reports are in
  `build/flexspline-layout/{coarse,reference,coarse-final,reference-final}/`.
- The rebuilt application completes its headless coarse cut in **5.55875 simulated
  seconds**, feed **3.600912 s**, **923 gash windows**, zero cutter/fixture-envelope
  collisions, sync **0.003932 arcseconds**, and completed inspection. This coarse
  smoke result is ungraded and is separate from whole-machine solid clearance.
- Mechanical **sampled layout clearance is established**. Continuous swept-volume
  proof, rotated B/C detail envelopes, tolerance stacks, fastener/retention detail,
  structural/modal stiffness and updated full-reference drive/error qualification
  remain open before HB12 acceptance/fabrication. Existing HB4/HB11 frozen cuts
  keep their original geometry-independent/numerical scope; they do not qualify
  the new steel structure, longer arbor or changed moving masses. No running
  reference process or frozen native provider was changed.

### Detailed hob CAD and display (2026-10-19)

- `HobSolid` constructs a single fused tool from the generating rake sections,
  with helical lands, straight gashes, assumed backing relief, bore, optional
  keyway and clipped mounting ends. Default machine/cutting envelopes stay fast.
- The detailed display tessellates that same fused solid. The separate overlapping
  core/land display construction and its end-cap fans were removed. Sharp normal
  seams retain coincident positions; mesh topology is checked after a 1 nm weld.
- Checks cover manifold edges and consistent winding, connected genus-one topology,
  duplicate/collapsed triangles, normals, mounting-end bounds and mesh/CAD volume
  agreement within 0.5%. Short tools cover zero and both signs of rake, two starts,
  keyed bores and the 12-gash reference. Native CAD edge samples remain within
  the configured 0.5 µm profile simplification tolerance.
- Backing/relief dimensions remain explicitly assumed; this is no claim of supplier
  grinding accuracy, sharpening kinematics, a new metrology grade or HB12 acceptance.
  See `HOB_GEOMETRY.md` and `check-hob.sh` for representations and focused checks.
- The actual full-length keyed reference scene also passes: 122,034 welded
  vertices, 244,068 triangles, one closed genus-one shell and positive signed
  volume 20,819.54475 mm³. Its initial CAD/scene generation took about four minutes
  and peaked around 4.5 GiB; subsequent opens can use Materia's artifact cache.
  The isolated tool is open in Materia. Parts/persistence checks: 997 passed;
  short native CAD checks: 18,623 passed (maximum sampled edge error 0.303 µm).
  Packed timing-belt mesh regression checks also pass.

### Installed detailed-hob inspection (2026-10-19)

- `detailed.project.json` now shows the full reference machine at CAM's full-depth
  setup: centre distance 45.625 mm, swivel −0.550° (negative hob lead angle),
  Z = 4 mm (mid-face), and the planned table phase. The installed hob has its
  actual plain bore; the isolated example's assumed keyway is not installed.
- Nominal installation: 16 mm bore/arbor diameters, tool seat from 40 to 80 mm
  on the arbor, coaxial tool and outboard bearing. Housing end clearances are
  40 mm at the spindle and 65 mm at the outboard support. The bearing width
  remains on the 15 mm support journal. Checks pass at 243 poses per configuration
  (X/Y/Z/A grid × three spindle phases), with generating rake points reaching the
  toothed rim at the CAM setup. Installation checks: 5,888; mechanical suite: 731.
- Exact common-volume audits use the conservative hob blank at 85 poses per
  configuration, adding the CAM inspection setup and pass entry/exit to the travel
  grid and initial pose. Coarse/reference: 19,660/19,666 pair checks, zero unintended
  intersections above 0.001 mm³, in 126.64/121.90 s. Tool/blank cutting engagement
  is the sole allowed overlap. This is sampled clearance, not continuous sweep proof.
- The exported reference scene passes all 99 member-pose comparisons and detailed
  tool containment checks: 244,088 hob triangles, maximum radius 16 mm, ends 0–40 mm.
  The body is clipped to the audited outer cylinder; its conservative clearance
  bound covers all spindle phases. Evidence: `build/flexspline-installed/validation.json`.
  `check-layout.sh` now includes the nominal installation checks.
- **Open before fabrication/HB12:** nominal datum seating does not provide physical
  axial retention. Stops/spacers/retainer hardware, retention capacity and bore/shaft
  manufacturing-fit tolerances remain unspecified. No full cut or metrology/error
  qualification was rerun or claimed for this scene inspection.

### Layout hold and HB4/HB11 follow-up (2026-10-19, superseded below)

- The horizontal-layout proposal is **on hold** at the user's request. No layout
  direction is confirmed; the current scene/layout remains unchanged. Turning is
  being considered under a separate lathe plan. Do not apply the proposed HB-D1
  replacement, foundation/axis redesign or turning-spindle requirements until
  the user confirms a direction.
- HB11 remains on its frozen workers/providers. Current completed cases are
  baseline, mandrel, encoder eccentricity and table radial (4/15); four further
  source cuts are active. The saved review has been refreshed from completed
  artifacts. Combined and table-axial adopted cuts previously stopped at
  64,900/92,151 and 68,700/92,151 windows without producing inspections. They
  are missing evidence, not measured class-6 failures.
- A future launcher resume now detects stale interrupted adoptions, archives their
  partial logs/execution evidence and retries them using the validated frozen
  binary/provider and unchanged settings. Finished but invalid measurements still
  fail; they are not silently replaced. The active launcher is not restarted and
  its workers are not signalled. Synthetic launcher regressions pass, including
  partial-log preservation, retry, reuse of completed cases and rejection of an
  invalid completed result. No new full physical cut was started for this fix.
- HB4's refined full-face run exited 1 after level 5: total-circle arc change
  **0.0617216 µm** exceeds the unchanged **0.05 µm** criterion. The worst circle
  is lead domain 1, radial index 1, axial index 93 (Z≈3.72 mm). HB4 remains open;
  the earlier local-boundary diagnostic does not establish full-circle acceptance.
  `build/flexspline-hb4-refined/postrun-review.json` records this result. Inspect
  these final worst circles before committing to another expensive full rerun.

### Confirmed moving-column revision — work in progress (2026-10-19)

The confirmed vertical GE-class decision above supersedes the earlier layout
hold and horizontal proposal. Source now has moving-column X, direct Z/A mounting,
twin Y rails, an integral fixture base and floating radial outboard support.
Enclosure panels are omitted for inspection; the electrical rack is hidden by
`inspection.scene.json` visibility overrides without deleting its wiring.

This checkpoint is not layout acceptance. Mechanical tests passed 766 assertions
and a short assembled/analytic work-frame cut agrees within 1e-8 mm/rad pose,
1e-8 mm arc and 1e-6 N load tolerances. The full travel-grid audit still reports
interferences; the loading check also needs exact-solid handling of hollow
objects rather than their enclosing boxes. The desktop rebuild is blocked by
the checkout's Haxeon transport API mismatch. HB6/HB9, final masses and footprint,
clearance acceptance and documentation reconciliation remain open. Frozen
HB11/HB4 providers and workers were not rebuilt for this revision.

### Main rebase (2026-10-19)

Rebased onto local main `b7f0b79b6` (ahead of fetched origin/main), preserving its
detailed HGR15 rails, supplier-block integration, ball nuts, screw races/journals
and retainer recipes. BallScrew retains the hobber's configurable input journal
and ground-screw procurement flag alongside main's detailed geometry. Shared
structural foundations and the single mineral-cast material remain authoritative;
PrecisionPart and duplicate casting classes remain removed. The working Haxeon
checkout now matches main's recorded `9e37bf4a` revision; existing vendor changes
were preserved. No native providers or frozen cut binaries were rebuilt.

The rebased build passes 761 mechanical/persistence assertions (the enclosure's
omission removes its checks) and the short assembled/analytic frame regression:
max pose delta 1.85e-13 mm/rad, boundary delta 3.17e-13 mm and load delta
1.82e-10 N, against unchanged 1e-8, 1e-8 mm and 1e-6 N criteria respectively.
Full travel-grid acceptance and the remaining layout gates above stay open.

### GE moving-column layout accepted as a CAD/app layout (2026-10-19)

- Replaced the remaining overhead head with a vertical stack: Z saddle front,
  compact concentric crossed-roller/motor/clamp cartridge, vertical swivel plate,
  twin front-face Y rails and a vertical spindle saddle with an open rear C bracket.
  The outboard bearing remains axially floating on the same Y saddle. Retired the
  old `A-yoke`, `Y-saddle`, `X-bridge` and `Z-saddle` designs without legacy aliases.
- Achieved guide-plane offsets X/Y/Z = **121/70/161 mm**, versus the old
  **347/137/325 mm**. The A bearing front face is **114 mm** behind the hob;
  the Z saddle mounting face is 135 mm. The initial moving-column Z offset was
  125 mm; moving its rail plane rearward by 36 mm permits the actual front-mounted
  bearing/plate/rail stack. Its 161 mm result is explicitly 11 mm beyond the
  approximate 150 mm target. The fixture shoulder stays at Z=363 mm; its integral
  base is not raised to conceal head clashes.
- Moved the X drive outside the tilted spindle envelope and repositioned lifting
  inserts. The first X block is at station 102 mm, giving 24.5 mm of carriage plate
  ahead of its front mounting bolt and full support under its nominal block foot.
  Used insert-only precision mounts, rail margins and declared overtravel remain
  checked. The stationary end shields fit below the head; moving bellows remain
  a reserved provision. Enclosure panels stay omitted and the electrical rack
  stays hidden in the inspection view, with its wiring retained.
- Native-volume masses: X **127.43 kg**, Y **13.18 kg**, Z **39.90 kg**, machine
  **343.67 kg** including the rack and excluding enclosure panels. Continuous
  thrust margins are X/Y/Z **888.28/911.97/1548.81 N** under the stated assumed
  0.5 m/s², 100 N process plus 100 N friction demand, Z gravity and Y gravity
  at ±15° tilt, after 20%
  continuous-thrust reserve. These are drive-sizing checks, not stiffness results.
- Validation: **775** mechanical/persistence assertions; **1000** parts assertions;
  **60** part-geometry assertions; **1532** loading/range checks at OD25/140 mm,
  face20 mm and ±15° A. Installation checks pass **5888** assertions over 243 poses
  per configuration. Exact-solid audits pass **85 poses per configuration**,
  **37,290** candidate pair checks in total, with zero unintended intersections
  above 0.001 mm³; only intentional hob/blank engagement is permitted. This is
  a sampled grid, not continuous swept-volume certification.
- Work-frame regression: max pose difference **1.78e-13 mm/rad**, cut boundary
  difference **1.85e-13 mm** and force difference **1.82e-10 N**, within unchanged
  1e-8 mm/rad, 1e-8 mm and 1e-6 N criteria. The nine-circle reference-geometry
  test uses one thread and recorded sweep/cut-pass tolerances of 0.00002/0.000005 mm.
  No full reference metrology or force cut is rerun for this rigid-frame check.
- HB6/HB9 acceptance now uses this front-mounted head. The coarse physical app cut
  completes in **6.00275 simulated seconds**, with 3.600912 seconds programmed
  cutting feed, **982** gash windows, zero cutter/fixture envelope collisions and
  **0.00393175 arcsec** sync error. The remaining 2.401838 seconds include ramps
  and positioning. This ungraded smoke cut does not establish HB12's full-reference
  ≤3 min cycle or class-6 acceptance. The earlier overhead app smoke was 5.55875 s;
  accepted axis acceleration/moving-load timing now differs.
- `MACHINE_DESIGN.md`, `LAYOUT_REPORT.json` and the dimensioned `LAYOUT_SECTION.svg`
  carry the comparison and evidence. The section is schematic and labels projected
  Y-offset parts; it is not a manufacturing drawing. Compact swivel procurement,
  manufactured fits/retention, modal stiffness and physical accuracy remain open.
  Frozen HB11/HB4 cuts/providers were left untouched.

### Centred X drive and supplier guide geometry (2026-10-19)

At the user's request the X screw and nut load path now lie between the bed
rails, in a recessed 110 mm casting channel, with the motor and fixed support
at the rear. A separate cast-in steel floor insert carries the screw deck; the
moving nut carrier drops centrally from the column carriage. The chip guard
and insert have 2.5 mm radial clearance. The screw axis and coupling sense were
reversed together; work-frame cutting kinematics remain unchanged. Channel
covers/bellows remain a design provision.

The hobber had selected nominal carriage envelopes despite the detailed HGR15
rail generator on main. All twelve blocks now select pinned HGH15CA/HGH25CA
supplier CAD; X rails select a matching HGR25R supplier-derived section. Shared
motion recipes persist source revisions and reconstruct from explicitly prepared
local assets, with no runtime network access or nominal fallback. Supplier fit,
bores, four cut lengths per series, revision rejection and recipe reconstruction
checks pass. CAD exteriors do not qualify preload, ratings or mixed-material mass.

Updated validation and calculated masses/thrust margins are recorded in
MACHINE_DESIGN.md and LAYOUT_REPORT.json. Frozen HB4/HB11 providers and workers
are unchanged. The app coarse cut retains its 6.003 s cycle, 982 windows and
zero cutter/fixture collisions; this does not close full-reference HB12 acceptance.

Acceptance: 801 mechanical/persistence assertions, 1000 parts assertions, 60
parts-geometry assertions, 1584 range/loading checks and 5888 installation checks
pass. Both 85-pose exact-solid grids pass at 0.001 mm³, including A ±15°, with
no unintended intersections. Work-frame regression remains within 1e-8 mm/rad
for poses, 1e-8 mm for cut boundaries and 1e-6 N for loads. The supplier-reference
checks and the new-layout app coarse cut pass. Updated X/Y/Z moving masses are
129.55/13.49/40.52 kg; calculated machine mass is 343.76 kg, with the stated
uniform-density supplier-CAD assumption.

### Rail-end shields removed (2026-10-19)

The user chose to remove the stationary rail-end shields rather than detail their
fastenings. Removed the assembly member, BOM item and unused GE-guard recipe;
the earlier end-shield acceptance above describes the previous layout. Rails,
blocks and the central X screw are exposed in the inspection model. Bellows,
attached end frames, chip protection and compressed-length checks at both travel
limits remain open; no replacement cover is fitted in this change.

Removal verification: 796 mechanical/persistence assertions pass. Recomputed
machine mass is 343.27 kg; moving masses and drive margins are unchanged. The
previous clearance/app evidence covers the guarded superset; no full cut rerun
was needed for removing these stationary parts.
