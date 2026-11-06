# Actuator plan

**Goal.** A parametric generator for our own robot joint actuators, for robotic arms first and
humanoid and industrial joints after. One actuator layer (housing, motor, sensors, brake, sizing,
outputs) wraps a **swappable reducer**: strain wave first, cycloidal second, planetary later. Every
design is modelled in materia, built, and measured on a dyno so the model and the hardware check
each other.

Status: draft, 2026-10-31. Written from the existing plans and READMEs listed under *What exists*;
the code behind them was not reviewed, so check each "exists" line against the code before relying
on it.

## Requirements the plan answers

1. **Arm joints use a T-style housing** (two perpendicular mounting faces, the housing is the
   structural elbow) **with a hollow through-bore** for cables. Reference: ZeroErr eRob T-style
   modules, already the mechanical baseline of `examples/cobot-arm`.
2. **Both strain wave and cycloidal** must be considered, in the same actuator frame, so they can be
   compared fairly on one dyno.
3. **Parametric.** A joint spec goes in; CAD parts, a drive motor profile, a robot model entry and a
   bill of materials come out.
4. **The drive is ours** (`materia-internal/drive`), and the integrated actuator gets a custom ring
   PCB. Drive specs (60 V, 21 A peak, 15 A rms, one SPI encoder) are bench-stage values, not fixed
   limits. The actuator sets the electrical requirements; the drive follows.

## What exists (to verify)

- `gearkit`: involute `GearSpec`/`ToothProfile`, `ConjugateRack`, `HobDesign`, ISO 1328 numerical
  inspection, `ErrorBudget`. No strain-wave or cycloidal tooth code.
- `gearkit/TOOTH_DESIGN_PLAN.md`: plan for flexspline deformation, circular-spline envelope, meshing
  analysis and a `StrainWaveDesign` output. Not implemented as far as the plan says.
- `machinekit/examples/flexspline-hobber`: hobber design and simulation, target ISO 1328 class 5-6
  at module 0.2-0.5. `examples/circular-spline-skiver/PLAN.md`: sibling skiving machine, plan only.
- `machinekit/STRAIN_WAVE_RESEARCH.md`: papers, open code, manufacturing routes, supplier prices.
- `machinekit/PRECISION_ROTARY_PLAN.md`: direct-drive rotary table and test bench; the bench
  methods (ISO 230-7 style) are reusable for an actuator dyno.
- `examples/cobot-arm`: four reach classes using bought modules; `actuator-selection.json` and
  `reference-sources.json` record the ZeroErr/MYACTUATOR/CubeMars comparison. Reference module
  eRob80H100I V6: 31 Nm rated, 70 Nm peak, 80 mm OD, 64 mm long, 18 mm bore, 1.09 kg with brake.
- `TRANSMISSION_PLAN.md`: couplings and `SimpleTransmission` exist. Checked in code
  (`machinekit/haxe/src/machinekit/`): `motion/ServoMotor` carries rated and peak torque, speeds,
  rotor inertia, encoder counts and an electrical rating (torque constant, windings for 48 V, AC and
  high voltage), each with a `ValueBasis` (Assumed / Stated / ...). `motion/Gearbox` carries ratio,
  efficiency, inertia and a `ReducerSpec`. `motion/PlanetaryReducer` is an assumed catalogue
  component (ratio, efficiency, backlash, torsional stiffness, rated/peak torque, bore, loads).
  `robotics/CobotJoint` (164 lines) is the current strain-wave cobot module: fixed ratio 100,
  efficiency 0.85, five hard-coded sizes (diameters 75-150 mm, output torque 12-330 Nm), built from
  `GearedArmJoint`, `Gearbox` and `ServoMotor`. So motor and reducer **data types exist**; what
  is missing is a generator that sizes a motor and housing from a joint spec, and a reducer
  design behind the data.
- Nothing exists for cycloidal reducer design, for planetary design beyond the catalogue envelope,
  for non-involute metrology, or for a parametric actuator-level model.

## Prerequisites and existing code (checked 2026-10-31)

No refactor blocks the start. Status of `RESTRUCTURE_PLAN.md` (MachineKit MK0-MK10), from git and the
source tree:

- **Landed** (`ba0a0c32e`): MK0-MK5 and MK7: hierarchical assemblies, per-level saved form,
  `ComponentRegistry`, typed `ComponentFacet`s, split of `MachineAssembly` into `MechanicalAssembly`,
  `ServiceNetwork`, `DriveSystem`, `AssemblyInventory`. Build the generator on these, not the old
  API. Register new parts through a `*Components` module, and use facets instead of capabilities.
- **Not landed** (as far as the tree shows):
  - **MK6**, structured recipe values. `ComponentValue` is still `Number | Integer | Boolean | Token
    | Unset`. A winding, a duty profile or a tooth table would be stored as JSON in tokens until MK6
    lands. Either land MK6 first (it bumps the schema) or keep the generator's records outside
    component values (in its own `design` object) and convert later.
  - **MK9**, the rename `machinekit.motion` -> `machinekit.drive`. It is a late mechanical commit
    that waits for no open MachineKit branches. Put new motor/reducer code where the rename will
    move it cleanly, and expect to rebase if this work is open when MK9 runs.
  - **MK8** (welding to ProcessKit) and **MK10** (`AssemblyIntegrity` across all examples) do not
    touch the actuator. `AssemblyIntegrity` already exists in `machinekit/assembly`; use it as the
    housing generator's support/interference check.
- **Reuse** from `machinekit/motion`: `ServoMotor`, `Gearbox`, `PlanetaryReducer`,
  `CrossedRollerBearing`, `AngularContactBearing`, `BrakeServo`, `MotorBrakeSpec`, `EncoderPart`,
  `LeadshineTorqueCurves` (a torque-curve representation), `MotorDrive`/`MotorDriver`. Check
  whether `CrossedRollerBearing` and `BrakeServo` can serve the output bearing and brake before
  writing new ones.
- **Coordination.** The working tree has uncommitted edits by others (sheet-router, gantry, mount
  cleats, a `haxeon` submodule change). `machine-tending` had unmerged MachineKit changes. This
  plan adds files only; confirm with the owners before touching `ServoMotor`, `Gearbox`,
  `CobotJoint` or `MachineKitComponents`.
- **Gearkit.** Involute behaviour must stay bit-compatible: `stockkit`, `camkit` and `app` hobbing
  code and the ISO 1328 grading depend on `GearSpec`/`ToothProfile`. The tooth-form abstraction
  (A3) is an addition behind the existing classes, guarded by the current gearkit, stockkit hobbing
  and metrology tests.

Recommendation: start A0 and A1 now (they only need the existing data types). Decide MK6 before
A2, because the housing and motor generators are the first code that wants structured values.

## Architecture

Three layers. The seam between layer 1 and layer 2 is the only place reducer types meet the rest.

```
JointSpec  ->  Actuator layer (shared)  <->  Reducer interface  <->  Reducer generators
                housing, motor, sensors,                              StrainWave (gearkit)
                brake, thermal, sizing,                               Cycloidal  (gearkit)
                outputs                                               Planetary  (later)
```

### Layer 1: `JointSpec` (input)

Requirement, not design. Fields (units in names):

- loads: `ratedTorqueNm`, `peakTorqueNm`, `shockTorqueNm`, `speedAtRatedRpm`, `maxSpeedRpm`,
  `dutyProfile` (torque/speed samples or a named cycle), `outputBearingLoadsN`/`Nm`
- envelope: `maxOuterDiameterMm`, `maxLengthMm`, `minBoreMm`, `massTargetKg`
- housing: `style` (`Inline`, `TStyle`), `mountPattern` (named, e.g. `eRobCompatible`, `Materia1`)
- accuracy: `maxBacklashArcmin`, `minTorsionalStiffnessNmPerArcmin`, output position accuracy
- electrical: `busVoltageV`, `maxPhaseCurrentA`, `pwmKhz` (inputs to the winding choice; may be
  loosened because the ring PCB is ours)
- features: `brake`, `outputEncoder`, `torqueSensing`, `ipRating`
- reducer preference: `Auto`, or a named reducer with its own opaque parameters

### Layer 2: actuator layer (shared, one implementation)

- **Housing/envelope generator.** See *Housing generator* below for styles, construction,
  interfaces, features and checks. Produces CadKit parts. Treats the housing as structure (bending
  stiffness and bearing support enter the accuracy budget).
- **Motor generator.** Frameless stator and rotor in the annulus left by bore and reducer: slot/pole
  count, winding (turns, wire, fill) from bus voltage and current, Kt, Kv, R, L, cogging estimate,
  thermal network (winding to housing). Emits a **motor profile** for the drive (Kt, R, L, pole
  pairs, inertia, thermal parameters).
- **Sensors, brake, drive board.** Motor-side and output-side hollow-shaft encoder rings, annular
  power-off brake, temperature sensors, ring PCB envelope (and the T leg variant). Outputs the
  mechanical keep-in volumes the PCB must fit; PCB design itself stays in `pcbkit`.
- **Sizing and analysis.** Torque, speed, efficiency, losses, thermal rise over the duty profile,
  mass, inertia, reflected inertia, torsional stiffness chain, backlash stack-up, shock check.
  Ranks candidates; reports why a candidate fails.
- **Outputs.** CadKit parts and assembly, bill of materials, drive motor profile, robotkit actuator
  model (ratio, efficiency, inertia, torque/speed limits), built on the existing `ServoMotor`,
  `Gearbox` and `ReducerSpec` types and `ValueBasis` rather than new ones, error-budget report.
  `CobotJoint` becomes a consumer of generated actuators instead of a table of five fixed sizes.

### Layer 3: reducer interface (the seam)

Haxe sketch, to be refined when the first generator is written:

```haxe
interface ReducerGenerator {
  var kind(get, never):String;                       // "strain-wave", "cycloidal", ...
  function propose(space:ReducerSpace, load:ReducerLoad, opts:Dynamic):Array<ReducerCandidate>;
}

// What the actuator offers the reducer.
typedef ReducerSpace = {
  outerDiameterMm:Float, lengthMm:Float, boreMm:Float,
  inputSpeedMaxRpm:Float, targetRatioMin:Float, targetRatioMax:Float
}

// What a candidate reports back.
typedef ReducerCandidate = {
  ratio:Float, efficiency:Float->Float->Float,       // (torque, speed) -> efficiency
  torsionalStiffnessNmPerRad:Float, backlashArcmin:Float, hysteresisArcmin:Float,
  ratedTorqueNm:Float, peakTorqueNm:Float, shockTorqueNm:Float, lifeHours:Float,
  massKg:Float, inertiaInputKgM2:Float,
  envelope:{outerDiameterMm:Float, lengthMm:Float, boreMm:Float},
  inputInterface:Dynamic, outputInterface:Dynamic,   // shaft/flange/bearing seats
  design:Dynamic,                                     // opaque: StrainWaveDesign, CycloidalDesign
  parts:Array<PartRecipe>,                            // CadKit recipes
  manufacturing:Array<ManufacturingOutput>,           // tooth data for hobber/skiver, profile data for EDM/grinder
  warnings:Array<String>
}
```

Rules:
- Reducer-specific parameters stay inside each generator and travel as the opaque `design`. The
  actuator layer never reads them and `JointSpec` never grows fields for one reducer.
- A generator that cannot satisfy the space returns no candidate and a reason, never a silent
  approximation.
- Efficiency, stiffness and life are **model values** until the dyno supplies **measured** values;
  both are stored and labelled.

### Layer 4: reducer generators (separate code)

- **Strain wave** (gearkit). Implements `TOOTH_DESIGN_PLAN.md`: tooth form, wave-generator cam,
  deformation, circular-spline envelope, meshing analysis, manufacturability rules, search. Adds
  flexspline fatigue and wave-generator bearing life. Natural fit for the hollow bore.
- **Cycloidal** (gearkit, new). Epitrochoid disc profile with profile modification, pin/roller
  circle, eccentricity, disc count and balance, eccentric bearing and pin loads, ratio and
  tolerance stack-up. Hollow bore is the hard part (large hollow eccentric, discs wrapped around
  it); the generator must report the diameter and length it costs.
- **Planetary** (gearkit, after cycloidal or before it; see *Reducer comparison*). Single stage,
  ratio 6-10:1, for quasi-direct-drive humanoid joints. Involute spur or helical gears, so the
  existing involute `GearSpec`/`ToothProfile` and hobbing pipeline cover the tooth work. New work:
  gear-train layout (tooth counts, assembly condition, planet spacing), load sharing between
  planets, carrier and planet bearings, efficiency and noise, and a hollow layout (ring-sun with a
  hollow rotor) whose cost the generator must report.
- **Shared helpers** in gearkit, not a shared tooth model: sampled curve and outline types
  (`Point2`), profile metrology and error budget extended past involute, contact sampling for
  sampled-profile pairs, stress/bearing/fatigue helpers, fit and tolerance helpers.

## Housing generator

The housing is structure and the part that makes the joint a T or an inline module, so its style
must not leak into the reducers.

### Styles

| Style | Description | Used for | Phase |
|---|---|---|---|
| Inline hollow | Output and mounting faces on opposite ends, through-bore on the axis | Humanoid joints, wrist roll, generic joints | First |
| T-style | Output axis plus a perpendicular second mounting face; housing is the structural elbow | Arm joints (ZeroErr style), main target | First |
| Pancake / flat | Large diameter, short length, big bore | Humanoid hips and shoulders | Second |
| Offset / L-style | Motor beside the reducer axis, belt or gear step between them | Humanoid knees, ankles, tight packaging | Later |
| Linear | Roller or ball screw, motor inline or folded | Humanoid knee/ankle linkages, grippers | Later, separate generator |
| Open frame / kit | Stator, rotor, reducer supplied for the customer to integrate | Customers with their own housing | Later |

Open question: a fixed list of styles is simpler; a composable body (main body plus optional
perpendicular faces and offsets) would make T, L and pancake variants of one generator at more
upfront cost. Lean towards composable if the CadKit structural casting recipes and the cobot-arm
module code can express it; not yet checked.

### Construction

Machined from billet first (prototype default). Then cast and machined (reuse `casting-shop` and
the `machinekit.structural` casting recipes). Extruded tube plus end caps for inline bodies, and
shell construction for links. Metal AM for prototypes only. The generator needs the construction
because it changes wall thickness, draft, fastening and cost.

### Interfaces the housing exposes

- Output: flange, bolt pattern, pilot, retention, and the output bearing type (cross-roller,
  four-point, dual angular contact).
- Mounting: base flange pattern and the named patterns (`eRobCompatible`, `Materia1`).
- Hollow bore: diameter, optional rotating seal.
- Connectors and cable exit: axial, radial or side, sealed gland, clocking.
- Link attachment: where the next link bolts on (the second face of the T).
- Cooling: conduction to the link, fins, fluid ports or air purge.

### Features generated

Bearing seats, shoulders, retaining rings and preload; stator seat and winding exit; rotor
retention; encoder ring and read-head mounts (motor side and output side); brake mount; drive PCB
pocket (ring, segment and T-leg variants) with thermal pad and connector access; seals, gaskets and
O-ring grooves; service access, fastener layout and locating pins; handling points; cable
pass-through with bend-radius checks.

### Checks

Bore and cable clearance, wall thickness and stiffness under joint loads, bearing life, thermal path
to the housing surface, assembly order and serviceability, manufacturability (tool access, minimum
radii), mass.

## Reducer comparison

| | Planetary (QDD) | Strain wave | Cycloidal |
|---|---|---|---|
| Typical ratio | 6-10:1 (up to ~100:1 multi-stage) | 50-160:1 | 10-100:1 |
| Backlash | Medium (5-15 arcmin) | Near zero | Low |
| Backdrivability | Excellent | Poor | Poor to medium |
| Efficiency | 90-97% | 60-85% | 80-90% |
| Torque density | Moderate (needs a big motor) | Highest in small sizes | High, better at larger sizes |
| Shock tolerance | Good (low ratio) | Weak | Strong |
| Stiffness | Moderate | Moderate, nonlinear at low torque | High |
| Hollow bore | Awkward | Natural | Hard |
| Axial length | Short | Short | Longer |
| Manufacturing | Standard, easy | Very hard (flexspline) | Hard (precision profile) |
| Torque sensing | Motor current is enough | Needs a sensor | Needs a sensor |
| Best fit | Humanoid legs, hips, knees, force-controlled joints | Arm wrists, precise positioning | Heavy axes, shock-loaded joints |

These are general figures to be replaced by model and measured values. Planetary is the easiest to
model and build, so it is a good first test of the reducer interface, possibly before cycloidal.

## Reducer fit by joint (starting hypothesis, to be tested)

| Joint | Housing | Reducer first | Why |
|---|---|---|---|
| Arm J1-J6 | T-style, hollow | Strain wave | Short, large bore, near-zero backlash |
| Heavy arm axes, shock-loaded | T-style or inline | Cycloidal | Overload tolerance, stiffness |
| Humanoid hip/knee/ankle | Inline or pancake | Planetary QDD, or cycloidal | Backdrivability, impact tolerance; bore matters less |
| Wrist | T-style | Strain wave | Size, backlash |

## Phases

Each phase ends with something that runs and a stated check.

- **A0. Joint table.** Derive per-joint loads (peak, rated, speed, duty, shock) from reference
  robots: the cobot-arm reach classes and a humanoid leg/arm. Cluster into 3-5 size classes.
  *Check:* table reviewed; sizes cover the eRob80/110/142 and 70 classes in the arm.
- **A1. Interface and sizing.** Write `JointSpec`, the reducer interface and the sizing model
  against a placeholder reducer (ratio, efficiency, size, stiffness). Validate the motor model by
  reproducing the published data of reference modules (eRob80H100I; OpenArm's DAMIAO DM8009P as a
  second benchmark; vendor figures are unverified). *Check:* predicted torque, mass and speed
  within a stated tolerance of the references.
- **A2. Housing and motor generators.** T-style and inline housing with bore, frameless motor,
  encoder, brake keep-outs, cable path, CadKit output. *Check:* assembly builds for two sizes;
  clearances and bore checks pass; motor profile loads in the drive host tool.
- **A3. Strain-wave generator.** Land `TOOTH_DESIGN_PLAN.md` (the dependency), then wrap it as a
  `ReducerGenerator`. *Check:* a flexspline/circular-spline pair for the eRob80-class size meshes in
  the analysis; hobber target and hob produced from the same record.
- **A4. Cycloidal generator.** Disc profile, pins, balance, loads, hollow-eccentric feasibility.
  Prototype by wire-EDM outsourcing. *Check:* profile and ratio verified numerically; hollow variant
  reports its cost; the drawing set can be quoted.
- **A4b. Planetary generator.** Gear-train layout, planet load sharing, bearings, efficiency, hollow
  layout cost. Reuses the involute gearkit code and hobbing. May precede A4 as the easiest second
  user of the reducer interface. *Check:* a 6-10:1 stage for a humanoid-leg joint fits the inline
  or pancake housing; load sharing and efficiency reported.
- **A5. Actuator-level model in robotkit.** Torque, inertia, efficiency, thermal and limits feed
  `TRANSMISSION_PLAN.md` couplings and the cobot-arm. *Check:* cobot-arm reach classes run on
  generated actuators in place of the bought ones.
- **A6. Dyno and first prototypes.** Build the dyno (torque, speed, efficiency map, thermal, backlash,
  stiffness, hysteresis, friction), reusing the rotary-axis bench methods. Build one size with each
  reducer in the same frame. Measured values replace model values. *Check:* both reducers
  compared on one dyno and the model corrected.
- **A7. Custom ring PCB** from the findings: dual encoder inputs, brake driver, STO, thermal
  coupling, bore clearance. Co-designed with `materia-internal/drive`.

## Decisions to make

1. **Mechanical compatibility with the ZeroErr T modules.** Same flange and bolt pattern lets our
   actuator drop into the cobot-arm models and be compared with the bought parts; the cost is
   inheriting their envelope. Recommendation: define `eRobCompatible` as one named `mountPattern`
   and also keep our own.
2. **Make vs buy** per component for the first prototypes: motor stator/rotor, flexspline blank and
   steel, wave-generator bearing, cross-roller bearing, encoder, brake. Recommendation: buy all but
   the reducer and housing at first.
3. **Bus voltage and current class** (24/48/60 V). Decided by A0 and A1, not assumed now.
4. **Whether cycloidal is a product line** or the prototype/shock-load option. Revisit at A4.
5. **Housing style model**: fixed list (inline, T-style first) or composable body (main body plus
   perpendicular faces and offsets). Check what the structural casting recipes and cobot-arm module
   code can express before choosing.
6. **Housing construction**: machined first, cast later; decide when a design has settled.
7. **Metrology for non-involute profiles**: extend `GearMeasurement` or add a separate profile
   inspection. Decide when A3 needs it.

## Risks

- The strain-wave tooth design is the critical path and is a plan today. Without it the hobber can
  cut only involute flexsplines.
- A single shared parameter set creeping toward one reducer. Mitigation: the opaque `design` rule.
- Model values taken as facts. Mitigation: label model vs measured everywhere; benchmark vendor
  figures are unverified until read from their datasheets.
- Hollow cycloidal may not reach a useful bore at arm-joint diameters.
- Disk space and other sessions: Codex sessions work in `materia-internal`; this plan only touches
  `materia`.

## References

- `machinekit/STRAIN_WAVE_RESEARCH.md`, `gearkit/TOOTH_DESIGN_PLAN.md`, `gearkit/METROLOGY.md`
- `machinekit/PRECISION_ROTARY_PLAN.md`, `machinekit/TRANSMISSION_PLAN.md`
- `machinekit/examples/flexspline-hobber/`, `examples/circular-spline-skiver/PLAN.md`
- `machinekit/examples/cobot-arm/` (`README.md`, `actuator-selection.json`, `reference-sources.json`)
- `materia-internal/drive/docs/requirements.md` and `docs/adr/0005-product-and-platform-decisions.md`
- Benchmarks (unverified, from resellers): OpenArm 2.0 / DAMIAO actuators, ZeroErr eRob
- Other actuators to compare (planetary QDD): MIT Mini Cheetah actuator, Unitree and Robstride-class
  leg actuators; DAMIAO range (DM-J4310, J4340, J6006, J8009, J10010), figures from resellers and
  unverified
- Open designs to read: Faze4 (cycloidal arm), OpenMonopedal (cycloidal leg actuator), Open HD and
  Compact Strain Wave Gear Actuators (printed strain wave)
