# 3D printing plan

**Goal.** Materia designs, slices, simulates and runs an FDM print on a CoreXY printer, and the
same printer and job can also be handed to Klipper.
- **Our stack.** A CoreXY printer is modelled in MachineKit, like the CNC router. A part made in
  CadKit is sliced into a ToolpathKit program. MotionKit plans it with the extruder as a planned
  joint, and RobotKit runs it on the simulated printer.
  - The controller holds the hotend and bed at temperature, with thermal runaway protection.
  - The printed part grows in StockKit from the nozzle's actual motion, and is measured against
    the CAD part.
- **Klipper.** The same assembly generates a Klipper `printer.cfg`, and the same job is written as
  Klipper G-code. Materia uploads it through Moonraker, starts it, and mirrors the printer's live
  state into the digital twin.

Both targets come from one model and one job. Neither is hand-written.

The printer example lives in `machinekit/examples/printer/`. The work touches ToolpathKit, CncKit,
CadKit, CamKit, MotionKit, TrajectoryKit, RobotKit (runtime and device), StockKit, ProcessKit,
ProjectKit and the app.

## What exists

Surveyed 2026-10-09 on main 1decbfc20.

**Program pipeline**
- `toolpathkit.path.ToolpathOp` has these ops: `Move`/`MachineMove(kind, geometry, feed, tolerance,
  provenance)`, `Dwell`, `Spindle`, `Coolant`, `ToolChange`, `ToolLengthOffset`, stops, `End` and
  `SetSetup`.
  - Geometry is `Line`/`Arc`/`Circular`, in metres. There is no extra axis and no process value.
- `cnckit` parses LinuxCNC only (`CncDialect { LinuxCnc }`). Comments are dropped in the lexer.
  - It rejects `A/B/C/E` words, `G92`, and every M-code outside 0–9 and 30.
  - `CncWriter` is the LinuxCNC post.
- `toolpathkit.motion.ToolpathLowering` joins moves into one `FollowPath` until a barrier.
  - It rounds corners only between equal speeds, through `CornerBlender.blendPerCorner`, with a
    maximum turn of 150°.
  - Spindle and coolant become `PathEvent`s on channels (`ToolpathChannels`), and spindle-on
    waits on `spindle.at_speed`.
  - `ToolpathMotionBinding` throws if any non-fixed joint is not bound to X/Y/Z.
- `motionkit.robot.ProgramCompiler` splits a `FollowPath` at every tangent discontinuity and
  times each section from rest to rest. It calls `PathTimingLimits(..., 0, 0, ...)`, so the
  `startPathSpeed`/`endPathSpeed` fields exist but go unused.
  - `CornerBlender` gives up and exact-stops on arcs, non-planar (Z-changing) lines, turns of
    150° or more, and lines too short to blend.
  - Each op or section has a 10000-sample budget.
  - `pathEventSchedule` is a hook after timing. Welding uses it with
    `processkit.ProcessRateSchedule` to make wire feed exact per distance.
  - `ProgramPlanner` plans on a worker and streams through `TrajectoryStream`.
- `app/src/CncProgramPlayer.hx` runs G-code: `CncCompiler`, `ToolpathMotion.lower`,
  `ManipulatorMotion` and `ProgramPlanner`.
  - It homes first, with `homingSeconds` counted separately.
  - Its job is `SceneArtifact.machining`, which has three mandatory axes and cutter-specific
    tools.
  - `MachiningStock` cuts StockKit stock from the actual simulated tool tip. `CncPanel` provides
    hold, resume, restart and override.
  - `MachiningRun.continuationFrom` replays only the four spindle/coolant channels.

**Machine**
- `machinekit/examples/corexy/CoreXyPlotter` is a CoreXY pen plotter:
  - two GT2 belt loops, 20T pulleys, `MGN12C` rails and NEMA 17 motors;
  - `GENERIC-TMC2209` drivers and a 24 V supply;
  - couplings with several leaders, from `AxisBuilder.turnWithBelt`.
  - It has no Z, no switches and no homing, and travels only ±35 mm.
  - `ProjectSourceTests` (`PROJECT_SOURCE_ONLY=corexy`) checks 16 couplings and prints
    `axes to 649.6 mm/s`.
- `machinekit.gantry.Gantry` has no CoreXY option.
- Parts that exist:
  - `NemaStepper` frames 17/23/34;
  - rails `MGN12C` and `HGR15` only;
  - `TimingBelt`/`TimingPulley` GT2;
  - `LeadScrew` (Tr/Acme/Ball) and a T8-proportioned `LeadScrewNut`;
  - `MotorDriver`, `PowerSupply`, `LimitSwitch`, `ProximitySwitch` (inductive M8/M12);
  - `TSlotExtrusion`;
  - `SpurGear`, `GearPair`.
  - There are no heaters, thermistors, hotends, extruders, fans or filament.
- Homing is in place: `HomingCycle`, `HomingSequence`, `HomingDynamics` and
  `RuntimeHomingDriver`, plus device homing scopes.
  - Switches come from `MachineAssembly.addTripSwitch` and `homeAfter`.
  - `processkit.perception.ContactSearch` is a bounded "move until contact" primitive used by
    weld touch sensing.

**Process, device and simulation**
- `ProcessDevice` (prepare/ready/fault/safe/apply) has two implementations,
  `ChannelProcessDevice` and `WelderProcessDevice`.
- Channel kinds are Digital, Analog and Process. Events are scheduled in path time, with
  per-channel safe values.
- Device `Board` trait (`robotkit/device_protocol/src/board.rs`): digital inputs; digital, analog
  and process outputs; steps. It has no analog input and no PWM. Sensors go to the host as
  `Sensor6` slots.
- No closed loop on a non-motion channel exists anywhere.
  - The precedent is `robotkit/device_protocol/boards/welder_retrofit.rs`: an IO trait (DAC out,
    ADC in), `tick` with time-based fault latches, and `safe`/`reset`.
  - Its virtual counterpart is `robotkit/device_virtual/src/welder.rs`.
- The simulation is mechanical only.
  - Pneumatics are force-only.
  - Spindles are velocity drives in `robotkit/runtime/src/simulation_robot.cpp`.
  - Non-mechanical models hook in through `SimulationStepObserver` (`SimulatedWelder`).
- `processkit.tool.WeldBead` deposits `wireSpeed·A·efficiency/v` from the actual simulated tip.
  `app/src/WeldBeads.hx` renders it as chunked runtime geometry.

**Stock and CAD**
- StockKit only subtracts. Its C API has `sk_stock_cut` and no add.
  - `sk_sweep_ray` returns exact swept intervals for an axisymmetric profile on 3-axis moves, so
    a stadium "bead" profile is expressible today.
  - `StockPreviewWorker` contours changed chunks on a worker.
- CadKit has no planar section call.
  - It does have `cad_wire_offset` (`BRepOffsetAPI_MakeOffset`, via `Curve.offset`),
    `Part.intersect`, `CamContour.fromFaceBoundaries` and `cad_shape_tessellate`.
- CamKit:
  - `CamPocketPlanner.plan(contour, radius, stepOver, islands)` is a scanline zigzag fill with
    islands, reusable for rectilinear infill.
  - `CamContour.inset` is convex-only.
- `toolpathkit.path.ArcFitting.fit(ops, tolerance)` exists.
- There is no Clipper, and no general polygon offset or boolean.

**Klipper side**
- `robotkit/remote` `NativeTransport` provides TCP connect and listen, with libwebsockets in
  NativeKit.
- No Klipper or Moonraker code exists. Klipper and Moonraker are GPLv3: we use only their
  documented config format and HTTP API, and copy no code.

## What is missing

1. Extrusion, heaters, fans and layer/feature/object markers in the toolpath IR.
2. A printer G-code dialect (Marlin/Klipper words, `E`, `G92 E`, `M82`/`M83`, `M104`/`M109`/
   `M140`/`M190`/`M106`/`M107`, `G10`/`G11`, comments as markers) and a printer writer.
3. A slicer: planar sections, perimeters, skins, infill, travel and retraction.
4. Smooth cornering at slicer density: blends that do not give up on short, non-planar or arc
   segments, and timing windows that carry speed across their boundaries instead of stopping.
5. A printer: CoreXY with Z, an extruder, a hotend, a heated bed, fans, switches and a probe.
6. An extruder joint planned from path distance, a flow limit, retraction and pressure advance.
7. Heater control on the controller, a thermal plant model, and temperature barriers and faults.
8. Additive stock, a deposition preview, and dimensional checks against the CAD part.
9. A print job in the scene artifact, a player and a panel.
10. A generated Klipper config, Klipper G-code, and a Moonraker client.
11. Input shaping.

## Decisions

**AM-D1. One model, two execution targets.**
- The printer assembly and the print job are the only sources.
- Our stack executes them on the simulated printer (and, later, on RKD devices).
- The Klipper exporter derives `printer.cfg` and G-code from the same assembly and job.
- Nothing about the printer is typed twice.
- Rejected: a separate "Klipper printer profile" data model.

**AM-D2. Extrusion is a process quantity along the path, not an axis.**
- New modal ops in `ToolpathOp`:
  - `Deposit(area, width, height, provenance)`: every following `Cut` move lays a bead of that
    cross-section area (m²) until the next `Deposit` or `DepositOff`. Travel kinds
    (`Rapid`/`Link`/`Retract`) never deposit.
  - `DepositOff(provenance)`.
  - `FilamentMove(length, speed, provenance)`: an extruder-only move, in metres of filament.
    Positive primes, negative retracts. It is used for retract/unretract and purge.
  - `Heat(heater, celsius, wait, provenance)`, where `heater` is a name such as `hotend` or `bed`.
  - `Fan(fan, duty, provenance)`.
  - `Layer(index, z, provenance)`, `Feature(kind, provenance)` (perimeter, external perimeter,
    infill, solid infill, skirt, travel…) and `PrintObject(name, start, provenance)`. These are
    markers for preview, restart and cancel-object; lowering ignores them.
- The IR stores bead area, which is machine-independent. Filament diameter belongs to the
  material and the extruder, so importing or writing G-code converts `E` with it.
- `Move` keeps its fields. Modal ops keep the destructuring consumers (about 15 non-test files,
  listed by the survey) unchanged, which is how `Spindle`/`Coolant` work.
- Rejected:
  - an `E` coordinate on `Move` or `Point3` (filament length is not a pose coordinate, and it
    ties the IR to one filament size);
  - a 4th axis in `MachineBinding`.

**AM-D3. The extruder is a planned joint driven by path distance.**
- The filament drive gear is a revolute joint in the robot model. A stepper drives it through a
  gear reduction (`Transmission.GearMesh`).
- Along each deposited primitive its angle is linear in path distance:
  `θ(s) = θ0 + area / filamentArea / gearRadius · s`. `FilamentMove`s are extruder-only joint
  moves.
- `ProgramCompiler` adds these "distance-driven joint" columns to the joint path samples after
  IK. Timing then respects the extruder drive's speed and acceleration limits like any joint.
- The X6 plan check, stepper slip, step rates, encoders and device streaming all apply unchanged.
- `ToolpathMotionBinding` binds the extruder joint with an explicit role instead of throwing.
- Melt rate is a separate limit. The hotend's maximum volumetric flow (assumed per hotend part)
  caps each primitive's speed at `maxFlow / area`, through the per-sample caps.
- Pressure advance is a joint-path term: `θ_cmd = θ(s(t)) + K · θ̇`, smoothed over a window
  (Klipper's default is 0.04 s) so the extruder stays within its limits. The plan check verifies
  it.
- Rejected: an analog rate channel through `ProcessRateSchedule`, the welder wire-feed pattern.
  - It is right for a feeder with its own controller.
  - It is wrong for a stepper on the motion controller: piecewise-constant rates lose position,
    retraction is a position move, and pressure advance needs the joint's position.

**AM-D4. Cornering: smooth paths and timing windows with boundary speeds; no velocity jumps.**
- TrajectoryKit, the runtime and the devices assume continuous joint velocity. We keep that,
  because robots and CNC need it.
- Instead of Klipper-style junction velocity jumps, the path is made C1 before timing:
  - `CornerBlender` blends every corner within the move tolerance, with a blend length limited
    to a share of each neighbouring segment.
  - Blending handles short lines, non-planar lines, and line–arc and arc–arc corners.
  - Exact stops remain only at near-reversals (above `maxBlendTurnAngleRadians`), at process
    barriers, and at extruder-only moves.
- Long smooth paths are timed in windows, not as one section. Each window's end speed is
  bounded so that the next window could still stop (`startPathSpeed`/`endPathSpeed` in
  `PathTimingLimits`). That keeps the sample budget, memory and lookahead bounded.
- Slicer and G-code polylines may first be arc-fitted (`ArcFitting`) within tolerance.
- These rules apply to CNC programs too.
  - Router and bench mill numbers may change (fewer stops). That is acceptable only if their
    gouge, deviation, removal and stall checks still pass. Record the new numbers.
- Rejected: velocity discontinuities at corners hidden by input shaping (Klipper's
  `square_corner_velocity`). They would break trajectory continuity across the whole stack.

**AM-D5. Slicing is exact, from the B-rep.**
- CadKit gains native calls:
  - a planar section of a solid at height z, giving planar faces with holes
    (`BRepAlgoAPI_Section`, wires assembled into faces);
  - offset of a planar face with all its wires (`BRepOffsetAPI_MakeOffset` on the face, so
    collisions and splits are handled);
  - booleans between planar faces (cut, common, fuse), for top/bottom skin regions.
- The slicer lives in CamKit as `camkit.additive`, because CamKit is "CAD to ToolpathProgram".
  It reuses `CamContour` sampling, `CamPocketPlanner` scanlines for infill, and `ArcFitting`.
- Rejected:
  - slicing a tessellation (it loses exact arcs; mesh input such as STL/3MF is a later step);
  - vendoring Clipper2 (no network here, and OCCT already does exact 2D offsets and booleans).

**AM-D6. Heaters are controlled on the controller.**
- A `heater` module in `robotkit/device_protocol` (shared, `no_std`) holds:
  - PWM duty out and thermistor ADC in (Beta model, 100k NTC 3950);
  - PID;
  - a runaway watchdog that latches a fault: heating gain too low over a window, leaving a
    hysteresis band once at target, over-temperature, and sensor open or short.
  - Its pattern is `welder_retrofit.rs`.
- The virtual device runs the same module against a lumped thermal plant in
  `robotkit/device_virtual`: `C·dT/dt = P·duty − h·(T − T_amb)`, with part-fan cooling on the
  hotend. Values are assumed per part.
- The host sends setpoints as Analog channel events and reads temperature and state from
  `Sensor6` slots. `heater.<name>.at_temperature` answers the `Heat(wait)` barrier, the same way
  `spindle.at_speed` does.
- Cold extrusion is guarded twice:
  - a static program check (no deposition or positive `FilamentMove` before a waited `Heat` at or
    above the material's minimum);
  - a device fault if the hotend falls below the minimum while the extruder joint moves forward.
- The printer example always runs with `virtualDevice = true`, so the controller logic is the
  device's.
- Rejected: host-side PID (a host stall would leave the heater on).

**AM-D7. What was printed is measured, not assumed.**
- Deposition comes from the actual simulated nozzle tip and the actual extruder joint motion,
  like `MachiningStock` and `WeldBead`. Slip, flow limits and pressure-advance errors therefore
  show.
- StockKit gets `sk_stock_add`, a union of swept spans with provenance, starting from empty
  stock. The bead is a revolved stadium profile of the actual layer height and the width that
  conserves volume.
- One representation serves both: `StockPreviewWorker` contours it for the preview, and the
  checks compare it with the CAD part (volume, bounding box, hole diameter).
- Rejected: separate tube meshes for the preview (a second geometry that can disagree with the
  measured one).

**AM-D8. The printer.**
- CoreXY over a 2020 T-slot frame, about 220 × 220 × 220 mm build volume:
  - two NEMA 17 motors, GT2 20T, 6 mm belts;
  - MGN12 rails;
  - a moving bed on three T8 lead screws, each with its own NEMA 17. The three are followers of
    one `z` joint today; independent Z tilt comes later.
- Toolhead:
  - a geared extruder (about 3:1, hobbed drive gear about 7.3 mm, pancake NEMA 17);
  - a hotend with a 0.4 mm nozzle, heater block, 40 W 24 V cartridge, 100k NTC, heatsink and
    hotend fan;
  - a 5015 part-cooling blower.
- Bed: aluminium plate with a 24 V silicone heater and a 100k NTC.
- Homing: X/Y limit switches, and an inductive M8 probe for Z homing at the bed centre.
- Electrics: 24 V supply, six TMC2209-class drivers, and a controller declared as the router
  job does (microsteps, step tick).
- New library parts go in `machinekit.printing`: `Hotend`, `HeaterCartridge`, `Thermistor`,
  `Nozzle`, `Extruder` (drive gear, idler, reduction), `HeatedBed`, `Fan`/`Blower`.
  - New rail rows (MGN9, MGN12H) and a pancake NEMA 17 row are added where missing.
  - Values that cannot be checked are marked assumed, as the existing `ServoMotor` rows are.
- The `CoreXyPlotter` coupling code is reused, not copied: lift what the printer needs into
  `AxisBuilder` or a small CoreXY helper.

**AM-D9. The Klipper target is generated and tested offline.**
- `printer.cfg` is derived from the assembly:
  - `[printer] kinematics: corexy`, with `max_velocity`/`max_accel` from `coupledLimits`;
  - for each stepper: `rotation_distance` from pulley teeth × pitch or screw lead, `microsteps`
    from the driver, `position_min/max/endstop` from axes, overtravel and switches, and homing
    speeds from `HomingDynamics`;
  - `[extruder]`: `rotation_distance`/`gear_ratio` from the drive gear and reduction,
    `nozzle_diameter`, `filament_diameter`, heater and sensor types, `max_extrude_cross_section`;
  - `[heater_bed]`, `[fan]`, `[probe]` with offsets, `[tmc2209 …] run_current` from the driver;
  - `[input_shaper]` when the model has a value.
- MCU pins come from the controller's wiring when the board is a known Klipper board. Otherwise
  each pin is a clearly marked placeholder in one alias section, and a check lists the
  placeholders.
- The Klipper G-code writer uses relative E (`M83`), `EXCLUDE_OBJECT_DEFINE`/`START`/`END`, and
  heater waits.
- The Moonraker client uses `NativeTransport` TCP with HTTP/1.1 (and the WebSocket subscription
  if it is cheap). It covers upload, start, pause, resume, cancel and a status query
  (`print_stats`, `toolhead`, `extruder`, `heater_bed`), and mirrors toolhead position and
  temperatures into the twin.
- Tests run against an in-process fake Moonraker server only. Nothing touches the network or a
  real printer.

**AM-D10. Input shaping is a trajectory filter.**
- A ZV/MZV/EI shaper convolves the planned toolhead trajectory with impulses. It is applied in
  TrajectoryKit/MotionKit after timing and before submission.
- For linear couplings (CoreXY), shaping the motor joints is equivalent to shaping X/Y.
- Validated in simulation with a compliant belt model: the residual vibration after a step at
  the modelled resonance drops by at least 5×.
- Shaper frequency and damping are machine data. Measuring them (accelerometer, chirp) comes
  later.

## Test parts

All are built in code, so tests need no files.
- `CalibrationCube`: 20 mm cube with a 6 mm through-hole. Used for dimensional checks.
- `TestTower`: 20 × 20 × 12 mm. It has a boss, a counterbored hole, a 45° chamfer and a
  cylindrical wall, which exercises arcs, islands and skins.
- `SmallTab`: 10 × 10 × 1.2 mm, six layers at 0.2 mm. It is the fast app test.
- Benchmark load: a `SpurGear` from MachineKit, about 60 mm, 40 teeth, 8 mm thick. Its involute
  teeth give slicer-like short segments. Add a cylinder if more segments are needed.
- `NemaMountBracket`: a printable L bracket for a NEMA 17, built from MachineKit companions.
  This is the demo print.

## Steps

**AM0. Base.**
- Worktree `/home/joao/dev/materia-worktrees/3d-printing`, branch `3d-printing` from main
  1decbfc20, with every submodule populated. `coal` and `proxsuite` are symlinks (they show as
  `T`).
- The baseline suite results are recorded in the session prompt.

**AM0b. Make main green.** Done. Seven failures were already on main 1decbfc20; each was diagnosed
before it was fixed, and none was fixed by loosening a check.
- **Step rate halved (MachineKit smoke and workspace, app screw and belt routers).** Not a code
  intent: 8c9e1d026 plans at the integer pulse interval of the f32 rate and steps per unit the
  device receives. A rate exactly at the step tick's ceiling can round below one tick per step in
  f32, so the interval came out as two and the CoreXY plotter and both routers were planned at half
  speed. The device derives the same interval from the same f32 values, so it would have run at
  half speed too. `PulseRate` (in `DeviceBinding.hx`) now derives, once, the rate a planner may
  use and the rate to deploy: a ratio within f32 rounding of an integer is that integer, and a
  deployed rate is nudged to the nearest f32 that gives the planned interval. 2fbea0683.
- **`testVirtualCncProgram` ("exceeds claimed limits: position on joint 0 at 0 s").** Caused by
  efd8c5c16 (confirmed by reverting it): homed axes return to their home coordinate, which defaults
  to the lower travel limit. A stepper cannot rest exactly on a limit; the pulse generator floors
  its target, so the device rested one or two steps (12.5 um) below zero, within the homing
  precision but outside the claimed bound, and four layers refused the first plan. A joint outside a
  claimed position bound by no more than the plan's start tolerance now begins there, with its
  bound reaching the start and no further, in each place that checks a plan against joint
  limits: `ExecutionPlan.create`, `ProgramCompiler`'s path check, the runtime's appended-path
  validation and the RKD6 device compiler. Rejected: returning homed axes a precision inside the
  limits (path planners start at the home coordinate and expect to find the axis there; motionkit's
  `PlannerTests` fail). ae6e95ec5.
- **`DLS allocates nothing per iteration (-40.8 bytes)`.** A measurement artefact: HashLink bills
  allocation in blocks, so a run is now and then charged a 2 KB refill. Every run allocated 752
  bytes whatever its length. The test takes the cheapest of six runs per length. cd29b1993.
- **processkit and robotkit-core tests ("libnativekit_scene.so" / "libnativekit.so" not found).**
  Neither test project built the NativeKit libraries it loads. processkit's declares the native
  build of the motionkit tests; the robotkit-core consumer builds NativeKit beside the runtime.
  dfa6e2940.
- **Editor tests ("Worker document tests failed: ... Could not open source file").**
  `StartPageTests` moved the process to the repository root and never came back, so the worker
  tests' `fixtures/worker-legacy.materia` resolved against the root. It restores the working
  directory now. 2611087d9.

Gate `am-suite.sh am0b` (full, 2395 s): every project passes. One workspace action, `toolpathkit`,
was blocked because this session edited its sources while the suite ran; it passes alone (97
assertions) on the committed tree. Assertion counts: RobotKit world 5439 (was 5433), terrain 3524,
MotionKit 1,224,712, CadBridge 173, CncKit 317, CamKit 12311, ProjectKit 152, ToolpathKit 97, KinematicsKit
325 + native 695, automationkit 86, manufacturingkit 54. Every number listed as not to move is
unchanged. The two routers now have numbers (the new baseline for every later step):
- **Screw router:** axes y 22.3 mm/s 5.21 m/s^2, x 25 mm/s 5.37, z 25 mm/s 5.65; plate milled in
  232.2 s of machining (homing 38 s, total 270.2 s), 9902.1 of 9996.5 mm^3 removed, leftover 140.3
  mm^3, gouge 63.9 mm^3; 128 plans, 0 flagged, 0 stalls, worst deviation 0.05 mm.
- **Belt router:** axes y and x 500 mm/s 12.29 and 14.1 m/s^2; plate milled in 215.2 s (homing
  26.2 s, total 241.4 s), same removal; 128 plans, 56 flagged, 62 over the 0.1 mm tolerance, 0
  stalls, worst deviation 1.76 mm on y (assumed belt drag).
  (The "last known good" values, 220.2 s and 201.6 s, predate the controller step-rate ceilings.)
- **CoreXY plotter:** axes to 649.6 mm/s, 71.549 m/s^2 (as before).

**AM1. Printer ops in the IR.**
- Add the AM-D2 ops to `ToolpathOp`, with validation:
  - area, width and height positive and consistent (`area ≤ width·height`);
  - a `FilamentMove` speed is required;
  - heater and fan names are non-empty;
  - fan duty is in [0, 1].
- Update every exhaustive `switch` on `ToolpathOp`:
  - `ArcFitting` and `ToolpathFrame` pass the new ops through;
  - `TravelEnvelope` and `Setup` ignore them;
  - `CncWriter` refuses them with a diagnostic (LinuxCNC has no extruder);
  - `stockkit.CutMoves` ignores them;
  - `ToolpathLowering` refuses them for now with "needs a printer binding" (AM8 lowers them).
- A `ToolpathProgram` query: total deposited volume, per-layer volume and per-object volume.
- Tests: validation, pass-through, volume queries.

Done. What was built and decided:
- `ToolpathOp` gained `Deposit(area, width, height)`, `DepositOff`, `FilamentMove(length, speed)`,
  `Heat(heater, celsius, wait)`, `Fan(fan, duty)` and the markers `Layer(index, z)`, `Feature(kind)` and
  `PrintObject(name, start)`, as AM-D2 says. `Move` is unchanged, so no consumer that destructures it
  changed. `PrintFeature` is the marker vocabulary (`ExternalPerimeter`, `Perimeter`, `Infill`,
  `SolidInfill`, `TopSolidInfill`, `Skirt`, `Travel`, `Other(name)` for a slicer's own label).
- Validation lives in `PrinterOps.validate`, run by the `ToolpathProgram` constructor with the op's
  index in the message: area, width and height positive and finite with `area <= width * height`; a
  nonzero `FilamentMove` length at a positive finite speed; non-empty heater, fan and object names;
  finite non-negative temperatures; fan duty in [0, 1]; a non-negative layer index.
- `ArcFitting`, `ToolpathFrame`, `TravelEnvelope`, `Setup` and `stockkit.CutMoves` already fell
  through unknown ops, and pass the new ones through in place (tested). `CncWriter` refuses a program
  holding any printer op before writing, naming its line ("LinuxCNC has no extruder, heater or
  fan"); `CncCompensator` and the app's `StockSimulationSession` list the new ops where they were
  exhaustive. `ToolpathLowering` ignores the markers and refuses the rest ("printing operations need
  a printer binding"), so a printer program cannot be half-run on a CNC machine; AM8 lowers them.
- `ToolpathProgram.depositedVolume()`, `layerVolumes()` and `objectVolumes()`: bead area times
  length of each `Cut` move (`Cut` only: `Rapid`, `Link` and `Retract` never deposit) under the
  `Deposit` in force, attributed to the current `Layer` (index -1 before the first marker) and
  `PrintObject`.
- Tests: ToolpathKit 97 -> 141 assertions (`PrinterOpCoverage`: validation, pass-through, volumes);
  CncKit 317 -> 325 (writer refusal); toolpathkit/motion (lowering refusal and markers). Quick suite
  `am1`: every project passes; the app project-source build compiles.

**AM2. Printer G-code dialect and writer.**
- `CncDialect` gains `Printer` (Marlin/Klipper/RepRap family). The lexer keeps comments as
  data in this dialect, so `;LAYER_CHANGE`, `;TYPE:`, `;LAYER:` and `M117` text are not errors.
- Words and codes:
  - `E` on `G0`/`G1`/`G2`/`G3`; `G92` (E only, with a diagnostic for XYZ);
  - `M82`/`M83`; `G10`/`G11` with firmware-retract settings from the controller;
  - `M104`/`M109`/`M140`/`M190` (`S`, and `R` where supported);
  - `M106`/`M107`; `M220`/`M221` as overrides; `M84` and `M201`–`M205` (diagnostic or ignored,
    documented);
  - `G28` as "require homed";
  - PrusaSlicer/Orca/Cura feature comments as `Feature`/`Layer` markers;
  - `EXCLUDE_OBJECT_DEFINE`/`START`/`END` and `M486` as `PrintObject`.
- `E` deltas convert to `Deposit` area through the controller's filament diameter:
  `area = ΔE · filamentArea / Δs`.
  - A new `Deposit` is emitted when the ratio changes beyond a tolerance (0.5 %).
  - An E-only move becomes a `FilamentMove`.
  - An E move with XY whose ratio is negative (retract while moving, "wipe") becomes a
    `FilamentMove` plus travel, with a warning.
- `PrinterWriter` writes the IR back as printer G-code, with Klipper and Marlin flavours (only
  the object markers and start sequence differ).
- Tests:
  - round trip IR → G-code → IR preserves deposited volume within 0.1 % and geometry within
    1 µm;
  - hand-written snippets in the style of PrusaSlicer, Cura and Orca parse;
  - LinuxCNC behaviour is unchanged (CncKit assertion count only grows).

Done. What was built and decided:
- `CncDialect.Printer`. A printer `CncController` carries `PrinterSettings` (filament diameter, the
  firmware retract for `G10`/`G11`, the ratio tolerance, the corner tolerance given to deposited
  moves, whether the controller draws arcs); a LinuxCNC controller has none. `CncCompiler` hands a
  printer controller to `PrinterInterpreter`; the LinuxCNC interpreter is untouched.
- `CncLexer.parse(source, Printer)` keeps comments as data (`CncComment`), reads string words
  (`A"name"`), ignores line numbers and checksums, takes the rest of `M117`/`M118` as a message and
  reads Klipper extended commands (`NAME KEY=value ...`).
- `PrinterInterpreter`: `E` is delta-converted (`M82`/`M83`, `G92 E`) and becomes
  `Deposit(area, width, height)` with `area = dE * filamentArea / distance` (the arc length for
  `G2`/`G3`), restarting when the ratio moves by more than 0.5 %; the bead is a layer tall and as wide
  as a stadium of that area, so area <= width * height always holds. E-only moves and `G10`/`G11` are
  `FilamentMove`s; a retract while moving becomes a `FilamentMove` plus a travel, with a warning.
  `M104`/`M109`/`M140`/`M190`/`M141`/`M191` become `Heat`, `M106`/`M107` `Fan` (fan 0 is `part`,
  fan n is `fan<n>`), `M220`/`M221` scale the speed and the extrusion of the moves after them, `G4`
  a `Dwell`, `M0`/`M1`/`M2` stops. `G28` only sets `CncCompileResult.homes`. Motion limits,
  stored settings and macros are `CNC_PRINTER_IGNORED` warnings, once each; unknown codes and
  `G92` on an axis are errors on their line and the rest still compiles.
- Markers: `;LAYER_CHANGE` + `;Z:`/`;HEIGHT:` (Prusa, Orca), `;LAYER:` (Cura, the height is the first
  move's Z), `;TYPE:`/`; FEATURE:` (all three slicers' names), `;MESH:`, `; printing object`,
  `EXCLUDE_OBJECT_START/END` and `M486`. `TEMPERATURE_WAIT` after `SET_HEATER_TEMPERATURE` is the
  wait of that `Heat`.
- Rapid travel carries its speed: a G0/G1 without extrusion is `Move(Rapid, ..., speed)`, so the
  slicer's travel speed survives. A CNC `Rapid` has speed 0 and keeps the machine's rapid speed.
  AM8's lowering uses the speed when it is positive.
- `PrinterWriter` (Klipper and Marlin flavours) writes `G21 G90 M83 G28`, the moves with
  `E = area * length / filamentArea`, `Layer`/`Feature` as Prusa-style comments, `EXCLUDE_OBJECT_DEFINE`
  up front and `_START`/`_END` (Klipper) or `M486` (Marlin), `SET_HEATER_TEMPERATURE` and
  `SET_FAN_SPEED` for generic parts (Klipper only). Controllers without arcs get chords no more than
  2 um off the arc.
- Tests: CncKit 325 -> 432 assertions (`PrinterGcodeTests`): PrusaSlicer, Cura and Orca snippets;
  M220/M221, E-only, G10/G11, wipe, G92, G20; refusals; IR -> G-code -> IR keeps deposited volume
  within 0.1 %, per-layer and per-object volumes, geometry within 1 um, markers, temperatures and
  fans in order; writing what was read is a fixed point. LinuxCNC fixtures are unchanged.

**AM3. Slicer v1.**
- CadKit native (AM-D5): section, planar face offset, planar face booleans. Also Haxe wrappers
  and HXI regeneration through `cadkit/scripts/generate-haxeon-hxi`.
  - Build CadKit native inside the worktree with the prebuilt OCCT (see the session prompt).
- `camkit.additive.Slicer(part, settings)` returns a `ToolpathProgram`. Settings:
  - layer height and first-layer height;
  - line width (nozzle × 1.125 by default);
  - perimeters (2) and top/bottom layers (4);
  - infill density and angle (rectilinear, alternating 45°/135°);
  - speeds per feature;
  - travel speed;
  - retraction (length, speed, minimum travel);
  - Z hop off by default;
  - temperatures, fan by layer, skirt (1 loop, 3 mm).
- Per layer:
  - section;
  - perimeters by face offset, (k + ½)·width inward, from the outside in;
  - skin regions = this layer minus the union of the layers within n above (top) or below
    (bottom);
  - solid infill in the skin regions, sparse infill in the rest, through `CamPocketPlanner`
    scanlines with islands;
  - travel with retraction when a travel crosses a perimeter or exceeds the minimum;
  - arc fitting of perimeter polylines within 5 µm.
- The start and end sequences come from a printer profile (heat, home, purge line, then end
  with heaters and fans off, park).
- Tests:
  - the cube's deposited volume is within 3 % of its CAD volume;
  - no extrusion outside the part's footprint plus width/2;
  - the hole stays open in every layer;
  - layers are monotonic in Z;
  - deterministic output for the same input;
  - slicing time of each test part is printed.

**AM4. Planning benchmark (measure, no gate yet).**
- `motionkit/scripts/benchmark-printing` (or a test filter) runs one sliced job through
  lowering and `ProgramCompiler` with planning only (no simulation):
  - the gear benchmark load;
  - the cube;
  - one G-code file written by `PrinterWriter` and read back.
- The machine is the CoreXY printer's axes if AM6 is done; otherwise use a three-prismatic-joint
  stand-in with the same limits (650 mm/s, 5 m/s²), and state which.
- It reports:
  - moves;
  - planned print time;
  - planning wall time and the real-time ratio, overall and for the worst 10 s window;
  - peak memory;
  - number of exact stops by reason;
  - number of timing sections.
- A reference estimator in the benchmark gives a lower bound on time: trapezoids with Marlin
  junction deviation 0.013 mm at the same acceleration and speeds.
- Record all numbers in this plan. They are the "before" of AM5.

Done. What was built and decided (the "before" of AM5):
- **Benchmark** `machinekit/examples/printer/benchmark` (a project of its own, `haxeon run --project=machinekit/examples/printer/benchmark/haxeon.json`):
  slices the load (or reads G-code), strips everything but the moves, lowers it with the printer's work origin and
  `rapidsAtFeed`, and plans it with `ProgramCompiler` on the printer's x, y and z (no plan check, no
  simulation). `PRINTING_BENCHMARK_ONLY=gear|cube|gcode` picks a load, `PRINTING_BENCHMARK_DUMP=dir` writes the
  sliced G-code, `PRINTING_BENCHMARK_GCODE=file` plans a G-code file instead of slicing, `PRINTING_BENCHMARK_LAYERS=n`
  keeps the first n layers, `PRINTING_BENCHMARK_TRACE=1` prints every plan. The reference estimator
  (`PrintTimeEstimate`) is a look-ahead planner with Marlin's junction deviation (0.013 mm) at the printer axes'
  limits; it is a lower bound.
- Machine: the printer's planning model (`DeviceBinding.planningModel`, 40 kHz step tick), axes x and y 250 mm/s
  with 24.09 and 12.56 m/s^2, z 100 mm/s with 18.1 m/s^2, from the scene artifact. (The app test, which takes
  its physical parts from a project session, finds 30.03, 15.59 and 22.87 m/s^2: the masses differ, the cause is
  not chased.) Joint coordinates are relative to the assembly's initial pose, so the work origin is
  (-0.11, -0.11, -0.01) m.
- Lowering and kinematics changes the benchmark needed, all independent of printing ops: `MachineBinding.rapidsAtFeed`
  (travels run at their own feed and round corners), `ToolpathLoweringResult.stops` (exact stops by reason),
  `ToolpathMotionBinding` gives joints that follow several axes (CoreXY pulleys) limits that are the sum of their
  leaders' and takes `heldJoints` (the extruder and its motor hold still), `AxisKinematics` places such joints from
  their couplings.
- **Numbers before AM5** (debug CadKit, shared machine):

| Load | Moves | Planned | Reference | Planning | Real time (all / worst 10 s) | Sections |
|---|---|---|---|---|---|---|
| Calibration cube (100 layers) | 4841 | 929.1 s | 867.6 s (+7.1 %) | 889.8 s | 1.04x / 0.75x | 4042 (4040 stops at a speed change, 1 with no tolerance) |
| Test tower through G-code (60 layers) | 4617 | 506.7 s | 469.6 s (+7.9 %) | 511.6 s | 0.99x / 0.67x | 4271 (4269 speed changes, 1 non-planar line) |
| Gear, layer 1 only (G-code of the sliced gear) | 2135 | 366 s | 357.6 s (+2.3 %) | 436.0 s | 0.84x / 0.12x | 554 |
| Gear, whole (module 1.5, 40 teeth, 8 mm; 61933 moves, 81453 segments, sliced in 45 s) | 61933 | not finished | | more than 3000 s | | |

  Managed heap peaks: 493 MB (cube; 48 GB allocated while planning), 503 MB (tower), 332 MB (gear layer 1).
- **Where the time goes.** Planned time is within 8 % of the reference even with thousands of exact stops: the axes
  are agile (12 to 24 m/s^2) against print speeds of 30 to 150 mm/s, so a stop costs a few milliseconds. The cost is
  planning: about 220 ms for each plan of the cube, and it is quadratic in the length of a smooth section. The gear's
  perimeter loops (about 400 mm of path, 1000 spans) take 162 to 171 s each for 20 s of motion, while 14 ms plans
  take 10 ms. Per-sample work that scans every primitive of the authored geometry is the cause to confirm:
  `distanceToGeometry` in the task-space check scans the whole op's authored path for each of the 1 ms samples, and
  `primitiveSpeedAt` and `PosePathRefinement.at` search linearly too. The sliced gear also failed in TOPP-RA
  (`path.time failed with MotionKit error -6`, the fourth plan) where the same program read back from G-code
  (rounded to a micrometre) did not.

**AM5. Cornering and windowed timing (AM-D4).**
- `CornerBlender`: blend short lines (blend length limited to a share of each neighbour),
  non-planar lines, and line–arc and arc–arc corners. Exact stop only above the turn limit.
- `ProgramCompiler`: time long smooth paths in windows with boundary speeds, and keep the sample
  budget and memory bounded.
- Gate, on the gear benchmark load:
  - planning at least 2× faster than real time over the worst 10 s window;
  - planned time no more than 15 % above the reference estimator;
  - path deviation within tolerance;
  - no limit violations.
- Router and bench mill checks must still pass. Record their new numbers and the reason.
- If the gate is not met after a bounded effort (profile, and fix the top costs), record the
  numbers and the cause, and continue. The user decides whether our stack stays a printing
  target.

Done. What was built and decided (AM3 comes before AM6 in the run order):
- **CadKit native** (`cadkit.h`): `cad_shape_section` (the solid in common with a big plane face, so
  holes and islands come out as OCCT builds them), `cad_planar_offset`, `cad_planar_cut/common/fuse` and
  `cad_planar_copy_boundary_bytes` (every boundary in one call: lines and circular arcs exact, other curves
  cut into lines within a tolerance). HXI regenerated; `cadkit.Shape` and the new `cadkit.PlanarRegion`
  (loops oriented outer counter-clockwise, holes clockwise) wrap them. `TKShHealing` is linked.
  CadKit was built into the worktree's `cadkit/build/debug` with the prebuilt OCCT; tested natively
  (`planar_smoke`) and through Haxe (`PlanarRegionSmoke`).
- Findings that shaped the native code:
  - OCCT booleans treat a face as one side of a sheet, so two coplanar faces looking opposite ways have
    nothing in common: every region face is turned to look along its plane's dominant axis (`looking_up`).
  - Sections at different heights lie in different planes. `PlanarRegion.section` moves its result to z = 0 so
    layers combine with booleans, and the boolean is run with a 1e-6 fuzzy value because two sections of one
    prism are equal only to within the tolerance of the shapes they were cut from.
  - `BRepOffsetAPI_MakeOffset` on a face with two separate loops that are concentric circles or arcs (a ring)
    crashes OCCT (SIGSEGV, not an exception): `MAT2d_MiniPath::MinimumL1L2` skips the `Extrema_ExtCC2d` results
    when `IsParallel()`, which two loops of eccentricity below about 1e-7 reach with inward offsets and both
    joins. Filed upstream as https://github.com/Open-Cascade-SAS/OCCT/issues/1606. Rectangular frames do not
    crash. `cad_planar_offset` therefore never offsets a face with its holes: the outer boundary is offset one
    way, each hole the other, and the holes are cut out. A face with a single wire never reaches that code,
    so the workaround avoids the crash completely.
- **Slicer** (`camkit.additive`: `Slicer`, `SlicerSettings`, `PrinterProfile`, `TestParts`): as in the step, with
  these decisions. Perimeters are exact lines and arcs from the offsets, so the plan's "arc fitting within 5 um"
  is not needed. The bead is a stadium (rectangle with semicircular sides) of the layer height and the line
  width, solid infill and perimeters are `lineWidth - height * (1 - pi/4)` apart (the spacing at which beads
  fill a layer exactly), and sparse infill is `lineWidth / density`. The infill core starts at
  `lineWidth/2 + (P - 1) * spacing + spacing/2` from the boundary. The solid region is the core less what
  the layers above and below all cover; layers without a full window are fully solid. Scanlines lie on a
  global lattice, so infill stacks from layer to layer. Travels retract when longer than 2 mm and leaving the
  part (even-odd test along the travel on the layer's section). The skirt is printed before the
  `PrintObject` marker, outside the object. `CamContour` gained a `validated` flag that skips its
  quadratic self-intersection check (a B-rep boundary cannot cross itself) and `CamPocketPlanner.fill` gives
  edge-to-edge scanlines.
- Numbers (debug CadKit, prebuilt release OCCT): SmallTab 6 layers in 19 ms, 467 operations, 120.0 mm^3 in the
  object (CAD 120). CalibrationCube (100 layers) 0.59 s solid: 7409 of 7435 mm^3 (-0.35 %); at 15 % infill
  0.84 s, 2791 mm^3. TestTower (60 layers) 0.32 s solid: 3196 of 3202 mm^3 (-0.2 %); at 15 % infill 0.46 s,
  51 retractions, 5029 lines of G-code. Checks: volume within 3 %, no bead outside the footprint plus half a
  width, the hole's wall a half bead from the nearest bead on every layer (the cube) and the tower's
  counterbore open, layer heights monotonic, deterministic output, G-code round trip within 0.1 %.
  CamKit 12311 -> 12442 assertions.

Done. What was built and decided:
- **Where the time went** (profiled with `perf` and the allocation census, in the order they were found):
  1. The task-space check measured each sample against the whole op's authored geometry (a scan of every primitive,
     62 000 for the gear): `GeometricPath.distanceTo` now uses a grid of the primitives' boxes, and `PosePath`
     finds the primitive at a distance by bisection (`indexAt`, `indexAfter`, `startOf`) instead of scanning.
  2. `AxisKinematics.solveDifferential` gave derived joints (a CoreXY motor's pulley follows both axes) a rate of
     zero while their positions moved, so the pulleys' joint paths were inconsistent, TOPP-RA did not see their
     real accelerations, and the lowered trajectory broke the limits by 3 % to 60 %. Each break started a
     smoothing retry (up to 24 whole re-lowerings); one section took 12 minutes. The rates are now the sum of the
     leaders'.
  3. `mk_path_lower` searched every sample for each of its 170 000 stages and every stage for each sample
     (quadratic): both are bisections now, and a stage whose joint is affine in the path (a straight move)
     skips the residual search, as does any stage whose coefficient sum is within the tolerance.
  4. `ProgramCompiler.projectCouplings` copied every segment of the trajectory into Haxe arrays and back
     (millions of boxed floats, and the collector marking them again and again): `mk_project_couplings` does it
     in native memory.
  5. The clock was sampled every millisecond for the task-space check; on an axis machine the executed position
     can only differ from the path's by the lowering tolerance, which the lowering bounds, so a section's own
     samples are checked and the clock at its ends. Timing bindings are read only when asked for.
- **Cornering.** Moves of different speeds join in one rounded path (the corner arc goes at the slower of the two),
  which removed the exact stop at every change of feature: the cube went from 4040 stops to 12, each of which
  costs milliseconds at 12 to 24 m/s^2. Not rounded: turns above 150 degrees, non-planar corners (one per layer
  change: a planar quintic cannot follow a z climb, and a stop costs a few milliseconds), and, new, corners whose
  rounding arc would be tighter than 50 micrometres (`CornerBlender.MIN_RADIUS`; a tolerance that itself asks for a tighter arc keeps it): between moves of a few tens
  of micrometres such a corner can only be rounded within tolerance by an arc of 8 to 15 micrometres, along which
  no machine moves, and whose accelerations the time law could not hold (TOPP-RA broke the limits by half again
  and retried 20 times). The tower's plan time fell from 410 s to 47 s with the rule, and its planned time from
  10.3 % above the reference to 1 %.
- **Windows.** A smooth path longer than `ProgramCompiler.MAX_SECTION_PRIMITIVES` (4000) is cut, and rests, at a
  primitive boundary: the memory and the sample budget of a plan stay bounded, and a rest every few hundred
  millimetres of path costs a few milliseconds. The plan's "windows with boundary speeds" is not done: with the
  stops this cheap and every layer change a stop anyway, carrying a speed across windows would add a feasibility
  fallback for no measurable time. Timing that still fails (`path.time failed`) is retried in two halves, each
  from rest to rest, down to a single primitive. (Removed in AM5b item 2.)
- **TOPP-RA.** The stock forward pass gave up ("forward pass" failure, seen on every load sliced from exact
  geometry) where the speed it had reached was the greatest the controllable set allows, a vertex where two
  constraints meet and rounding leaves no acceleration. The submodule is not ours to patch, so motionkit
  subclasses `TOPPRA` and retries the stage with the speed lowered by 1e-9, 1e-7, 1e-5, 1e-3 (relative); the lowered
  trajectory is still validated against the caller's limits. (Replaced by a fix in our TOPP-RA fork in AM5b item 2.)
- Gate (`machinekit/examples/printer/benchmark`, run with no arguments; in the workspace as `printing-gate`) on a thin
  gear (module 1.5, 40 teeth, 1.2 mm: 6 layers, 11 797 moves): planning at least 2x faster than the print over the whole
  job and its worst 10 s, planned time within 15 % of the reference, no failure. **Passed**: 1047.9 s planned,
  0.3 % above the reference; 94.1 s of processor time, 11.1x real time overall and 3.19x over the worst 10 s.
- **Numbers after AM5** (processor time, so other work on the machine does not count):

| Load | Planned | Reference | Planning | Real time (all / worst 10 s) | Sections | Before |
|---|---|---|---|---|---|---|
| Gear, whole (61 933 moves) | 2357.9 s | 2354.8 s (+0.1 %) | 350.1 s | 6.73x / 2.90x | 1004 | did not finish in 3000 s; sliced, failed in TOPP-RA |
| Calibration cube | 872.4 s | 867.6 s (+0.6 %) | 60.1 s | 14.52x / 9.66x | 1576 | 929.1 s (+7.1 %), 1.04x / 0.75x, 4042 sections |
| Test tower through G-code | 474.2 s | 469.6 s (+1 %) | 46.9 s | 10.12x / 1.87x | 1549 | 506.7 s (+7.9 %), 0.99x / 0.67x, 4271 sections |

  The tower's worst window is just under 2x: arcs are the dearest paths to lower (each joint is curved, so every
  stage is fitted and checked), and its 1324 corners stopped at for being below the minimum radius come from
  micro-segments in its G-code (the sliced chamfer and boss); the slicer should not emit them.
  Managed heap peaks 323 MB (gear), 198 MB, 213 MB.
- **The first full run of the gate failed, and was fixed before AM7 went on.** (1) CamKit's "a blended pocket stops
  at fewer corners" came out 90 against 90: the test asks for a 10 micrometre blend tolerance, which rounds a right
  angle with an arc of about 19 micrometres, and the new minimum radius of 50 micrometres stopped at those corners.
  The code was wrong, not the test: a corner whose own tolerance needs a tighter arc than the minimum keeps it
  (`min(MIN_RADIUS, 0.5 * the radius the tolerance asks for)`); the minimum exists to stop corners squeezed by tiny
  neighbours, not to override what was asked. (2) The CadKit suite loaded the shared `topological-naming` CadKit
  build, which lacks the AM3 natives (`undefined symbol: cad_shape_section`): the suite script's default
  `CADKIT_BUILD_ROOT` pointed at it. Every run since AM3 must use this worktree's `cadkit/build/debug`; the script
  now defaults to it. Earlier runs since AM3 were not tested against the wrong library: the quick suite does not
  run the CadKit suite, and the projects that slice (the machinekit smoke, the printing gate) build and load their
  own `libcadkit-core` (their logs say `Build CMake target cadkit-core`), as the slicing in them proves. The full run after the fixes (`am5b`, 2587 s) passed: every workspace project, KinematicsKit, CadKit, the MachineKit smoke, the app's project-source suite (physical router, bench-mill and plotter numbers unchanged; per-tick allocation down about 15 %, the belt router's slowest update from 1582 ms to 209 ms) and the editor tests.
- **CoreXY motor limits (coordinator review after the gate fix).** The review asked whether derived joints (a CoreXY
  motor's pulley) are limited by the sum of their leaders' limits instead of by their motors, so that a diagonal (the
  slicer's 45 and 135 degree infill) could drive one motor past its rate. Measured first, by `motorChecks` in the
  printing gate: a 100 mm move along x, along y and along both diagonals, planned with the drives' plan check on and
  sampled every millisecond. The sum is not looser than the motor: the axes' limits are shares of the motors'
  (`RobotModel.coupledLimits` divides a motor's rate by the sum of the couplings that use it, and its force by the
  coupling's share), so both axes at their limits turn a motor at exactly its rate. Diagonals turn the busy motor at
  78 of 79 rad/s (the other stands still), x and y moves turn both at 39 (half: the cost of a share is that a pure
  axis move uses half of what the motors give), the worst torque is 40 % of the pull-out curve, and the plan check
  finds nothing. So no plan exceeded a motor's rate or torque; the change is that `ToolpathMotionBinding` bounds a
  derived joint by its own drive's rate when the model has one (`min` with the sum), so the motor's limit is the
  constraint and the sum only the most the axes can ask. The acceleration stays the sum (a pulley's own inertia says
  nothing of the carriage it moves; the drive's torque is the plan check's job and it passes). Not done: axes at the
  motors' full rate (`coupledLimits(..., motorSpaceVelocity = true)` with `MotorSpaceConstraints` rows, as the
  plotter's machine program does) would make pure axis moves up to twice as fast. The printer plans derived joints as
  columns of the timed path, which carry the motor rows already, so the rows would be duplicates; whether to take the
  speed is a decision for the cube's print time (AM5b).
- Tests: `ToolpathMotionTests.printerLowering` (rounding across speeds, the arc's speed, stops by reason),
  `PlannerTests.testLongPathQueries` (grid distance against a scan, bisection at boundaries, sections of a long
  path), `CoreXyTests.testDerivedPulleysFollowBothAxes` (pulleys are the sums of their terms along a plan).

**AM6. The CoreXY printer.**
- `machinekit.printing` parts, plus catalogue rows (AM-D8), with unit tests: masses, connector
  frames, heater power, thermistor curve, extruder rotation per mm of filament.
- `machinekit/examples/printer/CoreXyPrinter`:
  - frame, CoreXY belts, Z bed on three screws, toolhead with extruder joint, fans;
  - switches and probe, `homeAfter` (Z after X/Y; probe at bed centre);
  - drivers, supply, controller.
- `CoreXyPrinterChecks` in MachineKit smoke: FK of nozzle versus axes, rails, OCCT interference,
  coupled limits, belt and screw couplings, homing order.
- `CoreXyPrinterPreview`, `materia.project.json`, a Start-page entry in
  `app/src/editor/ExampleCatalog.hx`, and a `PROJECT_SOURCE_ONLY=printer` check that the
  MuJoCo backend builds it and homes it.

Done. What was built and decided:
- **Parts** (`machinekit.printing`: `Thermistor`, `HeaterCartridge`, `Nozzle`, `Hotend`, `HeatedBed`, `Fan`,
  `Extruder`, registered in `MachineKitComponents`): typed and catalogue-backed, their values marked assumed
  where no datasheet is cited, every primitive named (`Solids.named`) for the naming audit. Catalogue rows
  added: pancake stepper `17HS08-1004S`, rails `MGN12H` and `MGN9C`. The Beta thermistor gives 24861.65 ohm at
  60 C and 6975.2 ohm at 100 C (`PrintingPartTests`).
- **Assembly** `machinekit/examples/printer/CoreXyPrinter`: 20x20 frame, MGN12C Y rails, 40x20 crossbar with an
  MGN12H carriage, GT2 belts A and B, two NEMA 17 on the rear posts, the toolhead (nozzle, hotend, cartridge,
  thermistor, hotend fan, blower, extruder with its pancake motor and an 18:54 module-0.5 gear pair, joint `e`,
  geared to the motor shaft by `GearMesh`), the bed on three Tr8x2 (4 starts) screws and two MGN9C rails, a
  home switch per axis, drivers and the supply. 53 definitions, 100 occurrences, 53 BOM lines, 17 kg. Axes: x
  and y 0..220 mm, z 0..220 mm; machine zero (-110, -110).
- **Decisions.**
  - Z homes with the bed lowered (the upper end of the joint), not at the nozzle. `DriveSystem.resolveSwitch`
    wants a home trip strictly beyond the soft limit and collinear with the joint, so a probe at the nozzle
    (which trips inside the travel) cannot be a home switch. An inductive sensor on a pillar from the base
    plate senses a flag under the bed's carrier; X and Y then home with the bed away from the nozzle
    (`homeAfter("x", ["z"])`, `homeAfter("y", ["z"])`). A second inductive probe beside the nozzle is kept for
    levelling and has no switch yet.
  - The CoreXY coupling helper moved from the plotter into `AxisBuilder.turnCoreXyPulley`; the plotter and
    the printer both use it.
  - A joint that follows several axes (a belt motor's pulley, the idlers of both belts) belongs to no axis.
    `MotionAxisCouplings.derived` lists such joints (`DerivedJoint`: terms of `ratio * leader + offset`) and
    `AxisPlanner` projects them from the axes' trajectories, so `MotionSystem` and `RuntimeHomingDriver` plan
    homing, jogs and axis moves on a CoreXY machine. Before, `expand` threw "A multi-axis follower requires
    coordinated polynomial planning" as soon as a home switch existed. Followers of joints no axis drives
    (the extruder's gear train) stay where they are; a follower that mixes driven and undriven leaders still
    throws. Test: `CoreXyTests.testDerivedPulleysFollowBothAxes` (six derived joints, each the sum of its belt
    terms to 1e-9 along a plan).
- **Checks.** `CoreXyPrinterChecks` in the MachineKit smoke (occurrences, joints, couplings, switches, forward
  kinematics of the nozzle, rail ranges, clearances, belt clamp, gear distance), `PrintingPartTests`, the full
  MachineKit smoke. In the app, `PROJECT_SOURCE_ONLY=printer` builds the assembly into the shared MuJoCo
  simulation and homes it (z first, then x and y) in 8.75 s on the deterministic backend. Planning limits from
  the motors: x 250 mm/s 30.03 m/s^2, y 250 mm/s 15.59 m/s^2, z 100 mm/s 22.87 m/s^2. A Start-page entry
  ("3D printing" category) opens the project.
- **Left for AM8.** Homing on MuJoCo with gravity: the bed (a follower of nothing the simulation actuates) falls
  freely, because the screw turns do not hold it. The virtual device cannot home CoreXY switches yet: a device
  input names one actuator whose channel joint is the switch's joint, and a CoreXY switch joint is driven by
  two motors.

**AM7. Heaters (AM-D6).**
- Device: the `heater` module (PID, Beta thermistor, runaway watchdog, safe state) with Rust
  unit tests for every fault path, plus a virtual thermal plant in `device_virtual`.
  - Channel and sensor bindings go in the deployment layout; extend RKD6 only if needed, and
    regenerate it with `python3 -B -m tools.wire generate --config robotkit/wire6.json`.
  - Nucleo board: compile-checked (`cargo check --offline`) with placeholder pins, like the
    welder profile.
- Host: heater and fan channels, the `heater.<name>.at_temperature` sensor, `Heat(wait)`
  barriers, and fault-to-stop.
  - Add the new channels to `MachiningRun.continuationFrom` and `safeShutdown`, or to a printing
    counterpart.
- Tests:
  - heat-up time of the hotend to 210 °C and of the bed to 60 °C (assumed plants; record them);
  - steady-state ripple;
  - runaway on a disconnected heater, a fallen-off thermistor and a fan blast;
  - the cold-extrusion static check and device fault.

Done. What was built and decided:
- **Controller** (`robotkit/device_protocol/boards/heater.rs`, `no_std`, shared by the board and the virtual device):
  a PWM duty out and a thermistor ADC in (Beta model: 100k NTC 3950 under a 4.7k pull-up on 3.3 V, so the
  controller reads 0.1 degree per LSB of a 12-bit ADC at 210 C); PID with derivative on the filtered measurement
  (2 s), integral held outside a 10 C range and anti-windup; and a watchdog that latches a fault and switches the
  output off until a session resets it: sensor open or short (within 1 % of a rail), over-temperature, not heating
  (a window without a 2 C gain while heating up; 20 s for the hotend, 120 s for the bed), left the band (below the
  target by more than the hysteresis for a window after reaching it), cold extrusion (the extruder actuator steps
  forward below the material's minimum), and a command or configuration that is not a number. The same module
  type-checks on the Nucleo target (`heater_profile.rs`, placeholder pins; `check-mcu-build.sh` passes).
- **Virtual device** (`robotkit/device_virtual/src/heaters.rs`): peripheral profile 2 runs the controller against a
  lumped plant `C dT/dt = P duty - (h + h_fan fan)(T - T_amb)` with a 12-bit ADC; one or two heaters, each with a
  sensor slot (a `tool_heater` frame of five values: temperature, target, duty, at target, fault), a target channel,
  an optional fan channel and an optional extruder actuator. A fault stops the robot (`ProcessFault`) and takes every
  target channel to its safe value. The session is refused unless every target channel is analog, safe at zero and
  keeps its value through a commanded stop (a pause does not cool the hotend; a fault or an emergency stop does).
  An experiment reaches the plant through `rkd_virtual_set_peripheral_input`: element connected, thermistor on its
  block, a draught (W/K), wire open, wire shorted. The peripheral parameter list grew from 16 to 64 doubles
  (`robotkit_simkit.h`, the HXI regenerated). No RKD6 wire revision: the target is an analog channel event and the
  report a `SENSOR6` slot.
- **Host.** `robotkit.tool.HeaterSensor` (the frame and its fault messages), `HeaterChannels` (target and fan
  declarations: zero when safe, kept through a commanded stop), the `tool_heater` external sensor kind (the runtime
  validates its frames), and `robotkit.runtime.VirtualHeaters` (configures the device from `HeaterSetup`s, publishes
  its reports to the sensors, reaches the plant). The `Heat(wait)` barrier and the print-side channels
  (`MachiningRun.continuationFrom` and `safeShutdown` counterparts) are lowered in AM8 with the printer binding;
  `heater.<name>.at_temperature` is the frame's fourth value. The static cold-extrusion check is
  `PrinterOps.coldExtrusion(ops, minimum)`: filament pushed or a bead laid before a waited `Heat` of the hotend to
  the minimum (and after one that turns it down) is a finding with its source.
- **Numbers** (assumed plants: hotend 10 J/K, 40 W, 0.06 W/K, +0.05 W/K with the part fan; bed 500 J/K, 250 W,
  0.7 W/K; room 22 C), through the whole stack (Haxe test over the virtual device): the hotend is at 210 C
  (within 3 C) after 65.1 s with a peak of 210.8 C; the bed is at 60 C after 84.2 s. Holding 210 C the block
  stays within 210.01 to 210.72 C (quantized ADC), and with the part fan at full duty within 208.2 to 210.0 C.
  The unquantized controller alone (`device_protocol` tests): hotend 63.9 s, bed 77.2 s, ripple under 0.1 C.
- **Tests.** `device_protocol/tests/heater.rs` (7: thermistor round trip, heat-up and hold of a hotend and a bed,
  every fault turns the output off and stays until reset, leaving the band, cold extrusion, bad commands),
  `device_virtual` (10 new: plant times, ripple, dead element, fallen thermistor, draught, open and short wire, cold
  extrusion, parameter lists, session channel rules, both blocks on the device with sensor frames, pause versus
  emergency stop, fault stops the robot and clears every target), `HeaterDeviceTests` (RobotKit world 5439 -> 5469
  assertions), `PrinterOpCoverage.coldExtrusion` (ToolpathKit 141 -> 149).

**AM8. Extruder joint and the first simulated print (AM-D3).**
- Lowering: printer binding (axes plus extruder joint, filament diameter, maximum flow, pressure
  advance K and smoothing).
  - Distance-driven joint columns in `ProgramCompiler`.
  - `FilamentMove` as an extruder-only move.
  - Flow caps.
  - Pressure advance.
- `SceneArtifact.printing` job: the program (G-code text, or a slicer request naming a part and
  settings), the printer's axes and extruder joint, heaters, fans, material (filament diameter,
  minimum extrusion temperature, temperatures) and controller.
  - Machining and printing are mutually exclusive. Follow the one-schema-version rule.
- App player: extract what `CncProgramPlayer` and a print player share (homing, compile, plan,
  stream, overrides, panel) instead of copying it. The print player adds heaters, the extruder
  and markers to the panel (layer, temperatures).
- Test `PROJECT_SOURCE_ONLY=printer` prints `SmallTab` on MuJoCo with the virtual device.
  - Check: no stalls, no faults; the extruder's deposited filament is within 1 % of the
    program's; a speed override keeps volume per distance within 2 %.
  - Print: print time, heat-up, planning ratio and plan-check results.

Done. What was built and decided:
- **Lowering and kit level** (the first commit of AM8): the extruder is a joint driven by path distance
  (`DistanceDrive`, ramps that conserve the nominal feed), `FilamentMove` is a joint offset, flow caps, pressure advance
  as a windowed polynomial transform (`mk_advance_joint`), `PrinterBinding` (see the commit).
- **`SceneArtifact.printing`** (version 28, 27 rejected): the program as Klipper G-code in machine coordinates, the three
  axes, the extruder joint, filament diameter, gear radius, melt rate, pressure advance and its window, the ramp
  length, the minimum extrusion temperature, the controller's step rate, and the heaters (sensor slot, mount, channels,
  PID and watchdog numbers and the thermal plant). Exclusive with machining and a mission. The "slicer request" variant
  is not offered: the generator slices when it generates (`CoreXyPrinterJob.smallTab`), so the artifact carries the
  program that was sliced, and a later slicer UI writes the same section. There is no work offset: printer G-code
  is in machine coordinates (the bed's front left corner is the machine's zero), as `CncCompiler` reads it.
- **Generator** (`CoreXyPrinterPreview.printer()`): slices `SmallTab` with the default settings and takes the numbers of
  the extruder (hob 7.3 mm, 1.75 mm filament, 15 mm^3/s), the hotend (13.5 J/K, 0.12 W/K, +0.05 W/K with the fan,
  cartridge 40 W, thermistor 100k beta 3950) and the bed (640 J/K, 300 W, 1.4 W/K) from the parts. The PID is derived
  from each plant, not typed: the loop closes in a twentieth of the plant's time constant (`kp = C / (P tau / 20)`),
  the integral time is an eighth of it, there is no derivative, and the watchdog expects 2 degrees in a quarter of the
  time constant. (A first tuning with an integral time equal to the time constant left the hotend creeping the last
  degrees until the watchdog called it runaway; the test caught it.)
- **App.** `ToolpathPlayer` (new) holds what a CNC and a print player share: compile from where the machine is, homing
  when it has switches, streaming plans to the robot, looping, speed override, feed hold, plan checks, stepper slip
  and encoders; `CncProgramPlayer` now extends it (restart, stock and tools stay in it) and `PrintProgramPlayer` is the
  second user. The print player plans over the whole machine (axes, belts, screws and the extruder, with the plan
  check on all of it), reads the heaters' reports before each step, answers `heater.<name>.at_temperature` waits from
  the `tool_heater` sensor, stops the print on a heater fault with the controller's message, and tracks the layer from
  the G-code's layer markers. `AssemblyRobot.add` gives a project with a print job a virtual device, the heaters'
  sensors and channels and `VirtualHeaters`. The CNC panel serves both (title, status line, restart only where the
  player can restart).
- **The virtual device and a CoreXY.** The device cannot read a CoreXY's home switches (a switch joint driven by two
  motors; AM6 left this open), so a print job's model drops its switch inputs and the machine is referenced. A virtual
  device counts steps from zero, and the assembly's actuators count from the joints' zero: the machine powers up at
  its origin, all three axes at their lower limit (nozzle at the bed's front left corner, touching it), so the player
  compiles from where the robot stands once the device has taken over (it waits 0.15 s for the clock sync, as the
  machine program does), not from the assembly's initial pose. Starting exactly on the limits works because of the
  start-at-limit rule (AM5b item 1 is about that rule).
- **The segment density problem, measured and fixed.** The first print stopped in the device's link qualification and
  then in the robot's queue: `SmallTab` plans to 163,392 polynomial segments over 56 s of motion (median 33 us, 140,323
  under 1 ms, 3,000 a second), because TOPP-RA's 32 stages per path sample span lower to one segment each and the
  extruder's ramp stations and the corner arcs make many tiny spans. The robot's queue holds 4,096 knots and the stream
  keeps 2 s ahead; the endpoint refuses a link that cannot refill 128 segments in time. Two merges, both exact to a
  fraction of a step: `mk_merge_segments` (MotionKit native, opt-in per compiler: `mergeSegmentsSeconds`, set to an
  owner period by the printer binding) replaces runs of lowered segments by one quintic through the run's end states
  where it stays within 10 microseconds of motion at the joint's speed limit, 1 % of its speed and 10 % of its
  acceleration of the pieces, never past a limit the pieces keep to, found by doubling and bisecting; a plan whose merged
  form breaks a limit anyway is built from the unmerged path. `SmallTab`: 163,392 -> 25,580 segments (457 a second);
  the gate's planning also gets faster (the gear: 10.37x -> 13.36x real time overall, worst 10 s window 3.11x ->
  4.12x, planned time unchanged, 0.3 % above the reference). `compile_device_segments6` merges again for the device's
  link: a segment under 2 ms starts a run of up to 4 ms that one quintic follows to a tenth of a step and a percent of
  the step rate of the originals (else the plan goes as it came, as does any plan the merge would take past a
  deployment limit). Segments of 2 ms or more are sent as planned: a first setting that merged up to an owner period
  everywhere also changed the gantry weldment, whose arc then could not be held. The 18,900 segments under 1 ms that
  remain are corner arcs (AM5b item 8). The link is the board's native USB (12 Mbit/s): at the welder's 3 Mbaud the
  qualification still refused the bursts of tiny segments at the arcs.
- **A change of mine that broke the gantry weldment, found by the full suite.** The extruder gear needs an acceleration
  from its motor's torque, and `RobotModel.coupledLimits` now gives a turning joint with no acceleration of its own
  one; that also gave one to every directly driven revolute joint of the arms, which changed the gantry welder's
  plans (the arc could not be held in three restarts). The rule now applies only to a joint whose motor is behind
  a gear train (the motor is a follower of the joint), as the extruder's is; the weldment passes again (159.8 s
  cycle, tip within 0.2 mm of the seams, legs of 5 mm).
- **Fixes the print found.** The chunk-boundary acceleration tolerance (`TrajectoryStream.programSubmission`) allowed
  the jump between chunks plus 1e-5, and a quintic with a large jerk differs between the previous segment's end and
  the instant a nanosecond before by more than that: it now adds the jerk over that nanosecond. The motor-space check
  (AM5 commit f91cb56cc) and a regression test for the diagonals came out of the review of CoreXY limits.
- **The print (MuJoCo, virtual device, 100 Hz).** `SmallTab`, six layers: 169.8 s from power-up to the end, of which
  the hotend reaches 210 C (within 3 C) at 103.6 s and the bed 60 C at 90.5 s (the program waits for both), then 66 s of
  printing. No stall, no heater fault, no failure; the extruder feeds 56.08 mm of filament against the program's
  56.08 mm (0.0 %); with the speed halved from layer 3 to 4 the filament per metre of path is 33.98 mm against 33.89 mm
  at full speed (0.3 %, limit 2 %). The plan check ran on 39 plans: no stepper stall, worst torque 45 % of the
  pull-out curve (a Z screw motor), 30 plans carry accuracy findings, worst deviation 329 micrometres on y (the belt
  stretching under acceleration: that is what the check is for, recorded not hidden). Planning cost 54 s of processor
  time for the 66 s of motion; the slowest tick 13.8 s, the first plan's synchronous compile: an interactive app would
  hold still that long at the start of a print (AM5b item 9).
- **Tests.** `PlannerTests.testMergedSegmentsFollowTheOriginals` (a ramp of 400 pieces to a few, the zigzag kept, limits
  kept), the C++ `robotkit_device_compiler6` test (400 pieces to at most 8, durations and ends kept), `ProjectKitTests.printing`
  (round trip and twelve rejections), `ProjectSourceTests.checkPrinter` (the print above with its assertions, and the
  panel), the printing gate's `motorChecks`. The CNC refactor is covered by the existing router and mill checks.

**AM9. Deposition and checks (AM-D7).**
- `sk_stock_add` and empty-stock creation, with C tests.
- App `PrintingStock` (the `MachiningStock` counterpart) deposits from the actual tip and
  extruder motion each tick, previewed through `StockPreviewWorker`.
- Checks for `SmallTab` (in the app test) and the cube (a slower test, or a kit-level test
  driving the simulation without the editor):
  - deposited volume within 3 % of CAD;
  - bounding box within 0.2 mm;
  - hole diameter within 0.3 mm;
  - no deposit outside the part's footprint plus a bead.
- Print the numbers.

Done. What was built and decided:
- **StockKit** (API version 3): `sk_stock_create_empty` and `sk_stock_add` (C ABI, HXI, `Stock.empty` and `Stock.add` in Haxe). A move's
  swept solid is merged into each ray's intervals (all three grids, the Z grid measuring the volume added); the surfaces it makes take the
  move's source and face out of the solid; a sweep inside material already there changes nothing; threads, ordering and determinism are
  those of `sk_stock_cut`. C tests: 20,906 checks pass; Haxe StockKit tests 100,681 assertions.
- **App.** `PrintingStock` (the `MachiningStock` counterpart): `follow` reads where the nozzle tip is and how much filament the extruder
  has fed (both from the simulation, so slip, a flow the hotend cannot melt and the pressure-advance lead show), and lays the stretch as a
  bead of the layer's height and the width that holds the fed volume (the filament per distance smoothed over 1.5 mm of travel so the stepper's few steps per tick
  do not swell and thin the bead; filament pulled back is owed first; filament pushed while the tip stands piles up as a blob). A melt lag
  (`Hotend.pressureLag`, assumed 40 ms, the job's `meltLag`) models the spring of filament and melt that pressure advance cancels. Shortfalls
  of beads that overlap their neighbours are made up by the next beads, except on the outer wall and the skirt, which face the air, so
  dimensions hold. `StockMeshes` (extracted from `MachiningStock`, now also its user) contours the stock on the preview worker; the
  `PrintedPart` marker at the bed's front left corner is replaced by the material (`SceneArtifactPrinting.deposit`). `SceneArtifact` version
  29 (28 rejected). Also a fix to a stale native test (`robotkit_device_compiler6`: a motor built with a double where the field became whole
  step ticks) and the device compiler accepts the position error of a float that carries an extruder angle many turns from zero.
- **The cube's hole, diagnosed.** The WIP check failed with "the hole measures 5.66 mm across, CAD 6". The slicer is right (its own test
  puts the nearest bead centre 3.225 mm from the hole's axis, wall plus half a bead, and the walls are exact arcs); the deposit is right (the
  bead's footprint conserves volume; the short cube below measures 6.0 mm both ways). The measurement was wrong: the calibration cube's hole
  is off centre by (2, -1) mm, and the check ran its ray through the cube's centre, 1 mm from the hole's axis, where the chord of a
  3 mm radius circle is 2 sqrt(9 - 1) = 5.657 mm. It now measures along x and along y through the hole's own centre (112, 109 mm),
  tolerance unchanged at 0.3 mm.
- **Checks** (`checkPrinterCube`, `measurePrinted`): the deposited material against the filament fed (2 %), the volume in the cube's footprint
  (solid: within 3 % of CAD; sparse: below the solid volume), the bounding box within 0.2 mm, the hole's diameter within 0.3 mm, and nothing
  laid beside the part but the skirt. `printer-cube-short` is the regular check (a 4 mm, solid block with the same hole: 20 layers), run in the
  app suite; the full 100-layer cube (`printer-cube`, default infill) runs by itself and in the full suite only (`am-suite.sh`).
- **Numbers.** SmallTab (`printer`): printed in 163.5 s (hotend 103.6 s, bed 90.5 s), filament 56.08 of 56.08 mm, 0.3 % flow difference at half speed;
  plan checks on 26 plans, 17 flagged, worst torque 42 % (motor A), worst deviation 329 um (y) (AM8 notes: 169.8 s, 39 plans, 30 flagged,
  45 %, 329 um: the planning fixes of AM5b shortened it); the print laid 135 mm^3 in 5,664 beads for 134.9 mm^3 of filament, the tab holds
  118 mm^3 of its CAD 120 (-1.7 %), spans 104.9..115 x 104.9..115 x 0..1.2 mm, 0 mm^3 beside it. Short cube: 526 s of simulation (534 s wall),
  1,470 mm^3 in the cube of 1,491 laid for 1,497 fed, CAD 1,487 (-1.1 %), spans 100..120 x 99.9..120 x 0..4 mm, hole 6.0 x 6.0 mm, 41,212 beads.

**AM9b. The app's print simulation at real time.** The full cube simulates its heat-up at about 9x real time but prints at about 1.7x slower than
real time (each 100 s of print costs about 155 s of wall time, steady across the 100 layers, so a constant per-tick cost, not one that grows with
the part); SmallTab keeps up (174 s of print in about 118 s; the short cube takes 534 s of wall time for 526 s). An editor preview of a dense part therefore lags.
Profile a slice of the cube print (from layer 10, a few minutes) with `perf` and the allocation census, split the per-tick cost between MuJoCo,
the virtual device's stepping, planning, deposition into the dexel stock, the stock preview's remeshing and the per-tick checks, and fix the top
costs. Done when the full cube simulates at least at real time on this machine with the same results (hole, volume, bounding box, SmallTab numbers
unchanged).

**AM10. Klipper config and G-code (AM-D9).**
- `PrinterKlipperConfig.generate(assembly, controller)` produces `printer.cfg` text.
- Tests:
  - golden values (`rotation_distance: 40` for GT2 20T, `8` for T8 screws, extruder
    `rotation_distance` and `gear_ratio` from the gears, limits equal to `coupledLimits`);
  - parsing the result back with a small INI reader;
  - a placeholder pin report.
- The Klipper G-code flavour from AM2 is used for the sliced demo bracket, written next to the
  config as the example's export.

Done. What was built and decided:
- **`machinekit.printing.KlipperConfig`** (typed model of a CoreXY `printer.cfg`, `render`, `parse` (a small INI reader),
  `pins`, `pinReport`) and **`CoreXyPrinterKlipper.printer(...)`**, which fills the model from the printer's parts and
  the print job; `CoreXyPrinterExport.write` writes `printer.cfg`, `pins.txt` and the sliced `NemaMountBracket` (a
  printable L bracket for a NEMA 17, new) as Klipper G-code into one directory.
- **Numbers come from the parts.** Belts `rotation_distance` 40 (GT2 pitch x 20 teeth), the three bed screws 8 (the Tr8x2
  four-start thread, now `CoreXyPrinter.zThread()` used by the model and the config), the extruder 22.93 (pi x the 7.3 mm
  hob) with `gear_ratio: 54:18` from the two gears, 16 microsteps, `max_velocity`/`max_accel` the slower belt axis's
  `coupledLimits` (250 mm/s, 12,563 mm/s^2), `max_z_*` the bed's (100 mm/s, 18,066 mm/s^2), the heaters' PID gains times 255
  (Klipper's base), pressure advance and minimum extrusion temperature from the job.
- **What the review of the unverified draft changed.** (1) The drivers' `run_current` was the model's 1.68 A times 0.7071,
  but `PhaseRmsCurrent` is already an RMS value and Klipper's TMC `run_current` is RMS: it is the model's setting
  (1.68 A, 1.0 A for the extruder), asserted. (2) The bed's second and third screw steppers carried their own endstop,
  travel and homing keys; only the stepper that owns the switch does now (`leads`), the others have just their drive, so
  the config is what Klipper accepts for several steppers on one rail and the pin report has 35 pins, not 36 (the model has
  one Z switch). (3) Added `[probe]` from the model (offsets `CoreXyPrinter.PROBE_X/PROBE_Y`, now shared with the builder,
  and `z_offset` = sensing distance - the face's height above the tip = 1 mm), and `[virtual_sdcard]`, `[pause_resume]`
  and `[exclude_object]` that Moonraker needs to run a print.
- **Homing speed** is a derived conservative value (a fifth of the axis's speed, at most 50 mm/s), not `HomingDynamics`'
  seek speed: the homing axes are internal to `MotionSystem`; Klipper's `homing_speed` is a preference, not a limit.
- Each pin is a `PIN_*` placeholder; the pin report lists every one and the test checks the file and the report agree.
- **Tests.** `PrinterKlipperTests` (machinekit smoke): 122 assertions (golden rotation distances, gear ratio, limits equal to
  `coupledLimits`, gains, drivers, probe, followers without switches, pin report against the file, INI round trip, a key outside
  a section refused, the export directory and the bracket's Klipper G-code: `M83`, `EXCLUDE_OBJECT_DEFINE`, `M109 S210`).

**AM11. Moonraker client and twin mirror (AM-D9).**
- A client over `NativeTransport`: HTTP/1.1 requests, JSON, file upload (multipart), print
  start/pause/resume/cancel and status query. WebSocket subscription is optional.
- An in-process fake Moonraker for tests: it accepts uploads, runs a fake job whose progress
  advances, and reports toolhead position and temperatures.
- App: "Send to printer" (endpoint in the editor settings), and a live mirror that drives the
  printer assembly's joints from the reported toolhead position and shows temperatures.
  - Test: against the fake server, the mirror follows within one poll period.

Done. What was built and decided (Moonraker and Klipper are GPL: written from their documented APIs only, none of their code):
- **`processkit.moonraker`**: `HttpFrames` (HTTP/1.1 framing by Content-Length; chunked refused), `MoonrakerClient` (HTTP over
  `NativeTransport`: file upload as multipart to any root, `printer/gcode/script`, print start/pause/resume/cancel,
  `server/info`, the running configuration), `MoonrakerStatus`, and the three additions below. An error status, a transport
  failure, a malformed body or a reply that does not come in time is the client's `fault`.
- **Addition 1: the live position.** The mirror follows `motion_report.live_position` (and `live_velocity`,
  `live_extruder_velocity`), where the nozzle is. `toolhead.position` is the end of the last queued move and runs ahead of the
  nozzle by everything Klipper has planned but not stepped; it is kept apart as `queuedPosition` and nothing follows it. The
  fake server keeps both and puts the queued end 8 mm of route ahead while printing, so the tests catch the wrong one (the app
  check asserts the model is exactly at the nozzle and more than a unit away from the queue's end).
- **Addition 2: the WebSocket subscription.** `Sha1` (RFC 3174, since the standard library has none) and `WebSocketFrames`
  (RFC 6455 handshake key and answer, masked and unmasked frames, 16- and 64-bit lengths, fragments, pings; the RFC's own
  example bytes are tests) under `MoonrakerSocket`: JSON-RPC `printer.objects.subscribe` for the objects `MoonrakerStatus`
  reads, then `notify_status_update` pushes of only the fields that changed, merged into the state kept. HTTP stays for upload and
  print control. `PrinterLink` has no poll period any more: an update is given to the mirror when it arrives, and the test
  asserts one advance of the machine is exactly one update and the model is at the nozzle after it (within 1e-9).
- **Addition 3: installing a Klipper config.** `ConfigInstaller` uploads the `printer.cfg` to the config root, sends
  `FIRMWARE_RESTART` through the gcode script endpoint, asks `server/info` until Klippy is `ready` again (or reports its error
  state and message, or that it did not come back in time), reads `configfile.settings` and `configfile.warnings` and
  `ConfigReadback.compare`s them with the file it sent: a missing section or key, a value that differs (numbers to 1e-6
  relative, booleans, text ignoring case; Klipper lower-cases keys and adds defaults) and each warning are a sentence in
  `mismatches`. The fake models it: the file is read at the restart, Klippy answers `startup` twice, `deviations` change
  values the way Klipper's own limits might and `warnings` warn, and a missing file leaves Klippy in `error`. The editor has
  `Printer: Install Klipper config` (the file is the setting Machine > Printer > Klipper Config File; the printer example's
  export writes one) and `Printer: Send to printer`, `Pause`, `Resume`, `Cancel` and `Mirror`; the address is the setting
  Machine > Printer > Moonraker Address.
- **What the review of the unverified draft changed.** It polled `printer/objects/query` on a timer and mirrored
  `toolhead.position`; its fake served one connection; the mirror test measured a lag of up to one poll period. All three
  changed as above; the fake now accepts any number of connections and `FakeMoonrakerServer.start` does the listening so
  that the processkit test and the app check share it.
- **Tests.** `processkit/tests/moonraker`: 164 assertions (framing, SHA-1 vectors, WebSocket frames, status parsing, readback
  comparison, and the conversation: upload and start, HTTP status, subscription updates, pause/resume/cancel, config install
  with a deviation and a refused config, a 404, a silent server, a closed connection). `ProjectSourceTests.checkPrinterMirror`
  (`PROJECT_SOURCE_ONLY=printer-mirror`, also part of `printer`): the print is sent and started, then 45 updates are mirrored, each
  leaving the model within 1e-9 of the nozzle and 8 units (mm) behind the end of the queued moves, the extruder within 1e-6 mm, and pause, resume and cancel follow; a config
  installs and reads back clean, then a value Klipper changed is reported. 14 s wall.

**AM12. Input shaping (AM-D10).**
- ZV, MZV and EI shapers in TrajectoryKit or MotionKit, with frequency and damping as machine
  data, on the axes or motor joints.
- A compliant belt model in the simulation, if the X9 belt and load model does not already give
  MuJoCo a belt spring.
- Test: a step move on the printer; the residual vibration at the modelled resonance drops by
  at least 5× with the shaper, and the cube's print time grows by less than 5 %.

Done. What was built and decided:
- **Shapers** (`motionkit.trajectory.InputShaper`, `ShaperImpulses`): ZV, MZV and EI from the standard impulse formulas (Singhose; the same
  parametrisation Klipper documents: MZV at 0, 0.375 and 0.75 of the damped period, EI at 5 % tolerance), with frequency and damping as machine data.
  `mk_shape_trajectory` (MotionKit native, `Trajectory.shape`) convolves a timed trajectory exactly: each piece of the result is the
  sum of the delayed polynomials, cut wherever any delayed copy changes segment, held at rest beyond either end. Amplitudes are positive and add up
  to one, so a shaped joint never goes faster or harder than the plan it came from; the result is as much longer as the longest delay
  (the shaper's tail). A joint without a shaper (the other axis, the extruder, the bed) is only delayed by the same mean delay to stay in step.
  CoreXY motors are shaped through their axes: the followers are projected again from the shaped leaders.
- **Where it is applied.** `ProgramCompiler.shapeJoints` after timing and before merging (the shaper is machine data of the axes: the printer job's
  `shaping`, `PrinterBinding.shapers`, `SceneArtifact` version 30, `CoreXyPrinter.SHAPER_X_HZ` 53.4, `SHAPER_Y_HZ` 34.4, damping 0.1, MZV). The plan's
  events and its path-distance clock move with the motion by the mean delay; the task-space check allows the spread that the impulses open between
  the motion and its delayed copy (a bound from the largest acceleration). The merge (`mk_merge_segments`) joins a run shorter than 4 us (a cut
  of a shaped path landing a few nanoseconds from another) to its neighbour through the end states of both.
- **The known gap, fixed.** With shaping on, `jointPathPlanner.checkMotion`, the configuration constraint and `recordClearance` checked the unshaped
  motion; they now check `timedMotion`, the motion that is executed (shaped, projected), and the clearance report's times are the shaped plan's.
- **The streaming fix a shaped print needed.** A shaped path is denser; the robot's trajectory queue filled before the stream's two seconds were
  in it and a refusal faulted the session. `TrajectoryStream` counts what it has submitted and the plan has not yet run, keeps a margin of 128 knots
  and treats `RK_ERROR_QUEUE_FULL` as flow control (the chunk goes again later), not a fault.
- **Resonances come from the model.** The gate checks each axis's shaper frequency against the drive model (`DriveLoads.forAxis`: the mass the axis
  moves on the stiffness of its belts, x 0.748 kg on 84,238 N/m = 53.4 Hz, y 1.806 kg on 84,238 N/m = 34.4 Hz) to ten percent.
- **Acceptance (the printing gate, `shapingChecks`).** A 50 mm step move along x, along y and on the diagonal, planned plain and shaped; the carriage
  (that mass on those belts, damping 0.1, the plan driving the far end of the belts, integrated at 20 us) is left shaking for 356 -> 7 um on x
  (51.8x), 370 -> 8 um on y (48.6x), and 59 -> 1 um and 361 -> 7 um on the diagonal (44x and 48x); at the design frequency and ten percent either side
  of it every case drops at least 5x. The shaped plan is the plain one lagging by the shaper's mean delay and ending its tail later (233 ms against
  211 ms on x), never above the axes' velocity and acceleration limits (1.001), and keeps its merged form (no merge refusals). **The cube's planned
  time grows from 870.3 s to 880.5 s (1.2 %, limit 5 %)**, 115,240 segments plain and 132,669 shaped (132 and 151 a second).
- **Belts in MuJoCo (X9).** X9 does give MuJoCo a belt spring (a soft equality with the belt's compliance), but the backend fixes its damping ratio at 1,
  critically damped, so the carriage cannot ring there. With the shapers on, the virtual-device print still runs on MuJoCo: the 50 mm move ends at X 80 mm
  within one microstep and completes later than the plain one by the shapers' delay and tail (`PROJECT_SOURCE_ONLY=printer-shaping`, numbers below).
  Validating the ringing itself in MuJoCo needs an authored damping ratio on the coupling (`JointCoupling`, the simulation's coupling description and the
  belt model; zero keeping today's default): left open, and the gate's check uses the same mass and stiffness analytically.
- **Tests.** `PlannerTests.testShapedTrajectoryIsTheWeightedSumOfDelays` (the sum of delays, limits not raised, the shapers' amplitudes and
  residuals, refusals), the C++ `motionkit_path` test (`mk_shape_trajectory`: sums of delays, a delay-only joint, three refusals), `ProjectKitTests` (shaper
  round trip and five rejections), the printing gate (`shapingChecks`; `PRINTING_BENCHMARK_ONLY=shaping` alone), `printer-shaping` in the app.

**AM5b. Motion cleanup (after AM12).** Planned work, not for the AM run. Each item removes a workaround that AM3 to AM8
left in the motion path.
1. The start-at-limit tolerance (`ae6e95ec5`) is duplicated in four places (`ValidationLimits.admittingStart`,
   `ProgramCompiler.checkJointPosition`, `runtime.cpp validate_appended_path`, `device_compiler6.cpp`) with a magic
   `1e-6` fallback: move the rule into trajectorykit's shared validator (`trajectory_core`) once, and settle where a
   homed axis should rest (a home coordinate exactly on the travel limit is the root cause). **Done.** The rule lives once,
   in `trajectory_core` (`validation.cpp admit_start`): `mk_limits` carries a per-joint `position_start_tolerance`, and
   `mk_validate` / `mk_plan_create` widen a claimed bound to the trajectory's first state when it rests outside by no more
   than that, the bound reaching the start and no further; `mk_limits_admit_start` is the same rule for callers that check
   samples before a trajectory exists (`ValidationLimits.admittedBounds`, used by `ProgramCompiler.checkJointPosition`).
   `ExecutionPlan.create`, the runtime's appended-path validation and the RKD6 device compiler only pass the plan's start
   tolerances; their copies and the `1e-6` fallback (a zero tolerance now admits nothing) are deleted. Test:
   `motionkit/native/tests/validation.cpp start_at_limit_admitted_once`. Root cause: `HomingAxis.home` was the authored home,
   which defaults to the lower limit, but homing knows the rest only to within its precision (the switch repeatability,
   `positionTolerance`; the cycle accepts any rest within it), so half the runs rested outside the claimed bound. A homed
   axis now rests that precision inside the travel (`HomingAxis.home`, clamped on both limits, tested in `HomingTests`);
   the authored home coordinate is unchanged. The rule stays in the validator for any other plan that starts a hair
   outside (a stop, drift): the toolpath motion tests pass with the rule disabled and the old home too, so the
   homing case is no longer what keeps it needed.
2. TOPP-RA's forward-pass failure: replace the lowered-speed retry subclass and the string-matched "path.time failed"
   halving with a real fix of TOPP-RA's forward-pass numerics (a fork and an upstream pull request). **Fixed in our
   fork; retry and halving deleted, one failure left (below).** The submodule now follows github.com/tritao/toppra
   branch `materia` (8e9fe00 on 0.6.10: three fix commits and a changelog line, the same as upstream pull request
   hungpham2511/toppra#302, branch `fix-degenerate-stages`; when it is merged, follow upstream again). Root cause, from 40 failures recorded on the gate and replayed
   bit for bit against stock 0.6.10: the forward pass clamps each state into the next controllable set, so wherever the
   profile follows the upper boundary (decelerating into a corner, holding an arc's limit) the state is exactly the
   set's upper bound, the optimum of the same stage LP the backward pass solved, usually a vertex admitting a single
   control. Re-solved with x fixed (`algorithm/toppra.cpp` computeForwardPass), rounding left it none, three ways:
   (a) Seidel's 2D loop (`solver/seidel.cpp` solve_lp2d) moves onto the line of any constraint violated by more than
   an absolute 1e-13 while its 1D problems accept crossings of 1e-10 relative to the line parameter, so it moves onto
   lines it already enforces (a duplicate row); there the other CoreXY motor's row, equal to the last digits, projects
   to a bound whose slope is the difference of slopes (4e-10) and whose offset is one rounding of 77.1 (1.4e-14),
   placing it anywhere on the line (this also failed the backward pass, ERR_FAIL_CONTROLLABLE, on the test tower);
   (b) a joint at rest along the path has a path-acceleration coefficient that is rounding (1.8e-16) next to its
   curvature (5e4), so at the vertex its row bounds the control arbitrarily (empty ranges of 0.3 to 115 in path
   acceleration); (c) the backward pass's own vertex overshoots by its tolerances (residual 1e-14 to 1.6e-8). Fixes:
   Seidel takes a constraint violated within 1e-10 of its terms as satisfied (a); `computeControllableSets` keeps the
   control that reaches the next set from each upper bound and the forward pass uses it when the stage fails there
   (b, c); a step that ends on the next upper bound to within its own rounding lands on it, else the next stage sits one
   rounding below the vertex (seen on the 8 mm gear). Four recorded spans are tests in TOPP-RA's suite
   (`test_degenerate_stages.cpp`, each failing without its fix).
   Before (retry subclass and halving): the gate had 146 forward-pass failures rescued by the retry; the test tower 19
   halvings (242 sections). After: none; the tower plans in 223 sections, the gear (8 mm) and cube unchanged in planned
   time (2356.9 s, 870.3 s, tower 471.5 s). The gate's extruder check then failed (`path.time failed`, op 15), which
   the halving had been hiding and which was not TOPP-RA's: `mk_time_path` rounded each stage down to whole
   nanoseconds and derived its acceleration from that, and on the 1.1 micrometre spans of a corner arc a stage lasts
   about 1 us, so 1 ns of rounding moved the path acceleration by about 60 m/s^2 (stage 16007: -59.7 against TOPP-RA's
   0.53), a joint acceleration 4.9x its limit that stretching and softening could not cure; each stage's speed error
   was handed to the next with its sign flipped, so it never died out. The app's `PROJECT_SOURCE_ONLY=printer`
   (SmallTab, op 15, layer 3) failed the same way. **Fixed** (`motionkit/native/src/path.cpp`, `stages_from_profile`):
   only knot times are rounded, each from the exact time since its span's start; every knot keeps TOPP-RA's speed, so
   a stage's acceleration is TOPP-RA's to 1 ns over its duration (relative); a reference stage shorter than a
   nanosecond merges into the next; each path sample is reached exactly at a knot (lowering needs it there), the span's
   duration rounding up and its interior knot speeds scaled by 1 - c t (L - t), the least-acceleration correction,
   at most about 6 v (1 ns) / L^2. The captured requests now pass at once: gate op 15 at 10.47 m/s^2 against 12.56
   (was 61.5, 24 attempts, 12 s; now 0 adjustments, 0.6 s), the app's SmallTab request the same, the gear and tower
   requests unchanged. Laws now cruise exactly at a limit where the old ones ended a 0.2 % stretch below it, which
   showed that `mk_merge_segments` checked merged quintics only at sample points and let a plateau rise 1e-6 above
   the original: the merged form of the gate's step moves broke 0.25 m/s by 1.9e-7 and was refused. Merging now bounds
   each quintic's speed and acceleration at its stationary points and allows a plateau only rounding (1e-9 of the
   limit) above the original. Tests: `motionkit/native/tests/path.cpp` (a 1 mm arc sampled every 1.5 um, which the
   old law missed by 3.7x; a step move cruising at its limit, merged). Gate: passes, planned 1047.2 s (unchanged;
   294 s of CPU with the native rebuild); gear 2356.9 s, cube 870.3 s, tower 471.5 s, all unchanged (404 s of CPU for
   the three). Item 4 would not have avoided it, only made it rarer: with an arc sampled at the same count, a larger radius lengthens its stages only as the square root of the
   radius at the speed the radius allows, so the error (2 v (1 ns) / stage^2) falls as 1 / sqrt(r), and 4x the
   radius still leaves 2.4x the limit; the error also hit any micrometre spans, not only arcs.
3. Replace the `MAX_SECTION_PRIMITIVES` forced rests with timing windows that carry boundary speeds (AM-D4's original
   design).
4. Derive `CornerBlender.MIN_RADIUS` from the machine (an arc whose v^2 / r exceeds the acceleration cannot be followed)
   or from the binding, instead of a global 50 micrometre constant, and stop the slicer emitting micro-segments.
5. Replace `Std.isOfType(solver, AxisKinematics)` (which skips the clock-sampled task-space check) with a capability
   the solver declares (exact, affine kinematics). **Done.** `KinematicsSolver.affineMap()` returns
   an `AffineJointMap` (joint = offset + a column per tool coordinate) when the solver is exactly affine, else null;
   `AxisKinematics` reads it off its axes and couplings, every other solver answers null. `ProgramCompiler` skips the
   clock-sampled task-space check by that capability, and item 10's fast path reads the same map for its closed-form
   limits. The planner factory (`defaultJointPathPlanner`) still chooses `AxisJointPathPlanner` by type: that is a class
   of planner for one solver family, not a decision about which checks to skip. Test: `KinematicsTests.testAxisKinematics`
   (the map reproduces IK and the differential, with a coupled follower; a joint-space solver declares none).
6. RKD6: send the step interval as an integer tick count so host and device cannot disagree (`PulseRate` now mirrors
   the device's f32 arithmetic). **Done.** The integer was already on the wire: wire revision 14 (`e7779c599`,
   `SESSION_BEGIN6.min_step_ticks`, `robotkit.runtime.StepTicks`) replaced `max_rate`, and `PulseRate` no longer exists; the
   device (`step_generator.rs`) only counts ticks. What was left was one mirror of the device's f32: the binding's steps
   per unit (an f32 on the wire) was rounded again in `StepTicks.deployed`, the compiler's rate check and a test. The
   binding now quantises its steps per unit once (`StepTicks.wire`) and plans, deploys and checks with that value, so
   `deployed` and the compiler's `static_cast<float>` are gone; the binding tests assert the held value is the wire's.
7. Decide whether the printer's axes take the motors' full rate with motor-space rows (see the AM5 notes on CoreXY motor
   limits), against the cube's print time.
8. The corner arcs still lower to tens of thousands of sub-millisecond segments (`SmallTab`: 18,900 of 25,580 under
   1 ms after merging): an adaptive TOPP-RA grid by span length and curvature (the fixed 32 per span is calibrated with
   the 0.8 margin; see the RobotKit Paint path that failed when it was coarsened), or arcs that are not made smaller
   than the machine can follow (item 4).
9. The first plan of a print compiles synchronously inside a tick (13.8 s for `SmallTab` in the app): plan it on the
   planner thread, as a mission's programs are, and start the print when the first plan is ready.
10. A fast path for machines whose kinematics are exact and affine (the capability of item 5: Cartesian, CoreXY,
   routers, gantries). TOPP-RA costs a grid of 32 stages per span and a small LP per stage; Klipper and Marlin plan the
   same machines with constant-cost trapezoids and a look-ahead pass, and the benchmark's Marlin-style reference is
   only 0.3 % faster than our plan for the gear. On an affine machine the limits along a straight piece are constant
   and along an arc depend only on its radius, so a piece's speed and acceleration bounds come in closed form from its
   direction against every joint row and the motor-space rows (a CoreXY motor bounds `|vx ± vy|`), and centripetal
   acceleration bounds an arc (`v^2 / r`). Plan the C1-blended path (still no velocity jumps, AM-D4) with a look-ahead
   pass over jerk-limited (S-curve) profiles in path distance, emit the same polynomial segments, and keep the plan
   checks (torque, stall margin, deviation) unchanged. TOPP-RA stays for every other machine and is the reference the
   fast path is tested against. Done when, on the printing gate, the planned time is within 0.5 % of TOPP-RA's (or
   shorter), no joint or motor limit is exceeded, the plan checks find nothing new, and planning is at least 10x faster
   than with TOPP-RA (the worst 10 s window included), aiming at real time on a Raspberry Pi class CPU.
11. The general path (TOPP-RA) after item 2: every machine that is not affine keeps it (arms, the welder, the tending
   cell, non-planar printing). AM5's profile predates AM8's segment merging, the rebase and the WIP planner changes, and
   the gate's peak memory has grown to 11.4 GB, so profile again first (`perf` and the allocation census, the printing
   gate and a robot-arm load), then fix the top costs. Expected, to be confirmed by the profile:
   - the fixed 32 stages per span (each stage is an LP in both passes and a lowered segment): an adaptive grid by span
     length and curvature (item 8), checked against the 0.8 margin that a uniform coarsening once broke on a Paint path;
   - lowering to one segment per stage and then merging (`SmallTab`: 163,392 -> 25,580): lower straight to long
     quintics where the path allows, so neither the surplus segments nor the merge's searches are paid for;
   - sections start and end at rest (layer changes, turns above 150 degrees, every 4,000 primitives), so they are
     independent: plan them on several cores, which matters most for the worst 10 s window;
   - count the retries that remain per load (smoothing re-lowerings when a lowered trajectory breaks a limit; the
     forward-pass retries and halvings are gone with item 2), since each one is a whole re-plan;
   - what keeps plans and trajectories alive long enough for an 11.4 GB peak, and the collector's cost of marking it;
   - the share of the Haxe-side checks (task-space, torque and deviation plan checks) that is left after AM5.
   Done when the printing gate and the robot-arm load plan measurably faster (numbers before and after, processor
   time) and use less memory, the planned times and the plan checks' results are unchanged within their tolerances,
   and the full suite passes.

**Later (not in this run).**
- Bed mesh probing (with `ContactSearch`), Z tilt with independent screws, skew correction.
- Pause and filament change, runout sensor, cancel object, power-loss recovery.
- STL/3MF import with mesh slicing, supports, variable width, multi-material.
- Real hardware: Nucleo heater pins, TMC UART configuration, a Klipper-known board in the model.
- Robot-arm and non-planar deposition, WAAM, hybrid print-then-mill.

## Order

```
AM0 ── AM1 ── AM2 ──┬── AM3 ── AM4 ── AM5 ──┐
                    └── AM6 ── AM7 ─────────┴── AM8 ── AM9 ── AM12
                         AM6 ── AM10 ── AM11
```

- AM4 can use a stand-in machine if AM6 is not done. Its "before" numbers are for the current
  planner.
- AM8 needs AM5's cornering only for speed, not for correctness. If AM5's gate fails, AM8 still
  runs and records its planning ratio.
- AM10 and AM11 need only AM6 (and AM2's writer).

## Progress

| Step | State | Commits |
|---|---|---|
| AM0 | done: worktree and baseline | |
| AM0b | done: main green, router baselines recorded | 2fbea0683, ae6e95ec5, cd29b1993, dfa6e2940, 2611087d9 |
| AM1 | done | see the commit below |
| AM2 | done | see the commit below |
| AM3 | done | see the commits below |
| AM4 | done | see the commit below |
| AM5 | done: gate passed | see the commit below |
| AM6 | done | see the commit below |
| AM7 | done | see the commit below |
| AM8 | done | see the commits below |
| AM9 | done | see the commit below |
| AM9b | planned | |
| AM10 | done | see the commit below |
| AM11 | done | see the commit below |
| AM12 | done | see the commit below |
| AM5b | in progress: items 6, 1 and 5 done | see the commits below |

## Next: a large-format pellet printer (PL0 to PL12)

Planned 2026-10-12, to start after AM0 to AM12, AM9b and the AM5b items needed for landing are done and merged. It builds
on everything above: the extruder joint, heaters, deposition, the slicer, input shaping and the trim side of the CNC
router.

**Goal.** A gantry pellet printer of about 1.5 x 1.2 x 1 m that prints large parts from dried, fibre-filled pellets,
then trims them to CAD with a spindle on the same gantry, simulated end to end (print, cool, deposit, trim) and driven
by our stack. The hybrid print-then-trim cycle is what our stack does that Klipper-class tools do not.

**Reference extruder: IEMAI Pellet Ext P10** (https://www.iemai3d.com/index.php/pellet-3d-extruders/). It is 10 kg and
makes 10 kg/h. The screw is 34 x 363 mm (L/D 10.7) with a 60 rpm maximum, driven by a 600 W servo (1.91 N m) through a
1:50 planetary reducer, which gives about 2.8 g per screw turn. It has 3 heater zones and a water-cooled feed throat, takes
2 to 10 mm nozzles and prints 0.5 to 5 mm layers, at 220 V. Its kit adds a 12 kg controller (servo driver, heater
controllers, cooling, feed control, compressed air at 0.6 to 0.8 MPa), a 25 kg hopper dryer and a 65 L compressor. The
vendor's STEP files (extruder, controller, dryer, compressor) are public on Google Drive but carry no licence: they stay
in `materia-cache` as display and collision geometry, are not committed, and the part model carries the specification.
In the extruder's STEP the nozzle tip is at the origin and an 84 x 132 x 8 mm mounting plate sits at z of about 287 to
419 mm. The CAD volumes overstate the mass (the barrel is one lump), so the mass is the specification's. Not documented:
how the controller takes screw speed (pulses, analog or fieldbus); ask the vendor. Alternatives: RobotDigg PE3000S
(3 kg/h, 7 kg) and PEA15000 (15 to 20 kg/h, 42 kg); IEMAI L35 (20 kg/h, 180 kg, with a melt pump and a melt pressure
sensor) for a later, larger machine.

**Decisions.**
- PL-D1. Cartesian gantry, not CoreXY. The head (10 kg plus a spindle) dominates the moving mass, belts are too soft over
  1 to 1.5 m, and milling needs stiff drives both ways. Y is a beam driven at both ends (two servos on racks, kept square:
  `GantrySpec.dualY` and its racking tolerance), X a carriage on the beam (rack and pinion or ballscrew), Z a ballscrew
  on the gantry. Servos throughout, with motor-derived limits.
- PL-D2. The screw is the extruder joint of AM-D3, driven by path distance through mass per screw turn, with its motor's
  limits through the reducer. Its flow responds over seconds (melt pressure), so the plan gets a screw-flow model in the
  simulation (lag, ooze, pressure) and a feed-forward term in the planner (pressure advance at this scale). Starts and
  stops are planned out where possible (continuous paths, coasting, wipes) because retraction barely works.
- PL-D3. The layer-time window is a planning constraint: a layer printed too fast slumps, too slow does not bond. A
  per-layer thermal model (bead size, length, convection, material) gives the window; the planner slows a short layer
  or the program waits. It is checked in simulation, not only asserted.
- PL-D4. Hybrid from the start: the slicer prints oversize by a trim allowance, the CAM trims the printed stock to CAD
  with the router's milling stack, on the same dexel stock the deposition fills.
- PL-D5. The extruder controller is a device peripheral: our device drives the screw (step and direction to its servo
  driver, if the vendor confirms) and the heater zones' targets; the vendor box's own heater loops are modelled, not
  replaced, unless we take over the heaters (then AM7's heater module runs three zones).
- PL-D6. Cell equipment (dryer, compressor, conveying) is ProcessKit sequencing and scene geometry, not motion.

**Test parts.** A tall thin-walled box 600 x 300 x 400 mm with a single bead wall (layer time and slump); a mould-pattern
segment printed oversize and trimmed (hybrid accuracy); a ribbed panel (sparse infill sized to the bead).

**Steps.**
- PL0. Survey and worktree: confirm the P10's drive interface and controller I/O with the vendor (or record an
  assumption); licences of the STEP files; references for large-format slicing and layer-time control (ideas only,
  licences checked); baseline suite.
- PL1. `PelletExtruder` part (`machinekit.printing`): screw, barrel zones, reducer, servo, feed throat, nozzle set, mount
  plate and tool frame, mass from the specification, assumed values marked; vendor STEP as optional display geometry
  from `materia-cache`. Controller, dryer and compressor as cell equipment parts.
- PL2. The machine: the gantry assembly (dual-driven Y on racks, X, Z ballscrew), vacuum table, the P10 and a trim
  spindle beside it, homing, a Start-page entry; motor-derived limits including the moving mass of the head.
- PL3. Screw-flow model: melt-pressure lag and ooze in the simulation (first order to begin with, parameters per
  material and nozzle, assumed), deposition rate from it, and a test that the deposited flow follows a step in screw
  speed with the modelled lag.
- PL4. Planning the screw: feed-forward on the screw joint from the flow model, start and stop handling (coast, wipe,
  prime), a check that the deposited bead width stays within tolerance along a path with speed changes.
- PL5. Thermal layer model: per-layer cooling from bead size, length and convection; the layer-time window per material
  as data; layer temperature at the next layer's start recorded per layer.
- PL6. Layer-time control in the planner: slow a layer that is too short (down to a floor), then wait; fail a program
  whose layers cannot meet the window. Test on the thin box: every layer in its window.
- PL7. Slicer for large beads (`camkit.additive`): single and double bead walls, continuous spiral and minimal-start
  paths, sparse infill sized to the bead, the trim allowance, layer-time awareness (reorders or merges regions).
- PL8. Deposition at this scale: bead footprints in the dexel stock at 5 to 15 mm beads, slump as a function of layer
  temperature from PL5 (a simple model), checks of volume, bounding box and wall thickness.
- PL9. Hybrid trim: CAM roughs and finishes the printed stock to CAD with the router's milling stack, the extruder
  parked or retracted, tool change to the spindle; check the trimmed part against CAD within tolerance.
- PL10. Cell: dryer and compressor as equipment, pellet conveying and hopper level as signals, the print-then-trim cycle
  in ProcessKit, heater and feed interlocks (a cold barrel never turns the screw, an empty hopper pauses).
- PL11. Input shaping and gantry dynamics: the gantry's structural modes in the simulation (beam bending, rack
  compliance), shaping tuned to them, residual vibration and the head's deviation checked at print speeds.
- PL12. End to end: the thin box and the mould segment printed, cooled, deposited and trimmed in the app; numbers
  recorded (print time, layer temperatures, slump, volume, trim time, accuracy).

**Later.** Robot-arm or track-arm deposition (non-planar), the L35 class with a melt pump and closed-loop melt pressure, an IR camera in the simulation, real hardware.

**Order.** PL0 -> PL1 -> PL2 -> (PL3 -> PL4) and (PL5 -> PL6) in parallel -> PL7 -> PL8 -> PL9 -> PL10 -> PL11 -> PL12.
