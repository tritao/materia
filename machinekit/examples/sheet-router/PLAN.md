# Full-sheet CNC router plan

**Goal.** A full-sheet CNC router for wood, wood panels and plastics, designed in materia from
real catalogue-class parts and **built for real**. It runs on our own control stack (MotionKit,
RobotKit runtime, RKD6, our board), not LinuxCNC. In the editor it cuts a nested sheet the way the
real machine would, with tool changes, hold-down and dust extraction modelled. On the shop floor
the same programs, limits and checks drive the physical machine.

1. A moving gantry rides on a welded steel frame. Y (the long axis) has two drives on helical
   racks; X uses the same rack; Z is a ball screw. All four axes are AC servos.
2. An ISO30 automatic tool-changer (ATC) spindle and a linear tool rack are fitted from the start,
   with a fixed tool-length setter. The tool change is a machine sequence with sensors and fault
   handling, not an operator handshake.
3. A zoned vacuum table holds a bleeder spoilboard. Extraction goes through a dust shoe that lowers
   for cutting and lifts for tool changes.
4. Sheet-goods CAM takes DXF or CadKit parts, nests them on the sheet and cuts them. Profiles get
   tabs or onion skins, leads and ordering, with feeds from a chip-load model. The simulation
   checks that every part freed by a cut is still held.
5. Our board generates step and direction pulse trains for the servo drives in hardware, reads
   their encoder outputs back and handles machine I/O. A hard-wired safety chain stops the machine
   independently of all software.
6. The machine is built from a fabrication pack (BOM, cut lists, plate DXFs, weld and alignment
   procedures) and commissioned against acceptance tests. Measured cycle times and following
   errors are compared with the simulation.

**Target work:**
- nested cabinet and furniture parts from 18 mm plywood and MDF;
- signs and pockets in solid wood;
- acrylic, HDPE and PVC foam sheet;
- spoilboard surfacing.

Aluminium is not a target. Occasional light cuts in aluminium composite panels and thin sheet are
allowed, but nothing is sized for them.

**Sheet sizes:** typical EU sheets fit the **work area** of 2600 × 1600 mm (Y × X):
- 2500 × 1250 plywood;
- 2440 × 1220 MDF and OSB;
- 1525 × 1525 birch plywood;
- 1525-wide sheets up to 2600 long.

The length is a parameter. 3050 × 1525 needs Y = 3150 mm, which changes only the frame, the racks,
the rails, the table segments and the cable chain (SR-D1).

The work spans kits:
- `machinekit/haxe/src/machinekit/routing/` (new): ATC spindle, tool rack and forks, tool
  setter, dust shoe, vacuum table, zones and spoilboard;
- `machinekit/.../motion`, `transmission`, `gantry`, `structural`, `power`, `control`: servo
  axes, helical racks, reducers, rail sizes, cable chains, steel gantry, electrical and safety
  parts;
- `machinekit/examples/sheet-router/`: the `SheetRouter` assembly, checks, preview, project,
  fabrication pack;
- `camkit` / `toolpathkit`: sheet operations, nesting, router tooling, feeds;
- `cadkit`: DXF import, plate drawings;
- `stockkit` and `app`: stock at sheet scale, freed-part hold-down check;
- `robotkit` device protocol, board firmware and runtime: hardware step and direction output,
  encoder feedback, drive I/O, spindle, ATC and vacuum channels, first hardware release of RKD6;
- `motionkit`: ATC sequence, probing and tool measurement.

## What exists

Surveyed 2026-10-19 on local main 37049c103.

**Machines and parts (main).**
- `gantry/Gantry.hx` (`Gantry(spec:GantrySpec) extends AxisBuilder`):
  - travel X/Y/Z, `dualY`, `racking`, a `head` (none, C or CA), `GantryDrive { Screw | Belt |
    Rack(moduleSize, pinionTeeth, width) }`;
  - it builds an aluminium T-slot frame, a beam on feet, MGN12/HGR15 rails and a `RobotFlange`;
  - **it is hard-wired to steppers.** Axis motors are `NemaStepper` (`AxisBuilder.hx:238` casts to
    it), drivers are TMC2209/DM542, brackets are aluminium boxes. There is no gearbox stage and no
    steel structure.
- `AxisBuilder`:
  - `slide`, `driveScrew`, `driveRack(axis, id, motor, pinion:SpurGear, pose, rackSide, rackId,
    rack, rackParent, driverForMotor)`, `rackPlacement` (phases the rack to a pinion tooth),
    `turnWithBelt`, `addHomeSwitch`.
  - `MachineAssembly`: `addMotor(…, ?gearbox, ?processVelocity)`, `addEncoder`,
    `bindMotorBrake`, `addCylinder`, `addTripSwitch`, `homeAfter`, `addAtSpeed`, `addPresence`,
    `addTransmission`, `addBomItem`, `billOfMaterials()`, `massProperties()`.
- `transmission/Rack.hx` and `SpurGear.hx` give straight involute teeth only. The rack has an
  assumed efficiency of 0.95 and is "rigid". `motion/Gearbox.hx` is a generic cylinder
  (ratio, efficiency) with no backlash or stiffness.
- Rails: MGN9C, MGN12C/H, HGR15 (`motion/LinearRailSystem.hx`, assumed). Screws: SFU1605 with
  BK12/BF12. Servos: `ServoMotor` GENERIC-SERVO-50 to 750 W (LV and HV windings). Drivers:
  `MotorDriver` AC-SERVO-AMP, 400V-SERVO-AMP and others, unverified. `ProximitySwitch`,
  `LimitSwitch`, `EncoderPart`, `ShaftBrakePart`.
- Spindles:
  - `milling/SpindleCartridge` + `Er20Holder`;
  - `Bt30Cartridge` (a packaging study), `Bt30SpindleMotor` (2.2 kW assumed);
  - `MillPowerDrawbarPart` and `MillDrawbarLayout.theoreticalPushForce`;
  - `MillSpindleHead` is "ATC-ready packaging"; its release joints are inspection motions only.
- Structure:
  - `structural/` `RectTube`, `RoundTube`, `Angle`, `Channel`, `FlatBar`, `TSlotExtrusion`;
  - `FrameAssembly` (`member(name, start, end, profile)`, `cutList()`);
  - `WeldedFrame` with weld seams through `welding/Weldment`, used by
    `examples/picking-station`;
  - `MillLevelingFoot` (concept, no load rating).
- FEM: `examples/bench-mill/fem` (Gmsh + CalculiX C3D10, static compliance budget, reactions,
  modes). **Mill-specific**: `MillFemExport.hx` hard-codes the mill's casting part ids.
- Pneumatic and vacuum: `AirSupply`, `PneumaticManifold`, `SolenoidValve`, `PneumaticCylinder`,
  `RoutedHose`, `VacuumGenerator` (ejector), `SuctionCup`, `VacuumControlValve`,
  `VacuumPressureSensor`.
- Power and control: `power/` (`AcMains`, `ThreePhaseRectifier`, `Inverter`, …),
  `control/CncController(axes=4, inputs=16, outputs=16, valves=8)` (a CAD/BOM cabinet with no
  wiring semantics), `RobotController`.
- Build outputs:
  - BOM: `component/Bom.hx`, `AssemblyInventory.billOfMaterials`;
  - STEP: `Shape.exportStep`;
  - drawings: `cadkit/drawing/Drawing2D.of(FlatPattern)` → `Dxf.write` (R12) and `Svg.write`,
    for flats and panels only;
  - cut lists: `FrameAssembly.cutList()`.
- Templates:
  - `examples/cnc-router/CncRouter.hx`: desktop, 300 × 300 × 80, 4040 frame, MGN12, NEMA 23,
    dual-Y screws, home switches, spoilboard plate, `EndMill`/`TwistDrill`;
  - `examples/gantry-picker`, `examples/gantry-welder`;
  - `examples/overhead-arm` uses `Rack(2, 20, 20)`.

**Homing and squaring (main, `GANTRY_PLAN.md`).** G10 switch parts and readings, G11 referenced
state and `HomingCycle`, G12 dual-drive squaring with a one-sided hold and a 1 mm racking
regression, and G13 device homing on RKD6 are all done, **in simulation and on the virtual device
only**.

**Programs and CAM (main).**
- `camkit` `CamJob(safeZ, spindleRpm, …)`:
  - one spindle speed per job, always M3, no coolant;
  - operations `profile` (outside, inside, on; step-down; vertical plunge), `profileFace` (holes
    first), `pocket` (offset rings or scanlines, ramped entry ≤ 10 %), `pocketFace` (islands),
    `drill` (single plunge);
  - arc fitting and G64 blending.
- Absent from CamKit: tabs, onion skin, leads, helical entry, ramps on profiles, climb or
  conventional choice (direction follows vertex order), surfacing, V-carve, chamfer, peck drilling,
  per-operation speed, tool selection. Input comes from sketches and horizontal CadKit faces;
  `CamSheetProfiles.fromPlan` handles rectangles only.
- **No nesting** anywhere. `manufacturingkit/.../sheet/SheetCutPlan` holds hand-authored
  guillotine plans (rectangles, kerf, 0/90°).
- **DXF and SVG are export only.**
- `toolpathkit`:
  - `ToolpathOp` (`Move`, `MachineMove`, `Spindle`, `Coolant`, `ToolChange`, `ToolLengthOffset`,
    `Dwell`, stops);
  - `MachineBinding` takes exactly three axes;
  - `Tool(number, length, diameter, ?cutter)`, `ToolLibrary` (a map), `CutterProfile` (flat, ball,
    bull nose, vee, tapered ball) with shank and holder zones;
  - no flute count, helix, up-cut or down-cut, chip load or feeds model.
- `cnckit`: a LinuxCNC dialect reader and writer (G0–G4, G17–19, G28/30, G40–43, G53–59, G61/64,
  G73/81/82/83, M0–M9, M30) and a `CncController` with work offsets and tool lengths. G38, G10 L,
  G92 and M64/65 are rejected or absent.
- Motion: `ToolpathLowering` joins moves into paths up to barriers; `ProgramPlanner` plans about 1 s
  ahead on a worker.
- `PlanCheck` checks:
  - drive torque against the stepper pull-out curve or servo peak/rated torque;
  - step slip;
  - compliance and backlash deviation;
  - a flat cutting-force allowance.
- `MachiningRun`: feed hold, restart at a line, continuation with tool reload. `CncPanel`:
  5–200 % override.
- Tool change: `ToolChange` lowers to `WaitInput("cnc.tool_change.N")`, answered by a simulated
  operator in `app/CncProgramPlayer.hx`.
- Channels (`ToolpathChannels.hx`): `spindle.direction/speed/at_speed`, `coolant.*`,
  `cnc.operator.resume`, `cnc.tool_change.N`.

**Stock and app (main).**
- StockKit is a tri-dexel on one uniform lattice in 16 × 16 ray tiles. It costs 48 B per interval
  plus 12 B per ray (`stockkit/docs/PLAN.md`); the largest measured case is 200 × 200 mm at
  0.1 mm, 244 MB.
- `app/MachiningStock` sweeps the tip each tick with spindle +Z, plus a target stock on the same
  lattice. `STOCK_SPACING = 0.0005` is hard-coded (`CncProgramPlayer.hx:109`).
- Estimate, not measured: a 2600 × 1600 sheet costs about 1 GB per stock at 0.5 mm Z-only and about
  250 MB at 1 mm, before the target and snapshots.
- `SceneArtifactMachining` (`projectkit/.../SceneArtifact.hx:697`): one G-code program, three
  prismatic axes, one work offset, one target. The machining JSON is **capped at 8 MB**
  (line 846).
- Materials: `plywood-birch` (look and density only). There is no MDF or acrylic, no cutting-force
  or spindle-power model, and no dust or chip model.

**Device and hardware (main).**
- RKD6 (`robotkit/runtime/DEVICE_PROTOCOL.md`, `schema/device_wire6.wire.idl`):
  - protocol 16, up to 64 actuators;
  - polynomial segments (degree ≤ 5) evaluated on the device, with commit and replace;
  - clock sync (`clock_estimator6.cpp`);
  - `State6` reports position, velocity, effort and step count. **There is no encoder count.**
  - up to 64 inputs with edge capture, 32 output channels with safe values and a stop policy,
    `Sensor6` slots, homing messages;
  - transport is **serial only** (`device_serial_endpoint.cpp`). The doc says the wire format
    freezes at the first hardware release, and none has shipped.
- Device-side safety (`src/scheduled_core.rs`): `StopReason { Underflow, LinkLost, Abort, Stop,
  EmergencyStop, DualDriveSkew, ProcessFault }`, a link-loss lease, safe values on stop. The
  dual-drive skew guard and squaring relaxation live in `StepGenerator`.
- Host-side safety: limit-switch faults, referenced gating, following error (`EncoderMonitor`) and
  `HomingCycle`.
- Board: `robotkit/device_protocol/boards/nucleo-g474re` is a **UART stub**:
  - 921600 baud, 2 joints, degree 1, 8-segment queue;
  - `step_pulse`, `step_count`, `set_digital` and `set_analog` are no-ops;
  - inputs are marked "hardware unverified".
- Step generation is a software tick at 40 kHz (`src/step_generator.rs`). Timer and DMA step output
  is planned in `TRANSMISSION_PLAN.md` X6, not done.
- Absent: quadrature input, CAN, EtherCAT, RS485/Modbus RTU, servo-drive interface (enable,
  alarm, ready), MCU watchdog, hardware e-stop input, STO.
- Modbus TCP exists host-side only (`processkit/.../modbus`). The EtherCAT stub is
  `machinekit/control/erob/ERobDriveInterface.hx`.
- **Nothing in the repo has driven a real motor.** Multi-board synchronisation does not exist.
- `robotkit/plans/TOOL_CHANGE.md` is about robot end-effector exchange (`ToolChangeTransaction`,
  `ChangerLockObservation`), not spindle ATC.

**Conventions.** Plan format and working rules as in `examples/lathe/PLAN.md`. Registration:
- `machinekit/scripts/build-examples`; `machinekit/tests/haxeon.json` source roots and
  `MachineKitSmoke.hx`;
- `app/src/editor/ExampleCatalog.hx`; `ExampleBrowser.categoryOf` ("Machining");
- `app/tests/src/tests/StartPageTests.hx:58` asserts **exactly 3 machining families**, so it must be
  updated;
- `app/haxeon.project-source.json`, `app/tests/project-source/haxeon.json`,
  `ProjectSourceTests.hx` (`suite`, the `router` group).

## What is missing

**Parts:**
- HGR20/25 rails and blocks; helical racks and pinions with a preloaded pinion mount;
- planetary reducers with backlash and stiffness; servo axes in `Gantry`/`AxisBuilder`;
- a steel gantry beam and side plates; cable chains;
- the ISO30 ATC spindle, tool holders, collets, tool forks, rack and rack cover; a tool-length
  setter;
- the dust shoe and its lift; the vacuum pump, plenum, zones and zone valves; the spoilboard;
- the VFD, safety relay, e-stop devices, contactors, STO wiring, cabinet hardware and levelling
  feet with load ratings.

**Control:**
- hardware step and direction pulse trains; encoder feedback; drive enable, alarm and ready;
- analog spindle speed; the ATC sequence; vacuum zones; dust-shoe and extractor interlocks;
- tool-length measurement; a hardware e-stop input and watchdog;
- the first frozen RKD6 release.

**CAM:**
- DXF import and nesting;
- tabs, onion skin, leads, cut direction, ordering for small parts, per-operation speed, peck
  drilling, surfacing;
- router bits with flutes and helix, a chip-load feeds model, cutting forces for wood and plastics.

**Simulation:** stock at sheet scale; freed-part hold-down; programs larger than 8 MB; more than
one work offset.

**Build:** plate drawings for outsourcing, weldment drawings and a jig layout, the rail alignment
procedure, the electrical schematic and wire list, the commissioning procedure.

## Decisions

**SR-D1. Envelope: work area 2600 × 1600 mm (Y × X), Z 250, length parametric.**
- Y is the long axis carried by both gantry sides, and X is along the beam. This matches
  `GantrySpec`.
- The gantry span is the expensive parameter, because the beam and side plates and their
  stiffness grow with it. The length only adds frame, rack, rail, table and cable chain, so Y is a
  `SheetRouterSpec` parameter: 2600 by default, 3150 for 3050 × 1525 sheets. Every check runs at
  both.
- Y travel = work area + the tool-rack zone at the rear (SR-D5), about 2800 mm by default.
- Z: 250 mm travel, 150 mm clearance from the gantry to the spoilboard top. Assumed; SR3 confirms
  it against the longest tool plus the rack height.
- **Rejected:**
  - a 1300 mm span (4 × 8 only): it cannot take 1525-wide sheets;
  - a moving table: it doubles the footprint at this size.

**SR-D2. A welded steel frame. Rail seats are set by replication against references, not
machined.**
- The base is rectangular steel tube welded in a jig (`WeldedFrame`), with levelling feet and
  integral table supports. Y rails sit on steel flat-bar pads welded to the long sides.
- We cannot machine 3 m surfaces, so we follow the bench mill's replication route (its README,
  "Precision interfaces"):
  - the pads are left low;
  - a thin epoxy layer is cast under each rail pad;
  - **height and twist** are set with a precision level and a straightedge;
  - **straightness and parallelism** come from a reference edge: the first rail is aligned to a
    tensioned wire or a laser line, and the second to the first with a gauge bar and a dial
    indicator on a block;
  - self-levelling epoxy gives flatness to gravity, not straightness, so straightness always comes
    from a reference.
- The frame is welded in a stated sequence and settles before replication, so weld distortion is
  taken out by the replicated layer. Full stress relief of a 3 m weldment is not available to us.
- Filling the tubes for damping (sand or a polymer concrete) is decided by SR5 modal results, not
  up front.
- Gantry side plates, Z plate and bearing plates are laser-cut steel or aluminium plate with
  machined faces, **made by a service** from our plate DXFs (SR6).
- **Rejected:**
  - a mineral casting (3 m molds and tonnes of casting for a wood router);
  - an aluminium extrusion frame (flex and creep at the rail joints over 3 m);
  - a bolted steel frame without replication (rail straightness would follow the tube).

**SR-D3. The beam and Z come out of a stiffness budget, not a catalogue habit.**
- The default beam is a steel RHS 200 × 100 × 6.3.
- Bending is not the issue. A simply supported estimate gives about 11 µm at 300 N mid-span over
  1.9 m (I ≈ 1.87e7 mm⁴, E 210 GPa). We expect the side plates, torsion from the Z overhang, the
  rail blocks, the pinion preload and the Z stack to dominate. SR5 measures where the compliance
  sits, as the bench mill's FEM does.
- Steel and aluminium have nearly the same stiffness per mass. Steel is cheaper and weldable;
  aluminium arrives with machined faces. The beam material is decided in SR5 from the budget and
  the moving mass (acceleration), and both stay valid `SheetRouterSpec` options.
- Targets (assumed): static stiffness at the tool tip ≥ 2 N/µm in X and Y at full Z extension;
  first gantry mode ≥ 40 Hz.

**SR-D4. All four axes are AC servos through 5:1 reducers. Y and X drive helical racks, Z a ball
screw.**
- Rack: helical, normal module 1.5, helix 19°31′42″. With 20 pinion teeth that is exactly
  100 mm per pinion turn (π·1.5·20 / cos β), so 20 mm per motor turn after 5:1.
- Y uses two 750 W servos, one per side, with skew guard and squaring from G8/G12. X uses one 400 W
  servo. Z uses a 400 W servo with a brake on a 16 or 20 mm ball screw with 5 mm lead (SR4 chooses).
- **The pinion is preloaded into the rack.** The motor and reducer sit on a hinged plate pushed in
  by a spring of stated force, so rack height and pitch errors do not open backlash or bind. The
  model carries the preload spring as a part and its stiffness in the drive's compliance.
- The helix puts axial thrust on the reducer bearings. The rack model carries the helix angle and
  the thrust.
- **Reducer:** a low-backlash planetary (assumed 5 arcmin, about 23 µm at the 15.9 mm pitch radius)
  or a 5:1 timing-belt reduction (no backlash, more compliance). SR4 chooses from the compliance
  and backlash budget; both are modelled.
- The **drives close the position loop** (pulse-train position mode). Our board sends pulses and
  reads the drive's encoder output back (SR-D7).
- Sizing (assumed motor class: 750 W 2.39/7.16 N·m, 400 W 1.27/3.8 N·m rated/peak; η 0.85):
  - Y: about 640 N rated and 1.9 kN peak per side, against a need of about 400 N per side
    (150 kg gantry × 3 m/s² + 300 N cut + friction);
  - X: about 340 N rated and 1 kN peak, against about 60 kg × 3 m/s² + 300 N;
  - 40 m/min is 2000 motor rpm.
- **Rejected:**
  - ball screws on Y and X: 2.8 m and 1.7 m screws whip unless the nut rotates;
  - belts on the axes: they stretch over 2.8 m;
  - straight racks: noisier, with less contact ratio;
  - closed-loop steppers: they lose torque above about 1000 rpm, which is where 40 m/min sits.

**SR-D5. An ISO30 ATC spindle and a linear rack at the rear of the bed from v1.**
- Spindle (assumed):
  - air-cooled ISO30 ATC electrospindle, about 6 kW S1, 18–24k rpm, 400 V three-phase;
  - pneumatic tool release at 6 bar with cone purge;
  - three sensors: clamped, unclamped, no tool.
- It runs on a VFD with a braking resistor and STO. The spindle must not need key orientation for
  the tool change; we choose a spindle whose datasheet confirms that.
- Tool rack (assumed): 8 forks gripping the holder's V-groove, in the rack zone behind the work area
  (SR-D1), with a pneumatic dust cover.
- Holders: ISO30 with ER32 collets and retention knobs of the spindle's form.
- The tool setter is a fixed length-measuring switch beside the rack. A tool's length is measured
  when it is loaded into a pocket and checked on request. A broken-tool check after each tool is an
  option in the sequence.
- The tool table maps tool to pocket, length, diameter, cutter, holder and life count. Pocket
  occupancy is machine state, persisted on the machine and in the simulation. Returning a tool to
  an occupied pocket is refused.
- The change is a **machine sequence**, the same in the simulation and on the machine (SR7):
  1. check spindle stopped (the VFD's zero-speed relay) and raise the dust shoe;
  2. open the rack cover;
  3. move along the fork opening into the current tool's pocket;
  4. release, confirm unclamped, and lift Z clear;
  5. move over the new pocket, descend, and clamp with purge off;
  6. confirm clamped and tool present;
  7. slide out along the fork, close the cover, lower the shoe;
  8. apply the length.
- Every step has a timeout and a named fault: tool lost, pocket not empty, clamp not confirmed,
  cover not open.
- **Rejected:**
  - ER spindle with manual changes (you asked for ATC from the start);
  - a carousel on the gantry (moving mass on the carriage, harder to build);
  - HSK (more expensive and not needed for wood).

**SR-D6. A zoned vacuum table from v1. Every freed part is checked for hold-down.**
- The table is an MDF bleeder spoilboard over a grid plenum split into zones (6 assumed), each on a
  valve. A side-channel blower (5.5 kW and about 300 m³/h, assumed) suits bleeder boards, which
  leak a lot.
- A zone valve is an output channel; the vacuum level is a sensor with a fault threshold.
- Holding force is the vacuum level times the part's contact area, reduced by bleed losses through
  open kerfs. Lateral resistance is that force times a stated friction coefficient.
- As a program runs, the simulation finds parts that the cut has **freed** (connected pieces of
  stock no longer joined to the sheet). It compares each part's lateral hold with the cutting force
  on its last cut. A part that would slide is reported with its name, the program line and the
  remedy: onion skin, tabs, or cutting it earlier.
- Spoilboard surfacing is a standard job, and its measured height becomes the Z reference of the
  table.
- **Rejected:**
  - T-track and clamps only (no nested cutting);
  - pods (a nesting router does not use them).

**SR-D7. Our stack drives the hardware. The board generates pulses and reads encoders, the drives
close the loops, and EtherCAT comes later.**
- The board (STM32G474 on a carrier) evaluates RKD6 segments as now and emits step and direction
  pulses from **hardware timers** (output compare or DMA), differential, at ≥ 200 kHz per axis.
  - At a drive electronic gear of 5000 pulses per motor turn and 20 mm per turn, that is 4 µm per
    pulse and 167 kHz at 40 m/min.
- It reads each drive's A/B encoder output on hardware quadrature timers. Following error is
  measured **on the device** (commanded pulses against counted feedback), with a stated bound per
  axis as a `StopReason`. `State6` gains encoder counts.
- Per drive: enable output, alarm and ready inputs, alarm reset.
- Spindle: analog 0–10 V speed from the DAC plus run, direction, fault, at-speed and zero-speed
  signals, mapped to the existing `spindle.*` channels. VFD Modbus RTU is a later diagnostic, not
  the control path.
- Fast inputs stay on MCU pins with edge capture: homes, setter, probe. Slow I/O (ATC sensors,
  vacuum zones, valves, cover, shoe, extractor, lamp) goes through SPI I/O expanders on the
  carrier at 24 V.
- The pin and timer map for 5 step outputs (4 axes + 1 spare), 4–5 quadrature inputs, DAC, UART and
  SPI is checked against the G474 before the carrier is drawn (SR15). If it does not fit, a second
  MCU for I/O is the fallback.
- Link: isolated serial (RKD6 as now). The VFD and servos are noisy, so the link and I/O are
  galvanically isolated.
- This release **freezes the RKD6 wire format** (the protocol doc's own rule). Additions go in
  before SR16 closes.
- **Rejected for v1:**
  - EtherCAT CiA402 CSP drives with host cyclic setpoints (`LANE_A` LA-D1). That needs a real-time
    Ethernet master on PREEMPT_RT, a different device class and new homing, and would replace the
    tested device-evaluated trajectory path. It is planned as Phase F.
  - Analog ±10 V velocity drives: the position loop would sit on our board, which is more risk than
    pulse-train drives.
  - Software step generation: 40 kHz is far too slow.

**SR-D8. Safety is hard-wired and independent of software.**
- Dual-channel e-stop buttons (cabinet, gantry pendant, rack end) go to a safety relay that
  removes **STO** from all servo drives and the VFD and drops the main contactor.
- It is a stop category 1 (controlled stop, then STO) with a delayed release sized for the spindle
  braking time. Category 0 applies on relay fault. The Z brake engages with STO. The target
  architecture is ISO 13849 PL d (assumed), checked in SR17.
- **Tool release is interlocked in hardware:** the release valve's power passes through the VFD's
  zero-speed contact and the safety relay, so no software fault can release a tool at speed.
- Software is the second layer:
  - an MCU watchdog that drops enables and outputs;
  - the RKD6 link-loss lease (exists);
  - limit switches, referenced gating and soft limits;
  - the device-side following-error stop;
  - the dual-drive skew stop (exists).
- The board reads the safety relay's state. It never closes the safety chain.
- Guarding v1: e-stops, gantry trip bars at both ends, and a marked operator zone. Light curtains
  or pressure mats are a Phase F option with interfaces reserved in the cabinet.
- Dust is a fire and health hazard. The extractor is interlocked to run while the spindle runs,
  and the cabinet is sealed from dust.
- Wiring follows EN 60204-1 practice. The machine is for our own use; CE marking is out of scope.

**SR-D9. Sheet-scale stock: StockKit stores uniform tiles once.**
- A full sheet at 0.5 mm Z-only costs about 1 GB per stock, and the app also keeps a target.
- StockKit gains **uniform tiles**: a 16 × 16 ray tile whose rays all carry the same intervals (an
  untouched slab, or fully removed) is stored as one record, and becomes dense when first cut. Kerf
  lines of a nested sheet touch a small fraction of the tiles, so memory follows the cut length,
  not the sheet area.
- The target is held per part (a nested part's own region), not as a second full sheet.
- Contours are rebuilt per touched tile on the existing preview worker.
- The machining program moves out of the scene JSON into its own artifact file, removing the 8 MB
  ceiling. A job carries several work offsets.
- **Rejected:**
  - a coarse 2 mm lattice (tab and onion-skin thickness need ≤ 0.5 mm);
  - a separate 2.5D height field (a second stock type for one machine; uniform tiles help every
    large stock).

**SR-D10. Sheet-goods CAM is native CamKit. External G-code is accepted through the LinuxCNC
dialect.**
- Inputs:
  - DXF import in CadKit (lines, arcs, polylines with bulges, circles, layers mapped to
    operations);
  - CadKit parts (horizontal faces, sketches);
  - `SheetCutPlan` placements.
- **Nesting** (SR12): true-shape nesting of part outlines with holes, rotation steps, grain
  direction per part, part spacing from the tool diameter, and an edge margin.
  - Method: no-fit polygons with a placement heuristic, plus a search over the order. Parts placed
    inside other parts' holes come later.
  - Output: a `SheetCutPlan` with free-form placements, plus the offcut.
  - References for ideas, licences to check before reading code: SVGnest (MIT), Deepnest,
    libnest2d (LGPL).
- Sheet operations (SR11):
  - profiles with tabs (count, length, thickness, auto-placed on long straight edges) or an onion
    skin with a final skin pass;
  - lead-in and lead-out (arc or line) and ramped entry on profiles; climb or conventional per
    operation;
  - inside features before outlines, small parts before large ones, and a freed-part order that
    cuts small parts while their neighbours still hold them;
  - pockets with per-operation spindle speed; peck drilling; spoilboard surfacing.
  - V-carve and chamfer come later.
- Tooling: router bits with flutes, helix sense (up, down, compression), cutting length, holder and
  collet.
- Feeds: chip load per tooth by material and diameter (assumed tables, marked), so feed = rpm ×
  flutes × chip load. A cutting-force model gives spindle power and hold-down force. MDF,
  plywood, solid wood, acrylic, HDPE and PVC foam are added to `MaterialLibrary` with cutting data
  marked assumed.
- External G-code (VCarve or Fusion LinuxCNC posts) runs through `cnckit`. `M6` maps to our ATC
  sequence, and G-code that our machine cannot honour is rejected by name.
- `MachineBinding` stays three-axis; the rack zone and setter are machine state, not program axes.

**SR-D11. The physical build is a deliverable of the repo.**
- `SheetRouter` produces a fabrication pack:
  - a BOM with Buy, Make-by-service and Make-ourselves categories;
  - frame cut lists and weld seams (`WeldedFrame`);
  - plate DXFs with hole tables (the gantry side plates, Z and bearing plates are planar parts, so
    they go through `Drawing2D`);
  - STEP of every made part;
  - a frame jig layout; rail-pad replication and alignment procedures; an assembly order;
  - an electrical schematic and wire list (SR17);
  - a commissioning checklist (SR21–SR24).
- Every catalogue row we buy is checked against the vendor datasheet before ordering. Its
  `ValueBasis` moves from Assumed to Catalog, and the check stays in the repo.
- Dimensioned 3D-to-2D assembly drawings are not needed: STEP + plate DXFs + procedures cover it.

**SR-D12. Calibration is modelled and measured.**
- Each rack axis gets a pitch error table, measured with a calibrated steel rule or tape and a
  microscope or camera, and applied in the runtime like screw mapping.
- Gantry squareness is set by G12's per-side home offset, measured with the diagonal method and a
  test cut.
- The tool setter's height comes from the surfaced spoilboard.
- Acceptance (assumed): ±0.1 mm over the work area, ±0.02 mm repeatability, a 1500 × 1000
  rectangle square to 0.1 mm on the diagonal difference.
- Simulated and measured following error, cycle time and hold-down are compared and reported.

**SR-D13. Generic names in code.** No trade names in identifiers or catalogue rows; vendors appear
only as sources in docs. Standards (ISO30 / ISO 7388, ER collets, EN 60204-1, ISO 13849) are fine.

## Test jobs

- **Nested cabinet sheet:** 18 mm birch plywood 2500 × 1250. Carcass sides, shelves, a back panel
  and small drawer parts with 5 mm shelf-pin holes, dados and a pocket. Three tools: an 8 mm
  compression bit, a 5 mm drill and a 6 mm down-cut. Nested, onion skin plus skin pass on small
  parts. This is the acceptance job for SR14 and SR24.
- **Sign:** pockets and inside profiles in 30 mm solid oak with a 6 mm up-cut and a 60° V-bit
  (once V-carve exists).
- **Acrylic panel:** 6 mm cast acrylic with a single-flute O-flute, holes and outline, no melting
  (chip-load check).
- **Spoilboard surfacing:** the full table with a 50 mm surfacing cutter.
- **Calibration plate:** a 1500 × 1000 rectangle with corner holes in MDF, measured for
  squareness and scale.

## Reference numbers (assumed until checked against a datasheet or a measurement)

| Quantity | Value | Source |
|---|---|---|
| Work area Y × X | 2600 × 1600 mm (Y 3150 option) | decided (SR-D1) |
| Travel Y / X / Z | ≈ 2800 (incl. rack zone) / 1600 / 250 mm | assumed |
| Gantry clearance to spoilboard | 150 mm | assumed |
| Rapid XY / Z | 40 / 15 m/min | assumed |
| Cutting feed | up to 20 m/min | assumed |
| Acceleration XY / Z | 3 / 3 m/s² | assumed |
| Gantry moving mass | ≈ 150 kg (beam ≈ 57 kg at 28.4 kg/m) | estimate |
| X carriage + Z + spindle | ≈ 60 kg (spindle ≈ 20 kg) | assumed |
| Rails | HGR25 on Y (2 blocks per side), HGR20 on X and Z | assumed |
| Rack and pinion | helical mn 1.5, β 19°31′42″, 20 T → 100 mm/rev | derived |
| Reducers | 5:1, ≤ 5 arcmin (planetary) | assumed |
| Servos | Y 2 × 750 W, X 400 W, Z 400 W + brake | assumed |
| Pulse resolution | 5000 pulses/motor rev → 4 µm; 167 kHz at 40 m/min | derived |
| Z screw | Ø16 or Ø20 × 5 mm ball screw | assumed |
| Beam | RHS 200 × 100 × 6.3 steel, ≈ 11 µm at 300 N mid-span | estimate |
| Static tool-tip stiffness | ≥ 2 N/µm X and Y at full Z | target |
| First gantry mode | ≥ 40 Hz | target |
| Spindle | ISO30 ATC, ≈ 6 kW S1, 18–24k rpm, 400 V 3~, air-cooled | assumed |
| VFD | 7.5 kW 400 V with braking resistor and STO | assumed |
| Tool rack | 8 forks, pneumatic cover, rear rack zone | assumed |
| Air | 6 bar, dry, oil-free (tool release, purge, shoe, cover) | assumed |
| Vacuum | 6 zones, side-channel blower ≈ 5.5 kW, ≈ 300 m³/h | assumed |
| Dust extraction | ≥ 20 m/s in a Ø100 hose (≈ 570 m³/h at the shoe), extractor ≈ 2.2–3 kW | estimate |
| Connected load | ≈ 17 kW (spindle 6, vacuum 5.5, extraction 3, servos 2.3) → ≈ 25 A at 400 V 3~ | estimate |
| Board I/O | 5 step/dir, 4–5 quadrature, ≈ 32 inputs, ≈ 20 outputs, 1 DAC | estimate |
| Accuracy / repeatability | ±0.1 / ±0.02 mm | target |

The connected load assumes a 400 V three-phase supply of at least 25–32 A. If the workshop has
single-phase only, SR0 changes the spindle (≤ 3.7 kW, 230 V three-phase from a single-phase VFD)
and the vacuum pump, and records the cost.

## Steps

### Phase A: parts, machine and sizing (simulation only, can start now)

**SR0. Baseline and requirements.**
- Run the gate on the starting commit and record failures that come from main.
- Record the workshop facts: supply (phases, amps), floor space and access for a 3 m frame, air
  supply, extraction route.
- Survey reference machines (industrial nesting routers and self-build ATC routers) for
  specifications and price, as the lathe plan does.
- Tests: the gate's numbers recorded in Progress.

**SR1. Catalogue parts.**
- `motion`: HGR20/25 rails and blocks; planetary reducers with ratio, backlash, torsional
  stiffness, axial and radial load ratings; servo rows with brake variants; cable chains with
  bend radius, fill and a moving-end model.
- `transmission`: a helical `Rack` and pinion (normal module, helix angle, hand, axial thrust,
  contact ratio, assumed mesh stiffness); a `PreloadedPinionMount` (hinge, spring force,
  stroke).
- `routing` (new package):
  - `AtcSpindle` (ISO30 taper, release cylinder, sensors, purge, power, speed range);
  - `ToolHolder` (ISO30 + ER32) and `Collet`;
  - `ToolFork`, `ToolRack` (pockets, cover cylinder), `ToolLengthSetter`;
  - `DustShoe` (lift cylinder, brush, hose);
  - `VacuumTable` (plenum grid, zones, valves), `Spoilboard`, `VacuumBlower`.
- `power` and `control`:
  - `Vfd`, `SafetyRelay`, `EmergencyStopButton`, `Contactor`, `CircuitBreaker`, `BrakeResistor`;
  - `ControlCabinet` with DIN rows and an I/O count derived from the parts;
  - `LevellingFoot` with a load rating.
- All rows are marked Assumed until SR19 checks them against a datasheet.
- Tests:
  - geometry valid; BOM rows;
  - helical pitch per pinion turn equals π·mn·z / cos β;
  - axial thrust equals tangential force × tan β;
  - setter and fork positions as frames;
  - cable chain length follows the moving end.

**SR2. Servo and steel axes in the shared builders.**
- `AxisBuilder.driveRack` and `driveScrew` take any `AxisMotor` (stepper or servo) with an
  optional reducer stage and preloaded mount, replacing the `NemaStepper` cast.
- `GantrySpec` gains `motor` (stepper or servo family), `reducer`, `structure` (aluminium T-slot
  or steel), a helical `Rack` drive and cable chains.
- Existing examples (cnc-router, gantry-picker, gantry-welder, overhead-arm) keep their geometry
  and pass unchanged.
- Tests: existing gantry and router checks pass; a servo rack axis gives motor-derived limits
  through the reducer; skew grouping still groups the two Y motors by leader joint.

**SR3. The `SheetRouter` assembly.**
- `SheetRouter(spec:SheetRouterSpec) extends Gantry` or a sibling on the same builders:
  - welded base (`WeldedFrame`) with rail pads, table supports and feet;
  - steel or aluminium gantry beam with side plates; X carriage; Z with brake and ball screw;
  - ATC spindle; rack zone with forks, cover and setter;
  - dust shoe on its lift; vacuum table and zones;
  - cable chains on Y and X;
  - home and limit switches on all axes, homing order Z, X, then Y squared;
  - a cabinet from SR1.
- `SheetRouterChecks` in MachineKit smoke:
  - FK at the travel corners, and every rack pocket reachable along its fork opening;
  - the longest tool clears the gantry over a full-thickness sheet;
  - the shoe clears the rack when raised; no interference anywhere in travel;
  - cable chains within their bend radius and travel;
  - BOM and mass properties at Y = 2600 and Y = 3150.
- `materia.project.json`, a Start-page entry (update the machining-family count in
  `StartPageTests`), `ProjectSourceTests.checkSheetRouter` (loads, MuJoCo holds pose, homes).
- Tests: smoke checks; app check.

**SR4. Drive sizing and compliance (SR-D4).**
- Inputs: the moving masses from `massProperties`, cutting forces from SR13 (a stated 300 N
  allowance until then), accelerations.
- Per axis:
  - motor torque and speed against rated and peak;
  - inertia ratio (load to motor, reflected through reducer and pinion);
  - pinion bending and contact stress;
  - reducer output torque and axial load;
  - preload spring force against the separating force.
- Compliance: rack mesh, reducer torsion, preload hinge, belt (if chosen) and Z screw, plus
  backlash. This decides planetary or belt reduction and the Z screw size.
- `PlanCheck` learns the rack drive (it knows screws and steppers today).
- Tests:
  - reflected inertia hand-checked for one axis;
  - the chosen sizing passes at 3 m/s² with the stated margin;
  - a 400 W Y drive is rejected by name at 3 m/s²;
  - preload below the separating force at peak acceleration is rejected.

**SR5. Structural study (SR-D3).**
- Generalise the bench mill's FEM exporter from hard-coded casting ids to an assembly packet: the
  selected bodies, interfaces with stated stiffness (rail blocks, bolted joints, pinion mesh) and
  load cases. The mill keeps working through the same exporter.
- Run Gmsh + CalculiX on the gantry (beam, side plates, X carriage, Z) and the frame. Load cases:
  cutting load at the tool tip in X, Y and Z at the corners and mid-span, and gantry acceleration.
- Outputs: tool-tip compliance and its budget per element, the first modes, support reactions and
  frame twist under its own weight on the feet.
- Decide the beam material and section, the side-plate thickness and whether the frame tubes are
  filled.
- Tests: the exporter reproduces the mill's reference packet; the router packet's analytic
  checks (beam bending against the formula, within a stated tolerance); targets met or the step
  names what limits them.

**SR6. Frame, fabrication drawings and alignment procedures (SR-D2, SR-D11).**
- Weldment: tube layout, weld seams and sequence, jig layout (frame points as jig stops), cut
  list.
- Plate DXFs and hole tables for the side plates, Z plate, bearing plates and motor plates,
  through `Drawing2D` from planar parts.
- Procedures in `machinekit/examples/sheet-router/build/`: rail-pad replication, first-rail
  alignment to a reference line, second-rail parallelism, rack setting (pitch across rack joints
  with a joining gauge), gantry squaring, Z perpendicularity (tramming).
- Tests:
  - cut list matches member lengths;
  - every made part has STEP and, if planar, a DXF;
  - plate holes match their mating parts' bolt patterns (an assembly check);
  - rack joints fall at whole teeth.

### Phase B: machine behaviour in simulation

**SR7. The ATC sequence and tool table (SR-D5).**
- A `ToolChangeSequence` in MotionKit answers `cnc.tool_change.N`:
  - pocket approach frames come from the assembly;
  - each step waits on its sensor with a timeout and names its fault;
  - pocket occupancy and the tool table are machine state, persisted;
  - length is measured on the setter.
- The simulated operator handshake stays for machines without an ATC.
- Decide whether the robot `ToolChangeTransaction` and lock observations can be shared (the same
  lock and confirm pattern), and record the answer.
- The simulated spindle has release, clamp and tool-present behaviour; a tool is a body that
  transfers between pocket and spindle.
- Tests:
  - a full change T1 → T3 → T1 in the app: right pockets, sensors in order, length applied;
  - a missing tool, an occupied return pocket, a clamp timeout and an at-speed release request each
    stop with their named fault and leave the machine in a stated safe state;
  - change time from the axis limits within 10 % of the run.

**SR8. Vacuum hold-down and freed parts (SR-D6).**
- Vacuum table with zones, a pressure model with bleed through open kerf length, and zone
  selection from the sheet size.
- StockKit gains connected-component detection on the sheet stock (pieces no longer joined to
  the sheet).
- Each freed piece's lateral hold is compared with the cutting force of its last pass.
- Tests:
  - a 50 × 50 mm part cut through in one pass at full feed is reported as freed and slipping, with
    its program line;
  - with an onion skin it is not;
  - closing a zone over an empty area raises the vacuum level for the rest.

**SR9. Sheet-scale stock and program artifacts (SR-D9).**
- StockKit uniform tiles (stored once, dense on first cut) in the core, with snapshots and
  bit-identical results across thread counts.
- Per-part targets; per-tile contours on the preview worker.
- `SceneArtifactMachining`: program in its own file; several work offsets; a configurable stock
  spacing.
- Tests:
  - the full test sheet at 0.5 mm uses less than 300 MB (assumed budget) after the cabinet job;
  - uniform tiles give results identical to dense tiles on an existing StockKit fixture;
  - a 20 MB program loads.

### Phase C: CAM

**SR10. DXF import.**
- CadKit DXF reader: LINE, ARC, CIRCLE, LWPOLYLINE with bulges, POLYLINE, SPLINE sampled to a
  stated tolerance, INSERT expanded, layers and units.
- Closed-contour assembly with a gap tolerance; inside and outside nesting of contours into parts
  with holes.
- Tests:
  - round trip with our writer;
  - a bulge polyline gives exact arcs;
  - an open contour within the gap tolerance closes, and a larger gap is reported with its
    location;
  - layer-to-operation mapping.

**SR11. Sheet operations (SR-D10).**
- In `camkit`:
  - profile tabs (auto and manual), onion skin and skin pass;
  - leads and profile ramps; climb or conventional;
  - per-operation spindle speed; peck drilling; surfacing;
  - inside-before-outside and small-before-large ordering, and freed-part-aware ordering using
    SR8.
- Tests:
  - tabs at the stated count, length and thickness, avoiding corners;
  - the skin pass removes the skin and nothing else;
  - lead arcs tangent; cut direction correct for climb inside and outside;
  - small parts ordered so that SR8 reports no slipping part on the cabinet sheet;
  - removed volume matches stock minus targets.

**SR12. Nesting.**
- True-shape nesting with no-fit polygons over part outlines with holes:
  - rotation steps (0/90/180/270 by default, free angle optional) and grain locks;
  - spacing from tool diameter plus a gap; an edge margin;
  - several sheets; a remnant output.
- Output: `SheetCutPlan` with free-form placements.
- Tests:
  - rectangles nest at least as densely as the guillotine baseline;
  - no overlaps or margin violations (exact check);
  - grain-locked parts keep orientation;
  - the result is deterministic for a seed;
  - utilisation and time on the cabinet set recorded.

**SR13. Router tooling, feeds and forces.**
- `toolpathkit/tool`: router bits with flutes, helix sense, cutting length, collet and holder.
- Chip-load tables by material and diameter (assumed, marked); feeds from rpm × flutes × chip load,
  clamped by the machine.
- Wood and plastics cutting force from specific cutting energy (assumed per material); spindle
  power and torque check in `PlanCheck`; the force feeds SR4 and SR8.
- Materials added to `MaterialLibrary` with their cutting data.
- Tests:
  - feeds from chip load for a stated case;
  - an acrylic cut below the minimum chip load is reported (melting risk);
  - spindle power exceeded at a stated depth and feed is rejected, naming the operation.

**SR14. End-to-end simulated acceptance.**
- The nested cabinet sheet in the app:
  - DXF → nesting → CamKit → ATC changes → vacuum zones → real-time cutting on the sheet stock;
  - freed-part check, inspection of each part against its target.
- The same job as LinuxCNC-dialect G-code through `cnckit` gives the same parts.
- Tests: all parts within tolerance; no slipping parts; no collisions; cycle time recorded; an
  allocation and frame-time budget at sheet scale.

### Phase D: control hardware (bench)

**SR15. Board firmware: timers, encoders, I/O, watchdog.**
- On the G474 (`boards/nucleo-g474re` grows into the router board):
  - hardware-timer step and direction for 5 axes, with direction setup and hold times and a stated
    maximum rate;
  - quadrature counting on hardware timers;
  - fast inputs with edge capture; SPI I/O expanders for slow I/O; DAC for the spindle;
  - an independent watchdog;
  - device-side following error and drive-alarm stops;
  - USART with DMA.
- First, the pin and timer map is checked against the G474 datasheet. If it does not fit, the
  fallback is recorded.
- Carrier board: schematic and layout (KiCad, sources under
  `machinekit/examples/sheet-router/electronics/`) with differential line drivers, isolated
  24 V inputs, 24 V outputs and an isolated link.
- Tests:
  - virtual-device parity tests run against the board on the bench (same segments, same step
    counts);
  - pulse rate and jitter measured with a logic analyser at the maximum rate;
  - quadrature counts a known pulse train exactly;
  - watchdog drops the outputs.

**SR16. RKD6 first hardware release.**
- Wire additions: encoder counts in `State6`, drive enable, alarm and ready, a following-error
  bound per axis, analog spindle output, safety-relay state input. Then the version freezes, as
  `DEVICE_PROTOCOL.md` requires.
- Deployment for the router board (`robotkit/deployment/sheet-router/`) with the real board id.
- Tests: protocol tests; deployment fingerprint; host and virtual device updated; RobotKit native
  and Haxe suites pass.

**SR17. Electrical design and safety chain (SR-D8).**
- Schematic and wire list:
  - supply, main switch, breakers, contactor;
  - safety relay with category 1 delay, STO to all drives and the VFD;
  - e-stops, gantry trip bars, the release-valve interlock through zero-speed;
  - 24 V supplies, drive wiring, shielding and grounding, cabinet layout.
- The cabinet's I/O count comes from the SR3 assembly; every channel in the model has a terminal
  in the wire list.
- Safety function list with the PL target and a check of the architecture (assumed component
  values until datasheets).
- Tests:
  - every model channel maps to one terminal and one board pin, and the reverse;
  - every e-stop path reaches STO on all drives (a graph check on the schematic data);
  - the release valve has no path to power that bypasses zero-speed.

**SR18. Hardware-in-the-loop bench.**
- One servo and drive on the bench with our board: direction, electronic gear, following error,
  alarm handling, homing on a real switch.
- Then two servos on a short rack section (or two motors with flywheels) for dual drive: skew
  guard, squaring, a forced skew fault.
- The ATC sensors and valve on a test rig with the spindle's release cylinder (or the spindle
  itself before mounting): sequence, timeouts, the zero-speed interlock.
- Tests:
  - measured following error at 40 m/min equivalent within the stated bound;
  - home repeatability over 20 cycles;
  - a forced skew stops both drives;
  - the release interlock holds with software forcing the output on.

### Phase E: build and commissioning

**SR19. Procurement.**
- Every bought part is checked against its datasheet (rails, racks, reducers, servos and drives,
  spindle, VFD, forks, setter, blower, extractor, safety parts). `ValueBasis` moves to Catalog;
  the checks re-run with the real numbers.
- The BOM is costed, and SR4, SR5 and SR17 are re-run on the bought parts.
- Tests: no Assumed rows left among bought parts; all checks pass on Catalog values.

**SR20. Frame, rails and racks.**
- Weld the frame, settle it, set the feet, replicate the rail pads, align the rails, set the
  racks, following the SR6 procedures. Record the measurements (rail straightness, parallelism,
  twist) in `build/measurements/`.
- Tests: measured values within the procedure's tolerances, or the deviation recorded with a
  decision.

**SR21. Axes commissioning.**
- Mount the gantry, X and Z. Wire the cabinet. Check the safety chain function by function
  first. Then: directions, electronic gear, servo gain tuning, homing, squaring, soft limits,
  following-error bounds.
- Tests: safety function checks recorded; homing repeatability; following error at the rated
  speeds against the simulation's prediction.

**SR22. Spindle and ATC commissioning.**
- VFD parameters, warm-up, at-speed and zero-speed signals, braking time against the safety
  relay's delay. Rack teach-in (pocket frames measured, compared with the model), setter height,
  tool changes over all pockets.
- Tests: 100 consecutive tool changes without fault; braking time within the delay; tool lengths
  repeat within 0.01 mm.

**SR23. Vacuum, spoilboard and dust.**
- Surface the spoilboard; measure vacuum per zone with and without a sheet; set the fault
  thresholds; measure extraction air speed at the shoe.
- Tests: vacuum levels against the SR8 model (the model is updated from measurement); the shoe's
  air speed meets SR-D8.

**SR24. Calibration and acceptance (SR-D12).**
- Rack pitch error tables; squareness from the calibration plate; scale.
- Run the nested cabinet sheet for real. Measure the parts, cycle time and following error.
- Report against the SR14 simulation, with every remaining assumed value listed.
- Tests: acceptance targets met or the limiting element named; real cycle time within a stated
  margin of the simulated one; every freed small part held as predicted.

### Phase F: later

- EtherCAT CiA402 drives with host cyclic setpoints, if the pulse-train path limits us.
- Light curtains or pressure mats; automatic sheet loading and unloading; a part labelling
  printer.
- A vertical boring block for cabinet work (drilling banks).
- A touch probe in an ISO30 holder; camera sheet alignment.
- V-carve, chamfer, 3D relief; part-in-part nesting; remnant library.
- A Y = 3150 build.

## Order

```
SR0 ── SR1 ── SR2 ── SR3 ──┬── SR4 ──┬── SR5 ── SR6 ─────────────────────┐
                           │         └──────────────────────────────┐    │
                           ├── SR7 ── SR8 ──┐                       │    │
SR9 (StockKit) ────────────┼────────────────┤                       │    │
SR10 ── SR11 ── SR12 ── SR13 ───────────────┴── SR14                │    │
SR15 ── SR16 ── SR17 ── SR18 ───────────────────────────────────────┴── SR19 ── SR20 ── SR21 ── SR22 ── SR23 ── SR24
```

- Phase C (SR10–SR13) and SR9 need no machine and run in parallel with Phase A.
- Phase D (SR15–SR18) can start after SR3, because the I/O list comes from the assembly. Bench
  parts for SR15 (Nucleo, one servo and drive, logic analyser) are the first purchase.
- SR4 and SR5 decide the beam, reducers and Z screw. Nothing structural is ordered before they
  are done.
- SR16 freezes RKD6. Coordinate with any other session changing the protocol before closing it.
- Long-lead items (spindle, servos, racks, rails) are ordered after SR19 on checked datasheets.

## Working rules

- Work in `/home/joao/dev/materia` on local `main`, per user instruction on
  2026-10-19 to remove the separate worktree and reuse existing build dependencies.
  Dependencies count only once they are on local `main`.
- A blocked step is marked `blocked: <what>` in Progress, and work goes on to the next unblocked
  step.
- Commit per step, e.g. `Sheet router: helical rack drive (SR1)`, with Progress updated in the
  same commit.
- Diagnose before working around. Numbers not from a datasheet or a measurement are marked
  assumed.
- No legacy compatibility: one schema version, reject others.
- Heavy suites go through the heavy-run wrapper.
- Shop-floor steps (SR18, SR20–SR24) record measurements in `build/measurements/` and photos by
  reference. A step is done when its tests are met on the real machine, not in simulation.
- Safety work (SR17, SR21) is never shortcut by software: a missing hardware interlock blocks the
  step.

## Progress

| Step | State | Commits |
|---|---|---|
| SR0 | started: reference survey recorded; gate blocked by disk headroom and toolchain setup; workshop facts pending | see SR0 notes |
| SR1 | planned | |
| SR2 | planned | |
| SR3 | planned | |
| SR4 | planned | |
| SR5 | planned | |
| SR6 | planned | |
| SR7 | planned | |
| SR8 | planned | |
| SR9 | planned | |
| SR10 | planned | |
| SR11 | planned | |
| SR12 | planned | |
| SR13 | planned | |
| SR14 | planned | |
| SR15 | planned | |
| SR16 | planned | |
| SR17 | planned | |
| SR18 | planned | |
| SR19 | planned | |
| SR20 | planned | |
| SR21 | planned | |
| SR22 | planned | |
| SR23 | planned | |
| SR24 | planned | |
| Phase F | later | |


### SR0 — starting evidence (2026-10-19)

Worktree: `/home/joao/dev/materia-worktrees/sheet-router`, branch `sheet-router`,
starting local main `4df48aacb`. The survey above predates this starting commit;
its dependency claims still need the baseline gate. The supplied plan was copied
from the untracked main-checkout file without changing that checkout.

Baseline setup:
- Pinned Haxeon: `9e37bf4ac00d8f5f5d50c542d35e0577a0a99673`.
  GitHub rejected fetching this commit (`not our ref`). Fetching that exact commit
  from `/home/joao/dev/materia/haxeon` succeeded; no version substitution.
- Toolchain preflight fails: `missing pinned Haxe compiler; run
  scripts/bootstrap-tools.sh first`. The new worktree has no `.tools` installation.
- MachineKit gate submitted through `/home/joao/dev/materia-cache/tools/heavy-run`
  with a 12 GiB cap and 4 GiB memory requirement. Both slots were busy; cancelled
  the queued attempt after finding the missing compiler. No tests executed and no
  baseline numbers or main regressions are claimed. Local logs:
  `build/sr0-machinekit.log`, `build/sr0-toolchain.log`.
- Next: bootstrap the pinned tools, initialise required native submodules, then run
  MachineKit and the router app regression sequentially through heavy-run. Existing
  native outputs are potential build inputs, not evidence for this commit.

Workshop facts pending user response: electrical supply phases/current, floor
space, frame delivery/access route, compressed-air capacity/quality and extraction
route. The plan's electrical and pneumatic assumptions remain assumptions.

Reference survey, checked on manufacturer pages 2026-10-19:

| Reference | Published specification | Published price and scope |
|---|---|---|
| [Format4 profit H08](https://www.felder-group.com/de-at/produkte/cnc-bearbeitungszentren-c1953/nesting-cnc-bearbeitungszentrum-profit-h08-p144268) | 13.25 work area 2500 × 1250 × 85 mm; dual-sided gantry drive, XY racks, 6 vacuum zones; 10/12 kW spindle options up to 24,000 rpm with HSK F63 automatic clamping; 8/11/15 tool positions | No public price on consulted page; quotation pending. |
| [Avid PRO4896](https://www.avidcnc.com/pro4896-4-x-8-cnc-router-machine-p-1337.html) | 2508 × 1260 × 203 mm work area; 2934 × 1700 mm footprint; rack XY, screw Z; modular assembly, servo and ATC options | Page displays USD 11,389 with configurable options. ATC/servo configuration, VAT, import, freight and installed cost are unverified; not an ATC package quote. |

The industrial reference supports the dual-drive/rack and zoned-table direction.
The modular reference provides an access/assembly comparison. Neither has the
1600 mm usable width required here. Neither validates the proposed servo sizing,
ISO30 spindle choice or complete build cost. A documented self-build ATC reference
and comparable configured prices remain open survey items.


### SR0 — disk preflight (2026-10-19)

Continuation was conditional on available disk space. `df -h` reports the worktree
and shared cache on `/dev/sda7`: 485 GiB total, 11 GiB available, 98% used.
The other mounted development volumes have only 2.9 GiB (`DATA1`) and 3.1 GiB
(`DEV`) available. Inodes are not limiting (20% used on root).

The existing main Haxeon `.tools` and `out` directories occupy 1.5 GiB and
4.2 GiB respectively; app native outputs occupy another 1.3 GiB. These are
observations, not a complete fresh-build size estimate. With native submodules
and baseline test outputs still needed, a full bootstrap/gate was deferred to
avoid exhausting the shared filesystem. No build was launched or files deleted.
Resume after freeing headroom or providing a build volume; then recheck disk and
set up the exact pinned toolchain before running the gate. A practical initial
headroom target is 25–30 GiB, subject to checking actual build growth.


### Workspace change (2026-10-19)

At the user's request, router work continues in `/home/joao/dev/materia` on
`main`, reusing its existing toolchain and builds to reduce disk usage. The
separate `sheet-router` worktree is removed; its branch retains commits
`e8ec34a25` and `837fea7b1`. SR0 logs are preserved in
`build/sheet-router-sr0/` in the main checkout. Earlier setup and disk notes
above describe the removed worktree; recheck the current main toolchain and
available disk before the next gate. Unrelated main-checkout edits are preserved.
