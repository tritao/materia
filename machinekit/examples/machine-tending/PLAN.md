# Machine tending plan

**Goal.** A six-axis arm with a pneumatic parallel gripper tends an enclosed benchtop vertical mill,
without anyone touching it. Each cycle runs like this:
1. The arm takes a blank from an infeed tray.
2. It asks the mill to open its door, puts the blank in a pneumatic vise and asks for a clamp.
3. It leaves the machine and asks for the door to close and the cycle to start.
4. The mill cuts the part in real time, as the router does.
5. The arm takes the finished part out to an outfeed tray.
6. The cycle repeats until the infeed tray is empty.

Everything physical is simulated the way the hardware works:
- the fingers grip and the vise clamps by force and friction;
- the door and the vise are moved by pneumatic cylinders driven by valves;
- the two machines are separate controllers that talk only through wired signals.

The same signal names would then drive a real cell.

The work is split into two examples:
- `machinekit/examples/bench-mill/`: the mill on its own, a CNC project like the router;
- `machinekit/examples/machine-tending/`: the cell. It includes the mill, the arm with its gripper, the trays, the controllers and the wiring, the same way `WeldingCell` includes `RobotArm`.

## What exists

Surveyed 2026-10-03 on local main 1048bf768, plus the `mobile-welder` and `x7-transmissions` branches.

**CNC**
- `machinekit/examples/cnc-router/`: an assembly with three prismatic axes, lead screws, steppers, overtravel and `CncRouterChecks`.
- The scene artifact's `machining` section (v12+): program, axes, spindle gauge line, `workOffset`, tools, stock, target and controller.
- `app/CncProgramPlayer` takes G-code through `CncCompiler`, `ToolpathMotion.lower`, `ManipulatorMotion` and `ProgramPlanner`, and plays it on the assembly robot.
- `app/MachiningStock` cuts StockKit tri-dexel stock from the tool tip's live pose. `StockPreviewWorker` contours it off the frame thread.
- `CncPanel` provides hold, resume, restart-at-line and speed override.
- Handshakes are `MotionOp.WaitInput` barriers answered by the player's input callback. The `cnc.tool_change.N` and `spindle.at_speed` handshakes are answered at once.

**Arm and tools**
- `machinekit/examples/robot-arm/RobotArm`: six revolute joints, ServoMotor and gearbox drives (X6), and a reach of about 0.73 m from the shoulder to the flange.
- Its wrist is inline (roll–pitch–roll). It is not the cobot layout of UR-type arms (shoulder, elbow and wrist 1 parallel; offset wrist).
- On the welder branch, `ArmTool` (`build`/`expose`/`ready`) and `RobotArm(withCell, ?armTool)`.
- MotionKit has analytic IK only for spherical wrists (`OpwKinematics`). Everything else uses KinematicsKit's numeric IK.

**Missions**
- `app/MissionPlayer` runs scene `mission` steps `goTo`, `pick`, `place` and `weld`.
- Pick and place go through `HandlingPlanRunner`. It moves straight down, with no orientation or yaw.
- Grasping is the suction kind only: `SimulatedSuctionTool` uses `holdObjectOnLink`, a kinematic hold.

**ProcessKit and I/O**
- `WelderProcessDevice` (outputs, feedback, channels) is the pattern for "one interface, sim or hardware".
- Outputs are process channels (`ProcessChannelDeclaration`, `MotionOp.SetOutput`).
- Inputs are sensor frames (`publishSensorFrame`).

**Collision**
- `robotkit.manipulation.ArmClearance` (welder branch d7d3d0f14): distances from the arm and tool hulls to the cell's hulls, and straight joint-space sweeps.
- `kinematicskit` `CollisionWorld` on coal (branch `collision`, CL1).
- There is no collision-aware planner yet (CL6).

**MachineKit parts**
- Rails: `LinearRailSystem` (MGN catalogue).
- Screws and motors: `LeadScrew` (trapezoidal or Acme), `ScrewSupport`, `ServoMotor` (50/100/200 W, assumed values), `Gearbox`, `ShaftCoupling`, `ShaftEncoder`, `LinearScale`.
- Grippers: `ParallelGripper`, an envelope box with `open`/`close` pneumatic ports and `Grip(stroke, force, open, close)`.
- Ports: `PortKind.Signal` exists.

**Simulation**
- MuJoCo joint targets can be POSITION, VELOCITY, EFFORT or SERVO.
- Free objects are boxes (`dynamicParts`).
- Each part gets one convex hull of at most 64 vertices; there is no convex decomposition.

## What is missing

**Mill hardware**
- No ball screw, cast structure, enclosure, door, vise, spindle drive, pneumatic cylinder or solenoid valve.
- No binary sensors: door switch, cylinder reed switch, part present.

**Simulation and runtime**
- Nothing turns a valve output into a force on a joint, so the runtime has no joints that a process drives.
- `ParallelGripper` has no jaws, so a gripper cannot be simulated by contact.
- There is no `gripper` robot-tool kind.

**Controllers and signals**
- One assembly becomes one robot. The mill and the arm cannot be separate controllers, and the CNC player and the mission do not coordinate.
- Mission steps have no signals and no waits, and there is no signal bus.
- There is no machine-side interlock logic: cycle start, door, clamp, robot clear.

**Parts and stock**
- Stock has to be a part on a robot link, so a loose blank cannot be machined.
- Nothing excludes contact between the cutter and the blank.
- The finished part does not get its mass back from the stock.
- Pick and place have no orientation. Grasp frames, the vise datum and the work offset are not derived from geometry.

## Decisions

**MT-D1. Two controllers, two robots.**
- The mill controller owns X/Y/Z, the spindle, the door and the vise. The robot controller owns the arm joints and the gripper jaws.
- A joint belongs to the controller that its actuator's driver or valve is wired to. This is derived from the X8 motor/driver/controller split, never listed by hand.
- `AssemblySimulationBridge` builds one `RobotModel` per controller from one cell assembly. The parts that no controller owns are environment.
- **Rejected:** motion groups on one runtime. Real cells have two controllers with their own timing, devices and faults. A runtime with two independent trajectory queues would be a second mechanism with nothing in the hardware behind it.

**MT-D2. The mill controller's interlock logic owns the door, vise and cycle.**
- The robot only *requests* (door open/close, clamp/unclamp, cycle start) and reports *robot clear*.
- The mill answers with states: ready, door open/closed, clamped/open, part present, in cycle, cycle complete, alarm. The signal set is the usual robot-interface set of commercial VMCs.
- The G-code stays the part program. Its only cell-specific line is the final `G53` move to the load position, which is derived from the machine.
- LinuxCNC `M62`–`M66` are a later option for programs that want to drive I/O themselves.

**MT-D3. Signals are wires.**
- Controller cabinets carry `Signal` ports, and `connectPorts` wires an output of one controller to an input of the other.
- The scene's signal map is derived from that wiring, the same way the welder's grounding is traced from the work lead.
- In simulation a `CellSignals` bus carries each wire with a latency of one controller tick. On hardware the same names map to digital I/O.

**MT-D4. Pneumatic cylinders are actuators.**
- A cylinder part (bore, rod, stroke) plus `addCylinder(joint, cylinder, valve)` becomes an `AssemblyActuator` of kind Pneumatic.
- Force is supply pressure × piston area: the full bore extending, the bore minus the rod retracting.
- A flow restriction limits speed, as a damping term sized from the rated piston speed.
- The valve's channel picks the direction. The runtime drives the joint with EFFORT and leaves it out of motion planning ("process-driven joints").
- Solenoid valves are parts wired to controller outputs; their channel names are derived, not chosen.

**MT-D5. Grip and clamp by contact.**
- Jaw force plus friction holds the blank, in the fingers and in the vise. There is no `holdObjectOnLink` for the gripper or the vise.
- Slip is measured, not assumed. A kinematic fallback is only added if measurements show MuJoCo cannot hold a clamped box under the mill's accelerations, and the plan is updated first.

**MT-D6. Geometry decides.**
- Grasp frames come from the blank's opposite planar faces: the closing axis is normal to them, and the width is the distance between them. This mirrors `WeldSeams.find`.
- The vise datum is the fixed jaw face, the top of the parallels and the end stop. G54 is derived from that datum at the axes' zero instead of being typed in, unlike `CncRouter.workOffset()`.
- The load position is the axis values that bring the vise nearest the door. Approach via-poses come from the door opening's frame.

**MT-D7. Blanks are free bodies with identity.**
- Every blank is a `dynamicParts` occurrence and owns its own StockKit stock.
- `MachiningStock` follows the blank's live object pose, not a part on a robot link.
- Physics never pairs the cutter with blanks. The cutter still touches the vise and the table.
- After unloading, the part's mass is the stock's volume × density, and the contoured stock becomes its runtime geometry.

**MT-D8. One tool, no tool changes.**
- The first job uses one 6 mm end mill: pockets and circular pockets only.
- A tool-change request during an unattended run is an alarm. The current instant answer is an operator shortcut.
- An automatic tool changer is a later step.

**MT-D9. Collision.**
- Until CL6, approaches are derived via-poses.
- Every segment is swept with `ArmClearance` against the cell hulls, with the door at its open pose, when the mission is generated. A failure rejects the mission.
- "Robot clear" is computed from the arm and tool hulls against the machine's zone box, not taught.

**MT-D10. Concave things are several convex parts.**
- The enclosure is panels, and the front is split around the door opening, so each part's hull is honest.
- T-slots and pockets stay in the CAD solids. Collision sees their hulls, which is fine for a table or a vise body.

**MT-D11. Cobot arms in size classes.**
- A new `CobotArm` family has the UR-type layout:
  - base pan;
  - shoulder, elbow and wrist 1 about parallel axes;
  - wrist 2 at right angles, then wrist 3 roll;
  - lateral offsets between the joint modules.
- Size classes follow the published sizes of the typical cobot classes. The names are generic, so no trade names appear in code.
- Each class fixes:
  - its kinematic lengths;
  - which joint module size sits at each joint;
  - joint speed limits;
  - rated payload;
  - total mass.
- Joints are one family of modules in sizes 0–4: a housing, a servo and a strain-wave gearbox (ratio about 100). The sizes share a catalogue of diameter, length, rated and peak torque, speed and mass.
- Classes are built from that catalogue, not from per-arm numbers. Datasheet figures are reference values, marked assumed like the existing `ServoMotor` rows, and the X6 plan check confirms that each class's drives carry its rated payload.
- `RobotArm` stays as it is, so the arm, welder and router examples are not touched.

## The machine (starting numbers, fixed in MT1–MT4)

| Item | Value |
|---|---|
| Travel X / Y / Z | 250 / 150 / 250 mm |
| Table | 400 × 130 mm, three 10 mm T-slots |
| Structure | Cast base, column bolted to the base's back, saddle (Y) on the base, table (X) on the saddle, head (Z) on the column |
| Rails | HGR15 on all axes (new catalogue rows) |
| Screws | Ball screws SFU1605 (Ø16, lead 5, preloaded), BK12 fixed end and BF12 support end |
| Axis drives | 400 W servos (new row, marked assumed), direct coupling. Z keeps its holding brake as a property. |
| Spindle | ER20 cartridge, max 10 000 rpm, 1.1 kW motor, HTD-5M belt 1:1. The spindle is a continuous joint driven by `spindle.speed`. |
| Enclosure | Steel panels on a chip tray, front frame around a 450 × 400 mm opening, sliding door with a polycarbonate window |
| Door | Door on a guide rail, moved by a pneumatic cylinder (Ø25, stroke 460). Door-closed safety switch plus a reed switch at each end of the cylinder. |
| Vise | 100 mm pneumatic vise. The moving jaw is preset by its screw for the blank, then a 6 mm air stroke clamps. Reeds sense clamped and open; an air-gauge sensor at the datum senses part present. |
| Stand | 750 mm. The controller cabinet sits on the side. |
| Air | FRL unit, then a manifold with 5/2 valves (door, vise; gripper on the arm) |
| Part | 608-bearing block from a 60 × 40 × 20 mm 6061 blank: Ø22 seat, two Ø10 counterbores and a contour pocket, one 6 mm end mill |

The current arm reaches about 0.73 m from shoulder to flange, and the gripper adds about 0.15 m. That is marginal through a door to a vise brought to the load position, so the cell uses a cobot from MT3 instead. The 850 mm class is the usual choice for tending a benchtop mill, and the 1300 mm class is the alternative. The MT4 reach study picks the class and the riser height before the gripper or the cell are built.

## Steps

**MT0. Base.**
- Branch `machine-tending` comes from local main 1048bf768, in the worktree `materia-worktrees/machine-tending`.
- Done (2026-10-03): local main b99c948fa was merged in (c5438ba4d). It brings the robot welder through W4's stop policy and clearance: `ArmTool`, `ArmClearance`, `ConvexDistance`, `ProgramPlanner.shutdown()` and the mission machinery.
- `x7-transmissions` through X8e (351e772c5) was merged after that (8ad034403). Its only conflict was `RobotArm.hx`, where X8's driver and gearbox members were kept, along with the welder's `armTool.build`.
- Build on the X7 `Transmission` API and the X8 motor/driver/supply members (`addMotor(id, joint, motor, driver, margin, ?gearbox)`), never on the old `Drive` names.
- `ArmTool` lives in the robot-arm example (`RobotArm.hx`). MT3 moves it into `machinekit.robotics`, so the library's `CobotArm` and the example `RobotArm` share it.
- All submodules are populated: cloned with `--shared` from the x7 worktree at the same pins, with `coal` and `proxsuite` as symlinks to the kinematicskit worktree. Disk is tight, so check `df -h /` before large builds.
- `CADKIT_OCCT_DIR` points at the shared prebuilt OCCT. Never rebuild it.

**MT1. Mill parts.**
- `motion.BallScrew`, with a thread family or variant `Ball(d, lead)`:
  - efficiency about 0.9, zero backlash when preloaded, critical speed as for lead screws;
  - catalogue row SFU1605;
  - `BallNut` with a flange;
  - `ScrewSupportUnit` BK12/BF12 that maps onto `ScrewSupport.Fixed`/`Simple`.
  - The X7 transmission resolver gets the ball-screw relation (efficiency, stiffness, backlash) from these parts.
- HGR15 rail and block rows in the rail catalogue.
- ServoMotor 400 W and 750 W rows, marked assumed like the existing ones.
- An HTD-5M profile for `TimingPulley`/`TimingBelt`, if only GT2 exists.
- Cast parts: `MillBase`, `MillColumn`, `MillSaddle`, `MillTable` (T-slots) and `MillHead`. They are solids with rail pads, screw bores and mounting holes, plus connectors for rails, nut brackets and motors.
- Spindle: `SpindleCartridge` (gauge line connector, ER20 nose), `Er20Holder`, `SpindleMotor` (1.1 kW, max rpm), with the existing `EndMill`.
- Tests: MachineKit unit tests for the ball-screw ratio and efficiency, catalogue rows, part masses, and connector frames.

Done. What was built and decided:
- First full gate `mt1-parts` exposed an unwanted MachineKit → ToolpathKit dependency in the
  spindle holder-profile adapter. Keep the spindle mechanical library independent; the CNC
  example will derive its cutter holder profile from these dimensions in MT2.
- Reuse the existing `Transmission.LeadScrew` resolver: `BallScrew` and `BallNut` are recipe-backed
  screw/nut parts, with a `Ball` race family and a separate `PreloadedBallNut` allowance. Existing
  sliding screws and unpreloaded ball-nut allowances are unchanged. SFU1605 is 16 mm diameter,
  5 mm lead, 90% assumed efficiency, zero assumed preloaded reversal clearance and 0.02 N m drag.
- The ball-screw relation derives shaft-only axial stiffness from race root and full length:
  a 400 mm shaft with an assumed 13 mm race root gives 66366.7 N/mm. Nut/bearing compliance is
  omitted and explicitly labelled assumed; measured total stiffness can override the relation.
- BK12/BF12 support envelopes map to `Fixed`/`Simple`. HGR15/HGH15CA reference rail/block
  dimensions and the 400/750 W servo rows are explicitly assumed. Named standalone servos now
  have recipes; custom rating objects remain code-only. HTD-5M already exists.
- Cast base/column have cores and machined rail pads; saddle has a screw bore; table has three
  10 mm T-slot mouths; head has a cartridge bore. Bodies and mounting holes carry semantic names.
- ER20 cartridge and holder share the gauge-line frame; the generic 1.1 kW spindle motor is rated
  at 8000 rpm and capped at 10000 rpm, with assumed ratings and envelope.
- MachineKit smoke passed on the current parts: computed casting masses are base 50.10 kg,
  column 46.04 kg, saddle 28.41 kg, table 13.30 kg and head 30.67 kg. The 400/750 W servos
  derive rated torques 1.273/2.387 N m at 3000 rpm; maximum speed is 5000 rpm (assumed).
- The first gate passed MachineKit and the application build, but its application run was interrupted
  (exit 130) during the router case. Following X7's documented workaround, run the replacement
  full gate in a separate process session with `setsid`. This is a test-process interruption,
  not a compiler bug. Arm mission 5.8/12/17.2/23.5 s and welder 20.2/19.9 s reproduced unchanged
  before the interruption. The replacement `mt1-final` gate passed every kit, MachineKit smoke,
  the application build and the complete project-source suite. All stated baselines reproduced
  unchanged, including screw/belt router 220.2/201.6 s and removal, CoreXY, arm, mobile and welder.
- Tests include recipe reconstruction, signed screw ratio, preload allowance, shaft stiffness,
  bearing boundaries, rail room, mounting frames, rated power and computed casting masses.

**MT2. The bench mill without enclosure.**
- `BenchMill extends MachineAssembly`, in the cnc-router idiom: `place`/`attach`/`slide`, overtravel from the rail room, `addTransmission`, `supportScrew`, `addMotor`, and `addEncoder` for the servos.
- The spindle belt is `TimingBelt`. The spindle is a continuous joint, not planned. It runs from `spindle.speed` once MT5 provides process-driven joints; until then it stays fixed.
- `BenchMillChecks` in the MachineKit smoke suite cover FK of the gauge line at travel corners, no overlap at the ends of travel, screw ratios, a servo-derived feed of at least 8 m/min rapid, and the BOM.
- `bench-mill/materia.project.json` uses a toe-clamped blank first, with the stock as an assembly part, as on the router. The single-tool bearing-block job comes from `BearingBlockJob` (CamKit) and goes into `scene.machining`.
- Start-page entry.
- `ProjectSourceTests.checkBenchMill` mirrors `checkCncRouter`: removed volume within 2 % of closed form, no gouge, `rapidContacts == 0`, `collisions == 0`, allocation budget, no stalls.

**MT3. Cobot arm size classes.**
- **Joint modules.** `CobotJoint` (`machinekit.robotics`) is a housing with a stator connector and a rotor connector.
  - It contains a `ServoMotor` with a `Gearbox` (strain-wave, ratio about 100, efficiency assumed), plus an output encoder through `addEncoder` (X6d).
  - It comes in sizes 0–4. Reference torques (rated / peak, N·m, approximate public figures): 0 = 12, 1 = 28, 2 = 56, 3 = 150, 4 = 330. Speeds are 180–360 °/s for small sizes and 120 °/s for size 4.
  - Each size's diameter, length and mass are chosen so the class masses below come out within about 10 %.
- **Links.** `CobotLink` is a round tube between two module seats, with the lateral offset of the cobot layout built into its end caps. Each link is one part, so it gets one convex hull.
- **Classes.** `CobotArm(cls:CobotClass, ?tool:ArmTool)`, with class lengths in the usual d1/a2/a3/d4/d5/d6 form. These are reference values from the published DH tables of the typical classes, to be checked against the datasheets when the catalogue is written:

  | Class | Reach | Payload | d1 | a2 | a3 | d4 | d5 | d6 | Modules J1–J3 / J4–J6 | Mass |
  |---|---|---|---|---|---|---|---|---|---|---|
  | `Reach500` (UR3 class) | 500 mm | 3 kg | 151.9 | 243.6 | 213.2 | 131.1 | 85.4 | 92.1 | 2 / 0 | ≈ 11 kg |
  | `Reach850` (UR5 class) | 850 mm | 5 kg | 162.5 | 425.0 | 392.2 | 133.3 | 99.7 | 99.6 | 3 / 1 | ≈ 21 kg |
  | `Reach900` (UR16 class) | 900 mm | 16 kg | 180.7 | 478.4 | 360.0 | 174.2 | 119.9 | 116.6 | 4·4·3 / 2 | ≈ 33 kg |
  | `Reach1300` (UR10 class) | 1300 mm | 12.5 kg | 180.7 | 612.7 | 571.6 | 174.2 | 119.9 | 116.6 | 4·4·3 / 2 | ≈ 34 kg |

- **Mounting.** `CobotArm` stands on a `RobotFlange` base plate. The cell can put it on a `Pedestal` or a riser of any height.
- **Tool.** The tool flange is an ISO 9409-1-50-4-M6 pattern, so `ArmTool`s fit every class. `tcp` and `ready()` work as on `RobotArm`.
- **IK.** KinematicsKit's numeric IK (DLS for tracking, LM for reaching) works on this layout. An analytic solver for the UR-type layout, with its eight closed-form branches, is a later option.
- **Checks** (`CobotArmChecks` in the MachineKit smoke suite), for every class:
  - FK of the flange at zero and at four poses equals the DH table within 0.01 mm;
  - reach (shoulder to flange, fully stretched) within 1 % of the class reach;
  - mass within 10 % of the reference;
  - no self-overlap at zero or at the joint limits taken one joint at a time;
  - the X6 plan check holds the rated payload at the reach, stretched horizontally, without exceeding rated torque on any joint.
- **Preview.** `CobotArmPreview.arm(cls)` gives a project entry per class with a short looping motion, like `robot-arm`, so each class can be looked at and simulated alone. `ProjectSourceTests.checkCobotArms` runs each class's motion on MuJoCo: joint tracking within 1 mrad, no collisions.

**MT4. Enclosure, door, vise and reach.**
- Panels, front frame pieces, chip tray, stand and cabinet as separate parts. The door rides a prismatic joint on its guide, but no actuator moves it yet.
- `PneumaticVise`:
  - body and fixed jaw;
  - moving jaw on a prismatic joint (6 mm stroke), with the screw preset as a construction parameter;
  - parallels and an end stop;
  - a `datum` connector.
- The work offset comes from the datum (MT-D6). Remove the hand-typed offset path for this machine.
- The load position comes from geometry: the axis values that bring the vise datum nearest the door opening.
- The job runs on a blank seated against the datum, still as an assembly part.
- Checks:
  - the door slides clear of the panels over its whole stroke;
  - the head at Z top clears the door opening;
  - the vise opening holds the blank with 1.5 mm clearance per side;
  - every enclosure part's hull error ratio is within the warning threshold.
- **Reach study** (static check `TendingReach`): solve IK for the gripper at the vise (load position, tool down), at every tray slot and at the via-poses outside and inside the door, with at least 10 % joint margin and away from wrist singularity. Start with `Reach850` and try riser heights in 50 mm steps. Use `Reach1300` if no height works, or if the margin is under 10 %. Record the chosen class and riser here.

**MT5. Pneumatics as actuators; switches and presence.**
- Parts:
  - `PneumaticCylinder` (bore, rod, stroke, catalogue: ISO 6432 / ISO 15552 sizes), with ports A and B;
  - `SolenoidValve` (5/2, single or double solenoid) with ports P, A, B and a `Signal` coil;
  - `AirSupply`/FRL with the supply pressure;
  - the manifold reuses `PneumaticManifold`.
  - Hoses from valve to cylinder use `connectPorts`, and the BOM picks them up.
- Assembly: `addCylinder(id, joint, cylinderMember, valveMember)` produces `AssemblyActuator` kind `Pneumatic{bore, rod, stroke, ratedSpeed}`. Pressure is traced through the ports to the supply. This needs a projectkit format bump.
- Bridge → RobotKit `PneumaticDrive` → runtime blueprint:
  - The joint is process-driven: it gets an EFFORT target each tick from the valve channel (± force, damping sized for the rated speed).
  - It is never part of a submitted trajectory.
  - Planning models leave it out, just as CNC planning models already keep only their axes.
- Sensors:
  - `addSwitch(id, joint, window, hysteresis)` produces a digital `joint_switch` sensor kind: reed switches, the door safety switch and the clamped/open switches.
  - `addPresence(id, connector, range)` produces a digital `presence` sensor that is true when a free object's box lies within range of the connector's face: the vise air gauge, tray slot sensors and the gripper's part sensor if used.
  - Both publish sensor frames, and the switch quantity uses hysteresis.
- The spindle's continuous joint becomes velocity-driven from `spindle.speed`, and a process `at_speed` reading replaces the instant answer.
- Tests, pure and MuJoCo:
  - door open/close times within 10 % of stroke ÷ rated speed;
  - vise clamp force = p·A within 5 % (measured from the joint constraint force);
  - switches flip at their windows;
  - a blocked door times out with the switch never reached;
  - the trajectory runtime rejects a plan that touches a process-driven joint.

**MT6. Controllers, robots and signals.**
- Controller parts:
  - `CncController` cabinet: digital I/O ports, axis driver outputs, valve outputs;
  - `RobotController` cabinet.
  - Axis servos, valves and switches are wired to them (X8 driver/controller wiring).
- A controller owns the joints its actuators are wired to (MT-D1). Joints with no controller are an error; parts with no controller are environment.
- Scene v15:
  - `controllers: [{id, kind:"cnc"|"robot", joints, sensors, channels}]`;
  - `machining` and `mission` name their controller;
  - `signals: [{name, from:{controller, output}, to:{controller, input}}]`, derived from the `Signal` wiring.
- App:
  - `AssemblyRobot.add` builds one RobotKit robot per controller, sharing one MuJoCo world.
  - `ApplicationSimulation` attaches `CncProgramPlayer` and `MissionPlayer` to their own robots.
  - A `CellSignals` bus copies outputs to inputs with one tick of latency and feeds each `ManipulatorMotion` input callback and sensor frames.
  - The bench-mill project alone still has one controller and behaves as before.
- Tests:
  - two robots from one assembly with disjoint joints;
  - signals arrive after exactly one tick;
  - `checkBenchMill` unchanged;
  - router, arm and welder project tests unchanged, since a project with no controllers keeps one robot.

**MT7. The mill's robot interface.**
- `cnckit.controller.RobotInterface` is a pure state machine with:
  - **inputs:** `robot.request_door_open/close`, `robot.request_clamp/unclamp`, `robot.cycle_start`, `robot.clear`, `robot.fault_reset`, plus switches and presence;
  - **outputs:** `cnc.ready`, `cnc.door_open`, `cnc.door_closed`, `cnc.clamped`, `cnc.unclamped`, `cnc.part_present`, `cnc.in_cycle`, `cnc.cycle_complete`, `cnc.alarm` (with code), and the door and vise valve outputs.
- It enforces the interlocks:
  - the door opens only when out of cycle, the spindle is stopped and the axes are at the load position;
  - a cycle starts only with the door closed, the vise clamped, a part present and the robot clear;
  - unclamping is refused while in cycle;
  - the door opening during a cycle (switch lost) is a hold plus an alarm;
  - every request has a timeout and becomes an alarm.
- `CncProgramPlayer` idles until cycle start, runs one pass, then raises cycle complete. The program ends with `G53` to the load position (MT-D6). `CamJob`/`CncWriter` gain an end position. Tool-change requests are alarms (MT-D8).
- Tests:
  - pure tests: every interlock refusal and timeout;
  - a scripted-signal sim test with no robot: open door, place the blank by script, clamp, close, start, wait for complete, open, unclamp. Volume as in MT2.

**MT8. The parallel gripper, for real.**
- `ParallelGripper` becomes an assembly:
  - a body;
  - two jaws on prismatic joints, the second mirror-coupled (ratio −1, X5 coupling);
  - an internal piston as a `Pneumatic` actuator on the first jaw. Force and stroke come from a catalogue of generic 2-jaw pneumatic grippers, sizes 25/32/40, marked assumed.
  - fingers as parametric parts (length, pad width, optional V or step for the blank), mounted on the jaws;
  - jaw `joint_switch` windows for open, closed-empty and gripped (the width band of the expected part).
- `ArmGripperTool implements ArmTool`: plate, gripper, valve on the arm (like the welder's feeder seat), hoses, TCP between the finger pads.
- Robot-tool kind `gripper`: scene validation, the cadbridge binding and `EndEffectorControls` (`Gripper` control). The app does not make a `SimulatedSuctionTool` for it, because the jaws are physical.
- `Grasp.find(part)` derives grasp frames from opposite planar faces (MT-D6), with the width and approach direction, ranked by finger fit. Pick and place targets carry a full pose.
- `HandlingPlanRunner` gets oriented approach and retract along the grasp frame's approach axis, and stays compatible with suction.
- The grasp is judged from the jaw switch: the gripped band means OK, closed-empty means missed.
- Tests on MuJoCo:
  - pick, carry and place a blank 50 times on a table;
  - slip relative to the fingers stays under 0.5 mm at the arm's planned accelerations;
  - a missing blank reports a miss;
  - the existing suction mission still passes.

**MT9. Blanks, trays and stock on free bodies.**
- `PartTray` (grid of pockets with chamfers, `slot-r-c` seat connectors, presence sensor per slot optional) for infeed and outfeed, on a cell table.
- N blanks are `dynamicParts` (free boxes) in the infeed slots.
- `MachiningStock` gets one stock per blank. The stock follows the blank's object pose, and the stock is chosen by which blank the vise's presence sensor sees when the cycle starts.
- Physics excludes the cutter–blank pair (MT-D7). Blanks keep contact with the vise, the fingers, trays and each other.
- At cycle complete, the blank's mass becomes stock volume × density, and the stock contour is attached as runtime geometry to the blank's scene node. It is still displayed after the part leaves the machine.
- Tests (scripted robot positions, before the mission exists):
  - a clamped blank moves less than 0.05 mm relative to the vise during the cutting pass, at the mill's real accelerations;
  - removed volume within 2 %;
  - the finished part's mass is within 2 % of the CAD target's;
  - nothing reports `rapidContacts` or `collisions`.

**MT10. The tending cell and its mission.**
- `TendingCell` includes the mill (`include("mill", new BenchMill(...))`), the `CobotArm` class with `ArmGripperTool` on the riser chosen in MT4, the trays, the cabinets, the air supply and the wiring.
- `machine-tending/materia.project.json`, plus a Start-page entry.
- Mission steps, each with a timeout:
  - `signal{name, value}`;
  - `waitFor{name, value, timeout}`;
  - `moveVia{poses}`, joint- or pose-space via-points;
  - oriented `pick`/`place` on `{occurrence, connector}` or a grasp.
- `TendingMission.generate(cell, slots)` produces the steps per part, unrolled like `WeldingMission.generate`:
  1. pick from infeed;
  2. request door open, wait for door open;
  3. via-poses in;
  4. place in the vise, request clamp, wait for clamped and part present, release;
  5. via-poses out, signal clear;
  6. request door close and cycle start, wait for cycle complete;
  7. request door open and go in, grip, request unclamp, wait for unclamped;
  8. come out and place in the outfeed slot.
- Via-poses come from the door opening frame and the vise datum. Every segment is swept with `ArmClearance` against the cell hulls with the door open when the mission is generated (MT-D9).
- "Robot clear" comes from the arm and tool hulls against the machine zone, each tick.
- Acceptance (`ProjectSourceTests.checkMachineTending`, MuJoCo):
  - 4 parts unattended, 4 finished parts in the outfeed;
  - every removed volume within 2 %;
  - 0 collisions and 0 interlock alarms;
  - cycle time and spindle use reported;
  - the welder, router and arm tests are unchanged.

**MT11. Faults and recovery.**
- Injected faults, each ending in the safe state with a named mission failure and an alarm code:
  - empty infeed slot;
  - missed grasp;
  - part dropped in transit (jaw switch leaves the gripped band);
  - clamp with no part (presence false);
  - door jam (switch timeout);
  - cycle alarm (a tool-change request, or a stall from the X6 plan check);
  - e-stop.
- After `robot.fault_reset` and the cause removed, the mission resumes at the step that failed. Parts already finished stay finished.
- Tests: one scenario per fault.

**MT12. Throughput.**
- Pre-staging: while the mill cuts, the arm picks the next blank and waits outside the door.
- A dual gripper (two grippers at 90° on one plate, `ArmTool`) swaps the finished part and the next blank in one door opening.
- The timeline is a per-cycle breakdown: door, load, clamp, cut, unload, idle. The panel shows spindle use.
- Target: spindle use ≥ 75 % for the bearing-block job, with numbers recorded here.

**Later.**
- Real I/O:
  - LinuxCNC HAL pins and `halui`, with `M62`–`M66` in `CncInterpreter` (`ToolpathOp.SetOutput`/`WaitInput`);
  - robot digital I/O through RKD6 device channels;
  - the same `RobotInterface` running on the CNC side or as LinuxCNC logic.
- An umbrella automatic tool changer (`M6` handled by the machine).
- Planned approaches with CL6/CL7 in place of derived via-poses.
- A second operation with a regrip or flip station.
- Tray registration by camera (VisionKit).
- Chips and coolant.
- Mobile tending: the arm on the mobile base, docking, and touch or vision registration to the machine (the welder's P3).

## Order

```
MT0 ─┬─ MT1 ── MT2 ─┬─ MT4 (reach study) ─┐
     ├─ MT3 (arms) ─┘                     ├─ MT6 ── MT7 ─┐
     └─ (welder + X7/X8 merged) ──────────┘              ├─ MT10 ── MT11 ── MT12
                        MT5 (after MT4) ── MT8 ── MT9 ───┘
```

- MT1–MT4 need only X7's transmission API. MT3 can run in parallel with MT1–MT2.
- MT5 is independent of controllers, but the gripper (MT8) and the vise and door actuation need it.
- MT6 needs X8 wiring.
- MT10 needs everything before it.

## Progress

| Step | State | Commits |
|---|---|---|
| MT0 | done: main and X7+X8 merged | c5438ba4d, 8ad034403 |
| MT1 | done | 592affc2f, 7d9f192b1, 5e40b76fe |
| MT2 | planned | |
| MT3 | planned | |
| MT4 | planned | |
| MT5 | planned | |
| MT6 | planned | |
| MT7 | planned | |
| MT8 | planned | |
| MT9 | planned | |
| MT10 | planned | |
| MT11 | planned | |
| MT12 | planned | |
