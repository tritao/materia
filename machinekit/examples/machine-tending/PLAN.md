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
- Main's CL0–CL7 provide `collisionkit.CollisionDescription`, `robotkit.collision.RobotCollision`, the CAD adapter `cadbridge.CadCollision`, and `motionkit.robot.CollisionPlannerSpace` over the coal-backed `CollisionWorld`.
- `robotkit.collision.CollisionClearance` remains the runtime clearance query for named grasp/contact windows. The branch's duplicate `ArmClearance` and `ArmPlanClearance` implementations are removed; generated joint paths use MotionKit's conservative planner.

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
- The cylinder's assumed 2 mm end cushion sets each process joint's tolerated stop compression;
  its critically damped stop rate is derived from moving mass and piston force. The physics
  stop remains at the stroke boundary, while the runtime tolerates that compression.
- The valve's digital coil channels pick the direction. The simulation reads them, owns the valve latch state and applies the cylinder effort every physics substep. The runtime owns only those outputs and leaves the cylinder joint out of motion planning ("process-driven joints").
- Solenoid valves are parts wired to controller outputs; their channel names are derived, not chosen.

**MT-D5. Grip and clamp by contact.**
- Jaw force plus friction holds the blank, in the fingers and in the vise. There is no `holdObjectOnLink` for the gripper or the vise.
- The simulated pneumatic fixture and loose workpiece contacts use an assumed critically damped 1 ms response. Robot hulls receive it only on the parent and child links of pneumatic joints; this keeps the vise contact firm without hardening unrelated mill joints or stalling the spindle against a rigid blank.
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
- Main's `CollisionDescription` and `CadCollision`/`RobotCollision` adapters describe the world and articulated hulls. MotionKit's `CollisionPlannerSpace` conservatively checks generated joint-space edges against that description; a blocked path is replanned or rejected.
- MachineTending supplies typed live-state observations to the shared MT12b description builder and derives approaches from via-poses. The door is open for paths inside the cell.
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
- `RobotArm` keeps its mechanical design and motion limits. The follow-up electrical
  review selects explicit assumed 48 V winding variants and sizes its amplifier,
  source and mobile-welder converter from their demand; this supersedes the earlier
  promise to leave all of its electrical hardware untouched.

**MT-D12. Tray vision uses a wrist camera, not a mast.**
- The arm is idle while the mill cuts, so a look pose over the trays costs no cycle time if it happens then, never on the door's critical path.
- One camera covers both trays at close range, and it measures blanks in the same frame the arm picks in, through the same kinematics. A mast needs a camera, or a wide view, per area plus its own robot registration.
- The camera travels with the arm, which mobile tending needs.
- The camera finds parts; it never feeds an interlock. It never enters the machine (chips and coolant): the vise keeps its air-gauge presence sensor and the gripper its jaw switches.
- **Rejected:** a fixed mast camera. It wins only when the cell needs observation while the arm is busy, which this cell does not.

**MT-D13. One cell description uses the shared CollisionKit and MotionKit APIs.**
- Main now supplies the shared low-level collision description and adapters:
  - `collisionkit.CollisionDescription` stores bodies, articulated models, stable object ids and pair rules;
  - `RobotCollision` describes RobotKit mechanisms, and `cadbridge.CadCollision` adapts CAD parts and mechanism hulls;
  - `CollisionPlannerSpace` provides conservative motion-planning checks, while `CollisionClearance` supports runtime scene changes and named contact windows.
- MT12b uses snapshot → one shared builder → `CollisionDescription`, plus typed ownership and carrier metadata only. TendingReach, app skills, welding and standalone fixtures all enter through a snapshot.
- Snapshot CAD parts use main's `CadCollision.describePart`; observed mechanism shapes use `RobotCollision.describe` and the shared `CollisionShape` vocabulary (`CadCollision.describeMechanism` remains the canonical direct assembly adapter); no second hull transformation or adjacency adapter is retained. Descriptions own object rules and margin classes (or per-object-pair margins). Held stock is checked against everything except explicit typed contact object sets, including on initial construction and on transfer.
- CollisionKit owns held-part attachment and its rules. Windows override the relevant object rule and cache their margin table until the window changes. CAD/tool identities are typed data; consumers never infer ownership, pads or work from display names or suffixes.
- A probe wire is an exact flat-ended cylinder on its tool link in that description; there is no Haxe GJK wire engine and no description-to-`ClearanceBodyData` round trip.
- The static reach study, app mission planner and welder stations consume that cell description. They must not build another collision engine or a second CAD/robot adapter.
- Clearance queries use the shared `CollisionClearance` and `CollisionPlannerSpace` APIs. Do not restore `ArmClearance`/`ArmPlanClearance` or call `ConvexDistance` directly from cell logic.
- CL0–CL7 are now in main, including the coal native backend and its test corpus. MT12b tests the MachineTending cell-description adapter against those shared APIs; it does not compare duplicate engines.
- **Known limitation:** planning uses a snapshot of the other robot's and the loose objects' poses taken when a skill starts. It is safe here because the interlocks keep the mill still while the arm is inside. The collision plan's pose-driven world removes it.

## The machine (starting numbers, fixed in MT1–MT4)

| Item | Value |
|---|---|
| Travel X / Y / Z | 250 / 150 / 250 mm |
| Table | 400 × 130 mm, three 10 mm T-slots |
| Structure | Cast base, column bolted to the base's back, saddle (Y) on the base, table (X) on the saddle, head (Z) on the column |
| Rails | HGR15 on all axes (new catalogue rows) |
| Screws | Ball screws SFU1605 (Ø16, lead 5, preloaded), BK12 fixed end and BF12 support end |
| Axis drives | X/Y 400 W and Z 750 W assumed servos, direct coupling (R2). The physical Z brake remains open in the review-fix R3 group. |
| Spindle | ER20 cartridge, max 10 000 rpm, 1.1 kW motor, HTD-5M belt 1:1. The spindle is a continuous joint driven by `spindle.speed`. |
| Enclosure | Steel panels on a chip tray, front frame around a 600 × 500 mm opening, sliding door with a polycarbonate window |
| Door | Door on a guide rail, moved by a pneumatic cylinder (Ø25, stroke 610). Door-closed safety switch plus a reed switch at each end of the cylinder. |
| Vise | 100 mm pneumatic vise. The moving jaw is preset by its screw for the blank, then a 6 mm air stroke clamps. Reeds sense clamped and open; an air-gauge sensor at the datum senses part present. |
| Stand | 750 mm. The controller cabinet sits on the side. |
| Air | FRL unit, then a manifold with 5/2 valves (door, vise; gripper on the arm) |
| Part | 608-bearing block from a 60 × 60 × 20 mm 6061 blank: Ø22 seat, two Ø10 counterbores and a contour pocket, one 6 mm end mill |

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
- MachineKit smoke passed on the corrected castings: current computed masses are base 58.6401 kg,
  column 50.9425 kg, saddle 25.4801 kg, table 11.7125 kg and head 33.2253 kg. MT5b's paired
  guide supports lengthen the column, widen the saddle and raise the head; these replace the
  previous 49.54/22.43/30.67 kg measurements for those three bodies. The 400/750 W servos
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

Done. What was built and decided:
- X/Y are moving-table coordinates: the table and saddle move against their positive CNC axes,
  while the head moves with Z. The zero poses derive from rail/block heights, casting seats,
  cartridge mount and tool length; overtravel is measured from rail room.
- Fitting the swept saddle and real bearing supports changed the designed base depth to 550 mm
  and saddle depth to 130 mm. The X nut needs matching clearance recesses in the saddle and
  table; its flange adapter and the Y screw bore share dimensions with `BallNut`. Review found
  the original 45 mm saddle severed by its 50 mm bore. The saddle is now 70 mm high with a
  continuous lower web; the column is 650 mm high to preserve Z rail room. The X bracket has
  two table bearing ears and the Y bracket mounts on the saddle's rear bore face. CAD checks
  confirm finite bearing area and no bracket/casting overlap at all travel extremes.
- SFU1605 now has assumed 12 mm machined journals (50 mm input, 25 mm output), so the BK12/BF12
  seats and coupling fit physical shaft surfaces. Screw speed still conservatively uses full length.
- The single-tool exercise has a 22 mm seat 7 mm deep, two blind 10 mm fastening recesses 5 mm
  deep, and a 12 × 12 mm contour pocket 3 mm deep with 3.5 mm corners. Blind recesses keep this
  first setup a single 6 mm tool exercise. The seat remains centred. R6 enlarges the blank depth
  to 60 mm and locates the contour from a 3 mm front wall, leaving 4 mm to the bearing seat
  and keeping the back-corner toe clamps clear. CAM reads pocket boundaries and floors from the solid.
- The CNC player previously applied a pulse-channel binding to every actuator, which rejected
  servo motors. It now derives pulse channels only for steppers and retains the servo motor/driver
  limits; mixed-drive binding tests cover this. RobotKit world assertions grow from 4926 to 4930.
- The rounded contour exposed an open-arc fitting defect: approximating the last vertex
  moved the fitted endpoint, making the following G-code arc inconsistent with its modal start.
  Open fits now interpolate both endpoints; closed-circle fits keep their existing construction.
  Three regression assertions raise CamKit from 12308 to 12311. The full gate preserves both
  router cycle times, removal, plan checks and tracking deviations.
- A contour corner exactly equal to the cutter radius collapses the sampled polygon offset.
  Giving the contour a larger designed radius avoids that degeneration without changing CamKit.

- Keep the controller period at 10 ms. The runtime supplies exact polynomial position and velocity
  at both tick endpoints. The drive evaluates their cubic Hermite interpolant at physics substeps,
  and adds reflected inertia times its second derivative as torque feedforward. Endpoint prediction
  shares the existing hold/resume clock integration; straight stop ramps supply their analytic endpoints.
  Reported servo-machine setpoints use the interval end, matching the measured physics timestamp.
- The generic servo amplifier assumes a 4000 Hz position loop. The blueprint reports the fastest
  specified loop and a conservative gain/inertia interval of `0.5 sqrt(J/k)`. SimulationSpace chooses
  the stricter interval and implicit-fast integration; ApplicationSimulation only attaches the space.
  The driver rate is optional assembly field 23 and survives nested assembly flattening.
- Coupling stiffness comes from the X7 transmission relation, converted to follower coordinates
  (`k_follower = k_leader / ratio²`). MuJoCo derives critical-damping solref from follower inertia,
  with a lower time constant of twice the physics timestep. No transmission-wide hardening override.
  See [MuJoCo solver parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters).
- A HashLink allocation census identified over ten million temporary endpoint `Pose3` objects
  in the authored-path distance check. Evaluate point-to-line distance directly from coordinates,
  preserving its arithmetic, task-space samples and tolerance. No allocation budget or tick change.
- Native tests cover Hermite tracking of an accelerating reference at 10 ms, force saturation with
  implicit integration, and matching tick endpoints across hold/resume/stop and a queued replan. All native MuJoCo and
  runtime tests pass, including normal queue underflow and a stop that outlives the queued path.
- The 72-member mill derives X rapid 15.10 m/min and Y/Z rapid 25.00 m/min; acceleration limits
  are X 30.06, Y 31.09 and Z 30.03 m/s². The job cuts at 1.2 m/min with an 8000 rpm spindle.
- At the 10 ms controller period, the drive requires an interval no longer than 0.248589 ms:
  41 physics substeps. Peak tracking error is X 0.07537, Y 0.05032 and Z 0.02937 mm.
- The bearing block takes 49.98 s: 3716.58 of 3702.78 mm³ removed, leftover 7.22 mm³, gouge
  0.58 mm³ (within the 1 mm³ numerical cutting allowance), no rapid contacts or collisions.
  Its 77 plans include 56 flagged plans: 42 peak-torque and 35 rated-RMS-torque findings.
  There are no stalls or accuracy findings. These torque findings remain visible sizing limits.
- Allocation is 68 KB per controller tick, below the unchanged 80 KB budget. All kit suites,
  MachineKit smoke and the full application gate pass; existing router, belt router, CoreXY,
  arm, mobile-base and welder measurements are unchanged. Only the added assertion counts grow.
  The final gate is `mt-suite-mt2-final.txt` (every suite exit 0); native runtime and MuJoCo
  assertion executables also pass. No haxeon compiler workaround was needed.

**MT3. Cobot arm size classes.**
- **Joint modules.** `CobotJoint` (`machinekit.robotics`) is a housing with a stator connector and a rotor connector.
  - It contains a `ServoMotor` with a `Gearbox` (strain-wave, ratio about 100, efficiency assumed), plus an output encoder through `addEncoder` (X6d).
  - It comes in sizes 0–4. Reference torques (N·m, assumed engineering values): 0 = 12, 1 = 28, 2 = 56, 3 = 150, 4 = 330 (assumed rated torques; peak twice rated). Speeds are 180–360 °/s for small sizes and 120 °/s for size 4.
  - Each size's diameter and length are assumed module envelopes. The assembled mass is computed
    from the parts; the maker's complete-arm mass is a reference, not an exact target for generic internals.
- **Links.** `CobotLink` is a round tube between two module seats, with the lateral offset of the cobot layout built into its end caps. Each link is one part, so it gets one convex hull.
- **Classes.** `CobotArm(cls:CobotClass, ?tool:ArmTool)`, with class lengths in the usual d1/a2/a3/d4/d5/d6 form. Dimensions are checked against the maker's [DH table](https://www.universal-robots.com/developer/hardware-and-motion/robot-motion-dh-parameters/); payload, joint speeds and mass against the [November 2023 technical data sheet](https://www.universal-robots.com/media/1829346/11_2023_collective_data-sheet.pdf). The table gives magnitudes of a2/a3; DH uses negative values:

  | Class | Reach | Payload | d1 | a2 | a3 | d4 | d5 | d6 | Modules J1–J3 / J4–J6 | Mass |
  |---|---|---|---|---|---|---|---|---|---|---|
  | `Reach500` (UR3 class) | 500 mm | 3 kg | 151.85 | 243.55 | 213.2 | 131.05 | 85.35 | 92.1 | 2 / 0 | 11.2 kg |
  | `Reach850` (UR5 class) | 850 mm | 5 kg | 162.5 | 425.0 | 392.2 | 133.3 | 99.7 | 99.6 | 3 / 1 | 20.6 kg |
  | `Reach900` (UR16 class) | 900 mm | 16 kg | 180.7 | 478.4 | 360.0 | 174.15 | 119.85 | 116.55 | 4·4·3 / 2 | 33.1 kg |
  | `Reach1300` (UR10 class) | 1300 mm | 12.5 kg | 180.7 | 612.7 | 571.55 | 174.15 | 119.85 | 116.55 | 4·4·3 / 2 | 33.5 kg |

- **Mounting.** `CobotArm` stands on a `RobotFlange` base plate. The cell can put it on a `Pedestal` or a riser of any height.
- **Tool.** The tool flange is an ISO 9409-1-50-4-M6 pattern, so `ArmTool`s fit every class. `tcp` and `ready()` work as on `RobotArm`.
- **IK.** KinematicsKit's numeric IK (DLS for tracking, LM for reaching) works on this layout. An analytic solver for the UR-type layout, with its eight closed-form branches, is a later option.
- **Checks** (`CobotArmChecks` in the MachineKit smoke suite), for every class:
  - FK of the flange at zero and at four poses equals the DH table within 0.01 mm;
  - shoulder-to-flange distance at DH zero equals the distance derived from the verified DH offsets;
    the published nominal reach remains class metadata, rather than changing DH lengths to fit it;
  - assembled mass matches its checked generic design value within 0.02 kg and is reported beside
    the maker's complete-arm reference; assumed gearbox changes are recorded explicitly;
  - no self-overlap at zero, ready, and three additional nontrivial DH poses; revolute ±2π
    endpoints are not counted separately because they are the same physical configuration as zero;
  - assembly-derived gravity moments hold the rated payload at the horizontally stretched DH-zero
    pose without exceeding rated torque on any joint. X6 steady loads currently model sliding
    gravity only, so its rotary plan check cannot establish this static holding claim.
- **Preview.** `CobotArmPreview.arm(cls)` gives a project entry per class with a short looping motion, like `robot-arm`, so each class can be looked at and simulated alone. `ProjectSourceTests.checkCobotArms` runs each class's motion on MuJoCo: joint tracking within 1 mrad, no collisions.

Done. What was built and decided:
- `ArmTool` and the existing suction implementation move into `machinekit.robotics`; the arm
  and welder keep the same construction, connectors and service wiring.
- Four size classes use the verified e-series DH offsets, joint speed limits and reference masses.
  Housing dimensions, internal construction, 100:1 reduction, 0.85 efficiency, torque ratings,
  80%-of-maximum rated motor speed, rotor inertia and output encoders remain assumed values.
  Position-loop stiffness assumes peak torque at 10 mrad motor error; damping is critical for
  the stated rotor inertia. Explicit gains select the physical interpolated drive model.
- Link seats subtract the preceding module length from DH translation, and the final seat
  subtracts the real flange thickness. Tube routes clear stator rims and gearheads leave room
  for the base flange pilot. Five FK poses and the full-turn limits pass for every class.
- Computed arm masses (offboard amplifiers/supply and optional tool excluded) are 11.450,
  19.484, 33.119 and 33.380 kg. The rated-payload shoulder moments are 29.666, 95.026,
  234.048 and 288.315 N m; elbow moments are 10.402, 35.134, 89.853 and 123.446 N m.
- After integrating main's derived steel-class `Gearbox.massKg` assumption, assembled masses are
  13.616, 19.484, 35.342 and 35.603 kg for Reach500/850/900/1300. Published complete-arm
  masses remain 11.2/20.6/33.1/33.5 kg. The generic gearhead model changes the assembled
  mass and gravity loads, especially for Reach500; its assumptions are not vendor internals.
  The updated rated-payload shoulder moments are 31.436, 95.026, 252.330 and 314.138 N m.
  The MachineKit smoke passes with these model-derived values; no reference row was rewritten.
- The rebased link-envelope and transmission integration now derives assembly masses of 14.72584,
  21.12097, 37.39430 and 37.73141 kg. `CobotArmChecks` asserts each checked assembly mass within
  0.02 kg and reports the maker's reference separately; it no longer accepts any positive mass
  under a broad percentage tolerance. These are values of the assumed generic module internals,
  not corrections to the published 11.2/20.6/33.1/33.5 kg complete-arm references.
- The proposed 1% equality between catalogue reach and a fully stretched DH flange distance
  does not follow from the verified offsets. Keep the source geometry and check its explicit
  shoulder reference distance instead; do not rescale a2/a3 to force the catalogue label.
- Each class gets a project/Start-page entry and a planned free-motion loop through a generic
  `moveJoints` mission step. Typed targets are absolute mechanical coordinates in radians/metres;
  MotionKit `MoveJ` and the trajectory runtime execute them. The 1 mrad check applies to free
  motion: the handling runner deliberately presses 3 mm into a rigid blank, so contact reaction
  forces make its pick dwell unsuitable for a free-motion tracking requirement. Existing arm
  handling tests continue to cover that separate behavior.
- The three-step free-motion loop takes 2.06 s (206 ticks) for each class. Peak errors,
  in class order, are 0.00475, 0.00631, 0.01115 and 0.01723 mrad at the unchanged 10 ms
  controller tick; no self-collision. The DH-zero shoulder reference distances are 515.462,
  855.569, 895.425 and 1225.283 mm.
- The complete `mt3-session` gate passes every kit, CAD, MachineKit smoke, application build
  and full project-source suite. ProjectKit grows from 146 to 151 assertions for mission-data
  round trip, duplicate/unknown/nonfinite targets and SI conversion of mechanical limits.
  All router, belt, CoreXY, arm, mobile, welder and MT2 mill measurements are unchanged.
- The first gate's application run received SIGINT (exit 130) during a project compilation,
  with no assertion failure. A separate session with SIGINT ignored completed the replacement
  full gate. No materia code change or haxeon compiler workaround was needed for it.

**MT4. Enclosure, door, vise and reach.**
- Panels, front frame pieces, chip tray, stand and cabinet as separate parts. The door rides a prismatic joint on its guide, but no actuator moves it yet.
- `PneumaticVise`:
  - body and fixed jaw;
  - moving jaw on a prismatic joint (6 mm stroke), with the screw preset as a construction parameter;
  - parallels and an end stop;
  - a `datum` connector.
- The work offset comes from the datum (MT-D6). Remove the hand-typed offset path for this machine.
- The load position comes from geometry: the axis values that bring the vise datum nearest the door opening.
- Export the job on a blank seated against the datum, still as an assembly part. Keep the running bare-mill entrypoint for this step; enable the enclosed physical cut in MT5 after its jaw has a real clamp force. A free unactuated jaw cannot hold its preset during table acceleration.
- Checks:
  - the door slides clear of the panels over its whole stroke;
  - the head at Z top clears the door opening;
  - the vise opening holds the blank with 1.5 mm clearance per side;
  - every enclosure part's hull error ratio is within the warning threshold.
- **Reach study** (static check `TendingReach`): solve IK for the gripper at the vise (load position, tool down), at every tray slot and at the via-poses outside and inside the door, with at least 10 % joint margin and away from wrist singularity. Start with `Reach850` and try riser heights in 50 mm steps. Use `Reach1300` if no height works, or if the margin is under 10 %. Record the chosen class and riser here.

Done. What was built and decided:
- `MillPanel` is recipe-backed; enclosure walls, roof, four front-frame pieces, chip tray,
  stand deck/legs and six cabinet panels are separate convex parts. The window uses reference
  polycarbonate density 1200 kg/m³. The default bare-mill project keeps running; a separate
  enclosed Start-page project previews the mechanics until MT5 supplies clamping force.
- The mill's swept part bounds are [-585, -455, 0]..[445, 275, 900] mm. They determine panel
  positions on the 750 mm stand and 2 mm tray. The 600 × 500 mm opening spans Z 888..1388 mm,
  with its sill 400 mm below the retracted head. The door travels 610 mm; an invariant Y
  separating plane proves clearance over its complete X stroke, with CAD samples as a second
  check. Every panel/window hull stays within 5% error.
- The preset vise has 43 mm open jaw separation for the 40 mm blank (1.5 mm per side), a 6 mm
  guided air stroke, two 10 mm parallels and an end stop. Its datum derives from the actual
  locating faces, giving G54 [95, 55, -230] mm. Projection of that datum onto the table travel
  gives the load position [95, 150, 0] mm. The typed toolpath ends with retract-then-position
  `MachineMove` operations; export/recompile verifies their final G53 coordinates.
- The MT4 placeholder-tool study chose **Reach850 on a 600 mm riser**, with 15 poses (vise, two door
  via-poses, twelve tray slots), 16.23993% minimum joint margin and minimum |sin(q5)| ≈ 1.
  Review fixed the tray poses in the cell frame and added both tray decks to the clearance
  scene; the earlier 550 mm riser collided with the outfeed tray at an infeed pose. It checks
  the actual mill, open-door and tray hulls with 5 mm clearance (1 mm for the tool only at the
  vise); the grasped blank is the intended contact target. The future gripper is explicitly
  an assumed 80 × 50 × 150 mm body/finger envelope with 16 mm opening travel, not an MT8
  operational gripper. World TCP frames and solved joints were returned for the later cell;
  MT8 replaces this provisional result with the physical gripper and records its final reach below.
- The doorway must pass the wrist as well as the tool: via height comes from its header,
  tool length and wrist-module geometry. Numeric IK checks alternate elbow/wrist branches
  when its first solution hits a panel; the first outfeed solution otherwise crossed the
  right front frame. No clearance or joint-margin threshold was relaxed.
- At the retracted Z limit, the enclosure check intersects the actual head casting with the
  top frame and every closed-door panel. It verifies physical solid clearance instead of
  comparing the opening edge with the same reference pose used to place it.
- The original branch's Haxe convex-distance cutoff and `ArmClearance` engine
  were superseded by main's CollisionKit native world. Collision coverage now
  runs through `collisionkit.CollisionWorld`; RobotKit retains its runtime
  adapter for clearance windows rather than a second distance implementation.
- A diagnostic enclosed MuJoCo cut exposed the unactuated jaw drifting during table
  acceleration (fault at 35.86 s). MT4 therefore exports and verifies the seated job but
  leaves its physical cycle for MT5, instead of adding an artificial jaw lock or motor.
  With the vise mass, derived acceleration is X 29.56 / Y 30.55 / Z 30.03 m/s²; rapid rates
  remain X 15.10 / Y 25 / Z 25 m/min.
- Full gate `mt4-final` passed every kit, CAD, MachineKit smoke, application build and complete
  project-source suite. The focused RobotKit tool/process/weld/clearance suite also passed,
  including the three new distance-cutoff assertions. All starting baselines and MT3 class
  tracking numbers are unchanged. The bare mill still takes 49.98 s, removes 3716.5811 mm³,
  and tracks X/Y/Z within 0.075366/0.050317/0.029369 mm at 10 ms. Its allocation measurement
  is 67.5 KB/tick after moving fault-message construction off the test's successful tick path;
  this is a measurement-harness change, not a simulation performance claim. The enclosed
  project generation contains 103 parts and all five mechanical slides.

**MT5. Pneumatics as actuators; switches and presence.**
- Parts:
  - `PneumaticCylinder` (bore, rod, stroke, catalogue: ISO 6432 / ISO 15552 sizes), with ports A and B;
  - `SolenoidValve` (5/2, single or double solenoid) with ports P, A, B and a `Signal` coil;
  - `AirSupply`/FRL with the supply pressure;
  - the manifold reuses `PneumaticManifold`.
  - Hoses from valve to cylinder use `connectPorts`, and the BOM picks them up.
- Assembly: `addCylinder(id, joint, cylinderMember, valveMember)` produces `AssemblyActuator` kind `Pneumatic{bore, rod, stroke, ratedSpeed}`. Pressure is traced through the ports to the supply. This needs a projectkit format bump.
- Bridge → RobotKit `PneumaticDrive` → runtime blueprint:
  - The joint is process-driven: the simulation reads the digital valve coils and applies an EFFORT target each tick (± force, damping sized for the rated speed).
  - A trajectory may carry its constant observed coordinate for full-robot indexing, but cannot move it; the runtime emits no position target for it.
  - Planning models leave it out, just as CNC planning models already keep only their axes.
- Sensors:
  - `addSwitch(id, joint, window, hysteresis)` produces a digital `joint_switch` sensor kind: reed switches, the door safety switch and the clamped/open switches.
  - `addPresence(id, connector, range)` produces a digital `presence` sensor that is true when a free object's box lies within range of the connector's face: the vise air gauge, tray slot sensors and the gripper's part sensor if used.
  - Both publish sensor frames, and the switch quantity uses hysteresis.
- The spindle's continuous joint becomes velocity-driven from `spindle.speed`, and a process `at_speed` reading replaces the instant answer.
- Emergency stop de-energises both door and vise valve coils. Both use double-solenoid valves, so their spools hold the last selected outlet and the trapped supply force remains; the FRL stays pressurised because this cell has no dump valve. This is a simulation decision, not a safety-rated machine design.
- Tests, pure and MuJoCo:
  - door extension within 10 % of stroke ÷ rated speed; retraction uses the annular-to-bore force ratio with the shared flow damping;
  - vise clamp force = p·A within 5 % (measured from the joint constraint force);
  - switches flip at their windows;
  - the trajectory runtime rejects a plan that touches a process-driven joint.

The blocked-door request timeout is tested in MT7 with `RobotInterface`, which owns requests,
switch interpretation and alarms; MT5 supplies the physical switch and cylinder only.

Done. What was built and decided:
- Reconstructible ISO 6432/15552 cylinders, moving rods, 5/2 valves and regulated supply;
  pressure follows A/B hoses through the manifold. All 13 bore/rod reference rows agree with
  the rounded theoretical 6 bar force tables in [Festo DSNU (2012), p. 13](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2012/EN/DSNU-ISO_EN.PDF)
  and [Festo DSBC (2024), p. 14](https://www.festo.com/media/pim/132/D15000100122132.pdf).
  Envelopes, fittings, flow-limited speed and end-cushion compression are assumed.
- The vise owns one cylinder body and rod and exports its air ports. The enclosure owns the
  valves, FRL and hoses. Saved assemblies keep actuator, switch, presence and process-velocity
  bindings through flattening and codec round trips.
- The runtime owns only process output channels. The simulation owns valve latch state, piston
  force and spindle velocity. An emergency stop drops both coils; double-solenoid spools hold
  their last outlet and the FRL stays pressurised without a dump valve. The runtime rejects
  planned motion on process joints and omits them from endpoint position commands.
- The 25/10 mm door cylinder delivers 294.5243 N advancing and 247.4004 N retracting;
  the 32/12 mm vise delivers 482.5486 N advancing and 414.6902 N retracting. MuJoCo measured
  door open 1.50 s, close 1.79 s, and vise clamp 0.18 s at 482.5486 N. Retraction is slower
  because its annular area produces less force against the same flow damping.
- The bare mill takes 51.12 s with a physical spindle at-speed wait, 78 plans and the same
  0.075366/0.050317/0.029369 mm X/Y/Z tracking peaks and 3716.5811 mm³ removed. Its
  earlier 49.98 s cycle moves by 1.14 s for spindle spin-up; the vise rod is only in the
  enclosed assembly and does not change the bare mill mass or tracking.
- The enclosed Start-page project now runs its pressure-clamped bearing-block job. With the
  vise spool initially latched to clamp, MuJoCo completes one pass in 51.73 s and removes
  3716.5718 mm³, with no rapid or holder contact.
- MachineKit smoke passed with cylinder force, valve, pressure, hose, saved-format, switch
  and vise checks. RobotKit checks cover coil emergency stop and trajectory rejection;
  the app's MuJoCo check covers physical timing and force. The CNC panel's hold, resume and
  restart check exposed a one-tick race at a completed plan boundary; MotionKit now waits for
  the runtime owner to apply Resume before starting another plan, and retires a held plan
  already at its endpoint without sending a stale Resume.
- The output-before-input barrier now has a one-controller-cycle stationary plan so spindle
  outputs reach the physical spindle before its at-speed wait. This changes both router jobs
  from 126 to 128 plans. The screw router remains 220.2 s with 0.05 mm worst deviation;
  the belt router takes 201.7 s (was 201.6 s), has 56 flagged plans (was 57) and 62 over
  tolerance (was 64), with the same 1.89 mm worst deviation. The changed plan boundaries,
  rather than a change to belt or screw mechanics, account for these numbers.
- The `mt5-typed-final` full gate passed: RobotKit world 4947, MotionKit 9762, CadBridge 156,
  CncKit 317, CamKit 12311, ProjectKit 151, and ProcessKit 23 plus welder 52 assertions;
  MachineKit smoke and app project-source passed. CoreXY, arm mission, mobile base and welder
  timing and geometry baselines stayed unchanged.

**Post-MT5 integration with main (RobotKit R0–R6, welder W4 and transmission X9–X10).**
- Process-owned spindle and pneumatic actuators are excluded from `DriveLoads` planner budgets;
  their typed channel bindings and simulation forces remain active. The merged runtime uses the
  profile-aware compiler and main's nullable limit semantics. The current main submodule pins
  are inherited unchanged; no new pin was chosen for machine tending.
- The screw router remains 220.2 s, 128 plans and 0.05 mm worst deviation. Main's belt path
  and elasticity work changes the belt router's worst Y deviation from 1.89 to 1.76 mm; it
  remains 201.7 s with 128 plans, now 56 flagged and 62 over tolerance. CoreXY still reaches
  649.6 mm/s; its derived X acceleration is 71.549 rather than 71.577 m/s² after main's
  transmission updates.
- The arm mission remains at pick 5.8/place 12/pick 17.2/place 23.5 s. The mobile mission's
  final goTo moves from 57.6 to 61 s, and its obstacle round from 59 to 65 s, with main's
  updated wheel/controller model. Main's W4 weld planning and stop-policy changes move the
  MuJoCo welder from 20.2/21.6 to 20.6/22.0 s (one restart, 11 mm overlap), and the
  four-sided post from 19.9 to 29.8 s. The mill remains 51.12 s bare and 51.73 s enclosed, with
  0.075366/0.050317/0.029369 mm X/Y/Z tracking, 1.50/1.79 s door and 0.18 s vise at 482.55 N.
- Main's removal of legacy saved-format tests changes ProjectKit's assertion count from 151
  to 136. RobotKit world rises from 4947 to 4993 and MotionKit from 9762 to 9869 through
  main's new checks; CadBridge rises 156 to 157. CncKit remains 317, CamKit 12311,
  ProcessKit 23 plus welder 52. Main's W4 far-plate welder case completes in 20.86 s
  with a 4.9973 mm first leg. The R6 sequential gate is `mt-suite-final-r6.txt`.

**Post-MT5 integration with main's welder W5.**
- W5's weld-pass planning and scene schema merge with the machine tending joint-motion mission
  path. The ordinary MuJoCo seam remains 20.6 s; recovery improves from 22.0 to 21.1 s,
  with one restart and overlap reduced from 11 to 3 mm. The four-sided post remains 29.8 s.
  The 7 mm woven pass measures 6.998 mm in 27.17 s, and the three-pass 10 mm weld measures
  9.999 mm in 65.55 s. Both backends complete the ten-seam weldment in 106.5 s.
- The bare and enclosed mill remain 51.12/51.73 s, with the same axis tracking, door and vise
  measurements above. Router, CoreXY, arm and mobile baselines also remain unchanged from the
  R6 integration. W5 adds 19 weld-pass path assertions to ProcessKit's 23 and raises
  ProjectKit from 136 to 143; RobotKit world remains 4993. The complete sequential gate is
  `mt-suite-final-w5.txt`; all kits, MachineKit smoke, app build and project-source pass.

**Post-MT5 local main sync (RobotKit R7, motion loose ends and transmission X9e).**
- The merged runtime keeps the machine tending process channels and servo reference clock while
  applying main's per-plan acceleration budget to stops and resumes. A plan's control-acceleration
  vector is per joint, so a two-joint, one-segment pneumatic plan must allocate two entries.
  The merged mission builder retains joint-motion frames alongside main's virtual device, and
  motion completion waits for a settled final state.
- Main's measured-endpoint and exact-stop changes move the bare mill from 51.12 to 51.19 s
  (5111 to 5118 ticks) and the enclosed mill from 51.73 to 51.92 s. Removed volume and
  0.075366/0.050317/0.029369 mm X/Y/Z peak tracking remain stable. The door remains
  1.50/1.79 s and the vise 0.18 s at 482.55 N. The same stop-policy change moves the screw
  router from 220.2 to 232.2 s and the belt router from 201.7 to 215.2 s; both retain 128
  plans, their material-removal figures and 0.05/1.76 mm worst deviations. CoreXY, arm,
  mobile, cobot and welder measurements remain unchanged.
- The latest RobotKit world suite passes 4997 assertions, MotionKit 67533, and toolpath motion
  2952 scenario assertions; the other kit counts and MachineKit smoke remain as above. The
  full sequential gate is `mt-suite-main-sync-final.txt`.

**Post-MT5 review fixes (MT5b).**
- The vise's clamped reed window follows its preset seated-blank jaw position (3 mm), and
  excludes the empty-vise end stop. The MuJoCo fixture now stops the jaw against a seated
  blank surrogate and measures **0.08 s at 482.5486 N**; the earlier 0.18 s measurement
  was the empty-vise full stroke. MT7 will replace that surrogate with a loose blank.
- The spindle at-speed input compares actual joint speed to the current commanded speed
  within 2% tolerance and 1% hysteresis. Its threshold no longer depends on starting
  angle or a fixed 7600 rpm. Crossed A/B hoses swap the extend/retract coil meanings
  of a double-solenoid valve; single-solenoid valves retain their spring-side logic.
  A normal stop retains a single-solenoid coil; emergency stop de-energises it.
- Saved machine assemblies preserve cylinders and sensors, nested spindle speed drives,
  and their actual schema version. MachineKit v10-13 and ProjectKit v4-7 gate newly
  saved features; ProjectKit v7 includes servo loop rate and command-relative at-speed.
  A pneumatic actuator without its drive kind is rejected. MotionKit clears a deferred
  refill when a plan finishes just after resume, allowing the program to advance.
- Servo acceleration feedforward becomes zero after its reference interval. Presence
  tests overlap the sensor's oriented volume with loose dynamic boxes, excluding fixed
  fixtures, actors and carried boxes; an offset blank can therefore register.
- The corrected saddle web and nut brackets reduce the derived Z rapid from 25 to
  **22.8666 m/min**. The bare bearing-block cycle is **51.86 s**, 78 plans, with
  0.077247/0.051552/0.043834 mm X/Y/Z peak tracking; the enclosed cycle is
  **52.30 s** with the same removal volume. Door open/close stay **1.50/1.79 s**.
- The MT4 placeholder-tool reach study fixed tray targets in the cell frame, included both tray decks,
  limits its 1 mm contact allowance to the vise, and asserts its result. It selects
  **Reach850 on a 600 mm riser**, 15 poses and 16.23993% minimum joint margin at that checkpoint.
  The operational gripper changes that geometry; its replacement study and result are recorded in MT8.
  Cobot mass checks use measured CAD masses rather than positivity alone. The unused
  `ConvexSolid.distanceBelow` helper and its tests were removed; clearance tests now
  exercise the production `ConvexDistance.between` path.
- The sequential `mt-suite-mt5b-review.txt` gate passed: RobotKit world 4999,
  MotionKit 67533, CadBridge 157, CncKit 317, CamKit 12311, ProjectKit 144,
  ProcessKit 23 plus 19 weld-pass assertions, MachineKit smoke and the full app
  project-source run. Direct SimKit session and MuJoCo native tests also passed.
  The screw/belt router remain 232.2/215.2 s and 0.05/1.76 mm worst deviation;
  CoreXY remains 649.6 mm/s and 71.549 m/s², arm and mobile mission times stay
  unchanged, and the welder remains 20.6/21.1 s for the two seam cases and 106.5 s
  for the ten-seam weldment. The geometry correction alone moves the bare mill from
  51.19 to 51.86 s and the enclosed mill from 51.92 to 52.30 s.

**Remaining review fixes, R1–R2 (MT5b).**

Done. What was built and decided:

- Require one current MachineKit schema (v14 after hardware recipe changes), ProjectKit v7,
  and the complete native runtime layouts. In-repo producers and tests migrate together;
  the earlier v10–13/v4–7 feature gates are superseded, with no compatibility fallback.
- Separate spindle motor and cartridge rotation with the actual assumed 1:1 HTD5M pulley
  transmission. The belt explicitly states assumed cord rigidity of 5,000 N/mm width;
  readiness follows the commanded motor speed through the relation. Both coupled joints
  belong to the process drive and cannot receive motion-plan targets.
- Derive exposed shaft and coupling dimensions from assumed motor rows (400 W: 14×30 mm;
  750 W: 19×35 mm; spindle: 20×30 mm). Geometry checks verify actual shaft engagement.
  The Z motor is 750 W to meet vertical-axis load, superseding the original 400 W proposal.
- Check servo phase RMS current, back EMF, winding losses, amplifier efficiency, combined
  source draw and regeneration. The follow-up reference design uses an explicit 400 V,
  50 Hz three-phase mains boundary and capacitor rectifier (565.685 V nominal DC bus),
  a 20 A DC source, 8 kW shunt, matching 400 V class windings and 20 A phase-RMS
  amplifiers. These are assumed component ratings, not a commercial hardware selection.
  Mobile arms use independently sized 48 V winding variants; cobot amplifiers are sized
  per module rather than all being just above the largest joint requirement.
  AC RMS/DC voltages and DC/phase-RMS currents are distinct quantity types.
  Larger offboard arm hardware changes assembly mass from 150.1 to 155.9 kg (welder:
  315.2 to 321 kg), without changing moving-arm mass.
- State the SFU1605 race root explicitly at an assumed 13 mm; every other ball race must
  supply its own root diameter. Nuts share the same race specification.
- Bare/enclosed bearing-block cycles now measure **51.77/52.49 s**, still 78 plans.
  Actual X/Y/Z peak tracking errors are **0.077237/0.051505/0.053502 mm**, at the 10 ms
  controller period. Rapids remain **15.0955/25/22.8666 m/min**; X/Y/Z acceleration limits
  are **29.87/28.97/26.93 m/s²**. Revised rotating shafts/couplings and the separate spindle
  transmission change inertia, acceleration and cycle time. Door open/close remain
  **1.50/1.79 s**, seated-blank clamp **0.08 s**, clamp force **482.55 N**.
- Focused and complete gate results are recorded in `REVIEW_FIXES.md` as each group closes.

**MT6. Controllers, robots and signals.**
- Controller parts:
  - `CncController` cabinet: digital I/O ports, axis driver outputs, valve outputs;
  - `RobotController` cabinet.
  - Axis servos, valves and switches are wired to them (X8 driver/controller wiring).
- A controller owns the joints its actuators are wired to (MT-D1). Joints with no controller are an error; parts with no controller are environment.
- Scene v17 (v16 already carried the pneumatic/process scene fields):
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

Done. What was built and decided:
- CNC and robot cabinet parts own joints through motor-driver and valve-coil signal wires. Five mill field heads share their logical sensor IDs and each has its own cabinet input wire; spindle at-speed comes from its drive. Saved parts and port connections reconstruct those ownerships on every rebuild.
- One cell assembly builds two runtimes in one MuJoCo world: 15 disjoint moving joints and 148 component records, including an unowned floor marker. Both machine roots stay with their robots.
- Two cabinet-to-cabinet wires carry `cnc.ready` and `robot.clear` through the `CellSignals` bus. A changed output becomes visible to the other controller on the next 10 ms tick, including a digital input sensor frame.
- The bare mill remains at 51.86 s and the enclosed mill at 52.30 s for the bearing-block job after the cabinet and sensor heads were added. The enclosed mill still opens its door in 1.50 s, closes it in 1.79 s, and clamps a seated blank in 0.08 s at 482.55 N.
- The sequential `mt-suite-mt6-final.txt` gate passed every kit, MachineKit smoke, the app build and the full app project-source suite. Counts remain RobotKit 4999, MotionKit 67533, CadBridge 157, CncKit 317, CamKit 12311, ProjectKit 144 and ProcessKit 23 plus 19 weld-pass assertions.
- No MT6 baseline moved: screw/belt router 232.2/215.2 s with 0.05/1.76 mm worst deviation; CoreXY 649.6 mm/s and 71.549 m/s²; arm mission 5.8/12/17.2/23.5 s; mobile final goTo 61 s and obstacle round 65 s; welder seam runs 20.6/21.1 s and whole weldment 106.5 s. These are the MT5b values, including the earlier recorded main-merge and review-fix changes.

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
- The CNC's alarm code crosses the controller boundary with four discrete output bits beside `cnc.alarm`; the robot mission decodes those bits to its stable operator alarm instead of collapsing every PLC fault into one generic code. The analog alarm-code channel remains CNC-local diagnostics.
- CNC-signal waits use the PLC's 5 s request timeout plus one second for its alarm and one-tick wire image. Tray-presence waits retain their independent 5 s deadline.
- The physical spindle-stop interlock uses an assumed 5 rpm encoder deadband; the separate at-speed signal follows the commanded RPM with tolerance and hysteresis.
- Tests:
  - pure tests: every interlock refusal and timeout;
  - a blocked door request times out when its physical open switch never arrives;
  - a scripted-signal sim test with no robot: open door, place the blank by script, clamp, close, start, wait for complete, open, unclamp. Volume as in MT2.
- **Done. What was built and measured:**
  - The two controller domains exchange typed digital signal images; the CNC controller owns the mill axes, field sensors, valve coils and one-pass job. Requests are edge-triggered, time out to named alarms, and the CNC program parks at its geometry-derived G53 load pose before reporting ready.
  - MuJoCo completed the scripted door-open, loose-blank presence, unclamp, seated-blank clamp, door-close, one-pass cut, door-open and unclamp sequence in 59.09 simulated seconds, with no interlock alarm. The clamp held the real dynamic blank and used the physical 482.55 N vise cylinder; the door cylinder model reports 294.52 N.
  - CNC-kit tests cover all four actuator timeouts and de-energize the valve coils after a timeout; all interlock refusal paths and the door-open-during-cycle hold are checked. `cnckit/tests` passed with 317 assertions.
  - That app run recorded a mill baseline of 52.55/53.22 s bare/enclosed, X/Y/Z peaks 0.048472/0.051497/0.060011 mm, 78 plans and zero rapid/collision contacts. The later controlled `Part.fuseAll` comparison and the post-rebase recheck are recorded below.

**MT8. The parallel gripper, for real.**
- Current suction handling keeps a held workpiece kinematic: its mass contributes
  neither to the arm's dynamics nor to acceleration feedforward. MT8's physical
  contact grip must retain the dynamic workpiece and verify the resulting payload
  reaction and feedforward. This is an existing simulation limitation, not a
  validated payload-capacity result.
- `ParallelGripper` becomes an assembly:
  - a body;
  - two jaws on prismatic joints, the second mirror-coupled (ratio −1, X5 coupling);
  - an internal piston as a `Pneumatic` actuator on the first jaw. Force and stroke come from a catalogue of generic 2-jaw pneumatic grippers, sizes 25/32/40, marked assumed.
  - replaceable fingers with pad depth derived from the housing depth; an optional assumed 6 mm fingertip toe is a separate convex collision piece. The tending cell uses flat 50 mm pads: the longer support toe intersects the vise end stop during an open-jaw insertion, while the flat pads clear the vise and carry the measured dynamic blank by friction;
  - jaw `joint_switch` windows for open, closed-empty and gripped. The gripped window is derived from the full supported part-width band and excludes both end-switch bands, so different accepted widths can stop the jaws anywhere inside the mechanical stroke.
- `ArmGripperTool implements ArmTool`: plate, gripper, valve on the arm (like the welder's feeder seat), hoses, TCP between the finger pads.
- Robot-tool kind `gripper`: scene validation, the cadbridge binding and `EndEffectorControls` (`Gripper` control). The assumed rubber-pad friction coefficient (1.5) is saved with the jaw limits in SceneArtifact v19 and applied to the two pad hulls only. The app does not make a `SimulatedSuctionTool` for it, because the jaws are physical.
- `Grasp.find(part)` derives grasp frames from opposite planar faces (MT-D6), with the width and approach direction, ranked by finger fit. Pick and place targets carry a full pose.
- `HandlingPlanRunner` gets oriented approach and retract along the grasp frame's approach axis, and stays compatible with suction.
- With an explicit grasp orientation, the runner stops at the raised retreat pose so the next handling operation starts there and carries the part without folding the loaded arm through its generic home posture. Unoriented suction handling retains its home return.
- The grasp is judged from the jaw switch: the gripped band means OK, closed-empty means missed.
- The logical gripper command keeps its value through a controlled stop or aborted motion so the held part remains gripped. A fault or emergency stop sets the command safe; double-solenoid valve state remains latched when its coils de-energise.
- Tests on MuJoCo:
  - pick, carry and place a blank 50 times on a table;
  - slip relative to the fingers stays under 0.5 mm at the arm's planned accelerations;
  - a missing blank reports a miss;
  - the existing suction mission still passes.

Done. What was built and measured:
- The physical gripper assembly has a routed double-solenoid valve, a cylinder-driven jaw and a
  mirrored jaw. Saved tool limits, finger reach and the assumed rubber friction stay together in
  SceneArtifact v19. Open, gripped and closed-empty switches use derived jaw-travel windows.
- The housing CAD solid and collision hull leave derived reliefs for the full jaw-carriage sweep;
  the reach study exposed that a solid housing box falsely collided with the moving jaws.
- Boolean inputs now receive semantic `housing`, `left-jaw-relief` and `right-jaw-relief` names
  before subtraction, so split faces retain strong topological names instead of weak input ordinals.
- The tending cell uses 50 mm flat fingers derived from the 50 mm housing depth. MuJoCo completed
  50 physical pick/carry/place round trips with a free dynamic blank: **0.373 mm** peak slip from
  the TCP, **0.208 mm** pickup seating, and **51.99 N m** peak arm-joint effort. The empty grasp
  reports a miss. Contact transmits payload reaction to the arm; inverse-dynamics feedforward still
  describes the robot assembly only, so this is not a payload-capacity or payload-feedforward claim.
- The physical-tool clearance study replaces MT4's placeholder result: **Reach850 on a 550 mm riser**,
  15 cell poses, **28.472%** minimum joint margin and `|sin(q5)| ≈ 1`. Both the selected result and
  the required joint/wrist margins are asserted. The approach check found the 60 mm pad depth
  touched the vise end stop; pads now follow the housing depth and fit the load pose.
- The existing suction arm mission remains at **5.8 / 12 / 17.2 / 23.5 s**.

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
  - nothing reports `rapidContacts` or `collisions`;
  - after cycle complete, the robot waits for `cnc.ready` (including physical spindle standstill) before requesting the door; the door stays closed while the spindle coasts.

**Done. What was built and decided:**
- The tending cell has six free dynamic infeed blanks and six pocketed outfeed positions, mounted on the cell table. Each tray slot has its own wired presence sensor. The vise starts empty; machining uses the blank selected by the vise's volume-presence sensor.
- The CNC simulation updates the selected free body's pose, stock contour and mass through cutting, and excludes only that blank from cutter collision. The bearing-block cut removed **3,843.89 mm³**; the finished body is **0.184021 kg** against a **0.183962 kg** CAD target (0.033% error), with no rapid contacts or collisions.
- At the mill's real accelerations, the clamped blank slipped **0.00265 mm**. After the final CNC motion, `cnc.ready` stayed false until the unpowered spindle coasted to the 5 rpm standstill threshold; this took **0.67 s**, while the door remained closed. The scripted load/cut/unload sequence took **59.71 s**.
- The reach study now checks the actual pocket and table collision geometry used by the assembled cell. The cell uses its checked outward-facing park pose while the mill runs; the loaded reference blank remains only in the reach and work-datum geometry.

**MT10. The tending cell and its mission.**
- `TendingControllers` assembles the empty-vise `EnclosedBenchMill(false)` under `mill/`, the selected `CobotArm` with `DualArmGripperTool` on the shared `TendingCellRiser`, six free infeed blanks, pocketed trays and their table, both controller cabinets, air service and wiring. `TendingCell` saves the generated mission with that cell.
- `machine-tending/materia.project.json`, plus a Start-page entry.
- SceneArtifact v27 stores signal, wait, via-motion, timeout, metric handling approach, fault-retry command references, per-step handling-tool selection and a tool-roll field separate from part heading; CNC jobs and missions may coexist only under distinct controller owners. Main's current artifact format is used directly by the scene producer and fixtures.
- Mission steps:
  - `signal{name, value}`;
  - `waitFor{name, value, timeout}`;
  - `moveVia{poses}`, joint- or pose-space via-points with a simulation-time deadline;
  - oriented `pick`/`place` on `{occurrence, connector}` or a grasp, with a simulation-time deadline.
- `TendingMission.generate(cell)` resolves the cell's configured `TendingCyclePlan` into steps, unrolled like `WeldingMission.generate` (MT12 moves slot selection into that shared plan):
  1. pick from infeed;
  2. request door open, wait for door open;
  3. via-poses in;
  4. place in the vise, request clamp, wait for clamped and part present, release;
  5. via-poses out, signal clear;
  6. request door close and cycle start, wait for cycle complete;
  7. wait for CNC ready after physical spindle standstill, request door open and go in, request unclamp, wait for unclamped, then grip;
  8. come out and place in the outfeed slot.
- The grasp approach distance is saved on each pick/place step and comes from the tray-seat height plus two collision margins. The selected tool roll is saved independently from part heading and compensated between grasp and placement. The same approach value drives reach checking and the physical handling planner; the former generic 120 mm handling default does not describe this cell.
- On unload, the mission requests and confirms unclamp while the blank is still supported on the vise parallels, then picks and retracts it. The handling skill includes its retract, so waiting for unclamp first avoids pulling the part against the clamp.
- Vise, doorway and tray targets come from their assembled geometry and the shared reach result. Exterior staging stays at the checked lift height. The transit height balances the stock collision hull's lower extent against the wrist-package clearance at the header; checked poses at the doorway plane and halfway through its inner stand-off split the crossing into short, exact-stop segments. The reach study checks waypoint configurations; MotionKit's `CollisionPlannerSpace` conservatively certifies each generated joint-space edge against the CollisionKit description before a move is admitted. With the door closed, the mission asserts `robot.clear` at the exterior staging pose; it approaches the doorway only after the door-open switch arrives, then drops `robot.clear` before entering.
- Acceptance (`ProjectSourceTests.checkMachineTending`, MuJoCo):
  - 4 parts unattended, 4 finished parts in the outfeed;
  - all four selected outfeed pockets report their physical presence switches;
  - every removed volume within 2 %;
  - 0 collisions and 0 interlock alarms;
  - cycle time and spindle use reported;
  - the welder, router and arm tests are unchanged.

**Done. What was built and decided:**
- The closing MT11 mission handles one blank at a time, completing its outfeed placement before the next infeed pick. Earlier code and assertions prematurely required MT12's two-part exchange and ≥75% throughput; those are deferred until after landing. The existing two-gripper hardware and both checked TCP pose sets remain available for MT12. The fault replay exposed an inactive finished-part collision during that premature exchange; MT12 must certify the two-payload contact and transit scenarios before enabling pre-staging.
- The saved cell has two independent controller owners and a 176-step sequential four-blank mission. The earlier 137-step / **74.31 s** startup checkpoint stopped before clamp confirmation; it is historical and does not certify the closing mission.
- The earlier assembled-tool checkpoint chose **Reach1300 / 550 mm**, **92 poses**, **391 mm** base stand-off and **12.181859%** minimum joint margin. The closing study adds separate load poses, the active jaw sweep and closed-door tray checks; its certified result is now Reach1300 / 550 mm / 96 poses / 541 mm stand-off, with 10.460036% joint margin and 0.878105 wrist sine (MT12/MT12b).
- Main's CL7 planner owns `moveVia` collision admission. Its description uses the robot adapter's structural pair rules, and phase-synchronized rest-to-rest timing stays on the certified joint-space edges. The older process-clearance post-check is removed from this joint-move path; process handling and welding retain main's process-clearance APIs.
- Independent cell hulls have their own fixed collision bodies outside the arm articulation, so reference-overlap allowances cannot hide an obstacle intersecting the initial pose. Confirmed payloads remain on their carrier links. Each move rebuilds this description from live door, inventory and payload poses, while reusing its controller and derived limits. The independent-obstacle and carried-object rejection checks pass with their original assertions.
- The full physical production and fault-recovery test passes; sequential integration coverage is recorded under MT11. MT12's pre-staging, two-payload exchange and throughput acceptance, and MT13's vision work stay after review and landing.

**MT11. Faults and recovery.**
- Injected faults, each ending in the safe state with a named mission failure and an alarm code:
  - empty infeed slot;
  - missed grasp;
  - part dropped in transit (jaw switch leaves the gripped band);
  - clamp with no part (presence false);
  - door jam (switch timeout);
  - cycle alarm (a premature cycle-start interlock; the CNC-kit also verifies the tool-change hold path);
  - e-stop.
- After `robot.fault_reset` and the cause removed, the mission resumes at the step that failed. Parts already finished stay finished.
- Tests: one scenario per fault.

Recovery policy: mission failures carry stable application alarm codes. The CNC PLC's discrete alarm-code outputs preserve the typed cause across the one-tick controller boundary. A failed wait may point to the asserted output request that owns it; retry repeats that request and holds `robot.fault_reset` high until one full cell-signal image has sampled it, then lowers it on the next controller feed. Recovery waits for the robot's controlled stop to reach `READY` before writing outputs and keeps the completed-step count and physical object state. Emergency stop uses RobotKit's safety-stop and explicit reset path before mission retry. A carried workpiece is monitored through via-motion and a lost jaw signal latches the mission as `PartDropped`.
- `DoorJam` is reserved for a physical door timeout or a door switch lost during the cut; refused interlocks remain `CncAlarm`. `ClampWithoutPart` requires the physical presence input to be false, while a clamp/unclamp stroke timeout with stock present is `ViseFailure`.

**Done. What was built and decided:**
- Stable mission alarm codes distinguish emergency stop, empty tray, missed grasp, dropped part, physical door jam, missing-part clamp, vise stroke failure and refused CNC interlocks. The missed-grasp fixture removes the blank only after the real close coil output fires, then checks failed grasp confirmation; it no longer moves the target before planning or forces a 0.25 s generic timeout. Genuine planning failures remain `MotionFailure`. Physical-gripper picks first request opening and wait their existing 0.4 s valve/stroke settling dwell; step-specific approach distances preserve this tool policy. The retry checks actual jaw-open feedback before the new grasp. Physical-gripper handling supplies a typed IK tolerance of 0.1 mm / 0.1 mrad; the generic suction defaults remain unchanged. The old 2 mm / 5 mrad planning tolerance was looser than the machining stock-axis requirement. Tightening IK alone did not resolve the physical seating error; the machining check correctly continued to refuse the tilted blank. The machining alignment check remains strict and reports its observed stock-local axis. Startup also requests and confirms vise unclamping while the arm remains outside the enclosure; the initial double-solenoid latch is closed, so skipping this handshake perched the first blank on the jaw tops. Later unloads share the same retryable unclamp helper. Recovery reasserts confirmed carried-gripper requests once the controller is ready, while safety-cleared logical outputs do not move a double-solenoid valve on their own.
- Retry preserves the completed-step count and physical inventory, waits for the controlled stop, and repeats the owning request only after a full controller image has sampled fault reset.
- Recovery confirms carried grippers before resuming and refreshes their measured payload transforms. In the dropped-part fixture the operator supports the reseated blank until both fingers contact it and jaw velocity settles, then removes support for a 50 ms grasp confirmation. The earlier fixture accepted the travel switch before closure; the blank fell nearly 8 mm before the next carry and eventually slipped out. No kinematic hold is used during the resumed carry.
- Placement resolves the station's live CAD frame across both controller owners, rather than its authored zero pose. CNC parking moves the vise before loading; a stock-presence assertion now checks the physical sensing volume before the missing-part injection. The shared part resolver owns frame and geometry-centre conversion for robot parts and free bodies. Physical held-object reporting includes both-jaw contacts for parallel grippers, not only suction attachments.
- The vise exposes separate `datum`, `seat` and `load` connectors. G54 uses the stock corner; `seat` locates the centred stock origin after clamping and installs the mill's initial stock. `load` releases the blank above the parallels and in the centre of the open jaw gap, both offsets derived from the preset clearance (1.5 mm). The reach study covers loading and seated unloading separately (96 poses). A free blank in the isolated contact fixture stays flat under the actual cylinder, whereas direct robot insertion against the fixed jaw left the stock tilted. Placement refreshes the currently observed grasp transform once before planning, then freezes it for that handling step; a contact grip can slip during transit. Using the machining corner for placement had shifted the blank 30 mm in each horizontal direction.
- Approach and contact IK share a tool roll, checked independently against the exported runtime model for all eight placements. The former endpoint-only study could choose headings 90 degrees apart. The enclosed vise mount moves 20 mm along X, derived from head width, vise width, travel and clearance; loading parks the complete vise beside the retracted head. The route crosses the doorway centre, then approaches the off-centre vise inside. Main's CL7 conservative edge checker and RRT planner certify the selected doorway transitions with the payload hull; the final route is checked again after all IK pairs are selected. MotionKit's shared `ManipulatorPlannerSpace` adapter fixes inactive DOFs for both this study and the live mission, replacing the app-local duplicate. A blocked direct joint segment may use a conservatively checked detour. Transport station order is shared by mission export and final reach-edge checks; pose-table adjacency no longer creates a fictitious loading-contact-to-unloading edge. Tray and outside-staging poses are checked against the closed door as well as its open parked leaf, derived from the door joint for every attached hull. The first physical run had completed one cut but refused the next tray pose because the open-door-only study put the upper arm 63.6 mm inside the closed window. The insertion lift is 50 mm: jaw clearance, two 5 mm margins and the stock hull's 20 mm axial span. The side pads have no axial locating stop, so the span reserves clearance for a valid grasp at any depth on the side face, so a measured grasp offset does not consume the collision margin. The preceding open-door study used a Reach1300/550 mm mounting shifted -238 mm in X and -300 mm in Y, for 691 mm front stand-off; the MT11 checkpoint certified 391 mm front stand-off; MT12/MT12b supersede it with 541 mm, recorded below. The earlier **16.13 s** startup checkpoint reached the pre-clamp command only; it did not prove seating. The strengthened gate waits for real clamp confirmation after the startup unclamp handshake. The bare mill mount is unchanged; the enclosed cycle must be remeasured. The closing study now uses main `CollisionDescription` / `ModelBodies` / `CollisionPlannerSpace` for pose checks as well as transit, with structured violations instead of app-local contact-window partitions. The selected pads may touch their selected blank; other contacts retain named margins. It includes the active gripper's geometry swept from open to nominal grip through the actual cylinder joint and mirrored coupling, while the idle gripper remains open for this sequential mission. The earlier frozen-open study missed a full-width end-stop/finger overlap and a pad/parallel edge contact. The vise now uses an assumed 6 mm square corner locator and parallels inset by the preset clearance at both end faces, preserving the datum and support height. The combined cycle and tracking effects are recorded in the closing measurements below.
- CAD hulls remain available independently of the editor collision flag. Machining stock uses a separate simulation contact policy; the former resolver returned no hulls and clearance then treated a held blank as a root-fixed bounding box. The physical replay exposed a false 35 mm blank-to-upper-arm collision. The stock now follows its gripper carrier using its actual convex geometry, and the route regression checks that relationship immediately after a confirmed pick.
- Haxeon `9366f24e` fixes proven-null instance, static and record field reads: they produce typed nulls in their declared nullable storage instead of a cast to `TNull`/IR `Void`. TestMain, a real HashLink fixture and official-Haxe parity pass; the pin follows the requested `ccfd1dd6`/main merge. The reduced class-field guard originally failed with `Unsupported cast from Bytes to Void`; no application workaround remains.
- Haxeon `835cf2c0` preserves final-field refinement stability across continuing branch joins. A valid constructor guard followed by an unrelated method call had rejected `vise.seat` as nullable. Guard, if, switch and loop regressions pass in TestMain and real HashLink/official-Haxe parity; mutable-field invalidation remains checked. The compiler is fixed directly, with no CAD-code workaround.
- RobotKit's large-cloud `ConvexSolid` fallback now constructs its box corners from the bounds. Those corners need not occur in a round cloud; the former extractor could throw when constructing a swept cylinder. A 128-point round-cloud regression checks all eight generated box corners.
- Reset and scene retirement drain each joint motion's own planning workers before disposing its collision world. Cancellation alone is asynchronous; a retained crash showed `ck_violation_sets` running concurrently with `ck_world_destroy`. Typed cached planner ownership replaces runtime casts, and a scoped-shutdown regression verifies that unrelated planning continues.
- The isolated pneumatic fixture derives its reversal and completion windows from cylinder stroke and flow. MT10's 600 mm opening requires a 610 mm stroke; the older fixed 1.6 s reversal stopped it before opening. Measured opening/closing is **1.99/2.37 s**, while the seated-blank clamp remains **0.08 s / 482.548632 N**. Nominal and resumed startup use the same bounded 120 s route budget. The former 16.13 s and 74.31 s checkpoints stopped before clamp confirmation; the closing test now requires actual seating and clamping.
- The dedicated app test injects every listed fault before continuing the sequential four-blank production mission. Integration checkpoint `81f5563f7` incorporates main through `0f574ed49`; the closing merge additionally incorporates `249ebcea2`, including preservation of declared CAD mass/inertia and powered mobile-welder charging. SceneArtifact v27 combines both schema changes without an old-format reader. The full physical closing run and rebuilt editor document refusal check pass; the final sequential integration coverage is recorded below.
- The physical recovery replay passes emergency stop, empty infeed and missed grasp at 12.31 s; dropped-part regrasp at 12.50 s; door jam and refused CNC interlock at 22.53 s; and missing-part clamp recovery at 50.65 s. Both grippers seat stock flat enough for the strict machining-axis check, and all four cuts take 53.70 s with 3855.14 mm³ removed. That replay reached step 171/176 before the test's fixed 800 s cutoff; it had no mission fault. The closing gate derives its overall deadline from the remaining authored step timeouts, retaining each runtime timeout, and requires four sensed outfeed parts and completion of all steps. It also checks CAD leftover plus gouge against 2% of nominal removal, independently of removed-volume and mass checks. The corrected replay passes all production acceptance checks: four sensed outfeed parts, one workpiece carried at a time, no CNC alarm, and all authored steps completed. First clamp to completed sequential mission is **772.94 s** (823.59 s including startup and fault injections). All four cuts have **11.50 mm³ leftover** and **2.73–2.76 mm³ gouge**. Encoder and CNC-panel utilization agree at 100% within the CNC cycle intervals; this is not whole-cell throughput, which MT12 will improve. The closing isolated vise contact test passes, and the nominal robot startup route physically seats and clamps its blank at **37.92 s**.

**MT12. Throughput.**
- Before enabling pre-staging, certify both carried-payload envelopes at insertion as well as transit, including the inactive gripper against the retracted head and seated workpiece. The premature prototype lost the finished gripper's part during the first exchange under main's collision model; MT11 now uses the sequential mission above.
- Pre-staging: while the mill cuts, the arm picks the next blank and waits outside the door.
- A dual gripper (`DualArmGripperTool`) mounts two independently valved physical grippers at 90° on one plate. Each has its own cylinder, channels, sensor and contact frame.
- The reach study derives the fixed TCP transform between the two grippers, solves the cell poses for each actual TCP, and checks both tool envelopes against the machine, doorway, fixed tray/table hulls and all six blanks in the initial infeed layout. The generated `moveVia` steps use the checked poses for the gripper that handles the next part. Each mission plan also checks live blank and carried-object poses as inventory changes.
- A carried blank stays in the live collision set and follows its confirmed gripper carrier in planning FK. Its named holder and gripper hulls, plus the four source-tray seat pads while lifting that blank, use explicit zero-margin contact interfaces: the hulls cannot penetrate, but do not need 5 mm separation from designed contact surfaces. Door, other tray members and the machine retain the 5 mm obstacle margin.
- Tray pitch is derived from the actual dual-tool CAD hulls at both grasp frames across the jaw sweep. Parts that can meet pocket-height hardware contribute their full projected radius, covering arbitrary tool roll. Neighbouring blank and sensor envelopes, the unchanged 5 mm collision margin and a 1 mm assumed fixture placement allowance determine X/Y pitch. The resulting tray layouts carry the pitch used to assemble the physical trays; there is no separate stock-plus-gap formula.
- While cutting, the free gripper collects the next blank and stages it outside. After `cnc.ready`, one door opening unloads the finished blank and loads the staged blank; the two physical grippers retain their separate parts during the exchange.
- The CNC dock panel measures spindle utilization from its encoder only during active CNC-cycle intervals, preserving the last completed measurement during idle time. The app acceptance check compares it with an independent encoder sample over the same signal window. The four-cycle acceptance records each interval and the overall mission time.
- Target: whole-cell spindle use ≥ 75 % for the bearing-block job, measured from first clamp to fourth sensed outfeed. Keep this distinct from the panel's within-CNC-cycle utilization, with both numbers recorded here.

Done. What was built and decided:

- `TendingCyclePlan` owns the typed production operations and derives their distinct transfer requests. Mission generation resolves those same operations into signals, handling steps and certified joint paths. The four-part program needs **23 certificates: eight inventory routes, eight tray clearance/approach routes, one empty-gripper startup route and six vise/exchange routes**. Typed transfer roles distinguish their payload contexts. Slot selection, tray dimensions and the initial stock reference follow that plan. The reach study retains all **96 pose checks**, both payloads during open-door exchange and the single carried payload during closed-door transport. Unused tool/station combinations and the obsolete adjacent-doorway guide chain no longer reject a program; private guide poses retain their endpoint checks.
- Pre-stage the next blank during cutting, return to the unloading gripper's staging posture, exchange the two independently retained parts in one door opening, and deliver the finish while the next cut runs. The final delivery adds no return. Structural checks track each gripper's inventory, CNC requests, prefetch and outfeed ordering. Every executed joint edge still passes main's live conservative clearance check against the current machine and loose-object poses.
- Derive insertion, tray and doorway geometry from the actual mounted dual tool. Tray sizing reuses the mounted arm scene and physical hulls, projects each part across both jaw endpoints at both TCPs, and includes any part whose sweep reaches pocket-height hardware. A radial bound covers tool roll; neighbour blanks and sensor heads, the unchanged **5 mm** margin and a **1 mm assumed placement allowance** determine pitch. Physical trays consume that pitch directly. Adding 2 mm to the old stock-plus-gap formula was insufficient: it moved the idle tool toward a neighbouring pocket and left a real **3 mm** gap. Numerical pitch is pending the closing replay.
- Check both carried-stock envelopes against the retracted head, independently seated blank, clamped jaw/rod and both door states. Closed-door transport replaces the open door leaves with their closed poses. Loaded-reference reach checks retract the vise jaw before loading and unloading; its clamped default otherwise produced a real **1.5 mm** collision. A loose payload cannot inherit structural reference-overlap permissions. Its actual holder contact is separately checked before a rigid grasp is used in transit.
- CollisionKit's explicit `Touch` rule permits intended zero-margin contact and rejects penetration beyond **1 nm** numerical precision. Ordinary `Check` and positive margins retain their behavior. A shared continuous-check fix cancels common rigid-body motion for shapes on the same body: the regression's **10 mm** gap clears **5 mm** even with **100 mm** common motion, instead of falsely requiring **205 mm**. Geometric inflation, penetration and stricter margins still count; independent bodies retain both motion bounds. The regression fails before the fix. All **3/3 native CollisionKit tests** and canonical HXI checks on four targets pass after it.
- MotionKit shortens RRT routes only through conservatively certified replacement edges, with a bounded final visibility pass. Execution and approach ranking use one `JointPathTiming`: synchronized, jerk-limited rest-to-rest edges at the actual mechanical rates. Independent joint minimum times provide a valid lower bound for detours. Approach candidates keep their insertion, contact and optional staging posture together; ranking first certifies the complete Cartesian insertion and seating pair. A ranked approach must own a certified entry path. Establish the first feasible incumbent with the full planner budget; only subsequent improvement attempts use the smaller optimization budget. A prepared staging posture is an alternative to the original checked posture. Retain the selected entry path for export and require its exact endpoints to match the final poses, instead of discarding the certificate and independently replanning it. Final exported routes retain the full **5000-iteration** search budget.
- Staging candidates must clear both door states and both payloads, and connect to the exterior transport graph with either single payload. Export the complete paths actually needed by the production sequence, including inter-tool exchange, and reverse certified tray paths to prepare the next unloading posture. This fixes the intermediate mission's mismatch between the posture ranked by the study and the posture used at runtime.
- Shared `PoseSampling` includes angular derivatives as well as translation and curvature. A short **3 mm / 0.23 rad** seating correction previously selected too few intervals for the candidate ladder. Handling uses **0.1 rad** angular spacing, half its **0.2 rad** jump budget; this controls selection density, not clearance certification. Compiler workers and planner replacements preserve that setting. Focused visibility, timing, lower-bound, blocked-prefix and angular-sampling checks pass **557 assertions**. The unequal signed-displacement / unequal-limit diagonal regression independently checks straight-edge phase synchronization and velocity, acceleration and jerk ceilings.
- The rebuilt app and geometry-free production-plan fixture pass. Independent fixtures check the eight inventory transitions, eight internal tray routes, empty-gripper startup, four cuts, four unloads/deliveries, three exchanges, reordered slots and invalid-slot refusals. The first geometry-sizing replay refused a standalone end effector without its mount; mounted-scene reuse fixes the assembly input without a placeholder mount or validation exemption.

Measured diagnostic runs (not the closing MT12 acceptance; these used mission completion as the window end):

| Revision / check | Result | What it established |
| --- | --- | --- |
| First physical four-part pipeline | **518.02 s**, **41.7%** whole-cell spindle use; four **53.70 s** cuts | All fault recoveries, two retained workpieces and four sensed outfeeds pass. Shape/mass checks pass: **10.9998–14.25 mm³** leftover and **2.7365–3.9279 mm³** gouge. Throughput is below 75%. MT11's sequential window was **772.94 s**. |
| Focused two-cut exchange | **219.16 s**, **49.15%** whole-cell spindle use | Both physical grippers retain their separate parts. Entry takes **15.07 s**, retreats **34.09/48.59 s**, exchange **1.87 s**, placement **2.12 s**. |
| Conservative visibility shortening | First retreat **39 → 7** waypoints and **34.09 → 12.99 s**; entry **12 → 6** waypoints | Avoidable rest-to-rest stops materially affect cycle time. This is diagnostic evidence, not four-part acceptance. |
| Insertion-compatible staging before angular sampling | Place fails at **3.0443 mm** along seating | Static approach/contact feasibility does not prove the complete runtime insertion. Earlier fault recoveries pass. |
| Angular-sampling replay | Missing-part clamp recovery **42.02 s**; first retreat **17.15 s** | Placement and all fault recoveries pass, but closed-door prefetch has no path at step 27. Endpoint checks alone do not prove transport connectivity. |
| Cleaned route with the interim 102 mm pitch | Source generation refuses **3 mm** idle-finger clearance at infeed-1-0 | Derive tray spacing from the actual tool projection; do not relax the margin. No physical run occurred. |

Later diagnostics and root fixes:

- Production-derived transfer requests replace pose-table adjacency and obsolete guide chains. An early 83-minute export reached startup, but the grouped tray tasks now retain clearance, approach and contact as one roll/IK branch and certify incoming plus internal routes together. Empty-gripper startup uses the actual saved park; reversing the same certificates supplies the return paths.
- Fixture placement reserves the complete prismatic door sweep. The actual **326.411 mm** table depth left a **5.206 mm** shortfall; shifting the complete fixture, slot frames and table together restores the unchanged **5 mm** margin. A native convex-hull regression checks it independently.
- RRT connect used to check an unusable suffix after the first blocked edge. Eight-edge batches retain the clear prefix and stop at the first obstruction. An independent thin-barrier regression fell from **604** edge checks to fewer than **60**, without changing steering or collision arithmetic.
- Four-part grouped-tray replay completed at **310.49 s / 69.5546%**. Exchange-cost ranking initially worsened this to **345.59 s / 62.4902%** because IK still used the unrelated staging branch. Seeding from the opposite tool's resolved approach and roll reduces both exchanges to **1.86 s**. Costs use exchange frequencies derived from the production operations.
- Exchange checks use two held parts and the empty open vise. Reversible entry/retreat checks certify each physical payload/jaw state separately, replacing open jaw/rod hulls with clamped copies rather than duplicating them. Final B unload retreat also certifies the active held finish. No collision margin is relaxed.
- Independent tray/vise rolls rotated a contact-held part: the third cut correctly refused **1207.75 mm³ leftover / 1194.45 mm³ gouge**. Inventory tasks now inherit their resolved vise grasp roll. The corrected replay passed all four cuts at **323.11 s / 66.8379%**, with **11.50 mm³ leftover / 2.72450–2.72763 mm³ gouge**.
- Physical-state correction reached **320.37 s / 67.4096%**. Timing-aware bend refinement reached **318.43 s / 67.8202%**. Four bounded refinement passes accept only faster synchronized edge timing and native-certified replacement edges; endpoints and the original certificate remain intact. The independent planar wall/gap fixture proves faster timing, safe edges, unchanged endpoints, rejection of invalid inputs and the disabled mode.
- The remaining routes crossed wrist branches: vise **+π/2**, prescribed holding pose **−π/2**. Holding has no placement-orientation requirement. Candidate postures now come from the certified retreat and FK, with every moving arm/tool/payload hull outside the mill front by **5 mm**, full cell/door-sweep clearance and certified connection to exterior transport with either single payload. The grid selects candidates only; executed edges remain continuously certified.
- The first orientation-free holding replay passed all four cuts, two-part retention, recoveries and four sensed outfeeds at **291.31 s / 74.1341%**, still below acceptance. Entry/retreat fell to roughly **2.5–3.5 s**. A timing lower bound based on the original endpoint cannot prune a search that moves that endpoint; that pruning is now disabled for holding-posture selection. The corrected search reproduces the same physical result; it fixes invalid pruning, rather than claiming a timing gain.
- Final delivery now overlaps the last door closure. Typed request and confirmation operations retain one owning request; all cuts still wait for door-closed feedback and the finite mission confirms closure before finishing. Outside transport is certified against the full convex prismatic door sweep, not only its endpoints.
- Whole-cell measurement now freezes at the first tick with four sensed outfeed seats, four completed stocks and no physically held object. Mission completion, final closure, part shape/mass and final sensor occupancy remain separate required assertions. The former window included the last placement's clearance retreat, so its historical percentages above are diagnostic mission-completion measurements. The final-overlap replay passes: **287.74 s / 75.1%** whole-cell spindle use, four **53.70 s** cuts, **11.50 mm³ leftover / 2.72411–2.72934 mm³ gouge**, all fault recoveries, two retained workpieces and four sensed outfeeds. CNC-panel and independent within-cut encoder utilization both remain **100%**. Final closure and all **144** declared steps complete. Door-open handshake is **2.05 s**, normal close handshake **2.43 s**; both exchange directions are **1.86 s**. Vise entries are **3.52/2.53 s**, load retreats **3.39/3.10 s**, final held-finish retreat **2.56 s** and final outfeed transport **3.13 s**. These are physical mission phase times, including controller settling.

Closing validation:

- The standalone reach check passes **Reach1300 / 550 mm riser / 96 poses**, **541 mm base stand-off**, **10.460036%** minimum requested-pose joint margin and **0.878105** wrist sine. The MT11 values were 391 mm / 11.763934% / 1.0. Wider CAD-derived trays and actual payload/door-sweep route certification select the farther mounting; orientation-free holding changes the wrist metric. Tray pitch is **196.411376 × 186.411376 mm**. Bare mill geometry is unchanged: X/Y/Z rapid limits remain **15.095544 / 25 / 21.521827 m/min**.
- MotionKit passes **1,225,280 assertions** (**+571** from the published baseline), including the exact wall/gap refinement, angular sampling and blocked-prefix regressions. Toolpath **3020**, CadBridge **178** and CncKit **317** pass unchanged.
- MachineKit smoke passes, including the shipped 144-step mission, standalone reach, controller ownership/wiring, saved parking, mill/enclosure, brake bindings and all cobot classes. Cobot assembled masses remain **14.725839 / 21.120969 / 37.394305 / 37.731414 kg**. All five HXI checks and CadKit pass. Native CollisionKit **3/3**, native MotionKit **16/16**, Haxe CollisionKit **364**, clearance **23**, cell collision **30**, collision MuJoCo **59** and CAD-bridge MuJoCo pass.
- The first RobotKit gate run exposes a timing race in `RuntimeEndpointTests`: its live owner may publish between the running snapshot and the thread join in `stop()`. Subsequent read comparisons now use a fresh immutable post-stop snapshot, captured before installing the sensor poller. Exact coordinates/source timestamps, owned arrays and no-sensor-poll checks remain. No runtime timing or production behavior changes. The focused complete RobotKit rerun passes **5461 world assertions** (`b71c18787`).
- The complete app project-source suite and rebuilt editor `app/tests` pass, including Worker document/gallery checks. Published runtime baselines remain: screw/belt router **232.2/215.2 s**, **128 plans**, deviations **0.05/1.75 mm**; CoreXY **649.6 mm/s / 71.549 m/s²**; arm mission **5.8/12/17.2/23.4 s**; mobile mission/obstacle **57.3/61 s**; welder normal/restart/post **21.5/24.9/27.7 s**, gantry/track weldments **159.8/99.9 s**. Bare/enclosed mill remains **52.73/53.70 s**, bare removal **3860.657759 mm³**, leftover/gouge **7.110556/0.657759 mm³**, **78 plans / 55 flagged**. At **10 ms**, X/Y/Z tracking stays **0.034544383/0.051463902/0.059015342 mm**, physics interval **0.25 ms**. Z-brake e-stop sag/drift stays **0.000375764/0.000019906 mm**. Standalone door opening/closing stays **1.99/2.37 s**, seated clamp **0.08 s / 482.548632 N**.
- The nominal route test initially checked alignment at the first clamped-switch observation, during the contact transient (**0.008311 rad** tilt). It now follows the authored arm retreat and confirmed door closure to the first machining request, then checks the unchanged machining alignment bound before cutting, with an empty physical gripper. No artificial settling delay, motion change or relaxed tolerance is introduced. The rebuilt focused route passes at **24.23 s** (`f4a2030a6`).
- Closing coverage is the complete sequential `mt12-close-20261011` gate plus the two focused repairs above. StockKit **100673 + 17494 reference rays**, CamKit **12311**, ProjectKit **192 + 18 contact-wire assertions**, ProcessKit **23 + 19 weld-path assertions**, HumanKit and AnimKit also pass. The original gate summary retains `failures=1` (a failure flag, not a failure count); the RobotKit and route failures are not erased or presented as an uninterrupted pass. Focused logs are `mt12-robotkit-post-stop-recheck.log` and `mt12-route-ready-recheck.log`; physical four-part acceptance is `mt12-final-overlap-production.log`, all in the shared scratch directory. No new Haxeon issue or compiler change was needed; the pin remains `8d675fdef7498c43cf3a11193b74656bd0d890e5`.

MT12 implements the four-part physical cycle and dual-part retention, with closing coverage from the sequential gate and focused repairs above. Release depends on completing MT12b and its full gate. The measured **287.74 s** cycle contains **214.8 s** of cuts: **74.65% cutting utilisation**. The recorded **75.1% spindle-on utilisation** also counts run-up/coast-down; it does not establish a cutting-utilisation target with margin. MT14 closes that measurement/acceptance gap and schedules the physical run in the maintained gate. MT13 has not started; tray sensors remain until its calibrated camera pipeline replaces them.

**MT12b. One cell collision description.**
- **`CellCollisionSnapshot` → `CollisionSceneBuilder`.** CadBridge captures typed CAD and live-state observations; RobotKit builds one `CollisionDescription` through main's `CadCollision` and `RobotCollision` adapters. The result contains geometry, stable ids and pair rules, plus a minimal ownership map.
  - Keep capture beside the CAD bridge, the geometry builder in RobotKit, and handling/welding policy in ProcessKit. `TendingReach`, `MissionPlayer`, standalone fixtures and `MobileWeldStations` consume that same builder.
  - Door, vise and other robot poses are inputs, never baked in.
- **Pair rules are data.** Vise jaws and gripper pads use the work-contact allowance; welder interfaces keep their own margin; other pairs use the general 5 mm margin. Use CollisionKit's pair-rule and group-margin APIs rather than call-site branches or a new distance engine.
- **Migration, clean break:**
  - `TendingReach` and `MissionPlayer` use the shared description for static checks and conservative planning; `MobileWeldStations` consumes the same description through the RobotKit clearance adapter;
  - the per-site body gathering and contact lists are deleted;
  - a single construction site remains.
- **Collision corpus.** Data fixtures with geometry, poses, rules and expected verdicts/distances run against CollisionKit's native `CollisionWorld` and CAD/robot adapters:
  - the thin door edge (clear at both ends and the midpoint, unsafe inside);
  - gripper B against the neighbouring tray blank;
  - the vise contact allowance;
  - tray neighbours;
  - the track welder's 4.0 mm case;
  - a cross-robot pair (arm against mill door).

  Main's coal backend is the checked implementation; do not reintroduce a parallel backend for these fixtures.
- **Acceptance:**
  - reach study result unchanged (class, riser, margin);
  - every mission clearance verdict in the tending, cobot and welder project-source cases unchanged, or explained;
  - the track welder's case resolved in the shared description, not by loosening its margin;
  - the corpus passes;
  - the full sequential gate passes.

Done. What was built and decided:
- One production path now runs snapshot → shared builder → `CollisionDescription`. Reach, app skills, welding and standalone fixtures use main's CAD/mechanism adapters. The alternate hull constructors, duplicate adjacency, geometry gathering and description-to-hull round trips are deleted. Capture stays in CadBridge, construction in RobotKit, and handling/welding policy in ProcessKit.
- CollisionKit owns held-part transfer and explicit contact exemptions. Other grippers and tool members remain checked; detached stock keeps its current world pose. `ModelBodies` is the sole follower map. Physical member identity survives compound hulls and rebasing, including descendant robot roots and multi-DOF state.
- Margins use bounded classes (air, installed, tool, work, held), cached tables and sparse pair overrides. Contact windows override and restore object rules. Native API **v2** carries query context per call. SceneArtifact makes a clean break at **v29**; producers and fixtures are migrated, with no old-format reader. Tool/pad and weld-member identity comes from typed records, not names.
- Margin changes are explicit: unrelated welder obstacles use **5 mm** in planner and runtime, replacing the former **3 mm** runtime air cap. Installed interfaces retain **3 mm**. Named welding work uses **3 mm air / 0.5 mm contact**, replacing the generic planner's former **5 mm** tool/work margin. Application checks pass with these aligned policies; no corpus allowance was widened. The shipped track-welder interface measures **0.004000000000000004 m** and its complete ready pose clears.
- Conservative motion checks use a typed timed-trajectory interface and preserve inactive DOFs. The probe wire is a capsule in the same description, with **rounded ends assumed**, declared overall stickout and diameter, and no separate Haxe GJK. Physical replay requires actual touch and an independent signed native wire gap: **−0.0100011921 mm**, with **1.2803716526e-15 m** sensor/FK error.
- Application export now distinguishes dynamic free bodies from externally owned static members. Joined static work stays environment; joined dynamic parts remain invalid. The probe fixture derives its contact from typed CAD faces; its old posture is only an IK seed. The tending manifest declares its ProcessKit dependency. Standalone fixtures use the shared builder and observed reference state; smoke loaders accept the existing app native-library layout without recreating the obsolete external build directory.
- Reach remains **Reach1300 / 550 mm riser / 96 poses / 541 mm stand-off**, **10.4600359266%** minimum joint margin, **0.8781054926** wrist sine and **196.4113759501 × 186.4113759501 mm** tray pitch. Production remains **287.74 s**, four outfeeds and 144 steps: **74.65% cutting utilisation / 75.1% spindle-on utilisation**. Startup seating is **24.23 s** and the physical CNC-interface check is **61.61 s**. Cutting utilisation with margin remains an MT14 acceptance item.
- Mill baselines remain **52.73 / 53.70 s** bare/enclosed, with **0.034544 / 0.051464 / 0.059015 mm** X/Y/Z tracking at the **10 ms** controller tick. Door travel remains **1.99 / 2.37 s**, seated clamp **0.08 s**, and declared actuator force **482.548632 N**; this is not a measured jaw–blank contact force. Routers retain **232.2 / 215.2 s**, 128 plans and **0.05 / 1.75 mm** worst deviation. No new timing baseline change was required by MT12b.
- The complete sequential gate passes, including native collision **3/3**, native motion **16/16**, CollisionKit **380**, clearance **23**, cell **97 (67 corpus)**, contact-search **143**, MuJoCo collision **59**, physical probe, tending-vise, tending-route, interface, four-part production, default app and full editor/Worker document checks. RobotKit **5461**, MotionKit **1225280**, CadBridge **178**, CncKit **317**, CamKit **12311**, ProjectKit **192 + 18** and ProcessKit **23 + 19** pass. Six affected kits were rerun serially after the final code edits and all passed.
- Evidence is retained under `/home/joao/dev/materia-cache/claude-scratch/`: `mt-suite-mt12b-close3-20261012.txt`, `mt12b-final-affected.txt` and `mt12b-machinekit-coverage.txt`. This was a repaired/resumed gate, not an uninterrupted run. Earlier fixture/dependency/loader failures were fixed. Host `earlyoom` terminated MachineKit during controller checks (SIGTERM, exit 15); existing independent shards completed all **66 groups**, with successful exits and coverage verified. The controller fixture builds one cell without removing assertions. Heavy stages ran serially through `heavy-run`; OCCT was not built.
- Haxeon commit **`1d96bab5`** fixes the root CFG defect: block-expression lowering discarded enclosing loop context for `break`/`continue`. Compiler unit checks and eight HashLink fixtures pass; this commit pins the fix. No Materia compiler workaround was added.
- MT13 has not started: tray sensors remain. All section B–D review follow-ups remain tracked as MT14 below. Constructor-time reach search, production force/seat evidence and safety hardening are not claimed complete. Review precedes any main update.

CL9 reconciliation (main `b3405449e`; merge checkpoint `a8afd9f13`, closed below):
- Main supplies the description-only `CollisionClearance` API, `CollisionBounds`, description `append`/`copy`, the `CollisionShape` vocabulary, KinematicsKit motion bounds/envelopes, inflated-convex narrowphase repair and the measured **512-pair** broadphase threshold. MT12b supplies typed source/member/tool identity, independent live `ModelBodies` carriers, held transfers, contact windows, margin classes and sparse object margins. These are combined in one adapter; the old RobotKit motion-bound copies and `ProbeWireClearance` remain deleted.
- The constructor consumes a description and an optional explicit mechanical mapping. Link-named authored descriptions bind into the same mapping and use the same build/query code. Source shapes use `CollisionShape`; live observations replace the model's source geometry rather than appending duplicate hulls. Motion bounds use main's shared geometry helpers and retain MT12b's held coordinates outside the active group. No separate `margin`/`contactMargin` instance fields or group-number tool convention remain.
- Main's v29 machine contact surfaces and MT12b's v29 typed tool/weld metadata are combined at **SceneArtifact v30**. Producers use the current codec; no old-format reader is retained. Description copies use main's append operation extended to preserve physical members, tool identity and sparse margins.
- Haxeon loop-context fix `1d96bab5` is replayed onto main `0072b941` as **`e65df868`**. Compiler unit checks and eight HashLink fixtures pass. The shared Haxeon main fast-forwards locally to it; no push occurs. RobotKit compiler-only validation passes on the reconciled source. That merge was explicitly a **checkpoint**; the final gate evidence is recorded below. Materia main is untouched.

Done. CL9 review fixes (after merge checkpoint `a8afd9f13`):
- Detach restores the original authored seat rules and sparse margins. Attach contacts accept exact object ids or complete physical members. Transfers involving an open contact window are rejected before mutation; windows use sparse overrides only, without allocating new groups. Corpus additions query a released vise seat, transfer refusal/window survival and member-name contacts.
- Rule storage has keyed pair indices. CollisionDescription owns held-body rules and margins for every piece, and payload declarations only call that owner. Noncontact pairs use the class default rather than dense per-object defaults. Installed welding margins are class entries; only named tool/work pairs retain sparse overrides. Compound-payload coverage declares one piece and checks the second piece's permitted and forbidden contacts.
- Mechanisms are captured before free project objects. Snapshot inputs use main's runtime `CollisionShape` records; the old AssemblyLinkHull type and production ClearanceBodyData are gone (plain fixture rows remain in test support only). Reach independently observed CAD enters through `snapshot.part()` instead of extracting geometry from a temporary description. The vise null guard precedes stockContacts(). The real track fixture uses `WeldPathPlanner.AIR_MARGIN`.
- The probe is now the **flat-ended cylinder cut by the welder**, preserving CAD diameter, stickout and distal rim. Its exact geometry needs no rounded-end assumption or additional envelope reserve. The prior capsule-dimensional assertion is changed to cylinder dimensions for this reason; original numeric diameter, stickout, gap and stopping-budget expectations remain. A new rim-obstacle case would be missed by the previous rounded capsule. Closing physical replay passes at **−0.0100011925 mm** signed wire gap and **1.2803716526e-15 m** sensor/FK error, independently rejecting actual wire penetration.
- Latest main's app manifests still referenced removed AnimKit native source files. Those inputs now point at the new Haxeon animation package in all affected app manifests; this is a dependency-path migration, not sheet-metal feature work. No extra build directories or OCCT build are used. The complete compile/native/application and sequential gate passes as recorded below.

Done. CL9 closing evidence and integration decisions:
- The final sequential gate has **37 stages with passing results**, ending `finished: failures=0` in `/home/joao/dev/materia-cache/claude-scratch/mt-suite-mt12b-cl9-close-20261013.txt`. It was **repaired and resumed**, not uninterrupted. Heavy stages ran serially through `heavy-run`. Failed phases and their successful replacements are retained. OCCT stayed prebuilt; no extra build roots were created.
- All kit suites, five HXI audits, CadKit and full MachineKit smoke pass. RobotKit **5483**, MotionKit **1225280**, ToolpathKit scenarios **3020**, CadBridge **178**, CncKit **317**, StockKit **100673 / 17494 reference rays**, CamKit **12311**, ProcessKit **23 + 19** and ProjectKit **275 + 18** pass; HumanKit and AnimKit pass. Main's regression additions increase RobotKit/ProjectKit counts. Native collision is **4/4** (main adds broadphase coverage), native motion **16/16**, CollisionKit **383**, clearance **23**, cell **104 (74 corpus)**, contact-search **144**, MuJoCo collision **63**, and physical CadBridge feedback all pass.
- Physical wire, vise, tending route, CNC interface, four-part production/recovery, full default project-source and full editor/Worker suites pass. Reach remains **Reach1300 / 550 mm / 96 poses / 541 mm**, **10.4600359266%** joint margin and **0.8781054926** wrist sine. Production remains **287.74 s**, four outfeeds/144 steps, each cut **53.70 s**. Startup seating is **24.23 s**; interface **61.61 s**, spindle coast **0.28 s**. Utilisation remains **74.65% cutting / 75.1% spindle-on**; MT14's cutting-margin acceptance is not claimed met.
- Mill remains **52.73 / 53.70 s** bare/enclosed; 10 ms tracking remains **0.034544 / 0.051464 / 0.059015 mm** X/Y/Z. Rapid limits remain **15.095544 / 25 / 21.521827 m/min**; steady-load accelerations **29.53 / 28.12 / 27.04 m/s²**. Door **1.99 / 2.37 s**, seated clamp **0.08 s**, declared actuator force **482.548632 N**, brake sag/drift **0.000375764 / 0.000019906 mm** remain. This force is not a jaw–blank normal-force measurement.
- Other recorded baselines remain: screw/belt router **232.2 / 215.2 s**, 128 plans and **0.05 / 1.75 mm** deviations; CoreXY **649.6 mm/s / 71.549 m/s²**; arm **5.8 / 12 / 17.2 / 23.4 s**; mobile straight **400 mm / 1 s**, turn **0.5 rad**, mission **57.3 s**, obstacle round **61 s**; welder **21.5 / 24.9 s**, four-side post **27.7 s**. All four cobot classes retain **2.06 s** and **0.005433 / 0.006833 / 0.014529 / 0.021859 mrad** peak errors. No existing tending/example timing baseline moved in this reconciliation.
- The gate caught an authored-margin regression in transfer. `setHeldContacts` now restores only the previous grasp's temporary margins, keeping authored process values. Main's `copy`/`append` also preserve that restoration history. The existing **0.002 m** authored-margin assertion remains unchanged; switching pads is covered for every compound piece. Test-only motion-envelope references now use KinematicsKit, and plain hull rows live only in MotionKit clearance-test support.
- The inherited press-brake physical test exposed **−0.1076 mm** soft home overtravel, outside its unchanged **±0.1 mm** switch activation window. Checkpoint **`17908acce`** declares critical ram-cap responses from maximum phase speed and an **assumed 50 µm** no-load cap-deflection allowance (`tau = d/v`, subject to MuJoCo's timestep floor). Both guides share that mechanical response. The original switch windows and position assertion stay; an additional independent deflection bound passes at **7.0882237983e-17 m** settled home displacement. Contacts, hydraulic stroke, home and reset/emergency lifecycle pass. This intentionally changes this inherited press's end-stop physics, without changing mill or other-machine physics.
- Main's **`5aa81d68d`** placed press brake in Machining. The stale Start-page family count changes **2 → 3** for that explicit reason; additional checks independently require both mill variants on one card and press brake on its own. Worker gallery's missed plain-material migration now stores the current JSON-wire envelope. All **19** material records/values and every other document section are independently verified unchanged. Both shipped worker demos use current v13 robot models; no compatibility reader or converter was retained.
- Failure evidence: `mt12b-cl9-old-envelope-fixture.log`, `mt12b-cl9-animation-unpopulated.log`, `mt12b-cl9-authored-margin-transfer.log`, `mt12b-cl9-press-home-refusal.log`, `mt12b-cl9-stale-machining-family-count.log`, `mt12b-cl9-gallery-material-wire-refusal.log`. Required animation cgltf/stb/ozz pinned checkouts were populated only here. Shared disk pressure reached **287 MB** free; generated objects and five completed-test CMake caches were removed only in this worktree, retaining installed libraries, executables and logs. App and remaining native-test build trees were retained. No source or dependency checkout was discarded.
- Constructor-time reach search remains MT14 B7: assembled mission **1586.6 s**, independent reach **1639.7 s**, controller check **1578 s** wall time. The full MachineKit run completed without earlyoom, at **1.6 GB** peak. No search or assertion was skipped to close this gate. MT13 and the remaining MT14 acceptance items remain open. Main's later **`4a025a691`** foam-cutter plan is documentation only and is included without changing the tested code. Stop for review before any Materia main update or push.

**MT13. Wrist camera for the trays.**
- **Hardware:**
  - `CameraBracket` and a camera part (MachineKit) on the `ArmGripperTool` plate, beside the gripper, looking along the approach axis.
  - An `optical` connector gives the CAD extrinsic, so the mounting is derived from geometry.
  - The housing and cable join the tool's collision hulls and mass, and must clear both grippers of MT12's dual tool.
- **Sensor:**
  - a `camera` sensor on the tool link at the optical connector;
  - SensorKit renders RGB and depth with its noise and latency models;
  - segmentation is test ground truth only, never pipeline input, because real cameras do not provide it.
- **Calibration:**
  - hand-eye (camera to flange) solved from ChArUco views at several arm poses;
  - VisionKit gains hand-eye solving, its open TODO, through OpenCV in the native kit;
  - the pipeline uses the solved extrinsic, not the CAD one, so simulation and hardware take the same path. The CAD value is the check.
- **Vision:**
  - tray registration from AprilTag or ArUco markers on the tray corners (`MarkerDetector`, `vk_solve_pnp`, RobotKit `FiducialPerception`);
  - slot occupancy, and each blank's pose from depth (top-face plane fit plus edges), measured in the registered tray frame;
  - pick targets come from the measured blank pose, not the nominal slot frame.
- **Mission:**
  - a look step while the mill cuts;
  - an unreadable or empty slot is a named failure in the MT11 style;
  - the jaw switch still judges every grasp.
- **Clean break:** MT13 replaces MT9's per-slot tray presence sensors, which are removed. The vise presence sensor stays.
- **Tests (MuJoCo plus rendered frames):**
  - a tray placed ±10 mm and ±5° off nominal registers within 1 mm and 0.5°;
  - blank poses are measured within 0.5 mm and 1° of simulation truth;
  - occupancy is right with empty and misplaced slots;
  - the solved hand-eye extrinsic is within 0.5 mm and 0.2° of CAD;
  - the look time is hidden behind the cut, so spindle use is unchanged from MT12;
  - the four-part acceptance still passes with the trays offset from nominal.

**MT14. Cell hardening (MT12 review sections B–D).**

Finish MT12b first. Each line below is an acceptance item, not a claim that the prototype already satisfies it. Small independent fixes may land earlier with their evidence recorded. Heavy stages use `heavy-run`; closing acceptance uses the full sequential gate, and review precedes any main update.

Safety and decisions:
- [ ] B1: Replace mission-authored `Clear`/`robot.clear` with a robot zone monitor over arm and payload hulls in the MT12b description and the enclosure-derived machine zone volume. Remove `Clear` from `TendingOperation`; PLC door/cycle permission uses the measured monitor.
- [ ] B2: Move robot-side request/wait/release, retry and alarm mapping into a reusable CncKit `RobotInterface` client. Replace station-prefix/message-substring alarm classification with typed alarm codes.
- [ ] B3: Derive mill interlock wiring from `connectPorts` and typed sensor/valve roles. Remove hard-coded door/jaw/sensor names, `input${index + 6}`, `baseX + 350`, `-1000,-1000` and string re-prefixing of CNC jobs.
- [ ] B4: Give gripper, door and vise one controller-layer double-solenoid driver. Move the gripper's 40 ms coil pulse out of `SimulatedPneumaticGripper`; simulation keeps pneumatic physics only.
- [ ] B5: Add `CamJob.endAt(machinePosition)`. Remove BearingBlockJob's last-operation splice and load-position inference from the final program move.
- [ ] B6: Put contact surface properties on materials/fixtures; remove the pneumatic-actuator-triggered scene-wide contact switch. Reconcile MT-D5's 1 ms claim with the actual 0.5 ms contact setting.
- [ ] B7: Restore a real offline reach search or declare class/riser/stand-off as named design inputs with rationale. Remove singleton search loops; store the certified layout instead of solving it in `TendingControllers`. Name/derive tool constants and dimensions (`-12`, `z = 10`, `tendingTool` arguments).
- [ ] B8: Introduce typed `TendingStation` values; remove parsing of infeed row/column, vise-load, approach and clearance station strings. Remaining sites include `TendingReach.trayStock`, `trayPose`, `environmentBodies`, request construction and the `request.name == "vise"`/`"vise-load"` branches (`infeed-`, `outfeed-`, `-approach`, `-clearance`, `machine:` ids); `MissionPlayer.cellCollisionSnapshot` also strips `project:` ids.
- [ ] B9: Assert that the statically certified grasp equals the runtime handling grasp.

Tests and physical truth:
- [ ] C1: At every production `CYCLE_START`, assert blank-to-seat alignment near 0.1 mm/2 mrad and settled contact on both jaw faces.
- [ ] C2: Measure settled MuJoCo jaw–blank normal force against p·A, using a real blank rather than a joint limit. Apply a modelled/equivalent cutting wrench to stock and reassert clamped hold.
- [ ] C3: Restore a relative clamp-time bound; explain or fix the measured −18% discrepancy. The current ±30 ms bound is about ±31%, compared with the former ±10% requirement.
- [ ] C4: Accumulate slip, both-pad contact and free-body evidence separately for every gripper channel inside the four-part production loop with the actual tending blank and dual gripper.
- [ ] C5a: Simulate e-stop mid-carry while holding stock.
- [ ] C5b: Simulate e-stop mid-cut.
- [ ] C5c: Simulate loss of air.
- [ ] C5d: Simulate a door-close jam.
- [ ] C5e: Simulate a stuck jaw and both `ClampTimeout` and `UnclampTimeout` recovery.
- [ ] C5f: Simulate a part missing mid-run.
- [ ] C5g: Simulate occupied outfeed.
- [ ] C6: Report cutting/in-cycle and spindle-on utilisation separately and state which target applies. 214.8/287.74 s is 74.65% cutting, while 75.1% includes spindle run-up/coast-down. Require real margin for 75% acceptance, fourth-outfeed elapsed time with about 5% slack, and panel/encoder utilisation agreement near 0.01.
- [x] C7: Add maintained sequential heavy-run stages for `tending`, `tending-route`, `tending-vise` and `interface`; the explicit four-part acceptance must no longer depend on one scratch log. The gate script now schedules all four unconditionally and removes its duplicate failure-only tending rerun; all four stages pass in `mt-suite-mt12b-close3-20261012.txt`.
- [ ] C8a: Assert tray slot pitch equals `pitchX`/`pitchY`, retaining independent non-overlap coverage.
- [x] C8b: Exclude same-part convex pieces from held-part Check rules using typed physical-object identity. MT12b `CollisionDescription.setHeldContacts` only creates checks against other bodies; compound physical pieces share a body. CollisionKit and the cell corpus cover the rule.
- [ ] C8c: Skip only the candidate whose timing refinement reports an uncertified edge; retain other reach candidates instead of aborting the study.

Structure and plan:
- [ ] D1a: Split `MissionPlayer` into a skill-runner core and handling, welding, energy and signal modules.
- [ ] D1b: Move generic `CellSignals` into a kit and delete unused `writeSignal`, which bypasses ownership validation.
- [x] D2a: Reconcile current reach statements and REVIEW_FIXES with the MT11 Reach1300 selection and MT12 certified 550 mm riser/541 mm stand-off/0.878 wrist sine. Label old Reach850/600 mm/15-pose values as historical checkpoints rather than current acceptance.
- [x] D2b: Keep MT12's release dependency on MT12b explicit in progress and acceptance text (amended at the MT12b review boundary).
- [ ] D2c: Decide whether to move the historical per-step evidence into `HISTORY.md`, preserving links and measurements while keeping this document a usable plan.

**Later.**
- Real I/O:
  - LinuxCNC HAL pins and `halui`, with `M62`–`M66` in `CncInterpreter` (`ToolpathOp.SetOutput`/`WaitInput`);
  - robot digital I/O through RKD6 device channels;
  - the same `RobotInterface` running on the CNC side or as LinuxCNC logic.
- An umbrella automatic tool changer (`M6` handled by the machine).
- Planned approaches with CL6/CL7 in place of derived via-poses.
- A second operation with a regrip or flip station.
- Fixed (mast) cameras, if a later cell needs observation while the arm is busy; MT13 covers tray vision with a wrist camera.
- Chips and coolant.
- Mobile tending: the arm on the mobile base, docking, and touch or vision registration to the machine (the welder's P3).

## Order

```
MT0 ─┬─ MT1 ── MT2 ─┬─ MT4 (reach study) ─┐
     ├─ MT3 (arms) ─┘                     ├─ MT6 ── MT7 ─┐
     └─ (welder + X7/X8 merged) ──────────┘              ├─ MT10 ── MT11 ── MT12 ── MT12b ── MT13
                        MT5 (after MT4) ── MT8 ── MT9 ───┘
```

- MT1–MT4 need only X7's transmission API. MT3 can run in parallel with MT1–MT2.
- MT5 is independent of controllers, but the gripper (MT8) and the vise and door actuation need it.
- MT6 needs X8 wiring.
- MT10 needs everything before it.
- MT12b comes after MT12, so it doesn't disturb MT10–MT12 in flight, and before MT13, whose camera housing joins the tool hulls through it.
- MT13 needs MT8's tool, MT10's mission and MT12's dual-gripper envelope. It replaces MT9's per-slot tray sensors.

**Articulated inertia and joint-plan admission (MT5b follow-up).**

Done. What was built and decided:

- Use every carried link in the neutral inertia estimate; simulation feedforward uses the live articulated mass matrix, including cross-axis terms, projected through nominal transmission coordinates. Remove unused scalar inertia from native targets and blueprints and migrate callers/bindings without a compatibility path.
- Admit mission joint moves only after checking their timed polynomials against the controllers' mechanical hulls. A speed bound controls interior sampling; this is a sampled check, not a continuous collision certificate. Require actual velocity limits, and keep installed mill drive references typed.
- Check a compiled G54-zero move against stock/spindle geometry; check posed doorway bounds, numeric casting masses, exported cobot limits and CAD overlap at zero/ready poses.
- The complete `review-inertia-clearance` gate passed every suite and app checks. RobotKit 4,999; MotionKit 67,535; CadBridge 157; CncKit 317; CamKit 12,311; ProjectKit 144; ProcessKit 23 + weld-path 19; StockKit 100,673 + 17,494 reference rays. Native runtime/MuJoCo checks passed.
- At 10 ms the bare/enclosed mill remains 51.77/52.49 s with X/Y/Z peaks 0.077237/0.051505/0.053502 mm. Rapids remain 15.0955/25/22.8666 m/min. Door and seated-blank clamp remain 1.50/1.79 s and 0.08 s at 482.55 N.
- Cobot cycles remain 2.06 s. Reach500/850/900/1300 peaks change to 0.004640/0.005987/0.011236/0.016598 mrad because feedforward now includes the complete articulated inertia. Other MT6/R2 motion baselines remain unchanged.
- Historical checkpoint (superseded by MT12/MT12b): Reach stays Reach850, 600 mm riser, 15 poses and 16.2399% minimum joint margin. R2's separate spindle motor increases the cell from 15 to 16 disjoint joints.
- The physical brake, remaining environment/limit checks and support geometry are still open in [REVIEW_FIXES.md](REVIEW_FIXES.md). This checkpoint does not finish those review groups.

## Progress

| Step | State | Commits |
|---|---|---|
| MT0 | done: main and X7+X8 merged | c5438ba4d, 8ad034403 |
| MT1 | done | 592affc2f, 7d9f192b1, 5e40b76fe, 53248d881 |
| MT2 | done | 06c9bf5fd, 73a14c58e |
| MT3 | done | 4d58cd5d3, 8a44aeb53 |
| MT4 | done | c945a0dc8, f518b0dff |
| MT5 | done | 453d890ad, 5f06c32e5, a397b187a, a6230c46e, 1484b924d |
| MT5b | done: review regressions | 04d3d092d |
| MT5b follow-up | done: review fixes and complete closing gate | 4ff013c90; full history in [REVIEW_FIXES.md](REVIEW_FIXES.md) |
| MT5b second review | done: verified and approved compiler pin recorded | 3c5eef1dd, 3ff303c2d; Haxeon 8be7ca0e; see [REVIEW_FIXES.md](REVIEW_FIXES.md) |
| MT6 | done, baseline attribution rechecked after integration | 4c9e6ebec, daf20e12d, 3c5eef1dd, 6240e8b5c |
| MT7 | done | 1780b60fa |
| MT8 | done | 371070b1d |
| MT9 | done; sequential closing gate passed | ac42c8866, 3dd21bd25 |
| MT10 | done; CollisionKit integration closing gate passed | a5c6bb9da, f2e04aff3, 9b5bfb299, 3dd21bd25 |
| MT11 | done; physical production, recovery and integration gates passed | a65789a38, 81f5563f7, 3dd21bd25 |
| MT12 | implemented: 287.74 s, 75.1% spindle-on / 74.65% cutting; release depends on MT12b; MT14 hardens acceptance | `e46b42297`, `a1e9ab0dd`, `955e5196e`, `b71c18787`, `f4a2030a6` |
| MT12b | done: CL9 reconciled, v30 clean break and full repaired/resumed 37-stage gate passed; awaiting landing review | `024ab9242`, `a8afd9f13`, `45c0a5c75`, `17908acce`, `6813c938a` |
| MT13 | planned | |
| MT14 | planned: cell hardening checklist from review sections B–D | |

**Environment clearance and support geometry (MT5b follow-up, verified in `8b22593be` and `20f0d3e16`).**

- Joint-plan clearance includes independent scene collision objects at their actual simulated poses. Held objects follow their physical carrier link in planning FK. Two app regressions reject an independent obstruction and a carried object's future collision before completion; clear cobot missions retain their 2.06 s cycles and tracking peaks.
- Four saved 30 × 30 × 2 mm steel tube legs support the cabinet to the floor. CAD checks verify floor reach, finite cabinet contact and the hollow section. The shared rectangular profile names its outer stock and bore separately; no naming-audit exception was added.
- The bearing blank becomes 60 × 60 × 20 mm, retaining the centred 608 seat. The 12 × 12 mm contour leaves a 3 mm front wall and 4 mm seat ligament; CAD material witnesses check the actual cut solid. Moving the seat in the old blank caused CAM to reject a toe-clamp collision, so the design gives all features and clamps sufficient stock room.
- Focused bare/enclosed app checks pass at 52.51/53.24 s, 78 plans. Nominal removal increases by 144 mm³ to 3846.78 mm³; measured removal is 3860.62 mm³, leftover 7.14 mm³ and gouge 0.618 mm³, with no rapid or holder collisions. X/Y/Z tracking peaks become 0.085925/0.051505/0.053502 mm at 10 ms. The larger blank increases carried mass; rapids remain 15.0955/25/22.8666 m/min and accelerations become approximately 29.86/28.95/26.93 m/s².
- Historical checkpoint (superseded by MT12/MT12b): Door 1.50/1.79 s and seated clamp 0.08 s at 482.55 N remain unchanged. Cabinet supports add 5.2752 kg of stationary structure and bring the enclosed scene to 122 objects. Reach remains Reach850 on 600 mm, with 15 poses and 16.2399% joint margin.
- All eleven kit suites, CadKit and MachineKit smoke passed. The first complete app run caught a stale analytical pocket-volume expectation; after correction, the complete app retry passed with exit 0 and no uncaught exception. Router, belt router, CoreXY, arm, mobile and welder baselines match their recorded values. The physical Z brake and rail support sizing remain open; no MT7 work has started.

**Brake controller and simulation integration (MT5b follow-up, mill binding still open).**

- The typed runtime blueprint binds a braked joint to a controller-owned digital release output and a released-contact sensor. Timeout and sample-age limits are controller policy; holding effort and physical contact delays stay in SimKit.
- The runtime freezes trajectory and ramp clocks until newer released feedback arrives. Waiting references have stationary Hermite endpoints, zero velocity and zero acceleration feedforward. Normal stops retain release; emergency stops drop it; reset requires a new feedback epoch. Feedback age uses the owner clock, leaving public wall-clock receipt metadata unchanged.
- Registered native checks pass for waiting, fresh release, normal stop, emergency stop, reset, missing/stale/lost feedback and rejection of program writes to the brake coil. The complete existing runtime, ABI validation and C API suites pass against rebuilt libraries. Device-owned queues require their own handshake and are explicitly unsupported for this host gate.
- Physical simulation descriptors now match each controller brake to holding effort and contact delays. Session commands carry the coil to the physics owner; joint snapshots carry actual contact feedback back. Idle and normally stopped servos hold position while released. Emergency stop disables motor effort through the complete braked transmission group; reset holds the observed stopped pose before release.
- The integrated 10 kg gravity fixture passes: a 150 N brake drifts 0.147 micrometres over 0.3 s after application, while a 50 N brake slips 693.96 mm. The assumed 30 ms application delay permits 12.34 mm of sag before the rated brake settles. Applied friction uses MuJoCo's two-physics-step response floor and maximum supported impedance; original friction response returns on release. Transmission stiffness remains independent.
- Recompiled native runtime, brake-runtime, ABI validation, C API, SimCore, SimKit MuJoCo and RobotKit MuJoCo suites pass. The physical mill brake part, assembly binding, saved-model migration and actual Z gravity-holding checks remain required, followed by the complete closing gate. The generic fixture does not establish the actual mill result; no MT7 work is introduced.


Physical mill brake follow-up (closing gate pending):

- Installed an explicitly assumed 4 N m spring-applied Z motor brake: rotating press-fit hub/disc and sixteen convex sectors for each fixed front/shell/back ring. Its assumed release/application delays are 50/30 ms. The motor shaft passes through the 8 mm mount and 10 mm brake envelope into a 32 mm coupling, retaining 1 mm stator-to-coupling clearance. Rotor and stator masses enter the ordinary derived mechanical model.
- The physical part supplies effort and delay ratings; the assembly binding supplies only scoped digital release/contact names and timeout/freshness policy. Driver controller wiring carries the release coil and feedback. No runtime cylinder or brake physics is introduced.
- Current formats are MachineKit description v16, ProjectKit assembly definition/state v8 and RobotModel v12. Old versions are rejected; in-repo robot fixtures/deployment files use v12. Brake bindings survive documents, nested include/rebuild and two distinct mill scopes.
- Native session and brake regressions found and fixed omitted-clock sensor freshness (`61fd8f6af`) and fault rollback discarding emergency outputs (`144010183`). The actual mill passes healthy release, motion, normal stop, emergency stop, reset and jammed-release checks. Its 4 N m brake permits 0.665237 mm emergency-stop sag during application; settled drift is 0.000020013 mm over 0.5 s. Native complete runtime checks pass. Geometry smoke, new mill cycle measurements and the complete closing gate remain pending.

- Brake geometry and saved-binding checks now pass in the complete MachineKit smoke. The actual added mass raises the checked Z guide utilization to 0.636247. App assembly setup exposed a 64-child compound cap; `295d426ea` removes that arbitrary limit from dynamically stored compound shapes. Complete native MuJoCo checks pass with 80 children, a clear concave gap and physical support on the last child. Full mill cycle measurements are pending the rerun.

- Physical mill checkpoint passed with geometry and installed mass: bare/enclosed bearing-block cycles 52.57/53.22 s (both +0.03 s from the paired-guide checkpoint); 78 plans, 55 flagged, no rapid or holder contacts. X/Y/Z peaks are 0.085940/0.051497/0.060011 mm at 10 ms. Rapids stay 15.095544/25/21.521827 m/min; accelerations are 29.77/28.24/27.16 m/s². Z acceleration increases from 26.44 m/s² because the coupling shortened around the installed brake, lowering net rotating inertia. Door 1.50/1.79 s and seated clamp 0.08 s/482.548632 N are unchanged. Final installed-brake emergency sag is 0.665314 mm, settled drift 0.000020016 mm over 0.5 s. The complete closing gate and cross-example baseline audit remain pending.


Final MT5b follow-up gate — done (2026-10-05):

- `mt-suite-review-closing-brake.txt` passes all eleven kit suites, CadKit, MachineKit smoke, app build and complete project-source tests. Twelve recompiled native suites and three generated-binding checks also pass. See [REVIEW_FIXES.md](REVIEW_FIXES.md#final-verification--2026-10-05) for counts, evidence and commit history.
- Historical checkpoint (superseded by MT12/MT12b): The full gate reproduces the final mill measurements above: 52.57/53.22 s bare/enclosed, 1.2 m/min pocket feed and 0.3 m/min entry feed, 0.085940/0.051497/0.060011 mm X/Y/Z tracking at 10 ms, 0.665314 mm brake application sag, door 1.50/1.79 s and seated clamp 0.08 s/482.548632 N. All four cobot classes retain 2.06 s cycles. Reach stays Reach850/600 mm at fifteen poses and 16.239926% minimum joint margin.
- Screw/belt router, CoreXY, arm, mobile and welder reproduce the previously recorded post-main/review baselines. Only intended mill geometry, rotating inertia and brake sequencing changes move its figures. ProjectKit's 146→144 count reflects removed legacy compatibility checks plus added current-format checks, as requested for the clean break.
- Closing fixes: `61fd8f6af` resolves one default session clock for every phase; `144010183` preserves staged emergency outputs after a fault; `295d426ea` removes the dynamic-compound child cap with last-child contact verification; `4ff013c90` installs and saves the physical brake and scoped controller binding. The generated binding receives the matching clock documentation. No new confirmed haxeon bug or compiler patch. MT7 is not started.


Electrical follow-up checkpoint (2026-10-05, MachineKit verified; application gate open):
- Replace the interim high-voltage mobile arm supply with assumed 48 V windings, a 75 A DC source and 32 A phase-RMS amplifiers. Keep the mechanical motor curves. Per-motor resistance follows an assumed 5% rated copper-loss budget; fast cobot wrists reserve DC-bus headroom at their own maximum speed.
- Separate AC RMS voltage, DC voltage, DC current and phase-RMS current with no-conversion abstract types. Carry phase system and frequency in `AcMains`. Remove the redundant `ElectricalSource.outputVoltage` API and migrate call sites directly.
- Haxeon's abstract receiver bug is fixed in local compiler commit `1c51f79e`; quantity accessors use `return this` without casts or class workarounds. The parent Haxeon pin remains unchanged. Compiler TestMain, five abstract runtime programs, operator/enum-abstract tests and official-Haxe parity pass. MachineKit compiled 1336 sources and its complete smoke suite passed.
- Moving arm mass stays 20.3 kg; revised amplifier envelopes change total arm mass 155.9 to 155.1 kg and mobile welding payload 583.6 to 582.8 kg. Its derived acceleration rounds from 0.207 to 0.208 m/s². Mill rapids stay 15.09554/25/21.52183 m/min versus the rebased checkpoint. Full application timing/tracking verification remains open.

Second-review closing work (validated):

- Powered emergency stop retains measured-position servo holding until fresh brake-engaged feedback; hard power loss removes torque immediately. The brake coil drops immediately in both cases. Brake state uses the existing joint-state flag word, and stator geometry uses the minimum sectors needed to clear its rotor.
- Joint plans require conservative continuous hull-motion certificates instead of fixed angular samples. Guide blocks carry transverse loads and residual moments while the screw carries axial force at its assembled line of action.
- MT6 validates sensor and program ownership, retains single-cabinet identities and animates each controller's belts independently. Shared mounted geometry has an explicit transient contact group; unrelated robots and loose objects retain contact. Environment CAD uses the existing bounded hull generator.
- Presence sensing tests every supported loose-object shape against its volume. CNC process scope is explicit in SceneArtifact v18; MT8 adds saved gripper limits as v19, with migrated producers and no old-format reader. Unpowered spindles coast; asymmetric belt tests distinguish coordinate direction from power direction.
- The sequential `review-final` gate and its focused retries covered the completed implementation at that checkpoint. Compiler corrections, regression evidence, final measurements and commits are recorded in [REVIEW_FIXES.md](REVIEW_FIXES.md). MT7 and later had not started yet.

Done. What was built and decided (second-review validation):

- The sequential kit gate passed every kit except the initial MachineKit compile, which exposed a compiler defect. The repaired compiler's MachineKit retry passed. Project-source coverage completed with focused retries after two invalid new fixtures were corrected; the full editor AppTests suite, thirteen native suites, supplemental RobotKit tool/process/weld/clearance suites and HXI binding integration all passed. Long welder searches were not repeated.
- At the pre-balanced-fusion closing-brake checkpoint, the mill retained a 10 ms controller tick and drive-derived 0.25 ms physics interval. X/Y/Z rapids were 15.095544/25/21.521827 m/min, with 0.085940/0.051497/0.060011 mm peak tracking. Bare/enclosed bearing-block cycles were 52.58/53.23 s, 78 plans and 55 existing torque diagnostics. Door opening/closing was 1.50/1.79 s; the seated-blank clamp was 0.08 s at 482.548632 N. Powered e-stop Z sag fell from 0.665314 to 0.000376 mm because servo holding persists until fresh engaged feedback; settled drift was 0.000020016 mm over 0.5 s. These are the sequential-fusion comparison values, not the later balanced-fusion result.
- Historical checkpoint (superseded by MT12/MT12b): Correct screw/guide reactions produce bare X/Y/Z guide utilizations 0.156532/0.383137/0.825162 and enclosed 0.216948/0.446731/0.825162. Capacity and cutting load remain assumed reference envelopes. The enclosure now has 156 parts after reducing the brake stator from 48 to 26 convex pieces. Reach stays Reach850 on a 600 mm riser, fifteen cell poses and 16.239926% minimum joint margin.
- Screw/belt router baselines remain 232.2/215.2 s, 128 plans and 0.05/1.76 mm worst deviation. CoreXY remains 649.6 mm/s and 71.549 m/s²; arm pick/place remains 5.8/12/17.2/23.5 s; welder remains 20.6/21.1 s seam runs, 29.8 s post and 106.5 s whole weldment on both backends. All cobot classes remain 2.06 s. Reach850 peak changes 0.005987→0.006131 mrad because upstream `574d49ba9` reacquires the live arm start between mission owners: disabling that reset reproduces the old peak exactly. The other three peaks remain 0.004640/0.011236/0.016598 mrad.
- The rebased upstream world-aligned floor grid (`75c98a563`) changes mobile mission completion 61→56.4 s and obstacle round 65→60 s. Restoring the old grid origin reproduces both prior timings exactly; restoring box lidar envelopes does not. The 400 mm straight move and 0.5 rad turn remain unchanged. Every diagnostic source change was restored.
- Quantity APIs separate AC RMS voltage, DC voltage, DC current and phase-RMS current without implicit conversions. Haxeon corrections `1c51f79e`, `d934742e`, `8be7ca0e` fix abstract receivers, generated callback handles and nullable comprehensions/IR array growth; compiler TestMain and real HashLink/HXI regressions pass. With explicit user approval, the parent Haxeon pin now records the tested `8be7ca0e` revision. No other submodule pin, main integration or MT7 work was changed.

Current-main integration checkpoint (2026-10-05):

- Integrate main snapshot `9e82c686b` and compiler `45da6c04`, retaining typed electrical quantities, drive interpolation, controller ownership and the brake handshake alongside upstream homing/squaring. C API v31 identifies the combined native layouts; four-platform binding audits pass.
- Joint-motion clearance keeps 5 mm at obstacles and 3 mm only at named mechanical interfaces derived from joints and their directly fixed mounting parts. Multi-hull parts keep one semantic part name on their physical link; each convex facet remains checked. CAD-hull held objects follow their actual carrier link. RobotKit passes 5454 assertions, MotionKit 71402, CadBridge and MachineKit smoke; all thirteen native suites pass.
- Historical checkpoint (superseded by MT12/MT12b): Reach remains Reach850/600 mm, fifteen poses and 16.239926% minimum joint margin. Reach850 tracking returns to 0.00598713171037768 mrad at 2.06 s because upstream `2ce91eb1f` seeds new manipulator programs from commanded setpoints; the live-runner reset is still present.
- New upstream examples pass: gantry picker 177.1 s, yaw picker 184.36 s (six cartons, 48 plans, no stalls); gantry welder 182.3 s for ten seams; track positioning 15.88 s and a 2.6 m track weld in 236.7 s with no clearance violation. Track drive packing uses matching assumed 18 mm rack/pinion faces, full rack support and a derived 6 mm drive clearance requirement.
- Robot-welder whole-weldment cycle is now 114.4 s on both backends (37.9/68.9/90.5/114.4 s milestones), with no clearance violation. Seam/restart cycles are 21.5/21.9 s on both backends, 5 mm legs and 3 mm restart overlap. Main changes shared planning intentionally: `efa1df734` introduces drive-derived limits, and `44404731f` adds free-about-tool orientation to welding approach, process and retreat paths. These alter planned motions; the cycle change is recorded rather than hidden. Attribution to the combined planner changes is based on source inspection and identical backend results, not an isolated before/after experiment.
- Closing coverage passes through the initial sequential gate and focused repairs/retries, including full rebuilt editor AppTests. CadBridge is now 181 assertions after a regression for flange export across controller domains; foreign occurrences do not require links in the selected controller's model. Original failed gate logs are retained; no uninterrupted monolithic success is claimed.
- The controlled mill comparison isolated main's balanced-tree `Part.fuseAll` change (`4b52eb740`): `MillCasting.geometry` builds its base/column/saddle unions through this shared method. With balanced fusion, the measured bare/enclosed cycles were 52.55/53.22 s and bare X/Y/Z peaks were 0.048472/0.051497/0.060011 mm. Overriding only `Part.fuseAll` to restore sequential fusion reproduced 52.58/53.23 s and 0.085940/0.051497/0.060011 mm. The +0.03/+0.01 s and +0.037468 mm X-tracking deltas belong to that geometry-fusion change; the brake stator/mass and braced stand were present in both sides of the controlled comparison. The post-merge `PROJECT_SOURCE_ONLY=mill` rerun reconfirmed the balanced-fusion measurements: bare/enclosed cycles 52.55/53.22 s, bare X/Y/Z peaks 0.048472/0.051497/0.060011 mm, and no rapid or holder contacts. The enclosure bracing introduces no measured baseline shift at this resolution.
- Main's physical router homing adds 104.8/57.8 s to unchanged screw/belt machining cycles of 232.2/215.2 s. Screw removal/leftover/gouge now measures 9899.8/105.7/27 mm³ with the calibrated homing path; upstream G13 explicitly changes the sensor mount and the recess voxel-volume tolerance. Worst screw/belt tracking remains 0.05/1.76 mm. CoreXY and mobile reproduce their recorded values. The post weld now measures 36.2 s; weave and multipass checks pass on both backends.
- At this integration checkpoint, no Materia main update or push had occurred; the branch later incorporated main through `a490c9dd2`. MT7–MT11 work and final validation are recorded in their step sections; MT12–MT13 remain after landing.

CollisionKit integration closing measurements (main `249ebcea2`, validated):

- The historical MT11 closing reach study asserts **Reach1300 / 550 mm riser / 96 poses**, with **391 mm front stand-off**, **11.763934% minimum joint margin** and wrist |sin(q5)| **1.0**. It includes the selected gripper's closing sweep, the idle gripper's open geometry, separate load/seat stations and the closed door at exterior stations. These replace the earlier open-door study's 691 mm stand-off; no obstacle margin was reduced. MT12/MT12b supersede this mounting with 541 mm stand-off, 10.460036% joint margin and 0.878105 wrist sine.
- Bare/enclosed bearing-block cycles are **52.73/53.70 s**, versus the preceding 52.55/53.22 s checkpoint. The bare example has no vise; its +0.18 s is isolated to exporting CAD inertia rather than the display-mesh-derived inertia, as verified below. With current fixture geometry, that same change reduces the enclosed cycle 53.74 to 53.70 s. The remaining 53.22 to 53.74 s comparison includes the off-centre loading mount and revised locator/parallels; it is not an isolated fixture-mass measurement. Bare X/Y/Z peaks at 10 ms are **0.034544/0.051464/0.059015 mm**, with drive-derived 0.25 ms physics. Rapids remain **15.095544/25/21.521827 m/min**; rounded derived accelerations are **29.53/28.12/27.04 m/s²**. The bare cut reports 78 plans, 55 existing torque diagnostics, 3860.657759 mm³ removed, 7.110556 mm³ leftover, 0.657759 mm³ gouge and no rapid/collision contacts. Door opening/closing remains **1.99/2.37 s**, seated clamp **0.08 s / 482.548632 N**. The installed 4 N m brake gives **0.000375764 mm** powered e-stop sag and **0.000019906 mm** settled drift over 0.5 s.
- Main's declared CAD mass/inertia preservation (`27866c5a5`) is retained by the assembly bridge. Bare/enclosed guide utilizations now measure X 0.156318/0.236653, Y 0.382564/0.432181 and Z 0.823716 in both; capacities and cutting loads remain assumed. These differ from historical checkpoints with earlier fixture geometry and physical-mass export.
- All four cobot classes still finish at 2.06 s. Peak joint errors for Reach500/850/900/1300 are 0.005433/0.006833/0.014529/0.021859 mrad. The physical gripper passes 50 round trips with 0.371 mm peak slip, 0.168 mm pickup seating and 55.04 N m peak arm torque. The generic arm remains 5.8/12/17.2/23.4 s; the six-carton picker remains 106.29 s, while the yaw picker changes 105.24 to 98.02 s (homing 8.56 s, 36 plans, no stalls). The new mass/inertia export changes derived drive limits and dynamics; this is a combined-merge comparison, not an isolated attribution experiment.
- Gantry/track/robot weldment cycles remain 159.8/232.5/99.9 s from the preceding collision integration checkpoint. Robot seam/restart/post cycles remain 21.5/24.9/27.7 s, with 5 mm legs and 3 mm restart overlap; weave and multipass checks pass on both backends.
- The initial sequential gate caught four integration omissions: the controller reach test still expected 92 rather than 96 poses; the standalone bench-mill source manifest omitted the package now owning `CollisionClearance`; and the recovery test incorrectly equated the cumulative completion counter with authored step count. The physical mission had finished at 823.59 s (176 authored steps, 178 cumulative completions because owning requests were replayed). Completion now checks the last authored step and allows the preserved retry count. The router test also re-quantized a rounded continuous rate, producing an extra pulse period; it now checks each axis against its deployed integer periods, installed channel ceilings and screw limits, and verifies that steady loads cannot raise unloaded speed. No runtime limit was changed. All are corrected. The closing validation resumes the failed and unrun phases without repeating already-passing heavy examples. The initial gate summary and failed app logs are retained. The full rebuilt editor suite, including Worker document refusal and both shipped demos, passes.

- Closing sequential coverage passes through `mt-suite-cl7-mt11-final.txt`, `mt-suite-cl7-mt11-close.txt` and `mt-suite-cl7-mt11-routers.txt` in `/home/joao/dev/materia-cache/claude-scratch/`. The initial run and first follow-up report failures from the corrected test/dependency omissions; their successful phases plus focused repaired phases cover the full gate. This is not an uninterrupted monolithic green run. RobotKit world 5461, MotionKit 1224709, ToolpathKit 3020, CadBridge 178, CncKit 317, StockKit 100673, CamKit 12311, ProjectKit 192, ProcessKit 23 plus 19 weld-path assertions pass. HumanKit/AnimKit, five HXI audits, CadKit, MachineKit smoke, app builds, full editor AppTests, native collision 3/3, native motion 16/16, CollisionKit 364, clearance validation 23, CadBridge collision 30, MuJoCo collision 59 and CadBridge MuJoCo all pass. App coverage includes real loose-blank clamping, 37.92 s startup seating, controller wiring and physical CNC interface, all recovery injections, all four production parts, physical router homing and the complete remaining router/mobile/control/background phase.
- Screw/belt machining stays **232.2/215.2 s**, **128 plans**, screw **0.05 mm** worst deviation; belt changes **1.76 to 1.75 mm**, with 56 flagged and 62 over-tolerance plans. Fresh exported mass/inertia and deployed integer pulse periods give homing **38.0/26.2 s** (preceding checkpoint 104.8/57.8 s), and both cuts report **9902.1 mm³ removed, 140.3 mm³ leftover, 63.9 mm³ gouge**. These shared-model diagnostics are recorded rather than pinned by the retired f32 pulse-period calculation. The homing and volume deltas are a combined refreshed-model comparison; no isolated causal measurement is claimed. CoreXY remains **649.6 mm/s / 71.549 m/s²**. Both mobile backends still travel 400 mm in 1 s and turn 0.5 rad; the mobile mission ends at **57.3 s** (preceding 57.6 s) and obstacle round at **61 s** (preceding 59 s). Main changes physical mass/inertia and wheel-odometry uncertainty/reanchoring; these timings are likewise a combined-merge comparison.
- The review-boundary Haxeon pin `835cf2c0` contains both requested ancestors `ccfd1dd6` and `0d14ccbd`; merged compiler fixes are described under MT11 above. At that boundary, Materia main had not been updated and nothing had been pushed. **MT12–MT13 remain planned after landing**.

- Closing merge **`3dd21bd25`** records the validated implementation and main `249ebcea2` as its second parent. JointMotionPolicy remains its separate **`9b5bfb299`** commit; the Haxeon merge/re-pin is recorded in **`6b843d326`** and subsequent compiler-fix integration **`81f5563f7`**. No required implementation or verification remains before review.

Publication gate (2026-10-10):

- After review, the user authorized publication to main. Haxeon merge `8d675fde` combines the three tending compiler fixes with upstream main `04821241`; Materia records this pin in `e3bd218fe`. The stale Haxeon remote pointing at a deleted worktree was corrected to its public repository.
- Haxeon `TestMain` passed. The complete sequential `mt-suite.sh main-replay` gate passed with `finished: failures=0`, including every kit, MachineKit smoke, native collision and motion tests, physical vise and tending route, the full app project-source suite, and editor `app/tests` with the shipped worker demos. Its summary is `mt-suite-main-replay.txt` in the shared scratch directory.
- The long production test is excluded from the default app run, so `PROJECT_SOURCE_ONLY=tending` was run separately after the gate and passed. All fault recoveries and four physical cuts passed; each cut remains 53.70 s, with 3855.14 mm³ removed and 11.50 mm³ leftover. First clamp to fourth sensed outfeed remains 772.94 s. Log: `mt-s-main-replay-production.log`. No production baseline changed with the compiler merge.
- Both remote main tips are ancestors of the validated commits. Publish Haxeon first so fresh Materia checkouts can resolve its pin, then fast-forward Materia. The shared Materia checkout remains untouched; MT12–MT13 are the next handoff.

Controlled mill inertia attribution (2026-10-10):

- Main `27866c5a5` adds both resolved mass properties and the raw CAD inertia tensor to `AssemblyPreview.part`. The former preview omitted `inertia`, so `SceneArtifact.encode` used `MeshMassProperties.compute` on the display tessellation. Current exports use the CAD tensor. The tensors affect derived acceleration, reflected inertia and servo response; the controller tick, drive rates, gains and transmission stiffness were held fixed throughout these comparisons.
- Keep current geometry and exporter definition-sharing, but omit resolved mass/inertia fields and select density-derived mass: bare/enclosed remain **52.73/53.70 s** with current tracking. Exported mass values alone do not explain the shift (`mt-s-mill-mass-attribution.log`).
- Restore only `AssemblyPreview` to `81f5563f7`, retaining every other current file and the same current fixture: bare/enclosed become **52.55/53.74 s**. Bare X/Y/Z peaks return to **0.048472051067/0.051497020691/0.060011112459 mm** (`mt-s-mill-preview-attribution.log`). Adding just the component `massProperties()` evaluation to that former exporter reproduces those same values, ruling out the extra evaluation alone (`mt-s-mill-mass-evaluation.log`).
- Starting from the former exporter, add **only its raw CAD `inertia` field**: bare/enclosed return to **52.73/53.70 s** and bare X/Y/Z peaks to **0.034544382687/0.051463902185/0.059015341764 mm** (`mt-s-mill-cad-inertia.log`). Thus the isolated inertia effect is **+0.18 s bare / -0.04 s enclosed**, with the measured tracking change above. This replaces the earlier code-inspection-only attribution to unspecified mass/inertia handling. No old-format reader, simulation tuning or production fallback was added.
- Every focused mill run also checks the real brake, pneumatic timing, machining contacts and physical robot/CNC interface. All four diagnostic runs pass. The temporary producer changes are restored byte-for-byte to the committed production exporter; only this evidence ledger changes. The already-passing sequential gate remains the production validation, and MT12–MT13 remain after review and landing.
