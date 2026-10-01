# CNC router plan

Goal: a desktop CNC router example that runs a real machining program and
cuts its stock in the editor as the machine moves, in real time.

Most of the machining stack already exists: ToolpathKit (path format),
CncKit (G-code in and out), CamKit (2.5D CAM from CadKit faces), the
ToolpathKit motion adapter (program to MotionKit plans, feed hold and
restart) and StockKit (tri-dexel material removal, preview meshing,
snapshots, pick-to-G-code-line). What is missing is a machine to run it on
and the link between the machine's motion and the stock.

## Steps

**C0. Look at the stock viewer.** The editor's *Stock simulation* object has
only been tested headlessly (first item in `stockkit/TODO.md`). Look at it in
the running app and fix what looks wrong before building on it.

**C1. The router, geometry and joints.** `CncRouter` is a `MachineAssembly`:
a 4040 T-slot frame, MGN12 rails on every axis, a moving gantry, X and Z
carriages, NEMA 23 motors with static lead screws, a spindle with an end
mill, and a clamped stock block on a spoilboard. Its prismatic joints `x`,
`y` and `z` carry machine coordinates in millimetres, so they line up with
`MachineBinding`'s axis ids. Checks: joint travel, BOM, and the tool tip
following the joints exactly.

**C2. The machine runs a program, without cutting.** Done as A3 below: the
manifest's `cnc` block runs a G-code program on the machine through
`CncCompiler`, `ToolpathMotion` and MotionKit's trajectory execution. The
travel envelope comes from the joint limits, so a program that leaves it is
diagnosed before it runs.

**C3. Cutting in real time.**
- The stock sits on the table: the program's setup origin is placed on the
  machine's stock frame.
- The simulation clock drives the cut: each frame cuts the moves finished
  since the last one plus the current move up to the tool's position. This
  needs a way to cut a `CutMove` short at a path parameter.
- The display uses `StockPreview` chunks on child nodes, so a frame only
  re-uploads the chunks it changed (about 0.8 s to contour the whole 200 mm
  benchmark stock against 2–8 ms per dirty chunk).
- Cutting runs off the frame thread, a slice of moves per step, so a heavy
  move adds lag but never skips material.
- Diagnostics appear as they happen: rapids through stock, shank and holder
  contact, fixture hits.

**C4. Machine a real part** (done). Program the part with CamKit from a
MachineKit part (a NEMA 23 motor mount plate) and compare the finished stock
with the CAD part: gouges and leftover coloured, removed volume reported.
What it took, each as the long-term shape rather than a stopgap:

- *The job is generated.* The project's entrypoint models the plate, programs
  it with CamKit from its faces and puts the whole job in the scene artifact
  (format 11, a `machining` section): program, axes, spindle, work offset,
  tool table with cutter profiles, stock, sacrificial parts, tool part, the
  tool loaded at the start, and the finished part as the target. The
  manifest's `cnc` block (A3) is gone: it repeated the model's tool sizes and
  work offset by hand. The stock is simulated from the stock part's own mesh.
- *Programmed points.* A `Move`'s geometry is now the programmed point, as
  G-code writes it: work coordinates with the active G43 length removed.
  `ToolpathFrame` maps it to the machine's controlled point (work origin plus
  G43 length) for travel checks and motion, and to the physical tip (less the
  loaded tool's own length) for stock simulation and stock validation, so a
  wrong H number shows as a shifted cut. Before, Z carried the length and the
  writer, CAM and setup validation each disagreed about it.
- *Real tool lengths and tool changes.* Machine Z is the spindle's gauge line;
  CamJob emits G43 for measured tools and retracts so that neither tool's tip
  drops below safe Z across a change (the first tool is loaded where the job
  starts). The simulated operator answers each tool-change handshake: the
  stock cuts with the new tool and the tool part takes its shape.
- *Each pass compiles from the commanded position*, which the planner knows
  exactly. The 0.5 mm start mismatch seen when compiling from the measured
  position was the servo still settling at completion (136 µm, gone 25 ticks
  later), not gravity, so no gravity compensation was needed.
- *Found on the way:* MotionKit's path timing rejected long rapids (a 200 mm
  line sampled every 2 mm) because its retries re-rounded every stage without
  re-imposing the rest boundary; haxeon resolved `Module.SubType` against the
  wrong package; a target meshed at preview resolution read as gouge (its
  chords sit inside round walls), so the generator meshes it to 2 µm.

**C4 follow-ups: exact geometry end to end.** The plate's circles reach the
machine as 128-sided polygons, and their pocket offsets shrink the sides to
50 µm: the job is 3240 exact-stop moves, about 10 s and 3 GB to plan, and
most of a pass is stopping at vertices. Blending the corners (CamJob's blend
tolerance, `G64 P`) showed the real limits are underneath:

1. *MotionKit lowers paths to joint motion exactly.* `ProgramCompiler`
   samples a path every 2 mm and estimates joint velocity by finite
   differences, with zero acceleration; across fine polylines that drifts
   about 10 µm from the path and hides acceleration peaks from the timing.
   Primitives should give their exact tangent and curvature, joint
   derivatives should come from `KinematicsSolver.solveDifferential`
   (exact for a Cartesian machine), samples should sit on primitive
   boundaries, and knots at a curvature break need a second derivative on
   each side (a native path sample addition).
2. *CAM keeps arcs.* `CamContour` becomes lines and arcs (circular CAD edges
   by their curve kind, other curves as simplified polylines), offsets keep
   arcs concentric, and CamJob writes G2/G3: a circular pocket ring becomes
   one or two moves.
3. *Then* the router job blends only genuine corners; expect a few hundred
   moves and sub-second planning.

**C5. Editor controls.** A G-code panel that highlights the running line and
links both ways (surface to line, line to moves); feed hold, resume, restart
from a line and a feed override, all of which the motion adapter already
supports.

**Later.** Turning lead screws coupled to the carriages; engagement and
cycle-time reports; rest machining from in-process stock; 5-axis or robot
milling (StockKit phase 7 and KinematicsKit); a lathe; streaming to a real
controller.

## Architecture fixes found in C1

C1 needed two stopgaps; these replace them, before C2 builds on them.

- **A1. Joint overtravel** (done). Assembly joint limits carry a per-joint
  `overtravel`, through RobotKit's `JointLimits` to a new runtime blueprint
  array `joint_overtravel`. The runtime faults only outside the limits
  widened by it, and the simulation puts end stops there. The router's axes
  derive it from their rails, `LinearAxis` from its end margin; joints without
  one get 1 mm or 1 degree from the assembly bridge. Replaced the app-wide
  `AssemblyRobot.OBSERVED_LIMIT_TOLERANCE`. The fault it fixed: Z parked on
  its upper limit read noise-level positions just above it, and the runtime
  compared exactly against the limit, where MuJoCo's stop also sat.
- **A2. Designed contact** (done). simkit already excluded collision between
  parts of one machine that overlap in the starting pose, but it measured
  only boxes, spheres, capsules and cylinders, so a convex hull (every
  generated part) never matched. With hulls measured by their vertices'
  bounds, the rail blocks and the screws in their nut brackets slide freely,
  and the manifest's `collisionDisabledParts` is gone. No guide constraint was
  needed; on a rigid gantry it would only over-constrain the solver.
- **A3. Program-driven motion (the new C2)** (done). The manifest's `cnc`
  block names a G-code program, a G54 work offset and whether the job loops
  (C4 replaced the block with the job the project generates).
  The app compiles it with CncKit against the machine's x/y/z joints (with
  `MotionAxisBlueprint` offsets from the starting pose, since simulated joints
  read relative to it), lowers it with `ToolpathMotion`, and a
  `CncProgramPlayer` streams it through MotionKit's `ManipulatorMotion` as
  runtime trajectory segments. Assembly joint limits gained `acceleration`.
  The outline trace is `outline.ngc`; motion tracks are gone from the router.
  Found on the way: the simulated robot refused targets on fixed joints,
  which a plan commanding the whole robot sends, and `ToolpathMotionBinding`
  no longer compiled under the pinned haxeon (field null narrowing).
- **A4. Homing** (folded into A1). Parking Z on its upper soft limit is how
  LinuxCNC-style machines sit after homing; the switch (here, the end stop) is
  beyond it by the overtravel, so no artificial pull-off is modelled.
- **A6. One link per rigid body** (done). The runtime caps a trajectory
  submission at 4096 coefficients across all robot joints; with one joint per
  part the router's 44 joints left 15 segments a chunk, and streaming cost
  12 ms a tick with multi-second stalls. The simulation bridge now builds one
  link per rigid body (parts bolted together; every world-fixed part joins the
  root link), with combined mass properties and one convex hull per part in a
  new per-link hull list of the simulation ABI. The router simulates as four
  links and its three axes: 3.1 ms of compute per 10 ms tick (motion 0.4 ms,
  meshing 2.4 ms), a 0.76 s compile when the program starts and a 0.2 s hitch
  when a plan starts. Moving that compile and the stock meshing off the frame
  thread is what remains for smooth real time.
- **A5. Shared preview.** One `AssemblyPreview` helper builds the scene for
  every MachineKit example, keeping connectors when parts share geometry.

## Order

C0 and C1 are independent. Then A5, A1 with A4, A2, and A3 (which is C2),
then C3 → C4 → C5.

## Progress

| Step | State | Commits |
| --- | --- | --- |
| C0 | not started | |
| C1 | done: router, checks, outline-trace motion, app test | `c82fe76d` |
| C2 | done as A3 | |
| C3 | done: 3.1 ms of compute per 10 ms tick after A6 | `29b986cc` |
| C4 | done: CAM from the plate's faces, generated job, G43 tools and tool changes | |
| C5 | not started | |
