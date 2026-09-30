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

**C2. The machine runs a program, without cutting.** A manifest key such as
`"cncProgram": "part.ngc"` compiles the G-code (`CncCompiler`), lowers it
against the assembly's `MachineBinding` (`ToolpathMotion.lower`), and installs
the timed x/y/z trajectories as the project's motion, the way the robot
arm's `robotMotions` do. The travel envelope comes from the joint limits, so
a program that leaves it is diagnosed before it runs. Motion tracks are piecewise
linear (at most 10,000 keys per track) and read in metres as offsets from the
starting pose, so the lowering samples the plan into keys and converts machine
millimetres; a long program may need a denser, streamed feed instead of tracks.

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

**C4. Machine a real part.** Program the part with CamKit from a MachineKit
part (for instance a NEMA motor mount plate) and compare the finished stock
with the CAD part: gouges and leftover coloured, removed volume reported.

**C5. Editor controls.** A G-code panel that highlights the running line and
links both ways (surface to line, line to moves); feed hold, resume, restart
from a line and a feed override, all of which the motion adapter already
supports.

**Later.** Turning lead screws coupled to the carriages; engagement and
cycle-time reports; rest machining from in-process stock; 5-axis or robot
milling (StockKit phase 7 and KinematicsKit); a lathe; streaming to a real
controller.

## Order

C0 and C1 are independent. After them, C2 → C3 → C4 → C5.

## Progress

| Step | State | Commits |
| --- | --- | --- |
| C0 | not started | |
| C1 | done: router, checks, outline-trace motion, app test | |
| C2 | not started | |
| C3 | not started | |
| C4 | not started | |
| C5 | not started | |
