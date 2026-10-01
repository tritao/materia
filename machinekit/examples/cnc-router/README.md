# Desktop CNC router

A three-axis gantry router generated from MachineKit parts, with a block of
aluminium stock clamped on its bed. From the repository root:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=./machinekit/examples/cnc-router/materia.project.json
```

Or open **Desktop CNC router** from the Start page and press **Play**: the
machine runs the job it generates for itself, milling a NEMA 23 motor plate
out of the aluminium stock: it pockets the plate's recesses with its end mill,
changes to a drill and drills the screw holes through, and you see the stock
take the plate's shape as the tools move. [PLAN.md](PLAN.md) lays out what
comes next.

## What it contains

`CncRouter` is a `MachineAssembly` of 44 parts:

- a frame of `HFS5-4040` T-slot extrusion (two side members, three cross
  members) with a birch plywood spoilboard;
- MGN12 profile rails and blocks on every axis: two along Y on the side
  members, two along X on the gantry beams, two along Z on the X carriage;
- a gantry of two aluminium uprights and two 4040 beams, an X carriage plate,
  and a Z plate carrying a 52 mm ER11 spindle in a clamp, with a 6 mm flat end
  mill (tool 1: 22 mm flutes, 30 mm out of the collet) and, loaded by hand when
  a program calls for it, a 5.5 mm twist drill (tool 2: 118° point, 28 mm
  flutes, 40 mm out of the collet);
- four NEMA 23 steppers with Tr10×2 lead screws and nut brackets: two for Y,
  one each for X and Z;
- a 120 × 90 × 20 mm aluminium stock block held by two step clamps.

The screws and motors are placed as fixed parts: they do not turn with the
axes yet.

## Axes

The prismatic joints `x`, `y` and `z` read in machine coordinates, in
millimetres, which is what `toolpathkit.motion.MachineBinding` expects:

| Joint | Moves | Range (mm) | Start | Overtravel (mm) |
| ----- | ----- | ---------- | ----- | --------------- |
| `x`   | X carriage along the gantry | 0 to 300 | 150 | 62.65 |
| `y`   | gantry along the frame | 0 to 300 | 150 | 6.65 |
| `z`   | Z plate and spindle | −80 to 0 | 0 | 0.5 |

Machine zero is the front-left corner of travel with Z at the top, and
machine coordinates say where the spindle nose (the gauge line) is, as on a
real machine: each tool hangs its own length below it, which G43 applies. The
assembly frame has the floor at z = 0 and the bed centred on the origin, so
the nose is at `CncRouter.noseAt(x, y, z)` = (x − 150, y − 150, 162 + z) mm
and the end mill's tip 30 mm below that. The starting pose puts the spindle
above the middle of the stock; the stock's top is at 78 mm, the spoilboard's
at 58 mm.

Every mate meets at the child's origin with world-aligned axes, so each
prismatic axis is a plain world direction. The mate connectors are named
after their members (`to-<child>`, `attach-<child>`), and parts that share a
designation share one definition carrying all of their connectors.

## Machining job

The project generates its job along with the machine, so nothing about it is
written by hand. `CncRouterPreview.router` models the motor plate
(`NemaMountPlate`: a pilot recess, and on the motor's bolt pattern four
counterbores over 5.5 mm clearance holes), and `MountPlateJob` programs it
with CamKit from the plate's faces: the top face's inner boundaries are the
recesses' outlines, each pocketed down to the floor found under it with the
end mill, keeping clear of the step clamps; then the drill makes the four
holes from the counterbore floors through the plate and 0.5 mm past its point
into the spoilboard. Cutting lines that lie on a circle are fitted back into
arcs and the remaining corners blend within 0.01 mm, so the pockets are cut as
G2/G3 rings without stopping at every vertex. CncKit writes it as LinuxCNC
G-code, each tool programmed at its tip through `G43 Hn`. Work zero (G54) is the stock's front-left top
corner, machine (90, 105, −84) mm.

The scene artifact's machining section carries what the app needs to run it,
in metres: the program, the axes, the spindle part (whose origin is the gauge
line), the work offset, the tool table (each tool's length and its cutter
profile, the spindle's collet nut included), the stock, the spoilboard as a
sacrificial part, the part that shows the tool, the tool in the spindle at the
start (tool 1), and the finished plate as the target. The manifest says
nothing about it.

## Motion and simulation

The app compiles the program with CncKit against the machine's axes and tool
table, lowers it to MotionKit paths, and streams it to the simulated machine
as trajectory segments; the planner respects each axis's velocity and
acceleration (500, 400 and 300 mm/s² for x, y and z). A program that leaves
the travel, or fails to compile, fails the simulation build with its G-code
line. The job loops: each pass compiles the program again from the position
the planner last commanded.

A tool change in the program waits on its handshake, which the simulated
operator answers by loading that tool: the stock simulation cuts with it from
then on and the tool part takes its shape (the cutter profile, revolved).

The stock is cut as the machine moves (`MachiningStock`, StockKit): every tick
the segment the loaded tool's tip travelled, its length below the simulated
spindle nose, is swept through a 0.5 mm tri-dexel stock with the tool's
cutter (flutes, shank and collet nut), and the stock part shows the result,
re-contoured a few times a second. Because the cut follows the simulated
machine, following error is in the material: the servos lag slightly on the
pocket circles, which leaves a few hundredths of a millimetre on their walls. A
pass takes 198 s of machining at about 2 ms of compute per 10 ms tick; motion is
planned on a worker thread about a second ahead of the machine while it runs,
so the program starts at once. The stock is coloured against
the finished plate: green where it is on the part, yellow where stock is left
on it, red where the cut went into it. Rapids that cut stock and shank or
holder contact are counted as they happen. Physical collision is off for the
stock and the spoilboard: the stock simulation, not the physics, decides what
touching them means.

## Operating the job

The **CNC** panel (beside Sensors in the simulate layout) lists the program
with the executing line marked and followed, and operates the machine:

- **Hold** brings the machine to a controlled stop on its path; **Resume**
  carries on from there.
- **Restart at line** restarts at the line picked in the listing, or the
  first later line that moves: the machine stops, climbs to just below the top
  of Z, moves across, loads the tool that line needs, starts the spindle as the
  program had it, descends and carries on.
- **Speed override** scales every move from 5% to 200%. It takes effect from
  the next move without stopping: what is left of the program is planned again
  at the new speed, within the axes' limits.

Every part collides in the simulation, through its convex hull. Parts of the
machine that touch by design (a block on its rail, a nut bracket whose hull
swallows the screw through its bore) overlap in the starting pose, and the
MuJoCo backend never collides parts of one machine that overlap there, so they
slide freely without any per-project setting.

Each axis declares its overtravel: the room its rail blocks have left before
the rail ends at either end of travel, where the end stops are (see the table
above). The simulation puts the stops there and faults only past them. Z
starts at the top of its travel, on its upper limit, as a machine does after
homing, and reads numerical noise on both sides of it; without the half
millimetre of overtravel the first reading above the limit would fault the
machine.

## Checks

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/cnc-router/haxeon.json
./haxeon/.tools/hashlink/hl machinekit/examples/cnc-router/build/host/main.hl
```

These also run in the MachineKit smoke suite (`machinekit/scripts/test-haxeon`).
`ProjectSourceTests.checkCncRouter` in the app's project-source suite opens the
project, builds it in MuJoCo and runs one pass of the job: it must change
from the end mill to the drill, the stock must lose the plate's recesses and
holes within 2% (9914.9 of 9996.5 mm³, with 63.5 mm³ left on the plate),
under 1 mm³ may be cut from the finished plate (0.6 mm³), no rapid may cut
stock and the holder must never touch it; the next pass must then start
cleanly. `checkCncControls` operates the job through the CNC panel, laid out
headlessly and clicked: hold, resume, restart at a picked line, and a restart
at the drilling that must load the drill.
`CncRouterChecks` checks:

- the joints and their limits, and mass properties for every part;
- the spindle nose at six machine positions (the corners of the travel box
  and points inside it), the end mill's tip its length below, with the frame
  and the stock staying put;
- the tool table: the end mill and the drill, by their lengths and shapes;
- the motor plate's volume: the block less its recesses and holes;
- every rail block within its rail's usable length;
- interference between posed solids: the Z slide and spindle clear the stock
  when cutting through it (and the clamps and spoilboard at full depth), the
  gantry clears the frame and the Y motors at both ends of travel, the X
  carriage clears the uprights, beams and X motor, and the Z plate clears the
  Z motor bracket.
