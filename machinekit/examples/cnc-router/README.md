# Desktop CNC router

A three-axis gantry router generated from MachineKit parts, with a block of
aluminium stock clamped on its bed. From the repository root:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=./machinekit/examples/cnc-router/materia.project.json
```

Or open **Desktop CNC router** from the Start page and press **Play**: the
machine runs its G-code program and cuts three slots in the aluminium stock,
which you see disappear as the tool moves. [PLAN.md](PLAN.md) lays out what
comes next.

## What it contains

`CncRouter` is a `MachineAssembly` of 44 parts:

- a frame of `HFS5-4040` T-slot extrusion (two side members, three cross
  members) with a birch plywood spoilboard;
- MGN12 profile rails and blocks on every axis: two along Y on the side
  members, two along X on the gantry beams, two along Z on the X carriage;
- a gantry of two aluminium uprights and two 4040 beams, an X carriage plate,
  and a Z plate carrying a 52 mm ER11 spindle in a clamp, with a 6 mm flat end
  mill (22 mm flutes, 30 mm out of the collet);
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

Machine zero is the front-left corner of travel with Z at the top. The
assembly frame has the floor at z = 0 and the bed centred on the origin, so
the tool tip is at `CncRouter.toolTipAt(x, y, z)` = (x − 150, y − 150,
132 + z) mm. The starting pose puts the tool above the middle of the stock;
the stock's top is at 78 mm, the spoilboard's at 58 mm.

Every mate meets at the child's origin with world-aligned axes, so each
prismatic axis is a plain world direction. The mate connectors are named
after their members (`to-<child>`, `attach-<child>`), and parts that share a
designation share one definition carrying all of their connectors.

## Motion and simulation

`materia.project.json` gives the machine a CNC job in its `cnc` block:

```json
"cnc": {"program": "slots.ngc", "workOffset": [90, 105, -54], "loop": true,
        "stockPart": "stock", "toolPart": "tool",
        "tools": [{"number": 1, "diameter": 6, "fluteLength": 22, "length": 30,
                   "holderDiameter": 24, "holderLength": 20}]}
```

`slots.ngc` is LinuxCNC G-code: spindle on, then three 80 mm slots along X,
2 mm deep, clear of the step clamps, with rapids 5 mm above the stock. G54
work zero is the stock's front-left top corner, machine (90, 105, −54) mm.
The app compiles the program with CncKit against the machine's axes, lowers it
to MotionKit paths, and streams it to the simulated machine as trajectory
segments; the planner respects each axis's velocity and acceleration (500, 400
and 300 mm/s² for x, y and z). A program that leaves the travel, or fails to
compile, fails the simulation build with its G-code line.

The stock is cut as the machine moves (`MachiningStock`, StockKit): every tick
the segment the simulated tool tip travelled is swept through a 0.5 mm
tri-dexel stock with the tool table's cutter (flat end mill, shank and collet
nut), and the stock part shows the result, re-contoured a few times a second.
Because the cut follows the simulated tool, following error is in the
material. The whole run takes about 3 ms of compute per 10 ms of machining. Rapids that cut stock and shank or holder contact are counted as
they happen. Physical collision is off for the stock part: the stock
simulation, not the physics, decides what touching it means.

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
project, builds it in MuJoCo and runs the job: the stock must lose the three
slots' volume within 3% (3047.9 of 3049.6 mm³), no rapid may cut stock and the
holder must never touch it.
`CncRouterChecks` checks:

- the joints and their limits, and mass properties for every part;
- the tool tip at six machine positions (the corners of the travel box and
  points inside it), with the frame and the stock staying put;
- every rail block within its rail's usable length;
- interference between posed solids: the Z slide and spindle clear the stock
  when cutting through it (and the clamps and spoilboard at full depth), the
  gantry clears the frame and the Y motors at both ends of travel, the X
  carriage clears the uprights, beams and X motor, and the Z plate clears the
  Z motor bracket.
