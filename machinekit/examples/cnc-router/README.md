# Desktop CNC router

A three-axis gantry router generated from MachineKit parts, with a block of
aluminium stock clamped on its bed. From the repository root:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=./machinekit/examples/cnc-router/materia.project.json
```

Or open **Desktop CNC router** from the Start page and press **Play**: the
tool traces the outline of the stock, 10 mm outside it and above the clamps.
The router does not run a G-code program or cut anything yet;
[PLAN.md](PLAN.md) lays out the steps from here to cutting the stock in real
time.

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

`materia.project.json` names `cnc-router.motion.json` in `robotMotions`: one
looping track per axis, 20.4 s long, at 40 mm/s in X and Y and 20 mm/s in Z.
Like every Materia motion track, positions are offsets from the starting pose,
and for these prismatic joints they are in metres, so `-0.07` on `x` means
machine X 80. The simulation drives each axis to its track; without a track an
axis is not driven and the Z slide would fall under gravity.

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
project, builds it in MuJoCo and requires the tool to follow the shipped motion
to within 2 mm (it tracks to about 1.3 mm).
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
