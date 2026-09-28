# StockKit

StockKit will simulate material removal: it tracks the in-process stock as
tools move through it, reports collisions and rapid moves through material,
compares the result with a target part, and meshes the stock for display.
Coordinates are metres.

The design is a tri-dexel model: three grids of rays along X, Y and Z, each ray
a sorted list of material intervals whose endpoints keep their exact depth, a
surface normal, and the operation, tool and move that made them. Moves reach
the stock only through a swept-volume ray query, so 3-axis, 5-axis and robot
motion share one stock representation. The phase plan and targets are in
[`docs/PLAN.md`](docs/PLAN.md); why the design looks like this (comparison
with CAMotics and FreeCAD, the OpenVDB decision, libraries surveyed, ideas
kept for later) is in [`docs/DESIGN.md`](docs/DESIGN.md); notes on the
open-source code this draws on are in [`docs/REFERENCES.md`](docs/REFERENCES.md).

## Current state (phases 0–3)

- Tool shapes live in CncKit so any `CncTool` can carry one:
  `cnckit.tool.CutterProfile` describes a tool as a surface of revolution from
  its tip upwards, with each segment marked as cutting, shank or holder. Flat,
  ball, bull-nose, V-bit and tapered-ball constructors are provided,
  `withShank`/`withHolder` stack non-cutting sections on top, and `below`
  clips a tool at the stock surface. `CncTool.shaped` builds a tool from a
  profile; a diameter-only tool simulates as a flat mill.
- `stockkit.CutMove` is one tool motion through the stock (tool, motion in the
  workpiece frame, rapid or feed, source op index and span).
  `CutMoves.fromOps(ops, tools, ?workOrigin)` builds them from CamKit programs
  (`program.tool`) or compiled G-code (`machine.tool`). CNC op geometry is in
  machine coordinates and includes the active G43 tool length, which the new
  `CncOp.ToolLengthOffset` records, so the adapter recovers tool-tip positions.
- The test project holds the exact reference (`tests/src/oracle/ExactOracle.hx`):
  it builds each move's swept solid with OCCT through CadKit, subtracts it from
  stock, and reads exact material intervals along any ray. It covers
  horizontal lines, vertical lines and XY arcs, and rejects other moves rather
  than approximating them. Every swept solid is checked against its
  closed-form volume (section area times path length plus the tool), because
  OCCT can return a valid solid with the wrong volume; arcs long enough for
  their end caps to reach back into the sweep are refused, since that formula
  no longer holds for them. So are arcs whose radius a curved tool edge
  reaches (a ball mill on an arc of its own radius), whose revolved section is
  a horn torus that OCCT builds invalid at some positions.
- Stock is cut one move at a time. Each step must remove no more than that
  move's checked swept volume and never add material, so a boolean that goes
  wrong between moves fails at the move that caused it.
- Swept solids use half-tool end caps that meet the swept body on shared
  planar faces. Whole-tool caps touch a revolved body tangentially along a
  curve, and OCCT 8.0.1 fuses that wrongly in about 12% of axis-aligned arcs,
  sometimes silently (see `docs/PLAN.md`).
- `tests/src/fixtures/OracleFixtures.hx` checks the reference against
  closed-form volumes and ray depths for flat, ball, bull-nose and V-bit slots,
  a plunge, a ball-mill arc, and a CamKit pocket cut end to end.
  `tests/src/fixtures/ChainFixtures.hx` cuts tangent-joined chains (closed
  rounded rectangles and open S-curves) with four tool shapes, two corner
  radii and three stock positions, against the Pappus volume of the swept
  tube.

- **StockKit core** (`core/`, phase 3) is a C++17 library with a C ABI and
  no dependencies, following CadKit's layering: opaque generation-checked
  handles, bulk arrays, no exceptions across the boundary.
  - The stock is a Z grid of rays in 16×16-ray tiles. Each tile stores its
    intervals field by field (depths, normals and sources in separate arrays)
    and keeps the highest and lowest material in it, so a move whose sweep
    lies wholly above or below skips the tile. A per-ray floor bound rejects
    rays whose material is already below the sweep before the exact query.
  - `SweptVolume` gives a move's bounds and the exact material it sweeps
    along a ray, for any surface-of-revolution profile. The profile is split
    into runs where the radius only widens or only narrows going up, so
    necked tools work. Horizontal lines and XY arcs use the closest point in
    closed form; ramps minimise a convex function (golden section, to
    floating-point resolution); helices and non-convex profiles scan and then
    refine each local minimum.
  - Stock comes from a box or from a closed triangle mesh. Mesh casting uses
    exact orientation predicates and a tie rule, so a ray through a shared
    edge or vertex is counted exactly once; open meshes are refused.
  - Only a tool's cutting zone removes material. Shank and holder contact is
    phase 4.
  - `sk_stock_cut` returns the volume each move removed, which is how rapid
    moves through stock are found.
- `stockkit.Stock` is the Haxe wrapper. It builds stock from a box, from
  triangles or from a CadKit mesh, and cuts `CutMove`s, one native call per
  run of moves with the same tool. It keeps the move history, so an
  interval end's source leads back to its `CutMove`, and from there to the
  op and `CncSpan`.
- `tests/src/oracle/SampledReference.hx` is the second, independent
  reference, for moves the OCCT oracle refuses (ramps, helices). It samples
  tool poses and returns an inner set (the union of the exact sampled tools)
  and an outer set (each sampled tool grown by the distance the tool can move
  between samples, computed in closed form). The swept set lies between
  them. Checks are set containment, so a wrong reference can only raise a
  false alarm.
- Core checks:
  - Every ray of a 9-ray-wide grid in every oracle and chain fixture matches
    the OCCT oracle's stock to 1e-9 m.
  - Ramps (five tool shapes, plus a climbing ramp), helices and CamKit's
    ramped pocket must lie within the sampled reference's bounds.
  - A deliberate 0.1 µm error in the core fails both kinds of check.
  - `core/tests/core_tests.cpp` adds closed-form checks through the C ABI:
    slots, arcs, plunges, capsule floors of ball ramps, necked tools, mesh
    tie rules, provenance and handle validation.

Build the core and run its checks and benchmark:

```sh
cmake -S stockkit/core -B build/stockkit-core -DCMAKE_BUILD_TYPE=Release
cmake --build build/stockkit-core
build/stockkit-core/stockkit_core_tests
build/stockkit-core/stockkit_core_bench        # optional ray spacing in mm
```

After changing `core/include/stockkit.h`, regenerate the Haxe binding with
`stockkit/core/tools/check-hxi.sh`. Haxeon's FFI allows one output array per
function and no other outputs beside it, which is why counts and contents
are read with separate calls.

Run the Haxe tests with the StockKit core and CadKit native builds on
`LD_LIBRARY_PATH`:

```sh
LD_LIBRARY_PATH=build/stockkit-core:/path/to/cadkit/build/debug/core:/path/to/cadkit/build/debug/lin64/gcc/libd \
  ./haxeon/scripts/haxeon run --project stockkit/tests/haxeon.json
```
