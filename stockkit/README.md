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
[`docs/PLAN.md`](docs/PLAN.md); notes on the open-source work this draws on
are in [`docs/REFERENCES.md`](docs/REFERENCES.md).

## Current state (phase 0)

- `stockkit.tool.CutterProfile` describes a tool as a surface of revolution
  from its tip upwards, with each segment marked as cutting, shank or holder.
  Flat, ball, bull-nose, V-bit and tapered-ball constructors are provided, and
  `withShank`/`withHolder` stack non-cutting sections on top.
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

Run the tests with the CadKit native build on `LD_LIBRARY_PATH`:

```sh
LD_LIBRARY_PATH=/path/to/cadkit/build/debug/core:/path/to/cadkit/build/debug/lin64/gcc/libd \
  ./haxeon/scripts/haxeon run --project stockkit/tests/haxeon.json
```
