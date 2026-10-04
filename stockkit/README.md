# StockKit

StockKit simulates material removal: it tracks the in-process stock as
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

## Current state (phases 0–5, phase 6 under way)

- Tool shapes live in ToolpathKit so any `Tool` can carry one:
  `toolpathkit.tool.CutterProfile` describes a tool as a surface of revolution from
  its tip upwards, with each segment marked as cutting, shank or holder. Flat,
  ball, bull-nose, V-bit and tapered-ball constructors are provided,
  `withShank`/`withHolder` stack non-cutting sections on top, and `below`
  clips a tool at the stock surface. `Tool.shaped` builds a tool from a
  profile; a diameter-only tool simulates as a flat mill.
- `stockkit.CutMove` is one tool motion through the stock (tool, motion in the
  workpiece frame, move kind, source operation, tool and provenance).
  `CutMoves.fromProgram(program, ?workOrigin)` builds them from a
  `ToolpathProgram` with its tool library and active setup position. CamKit
  returns that program directly. Toolpath geometry is in work coordinates and
  includes the active G43 tool length,
  which `ToolpathOp.ToolLengthOffset` records for recovering tool-tip positions.
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
  - The stock lives on a lattice (`sk_lattice`: origin, spacing, node
    counts in x, y and z, and which axes have rays). Each axis in it has a
    grid of rays through the nodes: Z rays through (x, y), X rays through
    (y, z), Y rays through (x, z). The Z grid is always there; with X and Y
    (tri-dexel) every move cuts all three. The Z grid measures each move's
    removed volume and contact; per-grid volumes are reported too.
  - Each grid is tiled 16×16 rays. Each tile stores its intervals field by
    field (depths, normals and sources in separate arrays) and keeps its
    furthest and nearest material along its rays, so a move whose sweep lies
    wholly beyond skips the tile. On Z rays a per-ray floor bound rejects
    rays whose material is already below the sweep before the exact query.
  - `SweptVolume` gives a move's bounds and the exact material it sweeps
    along a ray, for any surface-of-revolution profile. The profile is split
    into runs where the radius only widens or only narrows going up, so
    necked tools work. Horizontal lines and XY arcs use the closest point in
    closed form; ramps minimise a convex function (golden section, to
    floating-point resolution); helices and non-convex profiles scan and then
    refine each local minimum.
  - `SweptVolume` also answers X and Y rays (`sk_sweep_ray` with
    `SK_AXIS_X`, through (y, z), or `SK_AXIS_Y`, through (x, z)). At each height the tool is a disc, so a level move
    sweeps its path's 2D offset: a capsule for a line, an annular sector with
    end discs for an arc, whose crossings with the ray are found in closed
    form. A plunge sweeps its widest section over the heights it passes;
    ramps and helices scan and refine. Endpoint normals come from the profile
    at the endpoint's height.
  - Stock comes from a box or from a closed triangle mesh. Mesh casting uses
    exact orientation predicates and a tie rule, so a ray through a shared
    edge or vertex is counted exactly once; open meshes are refused.
  - Tools carry their whole profile with zones. Only the cutting zone, which
    must run from the tip, removes material. The shank and holder are swept
    as separate bands, and `sk_stock_cut` reports per move the volume removed
    and the stock each band overlaps after the move's own cut. Rapid moves
    through stock show as rapids that removed material. A climbing move whose
    shank meets material its flutes remove later in the same move is not
    seen; only upward moves can do that.
  - `sk_stock_compare` compares the stock with a target part cast on the same
    grid, ray by ray: leftover (stock outside the target), gouge (target
    missing from the stock), the largest stretch of each, and the move whose
    surface bounds the largest gouge, along any of the stock's grids. Z rays
    see floors and ceilings; X and Y rays see walls.
  - Cuts run on a pool of threads owned by the core (one per hardware thread
    by default; `sk_stock_set_threads` changes it). Each tile belongs to one
    thread (across all three grids), which applies every move of the batch
    to its tiles in order, and
    removed volumes are summed per tile in tile order. So the stock and the
    volumes are bit-identical for any thread count, which a native test
    checks. The core does not use NativeKit's task system, since it has no
    NativeKit dependency. An app can still run a long simulation as an
    `nk_task` that cuts a slice of moves per step.
- `stockkit.Stock` is the Haxe wrapper. It builds stock on a `StockLattice`
  (tri-dexel by default; `StockLattice.covering` puts nodes at cell centres
  with a cell to spare around the stock, so no ray lies in its faces) from a
  box, from triangles or from a CadKit mesh, reads rays of any grid
  (`StockGrid`, per `StockAxis`), and cuts `CutMove`s, one native call per
  run of moves with the same tool. It keeps the move history, so an
  interval end's source leads back to its `CutMove`, and from there to the
  op and `Provenance`. `cut` returns a `CutReport` with each move's
  `MoveOutcome`, rapid contacts, collisions and totals per operation.
  `compare` returns a `StockComparison` along one grid with leftover and
  gouge volumes, the deepest gouge and the moves that gouged;
  `compareAll` returns one per grid, so a wall gouged sideways (seen only
  by X or Y rays) is not missed.
- Preview and scrubbing (phase 5, headless):
  - `sk_stock_mesh` meshes a range of tiles. Each ray is a square column
    spacing wide with exact depths: top faces carry the stored normal and
    source move, walls stand where neighbours differ, and optional bottoms
    close the mesh (its volume is the stock's). Equal faces can be merged
    along rows. Streams come out as bytes ready for SceneKit (float
    positions and normals, uint32 indices, RGBA8 colours) plus each
    triangle's source move for picking. Colouring is by source through a
    palette (so by operation) or per ray (so by deviation from a target).
  - On a tri-dexel stock `SK_MESH_CONTOUR` (`Stock.contour` in Haxe)
    dual-contours the lattice instead, into a closed surface. Nodes are
    inside or outside by their Z rays; each edge between nodes of opposite
    sign lies on a ray of the grid along it, whose interval end gives the
    crossing's exact point, normal and source. Each cell gets the
    least-squares meeting point of its crossings' planes, clamped to the
    cell, so floors and walls stay flat and edges and corners stay sharp;
    each crossing edge becomes a quad with its crossing's normal and source.
    Curved surfaces are cut by chords (the ball slot in the core tests comes
    out 0.16% of its removed volume off at a sixth of the tool radius), and
    features thinner than a cell can be lost. Chunks own disjoint edges and
    join exactly. A Z tile's revision also changes when X or Y rays in its
    footprint do, so a finishing pass that moves a wall by less than a
    spacing still remeshes its chunk. `StockPreview` and the app use
    contoured meshes on tri-dexel stock.
  - `sk_mesh_color_by_deviation` colours a contoured mesh against a target
    surface by surface: each quad's surface is an exact interval end on its
    ray, and the ray's comparison with the target gives the gouge starting
    there on its empty side and the leftover ending there on its material
    side. A wall cut too deep shows on the X or Y rays crossing it, while
    the stock's other faces on those rays keep their own colour.
    `StockColoring.ByDeviation` uses it for contoured meshes.
    `sk_mesh_color_by_ray` takes a grid axis and colours the quads made from
    that grid's rays, for any other per-ray map.
  - Tiles are copy-on-write and carry revisions. `sk_stock_snapshot` and
    `sk_stock_restore` share tiles, so a snapshot costs a pointer per tile
    plus whatever later cuts replace.
  - In Haxe, `StockTimeline` scrubs a program (a snapshot every N moves;
    seeking restores the nearest one and cuts forward, history included),
    `StockPreview` keeps chunk meshes and rebuilds only chunks whose tiles
    changed, and `StockPreview.pick(chunk, triangle)` gives the `CutMove`
    under a picked triangle, hence its operation and `Provenance`.
  - The app's **Stock simulation** object shows a CamKit demo program cut
    with StockKit, with a timeline, colouring by operation or deviation, and
    click-to-G-code-line picking (see `app/README.md`). It meshes the whole
    stock into one geometry, which suits its small block; large stock would
    use `StockPreview` chunks as child nodes.
- CamKit's island-pocket test cuts its program with StockKit and requires no
  gouge, no rapid or shank through stock, and leftover only in the inside
  corners.
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
    tie rules, provenance, handle validation, shank and holder contact,
    target comparison, snapshots and revisions, closed preview meshes whose
    volume equals the stock's (also when built in chunks), colouring, and
    bit-identical results with 1, 2, 3, 8 and all threads. Contoured meshes
    must be closed and oriented; a box comes out exactly (every vertex on
    it, face normals only, volume to float precision, the same in chunks),
    flat and ball slots within their chord error, and a 0.1 mm wall shave
    that no Z ray sees moves the wall and renews the Z tiles along it.
    A slot too wide and too shallow colours its walls as gouged (seen only
    by Y rays), its floor as leftover and the box's sides on the same rays
    as on target, and a groove's gouge ends where the stock resumes.
  - Every oracle and chain fixture also contours the core stock on a
    60-cell-wide lattice and requires the enclosed volume to match the
    OCCT result within 1% of the removed volume.

For standalone C++ checks and benchmarks, build the core directly:

```sh
cmake -S stockkit/core -B build/stockkit-core -DCMAKE_BUILD_TYPE=Release
cmake --build build/stockkit-core
build/stockkit-core/stockkit_core_tests
build/stockkit-core/stockkit_core_bench        # optional ray spacing in mm, then "tri"
```

After changing `core/include/stockkit.h`, regenerate the Haxe binding with
`stockkit/core/tools/check-hxi.sh`. Variable-length results come back in one
call: the caller's capacity goes in through an in/out count, and a count too
small comes back as the size needed (haxeon's queried outputs, with an
initial capacity so small reads take one call). So `sk_stock_read_rays`
returns counts and intervals together, `sk_sweep_ray` answers in one call,
and `sk_stock_cut` returns per-move results and a summary.

Haxeon builds `stockkit_core` automatically from `stockkit/haxeon.json` when
StockKit, CamKit, or the app needs it. Haxeon also adds the built library to
the test runtime path, and builds CadKit's core the same way (`cadkit/haxeon.json`). CadKit finds its prebuilt OCCT
automatically (see the CadKit README), so the Haxe tests run with:

```sh
./haxeon/scripts/haxeon run --project stockkit/tests/haxeon.json
```
