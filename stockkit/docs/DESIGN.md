# StockKit design notes

Why StockKit is built the way it is, what was considered and rejected, and
ideas kept for later. `PLAN.md` holds the phases and targets;
`REFERENCES.md` holds detailed notes on code we read.

## How other simulators do it

| Approach | Example | Strength | Weakness |
|---|---|---|---|
| Implicit 3D field, marching cubes | CAMotics | General topology; stock can be queried and exported | Cost grows with volume; meshing rounds sharp edges |
| Height field (one Z per XY cell) | FreeCAD's legacy `VolSim` | Very cheap for 3-axis work | No undercuts, T-slots, side milling or 5-axis |
| Exact B-rep booleans per move | FreeCAD's old `Simulator.py` boolean mode | Exact CAD result | Slow and fragile over tens of thousands of moves |
| GPU depth/stencil CSG | FreeCAD's current `SimulatorGL` (and the OpenCSG library) | Instant scrubbing | Only an image; no stock to query, export or compare |
| Tri-dexel (rays along X, Y and Z) | Commercial simulators (as we understand it), rs_cam (Z only in practice), TRMachinist | Exact along rays; memory grows with area; fits 5-axis | Meshing is hard; the three grids must be kept consistent |

Ideas worth taking from FreeCAD:

- **Tool shape as a radial profile** taken from the tool's CAD, not a
  hard-coded list of tool types. We have `CutterProfile`; building it from a
  CadKit tool solid is future work.
- **Arbitrary starting stock** (castings, pre-machined blanks) from a CAD
  solid.
- **Separate stock and target models**, with the simulated result compared
  against the target.
- **Colour original stock and cut surfaces differently.** We generalise this
  to provenance on every interval endpoint: colour by operation, find which
  move cut a surface, and attribute gouges to moves.
- **Explicit sweep meshes for lines and arcs**, with a shear for sloped lines
  and a fallback to stamped tool instances when an arc's radius is below the
  tool radius. Relevant only if a GPU preview is ever built.
- **Quality settings**, but split in two: simulation tolerance (ray spacing,
  pose sampling) independent of display mesh quality.
- **A robust CAD-to-mesh path** (FreeCAD's `surface_generator.cpp`): preserve
  face winding, drop degenerate triangles, merge duplicate vertices, and also
  merge *duplicate facets* (coincident faces tessellate to identical
  triangles, which breaks topology code downstream).

What not to copy: FreeCAD writes motion out as G-code text and parses it
again inside the simulator. StockKit takes CNC ops and MotionKit motion
directly.

## Why tri-dexel rather than OpenVDB

OpenVDB (Apache-2.0) was the serious alternative. It provides sparse storage,
mesh-to-level-set conversion, CSG and meshing, and gives distance to a target
directly. We chose tri-dexel because:

- machined parts are mostly flat floors, vertical walls and sharp edges;
  dexels place those exactly along each ray, while a level set rounds every
  edge to about one voxel, which would look like gouges or leftover material
  exactly where we inspect;
- depth along a ray is exact at any ray spacing, while OpenVDB's accuracy
  needs voxels that small in all three directions (about 10⁹ voxels for a
  300 mm part at 0.05 mm);
- OpenVDB still leaves the hard part to us: building the swept volume and
  batching updates into changed regions;
- tri-dexel's weak point, meshing, becomes a known algorithm once each ray
  hit stores its surface normal (dual contouring with sharp features).

OpenVDB remains the fallback if tri-dexel meshing fails validation. It would
be the better choice only if Materia moves heavily into smooth freeform or
additive work.

## Libraries surveyed

| Library | Licence | Verdict |
|---|---|---|
| OCCT | LGPL-2.1 with exception | Already in CadKit. Used for stock and target input and as the exact test reference, never in the simulation loop. |
| OpenVDB | Apache-2.0 | Fallback representation (see above). |
| NanoVDB | Apache-2.0 | Only if we choose OpenVDB and want GPU queries or rendering. |
| Manifold | Apache-2.0 | Candidate faster exact-mesh reference if OCCT proves too slow or fragile; keeps face IDs through booleans. |
| OpenCAMLib | LGPL-2.1 | Its edge drop-cutter maths, reflected, gives exact ray depths for ramped moves, including the hard bull-nose case (offset ellipse). Reimplement from the maths for phase 3; later the basis for CamKit 3D toolpaths. See `REFERENCES.md`. |
| FCL | BSD | Largely redundant: RobotKit's `ToolCollisionShape` with SimKit/MuJoCo handles holder/spindle against machine and fixtures, and holder against the changing stock goes through the dexels. |
| Embree | Apache-2.0 | Overkill; a small BVH is enough to build dexels from stock meshes. |
| VTK Flying Edges, libfive, libigl | various | Read their algorithms, don't depend on them. libfive's mesher keeps sharp features. |
| OpenCSG, CGAL mesh booleans | GPL | Excluded: incompatible with the repo's Apache licensing. OpenCSG also only renders an image. |
| rs_cam | MIT | Z-only in practice. Worth porting: the tool-envelope lookup, per-tile air skipping, deterministic threading, its test style. See `REFERENCES.md`. |
| tridexel (Gruber) | BSL-1.0 | Don't adopt (primal EMC-style, indexing bugs). Lift its tables and grid-consistency rules only. See `REFERENCES.md`. |
| TRMachinist | Apache-2.0 | Alpha. Its `TRIPLE_DEXEL_ENGINE.md` is a design reference for target comparison. |
| Dual Contouring of Signed Distance Data (SIGGRAPH 2026) | MIT code | For signed-distance grids without normals. Not needed for dexels, which have exact normals; a quality benchmark for our mesher. |

To read before phase 6: Schleifstein et al., "Generalizing feature
preservation in iso-surface extraction from triple dexel models", CAD 177
(2024), on multiple sharp features per cell in CNC models; and Schaefer, Ju
and Warren (2007), manifold dual contouring.

## Architecture principles

- **The seams, not the storage, make it future-proof.** Moves reach the stock
  only through `SweptVolume` (bounds plus `intersectRay`). Tools are surfaces
  of revolution with cutting, shank and holder zones. Motion is in the
  workpiece frame. Adding tilted tools, robot machining or turning means new
  `SweptVolume` kinds, not a new stock model.
- **Tools are finite.** Shank and holder zones touching stock are collisions,
  checked against the live stock.
- **Diagnostics are live** (checked during simulation, not against a frozen
  snapshot) and must not depend on ray spacing. Always report the effective
  ray spacing, and never coarsen a grid silently.
- **Parallelism is per tile.** Expect memory bandwidth, not compute, to limit
  scaling (rs_cam's predicted 6–12× stopped at 2–3.4×).
- **The references must be independent of the simulator and fail loudly.**
  OCCT can return valid but wrong solids, so every reference result is
  checked against closed forms or bounds. A wrong reference must only ever
  cause a false alarm, never a false pass.

## Ideas kept for later

- **Target comparison as a first-class output:** leftover material (stock
  minus target) and gouges (target minus stock), per ray, as numbers and a
  colour map, attributed to operations.
- **Scrubbing:** copy-on-write tile snapshots every N operations; a GPU CSG
  preview only if playback is too slow, and only as a rendering layer.
- **Surface picking:** pick a stock surface, get its operation, then its
  `CncSpan`, then the existing editor highlighting.
- **Hybrid work:** adding material is an interval union, so additive steps
  fit the same stock.
- **Turning:** sweep in the rotating workpiece frame.
- **Stock and target from CAD:** triangles from CadKit tessellation, cast into
  dexels.
- **Export:** a watertight STL of the stock after phase 6.
- **Ball mill on a corner of its own radius:** check this against an
  analytic ray depth, since the OCCT reference refuses it. Alternatively,
  snap three-point arc endpoints exactly onto the revolution axis, which
  avoids OCCT's near-horn-torus bug.
