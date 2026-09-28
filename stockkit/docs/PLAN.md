# StockKit plan

## Layering

```text
camkit ──┐
cnckit ──┼─► stockkit (Haxe API) ─► stockkit-core (C++, C ABI, no dependencies)
motionkit┘        │
                  └─► cadkit (stock/target tessellation; OCCT oracle in tests only)
```

`stockkit-core` follows CadKit's ABI rules: C only, opaque generation-checked
handles, bulk buffers, no exceptions across the boundary. Stock and target
arrive as triangle buffers, so the core never links OCCT.

## Decisions

- **Representation:** tri-dexel. Rays are exact along their axis, memory grows
  with the stock's area rather than its volume, updates parallelise per tile,
  and interval endpoints carry provenance for free. Sparse signed-distance
  volumes (OpenVDB) stay a fallback if tri-dexel meshing fails validation.
- **One canonical backend.** No parallel dexel, voxel and GPU backends; a GPU
  preview is at most a rendering optimisation later.
- **Ray direction is a parameter from day one.** Tiles, the swept-volume query,
  snapshots and the ABI are written for three grids; milestone 1 instantiates
  only Z.
- **Exact point samples per ray.** No area-coverage blending: it makes rays
  approximate, stamps order-dependent and walls blurry (rs_cam's main source
  of workarounds).

## Phases

0. **Groundwork** (done): references read, tool profile type, exact OCCT
   oracle, fixture set.
1. **Tool model** (done): `CutterProfile` moved to `cnckit.tool` and carried
   by `CncTool`; a diameter-only tool simulates as a flat mill. Profiles from
   CadKit solids later.
2. **Motion input** (done for 3-axis): `CutMove` = tool + motion in the
   workpiece frame + provenance (op index, `CncSpan`); `CutMoves.fromOps`
   adapts CamKit programs and compiled G-code, using the new
   `CncOp.ToolLengthOffset` to recover tool tips. Motion is an analytic path
   with a +Z tool axis; sampled 6-DOF poses arrive with phase 7. Work offsets
   are a translation for now.
3. **Stock core, Z grid:** tiled rays of sorted intervals with endpoint normal
   and provenance, structure-of-arrays storage; `SweptVolume` with bounds and
   exact `intersectRay` for revolution tools on lines, arcs and helices; stock
   from box or triangle mesh; bulk submission through the C ABI; per-tile
   material bounds to skip air; tile ownership for deterministic threads.
4. **Diagnostics:** live rapid-into-stock, shank/holder engagement against the
   live stock, removed volume per operation, target comparison per ray with
   gouge/leftover attribution. CamKit tests adopt it.
5. **Preview and editor** (milestone 1): Z-grid mesh, SceneKit colouring by
   operation/deviation, copy-on-write tile snapshots for scrubbing, surface
   pick → operation → `CncSpan`.
6. **Tri-dexel and meshing** (milestone 2): X and Y grids updated by every
   move; manifold dual contouring with a QEF over stored normals; STL export.
7. **Multi-axis** (milestone 3): tilted-tool sweeps, MotionKit + kinematics
   adapter, holder/spindle against fixtures via RobotKit/SimKit collision.

## Milestone 1 targets

| Measure | Target |
|---|---|
| 50k moves, 200×200 mm stock, 0.25 mm rays | < 2 s single-threaded, < 0.5 s threaded |
| Scrub to any timeline point | < 100 ms |
| Remesh dirty tiles during playback | < 16 ms per frame |
| Depth along rays | exact to floating point against the oracle |
| Fine mode, 0.1 mm rays | < 10 s, < 500 MB |

The effective ray spacing is always reported; the core never coarsens a grid
silently.

## Oracle coverage

The exact oracle handles horizontal lines, vertical lines and XY arcs, which
covered all CamKit output until CamKit gained ramped pocket entries (lines
that move in XY and Z together). Ramps, helices and tilted tools need a
different reference before the simulator's ramp handling can be checked
exactly; until then the CamKit fixture uses a depth at which CamKit plunges.

### OCCT tangent-fuse findings (OCCT 8.0.1)

Fusing a revolved ball-mill section with whole tools at its ends, where the
tool spheres are tangent to the swept torus along a circle:

- fails in 53 of 448 axis-aligned arcs (quarter and half circles, four start
  angles, seven centres, units from metres to millimetres), and 6 of those
  failures are valid single solids about 17% too small;
- depends on position and scale unpredictably (metres at the origin work,
  metres 20 mm along X fail; millimetres fail at other positions);
- is unaffected by fuzzy values from 1e-9 to 1e-6 of the model scale, and by
  fusing all arguments at once instead of pairwise;
- does not happen with half-tool caps: 0 of 448 structured and 0 of 300
  random arcs failed.

OCCT's "unused faces/edges" builder warnings flagged all 111 bad fuses in one
720-case scan and none of the good ones, but the same warnings appear on
correct results elsewhere (a CadKit feature cut, StockKit's half-cap fuses),
so they cannot mark failure in general. CadKit now rejects boolean results
whose topology is invalid; valid-but-wrong results can only be caught by an
independent measure such as the oracle's closed-form volume check. The
standalone reproduction is a candidate upstream report.

A second case surfaced in the tangent-chain scan: a ball or bull-nose section
on an arc of the tool's own radius meets the revolution axis tangentially and
revolves into a horn torus. OCCT built it invalid at one of three stock
positions (two of four quarter arcs there). CadKit's revolve now rejects
invalid results, and the oracle refuses that configuration; flat mills and
V-bits meet the axis at a corner and revolve reliably at the same radius.
