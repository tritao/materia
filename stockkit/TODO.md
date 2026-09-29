# StockKit follow-ups

Phases and targets live in [`docs/PLAN.md`](docs/PLAN.md). Phase 6
(tri-dexel stock and meshing) is done apart from manifold contouring and STL
export; phase 7 (multi-axis) is next. These are the tasks around them.

## Next steps

Roughly in priority order.

- [ ] Look at the editor's Stock simulation object in the running app
  (lighting and sharp edges of the contoured mesh, colouring by operation and
  by deviation, the timeline, click-to-G-code-line) and fix what looks wrong;
  it has only been tested headlessly.
- [ ] Drive the viewer from real programs (loaded G-code, or CamKit and
  ToolpathKit programs) instead of the built-in demo, with a G-code panel that
  highlights both ways: surface to line, line to its moves.
- [ ] Show large stock as `StockPreview` chunks on child nodes, so playback
  re-uploads only the chunks that changed. Contouring the whole 200×200 mm
  benchmark stock takes about 0.8 s, a dirty 4×4-tile chunk 2–8 ms.
- [ ] Split `tests` (about 5 minutes) into a quick run (native checks, the
  sampled reference, a few oracle rays) and a thorough run (every OCCT
  comparison and contoured volume).
- [ ] STL export of the contoured stock (and of targets), reporting
  non-manifold edges until manifold contouring lands.
- [ ] Manifold dual contouring: one vertex per separate piece of surface in
  a cell, so features thinner than a cell no longer share a vertex.
- [ ] Phase 7: tilted-tool sweeps from sampled 6-DOF poses (MotionKit plus a
  kinematics adapter), so 5-axis and robot programs cut the same stock;
  holder and spindle against fixtures through RobotKit/SimKit collision. The
  OCCT oracle and the sampled reference cover +Z tools only, so this needs
  a reference too.
- [ ] Check shank and holder contact during a move, not only after its own
  cut: a climbing move whose shank meets material its flutes remove later is
  missed today.
- [ ] Report engagement per move (removal per unit length, radial engagement)
  as a basis for feed optimisation and warnings such as a full-width slot at
  finishing feed.
- [ ] Feed removed volume and cycle time per operation into CamKit reports and
  more CamKit tests than the island pocket.
- [ ] Move the cutter maths (profile pieces, drop- and push-cutter contact)
  into a layer shared with a future 3D CamKit (see `docs/DESIGN.md`).
- [ ] Let CAM read the in-process stock for rest machining and for skipping
  passes through air.
- [ ] Stock and fixtures from CAD (castings, pre-machined blanks) and targets
  straight from editor CAD parts.
- [ ] Persist simulation snapshots so a reopened scene need not re-simulate
  its program.
- [ ] Later: turning (sweep in the rotating workpiece frame), additive and
  hybrid steps (interval union), and a GPU preview only if CPU remeshing
  cannot keep up.

## Checks and measurements still missing

- [ ] The contour mesher's fallbacks when the grids disagree about a node
  (looking for a crossing up to half a spacing past its edge, and the node of
  margin when marking Z tiles over changed X and Y rays) have no test; none
  of the fixtures makes the grids disagree.
- [ ] Measure fine mode (0.1 mm rays) with tri-dexel and contouring against
  the milestone targets (under 10 s, under 500 MB); only Z-only fine mode
  has been measured.

## Haxeon

- [ ] Haxeon rejects a second `var` of the same name in one block
  (`E1001 Duplicate local`, `Scope.define`), which Haxe allows as shadowing.
  Supporting it touches how locals, captures and cells are keyed by name.
