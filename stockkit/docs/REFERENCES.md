# References

Notes from reading existing open-source work, with what StockKit takes from
each. Paths refer to the upstream repositories.

## rs_cam (Rust, MIT)

<https://github.com/RickyMillar/rs_cam>, `crates/rs_cam_core`.

Despite the tri-dexel name, production simulation is a Z-only multi-segment
height map. Cuts stamp 2D tool footprints with sub-cell coverage blending
(`ray_blend_above/below`); the X and Y grids are cut only by tools aligned with
them and are never reconciled with Z, and the project's own docs call them
dead code. No normals or provenance per interval, no stock from meshes, and
meshing is a bilinear height map with skirts that is watertight only for
single-segment rays.

Worth taking:

- Tool lower envelope as a lookup table indexed by squared distance (no square
  root), with at least 4096 samples; 256 overshot about 18 µm on small tips.
  Good for the Z fast path; other grids need exact profile/ray intersection.
- Interval subtraction (`ray_subtract_interval`), extended with endpoint normal
  and provenance.
- A monotone per-cell upper bound on material plus a per-tile max, used to skip
  moves that are entirely in air.
- Row-band (for us, tile) ownership, which keeps results bit-identical across
  thread counts.
- Test style: closed-form volume checks, determinism across thread counts, and
  regression tests named after the bug they guard.

Pitfalls they hit: coverage blending (the root of sliver, rib and blurry-wall
workarounds); approximate ramp depths patched by 0.02 mm subdivision; arcs
linearised at cell size; rapid checks against a frozen snapshot giving false
positives; diagnostic counts that changed with resolution; silent grid
coarsening at 16M cells; 24 bytes per ray holding 8 bytes of data; predicted
6–12× parallel speed-ups that stopped at 2–3.4× because the kernel is
bandwidth-bound (40–100 ns per cell visit).

## tridexel (C++, BSL-1.0)

<https://github.com/bernhardmgruber/tridexel>, a compact re-implementation of
Bernhard Gruber's master's thesis mesher
(<https://github.com/bernhardmgruber/master_thesis>) after Ren, Zhu and Lee,
"Feature conservation and conversion of tri-dexel volumetric models to
polyhedral surface models" (CAD&A, 2008).

The method is primal and extended-marching-cubes-like, not dual contouring:
corner occupancy is the union of the three grids, each cell edge is reduced to
at most one crossing by four regularisation rules, loops are found by a
depth-first walk over empty corners, and sharp edges come from intersecting
neighbouring tangent planes with the cell face. Output is an unindexed
triangle soup; neighbours may compute face vertices that differ in the last
bits, and face-ambiguous cases are not resolved consistently. The code has
bugs (occupancy indexing transposed relative to edges, an absolute rather than
relative snap distance, a negative-to-unsigned cast).

Decision: do not adopt the code. Implement manifold dual contouring (Schaefer,
Ju and Warren 2007) with a QEF solved by SVD over our stored endpoints and
normals, clamped to the cell. Lift, with attribution: the regularisation rules
as a documented fallback for inconsistent grids, the cell edge/neighbour
tables, three-plane intersection as a fast path for corners, and the
tangent-plane/cell-face construction for placing crossings.

Pitfalls named by the paper and thesis: regularisation deletes features that
do not cross a lattice point (thin slots, walls, ribs); rays nearly tangent to
flat stock faces create tiny alternating segments; adaptive refinement causes
cracks and T-junctions; feature points outside the cell must be clamped, not
dropped.

Still to read: Schleifstein, Lorenz, …, Kobbelt, "Generalizing feature
preservation in iso-surface extraction from triple dexel models", CAD 177
(2024) 103777, which targets several sharp features per cell in CNC models.

## Other

- TRMachinist (C#, Apache-2.0,
  <https://github.com/yasinisiktek-cloud/TRMachinist>): alpha; its
  `TRIPLE_DEXEL_ENGINE.md` is a design reference for target comparison and
  progress tracking. Not yet read in detail.
- Ju et al., "Dual contouring of Hermite data" (2002): the base algorithm for
  phase 6.
- Carrera et al., "Dual contouring of signed distance data" (SIGGRAPH 2026,
  MIT code): recovers sharp features without normals. Not needed for dexels,
  which have exact normals; a quality benchmark for our mesher and useful if
  signed-distance geometry appears elsewhere.
