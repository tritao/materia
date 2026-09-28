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

## OpenCAMLib (C++, LGPL-2.1-or-later)

<https://github.com/aewallin/opencamlib>, read at 95b036f. Toolpath
generation (drop-cutter, push-cutter, waterline), not stock simulation. The
algorithms date from about 2010–2012 (Anders Wallin); the repository is in
maintenance mode (last push 2025-02, build and packaging only). Licence rules
out copying code: reimplement from the maths and credit OCL and Wallin's
offset-ellipse write-up. The formulas below come from reading the code and
must be confirmed by our own fixtures.

**Why it matters.** For a vertical ray at P, the lowest point of a tool's
sweep is `min_t [z(t) + h(|P − C(t)|)]`, where `h(r)` is the tool's lower
profile. OCL's edge drop-cutter computes `max_q [q_z − h(|q − CL|)]` over an
edge. Reflecting the path (negate z) turns one into the other:
`z_low(P) = −edgeDrop(CL = P, edge (A_xy, −A_z)→(B_xy, −B_z))`, clamped with
the endpoint values `z_A + h(|P − A|)` and `z_B + h(|P − B|)`. A ramp is a
sloped edge, so OCL's per-cutter edge solutions give exact ramp depths.

**Constant-height moves need none of it.** For horizontal lines and XY arcs,
`h` is monotone, so `z_low = z + h(xy distance from P to the path)`. Only
ramps and helices need per-cutter maths.

**Per-cutter edge contacts** (edge rotated to run along x at y = d; `R` tool
radius, `s = √max(0, R² − d²)`, edge `z = z₀ + m·x`, `m = tan α`):

- Flat (a ring): contact at `x = ±s`, the higher of the two.
- Ball: `cl_z = z₀ + s / cos α − R`.
- Cone (half-angle β, `c = cot β`, `H = R·c`): if `|m| ≤ c·s/R`, the contact
  is on the hyperbolic section, `x* = m·d / √(c² − m²)` and
  `cl_z = z₀ − d·√(c² − m²)`; otherwise it is on the rim at `x = sign(m)·s`,
  `cl_z = z(x) − H`.
- Bull-nose (flat radius `r1`, corner radius `r2`): the torus touches the
  line exactly when the ring of radius `r1` at height `cl_z + r2` touches the
  radius-`r2` cylinder around the edge. Slicing that cylinder at the ring
  height gives an ellipse (semi-axes `a = r2/|sin α|` along the edge and
  `b = r2` across it); the contact is the ellipse's offset curve at distance
  `r1` passing through the ring centre. Only the y equation needs solving:
  `d + b·t + r1·a·t / √(b²s² + a²t²) = 0` with `(s, t) = (cos θ, sin θ)`.
  It is odd and monotone in `t`, so each half-plane has one root; use a
  bracketed Brent solve, evaluate both mirror solutions, and keep the higher.
  A horizontal edge is the exact case `cl_z = z − h(d)`; near-horizontal
  edges make `a` blow up, so switch to the horizontal formula below a slope
  threshold.

**Closed forms for fixtures** (depth of the sweep's lowest point relative to
the path at its closest approach, `d` = XY distance to the path line): flat
`−s·|tan α|`; ball `R − s / cos α`; cone `d·√(cot²β − tan²α)` on the
hyperbolic branch, else `H − s·|tan α|`; bull-nose at `d = 0`
`r2 − r2 / cos α − r1·|tan α|`.

**Profiles as primitives.** Decompose `CutterProfile` into explicit rings (one
per profile vertex), cones or frusta (sloped lines), flat annuli (whose
contacts are always their rim rings), and tori or spheres (convex arcs). Keep
each primitive's optimum only when its contact radius lies strictly inside
that segment's band. OCL relies on implicit rim contacts, which loses
band-edge contacts for general stacks.

**Arcs and helices** are not in OCL. For a helix, `d²(θ) = ρ² + |P|² −
2ρ|P|·cos(θ − φ)` and `z(θ) = z₀ + kθ`: a ring gives at most two roots of
`d(θ) = r_v`; a flat end reaches its lowest point at a range end; spheres and
tori need a 1-D minimisation of `kθ + h(d(θ))`, split where `d` is monotone
(at `φ` and `φ + π`) and at the range limits.

**X and Y rays.** Push-cutter along a fibre is the horizontal-ray analogue:
push the reflected tool along the ray against the path edge. A straight move
of a convex tool sweeps a convex region, so each ray gets one interval; arcs
can give several.

**Bugs to avoid.** `ConeCutter::singleEdgeDropCanonical` uses the tool
length where the derivation needs the cone height `H`, which misplaces the
contact whenever the tool has a shank (and is a likely cause of OCL issue
#93). `ConeCutter::facetDrop` short-circuits and never tests the rim once the
tip matches. Tolerances are absolute (`1e-7`, `1e-10`, `±1e-6` for composite
bands), with no scale awareness; use scale-relative epsilons.

**No test suite.** CI only builds; the Python examples are visual. Our
closed forms above, plus dense-sampling comparisons for bull-nose ramps and
helices, become the fixtures.

**For future CamKit 3D toolpaths.** `BatchDropCutter` (kd-tree of triangles by
XY bounds, OpenMP), `AdaptivePathDropCutter` (midpoint subdivision until the
step is small and the path flat), and waterline (push-cutter fibres on an X/Y
grid, a Weave graph, loop extraction). The same drop and push primitives,
without the reflection, serve drop-cutter over a mesh.

**Order to implement in phase 3:** profile decomposition; constant-height
moves; ramps for rings, spheres, cones and tori with endpoint clamping;
helices; then X and Y rays.

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
