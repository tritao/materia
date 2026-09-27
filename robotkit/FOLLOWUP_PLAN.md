# RobotKit construction follow-ups (implementation handoff)

These are the loose ends left after the construction roadmap
(`robotkit/CONSTRUCTION_ROADMAP.md`, M0–M12). Each item below is
self-contained: it states the problem, where the code is, what to change,
and how to prove it is fixed. Read `robotkit/ARCHITECTURE.md` and the
roadmap's "0. Read before writing code" section first; their conventions,
haxeon gotchas, git rules and test commands all still apply.

Ground rules (same as the roadmap):
- Work on `main` only if the owner says so; otherwise branch from it.
- One commit per item (more if large), each building and passing tests.
- Write the failing test first for every behavior fix.
- Stage explicit paths only; never stage build output; never push.
- Log each item in a new "Follow-ups" section at the end of the roadmap's
  Progress log.
- Do not spawn sub-agents.

Test commands (from the repo root):
- `./haxeon/scripts/haxeon run --project robotkit/tests/haxeon.json`
- `cmake -S robotkit -B robotkit/build -GNinja -DCMAKE_BUILD_TYPE=Debug && cmake --build robotkit/build -j 6 && ctest --test-dir robotkit/build --output-on-failure`
- CAD bridge and MuJoCo suites: see item F9, which folds them into one runner.
- SimKit (for F4/F6): `ctest --test-dir simkit/build -L sim --output-on-failure`
  and the MuJoCo native build per `robotkit/README.md`.

Suggested order: F9 first (so every later item runs all suites with one
command), then F1–F2 (correctness), then F3–F8, then F10.

---

## F1 — Review the three helper-agent commits

Commits `85cba1f9` (MuJoCo kinematic root without a free joint), `684b4deb`
(HolonomicDrive) and `64a7ec15` (MuJoCo full computed-torque control) were
written by agents that were told to research only. The batch agent says it
audited them, but no person has.

Do: read each diff in full against the M8.5/M9 sections of the roadmap and
write a short review note per commit in the Progress log: what it does,
whether it matches the plan, anything suspicious. Specifically check:
- `85cba1f9`: the kinematic root is pinned on every substep, and a root that
  is teleported mid-run carries its children (there is a test for this; confirm
  it asserts child poses, not just the root).
- `64a7ec15`: the torque is clamped to `max_force` per joint after the
  computed-torque term; `mj_mulM` is applied only over the robot's own DOFs;
  velocity-mode joints are not affected by the position gain.
- `684b4deb`: wheel geometry signs match the odometry (drive forward, rotate,
  and check odometry reports the same twist).
If a review finds a defect, fix it in a separate commit with a failing test.
Acceptance: three review notes in the log; any defects fixed with tests.

## F2 — Excavator overdig and bucket volume accounting

Symptom: the trench scenario removes 0.835 m³ for a 1.5 × 0.8 × 0.5 m
(0.6 m³) design trench, and the test only asserts "< 3×".

Code: `robotkit/haxe/robotkit/work/BucketSweep.hx` (capsule footprint that
lowers vertices, box/Voronoi-area volume estimate),
`robotkit/haxe/robotkit/work/DigCyclePlanner.hx`,
`robotkit/haxe/robotkit/skill/DigTrench.hx`, `ExcavatorTests.hx`.

Diagnose first, then fix:
1. Separate the two error sources with tests on `BucketSweep` alone:
   (a) volume *measurement* error — sweep a known rectangular path over a flat
   map and compare `removedVolume` with the exact geometric volume, and
   compare with `HeightMap` cut volume computed before/after;
   (b) real *overdig* — cells outside the design footprint or below design
   depth.
2. Measurement: compute removed volume as the exact difference of the
   height map's integrated volume before and after (reuse `HeightMap`'s
   cut/fill integration) instead of the per-vertex area approximation.
3. Overdig: clamp lowering so a sweep never cuts below the design surface
   (the `EarthworkRegion` design map) and never outside the region's
   footprint plus a configured tolerance; make `DigCyclePlanner` offset the
   cutting edge inward by half the bucket width at the trench walls.
Acceptance: `BucketSweep` volume matches the analytic volume within 2% on a
flat-map test; trench scenario removes volume within 10% of 0.6 m³ (the
tolerance band plus grid discretisation), no cell below design depth minus
grade tolerance, no cell outside the footprint plus half a cell lowered.
Tighten the scenario assertion from "< 3×" to those bounds.

## F3 — Sideways motion for the omnidirectional base

Symptom: `HolonomicDrive` sets vy = 0 because `Twist2`
(`robotkit/haxe/robotkit/mobile/Twist2.hx`) only has `linear` and `angular`.

Do:
- Add an optional `lateral:Float = 0.0` to `Twist2` (body-frame +Y), keeping
  the existing constructor signature compatible so no caller changes.
- `Pose2.integrate`/`integrateDisplacement`: integrate the lateral term
  (exact for constant body twist; add a helper rather than changing the
  existing arc semantics for lateral = 0).
- `HolonomicDrive`: use vy; `HolonomicOdometry`: recover vy from the wheels.
- `DifferentialDrive`/`AckermannDrive`: reject a non-zero lateral command
  explicitly (they can't execute it) instead of silently dropping it.
- The runtime base plant (`robotkit/haxe/robotkit/runtime/` — see the recent
  "exact twists" commits) must carry the lateral velocity.
- Navigator/GoTo stay unchanged (they produce lateral = 0). Optionally add a
  `StrafeTo` or use lateral in `WorkPatchPlanner` base moves — not required.
Tests: pure sideways twist moves the base along body +Y in simulation and
odometry agrees; diagonal twist; differential drive rejects lateral.

## F4 — MuJoCo self-collision between non-adjacent links

Symptom: `add_self_collision_excludes` in
`simkit/sim_mujoco/src/mujoco_backend.cpp` excludes every pair of bodies in a
robot's connected articulation, so an arm can pass through its own base.

Do: exclude only (a) parent–child pairs and (b) pairs whose geometries
overlap at the rest pose (computed once after the rest poses are built),
instead of whole components. Add a per-robot opt-out flag in the RobotKit
blueprint (`selfCollision: Bool`, default true) for models whose collision
shapes are too coarse, and document it.
Tests: a 3-link arm folded so link 3 would pass through link 1 is stopped by
contact in MuJoCo; adjacent links with overlapping shapes do not explode;
the existing M9 MuJoCo scenario still reaches ≥ 97% coverage (the arm fixture
may need its collision shapes set sensibly).

## F5 — Robot reset pose

Symptom noted in M8.5: reset returns every robot root to its spawn position
(`robot_index, 0, 0`) in `robotkit/runtime/src/simulation.cpp`
(`robot_initial_poses_`), with no way to choose it. Recent commits touched
this area, so first confirm current behavior with a test.

Do: add an optional initial pose to `Simulation.addRobot` (Haxe) and
`rk_simulation_add_robot` (C, via the `struct_size`-versioned descriptor
pattern), stored as the robot's reset pose; keep the current default. Do not
make teleport change the reset pose.
Tests: add a robot at (3, 2, 0) yaw 90°, move it, reset, and it returns to
(3, 2, 0) yaw 90° in both backends.

## F6 — Exact exclusion clipping in the raster generator

Symptom: `RasterToolpathGenerator` handles exclusions with a 1D stand-in
(footprint band per row, shrink interval ends touching an exclusion) instead
of offsetting polygons.

Do: implement proper polygon offsetting for the usable region: inset the
boundary by 0 (tool may run to the edge per the current policy), outset each
exclusion by `toolWidth/2`, then clip each scanline against
`boundary − ∪(outset exclusions)`. For the convex/rectilinear polygons BIM
produces, a Minkowski offset with mitred corners is enough; handle concave
exclusions by offsetting each edge and unioning (or restrict to convex and
split concave inputs, logging the choice).
Tests: existing ≥ 99% coverage / 0% exclusion test still passes; a rotated
(45°) exclusion and an L-shaped exclusion both get 0% process-on coverage
and ≥ 98% of the allowed area covered.

## F7 — Non-convex outlines in the CAD bridge

Symptom: `robotkit/cadbridge/haxe/cadbridge/*` orders wire vertices by angle
around the centroid, which is only correct for convex outlines.

Do: order vertices by following the wire's edges (walk edge endpoints from a
start vertex, using CadKit's edge/vertex subshapes), falling back to an
error rather than angle-sorting. Keep angle sorting nowhere.
Tests: a CadKit face with an L-shaped outer wire and a non-convex (L-shaped)
opening produce a `WorkSurface` whose boundary and exclusion match the
source loops (area and vertex order, CCW for outer, CW or normalised for
holes per `Polygon2` conventions).

## F8 — Adaptive sampling in CartesianTrajectory

Symptom: `CartesianTrajectory.build` samples every segment at a fixed
`sampleInterval`, so long segments produce very many samples and short ones
may under-sample rotation.

Do: sample by arc length and angle: the number of samples per segment is
`max(ceil(length / maxLinearStep), ceil(rotationAngle / maxAngularStep), 1)`,
with timing still from the trapezoidal profile. Keep `sampleInterval` as an
upper bound on time between samples for executors that need it.
Tests: a 20 m straight segment with a 5 cm step yields ~400 samples, not
`duration/sampleInterval`; a pure 90° reorientation with a 5° step yields
≥ 18 samples; the M4/M9 tests still pass.

## F9 — One command runs every RobotKit suite

Symptom: the CAD bridge tests need `LD_LIBRARY_PATH` pointed at
`cadkit/build/debug/core`, and the MuJoCo scenario lives in
`robotkit/tests/mujoco` because haxeon's `native.cmake` can't pass extra
CMake defines. Neither runs by default.

Do:
- Add `robotkit/tests/run-all.sh` that builds prerequisites if missing
  (CadKit core, the MuJoCo submodule), then runs: Haxe world tests, native
  ctest, cadbridge tests (setting the library path itself), the MuJoCo
  project (skipped with a clear message if MuJoCo is unavailable), and
  `world-tcp.sh` in both modes. Exit non-zero on any failure.
- Better fix for the library path if cheap: have the cadbridge haxeon project
  declare CadKit's native library so haxeon sets the runtime path (check how
  other projects depending on `cadkit` get `libcadkit-core.so` found; follow
  that pattern).
- Optionally, add a haxeon `native.cmake` field for extra CMake defines
  (e.g. `"defines": {"NKSIM_BUILD_MUJOCO": "ON"}`) in a separate haxeon
  commit with a haxeon test, then drop the `native-mujoco` wrapper directory.
- Document `run-all.sh` in `robotkit/README.md`.
Acceptance: `robotkit/tests/run-all.sh` passes on a clean checkout with
no environment variables set.

## F10 — Ignore generated directories

Untracked output keeps showing up in `git status`:
`cadkit/examples/modeling/build/`, `robotkit/worldd/build/`,
`simkit/.cache/`. Add them to the root `.gitignore` in the file's existing
style. Do **not** touch `exosuit/` — that is the owner's work in progress.
Acceptance: `git status` no longer lists those three.
