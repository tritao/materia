# Lane C — Planning math: path timing, IK backends, CncKit (handoff)

**Goal:** replace the remaining Haxe path planner with native,
geometry-preserving path timing; give arms real IK backends with consistent
configuration choice along a path; and add CncKit, which compiles a declared
G-code subset into `MotionProgram`.

By the end of this lane, a virtual gantry runs a G-code program with lines
and arcs:
- timed by TOPP-RA to the joint limits;
- blended within stated tolerances;
- validated in joint and task space;
- executed as runtime plans.

Once Lane A's A6 is on `main`, it also runs through the virtual device.

Read first:
1. `motionkit/plans/CONTRACTS.md`, especially C2 (`MotionProgram`), C3
   (lowering and task-space validation), C4 (kinematics) and the cross-lane
   rules. §P0 must be on `main`.
2. `motionkit/IMPLEMENTATION_PLAN.md`: decisions D1–D4 (D1: numerical code is
   native), Ground rules, and the P3–P5 log (native trajectory, validation
   tolerance rules, how Ruckig was vendored and audited).
3. Code:
   - `motionkit/native/` (API in `include/motionkit.h`, `src/`, `tests/`,
     `THIRD_PARTY.md`)
   - `motionkit/haxe/motionkit/planner/LineLookaheadPlanner.hx`
   - `motionkit/haxe/motionkit/path/*`
   - `motionkit/robot/haxe/motionkit/robot/MotionSystem.hx` (`movePath`,
     `moveLinear`)
   - `robotkit/haxe/robotkit/manipulation/*`

Work in `../materia-lane-c` on branch `lane-c-planning`, from `main` after
§P0. Same rules as the other lanes: failing test first, one commit per item,
merge to `main` after each green item, log at the end, and stop if the plan
turns out to be wrong. Lane B needs **C2 on `main`** for its B8, so land C1
and C2 first.

## Lane decisions

- **LC-D1 — Vendor TOPP-RA's C++ implementation.**
  - Pinned submodule, MIT licence, with Eigen (MPL-2.0) as its dependency.
  - Use its built-in Seidel LP solver only, with no qpOASES or GLPK.
  - Audit the compiled files and record them in `THIRD_PARTY.md`, as was done
    for Ruckig.
  - **Fallback:** if the vendored build can't be made to work cleanly (it
    fights the build, pulls in unwanted dependencies, or ABI issues appear),
    stop and log it. Then implement TOPP-RA directly: reachability analysis
    with a 2-variable LP per stage, for velocity and acceleration constraints
    only. The algorithm is small and well documented.
- **LC-D2 — Path jerk is honestly unchecked.**
  - TOPP-RA gives piecewise-constant path acceleration, so joint jerk is not
    bounded at stage boundaries. Path plans report the jerk check as
    `unchecked` (never `passed`) unless a jerk-limited method produced them.
  - Jerk-limited path timing is a later item, out of scope here.
- **LC-D3 — Direct machines use identity kinematics.** For XYZ gantries, q(s)
  is the Cartesian path itself mapped through the axis transmissions. The
  same path timing serves gantries and arms.
- **LC-D4 — Vendor Descartes Light's core, lean, and build on its
  interfaces.**
  - Vendor `descartes_light` (Apache-2.0,
    https://github.com/swri-robotics/descartes_light) as a pinned submodule
    under `motionkit/native/vendor/`.
  - **Compatible:** our configuration selection uses Descartes' own types
    directly, not a parallel copy of them:
    - `State`;
    - the `WaypointSampler`, `EdgeEvaluator` and `StateEvaluator` interfaces;
    - `LadderGraphSolver`.

    Our OPW/IK sampler and our evaluators *are* Descartes subclasses. Its
    Boost-graph solvers, or a Tesseract pipeline, can then be dropped in
    later with no changes to callers.
  - **Lean:** compile only the `core` component's sources, with our own CMake
    target, and never run its CMakeLists:
    - no `ros_industrial_cmake_boilerplate`;
    - no BGL component or Boost;
    - no required OpenMP;
    - no `console_bridge` library.

    Two shim headers in `motionkit/native/vendor/shims/`, placed on the
    include path *after* real system headers, stand in for the missing
    pieces. **Vendored files are never modified.**
    - `omp.h` is an empty header, used only when CMake finds no OpenMP. The
      core uses `#pragma omp` and `#include <omp.h>` but calls no `omp_*`
      functions. Verify that on the pinned commit, and stop if it's no longer
      true.
    - `console_bridge/console.h` provides the `CONSOLE_BRIDGE_log*` macros,
      `getLogLevel()` and the level enum. It routes them to MotionKit's native
      diagnostic sink, or to nothing, and is covered by a small test.
  - **Threads and determinism:**
    - The selector runs `LadderGraphSolver` with `num_threads = 1` by default,
      so results are identical on every platform, with or without OpenMP.
    - More threads is an explicit option, enabled only where OpenMP is found.
      A test proves the chosen path is identical to the single-thread one.
  - **Build scope:** use only the `double` instantiations (the `…D` aliases),
    unless a float path is needed later.
  - **Record** the pinned commit, the Apache-2.0 licence and NOTICE, the
    compiled source list and both shims in `motionkit/native/THIRD_PARTY.md`.
  - If there is no network access, stop and log it. Do not copy it in by
    hand or reimplement it.
  - **Later:** enable its BGL (Boost) solvers only when a lazy-evaluation
    need appears. That means candidates per waypoint regularly above about
    64, or collision-checked edges. Record which one triggered it.

---

## C1 — Native `PathTrajectory` and lowering (C3 contract)

Do (`motionkit/native`):
- `mk_path` type: a joint path q(s) given as samples of s, q, q′ and q″ (or a
  C² spline through joint waypoints), plus a C ABI to build it.
- `mk_time_law` type: s(t) as piecewise-quadratic stages, with ṡ and s̈ per
  stage.
- `mk_path_lower(path, time_law, tolerance, out trajectory)`:
  - produces cubic or quintic Hermite segments on integer-nanosecond knots;
  - inserts knots adaptively until the maximum joint deviation from the exact
    q(s(t)) is below `tolerance`;
  - checks the deviation at segment midpoints and at interior extrema.
- `mk_path_distance_to_time(time_law, s)`: used for event lowering (C1).
- Haxe wrappers in `motionkit.trajectory` / `motionkit.path`.
- Bump `MK_API_VERSION` and regenerate the binding.

Tests:
- lowering a known analytic q(s(t)) (a circle at constant speed, and one
  with a speed ramp) stays within tolerance;
- the knot count shrinks as the tolerance loosens;
- a lowered trajectory passes `mk_validate` for position, velocity and
  acceleration;
- distance-to-time is monotonic and exact at stage boundaries.

## C2 — TOPP-RA path timing backend; replace `LineLookaheadPlanner`

Do:
- Vendor TOPP-RA (LC-D1).
- `mk_time_path(path, per-joint velocity and acceleration limits, optional
  per-segment path-speed caps (feed), start and end path speed, out
  time_law)`:
  - built on TOPP-RA's reachability pass;
  - speed caps express feed rates;
  - the result reports **which constraint is binding** per stage (joint index
    and velocity or acceleration, or feed cap), so diagnostics can explain
    slowdowns.
- A Haxe `ToppraPathTiming` class implementing
  `motionkit.planner.PathTimingBackend` (contract C3, landed in §P0). It
  returns the binding-constraint report in `TimedPath`.
- **`MotionSystem.movePath` / `moveLinear` / `queuePath`:** build q(s) from
  the `GeometricPath` through the axis mapping (LC-D3), time it with
  `ToppraPathTiming`, lower it (C1) and submit.
  - The result must stay on the authored lines and arcs within the path
    tolerance. The task-space slot is filled by sampling (C3); for identity
    kinematics, forward kinematics is the axis mapping.
  - Delete `LineLookaheadPlanner` and its tests. Re-express the behaviour
    tests: lookahead corner speed, arc centripetal budget, exact stop, long
    queue.

Tests:
- A square path in exact-stop mode stops at each corner.
- A circle's arc speed respects the per-joint centripetal acceleration
  limits. For equal independent X/Y acceleration limits `a`, its instantaneous
  bound varies with angle: `v²·|cos θ|/r ≤ a` and
  `v²·|sin θ|/r ≤ a`. Check each joint's acceleration and compare sampled
  speed with this angle-dependent bound; `√(a·r)` applies at axis-aligned
  points, not to the peak speed over the whole circle.
- Cycle time is within 1% of the old planner on analytically optimal straight
  exact-stop paths. Target no regression on curved paths; record comparative
  numbers in the log.
- The binding-constraint report names the right joint on a path that
  saturates one axis.
- Jerk is reported `unchecked` (LC-D2).

## C3 — Tolerance blending at corners

Do:
- `PathPlanningOptions.blend(tolerance)` becomes real. Replace each corner
  between consecutive primitives with a blend curve (a clothoid-free
  quintic, or a circular arc with tangent continuity), bounded by the
  tolerance.
- **Blends change geometry**, so the task-space check verifies that every
  point stays within `tolerance` of the authored corner polyline. The report
  records the tolerance used.
- Corners sharper than a configurable angle may fall back to exact stop,
  with a diagnostic.

Tests:
- a 90° corner blended at 0.5 mm stays within 0.5 mm and never stops;
- cycle time is shorter than exact stop;
- a near-reversal corner falls back to exact stop.

## C4 — OPW analytic IK backend (C4 contract)

Do:
- Vendor `opw_kinematics` (https://github.com/Jmeyer1292/opw_kinematics) as a
  pinned submodule under `motionkit/native/vendor/`.
  - It is header-only C++ (Apache-2.0) and depends only on Eigen, which C2
    already vendors.
  - Use only its core headers. Don't build its tests, ROS packaging or
    install targets.
  - Record the pinned tag or commit, the Apache-2.0 licence and NOTICE, and
    the audited header list in `motionkit/native/THIRD_PARTY.md`, as for
    Ruckig.
  - If there is no network access, stop and log it. Do not vendor by copy or
    reimplement.
- Wrap it behind our own C ABI, so nothing outside `motionkit/native`
  includes it. The ABI takes the 7 OPW parameters, joint offsets and sign
  corrections (`opw_kinematics::Parameters`). It returns up to 8 solutions,
  each with a validity flag and a singularity flag. The singularity flag is
  computed by our wrapper from wrist and shoulder conditioning, because the
  library doesn't report it.
- `motionkit.robot.OpwKinematics` implements `KinematicsSolver`:
  - it **extracts** OPW parameters from a `RobotModel` + TCP when the
    geometry qualifies: parallel base, spherical wrist, within a stated
    geometric tolerance;
  - otherwise it fails with a diagnostic naming the violating joint, and
    callers keep using `ManipulatorKinematics` (§P0).
- `sampleCandidates` returns all valid OPW solutions, wrapped into joint
  limits (±2π variants where the joint range allows) and filtered by limits.

Tests:
- Round trip: forward kinematics of random joint vectors, then OPW, contains
  the original within 1e-9.
- The solutions agree with `ManipulatorKinematics` forward kinematics on the
  M9 6R arm, if it qualifies. If not, log why.
- Use the library's published example parameter sets (ABB, KUKA, Fanuc,
  Stäubli) as fixtures: `RobotModel`s built from them extract back to the
  same parameters, and forward kinematics agrees with
  `ManipulatorKinematics`.
- Parameter extraction rejects a non-spherical wrist.
- Near-singular wrist poses return solutions with the wrapper's singularity flag set.

## C5 — Configuration selection along a path (LC-D4)

Do (`motionkit/native`, C ABI, Haxe wrapper
`motionkit.robot.PathConfigurationSelector`):
- **Vendor and build** Descartes' core per LC-D4: submodule, our CMake
  target, the `omp.h` and `console_bridge` shims, `THIRD_PARTY.md`.
- **Sampler:** a Descartes `WaypointSampler` subclass. It returns the
  candidates for one path sample, which the Haxe layer supplies in one bulk
  call per path (never one call per candidate). They come from OPW (C4)
  inside native code, or from `KinematicsSolver.sampleCandidates` for other
  solvers. The sampler filters candidates by joint limits.
- **Evaluators:**
  - Prefer Descartes' existing edge and state evaluators where they fit, and
    compose them.
  - Add our own `EdgeEvaluator` subclass only for what's missing: joint
    distance weighted by joint velocity limits, with any per-joint jump above
    the bound infeasible.
  - An optional `StateEvaluator` adds a preferred-posture cost.
- **Solver:** Descartes' `LadderGraphSolver`, single-threaded by default.
  - When no valid sequence exists, map Descartes' build and search failure
    information to a diagnostic naming the first unreachable or disconnected
    sample distance.
  - Route its log messages through the shim so they appear in that
    diagnostic, not on stdout.
- **C ABI:** `mk_select_configurations(candidate sets, joint limits, jump
  bounds, weights, threads, out sequence, out diagnostic)`. No Descartes
  types cross the ABI.
- Lane B's `ProgramCompiler` (B3) uses the selector when the solver provides
  several candidates. Coordinate through the `KinematicsSolver` interface
  only.

Tests:
- A path that the nearest-seed method takes through a wrist flip is solved
  without a flip.
- A path with no continuous solution fails with the right distance.
- It is deterministic, and single- and multi-threaded runs give identical
  output where OpenMP is available.
- It builds and runs with the empty `omp.h` shim (force the no-OpenMP
  configuration in one CI build).
- Log messages from Descartes are captured into the diagnostic.
- A 1000-sample path with 8 candidates per sample solves in under 50 ms in a
  Debug build. Record the time in the log.

## C6 — CncKit v1: declared G-code subset → `MotionProgram`

Do: a separate haxeon project for CNC interpretation. The CNC-to-robot
binding belongs in an adapter above the motion packages.
- **Declared subset.** Anything outside it is **rejected** with a line and
  column diagnostic, never approximated:
  - `G0`, `G1`, `G2`, `G3` (XY plane, `G17` only; `I`/`J` centre form; `R`
    form rejected in v1);
  - `G4` dwell;
  - `G20`/`G21`;
  - `G90`/`G91`;
  - `G54`–`G59` work offsets;
  - `G43 H` tool length offset, `G49`;
  - `F`, `S`;
  - `M3`/`M4`/`M5` spindle, `M7`/`M8`/`M9` coolant;
  - `M0`/`M1` (barrier), `M2`/`M30`;
  - `T` + `M6` (tool change as a barrier; the tool change itself is out of
    scope).
- **Modal state machine:** units, distance mode, current WCS, tool offset,
  feed, spindle, coolant.
- **Rapids:** `G0` compiles to a `FollowPath` on the straight line at machine
  rapid limits. `MoveJ` would not preserve the line, and a straight rapid is
  the conservative choice for gantries.
- **Feeds:** `G1`/`G2`/`G3` compile to `FollowPath` segments with feed caps.
  Consecutive moves in one block are blended per a program-level
  `ToleranceBlend` (like `G64 P`, supported), or run as exact stop (`G61`,
  supported).
- **Spindle and coolant:** `SetOutput` events on declared channels
  (`spindle.speed`, `spindle.direction`, `coolant.mist`, `coolant.flood`).
- **Machine binding:** `MachineBinding` maps physical axes to `MotionSystem`
  axes and supplies travel limits. `ToolpathProgram` carries setup positions;
  the CNC controller maps G54–G59 to those setups.

Tests:
- a small program (square pocket outline with arcs, `G54` offset, spindle
  on/off) compiles to the expected `MotionProgram` ops;
- unsupported codes (`G18`, `G2 R`, `G41`) are rejected with positions;
- modal persistence across lines;
- `G91` incremental arcs;
- feed and unit conversion.

## C7 — Virtual CNC end to end (demo acceptance)

Do:
- A MachineKit-compiled XYZ gantry (three `LinearAxis` via
  `MachineKitRobotCompiler`) runs a C6 test program through
  `MotionSystem` / program execution in simulation.
- Once Lane A's A6 is on `main`, run it again through the
  `VirtualDeviceEndpoint` with virtual steppers.
- Add feed hold mid-arc (it must stay on the arc) and a link-loss injection
  (device-side controlled stop, when on Lane A's path).
- **Once Plan C (editor assembly simulation) is on `main`:** also run the demo
  on the gantry's *physical assembly model* from `AssemblySimulationBridge`.
  - One link per part, with material-derived mass and couplings in the model
    (v5).
  - It is driven through actuators attached from the MachineKit motor parts.
  - Collision uses the upstream convex hulls.

  This is the intended single model (architecture invariant 1). Record any
  gap that prevents it (for example missing actuator attachment) in the log
  instead of working around it.

Tests (a scenario test in the motion or CNC test project):
- the executed path stays within the declared tolerance of the programmed
  geometry at every recorded sample;
- the spindle events fire at the programmed positions;
- feed hold mid-arc stays on the arc;
- the run is deterministic.

## Out of scope for this lane

- Jerk-limited path timing and rolling contour lookahead;
- OMPL free-space planning and collision checking (and so Descartes' BGL
  lazy solvers): now `kinematicskit/plans/COLLISION.md`, with our own
  RRT-Connect and OMPL as the reference (CL-D7);
- `G18`/`G19`, cutter compensation (`G41`/`G42`), canned cycles, probing, CAM;
- external controllers (LinuxCNC, grblHAL backends);
- 7-axis redundancy (constrained differential IK, OSQP) and live servoing:
  see `LANE_D_REDUNDANCY_SERVO.md` (mink-shaped QP IK).

These are follow-on plans.

## Progress log

### C6 — CNC G-code to MotionProgram

Added a strict G17 XY I/J CNC parser in the independent CNC project.
It tracks units, absolute or incremental coordinates, work and tool offsets,
feeds, spindle and coolant state, exact stops, and G64 P corner blending.
Unsupported words report line and column. The robot binding compiles the
result through `ProgramCompiler` using logical XYZ axes and their model-derived
transmissions and couplings. Initial output events attach to the first plan;
blended paths retain their original line and arc geometry for task-space
deviation checks. CncKit passed 20 assertions and the full MotionKit Haxe
suite passed 6,601 assertions.

### C7 — Virtual CNC acceptance

The MachineKit XYZ gantry runs the C6 rapid and arc program through
`ProgramCompiler` and the plan executor in deterministic simulation and over
RKD6 virtual steppers. The scenario checks every recorded position against
the authored rapid or arc within the declared 1 mm tolerance, spindle start
and stop events, feed hold on the arc, repeatable samples, and a device-side
controlled stop after an injected link loss. The focused C7 suite passed 2,845
assertions; the full MotionKit Haxe suite passed 9,446 assertions.

RKD6 needed two boundary fixes for this dense three-actuator stream:
continuation queue anchors now use the previous wire polynomial's f32 endpoint,
and a fresh program after an idle completion starts a new device epoch. The
first queue epoch reserves the serial time needed to deliver its initial
segments and commit before the device path clock reaches them. The simulation
API exposes a reversible link cut for the acceptance test (RobotKit ABI v21).

The physical assembly model remains a recorded gap. `AssemblySimulationBridge`
creates one mass-bearing link per part, v5 couplings, and upstream collision
hulls, but it creates no actuators. `MachineKitRobotCompiler.compileXYZGantry`
creates actuators on a separate simplified gantry model. There is no mapping
from MachineKit motor parts into the physical assembly model yet, so the CNC
program cannot drive that intended single model without bypassing the actuator
contract. That attachment belongs to the physical assembly compiler work.

### Physical assembly actuator attachment follow-up

`MachineKitRobotCompiler.compileAssemblyAxes` now binds named MachineKit motor
parts to their existing continuous shaft joints. It checks the corresponding
lead-screw joint coupling and derives logical axis maps from the part-level
`RobotModel`; it does not create another topology. The bridge passes upstream
convex hulls in link order to `Simulation.addRobotAtPose`. A three-axis
motor/coupling/carriage assembly fixture runs a G17 CNC program through
`ProgramCompiler` in deterministic simulation. Fixed mounting joints receive
stationary validation limits, and lead-screw joint jump checks use the axis
scale. Native TOPP-RA normalizes joint coordinates by their velocity limits
before solving its LP, keeping rotary and linear coefficients comparable.

The fixture has representative part mass and hull data. A complete MachineKit
gantry scene artifact with every physical part and upstream generated hulls is
still needed for final collision and MuJoCo acceptance.

### C1 — Native path representation and adaptive lowering

Added validated native C2 joint-path samples and continuous piecewise-quadratic
time laws. Distance-to-time inversion is exact at stage boundaries. Lowering
splits at path and time-law knots, fits quintic Hermite joint segments on
integer-nanosecond intervals, and bisects until the joint error at the
midpoint and every interior polynomial extremum meets the requested tolerance.
Added Haxe wrappers and regenerated the binding with `MK_API_VERSION` 7.
The native test failed before the API implementation and now covers a circle
at constant speed and under a ramp, adaptive knot counts, inverse timing,
and velocity and acceleration validation. Commit: the commit containing this
entry.

MotionKit passed 5,866 Haxe assertions; RobotKit passed 4,437 aggregate
Haxe assertions; all 13 tests in the combined native CTest build passed.
Both FFI audits passed, and TCP integration passed in default, session and
lease-timeout modes.

### C1 integration test follow-up — Accept sampled start position

After merging Lane B's first pose-path item into this branch, the TCP lease
test repeatedly failed to start its plan. The test captured a settled state
frame, then required the plan to start at exactly the captured position. A
subsequent owner cycle could move the position before plan acceptance. The
lease test now allows 0.02 joint units of start-position difference and keeps
zero velocity and acceleration tolerances; the unchanged test failed before
this adjustment and passed after it. Commit: the commit containing this entry.

### C2 planning correction — Exact-stop cycle-time comparison

C2's original strict `new cycle time ≤ old cycle time` test is unattainable
for a straight exact-stop path where the old trapezoid already reaches the
analytic minimum. On a two-leg 0.1 m square with 2 m/s² acceleration, the old
planner takes 0.894427191 s. The native TOPP-RA backend takes 0.896218108 s
under the same limits (0.20% longer), including the time stretch needed for
exact validation after nanosecond rounding. Changed only that acceptance bound
to allow 1% on analytic-optimal straight paths; curved paths retain a
no-regression target. Stopped C2 implementation here as required by the
handoff ground rule when a work item shows the plan is wrong. The uncommitted
C2 implementation in `materia-lane-c` is incomplete and still needs task-space
reporting, planner replacement tests, full-suite verification and a commit.

### C2 planning correction — Circle speed under independent joint limits

The original circle acceptance expects the peak speed to match `√(a·r)`
within 1%. That is a vector centripetal acceleration bound, but TOPP-RA's
contract uses independent per-joint acceleration limits. On a 0.1 m radius
circle with 1 m/s² limits on X and Y, the sampled peak is 0.333930896 m/s,
while `√(a·r)` is 0.316227766 m/s. Each joint's sampled acceleration stays
within 1 m/s²; the peak occurs away from an axis-aligned point, where the
normal acceleration is shared by the two joints. Changed the acceptance to
the per-joint, angle-dependent bound. Stopped C2 implementation here under
the handoff ground rule. The C2 changes remain uncommitted in the Lane C
worktree; the new test currently fails on the superseded peak-speed assertion.

### C2 — TOPP-RA timing and Cartesian path submission

Vendored the pinned TOPP-RA C++ Seidel solver, added the native reachability
timing entry point and per-stage binding-constraint report, and exposed it
through `ToppraPathTiming`. `MotionSystem` now times direct XYZ lines and arcs,
lowers them to native polynomial segments, checks joint limits exactly, and
records a sampled task-space deviation report at no worse than 1 ms spacing.
Path jerk remains unchecked. The old `LineLookaheadPlanner` was removed;
exact-stop corners, circle joint acceleration, straight-path timing, binding
joint, long queues, hold/resume and task-space reporting are exercised by the
MotionKit suite. Commit: the commit containing this entry.

For two 0.1 m straight exact-stop legs at 2 m/s², the old planner takes
0.894427191 s and TOPP-RA takes 0.896218108 s (0.20% longer). A standalone
0.1 m radius quarter arc at 0.5 m/s velocity and 1 m/s² per-joint
acceleration takes 0.811937932 s with TOPP-RA versus 0.966774462 s with
the old planner. MotionKit passed 5,410 Haxe assertions, RobotKit passed
4,437 world assertions, all 13 combined native CTests passed, both FFI audits
passed, and TCP integration passed in default, session and lease modes.

### C3 planning correction — mixed line and arc corners

The first C3 implementation blends planar line-to-line corners and passes the
90-degree, cycle-time, near-reversal, and runtime execution cases. It emits an
exact-stop diagnostic for a non-tangent line-to-arc or arc-to-line corner.
That is short of C3's requirement to replace **each** eligible corner. Added
a line-to-arc regression assertion to make this gap explicit. The correct
adjustment is a tangent-continuous fillet for mixed primitives, with distance
checked against the authored line and arc. C3 remains uncommitted and is not
ready to merge. Stopped here under the handoff ground rule rather than treat
the unsupported corner as a completed C3 item. The Haxeon interface-type fix
is committed locally in its submodule as `560dd069` and has focused tests.

### C3 — Tolerance blending at corners

Planar line-to-line corners use a circular tangent fillet. Mixed line/arc and
arc/arc corners use a quintic curve matched to endpoint tangent and curvature.
The mixed curve is accepted only when sampled deviation plus a half-sample
travel bound fits within the requested tolerance. Junction speeds account for
curvature within the whole primitive; TOPP-RA samples the quintic at 32 spans
before polynomial lowering. The runtime receives the resulting polynomial
segments with nonzero junction speeds. The authored path is checked in task
space at 1 ms or better, and the report records the requested tolerance.
Near-reversal corners keep an exact stop and diagnostic.

The first mixed-corner test exposed both an interface `Std.isOfType` error in
Haxeon and an invalid moving-boundary speed conversion in native TOPP-RA.
Haxeon now checks the wrapped concrete object for interface values, with a
focused test in its own submodule commit. The native timing code squares the
initial path speed as TOPP-RA expects, and preserves moving boundary speeds
when exact validation cannot accept a law. Commit: the commit containing this
entry. MotionKit passed 6,430 Haxe assertions; RobotKit passed 4,485 world
assertions. MotionKit's four native CTests and RobotKit's 13 native CTests,
both FFI audits, and TCP default, session and lease modes passed.
