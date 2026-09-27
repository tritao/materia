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
- **LC-D4 — Configuration selection is our own dynamic programming.**
  - Choose one IK candidate per path sample, minimising a joint-motion cost
    with hard limits on joint jumps: a ladder graph over `sampleCandidates`,
    solved by DP in Haxe.
  - Descartes Light is not vendored, to avoid its ROS-flavoured dependency
    stack.

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
- Implement OPW (Brandstötter, Angerer, Hofbaur 2014) natively in
  `motionkit/native` from the paper. It is a short closed-form solver, so no
  third-party code is needed. Record the reference in `THIRD_PARTY.md` as
  algorithm provenance.
- The C ABI takes the 7 OPW parameters, joint offsets and sign corrections,
  and returns up to 8 solutions with validity flags.
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
  M9 6R arm, if it qualifies. If not, log why and add an ABB/KUKA-style
  parameter fixture.
- Parameter extraction rejects a non-spherical wrist.
- Near-singular wrist poses return solutions with a singularity flag.

## C5 — Configuration selection along a path (LC-D4)

Do:
- `motionkit.robot.PathConfigurationSelector`:
  - for each densified path sample, take `sampleCandidates` (OPW, or
    seeded numerical);
  - build the ladder graph, with edge cost as joint distance weighted by
    joint velocity limits, and edges exceeding the joint-jump bound removed;
  - DP for the minimum-cost continuous sequence;
  - no valid sequence gives a diagnostic naming the first unreachable or
    disconnected sample distance.
- Lane B's `ProgramCompiler` (B3) uses it when the solver provides several
  candidates. Coordinate through the `KinematicsSolver` interface only.

Tests:
- A path that the nearest-seed method takes through a wrist flip is solved
  without a flip.
- A path with no continuous solution fails with the right distance.
- It is deterministic.

## C6 — CncKit v1: declared G-code subset → `MotionProgram`

Do: a new haxeon project `cnckit/` (package `cnckit`) depending on
`motionkit` only. The CNC-to-robot binding lives in `motionkit.robot` or a
small `cnckit-robot` adapter if it needs RobotKit.
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
- **Machine binding:** a `CncMachine` description maps machine axes to
  `MotionSystem` axes, with rapid limits and work-offset storage. Probing is
  out of scope for v1.

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

Tests (a scenario test in `motionkit/tests` or a new `cnckit/tests`):
- the executed path stays within the declared tolerance of the programmed
  geometry at every recorded sample;
- the spindle events fire at the programmed positions;
- feed hold mid-arc stays on the arc;
- the run is deterministic.

## Out of scope for this lane

- Jerk-limited path timing and rolling contour lookahead;
- OMPL free-space planning and collision checking;
- `G18`/`G19`, cutter compensation (`G41`/`G42`), canned cycles, probing, CAM;
- external controllers (LinuxCNC, grblHAL backends);
- 7-axis redundancy (constrained differential IK, OSQP).

These are follow-on plans.

## Progress log

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
