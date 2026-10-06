# Process path planning plan

Status: planned (2026-10-05). Runs on branch `gantries` in `/home/joao/dev/materia-worktrees/gantries`,
after the gantries plan (`machinekit/GANTRY_PLAN.md`). Same worktree, same Codex session, and its
benchmarks (G17 track weld, gantry welder) are the starting point.

**Goal.** Plan long process paths (welds, surface passes, arm toolpaths, arm + track or positioner) in
seconds instead of minutes. Decide the discrete choices once, globally:
- IK branch;
- how much the external axes move;
- tool roll where the process leaves it free.

Then make the path smooth, time it once, and check collisions only where the chosen path goes.

Target: the G17 2.6 m track weld plans in under 15 s, down from about 306 s. The robot-welder and
gantry-welder whole-weldment missions plan at least 10× faster with the same weld quality.

## Why the current planner is slow (survey 2026-10-05, at `878931b06`)

Every arm process plan in production goes through numeric IK in `ManipulatorKinematics`. Descartes
and OPW are built and tested, but no production code uses them.

### Welds

`processkit/haxe/processkit/WeldPathPlanner.hx` + `WeldingPlanRunner.hx`.

**Search structure.**
- The plan, then its rotated closed runs, then the reversed plan (`plan` 157-206).
- For each, 8 fixed rolls at π/4 (`ROLLS` 115).
- For each roll:
  - 6 entry "ways" (`ways` 380-389);
  - 12 numeric entry goals (`sampleCandidates`) plus a continued solve (342-352);
  - corner styles (`follow` 263-288);
  - a budget of 1024 segment tries per entry (`BUDGET` 108).

**Per candidate.**
- The seam is sampled every 2 mm (`STEP` 100).
- `chain` (473-512) does sequential, seed-chained numeric IK per sample. Each result gets a jump check
  and an `ArmClearance.violation`.
- `verify` (291-324) then compiles the whole program (`ProgramCompiler.compile`: IK again inside
  `lowerPath`, differential IK for dq/ds, TOPP-RA) and checks the compiled trajectories for clearance.
- `launch()` compiles once more for the rate schedule (`WeldingPlanRunner` 358).

**Inside the compiler.**
- A plain 6R arm solves point by point.
- With external axes, `RedundancyResolver` runs a beam search. It does (1+2d)·B numeric IK solves per
  sample (up to 300 iterations each, B ≤ 48), plus B² edges and a smoothing re-solve.
- For the 2.6 m track weld that is about 190k IK solves per compile.

**Profile** (`machinekit/GANTRY_PLAN.md` 3851-3998).

| Run | Planning | Checked poses | Notes |
|---|---|---|---|
| G17 track weld, before clearance broad phase | 689.9 s | — | |
| G17 track weld, with broad phase | 385.4 s | — | |
| G17 track weld, with snapshot cache | 306.2 s | 224,878 | current |

- In one run, 49 candidates compiled and were rejected. **All 49 failed in the approach op**:
  - 26 joint-limit overshoots;
  - 17 IK-continuity failures;
  - 6 task-space failures.
- None failed on clearance.
- The time goes into building plans that are thrown away, and into numeric IK.
- Whole-weldment runs take 117–450 s for their worst run (robot welder) and up to 437 s (gantry
  welder).

### Other runners

- `SurfacePlanRunner`: one numeric solve, then `FollowPath`, solved point by point or through
  `RedundancyResolver`.
- `ToolpathPlanRunner`: numeric solve per waypoint, emitting MoveJ per point, with no path IK.
- `HandlingPlanRunner`: MoveL through `lowerPath`.

### What already exists and is unused

**Descartes ladder graph, native.**
- `motionkit/native/src/configuration.cpp`:
  - `mk_select_configurations` takes caller-supplied candidates;
  - `mk_select_opw_configurations` handles 6R arms (8 OPW branches × ±2π wraps, capped at 64 per
    sample, sample 0 pinned).
- Edge cost: Σ wᵢ|Δqᵢ|, infeasible above `max_jump`. Optional preferred-posture state cost.
- No free axes, external axes, collision or derivative limits.
- Haxe wrapper: `PathConfigurationSelector`.
- `ProgramCompiler`'s 14th constructor argument accepts it, but no production caller passes one.

**OPW analytic IK.** `OpwKinematics` (native `mk_opw_inverse`).
- Parameters are derived from the model and checked by FK.
- Tested on the ABB IRB2400, KUKA KR6, Fanuc R2000 and Stäubli TX40.
- It rejects UR-type offset wrists and any prismatic joint.
- `machinekit` `RobotArm` (used by both welders and the track arm) is documented as spherical-wrist and
  is probably OPW-compatible, but it has never been tested.
- `CobotArm` is UR-type, and there is no closed-form UR IK anywhere.

**Timing.**
- `mk_time_path` / `ToppraPathTiming` already accepts a fixed joint path:
  `JointPathSamples(s, q, q', q'', ?before)`. There is no IK inside timing.
- `ProgramCompiler` still checks the compiled plan against the Cartesian path at 1 ms
  (`checkTaskSpace`).

**Collision.**
- `ArmClearance` is pure Haxe: convex hull corner sets, a sphere/AABB broad phase, and GJK
  (`ConvexDistance`). It returns the first violating pair.
- coal is vendored in kinematicskit but built only for a smoke test.
- There is no native distance API.

**Tool freedom (G2).**
- `OrientationPolicy` is set per path primitive. `FreeAboutTool` frees spin about TCP +Z
  (`ToolFreedom`).
- A roll sampler enumerates `target.rotation * Quat(Z, roll)`; `WeldPathPlanner` 241-243 already does
  this.

## Decisions

**PP-D1. Geometry first, timing once.**
- The planner produces a complete, limit-respecting, continuous joint path before anything is timed.
- TOPP-RA runs once, on the accepted path, so it can only fail on dynamics.
- **Rejected:** compiling and timing candidates to find out whether they are geometrically feasible.

**PP-D2. Global choice over a lattice, continuous refinement after.**
- A ladder graph over sampled candidates decides the discrete structure: IK branch, external-axis
  usage, roll.
- Smoothing the redundant coordinates plus an exact analytic re-solve makes the result continuous.
- **Rejected for process paths:**
  - numeric beam search (`RedundancyResolver`);
  - per-candidate generate-and-test (the weld planner's roll × entry × corner loops).

**PP-D3. Analytic IK wherever the mechanism allows, derived from the model.**
- Analytic solvers return *all* branches. Supported families:
  - OPW 6R (spherical wrist);
  - UR-type 6R (three parallel axes, offset wrist);
  - Cartesian machines and their rotary heads (XYZ, XYZ+C, XYZ+C+A).
- Parameters are always extracted from the `RobotModel` and verified by FK, never typed in.
- Other arms use multi-seed numeric IK as a slower fallback. It is flagged in diagnostics.

**PP-D4. External axes are part of the graph, not a second solver.**
- A group splits into an external chain (track, gantry axes, positioner) and an arm sub-chain whose
  base pose comes from the external values (G3 derives the split).
- Candidates are external-lattice values × free-axis values × arm branches.
- **"Rule" presets** are a degenerate lattice with one value per sample. Examples:
  - "track follows the TCP at a fixed offset";
  - "positioner holds".

  They are the cheap industrial default, and the full lattice is used when a rule fails.

**PP-D5. Collision lazily, on the winning path.**
- Search ignores collision.
- The chosen path's samples and joint-space edges are checked. Colliding candidates or edges are
  removed and the search repeats, up to a bound.
- One final check runs on the compiled trajectory.
- **Rejected:** checking every candidate. Most are never on the winning path.

**PP-D6. Native core, Haxe orchestration.**
- Candidate generation, graph build, search and refinement are C++ in MotionKit's native library,
  next to `configuration.cpp`.
- Haxe passes the problem in and gets a joint path plus diagnostics back.
- Collision checks start in Haxe (`ArmClearance`), because lazy checking makes them rare. They move
  native (coal) only if profiling says so (PP10).

**PP-D7. Deterministic.** The same inputs give the same path: single-threaded by default, with stable
candidate ordering and tie-breaks.

**PP-D8. The process states preferences; the planner knows no process.**
- Welding, painting and handling supply:
  - tool freedom per sample;
  - a preferred roll and the cost of roll changes;
  - corner-continuation hints;
  - posture and limit-margin weights;
  - external-axis cost.
- The planner never imports ProcessKit.

**PP-D9. No legacy paths.** When a runner moves to the new planner, its old search code is deleted in
the same step. `RedundancyResolver` and `PathConfigurationSelector` are deleted once nothing uses them
(repo policy: no compatibility code).

## Design

```
task path (poses, per-sample freedom, preferences)
   │
   ▼ sample (s grid; process-defined step, coarse then fine)
candidates per sample = external lattice × free-axis lattice × analytic IK branches (± wraps)
   │                     (filtered by joint limits; cheap)
   ▼ ladder search (structured DP: edges only between compatible neighbours)
discrete route: branch, external values, roll per sample
   │
   ▼ lazy collision on the route ──(collides)──► remove candidates/edges, search again
   ▼ refine: smooth external + roll along s, re-solve arm analytically on the fixed branch
joint path q(s), q'(s), q''(s)  ──► TOPP-RA once ──► final checks (limits, task space, clearance)
```

**Structured connectivity.**
- Descartes' generic ladder connects every pair of candidates in neighbouring rungs. That is C² edges
  per rung: with C ≈ 8 branches × 24 rolls × 40 track values ≈ 7700, far too many.
- Candidates carry lattice coordinates (branch, roll index, external indices). Edges connect only:
  - the same branch, with neighbouring lattice cells (±1 per redundant coordinate);
  - different branches where joint distance is below `max_jump` (wrist flips near singularities, found
    by a joint-space hash).
- That makes edges O(C·k).
- Coarse-to-fine: a coarse pass (e.g. 20 mm, coarse lattices) finds a corridor, and the fine pass
  searches only inside it.

**Costs.**
- Edges: Σ wᵢ|Δqᵢ|/vᵢ, with external axes weighted by the process's external cost, plus roll-change
  cost.
- States: limit-margin penalty, posture preference, process-preferred roll.
- Infeasible when any |Δqᵢ| exceeds `max_jump`.

**Entry and exit.**
- Approach and retreat lines (e.g. along the wire) are part of the path, so the ladder covers them.
- The move from the robot's current state to the path's first configuration is a joint-space motion.
  The ladder's free start is costed by joint distance from the current state.
- Its clearance is swept. If blocked, the next of the k-best starts is tried.
- Joint-space sampling-based planning (RRT-Connect) for blocked entries is "Later".

## Steps

**PP0. Baseline and harness.**
- Prerequisite: the gantries close-out:
  - submodules at their pinned commits, haxeon runtime rebuilt;
  - main merged in;
  - one full suite passing;
  - G18 skipped or done.
- Then:
- A benchmark runner over the existing focused app checks prints, per run:
  - planning seconds;
  - checked poses;
  - IK solves;
  - quality numbers: travel, arm margin, leg size, cycle time, clearance.
- Checks covered:
  - `PROJECT_SOURCE_ONLY=track-weld` (G17);
  - `welder` (robot-welder whole weldment, MuJoCo + test backend);
  - `gantry-welder`;
  - a surface/paint run (WallFinishing);
  - handling.
- Record every number in this plan's Progress section. They are the targets PP8–PP9 must beat,
  with weld quality unchanged.

**PP1. Analytic IK backends** (PP-D3).
- A native `mk_analytic_*` family behind one Haxe interface. Given an arm sub-chain and a base pose, it
  returns all branches.
- **OPW:** reuse `mk_opw_inverse`, extended to a sub-chain whose base comes from upstream joints.
  `OpwKinematics` must accept a group that has external axes by solving only its arm part. Prove that
  `machinekit` `RobotArm` is OPW-compatible with a round-trip test on the real model.
- **UR-type 6R:** a closed form for three parallel axes with an offset wrist, up to 8 branches.
  Parameters come from the model and are FK-verified. Test on `CobotArm`'s size classes.
- **Cartesian machines:** XYZ, XYZ+C and XYZ+C+A heads (G14). C and A follow from the tool axis
  direction, giving 2 branches for C+A, with the axis singularity handled explicitly.
- **Fallback:** multi-seed numeric IK (seeds from neighbouring candidates and a fixed seed set), with a
  diagnostic naming the family as unsupported.
- **Tests:** FK round trips over random joint vectors contain the original branch, for every family.
  Wraps within limits. Singular configurations are reported, not silently dropped.

**PP2. Candidate sampler** (PP-D4).
- Native. Per sample: external lattice values (per external DOF: range, resolution, optional rule) ×
  free-axis lattice (`FreeAboutTool` roll grid; `Cone` tilt rings) × analytic branches, filtered by
  joint limits.
- Candidates carry lattice coordinates.
- Haxe builds the problem from a `KinematicGroup` and the `PathRequest` freedoms: arm/external split
  from G3, limits from the compiled model (X9a).
- **Tests:**
  - candidate counts and determinism;
  - a rotated track;
  - a positioner;
  - a gantry with a C+A head;
  - roll lattice vs `ToolFreedom.orientationError` = 0.

**PP3. Structured ladder search.**
- Native DP with the connectivity and costs in "Design", coarse-to-fine.
- Diagnostics name the first disconnected sample and why: no candidates (unreachable or limits), or no
  edges (jump).
- Use Descartes' `LadderGraphSolver` where its all-pairs edges are affordable (small C). Otherwise use
  the structured DP. Both sit behind one function.
- **Tests:**
  - brute-force agreement on small problems;
  - a dead-end branch avoided, as in `testPathConfigurationSelector`;
  - a 2.6 m track path at 2 mm with 7700 candidates per sample searched in under 1 s native (Release).

**PP4. Continuous refinement.**
- Smooth the external values and the roll along s with a spline, period-aware for roll. Keep the
  branch fixed.
- Re-solve the arm analytically at every fine sample.
- q'(s) and q''(s) come from the differential relation, with the redundancy rates known from the
  spline, cross-checked by finite differences.
- Output `JointPathSamples` for `ToppraPathTiming`.
- **Tests:**
  - task-space error ≤ the compiler's path tolerance by construction;
  - no jumps;
  - derivatives match finite differences;
  - TOPP-RA succeeds on the track weld without retries.

**PP5. Lazy collision** (PP-D5).
- After each search, check the route's samples and the joint-space interpolation between them with
  `ArmClearance` (broad phase plus GJK).
- Remove the failing candidates, or the failing edge for a sweep failure, and search again, up to N
  rounds.
- Return the closest clearance on the final route.
- **Tests:**
  - an obstacle forcing a branch or roll change is avoided in a bounded number of rounds;
  - an impossible case reports the blocking pair and sample.

**PP6. Compiler integration.**
- `ProgramCompiler` takes a `JointPathPlanner`, replacing the unused `configurationSelector` argument.
  `lowerPath` asks it for the joint path of each FollowPath / MoveL section, with the section's
  freedoms and the process preferences. The result goes straight to timing.
- Approach and retreat lines join their path section.
- The free start is costed against the current state, with k-best starts kept.
- Delete `PathConfigurationSelector` and the selector argument.
- **Tests:** the existing MotionKit program, redundancy, posture and freedom suites pass. Numbers that
  move are recorded with reasons (e.g. a different but better branch).

**PP7. Entry and exit as motions.**
- The joint move from the current state to the chosen start is swept for clearance. If blocked, try the
  next of the k-best starts, then report.
- The exit mirrors this to a safe retreat configuration.
- **Tests:** a blocked first-choice entry falls back to the second start; this mirrors
  `WeldPlanningTests`' blocked-entry case.

**PP8. Welds on the new pipeline.**
- `WeldPathPlanner` becomes a problem builder:
  - per seam chain: tool freedom (`FreeAboutTool`), preferred roll and continuation hints from
    `WeldCorner`, and corner styles as path variants;
  - closed runs: the start sample is a free choice in the ladder, not rotated retries;
  - reversed travel: one alternative problem.
- Delete the roll loop, the entry-ways enumeration, the 1024-try budget and the per-candidate
  compile-and-verify.
- `WeldingPlanRunner` compiles once. Reuse that compilation for the rate schedule instead of compiling
  again in `launch()`.
- **Acceptance (on the PP0 benchmarks):**
  - G17 track weld under 15 s planning: 2600 mm, 5 mm leg, arm margin ≥ the PP0 value, no clearance
    violation, cycle time within 5 % of PP0;
  - robot-welder and gantry-welder whole weldments at least 10× faster, with every seam welded and leg
    sizes and the `WeldPlanningTests` cases unchanged.

**PP9. Other runners.**
- `SurfacePlanRunner`, `ToolpathPlanRunner` (FollowPath instead of MoveJ per waypoint) and
  `HandlingPlanRunner` use the planner.
- Delete `RedundancyResolver`, `SwivelParameterization` and `ExternalAxesParameterization`, unless a
  non-process user remains (live servo uses the QP differential IK, not these). Record what was kept
  and why.
- **Tests:** WallFinishing, excavator toolpaths, handling, mobile-base missions, gantry picker, and the
  machine-tending missions if on main.

**PP10. Native clearance (only if profiling demands it).**
- If lazy checks still dominate after PP8, add a coal-based distance API to kinematicskit native
  (hulls from the same link geometry) and use it in PP5.
- Otherwise record that it wasn't needed.

**PP11. Cleanup and docs.**
- Update `motionkit/plans/README.md`, `LANE_C_PLANNING.md` (C5 configuration selection superseded) and
  `robotkit/ARCHITECTURE.md`.
- Remove dead code found on the way, and record the final benchmark table.

## Order

```
PP0 → PP1 → PP2 → PP3 → PP4 → PP5 → PP6 → PP7 → PP8 → PP9 → PP11
                                              PP8 → PP10 (only if needed)
```

PP1's three IK families are independent and can land in any order. OPW first, since both welders use
`RobotArm`.

## Later

- Joint-space sampling-based planning (RRT-Connect) for entries blocked by clutter.
- Whole-path trajectory optimization (TrajOpt-style) for clearance margin, warm-started from the ladder.
- Multi-threaded candidate generation (deterministic ordering kept).
- Coordinated multi-robot paths (after MT6 controllers).

## Environment

Same as the gantries work:
- the worktree and test commands in the gantries Codex prompt;
- the full suite at phase boundaries, with focused suites in between.

Submodules come from the main checkout's stores, not from other worktrees (which get deleted).

## Progress

| Step | State | Commits |
|------|-------|---------|
| PP0 | in progress: harness and diagnostics; baseline completion pending | `0d43b32c0` (partial) |
| PP1 | in progress: Cartesian and external-axis OPW verified; UR and offset RobotArm pending | `0d43b32c0` (partial) |
| PP2 | planned | — |
| PP3 | planned | — |
| PP4 | planned | — |
| PP5 | planned | — |
| PP6 | planned | — |
| PP7 | planned | — |
| PP8 | planned | — |
| PP9 | planned | — |
| PP10 | planned | — |
| PP11 | planned | — |

### PP0 setup (2026-10-06)

- `gantries` includes local main `5e4bca72b`; Haxeon is at its pinned
  `80d00f541f1f480e8187614303936c8aba2a5670`. Its native runtime was rebuilt
  in `haxeon/out/cmake/pp0-release`; dependencies were initialized from the
  main checkout's stores. G18 remains skipped.
- App, MotionKit tests and RobotKit tests compile with the pinned compiler.
  MotionKit's runtime suite passes `71403` assertions, including numeric-query
  counter isolation. Combined workspace/peer and native CTest evidence is
  recorded below; baseline completion remains pending.
  The first workspace gate found uninitialized root dependencies (proxsuite,
  MCAP/LZ4 and MuJoCo's vendored dependencies); their pinned commits have now
  been initialized from the main checkout's stores. This attempt is not a
  passing gate; it must be rerun after its current actions finish.
- `motionkit/scripts/benchmark-process-paths.py` saves raw logs and structured
  run/quality records for the five baseline cases. Numeric pose-query counters
  are isolated per thread or caller-owned kinematics context; their isolation
  test is included in the MotionKit suite.
- Handling metrics count worker compilation steps under the worker's condition
  lock, excluding lookahead waits; `ManipulatorMotion` retains cumulative
  metrics across completed programs. The focused handling check emits per-step
  deltas and pick/place quality. The app compiles with this instrumentation;
  its runtime benchmark remains pending.
- Surface planning uses a complete synchronous compilation of the same patch
  for measurement, since runtime lookahead startup does not measure full
  compilation. An initial startup-only measurement was discarded.
- Baseline results are not yet established; no speedup is claimed.

| Baseline case | Run | Planning seconds | Numeric pose queries | Motion cycle seconds |
|---|---:|---:|---:|---:|
| Wall finishing, default backend | 1 | 0.906981 | 310 | 86.624458 |
| Wall finishing, default backend | 2 | 1.007403 | 312 | 85.395973 |
| Wall finishing, default backend | 3 | 1.005243 | 310 | 85.397328 |
| G17 track weld, MuJoCo | 1 | 161.044064 | 634552 | 236.6 |
| Arm handling, MuJoCo | 1 | 0.199098 | 61 | — |
| Arm handling, MuJoCo | 2 | 0.189073 | 51 | — |
| Arm handling, MuJoCo | 3 | 0.277194 | 53 | — |
| Arm handling, MuJoCo | 4 | 0.277770 | 59 | — |

Handling completed four pick/place steps in `23.460000000000868 s`, with a
`0.14152727976814344 m` lift. Placement and reset assertions passed. Each run
reports complete worker compilation time excluding lookahead waits; individual
motion-cycle times are not yet emitted. Evidence:
`/home/joao/dev/materia-cache/claude-scratch/process-path-baseline-missions/handling.log`.

Track-weld baseline: `224878` checked poses, `2.5999986904836363 m` track travel,
`0.7754218150152417 rad` arm margin, maximum joint step
`0.00001774160193779295 rad`, full `2600 mm` bead with `4.998146645480292 mm`
leg, maximum seam error `0.000022282434410538006 m`, wire error
`2.980232238769532e-8 rad`, no clearance violation. This refreshed-toolchain
baseline replaces the historical 306 s value for speedup comparisons; the
absolute under-15-second requirement remains. Evidence:
`/home/joao/dev/materia-cache/claude-scratch/process-path-baseline/`.

Wall-finishing quality: coverage `0.9959946595460614`, excluded-region coverage
`0`, maximum tracking error `7.777754145844614e-16 m`, RMS tracking error
`2.516468919819489e-16 m`. The focused check passed 71 assertions. These
planning measurements include program construction and full compilation;
motion-cycle numbers sum trajectory durations (excluding dwell barriers).
The current surface checker does not expose a checked-pose or clearance-distance
count; these are still missing from the PP0 harness. Raw JSON/log evidence is in
`/home/joao/dev/materia-cache/claude-scratch/process-path-baseline-surface-complete/`.

### PP1 preparation

The first native Cartesian backend is implemented as
`mk_analytic_cartesian_forward/inverse`: model-derived zero-pose space axes,
origins and TCP pose describe XYZ, XYZ+C and XYZ+C+A. Free-spin C+A returns
both geometric branches; fixed orientation filters them. Axial C singularities
are explicitly flagged, preserving the supplied seed when spin is free.
The native test exercises 600 round trips with a rotated translation basis,
displaced rotary origins and a pitched home TCP, plus singular/invalid-model
cases. Release checks remain enabled. MotionKit/TrajectoryKit native CTest
passes all `9` tests; the portable ABI audit passes Linux, Windows and both
macOS targets, and bindings are regenerated. The Haxe `CartesianAnalyticIk`
adapter now extracts the descriptor from `KinematicGroup`, verifies it against
compiled FK, and enumerates all rotary lifts within finite compiled limits.
Unsupported freedoms and unbounded wrap ranges are diagnosed explicitly. Its
shared `AnalyticIk` interface preserves branch identity and singularity flags.
Haxe model round trips for all three families (rotated base, displaced frames
and mounted tool) pass in the focused C4 suite. Actual picker/welder gantry
round trips under `PROJECT_SOURCE_ONLY=gantry-analytic` pass: `200` each for
XYZ picker, XYZ+C yaw picker and XYZ+C+A welder, all preserving mounted TCP
position/orientation and using zero numeric pose queries. Evidence:
`process-path-pp1-gantry-analytic.log`. OPW also implements the shared interface,
retaining its native branch identities, singularity flags and legal periodic
lifts, including lifts beyond one turn. The combined focused suite now passes
`6659` assertions (`process-path-pp1-cartesian-haxe.log`).
`OpwKinematics` now accepts a `KinematicGroup` with external axes, separating
its six-joint arm at the first arm driver's parent link. `OpwGroupIk` derives
the arm base in the current group reference (including a moving work frame),
solves the arm analytically at the supplied external lattice values and
restores the complete group configuration. Its FK composes the analytic arm
pose with that base. The focused suite passes `103568` assertions, including
rotated-track and track-plus-positioner round trips, complete seeded solves,
legal branch recovery and zero numeric pose queries. Evidence:
`process-path-pp1-external-opw.log`. External joints inside the arm chain are
diagnosed as unsupported; no flattening into a fictitious six-axis model occurs.
Production path compilation still uses its existing solver until PP6; the
analytic external-cell API is ready for PP2 sampling. UR, the authored offset
RobotArm decision and numeric fallback diagnostics remain pending; PP1 is
incomplete.

The pinned workspace gate passes all standalone suites; its one failure is the
peer-dependent TCP integration launched without robotd. That integration has
now passed separately with the documented robotd fixture (camera, multi-joint,
test authorization), including SkillRunner GoTo and MissionExecutor. Evidence:
`process-path-pp0-peer-runtime.log`. This is combined suite evidence, rather
than a claim that the unassisted workspace command exits zero. RobotKit's
native runtime was rebuilt with `ROBOTD_BUILD_TESTS=ON`; all `19` native CTest
cases pass, including wire, serial PTY, virtual-device, inference and SimKit
checks. Evidence: `process-path-pp0-runtime-ctest.log`. The normal build option
disables runtime tests, so the earlier two-trajectory-test pass was insufficient
and is superseded by this full pass. PP0 baseline completion remains pending.

PP0's robot-welder MuJoCo whole-weldment measurement is available; the focused
welder check is still running its other cases/backends.

| Robot-welder MuJoCo run | Planning seconds | Checked poses | Numeric pose queries |
|---:|---:|---:|---:|
| 1 | 223.917895 | 428451 | 476366 |
| 2 | 17.033878 | 7892 | 9801 |
| 3 | 3.428612 | 4636 | 905 |
| 4 | 49.322336 | 5571 | 27818 |

Quality: all 10 seams in four runs, cycle `114.4 s`, no clearance violation,
maximum seam error `0.00028911608356520056 m`, wire error `0 rad`.
Legs (mm): `4.9921112577618745, 4.988490679606122, 4.9699010788607945,
4.921193060062008, 4.973832402680148, 4.940971967400651, 4.92748327327362,
4.961859324866635, 4.9973399334701885, 5.001373493520714`.
Lengths (mm): `40, 39.99999999999998, 40, 39.99999999999998, 40, 40, 40, 40, 180, 180`.
Raw evidence: `process-path-baseline-missions/welder.log` in the scratch directory.

| Robot-welder test-backend run | Planning seconds | Checked poses | Numeric pose queries |
|---:|---:|---:|---:|
| 1 | 161.057081 | 428451 | 476366 |
| 2 | 17.499276 | 7892 | 9801 |
| 3 | 3.326203 | 4636 | 905 |
| 4 | 47.265934 | 5571 | 27818 |

Test-backend quality: all 10 seams, cycle `114.4 s`, no clearance violation,
maximum seam error `0.0002891857583632866 m`, wire error `0 rad`.
Legs (mm): `4.970854314068294, 5.0150636878779045, 4.961363053145066,
4.921848223437177, 4.995847873437931, 4.938814463483445, 4.930586331039058,
4.942692233198144, 4.997339933470188, 4.9973399334701885`.
Lengths match the MuJoCo run. The focused welder check's additional regression
cases are still running; the process exit and gantry-welder baseline remain
pending.

`PROJECT_SOURCE_ONLY=arm-analytic` checks 100 deterministic joint vectors
within the authored MachineKit arm's limits, using the actual generated
RobotModel and mounted suction tool. It checks OPW/model FK agreement and that
analytic inverse solutions contain the original configuration. The test is
implemented in the app project-source checks and compiles. Its first runtime
check fails during model extraction: `OPW joint j4 violates the
parallel-base/spherical-wrist axis pattern`. The existing fixture extractor
therefore does not yet support the authored arm. Parameter extraction must
handle its actual joint frames and reference pose and then pass the round-trip
proof; do not assume its compatibility from the spherical-wrist description.
Evidence: `/home/joao/dev/materia-cache/claude-scratch/process-path-pp1-arm-analytic.log`.
The assembly bridge embeds the initial placement in each parent joint frame
and shifts robot limits to that reference; extraction must handle this without
hard-coded initial angles.
OPW external-chain support, UR and Cartesian backends remain
unimplemented; this preparation does not satisfy PP1.

Extraction is being extended to recover a canonical straight-arm reference
from the model's axis lines and to test wrist-axis intersection rather than
coincidence of the selected axis origins. This uses no assembly-specific
initial angles. The code compiles; authored-arm and existing OPW runtime
checks are pending. Its authored-arm runtime check now reaches wrist-axis
intersection and rejects the geometry: the `j4` and `j6` axis lines are
separated by `35 mm` along the `j5` direction. The authored wrist is therefore
offset, despite its spherical-wrist description, and cannot satisfy an OPW
round-trip proof unchanged. The user has been asked to choose between keeping
the geometry and adding its analytic family, or redesigning it as spherical.
Neither decision is assumed yet. Existing OPW fixture diagnostics have also
been restored for the UR arm's perpendicular-axis violation. The focused C4
suite now passes `310` assertions, including 30 inverse round trips after
shifting the model's joint references and displacing each axis origin along
its own axis. This verifies the new extraction conventions on a genuine OPW
chain; it does not make the authored offset wrist OPW-compatible.

PP1 native UR backend preparation: added parameterized standard-DH forward
and closed-form inverse functions, preserving shoulder/wrist/elbow branch IDs
and wrist/elbow singularity flags. The BSD-licensed ROS-Industrial inverse
was adapted to receive inferred dimensions, offsets and signs; no robot-size
table is used by the solver. A near-singular round trip exposed its original
absolute tolerance snapping a valid wrist angle; the tolerance was tightened.
Native tests now pass 1,600 round trips over four dimension fixtures, signed
and shifted joint conventions, wrist singularities, unreachable targets and
invalid parameters. All 10 native CTests pass, and the regenerated Haxeon ABI
passes the four-platform audit. Model-derived UR extraction and Haxe adapter
remain outstanding, so this is preparation rather than PP1 completion.

PP0 mission baseline harness has finished: handling, robot welder and gantry
welder all exited successfully. Gantry MuJoCo planning runs took
3.429471970, 23.115318775, 20.144918919 and 49.658793926 seconds
(total 96.348503590), checking 5,192 / 5,798 / 10,033 / 12,186 poses
and making 522 / 13,829 / 514 / 14,253 numeric IK queries. The ten-seam
weldment cycle was 182.3 seconds, with no clearance violation; bead legs were
4.995715197–5.015761457 mm. Full records are retained in
`process-path-baseline-missions/results.json` under the external scratch
folder. Surface checked-pose/clearance-distance diagnostics are still missing
from the PP0 harness, so PP0 is not yet claimed complete.

PP1 UR Haxe adapter now derives canonical joint references, axis signs and DH
geometry from the standalone arm model, absorbs fixed base/tool placement,
and verifies extraction against compiled FK. It enumerates native branches
and legal rotary lifts through the shared AnalyticIk interface. The existing
UR-like contract fixture passes 30 forward/inverse round trips; a second
30-round-trip fixture verifies shifted joint references. Compiler-only build
and focused C4 runtime pass (103,688 assertions). Actual authored Cobot models,
external axes, equivalent displaced axis origins and wider geometric coverage
remain to be verified; PP1 is still incomplete.

PP1 UR external-chain support is now implemented: arm branches are evaluated
at the upstream/workpiece coordinates supplied in the complete seed, then
restored into the group's joint vector. Rotated-track and track-plus-positioner
fixtures pass 30 round trips each, including all returned branch TCP/orientation
checks and exact preservation of external coordinates. The standalone shifted
reference fixture also displaces each joint origin along its own axis and
passes. Focused C4 runtime passes 180,868 assertions. Authored Cobot validation
is still underway; its examples declare moveJoints missions at the flange,
so the fixture must use missionFrames rather than toolFrames or a handling
runner.

PP1 authored Cobot acceptance now passes: Reach500, Reach850, Reach900 and
Reach1300 each complete 200 analytic FK/inverse round trips over legal joint
vectors with the original lifted configuration among the returned branches.
Their simulation models are built from the actual example projects, using the
mission flange connector; extraction reads model frames/axes, not CobotReference
size tables. Each check observes zero numeric pose queries. App compiler-only
build and authored runtime exit successfully; log
`process-path-pp1-cobot-runtime.log` is retained in external scratch.

PP1 unsupported-family fallback now exists behind the shared branch interface.
BranchIk selects FK-verified Cartesian/OPW/UR geometry or returns NumericBranchIk
with the failed extraction diagnostics. Numeric enumeration tries neighbouring
seeds followed by a deterministic centre/axis seed set, deduplicates converged
configurations and holds external coordinates at the supplied lattice cell.
Numeric seed IDs are explicitly not persistent geometric branch IDs, and their
singularity classification is explicitly unknown (`singularityKnown=false`).
Tests cover family selection, repeated deterministic results, neighbour
retention/deduplication, task residuals and held external coordinates. Compiler
build and focused C4 runtime pass (180,958 assertions). This factory is not yet
wired into production planning; PP2–PP9 migrations remain outstanding, as does
the authored RobotArm geometry decision noted above. PP1 is not complete.

PP2 preparation: native orientation lattice sampling now enumerates fixed
orientation, free TCP-spin grids and concentric cone tilt/azimuth rings with
free-spin grids. Every sample carries roll/tilt/azimuth coordinates. Count and
sampling APIs validate dimensions, capacity, finite poses and overflow before
writing. Native tests cover deterministic counts/coordinates, rotated targets,
unit quaternions, preserved position, exact ring tilt, invalid requests and
zero-angle cone reduction. All 11 native CTests and the four-platform ABI audit
pass. The Haxe adapter respects Cone's independent axis in the path frame;
313 orientation samples pass existing ToolFreedom constraint checks, including
a tilted cone axis. Compiler-only build and focused C4 runtime pass (182,213
assertions). This is the orientation part only: native external-axis Cartesian
products, analytic branch/wrap filtering, and full candidate assembly remain
outstanding. PP2 is not complete and production planning remains unchanged.

PP2 native limit/lift preparation: added complete finite-range periodic joint
lift counting and enumeration, with lexicographic wrap coordinates, nonperiodic
limit filtering and locked-joint handling. Count/index overflow and infinite
planning ranges fail explicitly rather than truncating candidates. Native
coverage includes mixed joints, exact boundary lifts, rejected branches,
insufficient output capacity, locked joints and 200 legal lifts beyond the
old 64-turn restriction. All 12 native CTests and the four-platform ABI audit
pass. Cartesian and UR adapters now use this native implementation through
JointLifts instead of their Haxe wrap loops; compiler-only build and focused
C4 runtime pass unchanged at 182,213 assertions. The Haxe wrapper retains wrap
coordinates for the forthcoming candidate lattice. OPW's existing seeded
unbounded-range policy has not been replaced here. Native external-axis cell
products and combined analytic candidate assembly are still outstanding; PP2
remains incomplete.

PP2 external lattice preparation: native Cartesian-product sampling now emits
complete seeded joint configurations plus external-axis coordinates, with
included endpoints and deterministic last-coordinate-fastest ordering. Rules
are one-point ranges; no external axes gives a single unchanged configuration.
Validation rejects duplicate joint indices, invalid/duplicate sample ranges,
nonfinite seeds and product overflow. Weighted endpoint interpolation avoids
finite-range subtraction overflow. All 13 native CTests and the four-platform
ABI audit pass. Haxe ExternalAxisGrid validates descriptor ranges against the
compiled group limits, orders axes by group DOF, and holds omitted axes at the
sample seed. Rotated-track and track-plus-positioner grids verify cell counts,
coordinates, preserved arm joints and original analytic UR branches in each
cell, with zero numeric IK calls. Compiler-only build and focused C4 runtime
pass (182,310 assertions). Combined native cell × orientation × branch/wrap
candidate assembly remains outstanding; this does not complete PP2.

PP2 combined native serial-arm sampling now exists for UR and OPW. It composes
upstream arm-base and moving-workpiece space screws at each external cell,
transforms each work-frame orientation target into the analytic arm frame,
enumerates geometric branches, and filters/enumerates legal lifts while holding
external values. Candidates retain branch/singularity flags, external/roll/
tilt/azimuth coordinates and joint wrap indices. Count and sample APIs reject
invalid descriptors, out-of-limit grids, count overflow and insufficient
capacity explicitly; unreachable tasks return zero candidates. Independent
native FK checks cover a rotated track, rotating positioner, displaced/pitched
tool, all fixed/roll/cone cells, deterministic output and legal bounds for both
families. All 14 native CTests and the four-platform ABI audit pass; added
unreachable/duplicate-driver cases also pass the focused candidate test.
Haxe export of the native space-screw model and combined Cartesian sampling
remain outstanding. These APIs are not yet called by production planning;
PP2 is still incomplete.

PP2 combined Cartesian native sampling is now implemented alongside UR/OPW.
The shared descriptor records the actual arm joint count (3–6). XYZ, XYZ+C and
XYZ+C+A use native analytic branches and limit/lift enumeration. Free-spin and
cone constraints solve the tool axis directly; singular C samples use the roll
grid as seeded C choices, while repeated nonsingular geometric configurations
retain their first lattice coordinates. This avoids losing C+A branches by
requiring an unrelated discretized full rotation. Native tests verify original
configurations, branch counts, legal limits and FK task constraints for all
three Cartesian families across fixed/roll/cone policies. All 14 native CTests
and the four-platform ABI audit pass. Additional focused checks reject double
application of a Cartesian TCP offset and periodic XYZ flags. Cartesian's native
model already contains its full TCP; its separate tool transform must be
identity. Haxe model-derived export and combined sampler invocation still need
to be implemented. PP2 remains incomplete and production planning is unchanged.

PP2 Haxe-to-native Cartesian candidate bridge is implemented. It exports the
standalone Cartesian group's joint ordering and compiled limits, uses its
FK-verified native screw/TCP descriptor, builds the orientation policy once,
and calls combined native candidate counting/sampling without Haxe per-cell
or per-orientation IK loops. Returned records retain branch/singularity flags
and wrap/roll/tilt/azimuth coordinates. OrientationLattice.describe is shared
with the orientation-only adapter. Integration tests cover 40 model vectors
per XYZ / XYZ+C / XYZ+C+A family and fixed/free-spin/cone freedoms, with rotated
bases, displaced joints and a pitched/displaced TCP. Original legal vectors
are retained, all returned task residuals satisfy compiled FK / ToolFreedom,
and numeric IK query counts remain unchanged. Compiler-only build and focused
C4 runtime pass (190,110 assertions). Serial-arm space-screw export, combined
external-cell Haxe integration, PathRequest problem building and production
planner integration remain outstanding; PP2 is not complete.

PP2 serial Haxe-to-native export and invocation are now implemented. SerialCellModel
walks the model's upstream and work-frame paths at q=0, exports their root-frame
space screws, and composes the FK-verified analytic base/tool conventions with
the group's reference frames. Shared external joints can move both base and
work frame (scope 2), verified by independent native FK tests. SerialCandidateSampler
uses model-derived limits and one combined native grid/orientation/branch/lift
call; it performs no Haxe per-cell/per-orientation IK. ExternalAxisGrid.describe
is shared with its cell-only wrapper. UR and OPW integration checks cover a
rotated track alone and with a positioner, fixed/free-spin policies, pitched and
displaced TCPs, original centre-cell branch retention, coordinate values and
all returned task residuals. The first test grid was far from its target's
external coordinates near full arm extension; it correctly returned no candidates.
The centred grid includes a known reachable configuration. Compiler-only build,
all 14 native CTests, four-platform ABI audit and focused C4 runtime pass
(262,066 assertions). PathRequest problem building, cone integration across
serial groups, authored combined-sampler acceptance and production migration
remain outstanding; PP2 is not yet complete.

PP2 PathRequest candidate construction is now implemented as CandidateProblem.
It selects the model-derived native family, validates aligned path/joint inputs,
passes each sample's orientation freedom and external grid/rule to the combined
sampler, and retains complete candidate layers with lattice metadata. The old
PathRequest.maxCandidates cap does not truncate native enumeration. Unsupported
geometry retains its explicit diagnostic and numeric neighbour-seed fallback;
its singularity metadata remains unknown. Options support pinned or free first
layers and per-sample external-axis rules. A pinned first layer samples the
start's exact FK pose after verifying the original task constraint, so valid
cone-start spin is independent of orientation-grid resolution. Tests verify
pinned/free starts, deterministic branch/joint ordering, complete native layers
beyond the legacy cap, one-point track/positioner rules for both serial families,
zero native-family numeric queries and a one-roll-grid pinned cone start.
Compiler-only build and focused C4 runtime pass (265,425 assertions).
Broader authored combined-sampler acceptance and serial cone/determinism checks
remain outstanding; this builder is not wired into production selection yet.
PP2 is not claimed complete, and PP3–PP11 remain to be implemented/verified.

PP2 serial acceptance coverage is extended across fixed/free-spin/cone policies
for UR and OPW with rotated tracks and optional positioners. Cone checks hold
the known external cell while testing all returned task residuals. Repeat calls
verify every returned joint, branch/singularity flag, wrap, external coordinate
and orientation coordinate; every configuration respects compiled bounds and
external coordinates are never independently lifted. Compiler-only build and
focused C4 runtime pass (1,159,649 assertions). Authored Cobot and Cartesian
example checks now include combined native sampling; their app compiler-only
build passes, and the authored runtime job is running (results not yet claimed).
PP2 acceptance is still pending that job and the appropriate phase gate.

PP2 authored validation exposed an empty candidate set after Cobot500 passed,
while checking Cobot850. Cone centres were built from ToolFreedom's canonical
axis-alignment quaternion, discarding the requested target spin. The lattice
centre now minimally aligns the target's tool axis to the cone axis while
preserving its spin preference; when the axes already match, the original
orientation is retained. Stable atan2 alignment handles antipodal and nearly
antipodal axes without the cancellation of a 1+dot construction. All orientation
and combined-sampler wrappers share this convention. Regression checks verify
spin retention and exact/near-antipodal alignment, and existing candidate/FK
coverage passes. MotionKit and authored-app compiler-only builds pass; focused
C4 runtime passes 1,143,524 assertions. The authored Cobot/gantry job has been
restarted after the earlier job terminated, and its acceptance remains pending.

PP2 authored Cobot checks now pass all four size classes: 200 analytic round
trips and 60 combined native candidate sets each, covering fixed/free-spin/cone
constraints, mounted TCP FK and compiled bounds. The gantry XYZ case also
passed, but the yaw variant exposed a wrapper rejection when KinematicGroup
marks axes inside the complete Cartesian chain as external. The verified
Cartesian descriptor already contains these task axes, so the sampler now
solves them geometrically; CandidateProblem does not classify them as separate
redundant lattice axes. Regression fixtures explicitly mark a leading axis
external across all three Cartesian families and verify FK and pinned layers.
MotionKit and app compiler-only builds pass; focused C4 runtime passes
1,143,540 assertions. Gantry authored checks are rerunning after the previous
job terminated. The MotionKit full phase-gate runtime is also running on the
preceding compiled module; its result and the authored gantry result are still
pending. PP2 is not claimed complete.

PP2 authored acceptance now passes: all four Cobot classes complete 200 analytic
round trips plus 60 combined native sets each; XYZ / XYZ+C pickers and XYZ+C+A
welder each complete 200 analytic round trips plus 30 native sets. FK, task
freedoms and compiled limits are checked on every returned candidate, with no
numeric queries. The full MotionKit phase gate remains running. A further audit
found OPW's singular inverse chooses a zero q4 representative, so a valid pinned
start with nonzero q4 can be absent. A seeded q4/q6 coupling correction and
zero/pi wrist tests with signed/offset conventions are staged, awaiting native
validation after the running gate releases the shared library. PP2 remains open.

PP2 wrist-pole acceptance: combined OPW sampling now preserves seeded q4 at
both q5=0 and q5=pi and derives the coupled q6 from the target rotation.
Signed/offset conventions exposed a roundoff-induced negative square root in
the vendor wrist cosine; finite cosine clamping fixes that without masking
unreachable-arm NaNs. Four native seed-retention cases verify every emitted
pose against FK. All 14 native CTest cases pass after this correction.
The preceding full MotionKit gate passed 1,214,827 assertions; that gate
predates this correction, which has native regression coverage. Production
planning migration and the remaining plan phases are still outstanding.
