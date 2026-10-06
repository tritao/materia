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
| PP1 | in progress: Cartesian, OPW and authored Cobot UR verified; offset RobotArm unresolved | `0d43b32c0`, `bfec0fa28`, `8569a2a98` (partial) |
| PP2 | in progress: native family samplers and Haxe problem construction implemented; close-out pending | `601fff4ba`, `8569a2a98`, `e28f1092e` |
| PP3 | in progress: structured/coarse search and Descartes dispatch implemented; authored gate pending | `7cb639434`, `489f2c360`, `f8e88ad4d`, `09cb60fbb` |
| PP4 | in progress: analytic/numeric refinement, cone rates and timing verified; transitions/authored gate pending | `4b9e750ff`, `0e5edbc06`, `39b6081c1` |
| PP5 | complete: lazy sample/edge/refined retries, physical acceptance and full native/Haxe gate | `e1435022f`, `e5336893f`, `bfd03d585` |
| PP6 | in progress: planner argument, axis/standalone OPW defaults, class removal and free entry verified; remaining defaults and joined approach/retreat pending | `46f29f74f`, `4edacf8af`, `fc0efe042` |
| PP7 | complete: generated entry/retreat selection, retries and emission; full MotionKit/native gate passed | `92b8c69ee`, `093e65a46`, `e41bdfffa`; retreat gate below |
| PP8 | in progress: launch rate precompile removed; weld problem builder/search migration and acceptance pending | final-clock rate scheduling below |
| PP9 | in progress: handling and surface use the structured planner and authored missions pass; toolpaths/deletions/remaining mission gate pending | runner migrations below |
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

PP2 Haxe wrist-pole regression: CandidateProblem retains exactly one pinned
OPW start at both wrist poles, keeps the full seed exact, reports singularity
and performs no numeric IK. Compiler-only build and the focused analytic/
candidate suite pass (1,143,558 assertions) against the corrected native
library. Structured ladder search (PP3) remains the next implementation step.

PP3 initial implementation: an internal native structured DP indexes same-
branch lattice neighbours (periodic roll/azimuth) and uses a three-coordinate
joint hash for nearby cross-branch moves, checking all joint jumps exactly.
It retains two cost layers plus route predecessors, includes weighted joint/
velocity travel, roll changes and supplied state costs, and reports the first
disconnected layer with empty-candidate versus no-edge classification.
One hundred random small problems agree with a brute-force structured-graph
reference, and dead-end/empty-layer/jump-disconnection cases pass with native
assertions enabled in Release. This is internal preparation: public ABI/Haxe
integration, Descartes dispatch, coarse-to-fine and the 1301 x 7700 performance
gate remain outstanding; PP3 is not complete.

PP3 input and seam checks: the internal structured solver rejects invalid
dimensions, nonfinite/invalid jump and speed values, unrepresentable hash
coordinates, out-of-range orientation indices and mismatched/invalid state
cost arrays before searching. A periodic-roll seam regression verifies legal
physical continuity despite changing wrap labels. Hash bucket width includes
the jump comparison tolerance so boundary branch changes are not lost.
The focused native structured-ladder test passes with Release assertions
enabled; no public API or production planning migration is claimed yet.

PP3 scale investigation: added an explicit synthetic track-shaped benchmark
(8 branches x 24 rolls x 40 track cells plus 20 periodic lifts = 7700
candidates). This is a performance probe, not an authored-track acceptance
fixture. The initial 1301-layer materialization terminated with exit 143
without a timing result; its observed resident memory exceeded 10 GB. The
current fixed-size 64-joint candidate representation therefore needs a
streamed/compact layer interface before repeating the full-size probe.
Exact branch bounding boxes now skip impossible cross-branch searches, and
single-cell orientation coordinates skip redundant neighbour probes. The
small random-reference test still passes; a 21-layer optimized probe took
0.183113 s and tested 1,367,600 edges. The <1 s requirement is not achieved.

PP3 streaming scale evidence: structured DP now accepts a layer provider
whose two-slot cache keeps only the current/prior candidate layers. Validation
runs as each layer is consumed. One hundred random problems produce identical
routes, costs, failure indices and edge counts with stored versus streamed
layers, in addition to brute-force agreement. The complete synthetic
1301 x 7700 probe now finishes with ~60 MB peak RSS rather than >10 GB.
Initial streaming timing was 13.305743 s; hashing/comparing only active
lattice coordinates reduced it to 5.741541 s (88,894,000 tested edges),
including streamed candidate generation. The one-second acceptance target
remains unmet, and this synthetic probe is not authored-track coverage.

PP3 native interface: mk_search_ladder reads candidate slices without copying
the candidate ladder and returns global selected indices, cost, edge counts,
first failed sample/distance and empty-layer versus no-edge diagnostics.
Requests carry compiled jump/speed limits, process weights, free-start
configuration and lattice dimensions; nonnegative per-candidate costs
(including +infinity for disabled states) carry margin/posture preferences.
Native API regressions cover successful selection, failed edges, empty
layers, invalid slices and invalid limits. All 15 native CTest cases pass,
and the regenerated portable ABI passes Linux/Windows/macOS x86_64/arm64
audit. Haxe bridging, small-problem Descartes dispatch, coarse-to-fine and
the subsecond large-path gate remain outstanding.

PP3 Haxe bridge: StructuredLadder.search exports CandidateProblem layers,
compiled request jumps/speeds, process joint weights, roll cost and optional
state-cost callback to mk_search_ladder. Selection retains full lattice
candidates and typed failed-sample/distance diagnostics; +infinity state
costs disable candidates for later lazy pruning. CandidateProblem retains
its configured orientation dimensions. The output report is now explicitly
annotated for FFI ownership/projection, and the four-platform ABI audit
passes. Compiler-only build and focused runtime pass (1,143,579 assertions),
including deterministic selected-route FK, disabled-state and empty-layer
diagnostics and no numeric IK calls. This bridge is not yet wired into
production compiler selection; PP3/PP6 and later phases remain outstanding.

PP3 initial coarse-to-fine implementation: sparse sample anchors and a
coarsened lattice produce a route; a streamed fine corridor interpolates
external/orientation coordinates, respecting periodic roll/azimuth seams.
All branches and wraps inside the corridor remain, and fine edges retain
the original jump limits. Disconnected corridors widen, then fall back to
the full graph. Corridor selection is approximate and does not promise the
full-graph global optimum. Tests cover unchanged full-corridor results on
100 random ladders, widening around an off-centre bend and full fallback.
The synthetic 1301 x 7700 probe completed in 0.963783 s, peak RSS 86412 KB,
17,653,870 tested edges, cost 2.602 (same as full search on that fixture).
This is one preliminary Release measurement including synthetic streamed
generation, not authored-track acceptance or a production speedup claim.
Coarse-to-fine remains internal: native/Haxe API integration, Descartes
dispatch and authored performance verification are outstanding.

PP3 coarse API integration: native mk_ladder_request now exposes opt-in
sample/lattice strides, corridor radius and widening count. Its default
remains the full structured graph. The native wrapper validates all input
candidate records, coordinates and state costs before corridor filtering,
so invalid excluded records cannot silently pass. Haxe CoarseSearchOptions
exports the same settings through StructuredLadder.search. Native API tests
exercise coarse selection and invalid options; Haxe verifies full-corridor
selection agrees with full search. Compiler-only and focused runtime pass
(1,143,581 assertions), all 15 native CTest cases pass, and the portable
four-platform ABI audit passes. Descartes small-ladder dispatch, authored
large-path acceptance and production migration remain outstanding.

PP3 bounded Descartes dispatch: the shared native entry point uses
LadderGraphSolver for ladders with at most 32 candidates per layer and
one million potential edges; large ladders retain indexed DP, and explicit
coarse settings select the corridor solver. The Descartes adapter preserves
structured neighbour/branch-change connectivity, exact joint jumps,
weighted travel, roll costs, per-state costs and free-start travel. A
structured reference supplies validation/failure diagnostics for these
small problems and provides fallback if Descartes cannot produce a route.
The result reports the actual backend (structured/corridor/Descartes), and
native tests explicitly prove Descartes executes and agrees with the
brute-force cost on reachable random small ladders; disconnected cases
agree on their first failure. All 15 native tests, four-platform ABI audit,
compiler-only and focused Haxe runtime (1,143,581 assertions) pass.
Authored large-track acceptance and the PP3 phase gate remain outstanding.

PP3 corridor-validation hardening: common settings validation now runs
before coarse lattice indexing, and all consumed source candidates are
validated before corridor/coarse filtering. Invalid state costs are checked
even when the candidate would be omitted. Regressions reject oversized
external dimensions and an off-corridor NaN joint rather than accepting
the remaining path. Focused native tests pass. The full synthetic probe
with this validation measured 1.050869 s and 1.179377 s after replacing
the hash-range division with its equivalent multiplication; these timings
do not establish a stable subsecond acceptance gate.
Authored TrackWelder inspection confirms it includes RobotArm, whose
35 mm offset wrist remains unsupported by OPW. A user geometry decision
(preserve offset wrist and add a supported solver, or redesign for OPW)
is pending; the authored OPW/track acceptance requirement is still open.

PP3 conservative coarse connectivity: initially use the original joint-jump
limits at coarse anchors, then retry with stride-scaled jumps only when
that coarse graph disconnects. This avoids building a dense cross-branch
coarse graph on paths whose arm posture moves little. Fine legality is
unchanged, and the off-centre bend test explicitly proves relaxation plus
corridor widening still finishes with the corridor backend. Focused native
regressions pass. The full synthetic 1301 x 7700 probe now tests 1,546,870
edges (previously 17,653,870); three measurements were 0.867291, 0.915042,
0.811200 s total. Candidate generation consumed 0.591217, 0.627135,
0.524589 s; search consumed 0.276073, 0.287907, 0.286611 s respectively.
All retained cost 2.602. This proves the synthetic search scale only;
authored-track acceptance and overall production planning remain open.

PP4 independent spline preparation: RedundancySpline fits a natural C2
cubic in path distance, unwraps periodic roll knots before fitting, and
evaluates the value and analytic first/second derivatives without wrapping
the output back across its seam. It rejects invalid knots, extrapolation
and unrepresentable intervals/coefficients/samples. Cubic overshoot is
explicitly left for refinement bounds validation rather than clipping
and breaking smoothness. Tests cover affine and two-knot reproduction,
roll-seam continuity, knot interpolation and C1/C2 continuity, derivative
finite-difference agreement and invalid inputs. Compiler-only and focused
runtime pass (1,144,398 assertions). This does not complete PP4: fixed-
branch analytic arm re-solving, differential joint derivatives,
JointPathSamples/timing integration and authored-track acceptance remain
outstanding, as do the earlier PP1/PP3 authored gates.

PP4 serial analytic refinement preparation: AnalyticPathRefiner fits
external values and orientation swing/twist from selected configurations
using actual FK, then re-solves a fixed geometric branch through the
combined native serial sampler at each supplied fine target. Recovering
spin from FK preserves pinned starts whose task pose has different free
spin, instead of assuming the pinned lattice roll index is their spin.
Every output is checked against compiled limits, original joint-jump
limits and task position/orientation tolerance. External spline overshoot
and unreachable branches are diagnosed. Explicit branch transitions must
be segmented; numeric and Cartesian refinement support remain separate
outstanding work. UR tests cover eleven fine fixed-orientation samples
and eleven free-roll samples, exact pinned spin recovery, continuous lifts
and no numeric IK calls. Compiler-only and focused runtime pass
(1,144,570 assertions). Differential q-prime/q-double-prime, complete
JointPathSamples/timing integration and authored acceptance are not done;
this helper alone does not complete PP4.

PP4 differential relation: native mk_path_differential solves J q-prime =
task velocity with redundant rates prescribed, then J q-double-prime =
task acceleration - J-prime q-prime with prescribed redundant curvature.
Column-pivoted QR checks rank, and residual checks reject incompatible
six-dimensional task derivatives. Rank loss is explicit. Native cases
cover a six-axis arm plus moving external coordinate, exact known rates,
Cartesian XYZ, all-known rates, task residuals and invalid inputs.
Haxe PathDifferential uses the compiled TCP Jacobian and a centred
directional difference for J-prime along the solved q-prime; q derivatives
are obtained from the differential relation, not joint-position differences.
AnalyticPathRefiner.derivatives supplies the external spline rates and
checks that the configuration matches them. Independent UR FK differences
recover the expected joint rate and zero curvature. All 16 native tests,
four-platform ABI audit, compiler-only and focused runtime pass
(1,144,582 assertions). Refined orientation task derivatives, full
JointPathSamples/timing integration, broader moving-reference acceptance
and the authored track gate remain outstanding; PP4 is not complete.

PP4 refined orientation derivatives: OrientationDifferential composes a
moving centre quaternion, spline swing vector and periodic roll with
second-order scalar jets. It returns analytic spatial angular velocity and
acceleration, including centre/swing/roll coupling. A squared-tilt power
series avoids the polar singularity at zero swing. AnalyticPathRefiner uses
this same composition for poses and refinedDerivatives adds these angular
rates before the Jacobian differential solve. Centre rates must describe
OrientationLattice.centre, including any changing cone axis. Independent
quaternion differences over 21 nonlinear cases check angular velocity and
acceleration, while a zero-swing case checks exact coupling and finite
rates. Existing fixed/free-spin refinement and differential tests remain
green. Compiler-only and focused runtime pass (1,144,735 assertions).
Full JointPathSamples/timing integration, centre derivative providers for
all geometric paths, broader family/moving-reference coverage, branch
segmentation and authored acceptance remain outstanding.

PP4 serial path/timing bridge: AnalyticPathRefiner.refinePath now produces
JointPathSamples over the complete selected distance range from a typed
RefinementTarget provider. It preserves the pinned initial configuration,
re-solves the fixed analytic branch at each fine target, supplies
differential q-prime/q-double-prime and carries incoming curvature
separately at C1 geometric knots. Identical incoming/outgoing task
curvature reuses the differential result. A UR circular TCP fixture
provides independent exact task derivatives; all 21 refined samples
match the analytic joint curve and derivatives. It succeeds in a single
TOPP-RA call, reaches its full endpoint, and passes sampled velocity/
acceleration checks plus native trajectory-extrema position/velocity/
acceleration validation. A piecewise C1 fixture verifies outgoing zero
and incoming 0.2 rad/path-unit-squared curvature at its central knot.
Compiler-only and focused runtime pass (1,146,337 assertions). This is
a serial fixture integration, not the authored track one-pass timing
acceptance; Cartesian/numeric refinement, geometric centre derivative
providers, moving-reference/OPW coverage and production integration remain
outstanding, so PP4 and the overall plan remain incomplete.

PP4 moving-reference acceptance: shared OPW and UR fixtures now refine
selected routes with a moving rotated track, both with and without a
simultaneously rotating work frame. Independent full-model FK differences
supply task derivatives at eleven fine samples. The analytic solve keeps
all arm joints constant, holds the spline external coordinates exactly,
recovers their prescribed 0.5 path-unit rates, and cancels moving-reference
curvature to zero joint acceleration. Existing numeric-query counters
also prove these refinement checks do not invoke numeric pose IK.
Compiler-only and focused runtime pass (1,147,331 assertions). Production
integration, Cartesian/numeric refinement, branch segmentation, geometric
centre derivative providers and authored timing/performance gates remain
outstanding; this does not complete PP4.

PP4 Cartesian refinement: AnalyticPathRefiner now dispatches its fixed
refined targets to the native Cartesian sampler for XYZ, XYZ+C and
XYZ+C+A, while retaining serial sampling for OPW/UR. Cartesian task axes
remain fully solved, including axes marked external by assembly ownership;
they are not held as serial redundancy coordinates. The existing branch,
physical lift, jump, task-freedom and FK checks apply to both families.
All three rotated-base/tool-offset Cartesian fixtures now produce complete
JointPathSamples on eleven fine samples with exact task-axis motion,
differential velocity and zero straight-path curvature. Their existing
numeric-query checks prove no numeric pose IK is introduced. Compiler-only
and focused runtime pass (1,147,730 assertions). Rotary Cartesian curvature
acceptance, numeric fallback refinement, branch segmentation, geometric
centre derivative providers and production/authored gates remain open.

PP4 Cartesian rotary acceptance: XYZ+C and XYZ+C+A now also refine a
nonuniform C-axis rotation with an offset tool and rotated base. An
independent circle formula supplies exact TCP velocity and centripetal/
tangential acceleration, with nonzero angular acceleration. Eleven fine
samples preserve the selected joint curve, recover the changing C-axis
rate and 0.4 curvature, and keep the translation axes stationary despite
the moving TCP. Existing numeric-query counters still pass. Compiler-only
and focused runtime pass (1,148,029 assertions). Numeric fallback refinement,
branch segmentation, geometric centre derivative providers, production
integration and authored timing/performance gates remain outstanding.

PP4 authored pose-path adapter: PosePathRefinement supplies RefinementTarget
from PosePrimitive.derivativesAt in the group's task reference frame. At
exact joins it selects outgoing task data and retains incoming curvature;
velocity discontinuities require a stop/segment or blend rather than a
single smooth differential solve. Fixed, interpolated and free-tool-spin
centres use primitive rates. Cone projection and full-free centres are
explicitly diagnosed until their centre derivatives are implemented.
Authored PoseLine paths now pass through CandidateProblem, StructuredLadder,
this provider and AnalyticPathRefiner to JointPathSamples for all three
Cartesian families. Compiler-only and focused runtime pass (1,148,293
assertions). Join/arc/weave acceptance, cone-centre derivatives, numeric
refinement, branch segmentation and production/authored gates remain open.

PP4 primitive joins and fixed-orientation geometry: acceptance now checks
an exact tangent line/arc join, outgoing arc curvature, incoming zero line
curvature, rejection of an unblended corner and invalid path distances.
PoseLine and PoseArc supply direct analytic translation derivatives for
Fixed policy (including 3D arc tangent and curvature); rotating policies
retain their existing derivative implementation. This removes numerical
geometry differences in the common fixed-orientation process case.
Compiler-only and focused runtime pass (1,148,298 assertions). Cone-centre
rates, broader rotating/weave paths, numeric refinement, branch segmentation
and production/authored gates remain open.

PP6 contract preparation: JointPathPlanner returns complete JointPathSamples
for an authored PosePath and PathRequest. StructuredJointPathPlanner composes
CandidateProblem, native StructuredLadder and AnalyticPathRefiner with the
pose-path task provider, retaining sampling/coarse-search settings. It
checks full-range coverage and alignment of searched geometry and freedoms
with the refined task, and reports family/selection failures explicitly.
All three Cartesian authored-path fixtures exercise this contract and
produce the same positions/rates as the separately verified pipeline.
Compiler-only and focused runtime pass (1,148,562 assertions). This is
preparation: ProgramCompiler still uses its existing path lowering, and
collision filtering, unsupported-family fallback, branch segmentation,
k-best starts and selector removal remain outstanding. PP6 is not complete.

PP5 sample-filter preparation: LazyCollisionLadder searches first, checks
only selected candidates, disables colliding states with infinite costs
and retries within an explicit round budget. It integrates ArmClearance
pose and sweep checks through a concrete wrapper, with checker injection
for deterministic tests. An impossible pinned state reports the blocking
pair and sample. Sweep blockage reports its edge and pair explicitly;
edge exclusion is still required, rather than incorrectly deleting either
endpoint. Clear-route, selected-only checking, impossible-start and sweep
rejection tests pass. Compiler-only and focused runtime pass (1,148,566
assertions). Native edge exclusion, physical obstacle rerouting acceptance,
closest-clearance reporting and refined-curve checking remain outstanding;
PP5 is not complete and this is not yet wired into production planning.

PP5/PP6 collision composition: StructuredJointPathPlanner accepts an
optional ArmClearance world, contact mode and collision-round budget. It
uses lazy sample filtering for discrete selection and checks the refined
samples and joint sweeps before returning the curve. Refined collisions
are diagnosed, not silently accepted. An injected-checker UR test rejects
the first selected endpoint, verifies a different task-valid candidate in
exactly two rounds, enforces a one-round failure budget, and proves the
source problem is unchanged. Compiler-only and focused runtime pass
(1,148,571 assertions). This is injected-checker rerouting, not physical
obstacle acceptance. Native edge exclusion, closest clearance, refined
collision re-search, physical integration acceptance and production
compiler wiring remain outstanding.

PP5 native edge-exclusion preparation: the internal structured_ladder
accepts an optional transition predicate keyed by destination layer and
local predecessor/destination indices. Exclusions are applied before
transition relaxation, preserving both endpoint candidates. A native test
blocks the cheapest edge, retains its destination through another
predecessor, checks exact route/cost for stored and two-slot streamed
layers, and verifies edge-only disconnection reports an edge failure.
Native rebuild and focused structured-ladder CTest pass. Public native/
Haxe blocked-edge records, coarse index mapping, Descartes handling and
lazy sweep retry integration remain outstanding. This internal primitive
alone does not complete PP5.

PP5 coarse edge-exclusion mapping: coarse_ladder now accepts the same
source-index transition predicate. Adjacent coarse anchors map their
indices back before checking it; skipped anchor transitions remain hints.
Every fine corridor edge maps both endpoints to original candidate
indices, and widening/full-search fallbacks retain exclusions. Native
acceptance covers adjacent/skipped anchors and a narrow corridor that
removes decoys, changes local indices and still returns the exact allowed
route/cost with the corridor backend. Native rebuild and focused
structured-ladder CTest pass. Public ABI/Haxe records, Descartes handling,
lazy sweep retry, closest clearance and physical acceptance remain open.

PP5 public native exclusions: mk_search_ladder_filtered accepts validated
mk_ladder_edge records (destination sample and local source/destination
candidate indices). The original entry point delegates with no exclusions.
A deduplicated transition set reaches structured DP, mapped coarse search
and Descartes edge evaluators; the structured reference used for Descartes
failure/fallback respects the same set. Random native problems exclude a
selected edge and compare filtered Descartes and full-corridor coarse
status, failure sample and optimal cost with the structured reference;
invalid sample-zero exclusions are rejected. Native rebuild, focused
structured CTest and four-platform ABI audit pass; bindings regenerated.
Haxe blocked-edge wiring, lazy sweep re-search, closest clearance and
physical obstacle acceptance remain outstanding.

PP5 Haxe sweep retry: StructuredLadder accepts validated BlockedLadderEdge
records and calls the filtered native API. LazyCollisionLadder records
failed swept transitions using original layer-local candidate indices,
then searches again within the same round budget while preserving both
endpoint states. Selected-pose filtering and edge filtering coexist.
Injected-checker acceptance verifies a blocked sweep selects an alternate
route in exactly two rounds, native exclusions reach Haxe, invalid sample-
zero records are rejected, and fully blocked/budget cases still fail.
Compiler-only and focused runtime pass (1,148,574 assertions). Physical
obstacle branch/roll acceptance, closest-clearance reporting, refined
collision re-search and production/compiler migration remain outstanding;
PP5 and the full plan remain incomplete.

PP5 closest-pair query preparation: ArmClearance.closest returns the nearest
checked hull pair even when all pairs exceed the required margin, retaining
body names and the effective contact/arm margin. It queries full hull
distance with no margin-based early exit and reuses placed hulls per pose.
As with ConvexDistance, an iteration-limit result is a conservative lower
bound, not a certified converged exact distance. A physical prismatic
fixture checks two obstacles, nearest-pair selection, independent known
box gaps at two configurations, contact margin and empty-world behavior.
Compiler-only and focused runtime pass (1,148,580 assertions). Route/sweep
closest-clearance aggregation, physical branch/roll rerouting acceptance,
refined collision re-search and production migration remain outstanding.

PP5 sampled route clearance: ArmClearance.closestSweep aggregates full hull
queries over the same straight joint-space sampling rule as sweep(), with
finite-step/joint validation. LazyCollisionLadder.select measures only the
winning route after collision retries and attaches its closest checked
pair to LadderSelection.closestClearance. The result is sampled clearance,
not a continuous swept-volume guarantee; GJK iteration-limit bounds retain
the documented conservative semantics. Physical box acceptance verifies
an interior collision between clear endpoints, a clear sweep's known
minimum gap and invalid sampling-step rejection. Compiler-only and focused
runtime pass (1,148,584 assertions). Physical branch/roll obstacle rerouting,
refined collision re-search and production/compiler migration remain open;
PP5 and the full plan remain incomplete.

PP5 physical obstacle acceptance: an XYZ+C cell carries an offset convex
tool body past a fixed convex post with free tool spin. The cheapest native
route collides; LazyCollisionLadder with the real ArmClearance world selects
an alternate roll within three rounds. Its joint sweep is clear, TCP
position is preserved and returned sampled clearance exceeds its margin.
A second physical world blocks the pinned start and reports sample zero
plus both body names. Compiler-only and focused runtime pass (1,148,591
assertions). Phase-boundary full-suite validation, refined collision
re-search and production/compiler migration remain outstanding; no authored
weld performance or full-plan completion claim is made.

PP6 optional compiler path: ProgramCompiler now accepts JointPathPlanner
and lowerPath consumes its complete JointPathSamples directly in the
existing timing/event/feed-cap/motor-space and trajectory-validation flow.
It does not recompute joint derivatives through the old solver for these
paths. JointPathPlanner.withSolver rebuilds structured planning on worker
ManipulatorKinematics; ArmClearance.withGroup reconstructs collision
geometry/reference exclusions on the worker's independent group. XYZ,
XYZ+C and XYZ+C+A MoveL programs compile through this path and through
forked workers. Compiler-only and focused runtime pass (1,148,600 assertions).
This is opt-in integration: default planner construction, fallback/cone/
branch support, approach/retreat/k-best behavior, selector removal and the
full phase-boundary integration suites remain outstanding. PP6 and the
full plan are not complete.

PP6 serial compiler acceptance: the UR fixture now compiles an interpolated
MoveL through StructuredJointPathPlanner, differential refinement, shared
timing and task/trajectory validation, both directly and in a forked
worker. The root compiled group records no numeric pose IK calls. The
compiler validates returned joint/sample counts, exact requested distance
alignment and pinned initial configuration before accepting planner output.
Compiler-only and focused runtime pass (1,148,603 assertions). Default
migration, unsupported families/cone centres/branch transitions, free-start
entry handling, selector removal and full integration/performance gates
remain outstanding; this serial fixture is not authored track acceptance.

PP4 primitive angular rates: PoseMath.angularRates now differentiates the
same shortest-hemisphere spherical/normalized-linear quaternion
interpolation used by authored geometry. Spherical interpolation has a
constant spatial angular rate; the small-angle normalized-linear case
carries its nonconstant rate and exact angular acceleration. PoseLine and
PoseArc combine these rates with analytic translation derivatives for all
policies, removing their previous numerical pose differences. Independent
quaternion-difference acceptance covers five small/large angles, a rotated
initial frame and five interior distances. Existing structured UR compiler
and Cartesian refinement tests pass. Compiler-only and focused runtime
pass (1,148,753 assertions). Projected cone-centre derivatives, branch/
numeric support, default migration and authored performance gates remain
outstanding; authored quaternion rates do not yet supply projected cone
centre rates.

PP4 cone-centre rates: OrientationDifferential.cone differentiates the
minimal tool-Z alignment to each primitive's fixed cone axis with
second-order quaternion jets, including normalization and authored spin.
PosePathRefinement supplies these projected-centre angular rates and
curvature to refinement, including the incoming primitive at joins.
Antipodal alignment is explicitly diagnosed because minimal alignment is
ambiguous there. Independent lattice-centre quaternion differences over
25 small/large authored rotation cases verify centre pose, spatial angular
velocity and acceleration. Compiler-only and focused runtime pass
(1,148,928 assertions). End-to-end cone compiler acceptance, branch/numeric
support, default migration and authored performance gates remain open.

PP4/PP6 end-to-end cone acceptance: a UR FollowPath with a fixed authored
cone axis and changing orientation now compiles through native orientation
sampling, structured selection, spline refinement, projected-centre rates,
shared timing and compiler task-space/trajectory validation. The compiled
group records no numeric pose IK calls. Compiler-only and focused runtime
pass (1,148,930 assertions). This verifies a serial cone fixture, not all
cone/path-policy joins or authored welding missions. Numeric/branch support,
default migration, selector removal and full integration/performance gates
remain outstanding; the overall plan remains active.

PP4 numeric continuation: refinement now supports explicitly diagnosed
numeric-fallback geometry as well as native analytic families. It fits the
selected orientation/external profiles, continues from the previous
physical configuration with one numeric neighbour seed, holds external
coordinates and enforces the same limits/jumps/FK task checks. Numeric
seed IDs are not treated as persistent geometric branches. The planner
retains fallbackDiagnostic instead of rejecting every unsupported family.
An unsupported skew-wrist 6R fixture completes selected-path refinement
and preserves authored task positions. Compiler-only and focused runtime
pass (1,148,937 assertions). This does not add redundancy profiles for more
than six unknown internal DOFs; that differential case remains unsupported.
Authored RobotArm acceptance/performance, branch segmentation, default
migration and full integration gates remain outstanding.

PP4 internal numeric redundancy: numeric refinement with more than six
internal DOFs now selects six independent task-Jacobian columns and fits
selected-route splines to the remaining internal coordinates. Numeric
continuation holds those coordinates together with external axes; the
differential solve receives their prescribed rates and curvature, leaving
six unknown task coordinates. The same chart must remain regular at all
selected knots; chart transitions/rank loss are diagnosed. A seven-axis
fixture now produces complete differential JointPathSamples and preserves
its authored pose-path geometry. Compiler-only and focused runtime pass
(1,148,943 assertions). Chart-transition segmentation, authored-model and
broader derivative/timing acceptance, default migration and final gates
remain open; this is not proof of all redundant arm geometries.

PP4 seven-axis derivative/timing acceptance: independent full-model FK
perturbations cross-check linear/angular task velocity and acceleration
from the internally redundant curve. This exposed artificial angular
curvature from splines fitted to tiny numeric IK residuals in fully
constrained orientation. Refinement now uses zero orientation redundancy
for Fixed/Interpolated tasks, preserving their authored centre derivatives;
free/cone tasks retain their fitted freedom coordinates. The seven-axis
curve now passes these checks, succeeds in one TOPP-RA call, reaches its
refined endpoint and passes native extrema position/velocity/acceleration
limits. externalState also rejects internal profile indices. Compiler-only
and focused runtime pass (1,148,964 assertions). Chart/branch transitions,
default migration and authored full-plan gates remain outstanding.

PP5 refined-path retry: LazyCollisionLadder now accepts a final refined
route checker reporting a blocked sample or incoming transition. It maps
that failure back to the selected candidate/edge and retries within the
same collision budget. StructuredJointPathPlanner runs analytic/numeric
refinement inside this loop and caches the accepted curve, checking its
samples and sweeps before release. Injected refinement failure acceptance
verifies two-round edge rerouting; the integrated XYZ+C planner also
refines the physical obstacle-avoiding free-roll fixture with a clear
sampled sweep. This exposed generic enum comparison rejecting identical
free-spin policies; explicit policy/cone-parameter comparison fixes it.
Compiler-only and focused runtime pass (1,148,967 assertions). Full phase
validation, branch/chart transitions, default migration and authored
performance gates remain open.

PP5 phase-boundary gate started: all 16 native CTests pass on the current
native build. The fresh full MotionKit Haxe module is running without
focused-test environment switches. It has passed new refinement/collision
coverage, existing compiler tests, redundant-arm paths and coordinated
external axes; the asynchronous ProgramPlanner group is still running.
No full-suite pass or PP5 completion is recorded until it terminates.

### PP5 phase gate: runtime evidence and output buffering

- Native CTest passed all 16 tests. A full MotionKit runtime run under GDB completed normally with 1,220,252 assertions, covering the refined collision retries and existing program, timing, runtime and homing groups. The run included temporary diagnostic waits/tracing; those experiments have been removed.
- Redirected HashLink stdout is fully buffered unless `HL_STDOUT_FLUSH=1` is set (`haxeon/vendor/hashlink/src/std/sys.c`). Earlier runs were mistakenly classified as stalled from their last printed group and explicitly terminated. Their sampled stacks showed continuing simulation work; they are unpassed, interrupted runs, not evidence of a planner or GC deadlock.
- Logged runtime gates now set `HL_STDOUT_FLUSH=1`. A normal full gate of the restored production/test code remains to be recorded before PP5 close-out.

### PP6 compiler argument and worker ownership

`ProgramCompiler` now takes `JointPathPlanner` in the former configuration-selector argument; the extra trailing planner argument and compiler selector branch are gone. Structured compiler callers use that argument directly. `forWorker()` always creates a compiler with its own planner and planning assumptions, including when an immutable OPW solver returns itself from `fork()`. Structured planner rebinding accepts both compiled manipulator and OPW adapters.

Compiler-only validation passes. The focused C4 runtime passes 1,148,969 assertions, including direct and worker OPW, UR and Cartesian compilation. Default process-runner migration and deletion of the legacy `PathConfigurationSelector` class (still used by OPW's transport-level `solvePath()` and its tests) remain pending. This is not PP6 close-out.

PP5 normal phase gate now passes on the restored production/test code: 1,220,252 MotionKit assertions, with `HL_STDOUT_FLUSH=1`, and the preceding 16/16 native CTest pass. The run validates PP5 before the subsequent PP6 constructor/worker migration. No planner or GC deadlock was established; the earlier interrupted runs were a buffered-output observation error.

### PP6 legacy selector removal

`PathConfigurationSelector` and all Haxe imports are deleted. Standalone OPW full-orientation `solvePath()` now builds a `CandidateProblem` and searches it with `StructuredLadder`; disconnected paths identify the failing sample and distance. External-group and reduced-orientation transport calls still use the differential solver until production migration. The obsolete selector-specific three-assertion test is removed; the OPW integration test now verifies disconnected-path diagnostics alongside successful branch selection.

Compiler-only validation passes with the class absent. The focused C4 runtime passes 1,148,970 assertions (`process-path-pp6-selector-removal-haxe.log`). Default compiler/process-runner migration, joined approach/retreat sections and k-best starts remain outstanding; PP6 is incomplete.

### PP6 exact logical-axis default

`AxisJointPathPlanner` maps authored positions and first/second path derivatives directly through `AxisKinematics`, retaining motor scales, follower offsets and ratios, and incoming curvature at primitive joins. It validates pinned starts, authored geometry/freedom, reachability and per-joint jumps. `ProgramCompiler` now selects it by default for logical XYZ axis solvers; explicit planners still take precedence and worker planners are rebound independently. `PosePathRefinement` accepts full-free authored primitives, whose quaternion supplies the centre; the serial orientation lattice still diagnoses unsupported full-free sampling.

Compiler-only validation passes. The focused C4 runtime passes 1,148,990 assertions, including coupled line/arc derivatives, incoming/outgoing curvature, inconsistent follower rejection, full-free axis travel and default/worker planner selection. This is partial default migration; compiled serial-arm defaults, process preferences, joined approach/retreat and k-best entry handling remain outstanding. No full phase gate or production speedup is claimed for this increment.

### PP6 process preference costs through collision retries

`StructuredJointPathPlanner` now accepts a per-sample candidate state cost and retains it when rebinding a worker solver. Both direct search and lazy sample/edge/refined collision retries use that cost; disabled candidates still receive infinite cost. This fixes the prior composition gap where lazy clearance replaced all process preferences with zero. Process builders must supply pure, worker-safe callbacks.

Compiler-only validation passes. The focused C4 runtime passes 1,148,993 assertions, including a preferred alternative route with a clear world and a collision retry that excludes that alternative while continuing to avoid the penalized initial route. Serial-arm default migration and construction of authored process preferences remain outstanding; this supplies the common cost plumbing, not their completion.

### PP6 standalone OPW compiler default

Standalone OPW compilers now select `StructuredJointPathPlanner` without an explicit constructor argument, so `lowerPath` sends the selected/refined joint derivatives directly to timing. The existing OPW MoveL acceptance now exercises that default and its worker compilation. Explicit planners take precedence; OPW groups with external axes or a moving work frame still require authored positioning rules and remain pending. Full-free serial orientation sampling remains unsupported and diagnosed.

Compiler-only validation passes; the focused C4 runtime passes 1,148,994 assertions. This verifies the standalone OPW default and existing axis/UR/numeric structured coverage, not the complete PP6 phase gate or process-runner migration.

### PP6/PP7 alternative entry selection

`StructuredLadder.kBestStarts()` retains bounded, ordered complete routes with distinct physical starts. Each route includes the native current-state travel cost; duplicate branch/wrap representations of the same joints are excluded together. A pinned start yields one alternative. Lazy collision selection now optionally sweeps the joint entry from `request.startQ` to the selected first configuration and excludes blocked starts before retrying, preserving process costs and the shared round bound. Structured planners with a free start and a clearance world enable this sweep automatically.

Compiler-only validation passes. The focused C4 runtime passes 1,149,003 assertions, including three ordered free starts, a pinned singleton, fallback from a blocked first entry to the second route, and bounded impossible-entry diagnostics naming the physical pair and sample zero. The compiler still requires its path curve to begin at its input joints: materializing a selected free entry as a separate motion, safe retreat selection and joined approach/retreat remain outstanding. This is entry-search foundation, not PP6/PP7 close-out.

### PP6 free entry emitted as a compiler motion

`JointPathPlanner` exposes whether its configuration permits a free start and accepts a per-call pinned-start override. The first FollowPath section is selected/refined once with a free start; `ProgramCompiler` reuses that curve for timing, generates a separate joint entry when the selected start differs, and pins subsequent sections and MoveL calls. The entry uses the compiler motor-space generator when configured. Both plans keep the authored op index; entry has no path-distance progress. Explicit preceding SetOutput operations retain their order on the entry, while path-distance events stay on the process path. Zero-length entries are omitted.

Compiler-only validation passes. The focused C4 runtime passes 1,149,019 assertions, including entry/path plan count, metadata, event placement, complete joint continuity and omission when already at the selected start. Existing compiler worker, axis, OPW, UR, cone and numeric refinement coverage also passes. The lazy free-entry sweep still checks the straight joint interpolation; generated state-to-state trajectories may follow a different joint curve. Checking that actual entry trajectory and retrying blocked alternatives remains required before PP7 close-out. Safe retreat, joined approach/retreat and remaining defaults/process runners are also incomplete.

### PP7 generated entry trajectories checked and reused

The compiler supplies an entry-check callback to structured free-start selection. Each attempted entry uses the same state-to-state/motor-space generator and coupling projection as the emitted motion. `TrajectoryClearance` samples that generated joint curve at 10 ms and sweeps between those states with ArmClearance; this retains the existing sampled collision contract. A failed entry excludes its physical start and retries within the shared collision round bound. The accepted trajectory is retained and emitted, rather than generated again. Rejected/interrupted candidates and unused zero-motion entries release their native trajectories.

MotionKit and app compiler-only builds pass. The focused C4 runtime passes 1,149,124 assertions. New physical coverage proves that a curved motion with clear endpoint interpolation can hit a mid-motion post; an end-to-end Cartesian cell test rejects the cheapest blocked generated entry, selects a rolled start, and verifies the emitted alternative entry against the physical world. Existing free-entry compiler acceptance now uses a clearance world, including an already-selected zero-motion start. The earlier straight-entry limitation is addressed for compiler-driven free starts. Direct planner callers without an entry callback still request a sampled straight joint entry. Safe retreat selection/emission, joined approach/retreat, process migration and the complete phase gate remain outstanding; PP7 is incomplete.

### PP7 safe retreat integration and full gate

Structured planners accept an optional safe retreat configuration, validate and copy its joints, retain it when rebinding workers, and expose a defensive copy. The last FollowPath section includes travel to that retreat in its route cost. Lazy selection checks the generated exit trajectory after route/refinement checks; blocked physical endpoints are excluded and complete routes retried within the shared round bound. The compiler retains and emits the accepted exit after the process path, with no path progress or inherited events, and omits a zero-motion retreat. This completes PP7's generated joint entry/exit orchestration; the process builder supplies the safe configuration and clearance world.

The first full gate exposed a separate axis-default regression: pinned measured starts were compared against authored positions using a metre tolerance on physical motor joints. AxisJointPathPlanner now checks the measured start's own affine mapping in logical metres, preserving admissible measured starts and rejecting inconsistent followers. The coupled measured-start regression and the existing XYZ gantry integration pass.

Validation on the final behavior:
- Focused C4: 1,149,248 assertions, including entry/process/retreat continuity, event/progress metadata, target ownership and physical checks of both emitted connection motions.
- MachineKit compiler focus: 74 assertions, including the previously failing compiled XYZ gantry.
- Normal full MotionKit runtime: 1,220,521 assertions, exit zero (`process-path-pp7-entry-exit-full-v2-haxe.log`). The preceding failed run is retained separately and is not a passing gate.
- Native CTest: 16/16 pass.
- Compiler-only: MotionKit tests, app, robotkit/cadbridge/tests and toolpathkit/motion/tests pass.

PP6 remains incomplete: joined authored approach/retreat lines, remaining compiled-group defaults and process preferences still need integration. The final timed process trajectory also needs the configured world's clearance check; current structured refinement checks its sampled joint curve, while generated entry and exit motions now receive their actual trajectory checks. PP8–PP11 migration, authored benchmarks and final downstream runtime gates remain outstanding. No production speedup is claimed by this increment.

### PP6 final compiled trajectory clearance

`ProgramCompiler.finish()` now checks the final trajectory through its JointPathPlanner's clearance world, after motor validation and coupling projection and before creating the execution plan. This covers timed process curves and ordinary joint moves as well as entry/retreat motions; it closes the previously recorded gap between checking sampled refined joints and checking the emitted polynomial trajectory. A failure rejects the compilation and identifies the operation, physical pair, distance and required clearance. This is a final acceptance check; it does not run another ladder search after timing fails clearance.

Compiler-only validation passes. The focused C4 runtime passes 1,149,251 assertions (`process-path-pp6-final-clearance-haxe.log`). A physical post fixture proves clear endpoints can still yield a rejected interior collision and verifies a subsequent obstacle-avoiding timed path succeeds using the same compiler. The preceding full 1,220,521-assertion integration gate belongs to `f253269ab`, before this increment; no full-suite rerun is claimed here. Remaining compiled-group defaults, process preferences and joined approach/retreat geometry remain outstanding in PP6.

### PP6 shared motion and roll-change preferences

`StructuredJointPathPlanner` now accepts nonnegative per-joint motion weights and a roll-cell transition cost. The native ladder receives them in direct and lazy sample/edge/refined/entry/exit searches. Retreat travel uses the same joint weights. The planner validates and copies the vector, and worker rebinding retains both preferences. Process builders can therefore price external-axis motion differently from arm motion and penalize roll changes without importing process types into the planner.

Compiler-only validation passes. The focused C4 runtime passes 1,149,264 assertions (`process-path-pp6-weights-haxe.log`), including weighted clear-route costs, bounded collision retry, invalid preference diagnostics, a weighted physical obstacle-avoiding route, caller-vector mutation and equivalent worker refinement. This establishes shared preference inputs; authored process builders and remaining compiled-group defaults still require migration. No full phase gate or production benchmark is claimed for this increment.

### PP9 handling runner migration

`HandlingPlanRunner.create()` now supplies `StructuredJointPathPlanner` explicitly. Its authored home orientation becomes a pure candidate orientation cost, evaluated from FK so numeric fallback candidates receive the preference too. Independent external axes use the held default lattice. The old `solver.preferredOrientation` assignment is removed; handling path compilation no longer calls the legacy path-selection methods. Shared legacy classes remain because the other runners and non-process solver callers are still pending migration.

Validation:
- MotionKit/app compiler-only builds pass.
- Plan-check focus passes 3,292 assertions, including coupled-drive/runtime handling and zero numeric pose IK queries for the Cartesian handling worker.
- The authored robot-arm MuJoCo mission completes all four pick/place steps, with 0.1412808 m lift and 25.46 s cycle (`process-path-pp9-handling-draft-v2/results.json` in external scratch). Per-step planning is 0.524431, 0.494917, 0.540350 and 0.519964 s; numeric pose queries are 3,078, 1,768, 1,810 and 2,770.

This authored case is slower than PP0 (0.189–0.278 s per step and 23.46 s cycle): cycle rises about 8.5%, and numeric candidate generation costs more than the old continuation. No handling speedup or final benchmark acceptance is claimed. The measurement records base revision b90a32b92 plus this uncommitted runner patch; the committed source below has the same runner behavior. Final PP11 benchmarking must use the completed pipeline at a clean revision.

The initial authored attempt stopped on a stale staged app MotionKit library missing `mk_external_lattice_count`, before planner acceptance. Its failed log is retained separately. The already-tested workspace library was atomically staged into the app's generated native directory; no native source changes were made. The rerun passes. PP9 remains incomplete: SurfacePlanRunner, ToolpathPlanRunner, legacy deletions and the other named mission gates are outstanding.

### PP9 surface runner migration

`SurfacePlanRunner.create()` now supplies `StructuredJointPathPlanner`. The separate approach and dwell retain their authored order. The approach chooses the nearest exact FK-verified analytic branch for supported geometry; unsupported geometry retains a single-pose reaching solve with tight tolerances. This prevents a tolerance-sized approach residual from changing the pinned process start during refinement. Joined approach/retreat geometry remains PP6 work.

The first authored run exposed accumulated-length roundoff at a later patch endpoint: subtracting the primitive prefix yielded a local distance slightly above its length. `PosePathRefinement` now bounds the local coordinate after validating the global distance, matching `PosePath.waypointAt()`. A regression checks the decimal-length endpoint, its derivatives and rejection of genuinely out-of-range distances.

RobotKit and MotionKit compiler-only builds pass. C4 passes 1,149,267 assertions (`process-path-pp9-surface-c4.log`). The default-backend WallFinishing mission passes all 71 assertions: three patches, 99.5995% coverage, zero exclusion coverage, maximum tracking error 8.76e-16 m, zero numeric IK queries during measured compilation. Planning takes 5.382185, 6.131609 and 7.554938 s, with cycles 86.119952, 85.144768 and 85.145687 s. These planning times are slower than PP0's 0.91–1.01 s; no speedup is claimed.

Successful records are in external scratch `process-path-pp9-surface-draft-v2/results.json`; they identify ed61ae5ec plus this then-uncommitted patch. The failed initial endpoint run remains separately in `process-path-pp9-surface-draft`. This is focused migration validation, not a full phase gate or MuJoCo WallFinishing acceptance. ToolpathPlanRunner, shared legacy deletion, remaining mission gates and performance work are still outstanding.

### Structured planning profile and reachable periodic bounds

`PROCESS_PATH_PROFILE=1` now emits successful structured-planner phase records: family, sample/candidate counts, construction time, search/check time and refinement time/attempts. The benchmark script retains these alongside its existing run/quality records. Lazy collision refinement time is separated from search/check time; construction includes forward-bound pruning. These are planner phase measurements, not whole compiler timings.

An authored Surface profile (`process-path-pp9-surface-profile` in external scratch) showed 23,041 candidates for a 46-sample UR section. `CandidateProblem.pruneUnreachableBounds()` now removes states outside a preceding layer's per-joint envelope expanded by the existing jump limits before planner search. It is conservative: every legal edge remains inside the envelope, so no complete feasible route is removed. There is no candidate-count cap or preference-based pruning. Raw candidate construction remains available unchanged for model/ladder consumers and comparisons.

Serial refinement now passes its jump window into native periodic lifting, intersected with physical joint limits. It retains every feasible lift while avoiding construction of unreachable wraps. The authored 46-sample section retains 46 search candidates; typical refinement drops from tenths of a second to a few milliseconds. Construction still enumerates the full native wrap set before Haxe forward pruning and is now the dominant measured planner phase; moving these bounds into construction remains useful follow-up work.

MotionKit and RobotKit compiler-only builds pass. C4 passes 1,149,284 assertions (`process-path-reachable-bounds-c4.log`), including equality of optimal route cost/physical joints before and after pruning and exhaustive comparison of feasible native bounded/unbounded lifts. WallFinishing passes all 71 assertions. Profiled planning takes 1.866827, 2.153304 and 2.284093 s versus 5.362379, 5.772040 and 6.764637 s in the preceding profiled migrated run. Cycles and quality records are identical (86.119952/85.144768/85.145687 s; coverage 0.99599466; zero numeric IK queries). This improves the migrated planner by about 3x, but is still slower than PP0's 0.91–1.01 s. It does not establish weld acceptance or full phase completion.

Results are in external scratch `process-path-reachable-bounds-surface/results.json`, with 84 planner profiles and four run/quality records. They record 87e3162f6 plus this then-uncommitted patch. The preceding profile's records were retained from its raw log after adding profile recognition to the benchmark collector. Toolpath migration remains pending: its four-joint excavator geometry and destination-waypoint cutting semantics require explicit handling rather than blindly substituting six-dimensional interpolated paths.

### Native construction with predecessor bounds

The structured planner now enables bounded serial candidate construction. Before each native layer is sampled, preceding retained arm joints define the conservative jump envelope passed to periodic lifting. A pinned first layer uses the existing 1e-7 matching window. Thus native sampling avoids allocating unreachable arm wraps rather than generating all of them for later pruning. Raw `CandidateProblem` construction remains exhaustive for independent consumers and the comparison oracle. The planner still applies forward pruning after construction, including Cartesian/numeric families and external-axis jumps.

External-axis grids keep their authored physical ranges and coordinate indices. Narrowing native external limits to an arm-style jump window would invalidate a legitimate wider grid, so external cells are filtered after sampling. Tests cover moving track/work-frame rules for both OPW and UR and pinned starts with a wider authored grid, as well as exact candidate-set equivalence with exhaustive construction followed by pruning.

MotionKit and RobotKit compiler-only builds pass; C4 passes 1,149,296 assertions (`process-path-bounded-construction-c4.log`). Default-backend WallFinishing passes 71 assertions. Profiled planning is 0.807167, 0.872686 and 0.922503 s, down from 1.866827/2.153304/2.284093 s before this change and near/below PP0's 0.91–1.01 s range. Cycle times, coverage and tracking records remain identical, with zero numeric IK queries. A typical 46-sample section's candidate construction now takes 1–2 ms, retaining 46 candidates. This establishes the Surface improvement; the weld performance acceptance, MuJoCo Surface gate and full phase close-out remain unproven.

External scratch `process-path-bounded-construction-surface/results.json` retains all 84 phase records and four run/quality records, measured at fa4ab9b97 plus this then-uncommitted patch. No native ABI/source rebuild was needed: the existing lift request already accepts narrower arm limits.

### PP8 rate scheduling on the final compilation

`ProgramCompiler.pathEventSchedule` is a pure worker-safe callback invoked after a path section is timed and before its native execution plan is created. It receives whole-operation distances, section-local times, authored events and the last-section flag. Its events are mapped with the same exact time law and receive the normal validation/checks. Worker compilers retain the immutable callback. The compiler rejects events outside the timed section and handles endpoint subtraction roundoff.

`WeldingPlanRunner.launch()` now captures its rate recipe as immutable scalar values and submits the process program once. The worker's actual compilation schedules wire feed directly through `ProcessRateSchedule.timedSection()`, preserving other channels, maintenance feed on already deposited prefixes, exact overlap restoration and final feed. The former synchronous rate-only compilation, disposal and subsequent recompilation of an edited program are removed. A new weld clears the preceding rate callback before candidate verification.

MotionKit, ProcessKit and app compiler-only builds pass. C4 passes 1,149,300 assertions (`process-path-final-rate-c4.log`), including native event clocks from direct and worker compilers. `PROCESSKIT_ONLY=planning` passes welder-process (56), weld-planning (53) and rate-schedule (20) assertions (`process-path-final-rate-processkit-v2.log`). Rate tests cover recovery spanning section boundaries, internal transition rejection and final-endpoint roundoff. These are focused checks; authored runner execution and full phase gates remain pending.

PP8 remains incomplete: `WeldPathPlanner` still has its roll/entry/corner loops and candidate compile-and-verify, and its accepted candidate compilation is not yet retained for execution. This increment removes the distinct rate-scheduling duplicate in `launch()`, not those earlier compilations. Track/weldment speed, clearance, cycle and deposited-leg acceptance still require the completed migration and clean-revision benchmarks.

### Final-clock scheduling: authored gantry-welder acceptance

The authored gantry-welder MuJoCo mission passes at clean revision 1a0e9dc1c, with four runs and all ten seams welded. Cycle time is 182.2 s versus PP0's 182.3 s. Every deposited length and leg-size record matches PP0 exactly (legs 4.995715–5.015761 mm); wire error remains 0.007218779 rad and seam error is 0.00012491246 m. No clearance violation is reported. Records are retained in external scratch `process-path-final-rate-gantry-welder/results.json` and its mission log.

Per-run planning is 3.344234, 23.282064, 19.673083 and 48.681022 s (94.980403 s total), versus PP0's 96.348504 s total. These numbers measure the still-legacy planner's search/verification and do not establish the required 10x weldment improvement. The removed rate precompile occurs afterward. This acceptance verifies actual execution/deposition with final-clock feed scheduling, not completion of the weld problem-builder migration.

### Empty-route exit diagnostics

Tracing safe retreat selection exposed a duplicated exit-check block inside the failed-search branch of `LazyCollisionLadder`: an unreachable ladder with a configured exit check attempted to dereference an endpoint of its empty route. Failed search now immediately returns its native diagnostic (or the existing preceding-collision report). Exit checks still run on successfully selected/refined routes. A regression verifies the failed sample and empty route, and that neither sample nor exit clearance is evaluated without a route.

MotionKit compiler-only validation passes, and C4 passes 1,149,301 assertions (`process-path-empty-exit-c4.log`). The authored gantry-welder result above belongs to the preceding clean revision; its module does not include this independent diagnostic fix. Full phase gates and the remaining PP8 migration remain outstanding.

### PP2 full-free orientation lattice

`OrientationLattice.describe(Free)` now uses the existing native cone lattice with half-angle pi: the tool axis spans the full sphere, with independent spin about each sampled axis. The authored quaternion supplies the lattice centre without becoming a hard orientation constraint. This removes the unconditional serial full-free sampling rejection without changing the native ABI or adding another sampler path.

MotionKit compiler-only validation passes; C4 passes 1,150,514 assertions (`process-path-full-free-lattice-compiler-c4.log`). Tests verify deterministic full-sphere/spin cells, unchanged target positions, inclusion of the opposite tool axis, regular UR full-free selection/refinement with zero numeric pose IK queries, and native validated timing preserving the pinned initial joints.

This establishes regular serial sampling/refinement, not complete PP2/PP4 or remaining compiler-default migration. Antipodal orientation-chart transitions still need segmentation. Underactuated Cartesian groups need their own reachable-orientation treatment rather than assuming a finite full-SO(3) lattice contains their exact fixed/limited orientation. `ManipulatorKinematics` defaults still need preference handling and explicit external positioning rules; they remain unchanged by this increment. PP8 weld search and PP9 toolpath migration remain outstanding.

### PP2/PP4 stable free-orientation frames and pinned cell coordinates

Full-free candidate construction now anchors its rotation lattice to FK of the request's initial configuration throughout the path. Authored positions/freedoms and original quaternions remain in the problem for hard-task validation and explicit preference costs. Unconstrained authored quaternion changes therefore cannot shift orientation-cell labels or exclude a physically continuous route. Refinement uses the selected first TCP rotation as its fixed free-orientation chart centre; its angular rates come from the selected swing/roll splines, not authored angular rates. Tilt extraction uses atan2 of the axis components to avoid acos amplification of roundoff near zero swing.

Pinned native analytic starts now carry the nearest actual orientation-cell coordinates for reduced tasks rather than unconditional zero indices. Their joint configuration remains exact and all joint-jump bounds remain unchanged. Numeric fallback retains its non-lattice orientation metadata. An independent free-spin regression shifts authored roll by pi and verifies the pinned cell is roll 2 of four and its continuation remains connected.

MotionKit compiler-only validation passes; C4 passes 1,150,556 assertions (`process-path-free-chart-spin-c4.log`). Coverage includes antipodal authored full-free orientations through refinement and final native/task-space validation, equal physical curves after changing unconstrained authored rotations, independence from supplied authored angular rates, and XYZ/XYZ+C/XYZ+C+A travel with an unreachable authored rotation while retaining seeded rotary joints. The initial antipodal regression failed before selection because of incorrect pinned cell labels; its failed log is retained separately (`process-path-free-chart-c4.log`). Later focused logs correspond to the subsequent fixes and final source.

This removes artificial antipodes caused by authored orientation and establishes regular underactuated Cartesian free travel using its reachable seeded orientation. It does not prove every Cartesian rotary alternative is sampled, or replace segmentation when the selected physical orientation itself crosses the refinement chart's pole. Those transitions, remaining defaults/preferences, joined approaches, weld search migration and final phase gates remain outstanding.

### PP6 standalone six-axis defaults and request preferences

`ProgramCompiler` now defaults standalone six-axis `ManipulatorKinematics` groups without external axes or a moving work frame to `StructuredJointPathPlanner`. Supported model families use analytic candidates; unsupported geometry retains the planner's explicitly diagnosed numeric candidate fallback. External groups still require process positioning rules, and other joint counts remain pending migration.

The structured planner can bind a manipulator preference source. Each request snapshots its current posture, explicit orientation and authored-target orientation preference. State costs add squared joint displacement and squared FK orientation error to any supplied process cost, including lazy retries and retreat costs. These costs choose among feasible lattice candidates; they do not add hard orientation constraints or guarantee an exact continuously optimized preference. Invalid posture dimensions and nonfinite joints are rejected. Worker rebinding reads the forked solver's preferences and evaluates FK through its planning group, avoiding a closure over the caller's mutable solver.

MotionKit compiler-only passes. C4 passes 1,150,565 assertions (`process-path-adapter-defaults-c4-final.log` in external scratch), including default UR compilation without numeric pose IK, worker compilation, mutable preference snapshots, worker posture independence and invalid preference diagnostics. Focused external posture (3 assertions) and tool freedom (340 assertions) also pass (`process-path-adapter-defaults-posture.log` and `process-path-adapter-defaults-freedom.log`). Those two focus runs precede only the additional snapshot-validation tests; production source is identical. This is focused validation, not a full phase gate. Remaining defaults, joined approach/retreat geometry, physical chart transitions, weld/toolpath migration and all final benchmark gates remain outstanding.

### PP6/PP8 contact policy for joined process geometry

`StructuredJointPathPlanner` now accepts a pure worker-safe TCP pose contact policy in the task frame. It overrides the constant contact flag per actual configuration for selected samples, joint-edge sweeps, refined curves, generated entry/exit checks and the final timed trajectory. Worker rebinding evaluates the policy using its own planning group. `ArmClearance.sweep()` evaluates an optional configuration policy at every interior sample; `TrajectoryClearance` forwards it through its sampled sweeps. This allows one future joined weld path to enforce air margins outside its seam contact zone while preserving contact margins near the work, without process imports in MotionKit.

MotionKit compiler-only passes. C4 passes 1,150,574 assertions (`process-path-contact-policy-c4-final.log` in external scratch). Physical hull regressions distinguish air/contact margins, accept a mixed-contact route and refinement, reject a contact configuration designated as air, check swept interiors, and cover final timed motion and worker rebinding. The first fixture used an insufficient contact-zone width for its hull dimensions and failed; that log is retained separately (`process-path-contact-policy-c4.log`). The corrected fixture passes. The check remains sampled, not a certified continuous collision bound.

This is a prerequisite for joined weld geometry, not its production migration: WeldPathPlanner still has its legacy search and compile-and-verify loops. Joined approaches/retreats, closed-run global starts, branch/chart transitions and weld performance/quality gates remain outstanding. No full phase gate or production speedup is claimed here.

### PP4/PP6 refinement sections on one selected route

Inspection of compiler lowering confirms that sharp-corner timing sections currently select separate ladder routes. Global weld choice must instead span these stops, with one-sided derivatives supplied separately for timing. `AnalyticPathRefiner.refineSection()` now refines any range bounded by selected route knots, seeds it from that range's selected configuration, and retains the same global roll/external redundancy splines. Task coordinates remain global. The existing full-path method still requires the complete range and preserves pinned-start checks; section tasks supply their own endpoint derivatives.

MotionKit compiler-only passes. C4 passes 1,150,853 assertions (`process-path-refinement-sections-c4-final.log` in external scratch). Independent sections reproduce the global geometry/rates and share their physical knot. A stop-and-reverse task verifies equal corner configurations with arrival +0.75 and departure -0.75 rad/m, without a refinement task-velocity discontinuity error. A section whose endpoint is not a selected knot is rejected.

Compiler orchestration still needs to search one whole FollowPath and retain these section curves before timing; this API alone does not change production section selection. Joined weld approaches/retreats, geometric branch/chart transitions and PP8 search deletion remain incomplete. No full phase gate or authored speedup is claimed.

### PP6 one structured selection across FollowPath timing stops

`StructuredJointPathPlanner.planSections()` now builds one candidate problem and one ladder across consecutive timing sections. It validates shared frames/hard endpoint tasks and requires every stop as a sampled knot. One refiner shares roll/external splines across the complete route, returning independent local curves with one-sided derivatives. Lazy sample/edge/refined collision retries retain global sample indices, and entry/retreat costs/checks use the entire route's endpoints.

For structured planners, `ProgramCompiler` now selects/refines all sharp-corner sections of a FollowPath once before timing its first section. It retains the resulting curves while delivering each timed section through the existing worker lookahead, then releases the cache. Generated entry checks use the selected global start; safe-retreat checks use the final endpoint. Timing stops, event section ownership and whole-operation distance offsets retain their prior behavior. This intentionally moves whole-operation geometric work before its first section can be delivered. Affine Axis planning has no discrete choices and retains its existing section lowering.

The initial integration regression caught global-prefix subtraction changing a local distance by one ulp. After checking that the returned grid differs only within 1e-12, compiler caching now restores its original local sample grid; lowerPath's strict planner/timing alignment check remains unchanged. That failed log is retained separately (`process-path-whole-sections-c4.log`).

MotionKit compiler-only passes. Final C4 passes 1,150,875 assertions (`process-path-whole-sections-c4-events.log` in external scratch). Physical mixed-contact sections retain shared joints and locally timed coordinates; missing stop knots are rejected. A two-leg corner compiles to two stopped plans while its state costs receive one global nine-sample ladder. Distance events at the corner and final endpoint belong to the outgoing/final section, and worker compilation uses the same global selection. Tool freedom (340 assertions) and plan check (3,292 assertions) pass (`process-path-whole-sections-freedom.log` and `process-path-whole-sections-plancheck.log`), before only the added frame validation and event/worker regressions.

This completes global section orchestration for structured FollowPath, not all PP6 or PP8: joined approach/retreat operations, remaining group defaults, geometric branch/chart segmentation, WeldPathPlanner search deletion and authored performance/quality gates remain outstanding. No full phase gate or production benchmark is claimed.

### PP9 authored surface acceptance after global section selection

Both WallFinishing backends pass against compiled source at clean production revision f46dd49d3. Compiler-only builds of the standard RobotKit tests and the separate MuJoCo project pass; existing native outputs were used without rebuilding them.

| Backend | Planning seconds (three patches) | Cycle seconds | Numeric pose IK | Coverage | Maximum tracking error |
|---|---|---|---|---|---|
| Default | 0.823407 / 0.964375 / 0.924055 | 86.119952 / 85.144768 / 85.145687 | 0 / 0 / 0 | 0.9959946595 | 8.7595e-16 m |
| MuJoCo | 0.832309 / 0.964837 / 0.985229 | 86.120070 / 85.144929 / 85.145073 | 0 / 0 / 0 | 0.9959946595 | 5.3762e-8 m |

The default-backend mission passes 71 assertions, retaining exact cycle/quality records from the preceding bounded-construction migration and zero exclusion coverage. Planning remains near the PP0 0.91–1.01 s range. Its profile now covers complete 306/308-sample raster routes rather than separately selected corner sections. Records: external scratch `process-path-whole-sections-surface/results.json` and mission log.

The separate MuJoCo runner passes 64 wall-finishing assertions, plus its preceding simulation pose-reset checks, with zero exclusion coverage and RMS tracking error 4.6417e-8 m. Its direct-run log and extracted records are `process-path-whole-sections-surface-mujoco.log` and `process-path-whole-sections-surface-mujoco-results.json`. This closes the previously missing focused MuJoCo Surface gate, not the full PP9 phase gate.

The benchmark harness now includes `wall-finishing-mujoco`, using the separate project's module, standard test cwd and the MuJoCo native workspace libraries ahead of standard backend libraries. The new harness case was executed successfully: 64 assertions, planning 0.824284 / 0.945753 / 0.985022 s and identical cycle/quality records, retained in `process-path-whole-sections-surface-mujoco-harness/results.json`. That collection records f46dd49d3 plus the then-uncommitted harness patch; the compiled mission source is unchanged from the clean direct run.

Weld search migration, toolpath migration, joined approaches/retreats, remaining mission gates, full phase boundaries and the required weld speedups remain outstanding. No weld performance acceptance is inferred from these surface results.

### PP8 joined weld geometry problem builder

`WeldPathProblem` now builds one authored approach/seam/retreat geometric alternative without IK, collision queries, roll enumeration or retries. It retains seam offset/length separately from total air travel for later engagement/event scheduling, preserves existing WeldCorner style/weave construction, and derives task-velocity timing stops through the compiler's now-public `timingSections()` helper. Its global sample request retains every primitive boundary and stop, with configurable spacing. A copied set of seam segments provides the geometric contact-zone policy for actual TCP poses, including nearby approach and burnback lift.

The UR weld fixture now exercises the builder through one `StructuredJointPathPlanner.planSections()` selection/refinement over all three timing sections, using the physical torch/neck/table world and the problem's air/contact policy. It checks deposited length, approach/retreat extent, seam range, boundary joints, every refined TCP position and physical clearance. The joined problem uses zero numeric pose IK queries. This test pins an analytically reachable approach configuration; selection of a free entry from the measured robot state remains production migration work.

ProcessKit compiler-only passes. Focused planning validation passes welder-process (56), weld-planning (269) and rate-schedule (20) assertions (`process-path-weld-problem-planning-clearance.log` in external scratch). The earlier builder check without the physical world also passed and is retained separately. No timing, process engagement/deposition or authored weld speedup is established by this geometric fixture.

WeldPathPlanner and WeldingPlanRunner have not yet switched to the problem builder: their old search/compile-and-verify path remains pending deletion in the production migration. Closed-run global entry positions, reversed alternatives, corner alternatives, external positioning rules, retention of selected geometry for one execution compile and engagement boundaries still need integration. PP8 and the full objective remain incomplete.

### PP8 reversed travel and explicit corner geometry alternatives

Reversed travel is now a domain operation, `WeldPlan.reversed()`, moved out of WeldPathPlanner without retaining a duplicate implementation. The existing planner calls it, preserving its established push/work-angle behavior. `WeldPathProblem.reversed()` creates the opposite complete geometric problem and remaps each corner style to the same physical join in reversed seam order. It snapshots the caller's styles and wrist limits. `withCornerStyles()` builds an explicit geometric alternative without IK, entry enumeration or candidate compilation. These methods do not enumerate a restricted preset set: callers can supply independent styles at every join.

ProcessKit compiler-only passes. Focused planning validation passes welder-process (56), weld-planning (322) and rate-schedule (20) assertions (`process-path-weld-alternatives-planning.log` in external scratch). New coverage checks opposite seam order, deposited length, immutable authored order, caller style mutation, physical corner-style mapping, double reversal restoring orientations/material frames, and distinct corner geometry with unchanged length. Existing reverse-travel push-angle and the joined analytic/physical-clearance fixture still pass.

This establishes geometry alternatives, not global alternative selection or production migration. Free entry from measured state, closed-run entry position choice, external positioning rules, selected-curve execution reuse, engagement scheduling and deletion of the legacy weld search remain outstanding. Weld performance gates and full phase completion remain unproven.

### PP2/PP3 bounded FFI packets for disconnected ladder components

A new free-entry joined-weld regression exposed the FFI's 256 MiB input limit: every `mk_lattice_candidate` reserves 64 joint slots, so a six-axis periodic-wrap ladder can exceed the packet even though the shared native DP supports its state count. `StructuredLadder` now partitions only at proven graph disconnections: a gap in any joint's values over all retained layers greater than that joint's jump bound. It searches all resulting components and compares their complete routes, preserving all candidates, costs, blocked-edge indices and native final/predecessor tie ordering. Recursive partitioning handles large subcomponents. A configurable packet budget can lower allocation size but cannot exceed the FFI safety limit. An oversized connected component is diagnosed explicitly; native streaming/compact candidate transport remains necessary for that case.

MotionKit compiler-only passes, and C4 passes 1,150,894 assertions (`process-path-component-packets-c4.log` in external scratch). Forced small packets match the unpartitioned optimal cost and physical route, including a globally indexed blocked edge and its alternative route. A component that still exceeds the budget is rejected without pruning states. Native source/ABI was unchanged; no native rebuild was made.

The experimental free-entry builder API and regression are still uncommitted and their runtime is still active at the time of this record (HashLink PID 3281053, parent exec session 9925, log `process-path-weld-free-entry-planning-partition.log`). They are not a passing gate. The earlier direct run failed on the packet limit (`process-path-weld-free-entry-planning.log`). The current run has passed that guard but is slow. A two-second perf observation found about 89% of sampled CPU cycles in HashLink `gc_flush_mark`; raw data/report are `process-path-weld-free-entry.perf.data` and `process-path-weld-free-entry-perf-report.txt`. This is diagnostic evidence of allocation/GC overhead, not a benchmark acceptance or evidence that the runtime is terminal. Poll the same live process/session before continuing; do not restart it on observation silence.

Production weld migration, connected native candidate streaming, free-entry acceptance and all required weld speed/quality gates remain incomplete.

### PP2/PP8 allocation reduction and measured-state free weld entry

Component-gap detection now uses a typed float comparator rather than `Reflect.compare` (which boxes each comparison as Dynamic). It compacts only the gap-discovery values: each jump-width integer bin retains its minimum/maximum, its span is verified against the jump bound, and unsafe/large-coordinate cases fall back to the exhaustive typed sort. A verified bin cannot contain a separating gap, so all graph components remain discoverable. Every candidate and cost still reaches search; there is no candidate cap or preference-based pruning.

MotionKit compiler-only passes; C4 passes 1,150,903 assertions (`process-path-component-bin-c4-final.log` in external scratch). Packet-vs-unsplit equivalence includes small drift across a zero bin boundary, global blocked-edge remapping, and coordinates outside integer-bin range requiring the exhaustive fallback. No native source/ABI changed.

`WeldPathProblem.select()` now delegates its complete geometry to the shared structured planner. Its default start is free from the measured configuration; cell external ranges/rules, motion weights, optional posture/orientation source, generated-entry checker and safe-retreat checker remain explicit inputs. Without a supplied preference source it softly prefers authored orientation. Timing and engagement remain outside this geometric selection API.

The original old-source free-entry runtime was explicitly ended with SIGTERM after more than twelve minutes and a perf observation showing dominant GC marking. Parent session 9925 returned actual HashLink exit -15; this is an interrupted diagnostic, not a pass or a proven reachability failure. Its log/data remain retained. The revised source was compiled only after that process was verified terminal, and a new run completed successfully (session 9187, HashLink PID 3325703, exit zero). There are no remaining owned runtime jobs from these checks.

ProcessKit compiler-only passes. Focused planning passes welder-process (56), weld-planning (331) and rate-schedule (20) assertions (`process-path-weld-free-entry-planning-bins.log`). The free UR fixture retains all 481,280 candidates over 94 samples, checks its actual generated entry trajectory against physical clearance, verifies that entry reaches the selected approach start, refines all three joined sections, and makes zero numeric pose IK queries. Measured planner phases are construction 4.977786 s, search/checks 15.634417 s and refinement 0.004292 s (one refinement attempt). The pinned counterpart retains 94 candidates and takes milliseconds.

This establishes free-entry geometric acceptance on the fixture, not production weld migration or acceptable long-weld speed: approximately 20.6 s for this short joined fixture is already too slow for the 2.6 m under-15 s target. Periodic-lift construction/transport/search still need substantial work. Production search deletion, engagement scheduling, retained-curve execution, closed-run global starts, external rules and authored weld speed/quality gates remain incomplete.

### PP2/PP3 candidate transport profile

`PROCESS_PATH_PROFILE=1` now emits `PROCESS_PATH_LADDER_PROFILE` per packet, separating record packing, state-cost evaluation and the native call. The benchmark harness retains these records. Native-call time includes generated FFI marshaling as well as native search; it is not a pure native DP measurement. The opt-in timing does not change candidate inclusion, graph edges or search costs.

ProcessKit compiler-only and its focused planning runtime pass (welder-process 56, weld-planning 331, rate-schedule 20 assertions), retained in external scratch `process-path-free-entry-detail-build.log` and `process-path-free-entry-detail.log`. The free-entry fixture still retains all 481,280 candidates across 94 samples, checks generated entry clearance, and uses zero numeric pose IK. Five packets report:

| Candidates | Candidate bytes | Packing seconds | State-cost seconds | Native-call seconds |
|---|---|---|---|---|
| 105,280 | 111,175,680 | 0.490334 | 0.141254 | 0.504493 |
| 120,320 | 127,057,920 | 4.117916 | 0.130792 | 0.431130 |
| 120,320 | 127,057,920 | 2.767935 | 0.150927 | 0.424311 |
| 120,320 | 127,057,920 | 2.627516 | 0.158072 | 0.432776 |
| 15,040 | 15,882,240 | 0.045152 | 0.013598 | 0.037764 |

Construction is 3.912805 s, search/checks 12.511691 s, refinement 0.004309 s, with one refinement attempt. These are diagnostic observations, not a controlled speedup comparison or long-weld acceptance. Approximately ten seconds of packing makes candidate transport the next optimization target.

Inspection of the pinned Haxeon emitter shows that a 1,056-byte numeric candidate constructor explicitly zeros every byte and reserves 132 pointer-root slots on this 64-bit host. Array marshaling copies each record and inspects its root slots. Raw byte casts cannot safely bypass this: the runtime's `structGetRoots` rejects storage without managed roots. No emitter, runtime or submodule pin was changed. Compact primitive-array transport with native views should preserve all candidates while avoiding per-candidate record/root allocation; connected native streaming remains necessary for very long paths. Neither is implemented by this profile change.

MotionKit compiler-only passes, and the focused C4 runtime passes 1,150,903 assertions (`process-path-ladder-detail-c4-build.log` and `process-path-ladder-detail-c4-runtime.log`). This verifies existing route/cost equivalence and blocked-edge/component behavior after separating cost evaluation from record packing. No native ABI changed or native rebuild was required. PP2/PP3 performance acceptance, production weld migration and full phase gates remain outstanding.

### PP3 compact native candidate views

The exact structured DP now accepts candidate views whose joint and external-coordinate fields point into compact primitive arrays. Lattice keys, validation and joint hashing share the existing algorithms. Coarse/corridor search materializes owning records only for its selected cells and centres, while its complete source can remain compact. Existing full-record inputs retain their complete records unchanged. No public ABI or Haxe transport has changed yet.

The native runtime and focused structured-ladder test target build successfully. The focused `motionkit_structured_ladder` CTest passes. Across 100 deterministic randomized four-joint problems, compact and full-record inputs produce identical exact routes, costs, failed samples and tested-edge counts. Compact coarse search agrees with the complete reference in the tested full-corridor configuration, including blocked-edge remapping. These tests establish the view abstraction, not the long-track performance target or a production packing speedup. Compact FFI input, Haxe packing migration and connected streaming remain outstanding.

### PP2/PP3 compact FFI transport and free-entry measurement

`mk_search_ladder_compact_filtered` now accepts candidate-major primitive joint and coordinate arrays with exact length validation. Each coordinate row contains external cells followed by roll, tilt, azimuth and branch. It shares validation, costs, blocked transitions, diagnostics and Descartes/structured/coarse dispatch with the full-record API. Native search views these arrays directly; wrap/singularity metadata remains in the caller's original candidates. Haxe `StructuredLadder` now uses this API without constructing a managed full record for each candidate. Packet budgeting uses joint/coordinate/cost bytes, with the existing exact disconnected-component fallback retained.

Native runtime and structured-ladder test target build successfully; focused CTest passes. Public compact/full API comparisons cover 100 deterministic randomized problems, identical selected indices, costs, failure diagnostics and backend, blocked edges with direct/coarse dispatch, and malformed joint-array length rejection. Generated bindings pass the portable ABI audit across Linux, Windows, x86-64 macOS and ARM64 macOS (`process-path-compact-hxi.log`). MotionKit compiler-only and focused C4 pass 1,150,903 assertions (`process-path-compact-c4-build.log`, `process-path-compact-c4-runtime.log`). Forced component packet tests now use compact per-candidate sizing, preserving their actual partitioning/oversized-connected coverage.

ProcessKit compiler-only and focused planning runtime pass welder-process 56, weld-planning 331 and rate-schedule 20 assertions (`process-path-compact-process-build.log`, `process-path-compact-process-runtime.log`). The same free UR fixture retains all 481,280 candidates over 94 samples in a single 34,652,160-byte packet (including costs), instead of five full-record packets. Packing is 0.094553 s, preference-cost evaluation 0.589345 s and native call including marshaling 1.090168 s. Planner construction is 3.798700 s, search/checks 1.780278 s and refinement 0.004168 s, one attempt: approximately 5.58 s total measured phases, compared with 16.43 s in the preceding profile. Native call timing is not pure DP time. This is a single diagnostic fixture observation, not a controlled production speedup gate.

Actual generated-entry clearance, all three joined sections and zero numeric pose IK remain verified. Construction now dominates this fixture. Connected streaming/compact candidate generation, production weld migration, engagement integration, full phase gates and the named long-weld/whole-weldment performance targets remain incomplete.

### PP2 compact native sampling output

UR, OPW and Cartesian samplers now expose compact output arrays directly from the existing cell/orientation/branch/lift enumeration. Joint and wrap rows use the actual joint count; coordinate rows contain external cells followed by roll, tilt, azimuth, branch and singular bits. The existing full-record API remains available to native callers. Compact sampling avoids a temporary full candidate array and keeps deterministic order and all metadata. Haxe serial/Cartesian samplers decode these arrays into their existing `LatticeCandidate` objects. Array lengths are checked before allocation, with an explicit streaming-required diagnostic for a layer above the FFI limit.

Native runtime and candidate-sampling test target build successfully, and focused `motionkit_candidate_sampling` CTest passes. Compact/full output comparisons verify every joint, wrap, lattice coordinate, branch and singularity bit for both serial families under fixed/free-spin/cone freedoms and base/shared external transforms, and Cartesian three/four/five-axis families. Invalid compact joint/coordinate lengths are rejected. The generated portable binding audit passes all four targets (`process-path-compact-sampling-hxi.log`). MotionKit compiler-only and C4 pass 1,150,903 assertions (`process-path-compact-sampling-c4-build.log`, `process-path-compact-sampling-c4-runtime.log`).

ProcessKit compiler-only and focused planning runtime pass welder-process 56, weld-planning 331 and rate-schedule 20 assertions (`process-path-compact-sampling-process-build.log`, `process-path-compact-sampling-process-runtime.log`). The same free UR fixture retains 481,280 candidates across 94 samples: construction 0.384632 s, search/checks 1.675969 s and refinement 0.005339 s, one attempt, approximately 2.07 s total measured phases. The immediately preceding compact-search profile measured construction 3.798700 s and approximately 5.58 s total. Current packing is 0.081336 s, preference costs 0.540551 s and native call including marshaling 1.048735 s. Physical generated entry and all three refined joined sections remain checked, with zero numeric pose IK. These single-fixture measurements establish a useful allocation reduction, not the production weldment or 2.6 m performance acceptance.

The native API still counts before generating, and Haxe still materializes the whole problem. Connected streaming and the named scaling gates remain pending. Production weld migration, engagement scheduling, closed-run global start selection, whole-weldment benchmarks and full phase gates remain outstanding.

### PP8 explicit joined-weld engagement boundaries

`WeldPathProblem` now retains approach, deposition, burnback lift and retreat as distinct stopped timing-section groups. Every section carries its engagement phase; corner sections within deposition retain the Weld phase. Global seam-end and burnback-end distances and the lift pose are explicit. Burnback uses the execution runner's lift distance/speed. Group boundaries are retained even when neighboring tangents and speed caps agree, so process stops are not inferred solely from geometric discontinuities. All groups still participate in one global ladder selection.

This also fixes a short-approach geometry mismatch: execution can lift beyond its requested retreat endpoint and return toward the work. The joined builder now preserves that reverse travel. When the retreat coincides with the lift it omits the zero-length final motion. Existing approach/seam/retreat geometry alternatives and physical contact policy remain intact.

ProcessKit compiler-only passes (`process-path-weld-phase-boundaries-build.log`), and focused planning runtime passes welder-process 56, weld-planning 350 and rate-schedule 20 assertions (`process-path-weld-phase-boundaries-runtime.log`). Coverage verifies phase order, burnback speed/distance, all engagement boundaries on the shared sample grid, coincident and shorter retreats, shared refined joints, physical clearance and generated free entry. The free fixture retains 481,280 candidates over 94 samples with zero numeric pose IK: construction 0.376106 s, search/checks 1.726473 s, refinement 0.005381 s. These are geometric acceptance checks; timed engagement/output integration is not established by them.

WeldingPlanRunner still uses its existing production path. Retaining selected section curves for one execution compilation, replacing the legacy weld search, closed-run global entry selection, engagement event integration and the authored performance gates remain outstanding.

### PP6/PP8 retained selected curves for execution timing

`SelectedJointPathPlanner` now adapts globally selected section curves to the existing compiler interface. It snapshots joint curves and authored task data, requires a pinned execution start, and rejects a changed frame, primitive geometry, grid, hard freedom or start instead of running a fallback selection. It verifies retained joints with execution FK and continuity bounds. Grid differences within 1e-12 are aligned to the compiler's exact distances without resampling joints/derivatives. Actual generated/timed trajectory checks delegate to a checking planner, including its pose-dependent contact policy. Worker rebinding retains private read-only snapshots and rebinds checking kinematics; callers receive copied curves.

A structural JSON signature covers the known Line/Arc/Weave primitives, alongside sampled hard-task and derivative comparisons. No serialization dependency or Haxeon change was introduced. The first compile attempts exposed unavailable `haxe.Serializer.run` and `Type.getClassName` in the pinned compiler; the final implementation uses supported JSON and explicit primitive kinds.

MotionKit compiler-only passes (`process-path-selected-curves-build.log`). Final focused C4 passes 1,150,975 assertions (`process-path-selected-curves-runtime.log`). Tests verify copied-return isolation, changed start/grid and free-start rejection, and main/worker compilation into stopped execution plans with the selected physical endpoints. A checking-planner state-cost counter stays zero, proving no candidate reselection during either compilation. The initial runtime assertion incorrectly required two compiler blocks; lookahead batches the two paths into one block. The corrected regression checks the two emitted plans and zero endpoint velocities, which directly establishes stopped-section behavior.

This supplies execution reuse infrastructure, not production weld migration. WeldingPlanRunner/WeldPathPlanner still need to retain the global selection, consume it with engagement outputs, compile once, remove the legacy search and pass authored weldment/long-track performance gates. Full phase gates remain outstanding.

### PP8 complete selected-weld engagement compilation

`WeldPathProgram` now constructs an engagement program from a joined problem and its retained section curves. It emits safe initial wire/arc states, voltage, the selected joint entry, and FollowPath operations for the approach, deposition, burnback and retreat. Arc establishment is awaited before the start dwell/deposition; crater dwell and wire-off precede burnback. Arc-off is an event at the completed burnback lift, including when no nonzero retreat remains. Section operation indices retain the mapping needed for execution progress. Construction does not time, solve or submit motion. Process phases and quantity/end rates are snapshotted.

`ProgramCompiler.withJointPathPlanner()` preserves configured limits, timing, per-joint continuity, couplings, controller period, assumptions, motor constraints and a forked plan check while installing the retained-curve adapter. `WeldPathProgram.compile()` invokes that compiler once, using the geometric contact policy for generated entry/final trajectory checks and scheduling deposition quantity against each final timed section. It does not compile again to build rate events.

ProcessKit compiler-only passes (`process-path-selected-weld-build.log`) and final focused planning runtime passes welder-process 56, weld-planning 452 and rate-schedule 20 assertions (`process-path-selected-weld-runtime.log`). The physical free UR fixture compiles all four retained sections, checks their exact selected endpoints and zero endpoint velocities, retains one ignition wait and both dwells, and verifies arc-off at the burnback endpoint. Integrating the emitted wire rates over their actual plan-clock intervals reproduces authored quantity per seam length within 1e-6. Selection and compilation use zero numeric pose IK. This is a checked compilation artifact, not a submitted mission or whole-weldment benchmark.

The initial test compile used the wrong package for StartTolerances; the corrected source builds and passes. WeldingPlanRunner/ManipulatorMotion still need production submission of this retained compilation and interruption/recovery integration. WeldPathPlanner's legacy search, closed-run global starts and production acceptance remain outstanding.

### PP8 runtime adoption of checked compilations

`CompiledProgram.takeBlocks()` now transfers ownership once; disposing the original artifact after transfer does not free runtime-owned plans. `ProgramPlanner.fromCompiled()` adopts complete aligned blocks, path progress metadata, barriers and notes without creating a worker or recompiling. It retains increasing unused plan IDs and exposes the next unused ID through the existing queue contract. The original worker-planning path remains available for ordinary programs.

`ManipulatorMotion.runCompiled()` validates idle readiness, joint count, unused IDs and a neutral speed override before adoption. It submits the checked plans through the same executor and start checks as normal motion. The existing plan clock and process events remain fixed; a running precompiled queue rejects a speed-scale change requiring retiming. Rejected overrides no longer change the motion's stored override before the queue accepts them.

MotionKit compiler-only passes (`process-path-compiled-runtime-build.log`). The new focused compiled-runtime group passes 32 assertions (`process-path-compiled-runtime.log`), covering actual simulated execution through an input wait/dwell, process on/off events, endpoint arrival, zero additional compilation metrics and no extra planning worker. Disposing the transferred artifact leaves queued motion valid; a second transfer and previously used plan IDs are refused. Existing worker-planner lookahead and manipulator motion checks pass in the same focused run. A normal worker-planned motion also succeeds immediately after the precompiled program, proving ID advancement and lifecycle continuity. The new regression is included in the full suite for the next phase boundary.

This is runtime submission infrastructure; WeldingPlanRunner still needs to submit the selected weld artifact and integrate its phase progress/recovery state. Legacy search removal, closed-run global starts and authored weld speed/quality gates remain incomplete. No full phase completion is claimed.

### PP8 selected-weld progress and barrier identity

Manipulator progress now exposes its active host barrier after a block's plans complete. Existing operation/distance/guarantee fields are unchanged. This distinguishes an input wait from a dwell instead of reporting only operation -1.

`WeldPathProgram.progress()` maps its section operation indices and adopted compilation metadata to explicit approach, ignition, pooling, deposition, crater fill, burnback, retreat and completion states. Deposition coordinates accumulate only Weld section lengths; approach/lift/retreat travel never advances authored seam distance. Ignition and pooling report zero, crater/burnback/retreat report full seam extent. An inactive cursor alone does not establish completion; the caller must supply confirmed execution completion. This provides the progress needed to classify faults and compute recovery backoff for a selected program with multiple section operations.

ProcessKit compiler-only passes (`process-path-weld-progress-build.log`), and focused planning runtime passes welder-process 56, weld-planning 463 and rate-schedule 20 assertions (`process-path-weld-progress-runtime.log`). The checked selected-weld compilation verifies ignition/pooling/crater barrier mapping and deposited versus air/lift progress, including inactive and confirmed-completion cursors. MotionKit compiler-only passes (`process-path-weld-progress-motion-build.log`), and compiled-motion runtime passes 33 assertions (`process-path-weld-progress-motion-runtime.log`), observing actual input-wait and dwell barriers in simulated execution while retaining ownership, clock and ordinary-worker behavior. No native ABI/source changed.

WeldingPlanRunner still needs to activate ProcessRun around a selected compilation and use this mapper for launch/fault/recovery handling. Legacy search deletion and closed-run/corner global alternatives remain unresolved, as do authored speed/quality and full phase gates.

### PP8 activation of an already prepared process program

`ProcessRun.activatePrepared(startDistance)` now activates a caller's checked program without constructing new geometry/events. Initial activation must start at zero; recovery activation must match the current interruption/backoff boundary within 1e-12. It shares readiness, paused-feed, device readiness and stopped/held recovery checks with `takeProgram()`. Rejected activation leaves the process state unchanged. The ordinary builder now uses the same activation path after successfully constructing its program. Prepared activation retains the existing interruption, recovery and completion lifecycle.

ProcessKit compiler-only passes (`process-path-prepared-activation-build.log`). Final focused runtime passes welder-process 66, weld-planning 463 and rate-schedule 20 assertions (`process-path-prepared-activation-runtime-final.log`). New coverage includes activation before preparation, incorrect initial distance, duplicate activation, recovery while motion is running, backoff mismatch, valid stopped recovery and safe completion. Existing program/re-strike checks continue to pass.

The first focused runtime exited with SIGSEGV before completing the welder checks (`process-path-prepared-activation-runtime.log`). A debugger reproduction located an enum comparison against the unstarted null process state in `availableStart()` (`process-path-prepared-activation-backtrace.log`). An explicit null guard now rejects the unstarted process before comparing enum values. The corrected source was rebuilt and the final run exited zero; the failed diagnostic is not a passing gate. No native/compiler changes were made.

The production WeldingPlanRunner launch path still needs to use prepared activation, retained submission and selected phase progress. Legacy weld search deletion, closed-run/corner global choices and authored performance/full phase gates remain outstanding.

### PP8 selected compilation through WeldingPlanRunner

`WeldingPlanRunner.runSelected(problem, curves)` now connects the staged global selection to real runner execution. It compiles the retained program once before device preparation, obtains the first unused runtime plan ID through the new idle-only `ManipulatorMotion.compilationPlanId()`, activates the prepared ProcessRun when ready, and submits the existing compilation through `runCompiled()`. Selected progress mapping drives seam travel/post-weld classification. Prepared artifacts are disposed on abort/preparation failure and released after transfer without double-freeing runtime-owned plans. Common preparation/recipe construction is shared with the existing entry point.

ProcessKit compiler-only passes (`process-path-selected-runner-build.log`). Focused planning runtime passes welder-process 66, weld-planning 467 and rate-schedule 20 assertions (`process-path-selected-runner-runtime.log`). The free UR fixture now executes through a SimulatedRobot and the actual WeldingPlanRunner preparation, ignition, deposition, burnback and completion lifecycle. Runtime-fired arc events drive the supplied electronic arc feedback. The runner completes, leaves the arc off, needs no restart and reports zero additional motion compilation/numeric pose IK at launch. Selected compilation retains the physical cell clearance checker. The test does not simulate deposited bead geometry or prove an authored mission speedup.

The staged API consumes caller-selected curves; its planningSeconds covers retained compilation/preparation, not the preceding caller selection. Default `run(requested)` still calls the legacy planner. Recovery still builds through the existing ProcessRun path and needs migration to a newly selected retained recovery problem. This intermediate API is not final compatibility or completion of PP8: default selection, recovery, legacy search deletion, closed-run/corner global choices and authored performance/quality gates remain outstanding.

### PP8 recovery geometry in original seam coordinates

WeldPathProblem.recovery(distance) now slices the authored pose path and builds its approach at the slice's actual start pose. It retains original seam extent and restart distance separately from remaining deposition length. This avoids regenerating corner turns or resetting weave phase from a shortened WeldPlan. Out-of-range and completed-seam restart positions are rejected.

ProcessKit compiler-only passes (`process-path-recovery-geometry-build.log`); focused runtime exits zero with welder 66, weld planning 477 and rate schedule 20 assertions (`process-path-recovery-geometry-runtime.log`). Added checks compare remaining path positions/orientations with the corresponding original coordinates. This is recovery geometry preparation, not runtime recovery migration: selected-curve matching still needs support for sliced primitives, engagement/rate scheduling must use the original-coordinate overlap, and WeldingPlanRunner must select and compile recovery from stopped state. Default planner replacement and the remaining PP8 acceptance gates remain outstanding.

### PP6/PP8 retained compilation of exact path slices

The exact primitive interval wrapper now lives in MotionKit as PoseSlice. ProcessPathSlice uses it to preserve source geometry and parameterization. SelectedJointPathPlanner recognizes slices recursively, comparing both interval boundaries and the known source primitive's structural geometry; unsupported source types remain rejected. Slice construction rejects nonfinite boundaries and derivative evaluation rejects out-of-range distances.

ProcessKit compiler-only passes (`process-path-slice-compile-build.log`). Focused runtime exits zero: welder 66, weld planning 478, rate schedule 20 (`process-path-slice-compile-runtime.log`). Added coverage globally selects a sliced recovery problem and compiles its retained sections with physical clearance checks, with zero numeric pose solves. This establishes recovery curve compilation, not recovery execution: restart engagement, original-coordinate rate overlap and runner submission are still pending, alongside default planner migration and all remaining phase acceptance requirements.

### PP8 retained recovery engagement and overlap scheduling

WeldPathProgram accepts an explicit original-seam interruption coordinate for recovery. It restrikes at WeldArcModel.MIN_WIRE_SPEED, omits the initial pooling dwell, retains crater/burnback/retreat, and schedules maintenance feed over the covered prefix before restoring timed deposition. Each Weld section receives its own remaining overlap extent; progress and completion report original seam coordinates. A sliced problem without explicit recovery engagement is rejected, as are invalid interruption coordinates.

ProcessKit compiler-only passes (`process-path-recovery-engagement-build.log`); final focused runtime exits zero with welder 66, weld planning 482 and rate schedule 20 (`process-path-recovery-engagement-runtime-final.log`). Tests inspect compiled barriers and native timed rate events, require restoration before the endpoint, and verify original-coordinate progress/completion. A test initially referenced a nonexistent durationNs property; the corrected comparison uses ExecutionPlan.durationSeconds. This proves recovery artifact engagement, not runner recovery submission or deposited-bead acceptance. WeldingPlanRunner integration, default planner replacement, global alternatives and remaining authored/full phase gates remain outstanding.

### PP8 selected-run recovery submission

WeldingPlanRunner now retains the selected WeldPathProblem and uses the retained pipeline for its recoveries. After the motion stops and the device clears, it reads ProcessRun.preparedStart(), slices the original authored seam, selects the remaining joined approach/deposition/burnback/retreat from measured stopped joint positions, compiles one recovery engagement artifact, activates the same ProcessRun at that backoff boundary, and submits via runCompiled(). Recovery selection/compilation are included in the runner's cumulative planning metrics. Old transferred compilation metadata is released when replaced; failure/abort/new runs clear selected state.

ProcessKit compiler-only passes (`process-path-runner-recovery-build.log`). Final focused runtime exits zero: welder 66, weld planning 487, rate schedule 20 (`process-path-runner-recovery-runtime-final.log`). The simulated selected runner now executes both an uninterrupted weld and a weld with one injected mid-deposition arc loss. Recovery completes, activates exactly BACKOFF before the interruption in original seam coordinates, leaves the arc off, and records zero worker planning/numeric pose solves. Feedback is electronic test feedback, not a deposited-bead mission. Generated entry and final timed motion receive compilation clearance checks; lazy generated-entry rejection during recovery selection still needs integration so another entry can be selected before compilation.

Default run() remains on WeldPathPlanner's legacy search; selected-run recovery does not complete PP8. Global closed-start/corner alternatives, legacy deletion, production reporting, authored performance/leg-size/collision/cycle acceptance and full phase gates remain outstanding.

### PP7/PP8 lazy generated-entry checks during recovery selection

ProgramCompiler.generateEntry() exposes the same stopped transition generator used by its FollowPath lowering, including drive limits, motor-space generation, coupling projection and stationary controller-period holds. It validates finite aligned joint configurations and returns a caller-owned trajectory. Selected-run recovery now generates and checks that actual transition in the ladder's entry callback, using the same pose-dependent contact policy and sampled physical clearance contract. A rejected entry can therefore select another route before the single retained execution compilation.

ProcessKit compiler-only passes (`process-path-recovery-entry-build.log`); final focused runtime exits zero: welder 66, weld planning 488, rate schedule 20 (`process-path-recovery-entry-runtime-final.log`). The sliced recovery fixture deliberately rejects its first generated entry and verifies another entry is checked before successful retained compilation. Uninterrupted and injected-arc-loss selected runner simulations still complete. This is focused recovery validation; default legacy planner replacement, global alternatives, authored acceptance and full phase gates remain outstanding.

### PP8 default runner selects and retains the joint path

WeldingPlanRunner.run() now builds the joined WeldPathProblem, performs global structured selection with lazy generated-entry clearance checks and current cell preferences, and compiles/submits through the retained pipeline. Initial planning metrics include selection and compilation. The runner's legacy ProcessRun.takeProgram()/worker-recompile launch and old single-FollowPath progress bookkeeping have been removed; recovery uses the same selection helper. Runtime no longer invokes WeldPathPlanner.plan(). Selected reports preserve checked sample count and end joints while explicitly marking continuous selection; MissionPlayer reports a globally selected joint path instead of inventing constant per-segment rolls.

Final ProcessKit and app compiler-only builds pass (`process-path-default-weld-build-final.log`, `process-path-default-weld-app-build-final.log`). Final focused ProcessKit runtime exits zero: welder 66, weld planning 489, rate schedule 20 (`process-path-default-weld-runtime-final.log`). The uninterrupted simulated weld now calls ordinary run() and verifies its selected report; explicit selected execution with injected arc loss still passes. During cleanup an assignment was removed from the wrong constructor; compilation caught it and the corrected final sources pass.

This is a production runner migration with temporary remaining geometry limitations, not final PP8 acceptance. It currently selects the authored start and AROUND corner geometry; global free closed-run start, reversed and corner alternatives remain to integrate. The standalone WeldPathPlanner and its legacy tests/CAD planning still exist and must be replaced/deleted. Unsupported model-family proof, authored weldment/track performance, leg-size/margin/collision/cycle gates and full phase/downstream suites remain unproven. No speedup or unchanged authored mission acceptance is claimed.

### PP8 authored track failure exposes frozen external axes

At clean revision 1bcbc7fa8, the authored track MuJoCo gate fails before arc-on: first approach layer has no reachable candidates. The sampler's omitted external ranges hold the rail at its seed. Actual terminal exit is 1; no run or quality records were produced (`process-path-default-track/results.json`, `track-weld.log`). Its sole ladder profile records 1,342 samples, 145,195 candidates, 12,196,380 packet bytes, packing 0.03782 s, preference 0.26854 s and native-call 0.00560 s. This failed selection does not prove any planning speed or quality gate.

Default runner selection now supplies every external joint's complete finite compiled planning range, sampled with spacing at most half its jump bound. No range is narrowed to a heuristic window and no candidate-count cap is introduced. Nonfinite external bounds require a diagnostic. ProcessKit and app compiler-only pass (`process-path-weld-external-ranges-build.log`, `process-path-weld-external-ranges-app-build.log`); focused ProcessKit runtime exits zero with 66/489/20 assertions (`process-path-weld-external-ranges-runtime.log`). Those runtime fixtures have no external joint and do not establish track correctness. The corrected authored track gate still must run, and its resulting lattice size/performance may require the outstanding streaming implementation. All remaining phase and acceptance requirements stay open.

### PP2 unsupported-family seed reuse after external holding

NumericBranchIk now eliminates exact duplicate solve seeds after applying held external coordinates. Neighbours that differed only in those coordinates previously caused identical numeric queries for each cell. Every distinct resulting configuration remains; the first original seed index is retained. JSON keys are only buckets: exact coordinate comparisons guard against key collisions, so no approximate seed pruning or candidate cap is introduced.

MotionKit compiler-only passes (`process-path-numeric-seed-reuse-build.log`); focused C4 runtime exits zero with 1,150,977 assertions (`process-path-numeric-seed-reuse-runtime.log`). The external-cell test supplies three neighbours collapsing to one held configuration, verifies exactly one numeric query and the first original branch identity. This reduces duplicate fallback work, not the unresolved RobotArm family proof or analytic performance gate.

The authored track gate at 1bf8fb1a6 was started separately before this edit (`process-path-external-track`). At this closeout its original runtime PID 3540931 and harness session 33952 remain live, with no run/quality records yet. Do not restart it on an observation timeout or report it as passing. It uses the preceding app module and therefore cannot measure this seed-reuse change. The next continuation must poll the same handle and inspect its actual terminal result before deciding the next benchmark action. Remaining migration and acceptance requirements stay open.

### PP8 compare authored and reversed geometric problems before timing

StructuredJointPathPlanner exposes the objective of its last successful complete ladder route, cleared at each new selection. WeldPathProblem.selectWithCost() returns that objective with the selected sections and problem; its existing select() delegates to it. Default WeldingPlanRunner selection now evaluates the authored geometry and its one reversed-travel alternative with the same measured start, lattice, preferences and generated-entry/clearance checks. It retains the cheaper feasible selection (authored direction wins exact ties), then compiles only that choice. If neither direction is feasible, both diagnostics are retained. Recovery keeps the initially selected travel direction and original seam coordinates.

ProcessKit compiler-only passes (`process-path-weld-direction-build.log`); focused runtime exits zero with welder 66, weld planning 490 and rate schedule 20 assertions (`process-path-weld-direction-runtime.log`). Added coverage selects both directions against the physical fixture and verifies finite shared-ladder objectives. Existing default-run and injected recovery execution tests pass. This does not add global closed-start or per-corner geometry choices, replace the standalone legacy planner, or satisfy authored speed/quality and full phase gates.

The preceding authored track process (PID 3540931, session 33952, module at 1bf8fb1a6) remains live after re-polling; no result or quality records exist yet. Continue observing that same handle rather than restarting on timeout. It predates numeric-seed reuse and reversed selection, so its eventual results apply to that earlier revision.

### PP2 candidate-construction progress and obsolete track closeout

With PROCESS_PATH_PROFILE=1, CandidateProblem now emits its family/unsupported diagnostic before sampling and progress after completed layers at one-second intervals or completion. Records include completed/total samples, retained candidates, numeric-query count and elapsed construction time. The benchmark harness retains both new record types; required run/quality records remain mandatory. Without the opt-in flag there is no new output.

MotionKit compiler-only passes (`process-path-build-progress-build.log`); profiled focused C4 exits zero with 1,150,977 assertions (`process-path-build-progress-runtime.log`). Its log contains 79 start and 79 progress records, parsed successfully, including the numeric-fallback fixture's five completed samples, 37 candidates and 91 numeric queries. Harness Python syntax validation passes; its generated validation cache was removed.

The earlier track run at 1bf8fb1a6 was still live after nearly eight minutes with no selection/run/quality records. Its unsupported-family neighbour/cell sampling predates the tested exact-seed deduplication and the new construction diagnostics. After confirming PID 3540931's exact app-module command, it was deliberately terminated as an obsolete diagnostic run, not restarted on an observation timeout. Harness session 33952 is now terminal: runtime exit -15, harness exit 1, zero records and missing run/quality (`process-path-external-track/results.json`). Debugger attachment had failed with a ptrace rejection, so no live stack trace was obtained. This is an incomplete authored gate, not performance or quality acceptance. No owned benchmark/compiler/runtime job remains live at this closeout. Rebuild the current app before the next authored diagnostic so the seed reuse and progress changes are actually included. All unresolved original requirements remain open.

### PP2 explicit orientation lattice for numeric fallback candidates

The current authored diagnostic at 85a656751 identifies numeric-fallback for TrackWelder: its RobotArm wrist lines do not intersect for OPW and j4 violates UR's parallel-axis pattern. Construction completed only four of 1,342 samples in 49.911 s, retaining 17,853 candidates after 97,650 numeric queries. Layer totals grew from 466 at sample two to 3,097 at three and 17,853 at four. Reduced numeric solves from every neighbour were producing additional arbitrary free-spin states instead of sampling the declared orientation lattice. These are actual construction records, not projected total runtime or acceptance results.

CandidateProblem's fallback now samples OrientationLattice cells, solves each cell with Fixed orientation, and preserves its roll/tilt/azimuth coordinates exactly as the analytic samplers do. Distinct numeric seeds and solutions are retained; there is no count cap or approximate pruning. Pinned starts remain exact and use the nearest authored orientation-cell coordinates. This corrects the reduced-task candidate model; it does not establish a supported analytic RobotArm family or guarantee finite solution count for genuinely redundant unsupported groups.

Final MotionKit compiler-only passes (`process-path-fallback-lattice-build-final.log`); focused C4 runtime exits zero with 1,151,083 assertions (`process-path-fallback-lattice-runtime-final.log`). Added skew-wrist reduced-task coverage verifies nonempty states, explicit roll indices and FK orientation agreement with each declared cell. Existing numeric continuation, held-axis, analytic and compiler checks pass.

After collecting the growth evidence, the obsolete diagnostic PID 3567932 was deliberately stopped with SIGTERM following exact command verification. Harness session 64574 is terminal: runtime -15, harness 1, four diagnostic records, missing run/quality, wall 208.702 s (`process-path-track-current-diagnostic/results.json`). No deposited-weld quality or planning-speed gate passed. No owned jobs remain live. The current app still needs rebuilding for this lattice change before another authored gate; remaining global geometry choices, standalone legacy removal, all authored acceptance/full phase gates and the original PP1 proof stay unresolved.

### PP1/PP8 corrected authored lattice still misses track planning gate

The current app was rebuilt successfully at 48a08f3a5 (`process-path-finite-track-app-build.log`) and the track MuJoCo diagnostic used that module. It reports the actual unsupported-family reasons: OPW wrist lines arm/j4–j6 do not intersect, and UR j4 violates the parallel-axis pattern. Explicit orientation sampling retained 201 states at layer one (3,472 numeric queries, 1.82149 s) and 661 cumulative states at layer two (56,544 queries, 29.19087 s). Thus construction alone has conclusively exceeded the under-15-second planning requirement after only two of 1,342 layers. Finite orientation cells correct the candidate model but do not make multi-seed numeric fallback meet the authored performance requirement; no final total is extrapolated.

After the measured construction gate failure, the exact command of PID 3587281 was verified and that diagnostic was deliberately terminated. Harness session 38744 is terminal: runtime -15, harness 1, three diagnostic records, missing run/quality (`process-path-finite-track/results.json`). This establishes a planning-time failure, not deposited leg, clearance, cycle or complete execution results. No owned jobs remain live.

The user has been asked which RobotArm model to target: preserve its 35 mm offset wrist and add a suitable solver, or change it to the documented spherical wrist. No geometry change is authorized by an unanswered preference request; retain the model meanwhile. The original goal remains active: supported authored-family proof and track performance still need implementation, alongside global closed-start/corner choices, standalone legacy deletion, other runner migration and all remaining full phase/acceptance gates. This evidence does not justify a smaller completion target or fallback-only acceptance.

### PP1 model-derived middle-axis offset wrist forward geometry

OffsetWristGeometry derives an OPW axis pattern with a displacement along the middle wrist axis, while retaining the physical model. It isolates six internal rotary joints from upstream external axes, recovers the canonical reference using the existing geometric axis-line helpers, extracts the middle-axis displacement, and calibrates the flange/tool transform. Its forward result adds the displacement along the middle wrist axis derived from OPW orientation and the final wrist angle, then applies the actual cell arm-base transform. Unsupported axis patterns/displacements are rejected. This class exposes forward geometry only; it is not registered as an inverse family, and no numeric fallback is falsely relabelled analytic.

MotionKit compiler-only passes (`process-path-offset-geometry-build-final.log`); focused C4 runtime exits zero with 1,151,164 assertions (`process-path-offset-geometry-runtime.log`). Added coverage displaces the final wrist axis by 35 mm along the middle axis in the shifted-reference OPW fixture, verifies extraction, and compares positions/rotations against the compiled model over 40 configurations. The fixture mutation is restored before existing spherical-wrist tests. Constructor probes also verify independent joint motions.

This establishes the offset-family forward formula on that fixture, not yet the authored RobotArm extraction/round-trip requirement. Next work must verify the actual authored arm and implement its inverse branch enumeration/native sampling and refinement before any authored speed claim. The outstanding user geometry preference does not authorize remodeling; the model remains unchanged. Original phase, global geometry-choice, legacy deletion and downstream/acceptance requirements remain open. No owned jobs remain live.

### PP1 authored RobotArm offset forward-model proof

The new arm-offset-geometry app selector loads the actual RobotArm generated project and its mounted tool, derives OffsetWristGeometry from the compiled group, checks the physical 35 mm offset, and compares position/orientation over 100 joint configurations sampled within real limits. It preserves the existing arm-analytic inverse selector rather than passing that unmet requirement through a forward-only substitute. RobotArm's source description now accurately names its offset wrist and axis-line separation; no mechanical geometry changed.

App compiler-only passes (`process-path-authored-offset-build.log`); the focused authored runtime exits zero (`process-path-authored-offset-runtime.log`), after decoding the actual 38-component assembly, and prints 100 forward comparisons passed. Position and orientation each agree within 1e-6 and inverse query count stays unchanged. This proves the new forward formula on the authored arm, not inverse completeness, native sampling/refinement integration, track performance or full phase acceptance. Next solver work can now use the verified middle-axis displacement instead of pretending it is spherical. The user preference remains unanswered, so mechanical geometry is retained. All remaining original requirements stay open. No owned job remains live.

### PP1 offset inverse reduction at a known final-wrist angle

OffsetWristGeometry.inverseSlice() transforms a target into its canonical flange frame, removes the middle-axis displacement for an explicitly chosen canonical final-wrist angle, and evaluates the resulting OPW branches. Each probe reports the wrapped difference between its solved final angle and the chosen angle. A zero residual establishes the consistency condition for this reduction. External-cell coordinates are preserved. Probes are explicitly not complete inverse enumeration, legal periodic lifts, or a registered analytic family.

MotionKit compiler-only and focused C4 pass: 1,151,244 assertions (`process-path-offset-slice-build.log`, `process-path-offset-slice-runtime.log`). Across 40 shifted-reference offset fixtures, the slice at the known actual final angle contains the original configuration modulo turns and its consistency residual is zero. App compiler-only and authored runtime also pass (`process-path-authored-offset-slice-build.log`, `process-path-authored-offset-slice-runtime.log`): 100 actual RobotArm forward and known-angle inverse-slice comparisons, with unchanged numeric-query count. Geometry remains unchanged.

This proves the inverse reduction, not finding the final angle from an unknown target. Root isolation/completeness, singular cases, stable branch identities, periodic lifts, native bulk sampling and refinement remain necessary before replacing fallback or claiming PP1/PP8 acceptance. Track performance, other global geometry choices, legacy removal, downstream/full phase gates and the entire original objective remain incomplete. No owned job remains live.

### PP1 unknown-angle scalar root-search prototype

OffsetWristGeometry.simpleRoots() now explores consistency roots without receiving the solution's final angle. It scans native OPW branch residuals, bisects sign changes, refines possible nearby-root intervals and follows branch validity boundaries. Returned roots are deduplicated modulo rotary turns and verified against the full offset forward target. This remains an explicitly incomplete prototype outside BranchIk/default sampling: narrow validity islands, tangent roots, singular continuums and complete root isolation are not certified, and legal periodic lifts/native integration are absent. Its finite scan/heuristic refinement must not substitute for final family completeness.

The initial scan failed fixture sample 39. Diagnostic probes show branch 7 appears near theta=1.4181677; the original root was skipped because a scan endpoint had no branch. Midpoint/quadratic refinement alone did not fix it. Adding validity-boundary refinement does. The failed runtime logs (`process-path-offset-roots-runtime.log`, `process-path-offset-roots-runtime-final.log`, `process-path-offset-roots-counterexample-runtime.log`) have actual exit 1 and are not passing evidence.

Final MotionKit compiler-only passes (`process-path-offset-roots-boundary-build.log`); focused C4 exits zero with 1,151,284 assertions (`process-path-offset-roots-boundary-runtime.log`). All 40 shifted-reference offset fixtures now recover their original configuration modulo turns from target poses and zero arm seeds, without supplying the known final angle. This is fixture evidence for the reduction/search approach, not actual RobotArm unknown-angle proof, complete inverse enumeration or any performance gate. The authored unknown-angle checks and rigorous/native solver implementation remain next, alongside all unresolved original plan requirements. No owned jobs remain live.

### PP1 authored RobotArm unknown-angle recovery

The arm-offset-geometry selector now runs simpleRoots() against all 100 authored RobotArm target poses using zero arm seeds, without supplying each target's actual final angle. It checks that the original configuration is among returned roots modulo turns, in addition to the earlier forward and known-angle slice checks. App compiler-only passes (`process-path-authored-roots-build.log`), and the actual authored runtime exits zero (`process-path-authored-roots-runtime.log`), printing 100 forward, known-angle slice and unknown-angle root comparisons passed. Numeric pose-query count remains unchanged.

This extends the prototype's evidence to the real mounted arm. It is not an all-roots/completeness certificate and does not satisfy the original OPW round-trip selector, production branch semantics, periodic limits, singular continuums, native sampling/refinement or track planning-time acceptance. The prototype remains outside default family selection. Those solver requirements and every other unresolved original phase/acceptance deliverable remain open. No owned jobs remain live.

### PP1 reuse exact offset-root probes and immutable target frames

The root prototype now transforms the target into its canonical arm/flange frame once per search and reuses OPW probe sets for exactly equal scalar angles. String keys only select cache buckets; exact Float equality guards reuse. Validity-boundary and interval refinement retain their existing logic and all accepted roots still pass full offset FK verification. inverseSlice() shares the same local-frame reduction helper.

MotionKit compiler-only passes (`process-path-root-reuse-build.log`); focused C4 exits zero with 1,151,284 assertions (`process-path-root-reuse-runtime.log`), retaining all 40 unknown-angle fixture recoveries and their boundary counterexample. This removes repeated frame evaluation/native probes, not root-completeness uncertainty. The preceding 100 authored-arm recovery proof belongs to d6c546a22; it was not rerun for this cache increment. No prototype or production speedup is measured or claimed. Rigorous root isolation, singular/branch/periodic handling, native family and sampler/refiner integration, all authored/full phase gates and the original whole-plan objective remain outstanding. No owned jobs remain live.
