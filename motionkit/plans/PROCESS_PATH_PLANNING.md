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
| PP0 | planned | — |
| PP1 | planned | — |
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
