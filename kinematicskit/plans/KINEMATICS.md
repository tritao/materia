# KinematicsKit — one kinematics core for assemblies and robots (handoff)

**Goal:** replace the three kinematics engines Materia has grown with one
kit, and grow that kit from "serial chain + one pose" into "compiled
kinematic tree + tasks + limits", the formulation Lane D already chose.

Today there are three engines, each with its own linear algebra:

| Where | Model | What it does |
|---|---|---|
| `cadkit/haxe/src/cadkit/modeling/AssemblyState.hx`, `AssemblyLoopSolver.hx` | `materia.assembly.AssemblyDefinition` (projectkit) | Forest of occurrences joined by tree joints; couplings (`target = ratio * source + offset`); root poses; closure joints solved as residuals (never FK edges) by Levenberg-Marquardt with trial-step acceptance, tolerance-scaled residuals, **finite-difference** Jacobians, rank-based DOF count and `converged` / `limit-blocked` / `conflicting` / `nonconvergent` statuses. |
| `robotkit/haxe/robotkit/manipulation/KinematicChain.hx`, `InverseKinematics.hx` | `robotkit.model.RobotModel` | One base-to-tip serial chain; analytic 6×n Jacobian; fixed-damping DLS IK with clamping. FK runs twice per iteration; every call allocates `Array<Array<Float>>`. |
| `motionkit/robot/haxe/motionkit/robot/ManipulatorKinematics.hx` | `Manipulator` | A second DLS normal-equation solver for `solveDifferential`, plus a hand-written flange→TCP Jacobian shift because the TCP is not a frame. |

The CAD engine and the robot engine use the same joint convention
(`parent · parentFrame · motion(axis, q) · inverse(childFrame)`), so one
compiled model serves both.

By the end of this plan:
- CadKit assembly FK and closure solving, RobotKit FK/IK/Jacobians and
  MotionKit's `ManipulatorKinematics` all run on `kinematicskit`;
- one problem can hold several frame targets on a branching tree, partial
  (masked) targets, posture and damping tasks, closures and joint limits;
- the solver reports *why* it stopped, not just `converged = false`;
- a native QP backend (Lane D's D1–D3) solves the same problem description.

Read first:
1. `motionkit/plans/LANE_D_REDUNDANCY_SERVO.md`: the mink-shaped task/limit
   design this plan implements. Its D1–D3 move here (see K3).
2. `robotkit/plans/HUMANOID.md` H7: floating base and whole-body control on
   the Lane D QP.
3. `robotkit/ARCHITECTURE.md`, "Joint-frame convention".
4. Code: the files in the table above, plus
   `projectkit/src/materia/assembly/AssemblyFrames.hx` (`axisMotion`,
   `compose`), `AssemblyDefinition.hx` (joint types, roles, couplings),
   `motionkit/haxe/motionkit/kinematics/KinematicsSolver.hx` (the planner
   contract, unchanged by this plan), `motionkit/robot/.../OpwKinematics.hx`.

Work in `../materia-worktrees/kinematicskit` on branch `kinematicskit`.
Failing test first, one commit per item, merge to `main` after each green
item, append to the Progress log, and stop and log whenever this plan turns
out to be wrong.

## Decisions

- **KK-D1 — The kit depends on nothing.** No RobotKit, CadKit, projectkit or
  MotionKit imports. It owns small math types (`Transform`, `Vec3`-like
  values) and flat `Array<Float>` workspaces. It is unit-agnostic: lengths
  are in the model's unit, and tolerances are passed in that unit.
- **KK-D2 — Authored models compile into it; compilers live with the
  authored model.** `robotkit.kinematics.RobotKinematics.compile(RobotModel)`
  and `cadkit.modeling.AssemblyKinematics.compile(AssemblyDefinition)`. Each
  keeps the map from its stable IDs (`JointId`, occurrence ID, `FrameId`,
  connector) to compiled indices; indices never leak into public RobotKit or
  CadKit APIs.
- **KK-D3 — Tasks and limits, not null-space projection.** Each task yields
  a residual and Jacobian rows with a weight; limits are hard. Redundancy is
  resolved by weighted secondary tasks (posture, damping), as in mink. Strict
  priority levels can be added later if a real case needs them.
- **KK-D4 — Closures are never FK edges.** FK walks the spanning forest;
  closures are equality tasks (the CadKit rule, kept).
- **KK-D5 — Couplings resolve at compile time.** A coupled joint shares its
  source's DOF with `value = scale * q[dof] + offset` (chains compose). The
  coupled joint's limits narrow the source DOF's range.
- **KK-D6 — Solvers never mutate their inputs.** They return a
  `KinematicSolution`; the caller commits (editor command, `AssemblyState`,
  MotionKit). This is what lets the editor solve on a preview state.
- **KK-D7 — Behaviour is preserved by solver policy, then changed on
  purpose.** The existing RobotKit DLS iteration and the CadKit LM iteration
  become two named policies, so migrated callers keep their answers.
  Switching a caller to a different policy is its own commit with its own
  test changes.
- **KK-D8 — Haxe first, native for the heavy solver.** K0–K2 are Haxe
  because they replace Haxe code one-for-one. The QP backend (K3) is native
  (MotionKit decision D1: numerical code is native), behind the same problem
  description, with the Haxe solver as reference and fallback.
- **KK-D10 — The native core uses Eigen, privately.** Eigen 3.3+ through
  `find_package(Eigen3 3.3 REQUIRED NO_MODULE)`, as MotionKit already does
  (no new dependency or licence). Rules:
  - Eigen never appears in the C ABI: `kinematicskit.h` takes flat
    `double*` arrays and sizes; Eigen is linked `PRIVATE`.
  - No allocation per solve: workspaces (`MatrixXd`/`VectorXd`,
    decompositions reused with `.compute()`) are sized once per
    model/problem shape; native tests build with `EIGEN_RUNTIME_NO_MALLOC`
    and solve under `set_is_malloc_allowed(false)`.
  - Fixed-size types for spatial values (`Quaterniond`, `Vector3d`, 6-vectors),
    dynamic sizes for Jacobians.
  - Quaternions cross the ABI as (x, y, z, w); `Quaterniond`'s constructor
    takes (w, x, y, z). Convert only through two helpers, covered by a
    round-trip parity test against the Haxe snapshot.
- **KK-D9 — Out of scope:** collision (a validator interface outside the
  kit), time parameterization and trajectories (MotionKit), dynamics,
  character IK (`animkit`/`humankit`), and the OPW analytic solver (stays a
  specialist `KinematicsSolver` in MotionKit).

Ownership after this plan:
- **kinematicskit:** compiled model, state, snapshot (FK, Jacobians),
  tasks, limits, solvers, solution diagnostics.
- **RobotKit:** `RobotModel` → model compiler, joint groups, `Manipulator`
  (tool/TCP semantics, `toJointTargets`), runtime.
- **CadKit:** `AssemblyDefinition` → model compiler, `AssemblyState`
  (authored coordinates, records), closure-solve API.
- **MotionKit:** `KinematicsSolver` contract and adapters, OPW, paths,
  configuration selection, timing. It never commands IK output directly;
  IK output goes through planning and validation like any other target.

## Layout

```
kinematicskit/
  haxeon.json            no dependencies
  README.md
  plans/KINEMATICS.md    this file
  haxe/kinematicskit/
    Transform.hx         position + unit quaternion value type
    JointKind.hx         Fixed, Revolute, Prismatic
    KinematicModel.hx    immutable compiled forest (+ builder)
    KinematicState.hx    q + root poses
    KinematicSnapshot.hx world poses and Jacobians for one state
    (K1) problem/, solver/, linalg/
  tests/
    haxeon.json          pure Haxe, no native build: the fast loop
    src/KinematicsKitTests.hx
```

Every consumer manifest gains `"kinematicskit": {"path": "../kinematicskit"}`
(haxeon resolves dependencies transitively, so only direct users need it).
`cadkit/scripts/test-haxeon` lists source roots by hand and needs the new
root too.

## K0 — Compiled model and snapshot; both engines' FK on it

Do:
- `KinematicModel` built through `KinematicModelBuilder`:
  - bodies (a forest; each root's pose comes from the state, default from
    the model);
  - joints: parent body, child body, `parentTJoint`, `jointTChild`, kind,
    unit axis in the joint frame, DOF index (−1 when fixed) with coupling
    `scale`/`offset`;
  - DOFs in authored order: ID, rotational/translational, optional lower and
    upper limits, `continuous` flag;
  - frames: ID, body, `bodyTFrame` (RobotKit frames, CadKit connectors);
  - closures: ID, two frames, kind (Fixed/Revolute/Prismatic), axis,
    optional tolerance — stored now, solved in K1;
  - compile-time checks: one incoming tree joint per body, no cycles, unit
    axes, finite transforms, coupling targets not also driven, coupled
    limits intersected into the source range.
  - precomputed topological order and, per body, the ancestor joint list
    (Jacobians only visit those).
- `KinematicState`: `q` in DOF order + root poses.
- `KinematicSnapshot.evaluate(model, state)` → world pose of every body and
  frame, and per movable joint its world origin and axis, into reused flat
  arrays. Then:
  - `bodyPose(i)`, `framePose(i)`;
  - `jacobian(frame, ?pointOffset)` → 6×n geometric Jacobian in world
    coordinates, linear rows at the frame origin (or the offset point),
    coupled columns summed with their `scale`.
- `robotkit.kinematics.RobotKinematics.compile(RobotModel)`:
  continuous → revolute without limits; `JointLimits` with `lower >= upper`
  → unlimited (the `JointGroup` convention); a floating base only marks its
  root pose as variable (used in K5).
- `KinematicChain` becomes a façade: compile once, evaluate once per call
  (fixing the double FK in IK), same public API and exceptions.
- `cadkit.modeling.AssemblyKinematics.compile(flattened definition)`;
  `AssemblyState.forwardKinematics` / `worldPose` / `worldConnector` read a
  snapshot. The loop solver is unchanged in K0.

Tests (kinematicskit, fast loop):
- Planar 2R and prismatic FK against closed form; UR5-style zero pose.
- Branching tree with two tips and two roots evaluates every body.
- Jacobian against central finite differences on a random tree with
  revolute, prismatic, fixed and coupled joints, at frames and offset points.
- Compile errors: two parents, cycle, non-unit axis, driven coupling target.

Acceptance: RobotKit `KinematicsTests` and the CadKit assembly smokes
(`AssemblyModelSmoke`, `AssemblyNestingSmoke`, `AssemblyDocumentsSmoke`,
`AssemblyFocused`) pass unchanged; the machinekit robot-arm preview still
builds and its FK check passes.

## K1 — Tasks, limits, solver, diagnostics; migrate all three engines

Do:
- `kinematicskit.linalg`: one dense workspace-based implementation of normal
  equations with damping, LDLᵀ/Cholesky solve, pivoted rank, norms. Deletes
  the three hand-rolled Gaussian eliminations.
- Tasks (each: residual rows, Jacobian rows, weight, tolerance):
  - `FrameTask(frame, target, ?offset)` with position and orientation masks
    expressed in a chosen basis, so "position + tool Z direction, roll free"
    is one task;
  - `RelativeFrameTask(frameA, frameB, target)` with the same masks;
  - `ClosureTask` built from a model closure (Fixed = 6 rows, Revolute =
    3 position + 2 axis rows, Prismatic = 2 transverse + 2 axis rows,
    exactly CadKit's residuals);
  - `PostureTask(qRef, weights)`, `DampingTask`.
  A frame reference is a model frame plus an optional extra offset, so a TCP
  needs no recompiled model.
- Limits: `ConfigurationLimit` (from the model; always on). Handled by an
  active set: a DOF at a bound whose step pushes outward is frozen for that
  step and reported, instead of being clamped every iteration.
- `KinematicProblem`: active DOF selection (the rest stay at the seed),
  tasks, limits.
- Policies (KK-D7):
  - `DampedLeastSquares` — fixed damping, reproduces
    `InverseKinematics.solve` iteration for iteration;
  - `LevenbergMarquardt` — adaptive damping with trial-step acceptance and
    rollback, tolerance-scaled residuals, step limit, prismatic scaling:
    CadKit's algorithm with analytic Jacobians.
- `KinematicSolution`: status (`Converged`, `LimitBlocked`, `Conflicting`,
  `IterationLimit`, `NumericalFailure`), state, iterations, residual norm,
  per-task position/orientation residuals and pass/fail, Jacobian rank and
  remaining DOF count, limit hits.
- Migrate:
  - `InverseKinematics.solve` → problem + `DampedLeastSquares`; `IKResult`
    filled from the solution.
  - `Manipulator.solveIkForTcp` → `FrameTask` at flange + TCP offset.
  - `ManipulatorKinematics.solveDifferential` → one differential step on the
    same Jacobian (TCP as a frame offset; the manual shift goes away).
  - `AssemblyLoopSolver.solve` → closure tasks + `LevenbergMarquardt`; keeps
    `AssemblyLoopSolveResult`, its status strings and "state untouched on
    failure".

Tests:
- Old-vs-new parity: every existing RobotKit IK case gives the same `q`
  within 1e-12 under `DampedLeastSquares`; CadKit closure cases give the same
  status, DOF count and closed configuration within tolerance (analytic vs
  finite-difference Jacobians can differ in the last digits; state the bound
  used).
- Masked task: a 6R arm reaches a position + approach axis with roll free,
  from seeds where the full-pose task fails.
- Limit-blocked target reports `LimitBlocked` and names the joint.
- Conflicting closures report `Conflicting` with the rank-based DOF count.

## K2 — Several targets on one tree; editor preview API

Do:
- Several frame tasks in one problem on a branching model; DOF selection by
  joint ID (e.g. "arm" vs "arm + torso").
- `AxisAlignmentTask` / look-at (a frame axis points at a world point).
- No per-iteration allocation: a solver workspace sized once per
  (model, problem shape); measure with the allocation census.
- A preview entry point for the editor: solve from the authored state,
  return the solution, never write back (KK-D6).

Tests:
- Dual-arm fixture reaches two targets in one solve; each arm alone matches
  the single-target solve.
- Arm + wrist camera: pose task on the tool, look-at task on the camera.
- Allocation count per iteration is zero after warm-up.

## K3 — Native QP backend (Lane D D1–D3, rehomed here)

Do:
- `kinematicskit/native`: C ABI that takes a compiled model once, then per
  solve the state, task rows and limits; FK/Jacobian and one QP step
  natively. OSQP first, behind a backend interface; Eigen as in MotionKit.
  Vendoring follows the Ruckig/TOPP-RA rules (pinned submodule, audited file
  list, `THIRD_PARTY.md`); with no network, stop and log.
- `KinematicsSolver` adapters in MotionKit use it for `solveDifferential`
  (Lane D LD-D2); the Haxe solver is the fallback with a diagnostic.
- Velocity limits; joint-limit avoidance; posture/elbow preference as tasks.
- mink oracle tests (skip with a message when Python/mink is absent).

Tests: parity with the Haxe solver on K1/K2 fixtures; 7-axis fixture with
posture preference; mink agreement within tolerance.

## K4 — Retire the façades

Do: migrate the remaining callers of `KinematicChain`, `InverseKinematics`,
`IKResult`, `ChainTip` (robotkit tests, `DigCyclePlanner`, toolpathkit and
motionkit tests) to the kit or to `Manipulator`, then delete them.
`JointGroup` stays in RobotKit as a named selection that builds a DOF
selection.

## K5 — Variable root poses (mobile and floating bases)

Do: optional root-pose variables in the problem (planar 3-DOF for a mobile
base, 6-DOF for a floating base), so a mobile manipulator can solve base +
arm together. H7's whole-body control (contacts, centre of mass, Pinocchio
dynamics) stays in `HUMANOID.md`; this item only makes sure the kit does not
block it.

## E1 — Editor IK gizmo (separate track, after K2)

Select a frame, drag a target gizmo, solve with the preview API, show a ghost
and per-task residuals / limit hits; commit through an undoable editor
command. For a design-mode assembly the commit updates `AssemblyState`
coordinates; for a running robot the target goes through MotionKit.

## Progress log

### K0 — Compiled model and snapshot (2026-09-30)

- `kinematicskit` exists with no dependencies: `Transform`, `Vector3`,
  `JointKind`, `ClosureKind`, `KinematicModelBuilder` → `KinematicModel`,
  `KinematicState`, `KinematicSnapshot` (world poses, joint origins/axes,
  frame/body/point Jacobians into caller-supplied arrays). Its own suite
  (`tests/`, pure Haxe, ~5 s) has 104 assertions, including Jacobians
  against central differences on a tree with revolute, prismatic, fixed and
  coupled joints and a side branch.
- **HashLink limit found:** the JIT rejects calls with more than 32
  arguments (`MAX_TMP_ARGS` in `hashlink/src/jit_emit.c`: "JIT ERROR
  get_tmp_args ... Too many arguments"). The builder hands the model a single
  `KinematicModelParts` object instead of 34 constructor arguments.
- `Transform.checked` accepts a quaternion whose squared norm is within 1e-4
  of one (the `AssemblyCodec.validateFrame` tolerance) and normalizes it.
  Assembly FK used to compose such frames unnormalized.
- RobotKit: `robotkit.kinematics.RobotKinematics.compile` (whole model,
  including `RobotModel.couplings`) and `.path` (base→tip only, no couplings,
  which is how chains always behaved). `KinematicChain` is now a view over
  `.path`, with one FK per call. Parity against the previous implementation
  (copied into a scratch project) on a UR5 and a random mixed-joint chain
  with rotated joint/child frames: FK and Jacobians within 6e-15, IK `q`
  within 6e-14 with identical convergence and iteration counts in all 80
  cases. FK + Jacobian is 3.7× faster (70 ms vs 257 ms for 20 000 calls).
- CadKit: `cadkit.modeling.AssemblyKinematics.compile` (occurrences →
  bodies, connectors → frames, tree joints, couplings, closures).
  `AssemblyState.forwardKinematics`/`worldPose` read a snapshot; the
  recursive tree walk is gone. **Behaviour change:** a coupled joint's pose
  now always follows its source; before, FK used the joint's stored
  coordinate, which could only differ when a definition's coupled defaults
  are inconsistent (and `record()` already rejects that state).
  `AssemblyLoopSolver` is unchanged and runs on the new FK.
- Acceptance: kinematicskit tests; RobotKit world tests (all suites, 4762
  assertions, plus a new whole-model-vs-chain FK test); CadKit
  `HaxeonSmoke` (all assembly smokes); MachineKit smoke including the
  robot-arm FK check; MotionKit and cadbridge suites.
