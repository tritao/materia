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
- **KK-D11 — Each solver keeps the job it is good at (revised in K4b).**
  - **Tracking** (sequential IK along a path, jogging, gizmo dragging):
    damped least squares. Its *fixed* damping, and stopping as soon as the
    tolerances are met, keep each step small, so joint paths stay smooth
    near singularities. Switching RobotKit's IK to Levenberg-Marquardt broke
    a wall-finishing path near a wrist singularity (K4b log), so this is a
    property to keep, not a legacy quirk. The K3 QP (damped differential IK
    with posture and limits) is its successor for tracking.
  - **Reaching** (closing assembly loops, a far target from a cold start):
    Levenberg-Marquardt, whose adaptive damping and trial steps converge
    reliably and whose diagnostics say why they did not.
  - DLS's `IterationLimit`-without-final-check quirk is still bug-compatible
    and gets fixed on its own, with its own test changes. Deleting DLS waits
    until the QP covers tracking.
- **KK-D12 — After K3, native leads and Haxe follows.** New solver features
  land natively. The Haxe solvers stay as the reference and fallback and must
  agree with native in parity tests; they gain only what those tests need.
- **KK-D13 — One QP solver across the stack: ProxQP, dense backend.**
  Kinematic QPs here are small and dense (one variable per DOF, 6 to about
  50; tens of rows, nearly all non-zero). OSQP, a first-order sparse solver,
  converges to modest accuracy with iteration counts that vary, which suits
  large sparse problems and not a fixed-rate servo loop. ProxQP
  (proxsuite) has an Eigen-native dense backend (KK-D10) and is what the
  humanoid whole-body controller (H7, TSID) will use, so K3 and H7 share one
  vendored solver and one set of behaviours to learn. No osqp-eigen: the
  backend interface already hides the solver, and converting to a solver's
  matrix format is a few dozen lines. OSQP stays out unless a large sparse
  case appears (e.g. whole-body control with many contacts). Before
  vendoring, confirm proxsuite's licence, pinned version and dependency set,
  and that its dense solver builds without optional extras; if it does not,
  stop and log, with DAQP (small C dense active-set solver) as the fallback
  to evaluate.
- **KK-D14 — Dynamics stays out; TSID + Pinocchio own it (H7).** TSID
  (stack-of-tasks) solves torques, accelerations and contact forces per
  control tick on Pinocchio's rigid-body dynamics; that is humanoid H7 in
  RobotKit's native runtime, not this kit. The kit keeps its task vocabulary
  close to TSID's (`FrameTask` ~ SE3 equality task, `PostureTask` ~ joint
  posture task) so a goal authored once can drive a kinematic solve or TSID.
  Once Pinocchio is in the build, its FK and Jacobians become a test oracle
  for the kit. Pinocchio does not replace the kit: the kit must run in Haxe
  without native code (editor, CAD design mode) and handles CAD closures and
  couplings.
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

## Known limits to document, not fix now

- `FrameTask`'s orientation Jacobian is first order (exact at convergence),
  as RobotKit's IK always was; `ClosureTask` uses the exact SO(3) derivative.
- `KinematicState.q` is a public mutable array. Solvers copy their seed;
  callers sharing one state must copy before mutating.
- Pose types are still duplicated (`Transform3`, `Pose3`, `AssemblyFrame`,
  `kinematicskit.Transform`); the kit converts only at its boundary.
- Haxeon's incremental build once produced a wrong kit test binary (an
  unchanged pure-arithmetic test failed after an unrelated edit; a clean
  build was correct). Until that is diagnosed (separate session), run the kit
  suite from a clean `tests/build`.

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
- **Jacobians over active DOFs only.** Today every task writes
  `rows × model.dofCount()` even when three DOFs are active, which is fine
  for an arm and wasteful for a MachineKit assembly with hundreds of joints
  solving one linkage. Tasks write only the problem's active columns (the
  problem hands them a column map); snapshots may skip joints that no task
  depends on. Change the `KinematicTask` interface now, while it has few
  implementations.
- ~~Cache compiled assembly models.~~ Measured instead (K2 log): compiling
  is ~4% of `AssemblyState` construction; not worth a cache.

Tests:
- Dual-arm fixture reaches two targets in one solve; each arm alone matches
  the single-target solve.
- Arm + wrist camera: pose task on the tool, look-at task on the camera.
- Allocation count per iteration is zero after warm-up.
- A 200-joint assembly solving one 3-DOF linkage: the Jacobian workspace is
  sized by the active DOFs, and the solve time does not grow with the
  unrelated joints.

## K3 — Native QP backend (Lane D D1–D3, rehomed here)

Do:
- `kinematicskit/native`: C ABI that takes a compiled model once, then per
  solve the state, task rows and limits; FK/Jacobian and one QP step
  natively. ProxQP's dense backend first (KK-D13), behind a backend
  interface; Eigen as in MotionKit.
  Vendoring follows the Ruckig/TOPP-RA rules (pinned submodule, audited file
  list, `THIRD_PARTY.md`); with no network, stop and log.
- `KinematicsSolver` adapters in MotionKit use it for `solveDifferential`
  (Lane D LD-D2); the Haxe solver is the fallback with a diagnostic.
- Velocity limits; joint-limit avoidance; posture/elbow preference as tasks.
- mink oracle tests (skip with a message when Python/mink is absent).

Tests: parity with the Haxe solver on K1/K2 fixtures; 7-axis fixture with
posture preference; mink agreement within tolerance.

## K4 — Retire the façades (soon after K2, before K3)

Do:
- Migrate the remaining callers of `KinematicChain`, `InverseKinematics`,
  `IKResult`, `ChainTip` (robotkit tests, `DigCyclePlanner`, toolpathkit and
  motionkit tests) to the kit or to `Manipulator`, then delete them.
  `JointGroup` stays in RobotKit as a named selection that builds a DOF
  selection.
- **`Manipulator` works on the whole compiled model**, not a base-to-tip
  path. `RobotKinematics.path` ignores `RobotModel.couplings` because chains
  always did, while `compile` honours them; until this lands the two can
  disagree on a coupled robot. The tool centre point becomes a frame offset
  on a `FrameTask` (no flange-target conversion). Delete `path` with the
  chain.
- **The two authored models of one machine agree.** The robot-arm example
  exists as a CAD assembly and, through cadbridge, as a `RobotModel`. Compile
  both and check that the tool frame's forward kinematics matches over
  random joint values, so the two descriptions cannot drift apart silently.

Tests: a coupled robot's IK through `Manipulator` moves the follower; the
robot-arm CAD/robot FK agreement check; every former façade caller passes.

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

## K6 — Consolidation after Lane D (2026-10-01)

Lane D added working pieces as separate special cases. K6 folds them into
one structure before collision checking (which should plug into one group
and one solver, not four IK paths).

- **KK-D15 — One kinematic group.** RobotKit's `KinematicGroup` is the
  joints from a root link to a tool frame, plus optionally the joints to a
  work frame (a positioner). Poses are in the work frame when there is one,
  else in the root (base) link's frame. It names its external axes (damped),
  its redundancy (`ArmSwivel`) and its base motion (K5). `Manipulator` is
  the fixed-base, no-work-frame case (a subclass keeping its constructor).
  IK has one entry point, `solve(target, seed, options)`. `IkOptions`
  carries the tolerances, the TCP or flange target, the method (tracking,
  reaching, prioritized), a swivel goal, a preferred posture and a moving
  base. `solveIk`, `solveIkForTcp`, `solveIkAtSwivel`, `solveIkWithBase` and
  `CoordinatedGroup` go.
- **KK-D16 — Prioritized solves are exact.** `PrioritizedSolver`
  (pure Haxe, so the editor and the browser keep IK) solves each iteration
  as an equality-constrained least-squares step. Hard tasks are equalities,
  soft tasks (posture, swivel preference, DOF damping) are the objective,
  and limits are an active set. This replaces the two-pass posture solve.
  It revises KK-D3 for this one solver: priorities are exact equalities,
  not weights. A ProxQP twin in the native core is added only if speed
  calls for it.
- **KK-D17 — Step limits live in the kit.** The servo's per-tick bounds
  (position, velocity, acceleration, braking, ramped chunks) become a
  kinematicskit `StepLimits`, used by `DifferentialIk` with a
  `FrameVelocityTask`. `ManipulatorServo` keeps only its API.
- **KK-D18 — Solvers own their path search.** MotionKit's `PathSolver`
  interface (`solvePath`) is implemented by OPW (native selection), by
  redundant groups (the redundancy lattice) and by a generic sampling
  selector. `ProgramCompiler` asks the solver; it no longer switches on its
  type.
- **KK-D19 — One redundancy resolver.** `RedundancyParameterization` names
  the motion left after the tool pose: the swivel of a 7R arm, the values
  of a cell's external axes, later a mobile base's pose. Each can report
  its values for `q` and solve at given values. `RedundancyResolver` runs
  D3's pipeline over any of them: grow a lattice along the path, select the
  cheapest route (Descartes), smooth, re-solve exactly.

Steps, each its own commit with all suites green:
- K6a: `KinematicGroup` + `IkOptions`; migrate callers; `Manipulator`
  subclass; `CoordinatedGroup` deleted.
- K6b: `PrioritizedSolver`; the group's preferred-posture solve uses it.
- K6c: `StepLimits` + `FrameVelocityTask`; `ManipulatorServo` on
  `DifferentialIk`.
- K6d: `PathSolver`; `PathConfigurationSelector` becomes the generic
  sampler.
- K6e: `RedundancyParameterization` (swivel, external axes) +
  `RedundancyResolver`; D3 and D6 paths both go through it.

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

### K1 — Tasks, limits, solvers, diagnostics (2026-09-30)

- Kit: `LinearAlgebra` (normal equations, pivoted Gaussian solve, damped
  step, rank; fixed summation order), `KinematicTask` (residual = desired −
  current, Jacobian rows over all DOFs, hard/soft), `FrameTask` (position
  mask in the target frame; `FrameOrientation.Full | Axis | Free`),
  `ClosureTask` (CadKit's residuals; exact Jacobians for position and fixed
  rotation rows via the inverse left Jacobian of SO(3), first order for axis
  and transverse rows), `PostureTask`, `KinematicProblem` (active DOFs,
  limit overrides), `DampedLeastSquares`, `LevenbergMarquardt`,
  `KinematicSolution` (status, per-task errors, rank, free DOFs, limit hits).
  Kit suite: 148 assertions (closure Jacobians against central differences,
  masked and axis tasks, posture choosing the elbow branch, four-bar closing,
  conflicting and limit-blocked cases, seeds untouched).
- **Deviations from the item as written:** no `DampingTask` (damping is a
  policy setting); no active-set limit handling yet — both policies clamp,
  as their originals did. Limits are diagnosed from the final state instead
  (next point).
- **Deliberate behaviour changes in the CadKit algorithm** (KK-D7):
  - exhausted damping (no step of any size lowers the residual) is
    `Conflicting`; CadKit called it nonconvergent with a message claiming a
    descent direction remained;
  - `LimitBlocked` whenever a non-converged solve ends with an active DOF on a
    bound and the descent direction pushing it outward. CadKit only reported
    it when the very last iteration stalled on a limit, so a four-bar with
    its rocker limited short of closure came back `nonconvergent`.
- Migrated: `InverseKinematics.solve` (problem + `DampedLeastSquares`; its
  matrix helpers are gone), `ManipulatorKinematics.solveDifferential`
  (`KinematicChain.pointJacobian` at the TCP + `LinearAlgebra.dampedStep`;
  the manual flange→TCP shift and `solveLinear` are gone), and
  `AssemblyLoopSolver.solve` (closure tasks + `LevenbergMarquardt`; the
  finite-difference Jacobian, residual code and linear algebra are gone;
  `finiteDifferenceStep` is still validated but unused).
- Parity (scratch harnesses with the previous code copied in as `legacy`):
  - IK on four chains, 80 cases: `q` within 5e-14, identical convergence
    and iteration counts.
  - Differential IK at a TCP offset, 80 cases: within 8e-12 relative (damping
    1e-6 amplifies last-digit Jacobian differences).
  - Loop solver: excavator, four-bar, slider-crank and a fixed-closure chain
    converge with the same iteration and DOF counts, joints within 1e-10; an
    unclosable four-bar fails the same way; the limited four-bar now says
    `limit-blocked` (above). 30-100x faster (excavator 0.1 ms vs 11 ms).
- CadKit had no loop-solver tests; `AssemblyLoopSmoke` adds the four-bar,
  slider-crank, fixed chain, unclosable loop and limit cases, with "state
  untouched on failure" checks.
- Haxeon incremental-build miscompile seen here (see "Known limits").
- Acceptance: kit tests (clean build); RobotKit world tests (4773
  assertions); CadKit `HaxeonSmoke` with `AssemblyLoopSmoke`; MachineKit
  smoke; MotionKit (6723); cadbridge (129).

### K2 — Active columns, allocation-free iterations, several targets (2026-09-30)

- **Active-column Jacobians.** `JacobianLayout` (one column per active DOF)
  is owned by `KinematicProblem`; `KinematicTask.evaluate` now takes it and
  writes `layout.width` columns. `KinematicSnapshot.pointJacobianColumns`
  only visits the body's ancestor joints that have a column. A four-bar
  solved inside a 203-DOF model uses a 5 x 2 Jacobian and gives the same
  answer and iteration count as the four-bar alone.
- **Zero allocation per iteration.** `FlatTransform` (shared flat-array
  transform arithmetic), `Rotations` writing into caller arrays,
  `SolverWorkspace` (buffers and snapshot sized per problem shape, reused
  across solves; both solvers take one optionally), in-place
  `LinearAlgebra.solveInPlace` / `normalEquationsDense`. Tested with
  `hl.Gc.totalAllocated()`: a 60-iteration solve allocates exactly as much
  as a 10-iteration one, for DLS (frame + posture tasks) and LM (closure).
  Note for haxeon/HashLink code: an optional `Int`/`Float` parameter is boxed
  per call, so hot helpers take required parameters (`LinearAlgebra.norm`).
- **Several targets and DOF selection.** `KinematicProblem.setActiveJoints`
  (by joint ID). Dual-arm fixture: both hands in one solve with a shared
  torso; one arm alone leaves the torso and the other arm at the seed.
- **`LookAtTask`**: a frame axis points at a world point (2 rows; the line of
  sight's own motion is in the Jacobian). Fixture: a hand holds its height
  while its X axis looks at a part.
- **Editor preview API**: nothing new was needed. Solvers never write back
  (KK-D6), `FrameTask.setTarget` / `LookAtTask.setTarget` move targets
  without rebuilding the problem, and a reused `SolverWorkspace` makes a
  per-frame solve allocation-free after warm-up.
- **Model caching dropped after measuring:** on the excavator (13
  occurrences, 16 joints) `AssemblyState` construction takes ~2.2 ms, of
  which compiling the kinematic model is ~87 µs (4%) and FK ~14 µs; the rest
  is definition validation and flattening, a CadKit concern outside this plan.
- Parity harnesses rerun after the refactor: unchanged from K1 (IK `q`
  within 5e-14, identical iterations; loop solver identical statuses,
  iterations and DOF counts).
- Acceptance (clean Haxe outputs): kit tests (159 assertions); RobotKit
  world tests (4773); CadKit `HaxeonSmoke` incl. `AssemblyLoopSmoke`;
  MachineKit smoke; MotionKit (6723); cadbridge (129).

### K4a — `Manipulator` on the whole robot; chain layer deleted (2026-09-30)

- `Manipulator(robot, baseLink, flangeFrame, ?flangeTTcp)` compiles the
  whole robot (`RobotKinematics.compile`), so `RobotModel.couplings` apply;
  arm `q` has one value per arm DOF (a coupled follower is not one). FK,
  Jacobians and IK targets are in the base link's frame; `withTool` shares
  the compiled model; `pathJoints()` feeds OPW's parameter extraction.
- Deleted: `KinematicChain`, `ChainTip`, `InverseKinematics`,
  `RobotKinematics.path`, `JointGroup.fromChain`. **Kept, deviating from the
  item as written:** `IKResult` (now with the solver `status`) as the
  arm-ordered result type, since every IK caller wants `q` in arm order.
- The IK is still `DampedLeastSquares` with flange-target conversion, so
  every answer is unchanged; the LM switch is the next, separate step
  (KK-D11).
- Callers migrated: RobotKit (tests, `PayloadChecker`, `DigCyclePlanner`
  docs), MotionKit (`ManipulatorKinematics`, `OpwKinematics`, tests),
  ToolpathKit motion tests, cadbridge tests, the robot-arm authoring tool.
  The gantry test with a link tip uses the compiled model directly.
  `robotkit/ARCHITECTURE.md` and the gap map describe the new layering.
- New RobotKit test: a planar arm whose third joint follows the second; the
  arm has two DOFs, FK turns the follower, IK recovers the leader, and joint
  targets go to leaders only.
- Acceptance: RobotKit world tests (4777 assertions); MotionKit (6723);
  cadbridge (129); ToolpathKit motion (9 + 10 + 2975); MachineKit smoke
  including the robot-arm motion check.

### K4b — Switching RobotKit IK to Levenberg-Marquardt: tried and reverted (2026-09-30)

- Tried `Manipulator.solveIk*` on `LevenbergMarquardt`, solving at the TCP
  directly. RobotKit's wall-finishing scenario then failed in TOPP-RA
  (`path.time failed with MotionKit error -1`: a time-law stage collapsed to
  zero speed). Instrumented rather than guessed: the time limits and caps
  were valid; the joint path had a spike of 7.4 rad/m at sample 99 of 300,
  with the wrist joint at −π/2 (a wrist singularity).
- Cause: sequential IK along a path. DLS takes fixed-damping steps from the
  previous sample and stops inside the tolerance, so it moves the joints as
  little as possible; LM drives its damping towards zero and heads for the
  exact solution, which near a singularity needs large joint motion.
- Reverted. KK-D11 now keeps DLS for tracking and LM for reaching (above).
  The TCP-as-frame-offset part of the item is deferred with it: solving at
  the TCP changes where tolerances apply, so it goes with the next tracking
  change (the K3 QP) rather than alone.

### K4c — CAD and robot models of one arm agree (2026-09-30)

- The robot-arm authoring tool (run by `machinekit/scripts/test-haxeon`)
  now compiles both descriptions of the arm, the CAD `AssemblyState` and the
  `RobotModel` cadbridge derives from it, and compares the cup contact over
  25 random joint vectors within limits: worst 1.4e-15 m and 9e-8 rad (the
  `acos` precision floor). The check fails above 1e-9 m / 1e-6 rad.

### K3a — Native core: model, forward kinematics, Jacobians (2026-09-30)

- `kinematicskit/native` is its own haxeon package (`kinematicskit-native`)
  so the kit stays pure Haxe for the editor and CAD; only callers that want
  the native solver depend on it. It builds `kinematicskit_core` (C ABI in
  `include/kinematicskit.h`, `bindings/kinematicskit.hxi` generated by
  `tools/check-hxi.sh`, which also audits the ABI on Linux, Windows and both
  macOS targets). Eigen is linked privately; nothing in the ABI names it.
- A model crosses the boundary once as two packed arrays (format 1, laid out
  in the header) built by `NativeKinematics.packInts/packReals`; the native
  side validates every index and number. Handles are generation-checked.
- `kk_forward` and `kk_point_jacobian` reproduce the Haxe snapshot
  operation for operation (`-ffp-contract=off`). Parity test
  (`native/tests`): five random forests with revolute, prismatic, fixed and
  coupled joints and two roots, 20 configurations each, every body, random
  column subsets in shuffled order: poses and Jacobians are bit-identical
  (worst difference exactly 0).
- proxsuite 0.7.3 was checked for K3b: BSD-2-Clause (Inria), header-only;
  its dense solver needs only Eigen plus a `proxsuite/config.hpp` that
  defines four version macros (upstream generates it with jrl-cmakemodules).
  SIMD (simde), serialization (cereal) and Python bindings are optional and
  not used. A 7-variable box-constrained QP re-solves in about 14 µs with
  warm start.

### K3b — Native QP step on ProxQP; mink-shaped differential IK (2026-09-30)

- proxsuite v0.7.3 vendored (pinned submodule, header-only dense solver,
  `THIRD_PARTY.md`). `kk_qp_create/solve/destroy` solve
  `min ½‖J·Δ − e‖² + ½λ²‖Δ‖²  s.t.  lower ≤ Δ ≤ upper` as a box-constrained
  dense QP, warm-started across calls; the answer is projected onto the
  bounds so limits hold exactly (ProxQP meets them only to its tolerance).
- `kinematicskit.native.NativeQpStep` wraps it; `DifferentialIk.step`
  evaluates a `KinematicProblem`'s tasks in Haxe (so every task kind,
  closures and look-at included, works unchanged) and bounds Δ by the
  configuration range and optional velocity limits, mink's formulation with
  a per-step `gain`. Task evaluation stays in Haxe until a native servo loop
  needs it.
- `PostureTask` gained per-DOF weights (mink's posture cost has them).
- Tests (`native/tests`, 339 assertions): unbounded QP = the Haxe damped
  step within 1e-6 (the solver's 1e-9 residual tolerance amplified by up to
  1/λ²); KKT conditions hold with active bounds; warm start does not add
  iterations; a 7-axis arm reaches a target with velocity limits and never
  leaves its joint range, and stops exactly on a limit that is in the way;
  a planar 3-link arm's base-angle preference slides along its self-motion
  while the target is met to 1e-6.
- **Haxeon FFI bug found:** `CHeaderImporter.expansionArgument` indexes the
  decoded header text with clang's byte offsets, so any non-ASCII character
  earlier in a header makes array-count annotations unresolvable ("could not
  resolve input-array count parameter"). `kinematicskit.h` stays ASCII;
  the importer should read bytes. To fix in haxeon separately.
- **Fixture lesson (7-axis):** a posture preference on one "elbow" joint is a
  poor redundancy test for a 7-axis arm: at special poses (a2 = a4 = a6 = 0)
  that joint is not in the self-motion at all, and at generic poses the
  self-motion moves the base joints ~9x more, so the preference crawls and
  fights the tool target. A swivel-angle task is the right tool (K3d).

### K3c — Deferred to Lane D's servoing item (2026-09-30)

- Nothing in production calls `KinematicsSolver.solveDifferential`; only
  MotionKit's own tests do. Its contract `(q, twist)` has no period and no
  limits, so the QP's reason to exist (velocity and joint bounds that hold
  exactly) cannot be expressed through it, and swapping the solver under it
  would only add a native dependency to every MotionKit consumer.
- Moving it to the QP also would not preserve its answers: it uses damping
  1e-6, and the QP's step accuracy is its residual tolerance amplified by up
  to 1/λ² (1e12 there).
- So the switch happens with its first real consumer, Lane D's live servoing
  (D5): a servo-step contract with a period, velocity limits and deadlines,
  built on `kinematicskit.native.DifferentialIk`, with the Haxe damped step
  as the fallback. `solveDifferential` stays on the Haxe solver until then.
- K3d's mink oracle needs Python with `mink` and `mujoco`, which this machine
  does not have; those tests will skip with a message where they are absent.

### K3c — Bounded servo step in MotionKit (2026-09-30, revises the deferral above)

- Done without waiting for Lane D, as a new contract rather than a change to
  `solveDifferential` (whose answers and unbounded contract stay as they
  are): `motionkit.robot.ManipulatorServo.step(q, twist, dt, ?velocityLimits)`
  solves the bounded QP over the TCP Jacobian with the group's position
  limits and velocity limits (`velocity <= 0` = unlimited), warm-started
  across ticks; if the QP does not solve, the Haxe damped step clamped into
  the same bounds is used and the answer says so (`fallback`, `qpStatus`,
  `limited`). Lane D's live servoing (D5) builds on it.
- `motionkit-robot` now depends on `kinematicskit-native`, so every MotionKit
  consumer builds `kinematicskit_core`; RobotKit world tests (4777),
  cadbridge (129), ToolpathKit (2975) and the MachineKit smoke pass with it.
- Tests (MotionKit, +22 assertions): unconstrained ticks equal the damped
  step; velocity limits hold exactly and report the held joints; turning the
  arm about its base axis drives the base joint into its +2π stop and holds
  it there, never leaving the range; a starved QP falls back within limits.

### K3d — Swivel task and the mink oracle (2026-09-30)

- `SwivelTask`: the elbow's angle about the shoulder-wrist line from a
  reference plane, one row that picks a 7-axis arm's configuration. Exact
  residual; Jacobian = gradient of the closed-form angle w.r.t. the three
  points (central differences, no extra FK) times their point Jacobians;
  allocation-free. Kit test: matches central differences through full FK,
  and joints beyond the wrist point have zero columns. Native test: one 6D
  tool pose reached at swivel 0.4 and −0.6 — two configurations, both meeting
  the tool target to 1e-6 and the swivel to 1e-6.
- mink oracle (`native/tests/mink_oracle.py`, `testMinkOracle`): the test
  writes the 7-axis arm as MJCF (`Mjcf.hx`, test-only; no couplings or
  closures), mink (1.3.0, MuJoCo 3.14, DAQP) solves the same step with
  `damping = λ²` and `ConfigurationLimit(gain = 1)`, and the answers are
  compared. Skipped unless `KK_MINK_PYTHON` names a Python with mink.
  - position target: 4.5e-6 rad/s on 30.6 rad/s (1.5e-7 relative);
  - position target with speed limits: 8e-9 (same active set);
  - pose target: 2.4% apart at 0.05 rad rotation error, 0.33% at 0.005 rad.
    mink's frame error is the SE(3) logarithm with its exact Jacobian; ours is
    world-frame position plus a first-order orientation Jacobian (exact at
    convergence). The gap is first order in the rotation error (the test
    checks the ratio). Both converge to the same target; they take slightly
    different paths. An SE(3)-log `FrameTask` option would close it if a
    use needs mink-identical transients.

### Orientation Jacobian: exact log-map derivative measured and rejected; limit-approach gain added (2026-09-30)

- Tried `FrameTask` orientation rows with the exact derivative of the error
  φ = log(R_target·R_currentᵀ), i.e. J_r⁻¹(φ)·J_ω (mink uses the exact
  Jacobian of its own error). It matched central differences, but measured
  on a 6R arm (40 targets per band, same seeds for both):

  | rotation error | DLS converged, first-order / exact | LM converged, first-order / exact (mean iterations) |
  |---|---|---|
  | ~0.3 rad | 40 / 40 | 40 / 40 (2.2 / 2.2) |
  | ~1 rad | 39 / 39 | 39 / 39 (3.7 / 3.7) |
  | ~1.8 rad | 28 / 25 | 34 / 34 (5.5 / 5.5) |
  | ~2.5 rad | 18 / 15 | 23 / 24 (6.7 / 7.6) |
  | ~3 rad | 13 / 11 | 14 / 15 (7.5 / 9.3) |

  Identical up to ~1 rad, worse beyond: J_r⁻¹ grows without bound as the
  error nears π (the logarithm is singular there), while the first-order
  rows are steepest descent on the rotation distance. It also broke
  RobotKit's reachability test (a cold-start point 2.54 rad off stalled at
  0.50 rad). The mink oracle's pose-target gap did not move, so that gap is
  mink's SE(3) translation/rotation coupling, not the Jacobian.
- Decision: one formulation (first order) and no option. Do not reintroduce
  the exact derivative without new evidence. If convergence from far
  orientations ever matters, fix the cause: better seeds (OPW solutions,
  `sampleCandidates`) or an error without a singularity at π (e.g.
  quaternion-based), as a separate task type.
- Kept: `limitGain` k in (0, 1] on `DifferentialIk.step` and
  `ManipulatorServo.step` (default 1 = hard stops as before): a DOF covers at
  most k of its remaining distance to a stop per step, mink's
  configuration-limit gain. Tests: with k = 0.5 the gap to the stop at
  least halves per step and never closes; the servo slows the base into its
  +2π stop without touching it.

### E1a — Assembly drag session, headless (2026-09-30)

- `cadkit.modeling.AssemblyDrag`: grab an occurrence (or one of its
  connectors), move world targets, get preview coordinates and a reason when
  it cannot follow (`Following`, `OutOfReach` with the distance, `Limited`
  naming the joints, `ClosureBroken` naming the linkages). Movable joints:
  the driving joints from the root to the grabbed occurrence plus the
  editor's dependent joints; every closure is a task, so linkages stay
  closed while dragged. Tracking = damped least squares from the previous
  preview. Never modifies the `AssemblyState`; `commit()` returns the record
  for one undoable edit. Position is weighted in metres whatever the drawing
  unit, so damping means the same in a millimetre assembly as for a robot.
- `DampedLeastSquares` gained an optional step cap (`maxStep`, prismatic
  DOFs scaled by `translationScale`), unlimited by default so existing
  answers are unchanged. Found by the drag test: a target far outside the
  reach made the uncapped step swing joints by radians per iteration (the
  arm ended pointing away, 2156 mm from a target it could get within
  ~1100 mm of). The drag caps steps at 0.1 rad / 0.1 m.
- `AssemblyDragSmoke` (CadKit): a 60 mm drag of a 3-link arm in 30 steps is
  followed smoothly (< 0.02 rad per step) and lands on target; out of reach
  reports the real miss; a blocking elbow limit is named; a four-bar's
  rocker swings 10° with the pin closed (the crank turns) and a stretch it
  cannot make is reported; the state is never modified and the committed
  record reproduces the preview.

### E1b — IK drag in the editor viewport (2026-09-30)

- Pressing a joint-driven assembly part (which the object drag refuses)
  now starts an IK drag from the pressed point: the viewport picks the
  world hit (`EditorScene.pickHitRayWithView`), the session builds an
  `AssemblyDrag` grabbing that point (`AssemblyDrag` gained a `grabPoint`),
  and the point follows the cursor on a camera-facing plane through it.
  Joints above the part and the project's dependent joints move; closures
  stay closed.
- Preview without history through `EditorScene.setAssemblyOccurrenceTransforms`
  (what the joint slider uses); release records one `EditOperation` with the
  before and after records (the object drag's `record` pattern); Escape or
  pointer-cancel restores the starting pose. A green pull line from the
  grabbed point to the cursor turns amber when the part cannot follow, and
  the viewport toolbar shows the reason ("Following", "Out of reach: …",
  "Joint "j3" is at its limit", "Linkage … cannot stay closed here").
- Layers: `SceneAssemblyDrag` (scene-facing interface), `ProjectAssemblyDrag`
  (session implementation), `ProjectDocumentSession.beginAssemblyDrag`
  (public, so it is tested without a window), viewport mode 5.
- Test (`ProjectSourceTests.checkArmDrag`, on the robot-arm project): the
  suction cup follows a 3 cm pull in 15 steps and lands within 1e-5 m;
  release commits, undo restores; a 5 m target reports "out of reach" and
  cancel leaves the state untouched; the fixed root part cannot be dragged.
- The app suite's `ScriptedSetupTests` fails at "inspector action is
  visible: sensor-pause" in this worktree with or without these changes
  (checked by stashing them and rebuilding); every suite before it passes.

### K5 — Moving roots: mobile and floating bases (2026-09-30)

- `KinematicProblem.setRootMotion(root, RootMotion.Planar | Floating)`:
  the root's pose becomes solver variables. `JacobianLayout` appends one
  block per moving root after the DOF columns (Planar: vx, vy, ωz;
  Floating: v, ω; world frame, twist about the root's origin), and
  `KinematicSnapshot.pointJacobianColumns` fills them (v + ω × (p − o), ω),
  so every task (frame, look-at, swivel, closure, posture) sees them with no
  change. `KinematicModel.bodyRoot` maps bodies to their roots.
- Steps apply on the manifold (`SolverSupport.applyStep`: rotation
  Exp(ω)·q on the left, position + v; no Euler angles, so no gimbal lock).
  DLS and LM handle root columns (LM rolls root poses back with the joints;
  its step cap scales root v by `translationScale`), native `DifferentialIk`
  bounds them only by velocity limits. Each moving root allocates one pose
  per iteration (roots live as `Transform`s in `KinematicState`).
- `RootDampingTask` (soft): a cost on base motion, so the joints do what
  they can and the base moves for the rest.
- Tests: root columns match central differences (perturbing on the
  manifold) for both modes; a 2.3 m arm on a cart reaches a target 5 m away
  only with a planar base, which stays on the floor and upright; with root
  damping the base moves < 10% of the undamped distance for a target in
  reach; a floating box is placed exactly (1e-9) from three corner targets
  under a 2.5 rad rotation, rank 6; native differential IK drives a
  speed-limited cart until its arm reaches a target 4 m away, all velocity
  and joint limits held. Loop-solver parity, CadKit assembly tests,
  MotionKit and RobotKit unchanged.
- Not done here: RobotKit's `Manipulator` still solves with a fixed base
  (a `baseMotion` option would wire `RobotModel.mobileBase` /
  `floatingBase` to this); H7's whole-body control (contacts, centre of
  mass, dynamics) stays in HUMANOID.md.

### Follow-ups: Manipulator with a moving base; DLS checks its last step (2026-09-30)

- `Manipulator.baseMotion()` maps the robot's flags to the K5 modes
  (`floatingBase` → Floating, `mobileBase` → Planar, else Fixed), and
  `solveIkWithBase(worldTarget, seed, rootPose, …, baseCost)` solves arm and
  base together (Levenberg-Marquardt, a reaching solve; tolerances at the
  TCP; `RootDampingTask` so the arm does what it can first) and returns the
  joints plus the new root pose (`IKResult.rootPose`). Planar treats the
  base as able to reach any floor pose: right for where to stand, not for
  the instantaneous motion of a differential drive. RobotKit test: a UR5-size
  arm and a target 2.2 m away — out of reach on a fixed base; reached by a
  mobile base that stays on the floor and upright, the world tool pose on
  target; reached with a floating base too.
- `DampedLeastSquares` now checks the state after its last step and reports
  `Converged` if that step landed inside the tolerances (it reported
  `IterationLimit`; the known quirk from KK-D11). Kit test: with exactly the
  needed budget a solve now converges; one step fewer is still the limit.
  The robot-arm motion check, MotionKit (6746), ToolpathKit (2975),
  cadbridge (129) and RobotKit (4785) are unchanged by it.

### K6a — One kinematic group (2026-10-01)

- RobotKit's `KinematicGroup` merges `Manipulator` and `CoordinatedGroup`.
  Every pose, Jacobian and IK target is in its reference frame: the work
  frame when there is one, else the root link's.
  - Jacobians are J_tool − J_reference, rotated into the reference frame.
    For a fixed base the reference term is zero, so nothing changes there.
  - The tool task targets the reference frame through
    `FrameTask.relativeTo` when there is a work frame. Without one, the root
    link's pose turns the target into a world target once, as before.
- `Manipulator` is now a small subclass: the fixed-base, no-work-frame case.
  It keeps `baseLink` and `withTool`.
- One entry point: `solve(target, seed, IkOptions)`.
  - `IkOptions` carries the tolerances, tracking (DLS) or reaching (LM),
    `flange()`, `atSwivel(angle, exact)`, `preferring(posture)` and
    `movingBase(rootPose)`.
  - `solveIk`, `solveIkForTcp`, `solveIkAtSwivel`, `solveIkWithBase`,
    `CoordinatedGroup` and MotionKit's `CoordinatedKinematics` are gone.
- `ManipulatorKinematics` adapts any group and carries an optional preferred
  posture; the D6 test runs through it.
- The default swivel and `redundant()` count the arm's DOFs, not the
  external axes.
- The preferred-posture solve still has two passes; K6b replaces them.

### K6b — Prioritized solver; one-pass posture solves (2026-10-01)

- `PrioritizedSolver` (pure Haxe) solves each iteration as an
  equality-constrained least-squares step:
  - hard rows are equalities;
  - soft rows plus λ² damping form the objective;
  - the KKT system is regularized by 1e-10 on the constraint block, so
    singular hard rows still solve;
  - limits are an active set: pin the violators, re-solve the rest.
- It converges once the hard tasks are met and the step has settled.
- `KinematicGroup` uses it whenever a posture is preferred
  (`IkMethod.Prioritized`), replacing the draw-then-polish two passes.
  Tracking without a posture stays on DLS (KK-D11).
- Test: on the 7-axis fixture, a preferred posture is another exact solution
  of the same tool pose. The solver slides along the self-motion to it
  (within 1e-3) with the tool exact (< 1e-9), while plain DLS stays more
  than 0.1 away. A posture beyond a limit leaves the joint on the limit,
  with the tool still exact.
- My first test preferred a posture the pose cannot reach. Only one DOF is
  free at a fixed tool pose, so nearly no improvement is possible there,
  whatever the solver does.

### K6c — Step limits and the tool's twist task live in the kit (2026-10-01)

- `StepLimits` (pure Haxe) bounds one differential step per layout column:
  - configuration, with the limit gain;
  - velocity;
  - with the previous velocity: acceleration, and a braking bound that is
    discrete (or ramped, for streamed plan chunks).
  - Where bounds conflict, position and braking win.
  - Each side of a DOF bounds itself, so one-sided limits work as before.
- `FrameVelocityTask` asks a frame point to move at a twist for one step,
  optionally relative to another body (as `FrameTask.relativeTo`).
- `DifferentialIk.step(problem, state, dt, qp, ?limits:StepLimits, …)`
  replaces the velocity-limit arrays and gain. It returns `fallback` (the
  QP did not solve: the damped step clamped into the same bounds) and the
  `limited` columns. `StepLimits.ofVelocity` covers the mink-style calls.
- `KinematicGroup` exposes `problem()`, `stateOf(q)` and
  `toolVelocityTask()`. `ManipulatorServo` keeps its API but is now a thin
  call to `DifferentialIk` on the group's tool twist task; its bound code
  moved into `StepLimits`.
- The mink oracle (with `KK_MINK_PYTHON`) gives the same numbers as before:
  position targets agree to 8e-9 rad/s.

### K6d/K6e — Solvers own their path search; one redundancy resolver (2026-10-01)

- `KinematicsSolver.solvePath(PathRequest)`: each solver searches paths its
  own way, and `ProgramCompiler` asks it. The OPW and redundant-arm type
  switches are gone; a selector passed explicitly still forces the generic
  sampled search.
  - OPW: its analytic branches through native selection.
  - `ManipulatorKinematics`: `RedundancyResolver` for a redundant group,
    else point by point (`PathRequest.followPointByPoint`).
  - Logical axes and test solvers: point by point.
- `PathConfigurationSelector` is the generic engine: sampled candidates and
  Descartes `select`. Its `nativeRequest`/`readResult` are shared with OPW.
- `RedundancyParameterization` names the redundancy. It can report its
  values for `q`, solve at given values, solve near the seed's, give
  lattice rates per metre, say which values wrap, and give a cost factor
  per DOF.
  - `SwivelParameterization`: a 7-axis arm, at 5 rad per metre.
  - `ExternalAxesParameterization`: the values of a cell's external axes,
    solved with `IkOptions.holding` (held DOFs leave the solve). External
    axes cost a tenth of the arm.
- `RedundancyResolver`:
  - A beam search over the lattice: candidates continue held or moved one
    step per value. Each new configuration gets its cheapest cost from the
    start over all previous candidates within `maxJump`. The 48 cheapest,
    one per cell, go on, and the route is traced back.
  - Then the redundancy is smoothed (Gaussian, values reflected through
    each end) and re-solved exactly.
  - D3's 7-axis path and D6's cell now both go through it.
- Found while doing it:
  - D3's original lattice kept held continuations first. Once a sample had
    48 candidates it stopped spreading, so the redundancy could only drift
    a couple of steps. The beam search keeps the cheapest candidates
    instead, whichever way they move.
  - D6 on the resolver: the turntable turns from the first sample, while
    the arm moves less than a tenth as much (0.33 rad against 6.2 rad).
    The worst joint step is 0.008 rad per 2.5 mm.
  - A one-sided smoothing window at the path's start bent a steady trend
    there (0.503 mm off). Reflecting through the end fixed it.
  - MotionKit timing: stretching carried each stage's start speed through
    the recurrence. Where the path speed nearly stops mid-path that drifted
    negative (-7e-8), and the law was rejected. `retime_stages` now aims
    every stage at its exact target end speed from the speed actually
    reached. Rounding is then corrected stage by stage instead of
    accumulating. `soften_stages` uses it too.
- Lattice steps are rates per metre of path, no longer per sample (the D3
  debt).
