# Constraint solving — diagnosis, drivers, mates (handoff)

**Goal:** give CadKit one trustworthy answer to "how constrained is this, and
why not solvable", shared by sketches, assembly loops and (new) assembly
mates; then add the assembly features that need it: drivers, more closure
kinds, and mates.

The solving itself mostly exists and stays:

| Where | Solves | Unknowns | Jacobian | Diagnosis today |
|---|---|---|---|---|
| `cadkit/haxe/src/cadkit/sketch/SketchSolver.hx` | 2D sketches | point x/y, radii | forward differences (`jacobian`, step `1e-6 × scale`), dense normal equations | rank by Gaussian elimination with an absolute tolerance; DOF = variables − rank; `redundantIds` drops each constraint and re-ranks, so every member of a dependency group is listed and separate groups merge into one flat list; `conflicting` = stationary residual |
| `cadkit/haxe/src/cadkit/modeling/AssemblyLoopSolver.hx` → `kinematicskit.LevenbergMarquardt` + `ClosureTask` | assembly closures | selected tree-joint coordinates | analytic | `freeDofs` (rank), `conflicting` / `limit-blocked` / `nonconvergent`, unsatisfied closure IDs; no redundancy report |
| — | assembly mates (placement) | — | — | — |

What is wrong is the diagnosis. One number (the Jacobian's rank at the
current geometry) answers four different questions, so a status flips when
geometry passes through a special position (a four-bar at toggle, two lines
that happen to be collinear) although the design did not change. And the
dependent coordinates of a loop are hand-listed by every caller.

Read first:
1. `kinematicskit/plans/KINEMATICS.md`: the kit, its decisions (KK-D4
   closures are never FK edges, KK-D6 solvers never mutate inputs, KK-D11
   solver roles).
2. Code: the files in the table, plus `AssemblyState.hx`,
   `AssemblyKinematics.hx`, `AssemblyDrag.hx`, `SketchSession.hx`,
   `projectkit/src/materia/assembly/AssemblyDefinition.hx`, and
   `app/src/ProjectDocumentSession.hx` (`assemblyDependentJoints`,
   `evaluateAssemblyConfigurationState`).
3. Background (reference only, nothing to vendor):
   - FreeCAD PlaneGCS (LGPL), `src/Mod/Sketcher/App/planegcs/GCS.cpp`
     (`System::diagnose`, `identifyConflictingRedundantConstraints`,
     `initSolution`), read at FreeCAD `84ae7a7c`. Summary in the
     2026-09-30 progress-log entry below.
   - Review of FreeCAD grant proposal #106 (Sketcher diagnosis):
     https://gist.github.com/tritao/c344bce33a241f8798d26829db1f8375

Work in `../materia-worktrees/constraint-solving` on branch
`constraint-solving`. Failing test first, one commit per item, merge to
`main` after each green item, append to the Progress log, and stop and log
whenever this plan turns out to be wrong.

## Decisions

- **CS-D1 — No new solver library.** No Ceres, no symbolic code generation,
  no native sketch solver. Problems are small (sketches: tens to low
  hundreds of variables; kinematics: 6–50), kinematicskit already has
  analytic Jacobians, and every consumer here is Haxe. The slowness that
  exists is algorithmic (finite differences, dense O(n³)) and is fixed in
  Haxe. Revisit only for (a) a native real-time consumer, or (b) a measured
  large-sketch problem that sparse Haxe cannot fix.
- **CS-D2 — Four questions, four answers.** Diagnosis reports separately:
  1. *structure*: independent subsystems (graph components);
  2. *local rank*: the Jacobian's numerical rank at the current geometry;
  3. *degeneracy*: whether the current geometry loses rank the design does
     not (see CS-D6);
  4. *feasibility*: whether the constraints can all hold, decided from
     residuals after a solve, never from rank alone.
  Rank deficiency and conflict are different things: `x = 0, x = 1` is
  infeasible, not redundant.
- **CS-D3 — Dependency groups are the diagnosis; removal is a suggestion.**
  A dependency among A, B, C is reported as `{constraints: [A, B, C],
  deficiency: 1}`. Which one to delete is a separate, replaceable heuristic
  (`suggestedRemovals`), never the mathematical result.
- **CS-D4 — Rank is relative and scaled.** Rows are divided by their
  tolerance (as `ClosureTask` already does), columns are equilibrated, and
  the rank threshold is relative to the largest pivot of a column-pivoted QR
  (Householder). No absolute pivot thresholds.
- **CS-D5 — Graph structure splits the problem; it is not the authority on
  DOF.** Structural (matching-based) rank cannot see algebraic dependencies
  (three concurrent lines), so it is used for decomposition only.
- **CS-D6 — Generic rank is an experiment until it passes the invariance
  suite.** Candidate: rank at a random nearby configuration that still
  satisfies the zero-valued constraints (dimension values taken from that
  configuration). Adopted only if it keeps every invariance fixture stable;
  otherwise the report carries local rank and a near-degenerate flag only.
- **CS-D7 — One diagnosis module, in CadKit.** Every consumer (sketch, loop
  solver, mate solver) is in CadKit, so `cadkit.solve.ConstraintDiagnosis`
  lives there, over plain inputs (row-major Jacobian, row owner IDs,
  residuals, protected owners). kinematicskit stays kinematics; it keeps its
  own `freeDofs` for IK callers.
- **CS-D8 — Drivers are design data.** Whether a joint is an input (a
  cylinder, a motor) is a property of the mechanism, so it lives in
  `AssemblyDefinition`, not in the app's scene record. Dependent coordinates
  are derived; an explicit list stays only as an override.
- **CS-D9 — Tree/closure role is chosen by the tool, then persisted.**
  Re-deriving the spanning tree on every evaluation would change which joints
  have coordinates and break saved states and motion tracks (keyed by joint
  ID). The builder/editor assigns `Closure` when a new joint closes a loop.
- **CS-D10 — Mates are residuals on the existing solver.** A mate is a
  kinematicskit task between two frames; its unknowns are the floating roots
  (K5 `RootMotion.Floating`) plus any joint coordinates it reaches. No
  separate SE(3) solver. Mates never become FK edges.
- **CS-D11 — Branch choices live in the constraint.** Tangent side and
  contact already do; signed point–line distance and angle offsets follow,
  picked from current geometry when the constraint is created.
- **CS-D12 — Out of scope:** SQP / augmented Lagrangian (until a feature
  needs "satisfy exactly while minimising something else"), parameter
  aliasing and sparse solves (until a benchmark asks), 3D sketches,
  multi-DOF tree joints (ball/free), FreeCAD-style solver cascades.

## Layout

```
cadkit/haxe/src/cadkit/
  solve/                     new
    ConstraintDiagnosis.hx   components, scaling, QR of Jᵀ, groups, report
    DiagnosisReport.hx       subsystems, localRank, dof, dependencyGroups,
                             conflictGroups, degenerate, suggestedRemovals
    DependencyGroup.hx
    JacobianCheck.hx         finite-difference oracle for tests
  sketch/                    SketchSolver adopts solve/ (C2)
  modeling/                  AssemblyLoopSolver, AssemblyMateSolver (C4)
cadkit/haxe/tests/
  DiagnosisInvarianceSmoke.hx
  JacobianCheckSmoke.hx
  fixtures: sketches and assemblies listed in C0
kinematicskit/haxe/kinematicskit/
  MateTask.hx                (C4) plus closure kinds (C3)
projectkit/src/materia/assembly/
  AssemblyDefinition.hx      driven flag (C3), mates (C4)
```

`cadkit/scripts/test-haxeon` lists source roots by hand; new packages need
adding there.

## C0 — Tests first: Jacobian oracle and invariance suite

Do:
- `JacobianCheck`: given a residual function and an analytic Jacobian,
  compare against central differences at random inputs with a relative
  tolerance; report the worst (row, column).
- Fixture corpus, each with the expected answer written down:
  - sketches: fully constrained bracket; under-constrained rectangle;
    a triangle with one redundant angle; `x = 0` with `x = 1` (conflict);
    two dependency groups in separate parts of one sketch; collinear lines
    constrained parallel and with point-on (degenerate at this position);
  - assemblies: four-bar generic; four-bar at toggle; slider-crank;
    excavator (planar linkages in 3D: closure rows redundant **by design**
    and consistent); an impossible four-bar (conflict).
- Invariance checks on every fixture: uniform scale 1e-6, 1, 1e6;
  translation; rotation; constraint order; geometry order; codec round
  trip; repeated diagnosis; a small non-degenerate perturbation.
  Classification and group membership must not change.

Tests: the suite runs against today's code. Record where it fails as known
failures (the list is the baseline), do not fix yet.

Done when: the suite and oracle run in `cadkit/scripts/test-haxeon`, and the
baseline failures are listed in the Progress log.

## C1 — `ConstraintDiagnosis`

Do:
- Input: row-major Jacobian, row → owner ID, residuals, per-row tolerance,
  owners that may never be blamed (structural rows, e.g. arc rules).
- Components by union-find over owners and variables; each diagnosed alone.
- Equilibrate rows and columns; Householder QR with column pivoting of Jᵀ
  (columns = rows of J) with a relative threshold (CS-D4). Local rank → DOF.
- Dependency groups: for each non-pivot row, the pivot rows it depends on
  (from R₁₁⁻¹R₁₂), merged when they share members; deficiency per group.
- Feasibility: after a solve, a group whose rows are all within tolerance is
  *redundant*; otherwise *conflicting*. Report both separately.
- `suggestedRemovals`: FreeCAD's heuristic (the owner in most unsatisfied
  groups, then fewer rows, then newest), never blaming protected owners.
- Generic-rank experiment (CS-D6) behind a flag, evaluated on the C0 corpus.

Tests: unit tests on hand-built Jacobians with known rank and groups; the C0
invariance suite on synthetic inputs.

Done when: unit tests pass and the CS-D6 experiment's result is logged
(adopt or not, with the fixtures that decided it).

## C2 — Sketch solver on the new diagnosis; analytic Jacobians

Do, one commit each:
1. Replace `rank` / `redundantIds` / the conflict test with
   `ConstraintDiagnosis`. `SolveDiagnostic` gains the report (keep `status`
   and `constraintIds` as derived fields so `SketchSession`, the app and
   CamKit keep compiling).
2. Analytic Jacobian row blocks per constraint kind (distance: 1×4, etc.),
   checked by `JacobianCheck`; delete the forward-difference path.
3. Branch storage for signed point–line distance and angle offsets (CS-D11).
4. Extend `SketchEditBenchmark` with 50 / 200 / 1000-point sketches. Only if
   they are slow: component-wise solves, then sparse normal equations.

Tests: `ConstrainedSketchSmoke` unchanged except where a status becomes more
precise (log each change); C0 invariance baseline failures for sketches
turn green.

Done when: sketch fixtures in C0 pass, including scale invariance.

## C3 — Assembly drivers, mobility, closure kinds

Do:
1. `KinematicJoint.driven:Bool` (optional, codec + validation). Dependent
   coordinates = movable tree coordinates on a closure loop that are not
   driven and not coupling targets. `AssemblyState.solveClosures()` and
   `AssemblyDrag` derive them; explicit lists remain as overrides. Migrate
   the excavator and `ProjectDocumentSession` (the scene record's
   `assemblyDependentJoints` is read once for old projects, then dropped).
2. Mobility report: `AssemblyLoopSolver` feeds `ConstraintDiagnosis`; the
   inspector shows DOF, redundant-by-design closure rows (normal for planar
   linkages), and conflicts.
3. Automatic closure role (CS-D9) in `AssemblyModel` and the editor's joint
   creation.
4. Closure kinds Spherical (point only), Cylindrical (axis line only),
   Planar (plane contact): kinematicskit `ClosureKind` + `ClosureTask`
   rows, native parity where closures exist natively, and the MuJoCo mapping
   in `AssemblySimulationBridge` / `Simulation`.

Tests: four-bar / slider-crank / excavator solve with no dependent list;
excavator mobility = number of drivers; each new closure kind closes a
fixture linkage and passes `JacobianCheck`; sim bridge round trip.

Done when: no caller passes a dependent list, and the excavator reports its
redundant closure rows as consistent.

## C4 — Mates

Do:
1. Schema: `AssemblyDefinition.mates:Array<AssemblyMate>` — kind
   (coincident, coaxial, planar, parallel, perpendicular, distance, angle,
   lock), two connector references, value, optional flip. Codec, validation,
   nested-assembly flattening.
2. `MateTask` kinds in kinematicskit (residual + analytic rows between two
   frames), each checked by `JacobianCheck`.
3. `AssemblyMateSolver` (CadKit): floating roots for unplaced occurrences +
   reachable joint coordinates, kinematicskit LM, `ConstraintDiagnosis`
   report. Output written to `initialPose` (design) or `rootPoses` (state).
4. B-rep references: a mate may name a face/edge via
   `PersistentReference`; resolved to a frame before solving.
5. Editor: author mates, drag under-constrained parts (reuse
   `AssemblyDrag`), "convert to joint" when a mate set leaves one DOF.

Tests: analytic placements (flange on face, shaft in bore, offset planes);
a mate that also moves an arm's joints; conflicting mates reported as
conflict groups; mates survive a codec round trip and topology remap.

## C5 — Interaction

Do: soft drag targets in the sketch solver (weighted rows, hard constraints
unchanged); "still free" colouring from a QR of J for sketches and
assemblies; reference (measured) dimensions, evaluated after the solve and
excluded from solve and diagnosis.

## Order

C0 → C1 → C2 → C3 → C4 → C5. The diagnosis is proven on sketches first
because they have the richest existing expectations; C3 and C4 then reuse
it. If mates become urgent, C3–C4 can follow C1 directly with the loop
solver as the first consumer.

## Progress log

### Plan written (2026-09-30)

Sources behind the decisions:
- **Current code** (main at bc3a4f20): the definition/state split, tree FK,
  q-space closure solving, the shared kinematics kit and MuJoCo closures
  already exist, so an earlier proposal's steps 1–5 were done. Missing:
  drivers in the schema, mates, redundancy reporting for closures.
- **FreeCAD PlaneGCS** (read at 84ae7a7c): dense Jacobians assembled one
  entry per virtual `grad` call; DogLeg default with LM/BFGS/SQP fallbacks;
  equality constraints removed by aliasing parameters; components via
  connected components; diagnosis by `FullPivHouseholderQR` of Jᵀ
  (`SparseQR` above 1000 parameters, unreliable, issue #10903), dependency
  groups from R₁₁⁻¹R₁₂, a hitting-set heuristic to blame, and a re-solve to
  split redundant from conflicting; driven dimensions measured after the
  solve; drag targets as soft rows solved by SQP; branch choices stored in
  constraints. Weak points: absolute tolerances (1e-10, 1e-13), dense
  everything, three solvers plus a retry cascade.
- **Proposal #106 review (gist):** separate structure, generic rank, local
  degeneracy and feasibility; report dependency groups, not one culprit;
  do not treat graph structural rank as exact DOF; test invariance under
  scale, transforms, reordering and reload.
- **Ceres and code generation considered and rejected** for now (CS-D1).
