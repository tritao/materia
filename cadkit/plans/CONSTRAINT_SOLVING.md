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
  suite.** (Adopted for sketches in C2.1b; see the progress log.) Candidate: rank at a random nearby configuration that still
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

### C0 — Jacobian oracle and invariance suite (2026-09-30)

- `cadkit.solve.JacobianCheck`: central differences with a per-column step
  `1e-6 × max(1, |x|)`, reports the worst entry by relative error. Its smoke
  checks a correct Jacobian at 20 random points and finds a flipped sign.
- `DiagnosisInvarianceSmoke` (in `HaxeonSmoke`): 6 sketch fixtures × 8 checks (expected answer, 6 transforms, repeat)
  and 5 assembly fixtures × 9 (7 transforms). The `KNOWN` list is the baseline; the
  smoke fails when it is wrong either way.
- Assemblies scale up by 1e5, not 1e6: the schema caps positions at 1e9
  units (`AssemblyCodec.validateFrame`).
- Not done: a codec round trip for sketches (no standalone sketch codec; the
  document codec is the only one). Add it with C2 if the sketch record grows.

Baseline (13 known failures):
- **Scale 1e6:** every sketch reports `conflicting` (absolute solve and rank
  tolerances; `characteristicScale` only normalises lengths above 1).
- **Scale 1e-6:** the redundant list gains `h0`/`h1` (absolute rank
  tolerance).
- **Redundant lists are unstable:** translate, rotate, perturb and even a
  second solve of the same sketch change which members of a dependency are
  listed (`height` comes and goes), because redundancy is "dropping this
  constraint keeps the rank" per constraint at whatever pose the solve lands.
- **The unclosable four-bar reports `nonconvergent`, not `conflicting`.**
- Passing, and worth knowing: touching circles and the toggle four-bar are
  not flagged, because the solve stops near the singular pose rather than
  on it; assemblies are invariant under every transform tried.

Found on the way (fixed separately): `AssemblyNestingSmoke` still expected the
old JSON form fd97d023 removed; haxeon forgot every flow fact at loop entry
when the body stored to any array element or field (haxeon e199f1d3,
branch `loop-flow-stores`), which had broken toolpathkit's
`ToolpathMotionBinding` under the current pin.

### C1 — `ConstraintDiagnosis` (2026-09-30)

- `cadkit.solve.ConstraintDiagnosis`: union-find subsystems (shared variable
  or owner); two-pass row/column equilibration; Householder QR with column
  pivoting of Jᵀ; rank relative to the first pivot (default 1e-9);
  fundamental circuits from R₁₁⁻¹R₁₂ (member if |coefficient| > 1e-7 of the
  largest), merged by shared owner. Merged circuits are the matroid's
  connected components, so groups do not depend on the pivot order (a smoke
  reverses the rows to check). `nearDegenerate` when a kept pivot is within
  1e3 of the threshold. Suggestions use FreeCAD's order (most groups, fewest
  rows, newest) and skip protected owners.
- Lesson: equilibrating columns means a variable seen only with a tiny
  coefficient is not a degeneracy (it is just in small units); the first
  "near-degenerate" test was wrong for that reason.
- The generic-rank experiment (CS-D6) moved to after C2.1: it needs real
  sketch Jacobians. Its target fixture is `touching-circles-exact`.

### C2.1 — Sketch solver on `ConstraintDiagnosis`, scale-free (2026-09-30)

- `SketchSolver` now steps in lengths divided by the characteristic size
  (no longer clamped at 1), so its damping and tolerances are dimensionless;
  at 1e6 the fixed damping of 1e-3 used to swamp JᵀJ (~1e-14) and the solve
  never moved. `SolverSettings.rankTolerance` is now relative.
- Diagnosis: central differences at the solution (the loop still uses
  forward differences until C2.2), fed to `ConstraintDiagnosis`; rows are
  satisfied within 10 × the solve tolerance. `SolveDiagnostic.report` holds
  the full report; `status`, `degreesOfFreedom` and `constraintIds` are
  derived from it (redundant ids = members of satisfied groups; conflicting
  ids = members of unsatisfied groups when the stop is stationary).
- Polish: after reaching the tolerance, up to 3 Gauss-Newton steps while each
  halves the residual (a unit-conversion smoke checks area to 1e-5 mm²).
- Invariance: all 12 sketch baseline failures pass; fixtures now pin the
  dependency groups themselves (e.g. `top-length,v0,v1,width`).
  New fixture `touching-circles-exact` (authored on the singular pose)
  reports `redundant dof=1`: the CS-D6 target, recorded in `KNOWN`.
- `SketchEditBenchmark` (script fixed: it lacked the projectkit and
  kinematicskit roots): solve p50 75 → 28 ms, document recompute
  190 → 122 ms, since one QR replaces a rank computation per constraint.
- CamKit (12223 assertions) passes on the new solver.

### C2.1b — Generic rank by a witness pose; CS-D6 adopted for sketches (2026-09-30)

- When the local diagnosis finds a dependency, `SketchSolver` builds a
  witness pose: every length moves by a fixed pseudo-random ±1% of the sketch
  size, then only the shape constraints (all but fixed, distance, radius,
  angle) are re-solved. The generic rank is the larger of the two; when the
  witness wins, its groups replace the local ones (feasibility still from the
  real residuals) and `SolveDiagnostic.degenerate` is set.
- Evidence: `touching-circles-exact` (authored on the singular pose) goes
  from `redundant dof=1` to `fully-constrained dof=0` with the flag set; the
  nudged version, the redundant rectangle and a new `parallel-lines` fixture
  (a dependency that holds only on the shape: pairwise parallel lines) keep
  their answers and are not flagged. Adopted: the invariance suite passes
  with only `four-bar-impossible/expected` left in `KNOWN`.
- Cost: nothing when the local rank equals the row count (the common case);
  otherwise one shape-only solve, one Jacobian and one QR more.
- Assemblies get the same treatment with C3 (toggle four-bar authored exactly
  on its toggle is the analogous fixture).

### C2.2 — Analytic sketch Jacobians (2026-09-30)

- `SketchSolver.analyticJacobian`: one hand-derived block per constraint
  kind, over the existing geometry (distance rows, cross/dot over |a||b|,
  signed point-line distance with its axis terms, the along-axis row of
  symmetry, tangency following the stored side and contact branch). Entries
  are with respect to the scaled variables: linear rows unchanged, angular
  rows × s. The forward-difference path and the central-difference
  diagnosis Jacobian are gone; LM, diagnosis and the witness pose all use it.
  JᵀJ is accumulated over each row's nonzeros.
- `SketchJacobianSmoke` (runs before the other sketch smokes so a wrong
  derivative is named, not seen as a failed solve): every kind, both
  line-circle tangent argument orders, internal and external contact, at the
  authored pose and 10 random poses, at scale 1, 1e-6 and 1e6.
  `SketchSolver.probe` is the test hook.
- `JacobianCheck` fixes found by it: the step is relative to the variables'
  magnitude (a fixed floor of 1e-6 probed a 1e-6 sketch with steps as large
  as the sketch), and the error is relative to the row's largest entry (with
  `max(1, …)` any tiny-scale entry passed).
- Mutation check: flipping one term of the angle row fails
  `SketchJacobianSmoke` at (10, 10), "row 10 is constraint angle".
- `SketchEditBenchmark` against main's solver, back to back on a loaded
  machine: solve p50 80–95 → 8–9 ms; document recompute is now dominated by
  profile building (~210 → ~150 ms).
- CamKit (12223 assertions) and MachineKit pass on it.
- Step 3 (branch storage) is moot: there is no point-line distance dimension
  (`distance` is point-to-point, `pointOn` is zero-valued) and `angle` has
  both cos and sin rows, so it is already oriented.
- Step 4 (large-sketch benchmark, sparse solves) not started: the dense
  normal-equation solve is now the only O(n³) part; measure before changing.

### C2.4 — Sketch scaling: parts, sparse Cholesky, sparse diagnosis; codec check (2026-09-30)

Measured first (`SketchEditBenchmark` scaling section: rectangles of 4
points, independent or chained into one part). With C2.2's dense solve,
chained 200 points took 12.8 s, 97% of it in the dense normal-equation LU.

- **Parts:** constraints are grouped by the variables they reference (not by
  Jacobian nonzeros, which vanish by accident at special poses); each part
  runs its own LM, polish, diagnosis and witness pose, and
  `ConstraintDiagnosis.merge` combines the reports (untouched variables are
  free). The Jacobian is kept as sparse rows.
- **Sparse LM step:** `cadkit.solve.EnvelopeCholesky` — reverse
  Cuthill-McKee ordering per part (from the reference graph, computed once),
  JᵀJ + λI accumulated in its envelope, Cholesky there.
- **Sparse diagnosis fast path:** `ConstraintDiagnosis.diagnoseSparse`
  factors JJᵀ (rows at unit norm) the same way; all pivots ≥ 1e-8 proves
  independent rows, so the report needs no QR. Otherwise (a dependency to
  explain, or anything near) it falls back to the dense diagnosis. Redundant
  constraints are exact in practice, so they show as zero pivots. The
  diagnosis smoke checks sparse and dense agree on every hand-built case.
- Numbers (load average 13–20, so noisy): chained 200 points 12.8 s →
  14–18 ms; chained 1000 points (2000 variables, one part) 30.5 s →
  0.11–0.15 s; independent 1000 points (250 parts) 60–200 ms; the bracket
  1–2 ms (80–95 ms on main before C2).
- Known limit: a large connected part *with* a dependency still takes the
  dense QR (≈30 s at 1000 points). A sparse rank-revealing path (e.g. QR
  restricted to the rows the failed pivots touch) would fix it if it matters.
- **Codec check:** the invariance suite now also saves each sketch fixture
  in a document (`DocumentCodec`, `ConstrainedSketchFeature`), reloads it
  without evaluating, and solves again; all pass. This closes the C0 gap.
- C2 is closed: step 3 was moot (see C2.2), steps 1, 2 and 4 are done.

### C2.5 — Sparse dependency search and incremental re-solve (2026-09-30)

- **Measured first:** a connected 1000-point sketch with one implied
  constraint took 89 s on the dense QR fallback (200 points: 460 ms per drag
  step), since the JJᵀ fast path gave up on any dependency.
- **Sparse dependency search** (`ConstraintDiagnosis.diagnoseSparse`): the
  JJᵀ Cholesky (rows and columns equilibrated, as the dense path does) drops a
  row whose pivot is at most t² and reads its fundamental circuit by
  back-substitution over the earlier kept rows (coefficients can reach past
  the row's envelope, so the solve runs over all earlier rows). Circuits merge
  and suggestions follow as in the dense path.
- **Threshold consistency:** a Gram pivot is about the square of a row's
  distance from the earlier rows and is accurate only to ~1e-16, so it
  cannot reproduce a σ threshold below ~1e-8. With a rank tolerance t ≥ 1e-6
  the sparse path decides alone (dependent at ≤ t², near-degenerate below
  (1e3 t)², as the dense flag); with a smaller t it only proves clear
  independence and defers the rest to the QR. `SolverSettings.rankTolerance`
  now defaults to 1e-6 (was 1e-7). Documents saved with 1e-7 still diagnose
  correctly, their dependencies just take the dense path. The diagnosis smoke
  compares sparse and dense at both tolerances on every hand-built case.
- **Incremental re-solve:** `SolvedSketch.partCache` keeps each converged
  part's report under a key of its structure (constraint ids, kinds,
  references, the entities they reach), with its numbers (settings,
  dimension values, fixed positions, sketch scale) compared exactly rather
  than through strings. A solve seeded from that solution skips unchanged
  parts. Per-solve setup is now O(n): one variable→part/local table instead
  of per-part arrays, residuals walk only the part's constraints, RCM runs
  only for parts that solve. `SketchIncrementalSmoke` checks edits,
  re-authored fixed points and reused redundant parts against cold solves.
- Numbers (load ~5–9): redundant connected 1000 points 89 s → 126 ms;
  redundant 200 per drag step 463 → 16 ms; 250 independent profiles per drag
  step 28 → 10 ms; connected 200 per drag step 9.5 ms; bracket edit 1.2 ms.
- Still slow for dragging: one connected 1000-point part (~69 ms per step,
  it must re-solve whole) and the O(n) per-solve setup (~10 ms at 1000
  points with nothing to solve). Next levers if needed: skip diagnosis while
  dragging (diagnose on release, as FreeCAD does), and keep the part
  structure between solves when only values change.
- Aside, not fixed (user's call): haxeon's `Parser.decodeString` only knows
  `\n \r \t \" \\`; any other escape (`\u0001`, `\x01`) silently compiles to
  the escaped letter plus the rest (`u0001`). Use `String.fromCharCode`.

### C2.6 — Drag-step profile: warm-start damping, cheaper merge (2026-09-30)

Profiled 20 drag steps (width +0.01 each, seeded) at 1000 points:
- One connected part took 7 LM iterations per 0.01 nudge: every solve
  restarted at damping 1e-3, while a 250-rectangle chain's JᵀJ has soft modes
  near (π/250)² ≈ 1.6e-4, so the damping swamped exactly what the edit moves.
  Seeded parts now start at damping 1e-9 (Gauss-Newton; rejected steps still
  raise it): 1 iteration, LM 48 → 18 ms per step.
- 250 independent parts spent 5.7 ms merging reports: the sort joined owner
  lists inside its comparator. Precomputed keys: 1.2 ms.
- Per drag step now: connected 1000 points ≈ 35 ms (LM 18, diagnosis 14,
  setup ≈ 3), independent 1000 points ≈ 5 ms. CamKit and MachineKit pass.
- Remaining levers for the connected case, if 60 fps on 1000-point parts
  matters: reuse the RCM orderings across solves (the structure is unchanged
  while dragging), reuse row buffers in the sparse Jacobian and envelope, and
  skip diagnosis during a drag (diagnose on release).
