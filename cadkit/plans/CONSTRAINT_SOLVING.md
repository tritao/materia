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

### C2.7 — Drags under a frame: reused structure, diagnosis on release, fewer allocations (2026-10-01)

- **Structure reuse:** `SolvedSketch.structures` keeps, per part structure,
  the solve's RCM ordering and envelope, the diagnosis row graph and its
  ordering, and the last diagnosis. The key now also lists the part's
  variables in order, so reordering points can never pair a stale envelope
  with new indices. The diagnosis row graph comes from what constraints
  reference (a superset of any pose's nonzeros), so no entry falls outside a
  cached envelope; `ConstraintDiagnosis.diagnoseSparse` takes it as an
  optional `RowStructure`.
- **Diagnosis on release:** `ConstrainedSketch.solve(seed, cancel,
  diagnose = false)` reports each re-solved part's previous diagnosis
  (`SolveDiagnostic.diagnosed` false) and keeps it out of the value cache, so
  the next normal solve re-diagnoses exactly the parts that moved. A part
  that fails to converge is still diagnosed (a conflict mid-drag is reported).
- **Allocations:** the LM step keeps each part's envelope, merges a row's
  entries in reused scratch arrays, and the sparse Jacobian reuses its row
  buffers; polish runs only while the residual is above 1e-3 × tolerance.
- `SketchIncrementalSmoke` covers drag mode, release and a mid-drag conflict.
- Per drag step (load ≈ 27, so upper bounds), with diagnosis / on release:
  connected 1000 points 23.6 / 11.5 ms (was 69); connected 1000 with a
  redundancy 62 / 11.6 ms (was 129); connected 200 points 3.5 / 2.1 ms; 250
  independent profiles ≈ 5 ms; bracket edit 1.0 ms. CamKit and MachineKit
  pass.
- The editor has no sketch drag yet (C5 adds soft drag targets); when it
  does, it should solve with `diagnose = false` while dragging and normally
  on release.

### C3.1 — Driven joints; dependents derived from the loops (2026-10-01)

- `KinematicJoint.driven` (optional, wire id 12): an input of the mechanism.
  Only movable tree joints that are not coupling targets can be driven
  (codec rejects the rest); the flattener and `AssemblyDocuments` (a boolean
  relationship property) carry it. `AssemblyModel.drive(id)` sets it.
- `AssemblyState.dependentJoints()`: every movable tree joint on the tree path
  between a closure's two occurrences (below their lowest common ancestor)
  that is neither driven nor a coupling target, in definition order.
  `solveClosures()` and `AssemblyDrag` use it when given no explicit list.
- Excavator: its three hinges are driven and `buildState` derives the rest.
  **Found:** its old hand-written list spelled the cylinder joints
  `boom-cylinder-…` while they are named `Boom-cylinder-…`, and was filtered
  by name, so the six cylinder coordinates were silently never dependent
  (their loops closed only because the authored pose was exact). The derived
  list has all eight; the invariance fixture had copied the same list and now
  derives it too.
- App: with no saved choice the session starts from the derived dependents;
  a saved list (older projects, or an explicit choice, including an empty
  one) still overrides, and the scene record saves the list only when it
  differs from what the definition derives, so later source changes apply.
  The inspector toggle is unchanged. (The plan said drop the scene field;
  keeping it as an override is less disruptive and costs nothing.) The app
  compiles; its native tests were not run (none exercise this path).
- Also fixed on main's current haxeon pin: haxeon now decodes `\x`
  escapes (3d205e9e), so projectkit's three `indexOf("\x00")` checks became
  NUL string constants, which HashLink rejects ("HashLink String cannot
  contain NUL"), breaking every build that includes projectkit. They had
  never worked (they looked for the text "x00"); `AssemblyCodec.containsNul`
  checks by character code.

### C3.2 — Closure diagnosis; unclosable loops are conflicting (2026-10-01)

- `AssemblyLoopSolveResult.report`: the closure rows (already divided by
  their tolerances) at the final state, diagnosed over the dependent columns
  by `ConstraintDiagnosis.diagnoseSparse` at `SPARSE_TOLERANCE`, owners =
  closure IDs. A planar four-bar's revolute closure shows
  `redundant(pin)-3` (its out-of-plane rows); the excavator's four closures
  are all consistent-redundant, none conflicting.
- Status: kinematicskit's LM calls a stop stationary only below
  1e-10 (1 + ‖J‖‖r‖), and a large-residual (unclosable) loop approaches its
  least-squares pose only linearly, so it ran out of iterations. The
  unclosable four-bar ends with ‖Jᵀr‖/‖J‖‖r‖ ≈ 1.15e-6; unfinished solves sit
  at 0.1–0.9. CadKit now reports an iteration-limit stop with that ratio
  ≤ 1e-4 as `conflicting` (kinematicskit's own status, which RobotKit uses,
  is unchanged).
- The invariance suite's `KNOWN` list is now empty.
- Not done here: the inspector does not show the report yet (the API
  carries it); the assembly witness check is C3.3.

### C3.3 — Witness pose for closure solves (2026-10-01)

- After a converged solve whose diagnosis has a dependency, the driven
  coordinates move by ±1e-3 (radians, or that share of the assembly size),
  the loops close again from there, and the first side that closes is
  diagnosed (a toggle is often a limit, so one side may not close). More rank
  there means the dependency belongs to the pose: its report replaces the
  local one and `AssemblyLoopSolveResult.degenerate` is set. Without driven
  joints there is nothing to nudge and the check is skipped.
- A four-bar authored exactly at its toggle reports `degenerate` with the
  general diagnosis `redundant(pin)-3`; an ordinary pose is not flagged.

### C3.4 — Loop-closing joints become closures automatically (2026-10-01)

- `AssemblyModel.mate`/`mateOnAxis` record a closure (via `constrainOnAxis`)
  when the child already hangs from a tree joint or is an ancestor of the
  parent, instead of throwing "already has a parent joint". A closure has no
  coordinate, so a non-zero value there is an error. The editor creates no
  joints yet, so the builder is the only place this applies today.
- Smoke: a four-bar built from four plain mates records `pin` as a closure;
  mating back up the tree is a closure too; a valued closing mate is refused.

### C3.5 — Spherical, cylindrical and planar closures (2026-10-01)

- kinematicskit `ClosureKind` Spherical (3 position rows), Cylindrical (the
  existing transverse-position and axis rows) and Planar (distance of B's
  origin along A's normal, exact derivative a·(v_B − v_A) + (a × d)·ω_A, plus
  the normal-parallel rows). Haxe only: native kinematicskit has no closures.
- `AssemblyJointType` spherical/cylindrical/planar, valid only as closures
  (they have more than one coordinate); `AssemblyModel` records them as
  closures; `AssemblyState.closureResiduals` measures each kind.
- `ClosureKindsSmoke`: a ball-pinned four-bar (`redundant(pin)-1`), a slider
  on a cylindrical guide (`redundant(guide)-2`), a three-link leg standing on
  a planar floor (`redundant(stand)-1`) each close from their driven joint,
  and their rows match central differences at the solution.
- Simulation: the bridge refuses these kinds with a clear message; MuJoCo
  mapping needs native simkit work (a spherical closure is one connect
  equality; cylindrical and planar have no direct MuJoCo equality).
- Found, left as is: the existing Prismatic closure has the same 4 rows as
  Cylindrical, so it does not hold the twist about its axis. Fixing it
  changes behaviour the MuJoCo mapping relies on; do it with that mapping.
- kinematicskit (181), CadBridge (129) and MachineKit pass.

### Pre-C4 cleanup 1–2: stationarity in kinematicskit; one tolerance policy (2026-10-01)

- `LevenbergMarquardt` calls a stop stationary when ‖Jᵀe‖ <= 1e-4 ‖J‖‖e‖
  (`STATIONARY_RATIO`, plus the old absolute floor), so an unclosable loop is
  `Conflicting` from kinematicskit itself; the CadKit-side ratio test from
  C3.2 is gone. The limit-blocked test keeps the strict threshold.
  `Manipulator` only reads `Converged`, and no RobotKit/MotionKit test
  asserts a failure label.
- `ConstraintDiagnosis.DEFAULT_RANK_TOLERANCE` is now the CAD default 1e-6,
  used by `SolverSettings` and the closure diagnosis (no more literals); its
  doc lists every threshold and why. Hand-built diagnosis tests that probe
  finer tolerances pass 1e-9 explicitly.

### Pre-C4 cleanup 3: sparse/dense agreement on random sketches; a vacuous test found (2026-10-01)

- `DiagnosisAgreementSmoke`: 40 seeded random sketches (rectangles and
  triangles, some chained, with implied, duplicate, parallel and
  contradicting extras), each diagnosed by the dense QR and the sparse Gram
  path on the same rows at the authored pose and the solution; rank, groups,
  unsatisfied owners and parts must match, and at least 10 systems must
  contain dependencies.
- **Found:** since C2.5, `SketchSolver.probe` returned zero rows (its
  constraint list was filled only by `parts()`, which `probe` never calls),
  so `SketchJacobianSmoke` compared empty matrices and passed vacuously. The
  list is now filled in the constructor, `JacobianCheck` refuses a residual
  with no rows, and the angle-row mutation check fails again as it should.
  The analytic Jacobians themselves still pass on every kind.

### Pre-C4 cleanup 4: `SketchSolver` split by concern (2026-10-01)

- `SketchLayout` (variables, constraints, validation, scale, starting pose),
  `SketchEquations` (each kind's residual and analytic rows, generated from
  the old code so the formulas are unchanged), `SketchPartition` (parts,
  references, RCM ordering, local indices), `SketchPartSolver` (LM + polish
  over a part, owning the envelope and scratch buffers), `SketchPartDiagnosis`
  (diagnosis, structural row graph, witness pose), `SketchSolveCache` (part
  keys and values); `SketchSolver` only orchestrates (~190 lines, was ~990).
- The hidden mode state is gone: constraint subsets and shape-only
  evaluation are arguments, rows go through a `SketchRowWriter`, buffers
  belong to the part solver. Same tests pass unchanged; benchmark within
  noise (bracket edit 0.9 ms; connected 1000-point drag 12.3 ms undiagnosed).

### Item 5a: prismatic closures hold their twist (2026-10-01)

- Correction to C3.5's note: the MuJoCo backend already models a prismatic
  closure exactly (an auxiliary body with a slide joint, welded to the
  child), so it was the kinematic solver that disagreed. `ClosureTask`
  Prismatic gains a fifth row, the relative rotation's component along the
  axis (dφ·a ≈ a·(ω_B − ω_A) near closure); `AssemblyState` checks a
  prismatic closure's full relative rotation, as for fixed.
- `ClosureKindsSmoke`: the slider guide as a prismatic closure shows
  `redundant(guide)-3` (its twist row is zero in a plane) and its rows match
  central differences.

### Item 5b: MuJoCo mappings for spherical, cylindrical and planar closures (2026-10-01)

- simkit: `NKSIM_JOINT_SPHERICAL/CYLINDRICAL/PLANAR` (closures only; tree
  joints still accept fixed/revolute/prismatic). The MuJoCo backend extends
  the prismatic pattern: an auxiliary body under body_a at body_b's relative
  pose carries the free motion and is welded to body_b — cylindrical: slide
  + hinge along the axis at the anchor; planar: two in-plane slides + a hinge
  about the normal at the anchor. Spherical is one connect at the anchor.
- RobotKit runtime: matching `RK_RUNTIME_JOINT_*` constants (values equal
  simkit's; the runtime passes closure types through); bindings regenerated
  with `tools/check-hxi.sh`. The app maps the assembly types; the cadbridge
  no longer refuses them.
- Built simkit with MuJoCo in the worktree (submodule cloned from the shared
  checkout at the pin; `_deps` copied, `FETCHCONTENT_FULLY_DISCONNECTED=ON`).
  `assembly_closures_compile_as_equalities` covers all six types; simkit
  ctest 58/58 (one uinput test skipped). The app compiles; an end-to-end
  simulation with the new kinds needs the app's native libraries rebuilt.

### C4.1–C4.2 — Mate schema; mates as closure rows (2026-10-01)

- Decision change: mates compile to kinematicskit closures instead of a
  separate `MateTask` (a closure already is a frame-frame equality with
  analytic rows, diagnosis and FD tests). Coincident → Spherical, coaxial →
  Cylindrical, lock → Fixed, planar → Planar (now with an offset `value`);
  new closure kinds Parallel (2 rows), Perpendicular and Angle (a·b against 0
  or cos value; d(a·b) = (a × b)·(ω_A − ω_B) exactly) and Distance
  (|d| − value, exact). `KinematicModel.closureValue` carries the values.
  `AssemblyKinematics.compile(definition, withMates)` adds mates only for the
  mate solver, so loop solves and drags never see them.
- Schema: `AssemblyMateKind`, `AssemblyMate` (two occurrence connectors, an
  axis in each connector frame, optional value) on definitions and nested
  assemblies; `AssemblyComponentOccurrence.grounded` (what mates never move,
  for C4.3). The flattener scopes and resolves mates like joints (and emits
  the field only when there are mates); the codec validates ids (distinct
  from joints), kinds, connectors, values (distance/angle need one; distance
  ≥ 0).
- **Found:** kinematicskit's LM damps Marquardt-style (λ·diag JᵀJ). For an
  under-determined problem, which a mate on a free part always is, JᵀJ is
  rank-deficient and a column with a small gradient gets a huge step that the
  step clamp then shrinks to nothing: a single distance mate over six free
  coordinates stalled. New opt-in `levenberg` mode damps λ·max(diag)·I (the
  minimum-norm step); existing callers are unchanged, the mate solver will
  opt in.
- `MateRowsSmoke`: every mate kind solved over a six-joint chain (three
  slides, three hinges) and checked against central differences at the
  solution; codec round trip and rejections; nested scoping.
- **haxeon:** `Reflect.deleteField` on a typed record (fixed layout) returned
  false and left the field; fixed in haxeon's runtime (see its commit), which
  the codec rejection test relies on.

### C4.3 — `AssemblyMateSolver` (2026-10-01)

- Free parts: every root occurrence except the grounded ones (or the first
  root when none is) is a floating rigid body; the movable, non-driven,
  non-coupled joints between a mated occurrence and its root may move too.
  Mates and joint closures are solved together by LM in Levenberg mode, then
  diagnosed by `AssemblyClosureDiagnosis` (shared with the loop solver now):
  the report's degrees of freedom are what the mates leave free.
- Output: `AssemblyMateSolveResult.state(definition)` (root poses + reached
  coordinates) for a configuration, or `AssemblyMateSolver.place` to bake
  the placement into initial poses and joint defaults (non-nested only).
- kinematicskit: `setActiveDofs([])` is allowed (a problem may move only
  roots).
- `MateSolverSmoke`: planar + coaxial seats a motor leaving one turn free;
  a lock places it fully on its target; contradicting planar offsets are
  `conflicting` naming both; a repeated mate is redundant; a coincident mate
  turns a grounded arm's joint to reach a fixture; `place` round-trips.
- haxeon bumped to cc2d0d3c (`Reflect.deleteField` on typed records; haxeon
  suite 353/353). Not pushed yet: push haxeon before the parent.

### C4.1b — Mates in assembly documents (2026-10-01)

- `AssemblyDocuments` stores mates as `cadkit.mate` relationships between the
  two occurrence elements (kind, connectors, axis, optional value; scoped
  like couplings, sorted by id on read) and `grounded` as an occurrence
  property; removing an assembly removes its mates with its relationships.
- `MateSolverSmoke` round-trips the motor assembly through documents and a
  `DocumentCodec` save/reload and solves it again (one degree of freedom).

### C4.4 — Mates on faces and edges: geometric connectors (2026-10-01)

- Design change from the plan: a mate still names connectors. A component's
  definition can also carry **geometric connectors**
  (`cadkit.parametric.GeometricConnectors`): a face or edge kept as a
  topology fingerprint plus its direction and radius, framed again from the
  current geometry whenever it is needed. The solver, the codec, the
  flattener and the MuJoCo export see ordinary connectors.
  `PersistentReference` holds no topology, and `TopologyReference` needs an
  owning `Feature`, which a registry-evaluated definition does not have.
- Frames (`frameOf`), with z along the feature:
  - planar face: at its centroid, z the outward normal;
  - axial face (cylinder, cone, sphere, torus): on the axis nearest the
    centroid;
  - circular edge: at its center;
  - straight edge: at its midpoint.

  x is the world axis least aligned with z. `flip` reverses z. An axis has
  no sign of its own, so it keeps the sign it had when captured.
- Resolution:
  1. Exact fingerprint (`TopologyResolver`).
  2. After an edit moved or resized the feature: the *one* face or edge with
     the same surface or curve kind, direction and radius.
  3. Several matches → `Ambiguous`; none → `Unresolved`.

  A fingerprint is exact by design, and a definition has no operation
  history to remap through.
- Native: `cad_face_axis` and `cad_edge_axis` (struct `cad_axis`, projected
  as `CadKit.GeometricAxis` because `Axis` clashes with
  `cadkit.modeling.Axis`), read from `BRepAdaptor`; the core smoke tests
  both.
- Documents: the connectors live on the CadKit `Definition` (property
  `cadkit.assembly.geometricConnectors`).
  - `AssemblyDocuments.toDefinition` appends them, framed from each
    occurrence's evaluated geometry. Occurrences of one component must
    agree (`assembly.instance-dependent-connector`); a lost face is
    `assembly.unresolved-connector`.
  - `fromDefinition` strips them from stored records, so frames never go
    stale.
  - In memory, `GeometricConnectors.apply` frames them into an
    `AssemblyDefinition` before solving.
- `GeometricConnectorSmoke`:
  - the frame of each feature kind;
  - a pin mated to a plate's top face and bore: solved in memory, from a
    document, and after a save and reload;
  - widening the plate moves the bore and the pin follows;
  - a second bore of the same radius → ambiguous;
  - a connector on a face the part lacks → reported.

### Post-C4.4 review: fixes and refactors (2026-10-01)

1. **Angle and distance mates are regular** (`f1dccb2f`).
   - The angle row is `acos(a·b) − value`, in radians.
   - At 0 or π, a single row cannot express alignment (that is two
     constraints, and the angle is not differentiable there). So those
     values compile to stereographic alignment rows
     `2σ(b·u)/(1 + σ a·b)`, which vanish only in the requested direction.
   - Validation refuses a zero distance (use coincident) and angles outside
     [0, π].
2. **One assembly-solve core** (`AssemblySolve`) for `AssemblyLoopSolver`
   and `AssemblyMateSolver`. It holds:
   - one options type and tolerance policy (1 µm, 1e-6 rad, 200
     iterations);
   - one assembly scale (the diagonal of the occurrence origins plus the
     connectors' reach);
   - the LM call, the status names, and the witness check for degenerate
     poses.

   The loop solver's witness nudges the driven joints; the mate solver's
   nudges everything the mates move. The mate result now reports
   `degenerate`. The unused `finiteDifferenceStep` option is gone.
3. **Negligible Jacobian entries are dropped before diagnosis**
   (`AssemblyClosureDiagnosis`). An entry is dropped when moving its
   variable over its characteristic range (a radian, or the assembly scale)
   changes the tolerance-scaled row by less than 1e-6. Without this, the
   diagnosis's row equilibration blew roundoff up into a unit row, and a
   singular pose looked regular. Test: a folded two-link arm whose tip is
   at its mated distance from a point on its own line is reported as a
   degenerate placement with one degree of freedom.
5. **A connector's x and the mates it takes come from its feature.**
   - `cad_axis` gained the geometry's own reference direction, and
     `cad_face_axis` now also covers planes. A connector's x is that
     reference; only a straight edge, which has none, falls back to the world
     axis least aligned with z.
   - Each connector records its feature: `plane`, `axis`, `sphere`,
     `circle` or `line`.
   - `GeometricConnectors.compatible` refuses mates that ask a feature for
     something it does not define:
     - a point (coincident, distance): only a sphere's or circle's center;
     - an axis line (coaxial): an axis, a circle or a line;
     - a plane (planar): a planar face or a circle's plane;
     - a direction (parallel, perpendicular, angle): anything but a sphere;
     - a whole frame (lock): only a circle.

     A planar face's centroid moves as the face grows, which is why it is not
     a point.
   - Checked when an assembly is written to a document and again when it is
     reframed.
6. **Geometric connectors live in the assembly record.**
   - `AssemblyConnector.reference` is an opaque string. projectkit only
     validates it as text, so the codec, the flattener and the solvers
     handle these connectors by name like any other.
   - `frame` holds the last resolved frame.
   - `GeometricConnectors.reframe(definition, geometry)` /
     `AssemblyDocuments.reframe(root)` is the one explicit step that frames
     them again from geometry. `toDefinition` is pure again: no geometry
     evaluation, and it returns the last frames.
   - Gone: the CadKit `Definition` property, the stripping of connectors
     from stored records, and the requirement to call `apply` before
     validation.
   - Errors carry codes: `unresolved`, `ambiguous`, `instance-dependent`,
     `incompatible-mate`; documents prefix them with `assembly.`.
7. **Edge fingerprints are anchored at the midpoint** (document version 10).
   - A first-vertex anchor depends on the edge's orientation and, for a
     closed edge, on where its seam vertex lies.
   - Fingerprint records carry `anchor`; edge records from older documents
     (no anchor) keep matching by their first vertex.
   - MachineKit's recovery of pre-v9 defaults compared against
     `DocumentCodec.VERSION`, so any version bump would have misfired. It
     now compares against a pinned constant, 9.

### C4.5 — Editor: mates (2026-10-01, in progress)

What the editor has: project parts arrive as meshes in the scene artifact,
with no B-rep. The assembly definition is generated by the project process,
and the editor never edits or saves it. Assembly editing today means
dragging an occurrence (`AssemblyDrag` over the dependent joints) and
setting joint coordinates in the inspector. There is no mate, connector or
joint authoring, no two-pick tool, and no diagnosis display.

Design:
- **Face data in the artifact.** The project process, which has the B-rep,
  describes each face's feature, frame data and fingerprint. The editor
  captures and reframes geometric connectors from that data alone.
- **Mates as an editor-owned overlay.** Mates and the connectors they
  create live in the project scene record next to `assemblyState`. The
  effective definition is the generated one plus the overlay, re-applied
  after every project rebuild. (Writing them back into the project's
  recipe document is a later option.)

Steps:
- **a.** Face descriptors and data-only matching.
- **b.** Session mates: solve, undo, save, diagnosis status.
- **c.** The two-pick mate tool.
- **d.** Dragging parts under their mates (a soft target in the mate solve).
- **e.** Convert to joint.

**a (done).** Face descriptors and data-only matching:
- `GeometricConnectors.describeFaces(shape)` writes JSON per face: index,
  feature, origin, direction, reference, radius, signed, fingerprint.
- `SceneArtifactPart.faceDescriptors` carries it (scene artifact version
  11); the MachineKit and CadKit example producers fill it.
- `GeometricCandidates` holds the faces and edges a connector may be found
  on, built from a Shape or from descriptors, and capture, framing and
  `reframe` all go through it. `captureDescribed` captures from
  descriptors.
- Matching is data-only:
  - `TopologyFingerprint.scoreAgainst` compares two fingerprints, and
    `score(Shape)` is now that applied to the candidate's capture, so there
    is one scoring rule;
  - `TopologyResolver.resolveAmong` resolves over fingerprints under the
    same ambiguity policy.
- Test: a bore captured from descriptors frames like one captured from the
  shape, and an assembly reframed from the widened plate's descriptors
  seats the pin at the new bore.

**b (done).** Session mates:
- `app/ProjectAssemblyMates` is an immutable overlay of face connectors
  (per component) and mates.
  - `effective(generated)` lays it over the generated definition and checks
    the mates against their features.
  - `reframe` frames the connectors again from the rebuilt artifact's
    descriptors.
  - `faceConnector` captures (or reuses) "face<index>" from the descriptors.
  - `solve` places the parts and merges the result into the current state,
    so the coordinates the mates don't reach keep their values.
- `ProjectDocumentSession`:
  - `addAssemblyFaceMate` / `addAssemblyConnectorMate` /
    `removeAssemblyMate` are undoable edits that swap the overlay and the
    state.
  - A mate that cannot hold is kept, the placement is left as it was, and
    the conflict is reported.
  - On open, rebuild and recipe refresh, `installAssemblyRuntime` reframes
    and re-solves (`layMates`); a lost face becomes `assemblyMateProblem`.
  - `assemblyMateStatus()` feeds the status bar (shown in red for a
    conflict or a problem).
  - The overlay is saved as `ProjectSceneRecord.assemblyMates`.
- `AssemblyMateSolveResult.implied` lists the mates that add nothing (the
  rank is unchanged without them). That is what the status calls redundant.
  `report.redundantOwners()` also lists overlapping mates, such as the
  usual planar-plus-coaxial pair, which share the axis rows on purpose.
- Fixture `app/tests/fixtures/pin-plate` (a grounded plate with a bore, and
  a free pin). `ProjectSourceTests.checkMates` covers:
  - planar: 3 degrees of freedom; plus coaxial: 1, with the pin at
    (30, 20, 10);
  - undo/redo, and save and reopen;
  - a contradicting planar offset reported as a conflict with the placement
    kept, then removed.
- Limits: root-scope occurrences only (not parts inside nested assemblies);
  faces only, not edges.

**c (done).** The two-pick mate tool:
- `app/MatePickTool` takes the clicked faces.
  - It refuses at once a face whose feature cannot take the mate, a click
    on nothing, and a second face on the same part; the first pick is kept
    so the user can pick again.
  - After the second face it adds the mate and reports the mate status.
- The perspective viewport routes left clicks on faces to the tool while it
  is active (`beginMatePick`). The viewport toolbar shows its prompt, and
  Escape cancels.
- Commands `assembly.mate-{planar,coaxial,parallel,perpendicular}` are in
  the viewport context menu, enabled when the project has described faces.
- The inspector lists the selected part's mates as checked rows under
  "Mates"; clearing one removes the mate (undoable).
- Two haxeon fixes found on the way:
  - `ad78d26c`: a switch value's case may end in an `if` without `else`.
  - `d11c48fb`: a source file is only the module its package declaration
    names. A package-scoped root was also reached as a plain root, which
    loaded `Runner.hx` twice when a type was written as `Runner.Scene`.
- Tests: `ProjectSourceTests.checkMatePick` (refusals, completion,
  inspector removal and undo).

**d (done).** Dragging under mates:
- `cadkit.modeling.AssemblyMateDrag` drags a point of a mated part. Each
  update makes two Levenberg-Marquardt solves from the previous preview:
  1. The mates and closures at their tolerance-scaled weight, plus a pull
     of the grabbed point toward the target with weight 1 (rows in length
     units), which finds the nearest placement the mates allow. A seated
     pin pulled sideways turns about its axis.
  2. The mates and closures alone, so the preview satisfies them exactly.

  The result reports whether the point is on target, to a thousandth of
  the assembly scale, or held ("Held by its mates N mm from the cursor").
- A first pull weight of `1/scale` was too light: Levenberg-Marquardt
  judged the problem stationary and stopped 19 mm short.
- `AssemblyMateSolver.setup` (the movable coordinates and the mate and
  closure problem) is shared by the solve and the drag;
  `AssemblyMateSolver.merge` folds a placement into an existing state (the
  editor overlay now uses it).
- In the editor, `beginAssemblyDrag` uses `ProjectMateDrag` for a part
  that has mates and the existing IK drag otherwise. Previews go through
  `previewAssemblyPoses`; a commit is one undoable edit.
- Tests:
  - `MateSolverSmoke.checkDrag`: the seated motor turns about its shaft
    and stays seated; pulled off the face it is held; the commit equals the
    preview; the grounded bracket refuses.
  - `ProjectSourceTests.checkMateDrag`: the pin dragged around the bore
    follows and stays in it; undo and redo; the grounded plate refuses.
- Limit: for definitions with nested assemblies the drag and solve report
  flattened ids, and `merge` refuses them (as `place` already did).

**e (done).** Convert to joint:
- `cadkit.modeling.AssemblyMateJoints.infer` is for a free root mated to one
  other part. It takes the null space of its mates' rows over the part's six
  rigid-body columns: the eigenvectors of JᵀJ by Jacobi rotations, with the
  translation columns scaled by the assembly scale.
  - One free motion: a pure slide makes a prismatic joint along it; a turn
    with no pitch makes a revolute joint through `o + ω×v/|ω|²`.
  - Otherwise none, with the reason: fixed, several motions, a screw motion,
    mated to more than one part, or already on a joint.
- `convert` builds the tree joint (coordinate 0 at the current placement)
  and its two connectors, `<joint>-parent` and `<joint>-child`, z along the
  axis.
- In the editor the overlay gains joints (`withJoint` swaps out the mates it
  replaces; pruning keeps connectors that a joint names). The session's
  runtime definition is now the generated definition plus the overlay.
  - `overlayDefinition` reframes and lays the overlay before a saved state
    is decoded, on open and on recipe refresh.
  - Mate edits keep the definition in step with the overlay.
  - `convertMatesToJoint` is one undoable edit that changes the structure
    (the hierarchy rebuilds); the converted part then moves, drags and
    simulates on its joint.
  - Command `assembly.convert-to-joint`, plus an inspector button "Make
    revolute joint" when the selected part's mates make one.
- haxeon `3de4d97a`: a try body may be a bare `return`.
- Tests:
  - `MateSolverSmoke.checkJoints`: planar + coaxial → revolute that keeps
    the placement and turns about the shaft; coaxial + parallel x axes →
    prismatic along z; coaxial alone → none ("2 motions").
  - `ProjectSourceTests.checkMateJoint`: a single planar mate makes no
    joint; with the coaxial mate a revolute joint plate→pin; the pin keeps
    its place and turns on the joint; joint and coordinate survive save and
    reopen; undo restores the mates.
