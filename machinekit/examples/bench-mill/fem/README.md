# Bench mill FEM tools

The tools export and validate CAD packets, verify CalculiX with analytical
fixtures, and solve a conformal casting/steel frame. The first useful milestone
now passes: an upper-bound
static baseline at the center pose, both support bounds, and a conditional
tool-to-workpiece compliance budget. This does not qualify machining accuracy.
The implementation sequence and remaining gates are in [FEM_PLAN.md](../FEM_PLAN.md).

## Gate status

- Solver-fixture gate: passed, including the common Gmsh-to-C3D10 deck path,
  known prescribed translation/rotation, reference-node reaction correction,
  and three analytical spring/leveling cases.
  Root/submodule patch snapshots and loaded build identities are now captured.
- Export gate: passed on 224 parts at center and a travel corner, including
  same-build concept comparisons, per-part density checks, fixed-frame STEP
  hashes across poses, and byte-identical full manifest/STEP repeat verification.
  The Haxe datum contract passes 33 assertions and Python packet checks pass 14 tests.
  An additional intermediate pose passes the 224-part comparison, with its
  requested pose automatically added to the concept reference.
- First-milestone numerical gate: passed at the center pose, including both
  support bounds, all 15 thickness probes on two counted meshes, the full
  compliance matrix/load-box reduction, equilibrium, reciprocity, scaling/sign
  checks and a base-casting energy-sensitivity cross-check.
- Geometry screening: the old cutter-dominated budget is superseded as a decision basis.
  The 30% target is judged on the machine loop excluding the job-dependent tool,
  at spindle nose and a 20 mm steel / 40 mm gauge test bar.
  Nine-pose upper-bound modal convergence passes; new reference-point statics
  and their travel screening are running. Assembly joints, contact/uncertainty
  and physical modal acceptance remain open.

## Tool-reference and modal follow-up

The machine-loop decision excludes the job-dependent tool. The revised budget
uses `spindle-nose` (no holder stick-out) and `test-bar-20` (solid steel,
20 mm diameter, 40 mm gauge). Context cases use 6/10/16 mm cutters at assumed
20/30/45 mm stick-outs, in steel and explicitly assumed 600 GPa carbide.
`static_study.py --tool-case NAME` moves the FE load and rigid-workpiece
reference, and all lumped-network lever arms, to the selected contact.
`tool_cases.py build/fem/tool-cases` combines converged studies; material-only
changes reuse the identical FE contact solution. It reports total and
machine-loop-without-tool budgets separately and judges casting share on the
latter for the two decision cases only. The machine-only decision contract passes a known masking test: total-loop
casting share 3.1%, tool share 90%, machine-only casting share 31%; the revised
decision detects the 30% target. Context cutters cannot trigger it, and mixed
CAD packets are rejected. All 25 Python tests pass. Numerical tool-case results
remain pending.

The repeatable sequence is `pose_sweep.py build/fem/modal-poses --output
build/fem/nine-pose-upper-bound --ccx PATH --threads 4 --resume` (paths shown
relative to this example). It runs all nine modal studies before the two static
decision cases at all nine poses. `sweep_report.py` compares them; `tool_cases.py`
adds the contextual cutter material variants after their three center FE contact
studies. All studies enter the shared heavy-job queue independently.

The modal mass/coupling analytical gate passes: six modes of a spring-supported
platform with a full mass/inertia attachment match analytical frequencies to
2.41e-7 relative error. Three Python checks verify complete inertia (including
products), physical triangle inequalities, and recovery of a known rigid motion
by the distributing coupling. Six symmetric point masses reproduce each
centroidal inertia; they add no stiffness. A zero-stiffness `SPRINGA` graph
supplies the off-diagonal mass sparsity missing for MPC-coupled `MASS` elements
in CalculiX 2.21; omitting it fails this fixture. The platform has explicit
0.1 kg per support node as well as the 12 kg inertia attachment.

Modal export gate passed on all 224 parts: native component centroidal tensors
rotated into world coordinates, physical positive/triangle checks, packet mass
and concept checks, and byte-identical manifest/STEP repeat export. No assumed
CAD fallback was needed in this packet. The retained `modal-center` packet SHA256
is `5f31a21c2645cb06e7bcce07bfcda6d86d6d79e1d0cd82ba9d5cb5e9893b5229`.
`MillFemExport --pose-matrix true --name NAME` exports center and eight corners
from one loaded build. All nine packets now pass the part/concept, mass/inertia and pose checks; all
36 fixed-frame STEP hashes remain identical across the matrix. The pose index
is `build/fem/modal-poses/poses.json`; center packet SHA256 is
`c96473be097a62a1410c45fdb45b3498d3e4bc53db26ed06ae366c4a52b7381e`.
The modal travel comparison passes its numerical gate; the static comparison is running.

The root filesystem filled during concurrent work. Generated FEM artifacts were
moved intact to `/media/joao/DEV/materia-fem-runs/bench-mill-20261019`, with the
usual ignored `build/fem` path retained as a symlink. Cited baseline hashes and
provenance are preserved. A tool-case stiff-support run interrupted by disk
exhaustion is not counted as evidence and must finish on resumption.
A second modal fixture checks solid density, units and C3D10 mass/stiffness
through the real node reorder and deck writer: a 100×10×10 mm steel cantilever
(E=210 GPa, artificial nu=0) gives **830.100 Hz**, versus the **835.517 Hz**
Euler–Bernoulli slender-beam reference (0.65% difference, within 5%). The 944 /
3,586-tet meshes differ by just **0.0043%**. The 3D model includes shear and
rotary inertia absent from that Euler comparison. Run `modal_solid_fixture.py
--ccx PATH`; the small report is retained at
`build/fem/modal-solid-fixture/report.json`, with source/binary hashes and
root/submodule patch snapshots.

The center upper-bound modal numerical gate now passes for 12 modes and both
supports on the 40/30 mm partitioned meshes. All thickness probes pass and
Jacobian determinants are positive; maximum frequency-order mesh change is
**2.55%**, below 5%. Total represented mass is **465.539 kg**, including 20 kg
payload; all mass-only components retain their full centroidal inertia through
exact symmetric point-mass constellations. Distributing attachments use their
represented ancestor; head/table/foot ancestors remain physically rigid.

| Fine mesh support | First six frequencies, Hz |
| --- | --- |
| Three-point | 48.643, 63.612, 77.587, 84.522, 122.089, 154.586 |
| Four vertical springs | 42.973, 47.550, 78.022, 87.581, 119.864, 127.465 |

The four-spring model adds low support-motion modes. Its first mode with the
column-bending proxy above 0.1 is at 78.022 Hz; the three-point model has mixed
column/support motion from 48.643 Hz. This proxy removes best-fit rigid motion
at the column bottom before measuring the top residual; inspect mode shapes
and crossings before identifying physical modes. No release limit follows.
The report includes effective masses, participation, contact motion and SVG
shape plots: `build/fem/modal-center-study/report.md` / `report.json`.
Report JSON SHA256:
`af7808033b7fbb77c45155ef34bb84d3eefd0fba45ee7a5449b1eb110774555a`.
All cited analysis source hashes matched the files at the gate.

CalculiX accepts explicit `--threads 1|2|4` (default 1). The nine-pose runner
requests 4 and records it in every report. On a byte-identical coarse frame
deck, all 12 printed frequencies match the one-thread result exactly; elapsed
time was 42.56 s versus 65.87 s. The six-mode analytical fixture also passes
with 4 threads. This is an execution setting, not a stiffness change. A resume
requires the same deck, successful log and requested thread count. Detailed
comparison is retained in `build/fem/modal-thread-check/comparison.json`.

Exact mesh snapshots now preserve binary floating-point values through JSON
round trips and verify source/config/Gmsh/geometry hashes before reuse. The
real coarse modal deck is byte-identical after cache reload. A cache-corruption
negative control passes; all 23 Python tests pass. Interrupted solves still
require an identical input deck and a completed solver log before reuse.

Frequency stability is a numerical gate; acceptance against spindle/tooth-pass,
motor and axis excitation requires declared operating bands. No lightening
candidate is selected and no `MillFrameLayout` geometry has been edited.

## Recorded center baseline

The run uses a frozen, validated 224-part packet at X=0, Y=0, Z=-125 mm.
Later CAD edits/export commands do not replace this run's inputs. Both models
bond the casting interfaces and use rigid moving axes; the numbers below are
frame contributions. The budget adds the separately documented lumped springs.

| Partitioned target | C3D10 elements | Soft support worst box | Stiff support worst box |
| --- | --- | --- | --- |
| 40 mm | 37,573 | 18.9162 µm | 18.6055 µm |
| 30 mm | 60,372 | 19.1470 µm | 18.8306 µm |

The maximum change in any unit-response vector is **1.40%**, against the 5%
target. All 15 normal-section probes pass at both levels, with at least six
quadratic tetrahedra intersected by each probe and conformal mid-planes imposing
two normal layers. Final Jacobians are positive. The support bounds change the
worst ±500 N-per-axis load-box response by **1.68%**. The 1% base-modulus increase
reduces X compliance by 0.3460%, versus 0.3491% predicted from its strain energy
(relative discrepancy **0.89%**, within the 2.5% gate).

| Casting share of total loop | X | Y | Z |
| --- | --- | --- | --- |
| Soft three-point | 0.88% | 0.61% | 12.31% |
| Stiff four-spring | 0.81% | 0.61% | 12.31% |

These historical shares include the job-dependent 6 mm steel tool and must
not decide casting geometry. The revised decision uses spindle-nose and short
stiff test-bar cases, with the FE contact and all lever arms moved explicitly.
Cutter cases (6/10/16 mm, assumed stick-outs 20/30/45 mm) are contextual; steel
and an explicitly assumed 600 GPa carbide option are separate inputs.
Lightening parameters will be proposed only after modal and travel screening;
no MillFrameLayout geometry has been changed.
The full matrices, principal directions, eight load-box corners, gravity
reactions, element-group energy shares and source/preload assumptions are in
`build/fem/partitioned-baseline/report.md` and `report.json`.

Run identity (retain the generated frozen packet and provenance snapshots):

- Packet SHA256: `1032888e88c98ab70be584909a01d441c785318233e030c2b4e945dbcd65698b`.
- Configuration SHA256: `3041e949a49d1fe0a927b88bbd949fbabdb86c01e1604bc7c24e6c9fb5e11847`.
- Report JSON SHA256: `9e91182f77eec121a17993e7d80b886f274a15add456716a29147beded850cf1`.
- Geometry source revision: `c52c085e1f3222d9683ef32d887be6fe1c981bea`, with captured root patch
  `f081166f0d35ed6f1933106fd8c8700107df72ee488d8a9c783c436f372b52e2`.
- Analysis source revision: `af5e4d36ad4e68926969082f8afc58a622677eb2`, with captured root patch
  `81c347de6a88faa778c3623aed66df6ae123909e60b19147985278196f48409d`.

The report and its frozen packet also record initialized submodules' own
patches, loaded haxeon/CadKit/OCCT build identities, Gmsh 4.12.2 and CalculiX
2.21 binary hashes, case/deck/result hashes, and mesh warnings. No incomplete
or unpartitioned pilot is counted in this convergence result.

## Export and check the CAD packet

Run from the repository root. These Haxe configurations share their compiler
output directory, so run them sequentially.

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fem-tests.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fem.json
python3 machinekit/examples/bench-mill/fem/packet.py machinekit/examples/bench-mill/build/fem/center/manifest.json
python3 machinekit/examples/bench-mill/fem/test_fem.py -v
```

The default pose is X=0, Y=0, Z=-125 mm. Other joint positions and packet names
are explicit arguments; invalid coordinates or names fail before export:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fem.json -- --name corner --x 125 --y 80 --z -250
```

`MillFemExport` exports all frame members, including the separate steel inserts,
clamp bolts and feet, plus the head, carriage, saddle and table. Other components
retain their individual CAD/declared mass, centre of mass and pose as
`mass-only` records. The static study allocates their gravity through rigid-axis references or
least-norm force/moment transfer to the represented ancestor. Modal attachments
and centroidal inertia tensors remain pending.

The version-1 `manifest.json` contains:

- `geometryFrame: world` and `poseApplication: already-applied`. STEP coordinates
  are already posed; applying `poseMm` again would move the solids twice.
- Stable component IDs, material IDs, posed bounds, volumes, masses, centres of
  mass, and hashes of each STEP file. Writer timestamps in STEP headers are
  fixed to the Unix epoch, and process-global `Product N` labels become stable
  member IDs. These metadata changes preserve geometry and repeatable hashes.
- Source revision, root and initialized submodule revisions, their actual tracked
  patches and hashes, and snapshots of untracked source files. Loaded haxeon,
  CadKit and OCCT binaries are hashed, with CadKit CMake/OCCT version identities.
  Dirty-tree runs retain these snapshots under the packet provenance directory.
- Separate cast-in interface pairs and four column clamp pad pairs. Their
  `connection` is deliberately `unassigned`: CAD mates do not set bond/preload.
- Four support stations with floor and seat datums, twelve guide block datums,
  and measured positive joint directions in world axes. Positive X/Y joints
  move the table in world -X/-Y; positive Z moves the head upward.
- The spindle/tool tip, table top, and a payload reference 50 mm above the table
  in its physical upward direction. The payload reference is not a verified
  cutting contact. At different Z heights a workpiece/contact definition must
  make the load pair physically consistent before a cutting solve is run.
- Material-library density and optional modulus snapshots. Mineral-cast density
  2400 kg/m³ and modulus 35 GPa remain assumptions. Poisson ratio, joint data,
  and some metal moduli remain unassigned.

`MillFramePreview` records independent per-part physical reference data at the
center, zero and eight corner poses in `mill-frame-concept.json`. The export
generates a packet-local copy through the same concept report writer, using
the same loaded CAD build, and checks every volume, mass, centre and bound,
plus tool/table/payload datums. Requested intermediate poses are added to the
reference set. The physical-reference writer records that it has not rerun the
preview clearance audits. Tolerances are recorded in the validation output.

`packet.py` checks hashes, units, representations, material references, positive
masses, normalized poses, casting volume/density consistency, total mass sums,
interfaces, support/guide references, and the payload datum. `MillFemExport`
checks valid connected solids and casting/insert overlap using CAD booleans.
`MillFemChecks` exercises XYZ motion, datum independence, and invalid travel.
`test_fem.py` adds negative controls for corrupted geometry, missing/duplicate
parts, wrong units, double pose application, invalid elasticity and results.

## Verify the solver

The pilot pins CalculiX **2.21** and Gmsh **4.12.2**. `solver_fixtures.py`
requires an explicit available CalculiX executable and refuses another version.
The complete fixture gate also requires the pinned Gmsh environment.
`packet.elastic_input` converts Pa and
kg/m³ into MPa and tonne/mm³ for the consistent mm/N/tonne/s solver system.
The CAD packet keeps mm/kg/Pa labels; a solver adapter must use the conversion.

```sh
python3 -m venv machinekit/examples/bench-mill/build/fem/venv
machinekit/examples/bench-mill/build/fem/venv/bin/python -m pip install -r machinekit/examples/bench-mill/fem/requirements.txt
machinekit/examples/bench-mill/build/fem/venv/bin/python machinekit/examples/bench-mill/fem/solver_fixtures.py --ccx /path/to/ccx
```

The report also includes a Gmsh C3D10 tension case through the same
`tet10_order`, node/element/equation writers and solver runner used by the
frame. Its reference-node load tests the correction `reaction = RF - CLOAD`.
A second case prescribes both translation and rotation about an offset
reference and checks every coupled node against `u = t + theta × r`.
The spring fixture checks unloaded support settlement, self-weight leveling via
equivalent initial spring forces, and a separate unit load increment. Ground
DOFs remain zero in that formulation.

Both C3D10 known-answer checks pass; the rotation fixture's maximum discrepancy
was 5e-9 mm in the initial verification.

The generated `build/fem/fixtures/report.json` records version, executable/input/
output hashes, displacements, reactions, force/moment equilibrium and mesh
refinement. The checks are one C3D8 tension element (1% tolerance) and three
C3D20 cantilever meshes (5% analytical tolerance and 5% final refinement change).
The fixture uses 210 GPa, zero Poisson ratio and a 100 x 10 x 10 mm bar.
The clamp-to-tip load gives `FL/(EA)` in tension and `FL³/(3EI)` in bending;
the latter comparison allows the small transverse-shear/end-load effects.
Only the support-node `RF` values are used for reaction balance, and no external
loads are applied to those nodes. See the [CalculiX author’s manual](https://www.dhondt.de/ccx_2.20.pdf)
for consistent units, C3D8/C3D20 connectivity and `*NODE PRINT` conventions.

On Ubuntu 24.04, a local test installation can be extracted under the ignored
build directory without changing system packages:

```sh
mkdir -p machinekit/examples/bench-mill/build/fem/tools
cd machinekit/examples/bench-mill/build/fem/tools
apt-get download calculix-ccx=2.21-1 libspooles2.2t64=2.2-14.1build2
dpkg-deb -x calculix-ccx_2.21-1_amd64.deb calculix
dpkg-deb -x libspooles2.2t64_2.2-14.1build2_amd64.deb calculix
cd ../../../../../..
LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  machinekit/examples/bench-mill/build/fem/venv/bin/python machinekit/examples/bench-mill/fem/solver_fixtures.py \
  --ccx "$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx"
```

The remaining runtime libraries must be available on the host. This package
version and architecture are specific to the tested host; use an appropriate
2.21 installation elsewhere. A missing executable/library or solver failure
produces a nonzero exit and no successful report.

## Mesh the casting geometry

Install Gmsh into an isolated environment under the generated output directory:

```sh
python3 -m venv machinekit/examples/bench-mill/build/fem/venv
machinekit/examples/bench-mill/build/fem/venv/bin/python -m pip install -r machinekit/examples/bench-mill/fem/requirements.txt
machinekit/examples/bench-mill/build/fem/venv/bin/python machinekit/examples/bench-mill/fem/mesh_pilot.py machinekit/examples/bench-mill/build/fem/center/manifest.json
```

The pilot validates the packet, imports each casting STEP, verifies its world
bounds and CAD volume, and generates second-order tetrahedra. It checks positive
Jacobian determinants and mesh volume within 0.1% of CAD volume. Generated
`base.msh`, `column.msh`, and their report are under `build/fem/mesh-pilot/`.
`--size` changes the maximum element size in mm; the default is 30 mm. This is
a geometry/toolchain pilot, not a converged structural mesh. The [Gmsh API](https://gmsh.info/doc/texinfo/)
provides the import, mass, bounds and element-quality operations used here.

## Run the upper-bound static study

```sh
LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  machinekit/examples/bench-mill/build/fem/venv/bin/python machinekit/examples/bench-mill/fem/static_study.py \
  machinekit/examples/bench-mill/build/fem/center/manifest.json \
  --ccx "$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx" \
  --sizes 40 30 --output machinekit/examples/bench-mill/build/fem/static-study
```

The driver freezes a validated copy of the packet under its output directory.
It captures analysis source/submodule patches before solving and rejects changes
to its Python sources or configuration during the run. CAD build identities
come from the frozen input packet. `--resume` reuses a completed solve only when
the regenerated input deck is byte-identical; all result gates are evaluated again.
On this shared workstation, run large meshes through the machine-wide
`/home/joao/dev/materia-cache/tools/heavy-run` queue with adequate memory headroom.
An unpartitioned 20 mm solve was terminated by the host's earlyoom service
when available memory
fell below its threshold; that incomplete run is not a passed result.

`frame-study.json` keys materials by `mineral-cast` and `machined-steel`, retains
library densities and records assumed elastic overrides. The nominal steel
values agree with the [ECCS reference values (§5.2.1)](https://www.steelconstruct.com/wp-content/uploads/PROGRESS_Design_guide_final-version.pdf);
applying them to unspecified machined steel remains a study assumption. Every packet part gets
a density/volume/mass comparison. The solver separately checks each meshed part's
density against its packet mass. Computed masses must agree; an explicitly
declared catalogue mass is retained and its density comparison is reported.

The model is explicitly an **upper bound**: casting/insert and column pad pairs
are bonded, moving axes are rigid, and guide/drive patches use kinematic
`*EQUATION` coupling. These equations stiffen the rail/seat regions. Physically
rigid foot nut annuli use `*RIGID BODY`; foot rotations remain free. Other lumped
gravity loads use `distribute_wrench`, the least-norm force allocation that
preserves their force and moment at the represented ancestor. This static load
allocation is not a modal mass/inertia attachment model.

Two named supports are required: `soft-three-point` uses one XYZ anchor, a
second XZ anchor, a third Z anchor and a vertically free fourth foot;
`stiff-four-springs` uses four vertical springs and the same lateral anchors.
The three-point triangle has two rear feet to keep the rear-heavy machine's
self-weight reactions compressive. Four-spring leveling uses gravity reactions
from a four-seat calibration: equivalent initial spring forces represent each
foot's precompression, while prescribed ground DOFs stay zero. The analytical
spring fixture checks settlement, leveling, unit response and element energy.

Independent X/Y/Z unit force pairs form the relative 3x3 compliance matrix;
gravity is a separate reaction case. Reports include all eight corners of the
±500 N load box, principal compliances/directions, reciprocity, half-load/sign
checks, and force/moment balance. CalculiX `ELSE` totals split strain energy
between base casting, column casting, inserts, clamp pads and support springs.
One base-casting stiffening solve cross-checks its X energy sensitivity.

The mesher inserts conformal planes at the minimum sections' mid-depth,
without changing material volume. These impose two quadratic layers without
uniformly refining every long wall. Every counted convergence level must pass
CAD-derived normal section probes
through the thin floors, walls, ribs and roofs: at least two quadratic elements
and no sampled element chord exceeding half the section thickness. The original
unpartitioned 40 mm
pilot fails this gate and does not count toward convergence. Partitioned levels
are checked separately. Reports retain
Gmsh warnings and final Jacobian checks. Stress/traction output and local
qualification at interface/constraint edges remain pending.

`compliance-budget.json` records catalogue proxies, preload classes, links and
explicit assumptions for unselected hardware. The budget adds guides, screw/
nut/support bearings, the X return belt reflected through its lead, clamp
contact, spindle bearings, tool holder and the current tool to frame FEM.
Supports already inside FEM are split by energy, without adding duplicate
series compliance. A provisional 30% casting share target controls geometry
screening for this pose, tool and assumed hardware. This is a conditional budget;
selected supplier data and joint measurements remain open inputs.

The first-milestone numerical and budget gates pass for the frozen center run.
Later work includes the distributing-coupling assembly model and uncertainty/contact
checks. The modal inertia/export and nine-pose numerical gates now pass;
the revised static travel budget is running. Geometry candidates stay deferred
until that budget and modal review justify them.

The nine-pose four-thread **modal numerical gate is provisional pending serial verification** for all 12 modes, both supports,
positive Jacobians and resolved minimum sections on every counted level.
Maximum last-refinement frequency change across poses is **3.59%** (limit 5%).
Two upper corners needed 27 mm third levels: `xminus_yplus_zupper` resolves
6.42% (40→30) to 1.14% (30→27); `xplus_yplus_zupper` resolves 17.83% to 1.41%.
Failed comparisons and pressure-interrupted solves are retained and not counted
as passing evidence. The nine-pose nose/test-bar static sweep is now running.

Lowest frequencies span **47.431–49.678 Hz** with three-point supports and
**41.460–44.312 Hz** with four springs. Shape tracking of the center first mode
has MAC ≥0.993 / 0.984 and frequency changes −2.49…+2.13% / −3.52…+3.12%.
The third, Z-drive-coupled mode spans 75.015–78.693 Hz / 75.872–78.655 Hz
(check the linked matrix for exact values). First-three-mode MAC stays ≥0.932.
The four-spring second mode crosses the arbitrary 0.1 column residual flag
at some upper corners: its continuous ratio changes from 0.0827 at center to
0.0759–0.1070, with MAC ≥0.973. Thus the first flagged frequency can change
from ~78 Hz to ~46 Hz without a new mode appearing. Use tracked shapes and
continuous residuals, not that flag alone. Operating-band acceptance remains
open; no candidate is selected.

Full pose comparisons, source hashes, motion/kinetic diagnostics and individual
mode-shape links: `build/fem/nine-pose-upper-bound/modal-report.md` / `.json`.
The runner requires 10 GiB available at startup (4 GiB need plus 6 GiB reserve).
`resource_guard.py` additionally monitors host pressure during the full run:
below 4.5 GiB available it terminates only large private CCX processes within
these output directories and `heavy-mill-*` scopes. It retries those pressure
interruptions with exact-deck resume; numerical errors are not retried. Shared
cgroup memory/no-swap limits and host earlyoom remain enabled. Active work
and logs are under `build/fem/nine-pose-upper-bound` and
`build/fem/follow-up-final.log`. No interrupted solve qualifies as evidence.

Modal mass-motion post-processing now reads the native FRD generalized mass,
checks mode/frequency and displacement normalization against the immutable
solver report, and decomposes the exact lumped mass/inertia kinetic energy.
The known six-mode platform reproduces its analytical kinetic fractions;
fixed-width negative FRD fields and process ownership/retry controls are tested.
All **32 Python tests** pass. Rigid reference-motion checks also verify spindle-nose/test-bar points and cancellation of a common rigid motion.

At center, the ~78 Hz mode carries **65% (three-point) / 80% (four-spring)** of
its kinetic energy in the 14.442 kg Z-drive mass group attached to
`frame/column/driveUpperLeft`. The first two modes carry roughly 54–60% in the
meshed frame. These are coupled modes; the casting shape proxy alone must not
be used to call the ~78 Hz mode pure column bending. Kinetic localization does
not locate strain energy. Diagnostics are in each completed pose's
`modal/mass-motion.md` / `.json`, and the modal matrix report links them.

Nine-pose modal aggregate JSON SHA256: `36054d2c6938de2f8ee398117022653095993d43cc4f03206c9f7ccec34c8cea`.


### Four-spring leveling correction

The new spindle-nose static run exposed an inadequate calibration: rigid-reference
RF-derived preloads missed loaded foot heights by up to 0.65 µm, and their gravity
resultants did not close. This run failed its gate and is not cited as a passing
static result. `leveling.py` now measures the actual four-spring model's vertical
4×4 foot flexibility plus unloaded-preload gravity deflection, then solves for
four positive compressive preloads. Reciprocity and the final loaded heights
remain checked; no tolerance is relaxed. Unit-force steps retain zero preload.
The coupled four-seat solver fixture recovers known preloads within 6.73e-7
relative error (`build/fem/leveling-influence-fixture/report.json`); it runs in
static preflight. All **33 Python tests** pass. On the center coarse frame the
maximum residual loaded foot height is 4.25e-9 mm, below the original 1e-6 mm gate.
The fine frame and travel sweep remain pending. Historical RF-calibrated spring
results are superseded for gravity/leveling by this correction; their no-gravity
linear unit-load matrices are retained as historical evidence only.
The already completed modal decks have no gravity/precompression and are unaffected;
the static helper change is recorded after their immutable source snapshots.


### Serial solver qualification in progress

A repeated center static check exposed nondeterministic four-thread results:
the identical isolated Z deck failed equilibrium with four threads, while its
one-thread result recovers the previously consistent response and closes
reactions near round-off. Isolating load cases alone did not cure this.
The earlier one/four-thread modal spot comparison is insufficient to qualify
all studies. Four-thread numerical gates are therefore provisional; a full
serial nine-pose modal matrix will precede the serial static sweep.
Raw four-thread results remain under `build/fem/nine-pose-upper-bound` for audit;
replacement runs are `build/fem/nine-pose-serial` and `tool-cases-serial`.
No geometry decision uses the provisional results. `pose_sweep.py` now defaults
to one thread. Static cases also run in fresh processes at solver time zero;
raw case DAT/deck/log hashes are recorded, with case-time relabeling only in
an explicitly derived concatenated DAT. The known coupled-seat solver fixture
passes that path; all **34 Python tests** pass. No gate tolerance is relaxed.


Serial center modal numerical gate passes on both meshes/supports; maximum
frequency-order refinement change remains 2.55%. Coarse frequencies and fine
three-point frequencies match the archived four-thread results exactly; the
fine four-spring maximum difference is 0.0031%. The full serial matrix remains
in progress. Center serial report: `build/fem/nine-pose-serial/center/modal/report.json`,
SHA256 `980b58cd4f3a541b6cf1ff4263a97799aadce97e6dbe1aa0233238ebcd4fd08c`.
The byte-identical static-deck thread comparison is retained at
`build/fem/static-thread-qualification/comparison.json`: force resultant ~0.0113 N
with four threads versus ~3.2e-12 N with one thread. `--modal-size-map PATH`
lets the runner explicitly request previously necessary third mesh levels per
pose without relaxing any numerical checks.
