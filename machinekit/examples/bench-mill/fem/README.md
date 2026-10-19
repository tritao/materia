# Bench mill FEM tools

Status and reproduction record for [FEM_PLAN.md](../FEM_PLAN.md). This study
uses a bonded-frame, rigid-axis **upper-bound** model and assumed catalogue
springs. Numerical qualification does not establish machining accuracy.

The serial nine-pose modal and static numerical gates pass. All 18 nose/bar
static studies pass both support bounds and two resolved quadratic mesh levels.
Maximum casting share is **18.792%** of the machine loop excluding
the job-dependent tool, against the provisional **30%** target. All eight center
tool-case budgets pass, including explicit steel/carbide cutter options. Physical modal
acceptance remains open; no `MillFrameLayout` geometry has been edited.

## Current qualified serial follow-up

The nine-pose undamped upper-bound **modal numerical gate passes** with
`solverThreads: 1`, 12 modes, both supports and 40/30 mm meshes at every pose.
All 15 minimum-section probes pass at both levels; Jacobians are positive.
Maximum frequency-order refinement change is **2.992%**, below 5%.
The retained four-thread experiments are unqualified audit data.
Their apparent third-level mesh needs do not apply to this serial matrix.

| Serial mode order | Three-point range, Hz | Four-spring range, Hz |
| --- | --- | --- |
| 1 | 47.431–49.678 | 41.460–44.312 |
| 2 | 60.751–65.657 | 45.976–48.905 |
| 3 | 75.015–78.693 | 75.942–78.655 |
| 4 | 82.769–87.365 | 84.083–92.007 |
| 5 | 111.501–130.289 | 112.225–122.262 |
| 6 | 146.405–157.562 | 123.847–131.141 |

First-mode travel changes are −2.49…+2.13% / −3.52…+3.12%, with shape MAC
≥0.9938 / 0.9835. First-three nearest casting-sample MAC values stay ≥0.935. Several higher/nearby modes share a best casting-only target; physical identity requires the mass-motion/phase evidence. The four-spring second
mode's continuous column residual spans 0.0739–0.1070, crossing the arbitrary
0.1 flag at some upper poses without a new mode appearing. Its tracked
frequency changes −3.31…+2.85%, with MAC ≥0.973. The ~78 Hz center mode has
65% / 80% of kinetic energy in the Z-drive lumped group; it is a coupled mode,
not automatically pure column bending. Reported kinetic localization does not
locate strain energy.

Pose/shape tracking, effective masses, relative nose/bar motion, point-mass
kinetic localization and rotation/tooth-pass frequency-equality speeds are in
`build/fem/nine-pose-serial/modal-report.md` / `.json`.
Aggregate JSON SHA256:
`e860a8a455a503f7c18bd2694abcfe07872815594ce4ac76293d3a81c7253b5b`.
Physical modal acceptance remains open. The BT30 concept's 8,000 rpm limit
corresponds to 133.333 Hz rotation; at `xplus_yminus_zupper`, the four-spring
mode 7 is **132.751 Hz**, only **0.437%** away. Its last refinement changes
**1.885%**, exceeding that separation. The normalized nose gap is 0.498 of the
casting-motion peak and 66.2% of kinetic energy is in meshed solids, so this
mode warrants a relative-path/operating-band review. Mode-shape ratios describe
normalized eigenvectors. Declared machining/motor/axis bands and joint/material
uncertainty remain required. See `build/fem/nine-pose-serial/excitation-context.md`. The serial nine-pose nose/test-bar static sweep has
passed all eighteen decision-reference studies. All eight center tool-context budgets pass. The modal operating-band review is open; no CAD geometry has changed.

## Qualified center decision budgets

Both center reference cases now pass serial 40/30 mm statics, both support
bounds, minimum-section resolution and all numerical/energy checks. The test
bar's maximum unit-response refinement change is **1.3925%**. Casting shares
below use the machine loop excluding tool/bar bending; full loop values remain
separate in each report.

| Reference | Support | X | Y | Z |
| --- | --- | --- | --- | --- |
| Spindle nose | Three-point | 17.37% | 14.27% | 14.16% |
| Spindle nose | Four springs | 15.84% | 14.27% | 14.16% |
| Steel 20 mm / 40 mm bar | Three-point | 14.23% | 10.48% | 13.26% |
| Steel 20 mm / 40 mm bar | Four springs | 13.08% | 10.48% | 13.26% |

The center maximum is **17.37%**, below the provisional 30% target. The casting
geometry decision uses the nine-pose maxima below and still requires modal
operating-band review. No geometry change has been made. The bar report is
`build/fem/nine-pose-serial/center/test-bar-20/report.json`, SHA256
`dc99cc07ed78e313c401c93862acff37742efb31ca631a0a7c2d7d27bba85cc7`.
All eighteen travel/reference static studies now pass. Contextual cutter budgets reuse their three center contact solutions for
steel/carbide variants; the qualified results appear below.


## Qualified nine-pose static gate

Maximum casting share: **18.792%**, `xplus_yplus_zlower`, `soft-three-point`, `spindle-nose`, X.
All nominal shares below 30%: **True**. These results remain conditional on casting and catalogue/joint stiffness assumptions.

| Pose | Three-point maximum | Four-spring maximum |
| --- | --- | --- |
| center | 17.374% | 15.844% |
| xminus_yminus_zlower | 13.748% | 13.200% |
| xminus_yminus_zupper | 14.682% | 14.682% |
| xminus_yplus_zlower | 16.791% | 14.758% |
| xminus_yplus_zupper | 17.723% | 16.136% |
| xplus_yminus_zlower | 15.457% | 14.848% |
| xplus_yminus_zupper | 14.682% | 14.682% |
| xplus_yplus_zlower | 18.792% | 16.545% |
| xplus_yplus_zupper | 18.566% | 16.136% |

Each entry is the maximum over X/Y/Z and the two decision references.
The reports retain full C matrices, principal directions, ±500 N load-box
corners, support differences, reciprocity, sign/scaling, equilibrium and
element-group energy/sensitivity checks at every pose.

Aggregate: `build/fem/nine-pose-serial/report.json`; SHA256 `b84c5ab338a8868008aa99cd280e25730a8a426b9d0b50821f9732082f8c7164`.
The ~132.75 Hz corner mode and missing declared operating bands still prevent
a claim of physical modal acceptance or selection of a lightened design.

## Qualified tool-case budget gate

Center pose; casting shares use the machine loop excluding the job-dependent
tool. Only spindle-nose and test-bar rows govern the casting decision.

| Case | Support | Casting X/Y/Z | Full-loop worst ±500 N | Machine-only worst ±500 N |
| --- | --- | --- | --- | --- |
| spindle-nose | soft-three-point | 17.37%/14.27%/14.16% | 112.464 µm | 112.464 µm |
| spindle-nose | stiff-four-springs | 15.84%/14.27%/14.16% | 112.045 µm | 112.045 µm |
| test-bar-20 | soft-three-point | 14.23%/10.48%/13.26% | 132.064 µm | 123.490 µm |
| test-bar-20 | stiff-four-springs | 13.08%/10.48%/13.26% | 131.684 µm | 123.117 µm |
| cutter-6-steel | soft-three-point | 12.36%/8.73%/13.22% | 411.765 µm | 135.171 µm |
| cutter-6-steel | stiff-four-springs | 11.39%/8.73%/13.22% | 411.365 µm | 134.809 µm |
| cutter-6-carbide | soft-three-point | 12.36%/8.73%/13.22% | 230.412 µm | 135.171 µm |
| cutter-6-carbide | stiff-four-springs | 11.39%/8.73%/13.22% | 230.024 µm | 134.809 µm |
| cutter-10-steel | soft-three-point | 11.18%/7.67%/13.22% | 264.827 µm | 144.823 µm |
| cutter-10-steel | stiff-four-springs | 10.32%/7.67%/13.22% | 264.451 µm | 144.470 µm |
| cutter-10-carbide | soft-three-point | 11.18%/7.67%/13.22% | 186.309 µm | 144.823 µm |
| cutter-10-carbide | stiff-four-springs | 10.32%/7.67%/13.22% | 185.944 µm | 144.470 µm |
| cutter-16-steel | soft-three-point | 9.48%/6.22%/13.22% | 225.303 µm | 163.475 µm |
| cutter-16-steel | stiff-four-springs | 8.78%/6.22%/13.22% | 224.954 µm | 163.136 µm |
| cutter-16-carbide | soft-three-point | 9.48%/6.22%/13.22% | 184.979 µm | 163.475 µm |
| cutter-16-carbide | stiff-four-springs | 8.78%/6.22%/13.22% | 184.635 µm | 163.136 µm |

Component-by-component compliance and tool fractions, material/stick-out
assumptions and FE group energies are retained in `build/fem/tool-cases-serial/report.md`.
JSON SHA256: `a79a500edfca8ad5ae80e6c1c90309f211e2bac0c291c658f5170837bbe10035`.
This is a conditional catalogue budget, not measured accuracy or modal acceptance.

## Conditional lightening parameter proposal

[lightening-candidates.json](lightening-candidates.json) records proposed
parameters only; none is evaluated or selected and CAD geometry is unchanged.
The static-share gate is complete; modal operating-band review remains a prerequisite.

| Parameter family | Baseline | Proposed levels | Constraint |
| --- | --- | --- | --- |
| Existing `roof` | 60 mm | 55, 50 mm | Relieved base roof becomes 35, 30 mm; column clamp-pad depth/position also change |
| New `baseRibThickness` | 40 mm | 36, 32 mm | Keep full rail-insert caps, foot regions and insert bosses |
| New `columnCrossRibThickness` | 40 mm | 36, 32 mm | Keep front drive web, sole and clamp stations |
| New `columnWallThickness` | 40 mm | 36 mm, later | Keep base and local supports at 40 mm; shared `wall<40` fails foot-pocket guard |

Screen each family separately using actual CAD mass, both supports, resolved
new minimum sections, nose/bar machine-loop share and mode shapes. A proposed
comparative screen is casting share <30% and relevant frequencies >=95% of
baseline; this is not a physical acceptance criterion. Tighten finalist mesh
convergence to 1% or better and assess excitation clearance with numerical and
material/joint uncertainty. The current 0.437% frequency separation cannot be
resolved by a 1% mesh gate. Full nine-pose checks precede combining families.
No mass savings are asserted before rebuilding the candidate CAD.

## Tool cases and model contract

The casting decision uses the machine loop **excluding job-dependent tool
bending**, evaluated at both the spindle nose and a solid steel 20 mm diameter,
40 mm gauge test bar. The nose has no tool or holder stick-out. Context cases
are 6/10/16 mm cutters at assumed 20/30/45 mm stick-outs, with 35 mm holder neck
and effective cutter section inertia fraction 0.5. Steel (210 GPa) and carbide
(600 GPa) are explicit assumed material options. Test-bar holder neck is zero;
the taper connection remains. Each contact moves both FE reference points and
catalogue-network lever arms. Merely removing the tool spring is insufficient.
Full-loop and machine-only budgets are reported separately. Tool cases are
center-pose context; the modal mass model retains the actual CAD tool.

Material overrides are keyed by MaterialLibrary IDs: `mineral-cast` uses
E=35 GPa, nu=0.25, density=2400 kg/m³; `machined-steel` uses E=210 GPa, nu=0.30,
density=7850 kg/m³. Sources and assumptions are retained in `frame-study.json`
and each report. Catalogue/joint stiffnesses remain conditional hardware
proxies; supplier selection, casting modulus and joint/contact uncertainty are
open inputs. The modal represented mass is 465.539 kg: 301.968 kg meshed,
143.570 kg lumped, and 20 kg payload.

## Qualification and provenance

The fixture gate includes the real Gmsh C3D10/tet10_order/deck path and a known
rigid-equation translation/rotation. The six-mode distributing mass/inertia
fixture matches analytical frequencies to 2.41e-7 relative error. Full tensors,
including products of inertia, are reproduced by six symmetric point masses.
A zero-stiffness SPRINGA graph supplies MPC mass sparsity required by CalculiX
2.21. A solid modal cantilever gives 830.100 Hz versus the Euler reference
835.517 Hz (0.65%); its two meshes differ by 0.0043%. Run
`modal_solid_fixture.py --ccx PATH` for that additional fixture. All 37 Python
tests pass, including corrupt-cache, loaded-reference reaction, thread-count
and tool-masking negative controls.

All nine packets pass the 224-part concept comparison, per-part density/mass
and full native inertia checks. The 36 fixed-frame STEP hashes are identical
across poses. Repeat manifest/STEP exports are byte-identical. The center
matrix packet SHA256 is
`c96473be097a62a1410c45fdb45b3498d3e4bc53db26ed06ae366c4a52b7381e`.
The exporter captures root and relevant submodule working-tree patches,
untracked sources, and loaded haxeon/CadKit/OCCT build identities. Reports
retain frozen packet/source/config/binary hashes and raw solver evidence.
Use fresh output names when reproducing cited runs.

Only **one-thread** solves qualify the aggregates. Four-thread experiments
produced inconsistent results on identical decks: a static Z force residual
was ~0.0113 N versus ~3.2e-12 N serial, and one modal frequency was 236.269 Hz
versus 222.280 Hz serial. The cause is not established. Their retained
`build/fem/nine-pose-upper-bound` output is audit evidence, not a qualified
gate. Earlier four-thread status statements are superseded; Git retains the
chronological record. Resume checks deck, successful solver log, exact mesh
cache and thread count; mixed/missing/parallel thread reports are rejected.

Static unit cases execute independently at time 0–1. Derived combined DAT
labels map to hashed raw case inputs/results. Four-spring leveling first
estimates preload from hard-seat gravity reactions corrected by RF − CLOAD,
then verifies actual loaded heights (<=1e-6 mm) and gravity equilibrium.
A failed calibration gate falls back deterministically to measured 4×4 foot
flexibility; failed unit-case numerical checks stop. At center both methods
produce bit-identical compliance matrices on all four mesh/support combinations.
The original flexibility-only run is preserved at
`build/fem/center-nose-influence-qualified` with an archive-location map.

Generated artifacts live behind the ignored `build/fem` symlink at
`/media/joao/DEV/materia-fem-runs/bench-mill-20261019`. Completed static FRDs
may be losslessly gzip-archived: `field-archives.json` records verified raw and
compressed hashes and source report/deck identities. Inputs, DAT files and
logs remain intact. Modal FRDs remain raw for mode-motion post-processing.
The shared heavy-job queue and private resource guard preserve host memory
headroom; do not lower the guards to force concurrent solves.

## Run the pose and tool sweeps

From the repository root, export the matrix with the already loaded CAD build:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fem.json -- --pose-matrix true --name repeat-poses
```

With the pinned Python environment and CalculiX executable configured:

```sh
fem_python="$PWD/machinekit/examples/bench-mill/build/fem/venv/bin/python"
fem_ccx="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx"
export LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$fem_python" machinekit/examples/bench-mill/fem/pose_sweep.py machinekit/examples/bench-mill/build/fem/repeat-poses --output machinekit/examples/bench-mill/build/fem/repeat-study --ccx "$fem_ccx" --threads 1 --resume
"$fem_python" machinekit/examples/bench-mill/fem/sweep_report.py machinekit/examples/bench-mill/build/fem/repeat-study
python3 -m unittest discover -s machinekit/examples/bench-mill/fem -p 'test_*.py'
```

The pose driver completes all nine modal studies before the eighteen static
nose/bar studies; `--stage modal` or `--stage static` resumes either stage.
Each heavy study enters the shared queue. `tool_cases.py ROOT` combines two
decision references plus three converged center steel-contact studies under
ROOT (`spindle-nose`, `test-bar-20`, `cutter-6-steel`, `cutter-10-steel`,
`cutter-16-steel`). Material-only carbide variants reuse the identical FE
contact solutions, producing eight tool-case budgets in total.

## Export and check the CAD packet

Run from the repository root. These Haxe configurations share their compiler
output directory, so run them sequentially.

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fem-tests.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fem.json -- --name repeat-center
python3 machinekit/examples/bench-mill/fem/packet.py machinekit/examples/bench-mill/build/fem/repeat-center/manifest.json
python3 machinekit/examples/bench-mill/fem/test_fem.py -v
```

The default pose is X=0, Y=0, Z=-125 mm. Other joint positions and packet names
are explicit arguments; invalid coordinates or names fail before export:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fem.json -- --name repeat-corner --x 125 --y 80 --z -250
```

`MillFemExport` exports all frame members, including the separate steel inserts,
clamp bolts and feet, plus the head, carriage, saddle and table. Other components
retain their individual CAD/declared mass, centre of mass and pose as
`mass-only` records. The static study allocates their gravity through rigid-axis references or
least-norm force/moment transfer to the represented ancestor. Modal attachments retain full native centroidal inertia tensors, rotated into
world axes and represented by symmetric point-mass constellations.

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
machinekit/examples/bench-mill/build/fem/venv/bin/python machinekit/examples/bench-mill/fem/mesh_pilot.py machinekit/examples/bench-mill/build/fem/repeat-center/manifest.json
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
  machinekit/examples/bench-mill/build/fem/repeat-center/manifest.json \
  --ccx "$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx" \
  --sizes 40 30 --threads 1 --tool-case spindle-nose \
  --output machinekit/examples/bench-mill/build/fem/repeat-static-study
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
series compliance. A provisional 30% casting share target is judged on the machine loop
excluding the job-dependent tool, using the spindle nose and stiff test bar
across the nine poses and both supports. This is a conditional budget;
selected supplier data and joint measurements remain open inputs.

The first-milestone numerical and budget gates pass for the frozen center run.
Later work includes the distributing-coupling assembly model and uncertainty/contact
checks. The modal inertia/export and nine-pose numerical gates now pass;
the revised static travel budget passes. Geometry candidates stay deferred
until that budget and modal review justify them.
