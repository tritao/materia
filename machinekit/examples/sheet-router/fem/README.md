# SR5 structural screen

## Guide-first candidate (SR6, 2026-10-20)

The current CAD implements the guide-first revision in [SOURCING.md](../SOURCING.md).
The initial SR5 results below describe the superseded single-block assembly.
The new candidate uses two X rails/four blocks, moving Z rails/four fixed blocks,
280 mm Y block spacing, flat 25/20 mm X/Z plates, 20 mm steel feet and a direct
two-level spindle saddle. It retains the original steel beam and nominal rack size.
Z is 1610, 200 mm travel and 10 m/min; X and braked Z are 750 W.

New exports default to `build/sheet-router-sr6`. Set `SHEET_ROUTER_FEM_OUTPUT`
to place packets on another volume; pass that same path to `study.py`, `report.py`
and the eccentric-load checker. `SHEET_ROUTER_FEM_BINARY`, `SHEET_ROUTER_BINARY`
and `SHEET_ROUTER_SIZING_BINARY` relocate compiler outputs independently.
This run uses `/media/joao/DATA1/materia-router-sr6/final` for packets
and `/media/joao/DATA1/materia-router-sr6/logs` for checks, because root is nearly full.

The updated model uses per-block radial/lateral translation springs at actual CAD
connectors and zero guide rotational springs. X/Z plate grillages span the contact
locations. Moving Z rails receive a conservative 60% gross rectangular-section
reinforcement proxy; mounting stiffness remains assumed. The spindle saddle is
a rigid body supported by four translational mounting springs; clamp, bearing
and tool flexibility are excluded. Lateral guide stiffness and plate Poisson
coupling still require qualification.

The table reacts through six actual bed-stock seats rather than one nearest stock.
Floor-standing blower and cabinet devices do not contribute to frame mass. Other
fixed masses attach to nearest stock on a fixed 50 mm analysis grid. Twenty modes
are reported, including global frame modes. Gantry-dominant modes use a declared
50% moving-mass kinetic-energy criterion; this is a screening classification,
not a validated separation of frame and gantry dynamics. Static and first-six-mode
convergence gates remain 3% and 5%. Per-case raw field files are gzip-compressed
with integrity verification; decks, tables, logs and source snapshots remain.

The settled CAD passes 426 assembly/clearance assertions for both beds, 292
component assertions, 82 revised SR4 sizing/planner assertions, and the guide/reach
gate. All five legacy gantry hashes and 59 axis assertions remain unchanged.
The shared exporter/centreline fixtures pass (33 mill plus seven router checks),
as do the independent beam/spring native fixture, mill packet/STEP comparison
and 37 mill Python regressions. These checks do not establish the structural
targets. The current candidate remains a screening design, with no fabrication release. The shoe envelope is 150 mm diameter with a 40 mm brush
and a front-directed short hose neck; its lift and rack clearances are checked
with the same assembly poses.

### Settled candidate results

All 36 solves (two meshes, nine full-Z poses, two beds) pass the numerical gates.
Static compliance changes by at most 1.092%; all twenty frequencies by 0.875%.
Maximum force-balance error is 0.0292%, energy error 0.0108%, reciprocity error
0.000031%, and artificial rigid-member compliance 0.0752%. Every support retains
positive compression under the combined envelope.

| Bed Y | Worst X stiffness | Worst Y stiffness | Global first-mode range | Lowest classified gantry-dominant mode |
|---|---:|---:|---:|---:|
| 2600 mm | 1.154 N/µm | 0.481 N/µm | 5.727–6.027 Hz | 27.249 Hz |
| 3150 mm | 1.005 N/µm | 0.480 N/µm | 5.177–5.468 Hz | 30.760 Hz |

The 2 N/µm X/Y and 40 Hz gantry targets still fail. The modal classification is
only the declared ≥50% moving-mass energy screen: mixed frame/gantry modes below
that threshold remain relevant and the global frame modes are not dismissed.
The lower spindle datum necessary for rack clearance leaves approximately 428 mm
from beam centre to longest-tool tip at full Z. Shorter-tool reach remains open.

In the worst Y cut, the Z plate plus its moving-rail reinforcement account for
about 56% of compliance; the X plate contributes about 13% and the gantry beam
about 9%. The worst X case is led by the base/frame contribution. Investigate the
Z load path and frame bracing/supports before selecting a box X carriage. These
are model-based priorities, not fabrication approval.

Worst reserved eccentric Y branch demand is 874.60 N. Both drives pass torque,
reducer, preload and conditional manufacturer rack checks; the mounting-derated
rack limit is 3000 N, versus 935.77 N after including preload-flank demand.
The 715.08 MPa Hertz diagnostic remains visible without
using the old assumed 700 MPa limit to resize the rack. The specified hardened
family, mounting and application factors still require qualification.

The long-bed Y drive-only estimate is 49.58 Hz at 175.86 kg. Halving hinge
stiffness gives 38.61 Hz; 10% extra mass gives 47.27 Hz. Hinge compliance is 64.9%
of the series drive compliance. These estimates do not replace the full structure.

`summary.json`, `results.json`, `eccentric-drives.json`, posed STEP files, native
decks/tables, compressed field files and solve-time source snapshots remain in
the external packet directory named above. `summary.json` records the actual
postprocessor SHA separately from the solve-time source snapshot.

## Initial SR5 baseline (historical)

The current SR3/SR4 assembly is rejected for fabrication. The structural screen is
complete; the SR-D3 targets are not met. A redesigned carriage/guide layout must
be studied before SR6 releases drawings.

The shared `machinekit.structural.fem.AssemblyFemExport` writes posed STEP,
materials, mass, centre of mass, world centroidal inertia and bounds from any
`MachineAssembly`, with a caller-supplied solid selection. Both the mill and router
use it. Fresh mill exports before and after extraction match in all 220 component
records and every byte of the 36 STEP bodies; source/build provenance is excluded
from that comparison because the exporter source changed.

## Reproduce the current candidate

Use the preceding SR6 model description for current runs. The model description
and baseline results below this command section are historical.

Run from the repository root. Reuse the pinned mill Gmsh 4.12.2 environment and
CalculiX 2.21 installation; no native build or new Python environment is needed.
Use the project's `heavy-run` resource gate for compilation and the full study.
The example gate below is the workstation tool used for this run.

```bash
heavy_run=/home/joao/dev/materia-cache/tools/heavy-run
fem_python=machinekit/examples/bench-mill/build/fem/venv/bin/python
fem_ccx="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx"
export LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

"$heavy_run" --mem 10G --need 1G --label sr6-export -- machinekit/scripts/test-sheet-router-fem
"$fem_python" machinekit/examples/sheet-router/fem/verify.py --ccx "$fem_ccx" --output build/sheet-router-sr6/verification
"$heavy_run" --mem 10G --need 1G --label sr6-study -- "$fem_python" machinekit/examples/sheet-router/fem/study.py --packets build/sheet-router-sr6 --ccx "$fem_ccx"
"$fem_python" machinekit/examples/sheet-router/fem/report.py build/sheet-router-sr6
"$heavy_run" --mem 10G --need 1G --label sr6-eccentric -- machinekit/scripts/test-sheet-router-fem --y-loads
```

`--checks-only` runs the Haxe checks without exporting. `--export-only`
exports a geometry iteration without repeating the unchanged exporter fixtures;
run those fixtures separately before completing a study. `--layout` checks the
new guides, moving-rail reach and selected drives; `--baseline` checks legacy
gantry hashes and axis behavior. The optional verifier
arguments `--mill-before <folder> --mill-after <folder>` compare fresh legacy mill
packets. `study.py --one` is a diagnostic centre-pose run; it writes partial output
and cannot produce a completed study. The full study publishes `results.json`
only after every solve, force/energy check and convergence gate passes. Per-case
input decks, solver logs and numerical tables remain beside the report.

## Model and limits

The packet includes all 165 posed components, seven selected STEP bodies, 24 RHS
stock descriptions, plate dimensions, nine full-Z poses and six foot locations.
Each bed length has a 3 × 3 corner/midspan grid at Z = 250 mm. At each pose:

- X/Y/Z cutting cases apply 300 N to the tool and the opposite force to the bed;
- gravity includes all authored part masses;
- X and Y acceleration cases apply 3 m/s² to the corresponding moving masses;
- six modes are solved with the same restraints and mass model.

Gmsh meshes quadratic lines. CalculiX B32R BOX sections represent the hollow
stocks; RECT sections represent full-width plate strips. The beam feet are strips
between the two Y blocks, 140 mm apart. The X plate joins the X block to the Z
column head; the Z carriage is a strip from its block to the spindle attachment.
Stiff offsets carry forces and moments to actual tool/guide positions. Each joint
uses translation and rotation reference nodes and six independent springs.
Direct SPRING2 rotational DOFs on beam nodes are unsuitable here; the analytical
fixture verifies the reference-node coupling.

Assumed inputs are steel E = 210 GPa, aluminium E = 69 GPa, nu = 0.3, guide
translation = 100 kN/mm and individual guide rotation = 10 MN·mm/rad. Bolt springs
are 1 MN/mm and 1 GN·mm/rad. Feet have 50 kN/mm vertical springs and three horizontal
restraints in a 3-2-1 arrangement. The actual SR4 rack/reducer/preload and screw
stiffnesses supply the held-axis drive springs. They include elastic compliance;
backlash, tuning and following error are excluded.

Native stock sections carry distributed mass. Base stock densities are scaled to
match the CAD weldment union mass, avoiding double-counting stock intersections.
Remaining part masses, centres and full inertia tensors are aggregated per rigid
carrier, then represented by six positive point masses. Entire cable chains move
with their carrier, matching SR4's conservative allowance. Fixed accessory masses
are transferred to nearby frame supports; separate accessory floor supports are
not modelled.

The bed reaction is a rigid link to the nearest authored bed stock member. Table
diaphagm action and distributed table attachment are excluded. Spindle mount,
spindle bearings, tool and local tube-wall deformation are rigid/omitted. No weld,
bolt/contact, stress, buckling, fatigue, damping or nonlinear qualification is
claimed. The very large calculated deflections exceed the useful precision range
of a linear model: they reject this layout rather than predict cutting accuracy.

CalculiX's authored beam/section and output definitions are documented in the
[official manual](https://www.dhondt.de/ccx_2.20.pdf). Line connectivity follows the
[Gmsh API reference](https://gmsh.info/doc/texinfo/): Gmsh start/end/mid nodes are
reordered to CalculiX start/mid/end.

## Result, 2026-10-20

| Bed Y | Worst X stiffness | Worst Y stiffness | First mode across poses | Largest gravity frame twist |
|---|---:|---:|---:|---:|
| 2600 mm | 0.006106 N/µm | 0.005849 N/µm | 2.7318–2.7329 Hz | 0.00825 mm |
| 3150 mm | 0.006100 N/µm | 0.005849 N/µm | 2.7316–2.7327 Hz | 0.00902 mm |

Targets are ≥ 2 N/µm XY and ≥ 40 Hz for the first gantry mode. The lowest mode has
substantial tool-to-bed motion, rather than being an irrelevant accessory mode.
The report retains all six frequencies and relative tool-motion indicators.

Worst full-Z X compliance is about 164 µm/N: the X guide contributes 90, the X
plate 45, and the Z guide 26 µm/N. Worst Y compliance is about 171 µm/N: the X plate
contributes 92, X guide 60 and Z guide 15 µm/N. The X plate term alone rejects the
2 N/µm target even if the guide assumptions are made much stiffer. Budgets are
strain-energy contributions for the closed tool/bed load loop, including springs.
Support reactions and acceleration displacements are retained for every case.

The eccentric Y branch sees up to 827.65 N after the 1.5 reserve, including the
300 N cutting vector envelope, Y acceleration and 15 N side drag. The servo torque
and preload still pass, but the original 20 mm engaged face reaches 706.06 MPa
against SR4's assumed 700 MPa contact limit. A **25 mm engaged face on both rack
and pinion** passes that tooth screen (about 632 MPa); its physical fit and the
redesign's changed moving mass must be checked before changing the selected parts.

Decisions for the next design iteration:

- Retain **steel RHS 200 × 100 × 6.3** as the beam baseline. Same-section aluminium
  has only 0.329 times the bending rigidity. Enlarging the beam alone cannot cure
  the dominant carriage/guide terms.
- Replace the open 12 mm aluminium X plate with a **closed box bridge**. Use spaced
  X rails and blocks to support roll as well as pitch/yaw; two blocks on the same
  X rail alone do not fix roll about that rail.
- Replace the single Z block with spaced blocks/rails and obtain supplier stiffness.
- Reject the **12 mm aluminium beam feet** for final design. **20 mm steel** is the
  next candidate, with about 14 times the strip bending rigidity. This is a design
  choice to package and recheck, not an analysed replacement assembly.
- Keep tubes **empty**. Filling the beam adds 39.31 kg/m to its 28.43 kg/m steel mass.
  Even a perfectly bonded 30 GPa fill gives only 1.37/1.24 times strong/weak bending
  rigidity; an unbonded fill gives no credited stiffness. Damping is unquantified.

These candidates are not claimed to meet SR-D3. SR6 must begin with structural
redesign and rerun SR4/SR5 before fabrication drawings are released.

## Verification

The 100/50 mm global meshes use four-times-finer plate-strip spacing. Across all
18 poses, maximum diagonal compliance change is 0.61% and maximum change of the
first six frequencies is 0.31% (gates 3% and 5%). The 2 m RHS cantilever agrees
with the stated bending/shear formula within 2%, and the rotational-spring
fixture within 0.07%. Halving its mesh changes deflection by 0.67%.

Each solve checks positive compliance, summed energy against external work, and
support force equilibrium. Postprocessing checks compliance reciprocity, negligible
artificial-member compliance and compressive foot reactions under gravity plus
the cutting vector and both acceleration envelopes. The smallest remaining foot
compression is about 75 N within this assumed foundation model. The Haxe gate
passes seven shared-exporter/frame-data checks and 33 existing mill datum checks;
the existing mill Python suite passes 37 tests.

Generated evidence is under `build/sheet-router-sr5/`: `summary.json`,
`results.json`, `verification/verification.json`, `y-loads.json`, and
`eccentric-drives.json`. Packet, solver executable, study source and report hashes
are recorded. Generated files are intentionally outside project history.
