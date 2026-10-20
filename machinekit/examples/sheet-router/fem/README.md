# SR5 structural screen

The current SR3/SR4 assembly is rejected for fabrication. The structural screen is
complete; the SR-D3 targets are not met. A redesigned carriage/guide layout must
be studied before SR6 releases drawings.

The shared `machinekit.structural.fem.AssemblyFemExport` writes posed STEP,
materials, mass, centre of mass, world centroidal inertia and bounds from any
`MachineAssembly`, with a caller-supplied solid selection. Both the mill and router
use it. Fresh mill exports before and after extraction match in all 220 component
records and every byte of the 36 STEP bodies; source/build provenance is excluded
from that comparison because the exporter source changed.

## Reproduce

Run from the repository root. Reuse the pinned mill Gmsh 4.12.2 environment and
CalculiX 2.21 installation; no native build or new Python environment is needed.
Use the project's `heavy-run` resource gate for compilation and the full study.
The example gate below is the workstation tool used for this run.

```bash
heavy_run=/home/joao/dev/materia-cache/tools/heavy-run
fem_python=machinekit/examples/bench-mill/build/fem/venv/bin/python
fem_ccx="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx"
export LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

"$heavy_run" --mem 10G --need 1G --label sr5-export -- machinekit/scripts/test-sheet-router-fem
"$fem_python" machinekit/examples/sheet-router/fem/verify.py --ccx "$fem_ccx" --output build/sheet-router-sr5/verification
"$heavy_run" --mem 10G --need 1G --label sr5-study -- "$fem_python" machinekit/examples/sheet-router/fem/study.py --packets build/sheet-router-sr5 --ccx "$fem_ccx"
"$fem_python" machinekit/examples/sheet-router/fem/report.py build/sheet-router-sr5
"$heavy_run" --mem 10G --need 1G --label sr5-eccentric -- machinekit/scripts/test-sheet-router-fem --y-loads
```

`--checks-only` runs the Haxe checks without exporting. The optional verifier
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
