# Bench mill FEM tools

The current implementation exports a geometry packet, validates its physical
datums, verifies CalculiX with analytical fixtures, and meshes the two mineral
castings independently. It does not yet calculate assembled-machine stiffness.
The implementation sequence and remaining gates are in [FEM_PLAN.md](../FEM_PLAN.md).

## Export and check the CAD packet

Run from the repository root. These Haxe configurations share their compiler
output directory, so run them sequentially.

```sh
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
`mass-only` records. These records still need an attachment/load-transfer model;
they are not already connected finite elements or modal inertia tensors.

The version-1 `manifest.json` contains:

- `geometryFrame: world` and `poseApplication: already-applied`. STEP coordinates
  are already posed; applying `poseMm` again would move the solids twice.
- Stable component IDs, material IDs, posed bounds, volumes, masses, centres of
  mass, and hashes of each STEP file. Writer timestamps in STEP headers are
  fixed to the Unix epoch for repeatable file hashing; geometry is preserved.
- Source revision, tracked working-tree diff hash, and exporter source hash.
  Untracked files outside the exporter are not covered by the diff hash; retain
  the packet's exact geometry hashes when using it as analysis input.
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
It uses only Python's standard library. `packet.elastic_input` converts Pa and
kg/m³ into MPa and tonne/mm³ for the consistent mm/N/tonne/s solver system.
The CAD packet keeps mm/kg/Pa labels; a solver adapter must use the conversion.

```sh
python3 machinekit/examples/bench-mill/fem/solver_fixtures.py --ccx /path/to/ccx
```

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
  python3 machinekit/examples/bench-mill/fem/solver_fixtures.py \
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

## Next implementation step

Add named interface face regions and pilot the casting/steel mesh partition.
Then configure explicit elastic inputs, support constraints, guide/drive load
paths and column-clamp behavior for the first upper-bound static model. Assign
a physically consistent tool/workpiece load pair and distribute its loads to
the interfaces. Report displacement relative to the same pose's gravity-only
state. Mesh the full load path and pass equilibrium/convergence checks before
using any result to change the casting geometry.
