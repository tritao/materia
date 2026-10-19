# FEM implementation plan for the mineral-cast bench mill

## Purpose and boundary

Build a repeatable structural study of the `MillFramePreview` mechanical concept.
The primary result is elastic displacement of the tool tip **relative to the
workpiece contact point** under cutting loads across the 250/160/250 mm XYZ
travel. Report mass, support reactions, local interface loads, and mode shapes
alongside that displacement. Use the baseline to compare changes to the base,
column, and column foot. This is an engineering study, not manufacturing
qualification or a prediction of machining accuracy by itself.

Start with a linear static model and an undamped modal model. Add detailed
contact, thermal distortion, forced vibration, fatigue, and insert anchorage
strength only after the baseline is reproducible and the necessary input data
exist. Keep the working legacy `BenchMill` and its motion/control model separate
from this mechanical concept.

## Existing inputs and gaps

- `MillFramePreview.hx` constructs the complete concept and writes
  `mill-frame-concept.mtrg` and `mill-frame-concept.json`.
- `MillDesignStudy.hx` writes `design-review/review.json`, computes static guide
  admission, and exports twelve selected STEP parts in **local** coordinates.
  It does not export every steel insert or an assembly placement manifest.
- `MillFrameLayout.hx`, `MillFrame.hx`, `MineralCasting.hx`, and
  `MineralFoundation.hx` define the separate base and column castings, internal
  voids, steel inserts, four column clamp stations, and four leveling feet.
  Assembly `fixed` mates describe geometry, not measured joint stiffness.
- `MaterialLibrary.hx` assigns mineral cast an **assumed** density of
  2400 kg/m³ and Young's modulus of 35 GPa. It also has assumed damping and
  thermal values. No Poisson ratio, casting strength, steel-to-casting bond
  properties, or measured joint stiffness is supplied for the FEM study.
- The present guide calculation uses a 20 kg payload 50 mm above the table,
  ±500 N per world axis, ±1 m/s² acceleration, and static guide safety factor
  two. These are study envelopes, not a validated cutting-force history.

## Implementation sequence

### 1. Fix the analysis contract and solver workflow

Create `machinekit/examples/bench-mill/fem/` for source-controlled scripts,
configuration, small reference fixtures, and result-schema documentation.
Use generated files under `machinekit/examples/bench-mill/build/fem/`; do not
commit meshes, solver databases, STEP exports, or result dumps. Choose one
versioned, scriptable mesh/solver workflow, initially Gmsh plus CalculiX if it
passes the geometry and interface pilot. Record exact versions and invocation
in the run manifest. A FreeCAD FEM workflow can be used if it proves more
reliable for these CAD solids, but keep solver-specific code behind the same
export and result contracts. Fail clearly when a tool is missing; do not
silently use different solvers between candidates.

Define one canonical unit system at the boundary: CAD positions and STEP in
mm; analysis input in mm, N, tonne, s (stress in MPa; density in tonne/mm³),
or an explicit SI alternative chosen once for the solver adapter. Document
every conversion, especially `youngsModulusPa` and kg/m³ from `MaterialLibrary`.
Store the chosen units in every generated manifest and reject missing or
ambiguous units.

**Gate:** a single-element tension fixture and a meshed cantilever reproduce
analytical displacement and reaction within 1% and 5%, respectively, after
appropriate refinement; a wrong-unit fixture fails validation.

### 2. Export an assembly-aware analysis packet

Extend `MillDesignStudy.write` or add a dedicated `MillFemExport` called by
`MillFramePreview.main`. Use `assembly.solvedPoses()` and `PosedParts` to export
the **posed** base and column castings and all structural steel interfaces,
including Y/Z guide seats, column pad pairs, and four foot seats. Include the
head, carriage, saddle, and table where the selected fidelity level requires
their flexibility; otherwise represent their masses and load points explicitly.
Do not collapse mineral cast and steel into one material. Keep stable member
IDs such as `frame/base/casting` in filenames and metadata.

Write `manifest.json` with schema version; repository revision and relevant
working-tree diff hash (or a clean-tree requirement); geometry and
configuration hashes; units; component ID, material ID, volume, mass, STEP path,
and pose; named contact/interface pairs; four support stations; guide/block
locations; spindle tool-tip and workpiece contact datums; XYZ joint positions;
and load-case definitions. The workpiece contact datum must move with the
table/stock, while the tool-tip datum moves with Z. Include the selected
material-property values and their source/assumed status. Reject missing
parts, non-finite poses, zero-volume solids, and unresolved material IDs.

Check that the exported solids are valid and that casting/insert volumes do
not overlap. Compare exported total volume, mass, bounding boxes, and key
datums with CAD and `mill-frame-concept.json`. Define contact faces by stable
geometric features or datum regions rather than transient mesh node numbers.

**Gate:** re-exporting the same revision gives the same manifest and geometry
hashes; changing an XYZ pose moves the appropriate tool/workpiece datum and
preserves the fixed frame; the assembled STEP view matches the concept scene.

### 3. Establish material and joint input data

Keep the mineral-cast modulus and density already in `MaterialLibrary` as
explicit *assumed* baseline inputs. Add a Poisson ratio only in a study
configuration until a material specification exists; record its source and
range. Give steel parts suitable documented elastic and density inputs rather
than deriving all steel grades from appearance labels. Reject missing required
properties. Run sensitivity cases for the uncertain mineral-cast modulus,
Poisson ratio, guide stiffness, column joint stiffness, and foot support.

Build two clearly labeled interface models:

1. **Upper-bound stiffness model:** bonded casting-to-insert interfaces,
   bonded column pad pairs, and idealized guide connections. This verifies
   geometry/load transfer but tends to overstate machine stiffness.
2. **Assembly model:** casting-to-insert connection according to the selected
   process assumption; four discrete column clamp/contact stations with
   justified preload or equivalent stiffness; guide/block stiffness in the
   relevant directions; and realistic foot support. Model bolts as connectors
   or preload elements when source data allow. A `fixed` CAD mate alone is not
   evidence that the physical joint is rigid.

Avoid point loads and point constraints at mesh nodes. Distribute loads across
the actual mount/contact region or a remote reference point coupled to it.
Use a support model that removes rigid-body modes without locking every foot
rotation or inventing a fully fixed floor. Report how foot restraint and floor
stiffness change the result. Do not claim insert anchorage safety from a bonded
linear model; that needs interface shear/tension and embedment data.

**Gate:** all parts participate in a continuous load path from tool/workpiece
to supports; contacts do not penetrate or disconnect; load and reaction sums
balance; replacing assembly joints with the idealized bonded variant changes
the result in the expected direction.

### 4. Run baseline static cases

Use a compact, named matrix: center and eight XYZ travel corners, plus any
intermediate position found to be worse during a coarse sweep. At each pose,
apply three separate ±500 N force directions and the load combinations needed
to find the maximum tool-to-workpiece displacement. Apply equal and opposite
cutting forces at tool and workpiece, with gravity, moving-part masses, and the
20 kg payload represented at their actual locations. Keep ±1 m/s² inertia
cases separate from the pure cutting-stiffness cases so their effects remain
traceable. Ensure the force direction and sign are in a declared world frame.

For every case, calculate the change in displacement between tool tip and
workpiece contact point relative to the gravity-only state at the same pose.
Report each directional compliance in µm/N, the worst displacement in µm for
the stated force, and contributions from base/column bending, head/table
flexure, guides, column joint, and supports where the model permits
decomposition. Also report support reactions, pad/interface tractions, and
stresses at mesh-converged locations. Flag contact-edge and constraint
singularities instead of using
their peak stress as a design limit.

**Gate:** linear scaling check at half and full force, force/moment balance,
expected sign symmetry for the linear model, and mesh refinement until the
primary relative displacement changes by no more than a stated tolerance
(target 5%). Refine locally around column feet, insert recesses, guide seats,
rear chambers, and motor bay; track element quality and solver warnings.

### 5. Run modal and uncertainty studies

With the same masses and boundary conditions, solve undamped eigenmodes for
the no-cutting-load machine at representative poses. Report the first several
non-rigid frequencies and shapes, effective/modal participation where
available, and whether a mode bends the relative tool-to-workpiece path.
Repeat with at least the soft and stiff plausible joint/support settings;
the frequency range is more useful than a single value based on unknown
preload. Compare relevant modes with known spindle, tooth-pass, motor, and
axis excitation bands once their operating ranges are established. Do not
use the assumed damping factor to claim forced-response amplitudes.

When a physical prototype exists, measure static deflection under known loads
at the spindle and table, record support conditions, and measure modal response
at the head, column, and base. Use these measurements to calibrate material
and joint inputs. Preserve the measured data and calibration changes as
versioned inputs with provenance.

**Gate:** no unexplained rigid-body modes; frequencies are stable with mesh
refinement; predicted static deflections and modes are compared with measured
values before setting release limits.

### 6. Make geometry changes as controlled candidates

Start with parameters already owned by `MillFrameLayout`: `wall`, `roof`,
`baseHeight`, `columnWidth`, `columnDepth`, `columnFootWidth`, `seatWidth`, and
`insertThickness`. Expose rear-chamber/front-web and underside-core dimensions
as explicit parameters only after the baseline identifies them as important.
Change one family at a time, then test a small combined candidate set. Do not
let the FEM script modify generated STEP directly; rebuild each candidate
through the CAD recipes.

For every candidate, run `MillFrameChecks`, `MillXYStageIntegrity`,
`MillZAxisIntegrity`, `MillSpindleIntegrity`, assembly clearance/service checks,
and the same FEM case matrix. Reject invalid solids, lost insert support,
blocked motor/bolt access, reduced travel, or a worse worst-case deflection.
Rank surviving candidates by worst relative displacement, relevant modal
frequency range, casting mass, and added fabrication complexity. Show a
Pareto table; do not pick a single score until the design team sets stiffness,
mass, cost, and frequency targets. Re-run the finalists with the refined mesh
and soft/stiff material and joint assumptions.

**Gate:** the chosen change improves the target result beyond mesh error and
uncertainty spread, preserves the existing geometric checks, and has a clear
mass and manufacturing tradeoff. If rankings reverse under plausible joint
properties, obtain better joint data before selecting geometry.

## Deliverables and review evidence

The first useful milestone is the assembly-aware export, one static baseline
case, and an automatically generated report with relative displacement,
reactions, and a mesh convergence table. The second milestone adds the full
position/load matrix, modal study, and uncertainty ranges. The third adds
geometry candidates and a comparison report. Each report must point to the
source revision, manifest/hash, solver version, material data, boundary
conditions, mesh settings, warnings, and exact case IDs. Keep a small
analytical solver fixture and exporter contract tests in version control;
large solver artifacts stay generated under `build/fem/`.

Before manufacturing release, obtain measured mineral-cast properties,
selected guide and foot stiffness/rating data, column-clamp preload and
anchorage evidence, and physical deflection/modal measurements. The FEM
study can then be reconciled with those data and with the separate thermal,
fatigue, process, and controls requirements already listed in `README.md`.
