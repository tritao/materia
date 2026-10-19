# FEM implementation plan for the mineral-cast bench mill

## Purpose and boundary

Build a repeatable structural study of the `MillFramePreview` mechanical concept.
The primary result is elastic displacement of the tool tip **relative to the
workpiece contact point** under cutting loads across the 250/160/250 mm XYZ
travel. Report mass, support reactions, local interface loads, and mode shapes
alongside that displacement. Use the baseline to compare changes to the base,
column, and column foot. This is an engineering study, not manufacturing
qualification or a prediction of machining accuracy by itself.

The castings are one element in series with guides, screw/nut/bearing drives,
the X belt return, the column clamp, spindle bearings, tool holder, fixture and
supports. The study first establishes **how much of the tool-to-workpiece
compliance the castings carry**. Casting geometry work is justified only if
that share is significant (see step 4).

Start with a linear static model and an undamped modal model. Add detailed
contact, thermal distortion, forced vibration, fatigue, and insert anchorage
strength only after the baseline is reproducible and the necessary input data
exist. Keep the working legacy `BenchMill` and its motion/control model separate
from this mechanical concept.

Explicitly outside the first milestones: workpiece and fixture compliance (the
first models use a rigid fixture up to the tool-tip height), spindle/tool
compliance inside the FE model (they enter through the step 4 budget), and
thermal mismatch between mineral cast (assumed 14e-6/K) and steel inserts
(about 11–12e-6/K). For accuracy that mismatch may matter more than static
stiffness. It needs its own study and is not covered by these results.

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
  2400 kg/m³ and Young's modulus of 35 GPa (`physical.youngsModulusPa`). It also
  has assumed damping and thermal values. Steel grades carry no
  `physical.youngsModulusPa`. Sheet steels carry `mechanical.youngs` in MPa
  instead, so the library has two modulus fields in different units. No Poisson
  ratio, casting strength, steel-to-casting bond properties, or measured joint
  stiffness is supplied for the FEM study.
- The present guide calculation uses a 20 kg payload 50 mm above the table,
  ±500 N per world axis, ±1 m/s² acceleration, and static guide safety factor
  two. These are study envelopes, not a validated cutting-force history.
- Supplier catalogues publish stiffness for the HGR15 guide blocks (per preload
  class), screw support bearings and ball nuts, and for spindle bearings. These
  are usable baseline inputs now. Joint and foot stiffness still need data or
  bracketing.

## Implementation status

Progress is tracked in `fem/README.md`. This file is the plan and its gates.

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

Provenance must cover everything that changes the geometry. `git diff HEAD --
haxeon` records only the submodule pointer, so also hash each relevant
submodule's own working-tree diff. Record the haxeon and CadKit/OCCT build
identities, because they affect STEP output. Exploratory runs may use a dirty
tree. A run cited in a report must come from a clean tree, or record those
submodule diffs.

**Gate:** a single-element tension fixture and a meshed cantilever reproduce
analytical displacement and reaction within 1% and 5%, respectively, after
appropriate refinement. A Gmsh-meshed C3D10 fixture passes through the same
node reordering and deck writer used for the frame, and reproduces its
analytical result. Any coupling/equation helper used by the frame deck is
exercised by a fixture with a known answer. A wrong-unit fixture fails
validation.

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
not overlap. Automatically compare each exported part's volume, mass, centre
of mass and bounding box, and the key datums, with the same records in
`mill-frame-concept.json`. Define contact faces by stable geometric features
or datum regions rather than transient mesh node numbers.

**Gate:** re-exporting the same revision gives the same manifest and geometry
hashes; changing an XYZ pose moves the appropriate tool/workpiece datum and
preserves the fixed frame; every exported part matches its
`mill-frame-concept.json` record within a stated tolerance.

### 3. Establish material and joint input data

Keep the mineral-cast modulus and density already in `MaterialLibrary` as
explicit *assumed* baseline inputs. Add a Poisson ratio only in a study
configuration until a material specification exists; record its source and
range. Give steel parts suitable documented elastic and density inputs rather
than deriving all steel grades from appearance labels. Key study material
inputs by the parts' `MaterialLibrary` IDs. Use no generic "steel" bucket.
Record any study override of a library value with its source, and check that
the density used reproduces each part's packet mass. Reject missing required
properties. Run sensitivity cases for the uncertain mineral-cast modulus,
Poisson ratio, guide stiffness, column joint stiffness, and foot support.

Build two clearly labeled interface models:

1. **Upper-bound stiffness model:** bonded casting-to-insert interfaces,
   bonded column pad pairs, and idealized guide connections. Moving axis
   assemblies may be rigid. This verifies geometry/load transfer but tends to
   overstate machine stiffness. Kinematic (rigid) coupling of guide/drive
   patches is acceptable here only because the model is labelled an upper
   bound. It stiffens the coupled rail/seat region.
2. **Assembly model:** casting-to-insert connection according to the selected
   process assumption; four discrete column clamp stations with linearized
   equivalent stiffness derived from justified preload; guide/block stiffness
   in the relevant directions from catalogue data; screw/nut/bearing axial
   stiffness for the drives; and realistic foot support. Connect each spring to
   its frame patch with a distributing (RBE3-type) coupling, not a rigid one.
   A `fixed` CAD mate alone is not evidence that the physical joint is rigid.

Both models stay **linear**, so the step 5 load superposition and gates apply.
Contact with bolt pre-tension is a separate nonlinear check. Run it at the
worst combined load from step 5 to confirm that the clamp pads and feet do not
open or slip, and to calibrate the linearized clamp stiffness. Do not sweep
poses or candidates with the contact model.

Avoid point loads and point constraints at mesh nodes. Distribute loads across
the actual mount/contact region with a distributing coupling to a reference
point. Use kinematic coupling only where a physically rigid body exists. Use a
support model that removes rigid-body modes without locking every foot
rotation or inventing a fully fixed floor. Four rigid vertical supports are
statically indeterminate and stiffen the base in torsion. Bracket the floor
condition with:

- a **soft bound:** three-point kinematic support (one foot fixed in XYZ, one
  in two directions, one vertical, the fourth free or on a soft spring), with
  foot rotations free; and
- a **stiff bound:** four feet on vertical springs representing a leveled foot
  on a stiff floor.

State the leveling assumption (feet leveled under self-weight) and report how
much the result changes between the two bounds. Do not claim insert anchorage
safety from a bonded linear model; that needs interface shear/tension and
embedment data.

**Gate:** all parts participate in a continuous load path from tool/workpiece
to supports; contacts do not penetrate or disconnect; load and reaction sums
balance; replacing assembly joints with the idealized bonded variant changes
the result in the expected direction.

### 4. Build a tool-to-workpiece compliance budget

Before casting geometry is studied, build a lumped series/parallel spring
model of the whole loop for each world direction. Include guide blocks,
screw/nut/bearing axial stiffness, the X belt return (reflected through the
screw lead into axial stiffness), column clamp, spindle bearings, tool holder
at the study stick-out, and the bracketed support. Use catalogue values with
their source and preload class. Put the frame compliance from the step 3
upper-bound model in series with these springs.

Report each element's share of the total compliance in each direction. Use the
budget to set a provisional frame target, e.g. "frame contributes at most N% of
loop compliance", so that step 7 has a stopping criterion. Record the target
and its rationale in the study configuration.

**Gate:** the budget states each input's source and is repeated with the soft
and stiff bounds. If the castings carry a small share in every direction under
both bounds, step 7 is deferred, and joint/guide/drive work takes priority.

### 5. Run baseline static cases

Use a compact, named matrix: center and eight XYZ travel corners, plus any
intermediate position found to be worse during a coarse sweep. At each pose,
apply equal and opposite unit force pairs at the tool and workpiece datums in
world X, Y and Z, with loads and reactions in a declared world frame. The three
relative displacements form the 3×3 tool-to-workpiece compliance matrix **C**
at that pose. For the ±500 N study envelope, the worst displacement is the
largest |**C** f| over the eight corners of the load box. Report it, along with
the principal compliances (eigenvalues of the symmetric part of **C**) and
their directions in µm/N. No extra load-combination solves are needed in the
linear models.

Gravity, moving-part masses, and the 20 kg payload are applied at their actual
locations in a gravity-only case at each pose. That case gives support
reactions and self-weight stresses. In the linear models it does not change
**C**. Subtracting the gravity-only state is required only for the nonlinear
contact check. ±1 m/s² inertia loads on the moving masses are tens of newtons;
keep them as a separate, low-priority case.

Decompose each directional compliance by strain energy. For a single load
pair, the strain energy in each element group divided by the total gives that
group's exact share of the compliance in that direction. Report the base
casting, column casting, inserts, clamp pads and each spring set. Cross-check
one decomposition by stiffening a group and comparing the change. Also report
support reactions, pad/interface tractions, and stresses at mesh-converged
locations. Flag contact-edge and constraint singularities instead of using
their peak stress as a design limit.

**Gate:** in the linear models, a linear scaling check at half and full force,
force/moment balance, sign symmetry, and symmetry of **C** within solver
tolerance; and mesh refinement until the primary relative displacement
changes by no more than a stated tolerance (target 5%). Every refinement level
has at least two quadratic elements through the thinnest wall, rib and roof;
coarser levels are not counted toward convergence. Refine locally around
column feet, insert recesses, guide seats, rear chambers, and motor bay; track
element quality and solver warnings.

### 6. Run modal and uncertainty studies

With the same masses and boundary conditions, solve undamped eigenmodes for
the no-cutting-load machine at representative poses. Represent mass-only
components as point masses with their inertia tensors at their centres of
mass. Attach each one to its represented ancestor through a distributing
coupling, and export the inertia tensors in the packet. Rigid moving
assemblies need their mass and inertia on their reference nodes. Treating the
axes as rigid raises mode frequencies, so state this in every modal report
from the upper-bound model.

Report the first several non-rigid frequencies and shapes, effective/modal
participation where available, and whether a mode bends the relative
tool-to-workpiece path. Repeat with at least the soft and stiff plausible
joint/support settings; the frequency range is more useful than a single value
based on unknown preload. Compare relevant modes with known spindle,
tooth-pass, motor, and axis excitation bands once their operating ranges are
established. Do not use the assumed damping factor to claim forced-response
amplitudes.

When a physical prototype exists, measure static deflection under known loads
at the spindle and table, record support conditions, and measure modal response
at the head, column, and base. Use these measurements to calibrate material
and joint inputs. Preserve the measured data and calibration changes as
versioned inputs with provenance.

**Gate:** no unexplained rigid-body modes; frequencies are stable with mesh
refinement; predicted static deflections and modes are compared with measured
values before setting release limits.

### 7. Make geometry changes as controlled candidates

Proceed only when the step 4 budget shows the castings carry a significant
share of loop compliance, measured against the recorded frame target.

Start with parameters already owned by `MillFrameLayout`: `wall`, `roof`,
`baseHeight`, `columnWidth`, `columnDepth`, `columnFootWidth`, `seatWidth`, and
`insertThickness`. Expose rear-chamber/front-web and underside-core dimensions
as explicit parameters only after the baseline's strain-energy decomposition
identifies them as important. Change one family at a time, then test a small
combined candidate set. Do not let the FEM script modify generated STEP
directly; rebuild each candidate through the CAD recipes. Screen candidates at
the baseline's converged mesh settings. Repeat full convergence only for the
finalists.

For every candidate, run `MillFrameChecks`, `MillXYStageIntegrity`,
`MillZAxisIntegrity`, `MillSpindleIntegrity`, assembly clearance/service checks,
and the same FEM case matrix. Reject invalid solids, lost insert support,
blocked motor/bolt access, reduced travel, or a worse worst-case deflection.
Rank surviving candidates by worst relative displacement (frame-only and with
the step 4 budget), relevant modal frequency range, casting mass, and added
fabrication complexity. Show a Pareto table; do not pick a single score until
the design team sets stiffness, mass, cost, and frequency targets. Re-run the
finalists with the refined mesh and soft/stiff material and joint assumptions.

**Gate:** the chosen change improves the target result beyond mesh error and
uncertainty spread, preserves the existing geometric checks, and has a clear
mass and manufacturing tradeoff. If rankings reverse under plausible joint
properties, obtain better joint data before selecting geometry.

## Deliverables and review evidence

The first useful milestone is the assembly-aware export, the upper-bound
static baseline at one pose (compliance matrix, strain-energy decomposition,
reactions, and a mesh convergence table), and the step 4 compliance budget,
all in an automatically generated report. The second milestone adds the
assembly model, the full pose matrix, both support bounds, the contact check,
the modal study, and uncertainty ranges. The third adds geometry candidates
and a comparison report, if the budget justifies them. Each report must point
to the source revision, manifest/hash, solver version, material data, boundary
conditions, mesh settings, warnings, and exact case IDs. Keep small analytical
solver fixtures and exporter contract tests in version control. Large solver
artifacts stay generated under `build/fem/`.

Before manufacturing release, obtain measured mineral-cast properties,
selected guide and foot stiffness/rating data, column-clamp preload and
anchorage evidence, and physical deflection/modal measurements. The FEM
study can then be reconciled with those data and with the separate thermal,
fatigue, process, and controls requirements already listed in `README.md`.
