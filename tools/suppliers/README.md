# Supplier imports

Vallder's public WooCommerce Store API provides products and variants without
credentials. The importer uses read-only requests and Python's standard library:

```sh
python3 tools/suppliers/vallder.py
python3 tools/suppliers/vallder.py --refresh
python3 tools/suppliers/vallder.py --offline
python3 -m unittest discover -s tools/suppliers/tests
```

Output defaults to `build/suppliers/vallder/` (ignored by Git). Use `--output DIR`
to choose another directory. `catalog.json` contains supplier identity, products,
variants, source URLs and fetch timestamps. `raw/` preserves the original API
and sitemap response bodies with their headers. Normal runs reuse cached pages;
`--refresh` fetches them again, and `--offline` requires a complete cache.
Keep each snapshot's cache together; after an interrupted refresh, complete it
with another `--refresh` before using offline mode.

The importer requests up to 100 entries per page, waits at least 0.5 seconds
between requests and retries transient failures. It checks pagination totals,
unique IDs, parent references and variant coverage before replacing the previous
catalogue atomically. Sitemap differences are reported in `coverage`; they do not
silently filter API listings.

Prices are decimal strings converted using the API's currency minor unit. Parent
prices can represent a range; use variant records for a selected SKU. Prices and
availability are snapshots, and tax basis is explicitly unspecified. Stock flags
do not promise inventory quantities.

Supplier listings are separate from MachineKit engineering components and BOMs.
Every record starts with `engineering_verification: "unverified"` and a null
`component_id`. Variant option selections come from the parent's variation index
because individual variant responses may omit structured attributes.

Descriptions remain available as HTML. `specification_tables` preserves text rows
without inferring units, column meanings or compatibility; row/column spans are
not expanded. `description_links` includes drawing, document and image references
without downloading them. Structured dimensions and weight are retained exactly
as reported, without assuming their units or completeness. Raw responses retain
all fields for future supplier-specific extraction.

## Coverage, generator recipes and purchased CAD

After importing, run:

```sh
python3 tools/suppliers/audit.py
python3 tools/suppliers/cad_library.py
./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.cad-check.json
python3 tools/suppliers/validate_cad.py
python3 tools/suppliers/profile_sections.py
python3 tools/suppliers/coverage.py
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.json
python3 tools/suppliers/coverage.py
```

The audit creates `audit.json` and `audit.csv` beside the catalogue, covering
every parent product with variant counts, specification availability, candidate
existing MachineKit families and CAD/document links. Category matches are a
triage aid, not a compatibility decision. ZIP links are listed as candidates;
the audit does not download archives or claim they contain usable STEP files.
Products with no candidate remain visible in the report for new-family,
manufacturer-CAD or dimensional-evidence work. Packaging, machining services
and the test listing are identified separately.

`fasteners.json` joins the Vallder A2-70 and class 8.8 socket-head screw families' exact SKUs,
dimensional row and variation option into MachineKit recipe inputs. Changes
to the expected table layout or inconsistent identities stop the mapping.
`recipes.json` binds these SKUs, HGR15 rail and seven 30/40-series slide-in nuts
to MachineKit recipes.
`SupplierRecipeLibrary.create("vallder", sku, requestedInputs)` instantiates
them through a supplied `ComponentRegistry`; it contains no geometry code.
Only explicit required inputs can be supplied by the caller; supplier-bound
size and property class cannot be overridden. The rail requires a cut length.

### Slot hardware evidence and fit checks

`coverage.py` also writes `hardware.json`. Slide-in nut variants join exact
option identities to the dimension table and drawing at
[Vallder's T-slot nut page](https://vallder.com/product/t-slot-nut/).
`machinekit.structural.slide-in-slot-nut` is a supplier-independent, saveable
recipe: width, length, total height, ridge width/height and metric thread.
The model retains sharp corners where chamfers are unspecified and attaches
semantic threads. Its local ridge top is Z=0, shoulders are Z=-ridgeHeight,
and X runs across the slot. No STEP download is required.

Run the focused native check after generating coverage:

```sh
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.slot-hardware.json
```

It validates all seven threaded models and recipe/BOM identities, then checks
the unperforated nut envelope against every slot in the four exact drawing
sections. `hardware-fit.json` records the section hash, face, slot offset,
ridge-top depth and intersection volume. It searches depths 0–5 mm in 0.25 mm
increments. `static_clearance_candidate` means only that this envelope has a
clear static position. `envelope_interference` can require omitted chamfers or
indicate a mismatch; it does not by itself prove the real nut incompatible.
Neither result establishes seated bearing contact, insertion motion, dimensional
tolerances, thread engagement, tightening torque or joint strength. All offers
remain unverified, with optional sourcing separate from engineering BOMs.
`slot-nuts.assembly.json` saves the seven engineering definitions without supplier
metadata. `slot-nuts.bom.json` lists their optional Vallder offers, all unselected.
Add `-- --recipes-only` to validate models, assembly persistence and sourcing
without repeating the fit sweep. Regenerate the fit report after changing nut
dimensions or source sections.

The initial sweep found envelope interference in all 22 slots. Best sampled
ridge-top depths were 0.5 mm for the 30-series nut and 1.5 mm for the 40-series
nut. The supplier illustration shows corner relief without dimensions, which
the conservative generator does not remove. Obtain those relief dimensions
and tolerances before approving a supplier fit; these sampled depths are not
qualified mounting positions.

The same report preserves evidence gaps for 20/45-series slide-in nuts, hammer
nuts, spring nuts and brackets. The bracket interface records retain known slot
width/travel and the 8080 bracket's 40 mm hole pitch. Matching that pitch is only
a candidate for the shared 40 mm grid. Missing dimensions prevent full bracket
models and screw/tool-access qualification; existing table hardware assumptions
are not substituted for those supplier parts.

### Motor mounts

`machinekit.assembly.nema-mount-plate` makes the existing `NemaMountPlate`
saveable in assemblies and editable documents. It retains the motor pilot and
bolt-pattern cutout, bolt connectors and existing screw-engagement checks.
Choose a named motor model, plate thickness and margin. Custom motor interfaces
remain code-only, so they cannot reload with silently changed dimensions.

An optional profile-side pattern adds four through holes, two on each of two
parallel rails running along local Y on the plate's +Z face. `profileSystem`
is `TSLOT-30-SLOT8` or `TSLOT-40-SLOT8`; `railSpacing` must be a positive grid
multiple. `alongPitch` sets hole spacing along each rail, and
`profileHoleDiameter` is the drilled clearance diameter. Increase `margin` to
contain the holes. Validation reserves 2 mm hole/edge ligaments and rejects
motor-hole overlaps and a nominal rail gap narrower than the pilot passage.
`profileBolt1` through `profileBolt4` provide the mounting frames. These checks
do not qualify supplier slots, nuts, screw engagement, driver access or loads.
For a plain plate use `profileSystem=None` and zero profile-hole dimensions.

```haxe
var plate = new machinekit.assembly.NemaMountPlate(
    machinekit.motion.NemaStepper.frame(23), 8, 30,
    {systemId:"TSLOT-40-SLOT8", railSpacing:80, alongPitch:40, holeDiameter:6.6});
```

`coverage.py` writes `motor-mounts.json` and resolves all four RBK CAD variants
by exact model, SKU and download columns in Vallder's transposed table. The
validated asset hashes and paths are available through `cad/library.json` and
the motor-mount bindings. The four reviewed revisions also bind to the saveable
`machinekit.assembly.rbk-motor-mount` recipe described below. Cube and
other motor holders stay separate from custom square plates. DB57's listed
38 mm opening is smaller than our 38.1 mm NEMA 23 pilot; DB86 has zero nominal
pilot clearance. Both need tolerance review. Generic plates receive no inferred
supplier SKU or offer.

```sh
python3 tools/suppliers/coverage.py
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.motor-mounts.json -- --supplier-cad
```

The native check covers NEMA 17/23/34 geometry, both profile grids, assembly and
editable document round trips, invalid hole layouts and the existing four-screw
NEMA 17 assembly. It saves `motor-mounts.assembly.json` and `motor-mounts.bom.json`.
The optional `--supplier-cad` loads the four pinned RBK recipes and writes
`supplier-motor-mounts.bom.json` with unselected Vallder offers and
`supplier-motor-mounts.assembly.json` with their engineering definitions.

### RBK mounting interfaces

`RbkMotorMount("RBK12-57")` also supports `RBK15-57`, `RBK15-86` and `RBK17-86`.
Each revision pins a specific STEP hash and reviewed interface metadata from
`rbk-interfaces.json`; regenerate the runtime catalogue explicitly with
`python3 tools/suppliers/rbk_interfaces.py --write-haxe`. Running that script
without the flag checks that source metadata and runtime data agree. A changed
supplier asset does not inherit the old mounting frames or recipe binding.
Published revisions are immutable; retain the old entry and introduce a new
revision when changing its asset or interface datums.

The shared local frame is **X across, Y up, Z from motor to bearing**. The motor
mounting face is Z=0, the base bearing plane is Y=0. Named connectors are:

| Connector | Datum and joint-axis direction |
| --- | --- |
| `base` | Origin on the base plane beneath the motor centre; +Y up |
| `motor` | Motor face centre at the shaft height; +Z into the mount |
| `shaftAxis` | Coaxial reference at the motor face; +Z, without a shaft seating offset |
| `motorBolt1`–`motorBolt4` | Measured CAD thread-hole axes on the motor face; +Z |
| `baseBolt1`–`baseBolt6` | Measured CAD hole centres on Y=0; +Y (four holes on RBK17-86) |
| `baseAux1`–`baseAux2` | RBK17-86 auxiliary holes; +Y |

```haxe
var support = new machinekit.assembly.RbkMotorMount("RBK15-57");
machine.addComponent("support", support);
machine.addComponent("motor", machinekit.motion.NemaStepper.frame(23));
machine.addMate("motor-to-support", "fixed", "support", "motor", "motor", "mountFace");
```

Alternatively instantiate the exact SKU through
`SupplierRecipeLibrary.create("vallder", "1173002")` and the default registry.
Geometry loads lazily, verifies the asset hash and applies the pinned coordinate
transform. Saved assemblies/documents store model and revision, with no absolute
cache path. The deployment must retain the STEP files under
`cad/assets/<sha256>.step`. Configure the supplier directory with
`RbkMotorMount.setSupplierDirectory(path)`, `MATERIA_VALLDER_ROOT`, or
`MATERIA_ROOT` (defaulting to the workspace's `build/suppliers/vallder`).

`interfaceMetadata()` returns a defensive copy including pilot diameter/recess,
motor thread size/depth where known, base-hole/counterbore dimensions, source
drawing/hash, shaft height and limitations. The motor flanges protrude below
the bearing plane by **5, 2, 15 and 4 mm**, respectively. Base plates must end
behind the flange or provide a relief. The 57 variants use M4 mounting threads;
the 86 variants use M6. Do not infer the bracket thread from the motor's default
mounting screw. Unknown thread depths remain null; listed depths are drawing
values, not measured effective engagement. Counterbore dimensions follow the
CAD. Screw lengths, driver access, tolerances, bearing preload, shaft axial seating, materials/density
and load ratings still require engineering qualification. The imported compound
contains the supplier's bearing/hardware geometry and is treated as rigid.
Automatic threaded fastenings are not inferred from STEP faces.

The pinned CAD and supplier drawings disagree in some base details: the first
hole pair on both RBK15 models uses 58 mm CAD pitch versus 54 mm in the drawings;
RBK17-86 auxiliary holes use 70 mm CAD pitch versus 54 mm drawn. RBK12-57 has
four modeled counterbores rather than the six indicated. Connectors follow the
measured CAD; metadata retains the drawing values and discrepancies. Confirm
the physical part before manufacturing mating plates from either source.

```sh
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.rbk-mounts.json
```

The focused check compares connector planes and passage probes with the pinned
solids, tests motor/base clearance, then exercises named mating and saved
assemblies. It writes `rbk-interface-checks.json` and
`rbk-mounted-motors.assembly.json`. Source compatibility remains unverified;
these checks establish nominal CAD interfaces, not a complete machine rating.
Add `-- --persistence-only` to check bearing-axis alignment, named mating and
editable document/assembly round trips without repeating the full clearance sweep.
The example creates two assembly occurrences of each screw configuration,
exercises the rail at 300 mm and writes:

- `geometry.json`: generator recipe inputs, supplier evidence and dimensional comparisons;
- `fasteners.assembly.json`: reusable MachineKit assembly description;
- `engineering-bom.json`: supplier-independent part requirements and quantities;
- `sourcing-bom.json`: optional offer selections, stock flags and minor-unit cost estimates.

The default path builds geometry directly from generators and does not read or
write generated STEP files. Add `-- --export-step` to the example command for an
optional CAD export with round-trip checks. Legacy exports in `step/` are not
runtime dependencies. Threads are semantic rather than helical geometry.

`profiles.json` contains five supplier-authored extrusion sections read from
R12 DXF archives. `DrawingExtrusionProfile` reuses CadKit's curve, sketch and
extrude operations with exact line, bulge-arc and circle data, preserving the
actual slot shapes instead of guessing a match to MISUMI profiles. Original
XY offsets, elevation and any section rotation are recorded. The example checks
solid validity, section area, external dimensions and volume scaling at 100
and 250 mm, then writes `profile-validation.json`. Cut length is a runtime
parameter. Profile material/alloy, mounting compatibility and per-metre sourcing
remain unqualified. Shared T-slot and drawing-derived `ProfileMember` instances
are registered recipes and can be saved through the assembly or document codec.
`profiles.assembly.json` is a complete local reconstruction example: measured
sections are embedded as geometry data and require no supplier downloads when
reopened.

The 30×30, 40×40, 40×80 and 80×80 sections now belong to shared dimensional
families: `TSLOT-30-SLOT8` and `TSLOT-40-SLOT8`. These are slot-8 candidates,
not claims of universal hardware interchangeability. `TSlotExtrusion.forSystem`
builds a supplier-neutral nominal layout with slots positioned on the 30/40 mm
grid and bores at cell centres. It supports multiple slots per face (six total
for 40×80, eight for 80×80). Existing `forProfile` catalog entries and their table
hardware remain unchanged. The 180×20 section stays in a separate slotted-plate
family; the custom CNC axis sections are not reclassified as generic T-slot.

```haxe
var nominal = TSlotExtrusion.forSystem("TSLOT-40-SLOT8", 40, 80);
var detailed = TSlotExtrusion.forSystem("TSLOT-40-SLOT8", 40, 80,
    new DrawingExtrusionProfile(section));
var stock = new ProfileMember(detailed, 500, "unspecified");
```

The nominal model uses approximate slot/bore dimensions for layout. Supplying
a measured section variant preserves its actual curves, cavities and wall
thicknesses. Variant bounds must agree with the family. Hardware depth cannot be
inferred from a drawing variant's nominal family dimensions. Geometry/BOM
identity includes the actual section variant but excludes supplier names, SKUs,
prices and source URLs. Different wall/cavity variants remain distinct parts.

The built-in registry includes `machinekit.structural.tslot-member` (system,
width, height, length, optional embedded section and material) and
`machinekit.structural.drawing-profile-member` (section, length and material).
Both use the normal assembly/document save and reload paths. Section payloads
contain only validated dimensions, loops, circles and area; supplier metadata
and cache paths are omitted. Geometry/BOM keys use a compact section digest
rather than the full JSON payload. Non-default materials get distinct BOM
part numbers. Invalid payloads or section/family dimension mismatches are
rejected. Legacy/custom profiles outside these supported recipes retain their
previous code-only behavior.

Run the portable-profile checks without a supplier cache:

```sh
./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.profile-recipes.json
```

They cover assembly and document reload, editable cut length, material and section
identity, compact BOM keys and rejection of invalid or unsupported configurations.

`profile-families.json` separates supplier-neutral family definitions from
Vallder offers targeting specific measured section variants. The example writes
`profile-sourcing-bom.json` with optional offers and no selected supplier.
Per-metre pricing stays in the offer data; a cut-piece quote is required before
assigning a per-piece BOM price. `profile-validation.json` records which shared
family and section variant was exercised by each native geometry check.

`cad_library.py` downloads the linked archives, saves URL/timestamp/hash metadata,
lists every member and stores only STEP members as content-addressed assets.
It never executes downloads or extracts archive paths. Limits are 64 MiB per
download, 320 MiB per member and 512 MiB expanded per archive. Use `--offline`
for cached replay, or `--refresh` to fetch again. No-STEP archives may contain
DXF, IGES, Rhino files, manuals or software rather than usable solid models.

`validate_cad.py` imports each asset in an isolated process with a 90-second
timeout. It records validity, solid/shell counts, volume, bounds in mm and
analytic face evidence for smaller files. Validation is cached against asset
and worker hashes. A different `--timeout` retries timed-out assets; timed-out
or invalid geometry is excluded from the usable library. All assets are
hash-checked again even when reusing validation results. Declared source units
are recorded separately from the importer's mm output.

`cad/library.json` links valid geometry to products and, where a table contains
an exact SKU and CAD link together, to variants. Other assets remain product
references. `SupplierCadComponent` loads a local hash-pinned solid when no
suitable generator is available; it downloads nothing and invents no mounting
connectors. Close it after use; its returned geometry has independent ownership.
Imported references are code-only; retain their asset hash/path and explicit
engineering part definition separately from saved recipe assemblies.

`coverage.json` and `coverage.csv` cover every product and variant, reporting
usable generators, validated CAD, ambiguous identities and remaining gaps.
`support-interface-review.json` checks BK/BF base planes and dimensions against
table evidence. Matching a base plane does not qualify mounting bores or shaft
fits. Geometry validity likewise does not establish assembly compatibility,
mass, material, load ratings or supplier certification.

`fastener-dimension-review.json` compares reported screw head dimensions with
primary manufacturer reference ranges recorded in `fastener-reference.json`.
These external URLs are dimensional evidence, not additional supplier offers.
M4/M5/M6 head values fall within those reference ranges; the listed M8 diameter
does not. Our nominal generator is unchanged, and all initial offer selections
remain `review_required`. This is a head-dimension comparison, not a complete
standard or supplier qualification. Sources and prices retain
their original page-fetch timestamps. The example uses fixed demonstration
quantities; it does not prescribe an actual machine purchase.

`machinekit.component.SourcingPlan` is reusable for other BOMs and suppliers:
add `SupplierOffer` records keyed to an existing BOM `partNumber`, optionally
select an offer, then export `plan.lines(bom)`. Without a selection, the line
remains `unselected`. Multiple offers do not duplicate engineering BOM lines.
An unverified selection remains `review_required`, and a verified out-of-stock
offer is `unavailable`. Clear the selection to return to an unsourced requirement.
Compatibility verification is an explicit caller-supplied fact; this module
does not infer it from product names or price. Estimates retain their currency
and tax basis per line and are not summed across currencies. Persist selections
separately from the assembly; the assembly codec and BOM schema are unchanged.

The example also runs sourcing checks for optional selection, multiple suppliers,
SKU-to-part mismatches, stock status, integer price calculations, defensive
copies and unchanged engineering BOM identity. Python tests cover the catalogue
join, table extraction, pagination, caching and failed-output preservation.

The drawing-based SFU16 bindings in `sfu16.json` use Vallder's standard machined
SFU1605 ends (10 mm coupling, 12 mm fixed journal, 10 mm floating journal),
SKU1035001 nut (46 mm overall, offset flange), and SKU1013002 BF12 (10 mm bore).
These are separate recipes; existing generic BallScrew/BallNut/support recipes
retain their defaults. Changed BF12 CAD hashes do not inherit measured datums.
Supplier-intended compatibility is the working assumption. The RBK CAD's 25.5 mm
bearing/sleeve stack versus the standard 25 mm journal is recorded as a reference
seating discrepancy and does not produce a custom machining requirement.

Run the worked axis and its checks with:

```sh
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run \
  --project machinekit/examples/supplier-fasteners/haxeon.sfu-axis.json -- --export-step
```

This builds a 400 mm screw / 200 mm travel reference axis with RBK12-57, BF12,
NEMA23, coupling, nut bracket, twin HGR15 guides, base, risers and installed
fasteners. It writes `rbk-sfu1605-axis.assembly.json`, `.bom.json`, `.step`, and
`rbk-sfu1605-axis-checks.json` in the supplier output directory. Reload the saved
assembly with `RbkSfuAxis.registry()` (it includes the example's fabricated parts
and coupling). These fabricated parts are an example machine design, not changes
to purchased components. Travel is kinematic only: RBK axial stiffness/preload
has not been specified. The report checks selected part pairs at three travel
positions and nominal fastener engagement; it is not a swept collision, driver
access, load capacity, or tolerance qualification. Motor envelope/access and
assumed tapped depths remain explicit limits.

The worked axis uses `OPENBUILDS-NEMA23-56`, a separate named motor variant.
Its native analytic geometry is adapted from bd_warehouse's StepperMotor at
revision `e6ae6e7bae74cb3a50cbde395930ba24a1051f88` (Apache-2.0; attribution and
license in `machinekit/third-party/bd_warehouse`). It preserves the shaped housing,
0.1 mm inset stack, end caps, 4.8 mm front flange and mounting ears, 5.1 mm
clearance holes, pilot and D shaft. It does not drill cylindrical access channels
through a generic motor body. The shaft tip is 20.6 mm from the mounting face;
the example coupling and fastener seats use that variant's dimensions.

`tools/suppliers/openbuilds_motor_reference.py` documents the isolated environment
and executes the pinned original generator to export an independent STEP and
measurement record. Run `haxeon.openbuilds-motor.json` in the example directory
for the native geometry/recipe regression: its fused solid volume agrees with
the independently generated original within 0.01 mm³. The complete axis additionally
checks installed motor bolts/washers, a 5 mm axial driver approach, and travel.
The source is an OpenBuilds reference, not a claim about StepperOnline or Leadshine
construction. It has no inherited StepperOnline electrical ratings. Existing
motor recipe IDs and named StepperOnline variants retain their own identities.

## HZMotion motor mounts and screw supports

The HZMotion catalog is separate from the Vallder import. Its reviewed reference
is `hzmotion-reference.json`, with source-PDF SHA-256 hashes, page references,
135 coverage-matrix combinations across A/B/C/H/L/KK, and ten modeled parts:

- `M60D-BK12-C`, `M60D-EK12-C`, `M60D-FK12-A`, `M80E-BK15-C`;
- `KK86-FK10L` and the separate `KK86-80EP-adapter`;
- `BF12`, `BF15`, `EF12` and `FK12` supports.

Coverage entries are a discovery index, not claims of current stock, CAD availability
or dimensional interchangeability. Individual model drawings govern geometry.
The reference retains discrepancies: for example, the general flange table's
60D thickness differs from the BK12-C drawing, and its 130G bolt circle differs
from the larger individual drawings. No 130G geometry is generated from that row.

```sh
python3 tools/suppliers/hzmotion.py --write-haxe
python3 tools/suppliers/hzmotion.py --offline --check
python3 -m unittest discover -s tools/suppliers/tests -p test_hzmotion.py
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.hzmotion.json
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.hzmotion-preview.json
```

The importer pins and caches `Motor_Bracket.pdf` and `Ballscrew_support_unit.pdf`
under `build/suppliers/hzmotion/raw`. It rejects changed PDF content rather than
silently updating dimensions. `--write-haxe` explicitly regenerates typed runtime
records; `--check` verifies them without network access. Prices and ordering
identities are not inferred from geometry model names. The recorded supplier STEP
links currently return HTTP 403 with “Login required to download CAD files”.
No failed download is treated as an asset or worked around with a guessed file.

Open `machinekit/examples/supplier-fasteners/materia.hzmotion.project.json` in
Materia. The default catalog entry shows editable model/revision/bearing-grade
recipes. The axis entry shows a separate worked Leadshine Y-drive stack.
The command-line preview also writes `catalog.mtrg` and `y-drive.mtrg` under
`build/suppliers/hzmotion`. Saved preview reconstruction is exercised by the run.

`HzMotionComponent` is the reusable component recipe. `HzMotionCatalog` exposes
fresh typed records and compares its receiving interface with
`ServoMotor.mounting()`. Model, drawing revision and selected bearing grade survive
assembly/document reconstruction. Bearing grade defaults to **unspecified**;
C3/C4/C5 here are bearing configuration options, not a statement of ball-screw
lead accuracy or an automatically qualified procurement SKU.

Geometry is reconstructed from drawings, not manufacturer STEP. Pilot and shaft
interfaces, published mounting patterns, overall dimensions and nominal shaft
height are modeled. Undimensioned webs, recess depths, chamfers, bearing internals,
seals, counterbores without published depths, dowels and adapter attachment details
are simplified or omitted. In particular, the 80EP adapter's pilot depth and
smaller M5 fasteners remain unqualified. Close tool-access or machining decisions
must await exact CAD/controlled drawings. The C/KK unit is one purchased assembly:
its bearings and included locking hardware are not added to the BOM a second time.
The locking hardware is not independently articulated in these rigid models.

Published assembly masses are retained where available; the center of mass is a
geometric estimate and inertia is not declared. Parts without published masses
reject engineering `massProperties()` calls. The preview alone uses explicitly
identified steel-volume estimates for those parts, so it can be opened; those
values must not be propagated into machine sizing.

The first Y-drive study combines `ELM2H-0400LA60F`, `M60D-BK12-C`, a reference
D25/L32 coupling with 14/10 mm bores, a nut and BF12. It uses an explicitly named
`HzMotionDrawingScrew` machining requirement from Motor_Bracket.pdf p10:
51 mm fixed end, 12 mm bearing seat, M12x1 locking thread and a 10x15 mm coupling
seat. **This is not the Vallder SFU1605 with its 54 mm fixed end.** Neither that
supplier model nor the current mill was modified to make it fit. Shaft end gap
is 4 mm and nominal engagement is 14 mm per hub; the coupling is not a selected
supplier SKU. Nominal M5 screw penetration is 5.4 mm, but effective bracket thread
depth is unpublished, so fastening is not manufacturing-qualified.

The focused checks verify all ten solids, pilot/base passages, catalog isolation,
known/unknown mass behavior, modular FK12 seating, motor-frame mismatches, drive
clearances, saved BOM identities and right-hand 5 mm-lead motion at 0/100/200 mm.
They write `checks.json`, `y-drive.assembly.json` and `y-drive.bom.json`. There is
no guide/carriage structure or attachment to the actual mill in this first drive
stack. The reference's machine suitability records preserve the mill's X return
belt and flag Z/hobber/lathe/router arrangements for independent selection.
