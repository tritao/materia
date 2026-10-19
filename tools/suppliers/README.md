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
