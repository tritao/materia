# Vallder motion families

The recipes separate geometry/interface identity from optional Vallder sourcing.
HG rails and carriages are rows of the one `linear-rail` / `linear-rail-block` recipe
pair; their `geometry` input selects the measured section or the supplier CAD.

| Family | Supported geometry | Vallder bindings in the cached catalogue |
| --- | --- | --- |
| HGR running rails | 15, 20, 25, 30, 35, 45; exact per-size analytic sections, arbitrary cut lengths and counterbores | 15, 20, 25, 30 |
| HGH / HGW blocks | 15, 20, 25, 30, 35, 45; pinned detailed reference CAD | 15, 20, 25, 30 for each style |
| BK / BF supports | 10, 12, 15, 17, 20, 25; included bearing/hardware, named mounting axes | All twelve; original BF12 binding retained |
| FK / FF supports | 10, 12, 15, 20 | Eight; size 17 quarantined after table/CAD flange-hole mismatch |
| SFU nuts | 1605, 1610, 2005, 2010, 2505, 2510 | All six; original 1605 binding retained |
| DSG nut housings | 16H, 20H, 25H | All three; 12H lacks sufficient cached dimensions |
| SFU standard-end screws | 2005, 2010, 2505, 2510 | Stock-by-length offers with the published standard end option |

No supplier variants are invented from table-only rows. These HG profiles are an
explicitly checked reference family, not a promise that every supplier's similarly
named part is interchangeable. Rail and carriage masses and load ratings are the
catalogue rows' datasheet values; supports retain their mixed-material CAD with
material locked to unspecified. Rails generate offline; supplier carriage drawings
and supports require their pinned local STEP assets.

Each carriage row lists its published ratings, the first being the one sizing uses:
Vallder's table, which reproduces HIWIN's 2008 catalogue (G99TE13-0809), then HIWIN's
2008 and 2024 (G99TE24-2410) editions where reviewed. The 2024 figures are 30 to 50 %
higher; sizing applies its own static safety factor to the Vallder value.

## Prepare and verify

```sh
export MATERIA_ROOT="$PWD"
python3 tools/suppliers/hg.py --prepare build/suppliers/vallder/hg-reference
python3 tools/suppliers/hg.py --check
python3 tools/suppliers/supports.py --check
python3 tools/suppliers/sfu_family.py --check
python3 tools/suppliers/coverage.py --output build/suppliers/vallder
python3 -m unittest discover -s tools/suppliers/tests
./haxeon/scripts/haxeon run --project machinekit/examples/linear-rail-reference/haxeon.hg-family.json
./haxeon/scripts/haxeon run --project machinekit/examples/linear-rail-reference/haxeon.support-family.json
./haxeon/scripts/haxeon run --project machinekit/examples/linear-rail-reference/haxeon.sfu-family.json
./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.vallder-catalog-axis.json
```

Support STEP assets use the existing Vallder CAD preparation/validation pipeline.
`MATERIA_VALLDER_ROOT` can relocate that library; `MATERIA_HG_REFERENCE` relocates
HG reference CAD. Paths are deployment configuration, never saved recipe inputs.
The extraction command `haxeon.hg-audit.json` writes the 18 reference bounds/hashes
and exact rail sections to `hg-reference/geometry-audit.json` for a deliberate
source review. `hg.py --emit`, `supports.py --emit` and `sfu_family.py --emit`
regenerate typed Haxe tables from the committed evidence manifests.

Outputs under `build/suppliers/vallder` include `hg-reference/family.mtrg`,
`sfu-family.mtrg`, geometry reports, support bore checks and refreshed coverage.
`build/suppliers/configurable-axis/vallder-catalog.mtrg` is a complete 250 mm travel
axis using the shared HG15 rail/block, SFU1605 nut, original end/support stack and
StepperOnline motor. Its saved assembly, engineering BOM, optional Vallder offers,
and three-state installation clearance report are exported alongside it.

## Code

```haxe
var rail = machinekit.motion.LinearRail.metric("HGH20CA", 500);  // measured HGR20R section
var carriage = machinekit.motion.LinearRailBlock.metric("HGH20CA", machinekit.motion.LinearRailBlock.SUPPLIER);
var fixed = new machinekit.motion.VallderScrewSupport("BK15");
var floating = new machinekit.motion.VallderScrewSupport("BF15");
// Root/depth inputs here are explicit engineering assumptions, not supplier measurements.
var screw = new machinekit.motion.SfuDrawingScrew("SFU2005", 500, 16);
var nut = new machinekit.motion.SfuNut("SFU2005", 16);
var housing = new machinekit.motion.DsgNutHousing("DSG20H", 10, 10);
```

Use the ordinary component registry, assembly mates and BOM APIs. SupplierRecipeLibrary
also creates these recipes from actual Vallder SKUs, requiring explicit length,
race-root or blind-thread-depth inputs where the drawing does not establish them.
Editor recipe defaults are modelling assumptions (13 mm root for the default 16 mm
nut, 16 mm for the default 20 mm screw, 10 mm blind threads); they remain editable
and are persisted. Supplier bindings deliberately do not fill those missing inputs.
SourcingPlan adds optional offers without changing the engineering BOM. Stock
length pricing requires a quote; no piece price or availability is invented.

DSG geometry uses Y=0 as the base and +Z as the shaft. Nut flange contact is at Z=0;
rotate the SFU nut 90 degrees about Z and translate it by
`(0, housing.shaftHeight, -nut.flangeOffset-nut.flangeThickness)` so its flats and
six mounting axes match. Base and flange threads remain separate interfaces.
Their unknown engagement depths do not imply a request to modify purchased parts.

## Evidence and limits

- Rail sections are extracted between holes from each pinned Matis-hosted HIWIN
  reference, retaining exact lines/arcs. STEP endpoint gaps below 0.000001 mm are
  welded when making the wire. Nothing is scaled from HGR15.
- The HGR20 table specifies a 6.1 mm through bore; its reference CAD has 6.0 mm.
  Generated drilling follows the table. The equivalence check accounts for the
  exact expected removed annular volume rather than widening its tolerance.
- HGH mounting depths come from the published MxL table. HGW gives only M, so
  `mountThreadDepth()` returns null for those blocks.
- BK/BF mounting bores and FK/FF flange bores are checked by CAD intersections,
  including material controls beside the holes. FK17/FF17 fail and receive no
  supplier recipe. Their CAD remains available for further investigation.
- BK height table/CAD differences remain in `support-interface-review.json`.
  Mounting-axis checks do not certify the whole dimensional table.
- SFU20 pairs with size-15 journals/supports; SFU25 pairs with size-17 BK/BF.
  Those journal-centre checks do not establish bearing axial seats, preload,
  accuracy, tolerances, lubricant ports or load capacity. Larger complete axes
  still need their own installation/axial-stack validation.
- Screw races and lock threads are semantic/root envelopes. Minor chamfers and
  shoulder fillets are omitted; material/load/friction assumptions are not
  upgraded by a successful geometric fit.
