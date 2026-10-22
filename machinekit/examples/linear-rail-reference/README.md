# HGR15R / HGH15CA and HGR25R / HGH25CA supplier references

For the expanded HG, BK/BF/FK/FF, SFU and DSG recipes, see [motion families](FAMILIES.md).

This example checks the drawings of the one rail recipe (`machinekit.motion.linear-rail`) and the one carriage
recipe (`machinekit.motion.linear-rail-block`) against public supplier CAD. Download the pinned references, then
run the CAD checks and export:

```sh
python3 machinekit/examples/linear-rail-reference/prepare.py
./haxeon/scripts/haxeon run --project=machinekit/examples/linear-rail-reference/haxeon.json
./app/run-built.sh --project="$PWD/machinekit/examples/linear-rail-reference/build/comparison.mtrg" --perspective
```

The scene groups are `nominal` (left), `measured` (centre, the measured rail with the supplier carriage), and
`supplier` (right). The rail length is 150 mm and both carriages are centred at 75 mm. `build/comparison.json`
records bounds, solid volumes and the interference comparison for the 15 and 25 sizes.

## Source and provenance

Matis provides these files on its [CAD downloads page](https://www.matis.sk/sk/info/cad):

- [HGH15CA.STP](https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/HGH15CA.STP),
  SHA-256 `3bc285a6e79d23be1a5d6408382785361402575449bf40175d2319b27a55c9d8`.
- [HGR15R.STP](https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/HGR15R.STP),
  SHA-256 `2897086393c72de3ca2a1af658da9e471bfbe38e234605e89dbb8497283e4351`.

Retrieved 2026-10-18. The STEP header attributes the carriage model to HIWIN GmbH,
dated 2007-01-11. These are distributor-hosted design references, not a current
manufacturer approval. No STEP files are redistributed in this repository;
`prepare.py` downloads them into the one HG reference directory,
`build/suppliers/vallder/hg-reference` (the same store `tools/suppliers/hg.py --prepare`
fills), and rejects changed content. Public download availability does not establish redistribution rights.
HIWIN's [current block dimensions](https://www.hiwin.de/en/Products/Linear-guideways/Blocks/Ball-guides/Series-HG-QH/HGH-QHH/c/4374)
and [rail dimensions](https://www.hiwin.pl/en/Products/Linear-guideways/Profile-rails/Ball-guides/Series-HGR/c/4377)
provide the dimensional cross-check. Load ratings, masses and preload come from the catalogue
rows (`LinearRailSystem`), never from the STEP files.

## One recipe, three drawings

```haxe
var rail = LinearRail.metric("HGH15CA", 400);                               // measured HGR15R section
var nominal = LinearRail.metric("HGH15CA", 400, LinearRail.NOMINAL);         // catalogue section
var block = LinearRailBlock.metric("HGH15CA", LinearRailBlock.SUPPLIER);    // pinned HGH15CA CAD
```

Catalogue rows are keyed by carriage (`MGN12C`, `HGH15CA`, `HGW25CC`) and name their
rail (`MGNR12R`, `HGR15R`). A rail's `geometry` is `nominal` or `measured`; HG rails
default to `measured`, the analytic line/arc running section taken from the pinned
HGR model (`HgReferenceData.section`), drilled with the rail table's bores. It needs
no STEP file at runtime. A carriage's `geometry` is `nominal` (the default) or
`supplier`, the pinned HGH/HGW CAD, imported on each `geometry()` call from
`MATERIA_HG_REFERENCE` or `build/suppliers/vallder/hg-reference` and checked against
its digest and reviewed frame. Saved assemblies record the choice; the drawing
changes neither the part number nor the datasheet mass.

The supplier carriage is already in our interface frame: X across, Y normal to the
mounting face and Z along travel. Its upper mounting face remains at Y=13, but its
sides reach Y=-10.7. The nominal block omits these lower sides; the supplier
drawing's `Envelope` is the complete exterior box, including the rail channel, so
use the Preview CAD for mating/contact checks. The reference rail is centred at Z=0
in its STEP and is shifted +75 mm for our start-at-zero frame. The HGH15CA tapped
holes are M4 x 5 deep, from the dimensional table; HGW tables give no depth.

The checks compare the measured section against the supplier rail using matching holes,
then check carriage clearance at three travel positions on each of 150, 237, 440
and 600 mm rails, for both sizes. They also check envelopes, bores and tapped holes.
Production cut lengths keep the fixed 20 mm first-hole / 60 mm pitch policy;
the supplier reference instead uses 15/75/135 mm. A cut-length change does not
stretch the running section.

## Hybrid mill preview

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/linear-rail-reference/haxeon.mill.json
./app/run-built.sh --project="$PWD/machinekit/examples/linear-rail-reference/build/mill-hybrid.mtrg" --perspective
```

This separate review scene fits all twelve carriages to the mill's measured HGR15
rails with their supplier drawing. XY and Z layouts accept an optional matching
`LinearRailBlock`. The normal mill build remains self contained and uses the
nominal block.

The reference fit checks qualify the rail/carriage interface only. They do not
repeat the mill's complete structural swept-volume audit with the larger carriage
exterior. Supplier CAD contains simplified internals and mixed real materials, so
carriage and rail masses are the catalogue's datasheet values, not CAD volume.

## Ballscrew exterior details

The generic SFU1605 nut now includes annular seal recesses, flange flats,
shoulder relief and a radial lubrication fitting. Its bore, six mounting holes
and flange face retain their original datums. The Envelope includes the fitting.
Shaft previews have end chamfers, shoulder reliefs and a floating-end retaining
groove; the nominal shaft envelope and stiffness model keep their original
journal sections. These small details are assumed reference geometry, not a
supplier end-machining specification or a qualification of the stiffness model.

`BallScrewRetainer` is a distinct notched retaining nut/washer kit in the mechanical
mill's scene tree and BOM. Its washer meets the input bearing face at shaft Z=25;
the kit also clears the X belt pulley. It is not a supplier-selected locking system.

The hybrid export checks added fitting material and all three retaining kits
against the other installed parts at 27 combinations of XYZ stops/midpoints.
This sampled check supplements the geometry tests; it does not replace a complete
continuous swept-volume audit of the detailed mill.

## Pinned references

The additional pinned references, retrieved 2026-10-19, are:

- [HGH25CA.STP](https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/HGH25CA.STP),
  SHA-256 `d1928e51da8a04694e98678ece89710cef6fa22dad3a6ddac47e54261fa491c6`.
- [HGR25R.STP](https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/HGR25R.STP),
  SHA-256 `a5c25503c681841fbbee95b24a1ebcd890afcf2001b074ee4a5b817e1fc07909`.

The other HG sizes (20, 30, 35, 45 and the HGW carriages) are pinned in
`tools/suppliers/hg-reference.json`; see [motion families](FAMILIES.md).
