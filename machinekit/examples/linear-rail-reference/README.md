# HGR15R / HGH15CA supplier reference

This example compares nominal envelopes, a generated HGR15R running profile, and public supplier CAD.
Download the pinned references, then run the CAD checks and export:

```sh
python3 machinekit/examples/linear-rail-reference/prepare.py
./haxeon/scripts/haxeon run --project=machinekit/examples/linear-rail-reference/haxeon.json
./app/run-built.sh --project="$PWD/machinekit/examples/linear-rail-reference/build/comparison.mtrg" --perspective
```

The scene groups are `nominal` (left), `generated` (centre, with supplier carriage), and `supplier` (right). The rail length is
150 mm and both carriages are centred at 75 mm. `build/comparison.json` records
bounds, solid volumes, drilling patterns and the interference comparison.

## Source and provenance

Matis provides these files on its [CAD downloads page](https://www.matis.sk/sk/info/cad):

- [HGH15CA.STP](https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/HGH15CA.STP),
  SHA-256 `3bc285a6e79d23be1a5d6408382785361402575449bf40175d2319b27a55c9d8`.
- [HGR15R.STP](https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/HGR15R.STP),
  SHA-256 `2897086393c72de3ca2a1af658da9e471bfbe38e234605e89dbb8497283e4351`.

Retrieved 2026-10-18. The STEP header attributes the carriage model to HIWIN GmbH,
dated 2007-01-11. These are distributor-hosted design references, not a current
manufacturer approval. No STEP files are redistributed in this repository;
`prepare.py` downloads to the ignored local build directory and rejects changed
content. Public download availability does not establish redistribution rights.
HIWIN's [current block dimensions](https://www.hiwin.de/en/Products/Linear-guideways/Blocks/Ball-guides/Series-HG-QH/HGH-QHH/c/4374)
and [rail dimensions](https://www.hiwin.pl/en/Products/Linear-guideways/Profile-rails/Ball-guides/Series-HGR/c/4377)
provide the dimensional cross-check. Load/preload ratings and material composition
are not imported from the old STEP file.

## Component integration

```haxe
var block = new machinekit.motion.HiwinHgh15ca(pathToPinnedStep);
// Use anywhere a LinearRailBlock is accepted, including AxisBuilder.slideOn.
// Export geometry / perform checks while the component remains open.
block.close();
```

For a saved assembly, `new HiwinHgh15ca()` selects the registered
`machinekit.motion.hgh15ca-reference` recipe. It imports lazily, returns owned
geometry copies and retains no imported native CAD owner between calls. The
asset path comes from `MATERIA_HGH15CA_STEP` or the prepared directory under
`MATERIA_ROOT`; the file hash and mounting frame are checked on load. Material
is locked to unspecified for this recipe because the model contains several
materials. Explicit-path constructors retain their original code-only API and
caller-owned CAD lifetime. The mounting-hole thread depth is 5 mm (M4 x 5),
consistent with the HGH15CA dimensional table; it is not inferred from a plain
cylindrical hole in the STEP file.

The wrapper retains the existing HGR15 block's rail, axis, top and four mount
connectors, and is distinguished from the nominal block in geometry/BOM identity.
It imports once, then returns owned copies from `geometry(Preview)`. The caller
closes those copies as usual and closes the component after all exports/checks.
`Envelope` is the complete conservative exterior box, including the rail channel;
use actual Preview CAD for mating/contact checks. All dimensions are millimetres.

The supplier carriage is already in our interface frame: X across, Y normal to the
mounting face and Z along travel. Its upper mounting face remains at Y=13, but its
sides reach Y=-10.7. The nominal block omits these lower sides. The reference rail
is centred at Z=0 in its STEP and is shifted +75 mm for our start-at-zero frame.

The generated HGR15 preview uses an analytic line/arc section measured from the
pinned HGR15R model, with 4.5 mm through holes and 7.5 mm counterbores, 5.3 mm deep.
It needs no STEP file at runtime. Envelope detail remains a conservative box;
other profile sizes retain their nominal geometry until separately verified.

The checks compare generated CAD against the supplier rail using matching holes,
then check carriage clearance at three travel positions on each of 150, 237, 440
and 600 mm rails. They also check envelopes, bore clearance and connector frames.
The matching 150 mm pair has zero measured symmetric difference. Production cut
lengths preserve the generator's fixed 20 mm first-hole / 60 mm pitch policy;
the supplier reference instead uses 15/75/135 mm. Hole geometry follows the
existing mount connectors. A cut-length change does not stretch the running section.

## Hybrid mill preview

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/linear-rail-reference/haxeon.mill.json
./app/run-built.sh --project="$PWD/machinekit/examples/linear-rail-reference/build/mill-hybrid.mtrg" --perspective
```

This separate review scene fits all twelve HGH15CA carriages to the mill's
generated HGR15 rails. XY and Z layouts accept an optional matching `LinearRailBlock`,
so the supplier wrapper can be selected explicitly and shared for export. Callers
own its lifetime. The normal mill build remains self contained and uses the
nominal block; its HGR15 rail previews receive the improved generator automatically.

The reference fit checks qualify the rail/carriage interface only. They do not
repeat the mill's complete structural swept-volume audit with the larger carriage
exterior. Supplier CAD contains simplified internals and mixed real materials;
CAD volume with a uniform steel density is not a verified carriage mass or inertia.

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
