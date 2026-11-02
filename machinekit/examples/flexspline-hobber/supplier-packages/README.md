# Supplier packages

Builds the files sent to suppliers for the reference flexspline, from the same
`gearkit` rack and `HobDesign` that `FlexsplineHobber` cuts with (m 0.3, z 200,
20°, single start, Ø32 × 40 mm hob, 12 gashes, Ø16 bore), and the workholding
derived from the same gear the way `FlexsplineHobber` builds it. The exporter
depends on `gearkit` and `camkit`.

```sh
cd ~/dev/materia
./haxeon/scripts/haxeon build --compiler-only \
  --project=machinekit/examples/flexspline-hobber/supplier-packages/haxeon.json
LD_LIBRARY_PATH=$PWD/haxeon/.tools/hashlink:$PWD/haxeon/out ./haxeon/.tools/hashlink/hl \
  machinekit/examples/flexspline-hobber/supplier-packages/build/host/main.hl /tmp/supplier-package.json
python3 machinekit/examples/flexspline-hobber/supplier-packages/hob_package.py /tmp/supplier-package.json /tmp/hob
python3 machinekit/examples/flexspline-hobber/supplier-packages/mandrel_package.py /tmp/supplier-package.json /tmp/mandrel
```

Hob package: `hob-datasheet.pdf`, `gear-datasheet.pdf`, `hob-rack-profile.dxf`
(DXF R12, mm, 1:1) and `hob-rack-profile.csv`.

Mandrel package: `mandrel-datasheet.pdf`, `mandrel-section.dxf` (half section,
r = x, z = y, mm; layers CUP, SLEEVE, STOP_LAND, MANDREL_NOMINAL, HOB_KEEPOUT,
MANDREL_MAX_ENVELOPE, AXIS, SEAT) and `hob-keepout.csv`. The keep-out is the swept
hob body at full depth over the hob centre heights of the real CAM job
(`camkit.HobbingJob` with the cup blank: engagement overrun plus axial clearance,
7.49 mm below the seat for the reference), swivel over the exported range, with
0.3 mm minimum clearance. It is sampled and checked against the closed-form in-plane
envelope (agreement better than 0.001 mm). It covers the hob only; check a supplier's
mandrel STEP in the machine model for the arbor and outboard support.

PDFs need Chrome or Chromium on the PATH.

The constants in `SupplierPackageExport.hx` mirror the `FlexsplineHobber`,
`FlexsplineCupBlank`, `FoundationLayout` and `HOB_GEOMETRY.md` values; change them
together. Ordering requirements the model does not hold (accuracy class, bore
tolerances, coating, material, inspection) live in the two Python scripts.
