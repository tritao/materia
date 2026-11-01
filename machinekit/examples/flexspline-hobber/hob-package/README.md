# Supplier hob package

Builds the files sent to hob makers for the reference flexspline hob, from the
same `gearkit` rack and `HobDesign` that `FlexsplineHobber` cuts with (m 0.3,
z 200, 20°, single start, Ø32 × 40 mm, 12 gashes, Ø16 bore).

```sh
cd ~/dev/materia
./haxeon/scripts/haxeon build --compiler-only \
  --project=machinekit/examples/flexspline-hobber/hob-package/haxeon.json
LD_LIBRARY_PATH=$PWD/haxeon/.tools/hashlink:$PWD/haxeon/out ./haxeon/.tools/hashlink/hl \
  machinekit/examples/flexspline-hobber/hob-package/build/host/main.hl /tmp/hob-package.json
python3 machinekit/examples/flexspline-hobber/hob-package/hob_package.py /tmp/hob-package.json /tmp/hob-package
```

Output: `hob-datasheet.pdf`, `gear-datasheet.pdf`, `hob-rack-profile.dxf` (DXF R12,
mm, 1:1) and `hob-rack-profile.csv`. PDFs need Chrome or Chromium on the PATH.

The constants in `HobPackageExport.hx` mirror the `FlexsplineHobber` defaults and
`HOB_GEOMETRY.md`; change both together. Ordering requirements that the model does
not hold (accuracy class, bore tolerance, coating, material) live in `hob_package.py`.
