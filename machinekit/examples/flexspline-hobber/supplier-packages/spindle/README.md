# Hob spindle package

Builds the hob spindle drawing package from `machinekit.hobbing.HobSpindle` as
`FlexsplineHobber` installs it (coupled 1 kW servo, floating rear 6205, P4 pair,
1:5 taper-and-face arbor held by four M4 screws). The exporter depends on `machinekit`
and needs CadKit's native library at run time.

```sh
cd ~/dev/materia
./haxeon/scripts/haxeon build --compiler-only \
  --project=machinekit/examples/flexspline-hobber/supplier-packages/spindle/haxeon.json
N=$PWD/app/build/host/native
LD_LIBRARY_PATH=$PWD/haxeon/.tools/hashlink:$PWD/haxeon/out:$N/cadkit ./haxeon/.tools/hashlink/hl \
  machinekit/examples/flexspline-hobber/supplier-packages/spindle/build/host/main.hl /tmp/spindle
python3 machinekit/examples/flexspline-hobber/supplier-packages/spindle/spindle_package.py /tmp/spindle
```

The exporter writes a STEP per made part (shaft, housing, gland,
locknut, arbor) in its own frame, `spindle-assembly.step` (all members posed), and
`spindle-package.json`: the exact axial section of each made part (CadKit planar
section through the axis), member positions and the BOM. The script writes
`spindle-sections.dxf` (layer per part, x axial from the housing rear datum, y radial)
and `spindle-drawings.pdf`: an overview and one page per made part.

Tolerances, materials, heat treatment and inspection live in `spindle_package.py`.
Every toleranced diameter row must match a cylinder in the model section over its
span, and the script stops if a bearing, the encoder or arbor moved, so the
specification cannot silently drift from the model. PDFs need Chrome or Chromium.
