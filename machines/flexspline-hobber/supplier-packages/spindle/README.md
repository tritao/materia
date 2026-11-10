# Hob spindle package

Builds the **BT30 hob arbor** drawing package from `machinekit.hobbing.HobSpindle` as
`FlexsplineHobber` installs it. The cartridge is one bought RobotDigg 2049 option A
D80 BT30 belt spindle, with rotor/bearings/drawbar included. The separate 1 kW
servo drives it through the existing 40T/40T HTD5M belt; the B encoder remains on
the rear stub. See FRAME_PLAN Decisions for the verified facts and explicitly
assumed packaging, accuracy and allowable belt load. The linked NT30 drawing
on the listing is not BT30 interface evidence.

```sh
cd ~/dev/materia
./haxeon/scripts/haxeon build --compiler-only \
  --project=machines/flexspline-hobber/supplier-packages/spindle/haxeon.json
N=$PWD/app/build/host/native
LD_LIBRARY_PATH=$PWD/haxeon/out:$N/cadkit ./haxeon/scripts/hl \
  machines/flexspline-hobber/supplier-packages/spindle/build/host/main.hl /tmp/spindle
python3 machines/flexspline-hobber/supplier-packages/spindle/spindle_package.py /tmp/spindle
```

The exporter writes the made arbor's STEP and exact axial section, the posed
`spindle-assembly.step`, and `spindle-package.json` with member positions and BOM.
The script writes `spindle-sections.dxf` and `spindle-drawings.pdf` (overview and
arbor sheet). It checks specified cylinders against the model's exact sections
and checks the encoder and arbor poses. PDFs need Chrome or Chromium.

The arbor has a MAS BT30 shank, Ø16 plain hob seat with integral face key,
M16x1 retaining nut and Ø15 x 22 outboard journal. Materials, hardening, grinding
and inspection are ordering requirements in `spindle_package.py`. The BT30
pull stud's head must match the bought gripper; its geometry remains assumed.
The machined package also exports the arbor with its manufacturing route.
