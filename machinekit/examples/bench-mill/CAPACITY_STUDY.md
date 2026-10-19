# Compact VMC capacity study

This is a separate packaging study alongside the committed 250 × 160 × 250 mm
mill. Open the `compact-vmc` entrypoint of `materia.project.json` (the old
`materia.capacity.project.json` remains a compatibility shortcut), or run from the repository root:

```sh
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/bench-mill/haxeon.capacity.json
```

Outputs under `build/capacity/` are `mill-capacity.mtrg`, a persistent assembly,
`dimensions.svg`, and `checks.json`. The viewer exposes X/Y/Z joints. The initial
pose is centered in X/Y and at the upper Z stop. The shared Z axis retains its original convention: Z=0 is
the upper stop and Z=-380 mm is the lower stop.

## Working envelope

| Parameter | Study target |
| --- | ---: |
| XYZ travel | 400 × 300 × 380 mm |
| Table | 700 × 400 mm |
| Minimum table-edge coverage at the spindle axis | X 150 / Y 50 mm |
| Spindle gauge line above table | 150–530 mm |
| Column-face to spindle-axis reach | 400 mm |
| Fixture envelope above table | 100 mm, 200 × 150 mm plan |
| Payload design target | 150 kg, not verified |
| Overtravel allowance | 5 mm beyond each working stop |

The reference is the [SYIL X7 published specification](https://us.syil.com/x7)
(400 × 300 × 380 mm travel, 700 × 300 mm table, 358 mm reach and 150–530 mm
spindle-to-table range). This study uses a deeper table to keep 50 mm of Y
coverage at both stops. That also requires more spindle reach. Matching travel
and taper does not establish equivalent cutting performance.

`MillCapacityLayout` derives guide lengths, saddle size, spindle reach, rail
positions, column height and base footprint from the travel. At the defaults:

| Structure / guide | Study dimensions (mm) |
| --- | ---: |
| Base, width × depth × height | 810 × 1354 × 280 |
| Column, width × depth × height | 460 × 400 × 1170 |
| Saddle, width × depth × height | 710 × 360 × 100 |
| X / Y / Z rail lengths | 670 / 720 / 750 |
| X / Y / Z rail spacing | 260 / 360 / 300 |
| Table top above casting bottom | 574 |

HG25 is a preliminary packaging choice using local pinned HGR25R/HGH25CA data
and detailed carriage geometry. It is not a guide life or load qualification.
The frame reuses the shared pocketed mineral-casting construction. The complete Z mechanism reuses `MillZAxis`, including the carriage and nut shelf,
head, screw, nut, fixed/floating supports, motor stand, brake, coupling, shaft,
retainer, guard and fasteners. It is nested under `zAxis/` in the scene. The frame
includes its central screw channel and continuous steel mounting strips. The full shared spindle assembly surrounds the BT30 study cartridge: motor,
belt drive, guard, rotor, springs, holder, drawbar/gripper, reaction frame and
pneumatic release cylinder are all retained. The larger head is 170 mm high so
its top clears the existing belt/drawbar stack; the fixture/stock envelope is
100 mm high so the holder clears it at the lower Z stop. Tool/work engagement
is a named exclusion, not a general collision exemption. Column height now follows the upper
rail/bearing mount instead of leaving an unused 200 mm extension behind the motor.
The original reference 750 W servo, 16×5 screw and BK12/BF12 supports are restored
for packaging; this is not a completed drive selection or load/critical-speed check.
The saddle, table and risers are solid packaging envelopes. They have
no finished slots, screw corridors, bearing seats or drilling. The fixture is an
envelope, not a purchased vise.

## Checks and limits

The executable checks 27 XYZ poses, signed motion, screw lead coupling, stationary
Z motor position, table coverage, guide support
at both working limits plus overtravel, and save/load persistence. It checks
continuous swept bounding volumes of all translating members against the fixed
structure and the complete Z moving group against the XY moving group. The shared
`MillZAxisIntegrity` audit also checks the detailed carriage plate, nut shelf,
nut, head and fasteners against the frame and fixed Z hardware through the stroke
plus 5 mm overtravel. Head bolts retain straight tool approaches and the motor body
has 150 mm of vertical removal space after unbolting. Intended
block/rail contacts are checked separately against an undrilled HG25 running
profile; a deliberately misplaced block must collide. The bounding volumes are
conservative supersets of the rigid translations, not a sample-only clearance
claim. Fixtures move with the table, not with the spindle.

Checks cover the represented packaging. They do not qualify hardware absent from
the study. In particular:

1. Detail the table and saddle, including nut and bearing access. Fit
   candidate SFU20/SFU25 drives from the local catalog and check complete races,
   supports, couplings, screw buckling and critical speeds.
2. Size Leadshine motors from the resulting moving masses, payload, acceleration,
   friction, duty and Z holding requirements. Existing small-mill motors are not
   automatically selected for this variant.
3. Qualify the existing spindle drive and release mechanism; package the ATC,
   enclosure and remaining service paths.
4. Rebuild the FEM input from this geometry and measure tool-to-workpiece static
   compliance and modes. Do not reuse the smaller mill's structural results.
5. Calculate finished mass, individual foot reactions and support compliance,
   including payload and cutting loads, before choosing Vallder feet.

No finished-machine mass or X7-equivalent performance is claimed. The study is a
reviewable starting layout; neither the original mill nor its previews are changed.
