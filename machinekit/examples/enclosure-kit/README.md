# Enclosure kit example

Open `materia.project.json` in Materia and choose **enclosure**.
The example is a 1000 × 600 × 800 mm clear opening with twelve 40-series
extrusions, five ACM infill panels and a 500 mm polycarbonate sliding leaf.
The leaf travels 510 mm along X. Dimensions are a kit demonstration, not a
machine-specific envelope.

Panels use X for width, Y for thickness and Z for height from the bottom edge.
The panel recipe selects a complete catalogue construction; individual material
overrides are disabled so that layer geometry, mass and BOM stay consistent.
Layer order runs from negative to positive Y. Composite mass and centre of mass
include each layer; panel geometry is the overall rectangular blank.

The BOM carries construction, layers, dimensions, area and per-piece planning
mass. Panel recipes are registered for assembly save/reopen; supplementary BOM
records also preserve construction metadata through the wire schema.

`EnclosureRules.check` accepts overlapping panel zones and a declared process.
It reports unknown heat, coolant, fire and impact qualification as unresolved,
with panel ID, zone and rule. Qualification records are caller-supplied evidence
references; the checker does not certify their contents or applicability.
Default catalogue entries therefore do not pass protective-use acceptance.

This example demonstrates geometry and motion. It has no roof, sealed floor,
actuator, carriage hardware, guard lock or control state machine. Frame fasteners,
panel retention and service access remain design work. Swept machine-envelope
layout generation is EN4; shared guard behavior is EN5.

Focused data checks:

```sh
HAXEON_COMPILER_SERVER=0 ./haxeon/scripts/haxeon run --project=machinekit/tests/enclosure/haxeon.json
```

CAD acceptance (through the shared resource wrapper):

```sh
/home/joao/dev/materia-cache/tools/heavy-run --mem 4G --need 3G --label enclosure-geometry -- env HAXEON_COMPILER_SERVER=0 CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release ./haxeon/scripts/haxeon run --jobs=1 --project=machinekit/tests/enclosure/haxeon.geometry.json
```

Checks cover panel volume and mass, recipe reconstruction, BOM aggregation and
serialization, assembly save/reopen, and door intersections at eleven stroke
positions. The stroke check is sampled evidence. The runner exports
`machinekit/examples/enclosure-kit/build/enclosure.scene`.
