# Fitted hobber enclosure

Open `materia.enclosed.project.json` in Materia. **enclosed** shows the fitted
shell; **inspection** hides its occurrences. The original project and cutting
entrypoints are unchanged.

The current machine's CAD supplies the envelope: X/Y/Z endpoints, 31 swivel
angles at one-degree spacing, and rotation-centred spheres for B/C descendants.
The envelope is padded by the maximum axis-to-box-corner radius times half the
angular step. The separately mounted electrical rack stays outside this shell;
acceptance nevertheless checks enclosure collisions against it too.

Initial fit: approximately 1472 × 1103 × 787 mm clear shell size above the plinth,
a 600 mm loading opening and a 610 mm door stroke. The tray interface datum is
847 mm in the machine frame. Clearance is an assumed 75 mm, not a qualified
containment distance. The front reservation encloses the OD140 side-jaw envelope
from the existing loading study and extends outward through the enclosure; it
does not author or certify a robot path.

The layout includes a roof, two removable side service panels, a rear panel with
a 100 mm mist opening, and a steel tray ring with perimeter upstands. Its central
opening reserves the casting/plinth interface and existing chip volume. Service
opening checks establish unobstructed access through the frame after the named
panel is removed, not human reach or a complete tool-change procedure.

The tray and panel seams carry an explicit seal/gasket fabrication requirement.
Seals, the plinth transition, mounting/retention hardware, tray slope, coolant
flow, extractor performance and impact/fire ratings remain unqualified. The
layout report lists these rule violations; geometric clearance is not a claim
of fluid containment or protective qualification. Door actuator/guard controls
remain EN5 work. The opening collision proxy is exact for a rectangular cutout;
a round port uses a circumscribed-square aperture proxy, while its CAD is round.

Run the focused CAD acceptance from the worktree root:

```sh
/home/joao/dev/materia-cache/tools/heavy-run --mem 4G --need 3G --label hobber-enclosure -- env HAXEON_COMPILER_SERVER=0 CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release ./haxeon/scripts/haxeon run --jobs=1 --project=machinekit/examples/flexspline-hobber/haxeon.enclosure-checks.json
```

Outputs under `build/enclosure/`: `hobber.mtrg`, `layout.json`, `checks.json`.
Acceptance checks 48 machine/door positions, exact near-pair intersections,
the loading reservation, removable service openings, the round port cut and
mass, collision aperture proxy and assembly save/reopen. It is sampled clearance
evidence; the scene does not run a production cycle.

## Electrical rack mounting

The separate rack is floor-standing at machine Z0, using the machine/work-frame
transform instead of placing its base at the elevated work origin. Its upright
is joined to the base by two triangular steel webs. The six drives are arranged
in two rows of three with 20 mm horizontal gaps; the supply is below them.
Device local XY mounting faces lie on the vertical plate, with their depth
projecting outwards. Geometry checks confirm floor support, seven flush mounts,
plate coverage and no device/device or device/stand penetration. Supplier hole
patterns, fasteners, cable routing and electrical guarding remain design work.
The renewed enclosure acceptance passes 48 machine/door states and 624 exact
near-pair checks. The exported scene was reopened and visually checked.
