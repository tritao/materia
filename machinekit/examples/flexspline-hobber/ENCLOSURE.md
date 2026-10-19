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
of fluid containment or protective qualification. The shared EN5 guard controller and fitted-door kinematic adapter are implemented
and exercised below. The enclosed desktop scene also binds the guard to the live
MuJoCo cutting runtime. Hardware actuator/lock selection remains separate work. The opening collision proxy is exact for a rectangular cutout;
a round port uses a circumscribed-square aperture proxy, while its CAD is round.

Run the focused CAD acceptance from the worktree root:

```sh
/home/joao/dev/materia-cache/tools/heavy-run --mem 4G --need 3G --label hobber-enclosure -- env HAXEON_COMPILER_SERVER=0 CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release ./haxeon/scripts/haxeon run --jobs=1 --project=machinekit/examples/flexspline-hobber/haxeon.enclosure-checks.json
```

The external electrical rack, power supply, and all drivers are hidden by default
in enclosure and inspection views. Their geometry remains available in the scene tree.

Outputs under `build/enclosure/`: `hobber.mtrg`, `layout.json`, `checks.json`.
Acceptance checks 48 machine/door positions, exact near-pair intersections,
the loading reservation, removable service openings, the round port cut and
mass, collision aperture proxy and assembly save/reopen. It is sampled clearance
evidence; these checks do not qualify a production cycle.

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

## Door control and guarding (EN5 simulation)

`processkit.guard.EnclosureGuard` provides a reusable controller contract.
`HobberEnclosureControl` binds it to `enclosure/door/slide` and requires a host
consumer for its stop, cutting permission, and setup-motion permission outputs.
The exported assembly contains `enclosure/door/closed` and `enclosure/door/open`
coordinate switches with 0.5 mm end windows. The kinematic actuator is bounded
to 0–610 mm at an assumed 200 mm/s; it does not model pneumatic force or stopping
distance and does not select physical actuator hardware.

An open request removes cut permission and requests a stop. The lock stays
commanded until both spindle and axis standstill are confirmed. Opening then
waits for separate unlocked feedback. Reclosing waits for the closed switch
and independent lock confirmation before accepting a fresh cut request. A
request held through opening, closing, a fault, or reset cannot restart a cut.
A lost guard, invalid feedback, E-stop, boundary intrusion, or feedback timeout
latches a stop. Reset requires closed-and-locked feedback, both standstill
confirmations, a healthy safety chain, and released start/access requests.

Door-open setup motion requires setup mode plus hold-to-run, spindle standstill,
and measured axis speed within a configured limit. The hobber simulation uses
an assumed 0.025 m/s policy limit; it never grants cutting/spindle permission in
setup mode. This value is a model parameter, not a qualified machine safety limit.

Run the focused controller and fitted-door checks:

```sh
./haxeon/scripts/haxeon run --jobs=1 --project=processkit/tests/guard/haxeon.json
./haxeon/scripts/haxeon run --jobs=1 --project=machinekit/examples/flexspline-hobber/haxeon.enclosure-guard-checks.json
```

The full enclosure acceptance also checks the control adapter and samples the
moving leaf against stationary enclosure parts at 21 positions across its stroke.
`build/enclosure/guard-checks.json` records the controller simulation results.
Lock feedback and spindle/axis standstill are independent simulated inputs in
these checks; commands and elapsed time never substitute for their confirmation.
The desktop's manual joint slider remains a geometry preview. In the enclosed
scene, press Play and use the Machine panel's enclosure controls: Open door,
Close door and Start cut. Cutting requires a fresh Start cut after closed/locked
confirmation. Opening during a cut requests a controlled stop and waits for
measured spindle and axis standstill before releasing the independent simulated
lock. Door movement uses a process-owned joint, separate from cutting plans.

The live model assumes a 200 mm/s door rate, 400 mm/s² acceleration, 50 ms lock
actuation, and 250 ms of consecutive measured standstill. Thresholds are
0.01 mm/s for linear axes and 0.0001 rad/s for rotary axes. These are simulation
policy values. MuJoCo is required for the authored rotary brakes. Door-open setup
jogging is not exposed by the live controls.

After access interrupts a cut, use Reset simulation before starting again: this
restores stock and requires another explicit start. Reset guard only clears a
safe guard fault; it does not resume an interrupted cut. Native E-stop or boundary
interruption removes cutting permission and latches the guard. No hardware safety
qualification is claimed by this simulation.

Run the native integration check (using this branch's native libraries):

```sh
/home/joao/dev/materia-cache/tools/heavy-run --mem 6G --need 5G --label enclosure-live -- env HAXEON_COMPILER_SERVER=0 ./haxeon/scripts/haxeon run --jobs=1 --project=app/tests/enclosure-guard/haxeon.json
```

Validation: 43 shared-controller assertions and 971 fitted-door assertions pass.
The CAD acceptance passes 48 machine/door states, 624 exact near-pair checks,
21 leaf-travel samples, loading/service clearances and assembly save/reopen.

Live native acceptance passes 18,798 assertions using the fitted coarse hobber:
artifact binding validation, native end switches and lock feedback, no automatic
cut on Play, actual hob-spindle rotation, controlled stop before unlock, door
reclosure, no access/reset restart, boundary interruption, native E-stop feedback
and operator-panel actions. Contact collisions are disabled in this controller
fixture; the separate CAD clearance checks remain the geometry evidence.
