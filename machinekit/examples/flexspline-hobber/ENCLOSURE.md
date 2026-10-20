# Fitted hobber enclosure

Open `materia.enclosed.project.json` in Materia. **enclosed** shows the fitted
shell; **inspection** hides its occurrences. The original project and cutting
entrypoints are unchanged.

The current machine's CAD supplies the envelope: X/Y/Z endpoints, 31 swivel
angles at one-degree spacing, and rotation-centred spheres for B/C descendants.
The envelope is padded by the maximum axis-to-box-corner radius times half the
angular step. The separately mounted electrical rack stays outside this shell;
acceptance nevertheless checks enclosure collisions against it too.

Opposed-door fit: approximately 973 × 1103 × 787 mm clear shell size above the plinth,
a 400 mm loading opening and a 210 mm stroke per leaf. The previous single-leaf
fit was 1472 mm wide; splitting the opening removes its large right-hand parking bay. The tray interface datum is
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
to 0–210 mm at an assumed 200 mm/s; it does not model pneumatic force or stopping
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
The Machine panel shows Open door and Close door before Play, under
`ENCLOSURE · DESIGN PREVIEW`. These are undoable geometry poses, including the
coupled leaf, and do not start simulation or change the saved initial pose.
During simulation the panel is labelled `ENCLOSURE · SIMULATION` and uses the
measured guard controller. In the enclosed scene, press Play and use Open door,
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

Validation: 43 shared-controller assertions and 371 fitted-door assertions pass.
The CAD acceptance passes 48 machine/door states, 624 exact near-pair checks,
21 leaf-travel samples, loading/service clearances and assembly save/reopen.

Prior single-leaf live native acceptance passed 18,798 assertions using the fitted coarse hobber:
artifact binding validation, native end switches and lock feedback, no automatic
cut on Play, actual hob-spindle rotation, controlled stop before unlock, door
reclosure, no access/reset restart, boundary interruption, native E-stop feedback
and operator-panel actions. Contact collisions are disabled in this controller
fixture; the separate CAD clearance checks remain the geometry evidence.


## Opposed sliding doors and proposed actuation

Two 200 mm wide leaves meet at the centre of the 400 mm opening. Each travels
210 mm outward, leaving 10 mm beyond its opening edge. The loading reservation
is 190 mm wide (OD140 plus 25 mm side jaws on each side), giving 105 mm lateral
allowance per side at the doorway. This remains an assumed gripper envelope.
The existing 75 mm chamber clearance is retained.

Proposed actuation: one bought rod-style electric actuator with at least 210 mm
usable stroke drives one leaf. A passive endless belt links the second leaf to
the opposite run. Its motor, feedback, speed, force and duty cycle are selected
from the eventual supplier drawing; no actuator rating is claimed yet. The
actuator carries push/pull loads, while the door suspension carries door weight.
The actuator is separate from the guard locks.

The modeled suspension uses a 44 mm wide steel captured track with an 8 mm
bottom slot. Two hangers per leaf carry paired nominal 608 rollers (22 x 7 mm)
on removable 8 mm shoulder axles, with M8 washer spacers and an M6 washer/nut
retainer. Roller bottoms sit on the lower track lips; the roof leaves 4 mm
anti-lift clearance. Each hanger neck has 1 mm side clearance in the slot.
These are assumed design envelopes for selecting an industrial roller kit, not
supplier-qualified load ratings. Roller spin is omitted from the kinematic model.
End stops bound the open stroke, and a removable 2 mm steel splash hood covers
the front and top of the track. The T-slot enclosure frame remains its support.

Each leaf has a downward-open bottom channel. Two stationary UHMW guide shoes
per side transfer engagement as the short leaf travels, with 2 mm nominal side
and roof clearance. They sit outside the loading corridor; no track crosses the
loading threshold. The CAD acceptance checks shoe engagement throughout travel,
moving suspension/track clearances, and the open loading corridor.

The track, hangers, axles, rollers, channels, shoes, stops and splash hood are
separate persisted parts. Bracket bores, adjustment slots, mounting fasteners,
retainer locking method and attachment fasteners for track/shoes remain fabrication
details; the bracket/axle seat is an intentional nominal mounting overlap.
The belt and rod actuator are not yet modeled or included as selected hardware.
The retained 200 mm/s leaf speed and 400 mm/s² acceleration are assumptions.
A trapezoidal 210 mm move at those limits takes about 1.55 s, before
unlock/standstill delays. The simple kinematic adapter advances at constant
speed; the live runtime owns acceleration limits.

The existing guard controls the master joint; the second leaf follows through
the assembly coupling. Its current end-switch simulation assumes an intact
coupling. Physical closed/locked sensing must cover both leaves so belt/clamp
failure cannot leave an undetected opening. Include monitored closing-edge
protection and separate guard locking; motor current alone is not a guard lock.
These hardware details remain open, as with the original single-leaf design.

Actuation references (architecture examples, not selected products):
[DICTATOR belt-operated double-leaf doors](https://en.dictator.de/products/door-drives-gate-drives/operators-sliding-doors/dictamat-900-21/)
and [Mayser monitored safety edges](https://www.mayser.com/en/safetytechnology/safety-edges/closing-safety-edge).


Current opposed-door acceptance: 48 machine/door states, 624 exact machine/shell
near-pair checks, 21 suspension-travel samples with both lower channels engaged,
loading and service clearances, mass and save/reopen all pass. The fitted guard
passes 371 assertions. The complete collision scene builds, and live native
control passes 10,493 assertions covering opening/reclosure, cut gating,
controlled access stop, reset, boundary interruption and E-stop. Runtime contact
qualification remains separate; controller motion is tested with contacts
turned off, as in the preceding single-leaf fixture. See
`ENCLOSURE_CHECKS_REPORT.json` and `ENCLOSURE_LAYOUT_REPORT.json`.

## Base interface: router-cut laminate study

`HobberBaseInterfaceStudy` compares the proposed seating details against the
current casting dimensions, core openings and mounting-insert envelopes. It
selects `compact-laminate-13` (13 mm, RouterCut) for the study. This catalogue
construction carries assumed generic material properties; its commercial
chemical grade, coolant/cleaner compatibility and chip-heat suitability remain
unqualified.

The starting layout retains a 25 mm-wide, 4 mm-deep moulded ledge. With an
assumed 2 mm compressed gasket, the panel top is 11 mm above the casting top.
The flush alternative needs a 15 mm recess. Both candidate rebates clear the
existing mounting inserts and lightening cores. Both intersect the rear X-drive
channel, requiring a removable sealed service bridge. The reported 21 / 10 mm
clearances to the general-core roof datum are conservative datum comparisons,
not remaining wall thicknesses or strength calculations.

The raised inner edge requires a sealed collar/drainage transition so coolant
cannot collect against it. Moving-machine clearance, panel supports/deflection,
local insert bosses and clamp loads, gasket compression stops and dedicated
frame mounting pads remain to design. The casting and current steel enclosure
tray are unchanged by this study.

Generate the checked comparison and section drawing from the worktree root:

```sh
/home/joao/dev/materia-cache/tools/heavy-run --mem 3G --need 2G --label hobber-base-interface -- env HAXEON_COMPILER_SERVER=0 ./haxeon/scripts/haxeon run --jobs=1 --project=machinekit/examples/flexspline-hobber/haxeon.base-interface.json
```

Outputs: `build/base-interface/interface.json` and `interface.svg` beneath this
example directory. The SVG shows the casting plan, interrupted rear seal route,
and raised/flush edge sections at the same scale.

Editor preview acceptance: the extended live/UI fixture passes 10,502 assertions,
including controls before Play, coupled-leaf motion, Undo/Redo and closing without
starting simulation. The existing live guard checks remain in the same fixture.

A prepared but paused simulation offers Resume simulation directly in the door
panel. Until the first physics samples arrive it reports unsampled feedback,
rather than claiming that the door is moving or unlocked. Opening a door in
Design is retained when simulation is prepared; live guarding recognizes the
open pose and can close/lock it after resuming. Guard faults remain visible while
paused. Preparing or resuming never constitutes a fresh Start cut request.

Open-pose startup acceptance: 52 shared guard checks and 10,507 live/UI checks
pass. The transition fixture previews both leaves open, prepares simulation,
waits for confirmed Open access, and closes/locks through the live controller
without granting cutting permission. Missing initial standstill still times out,
and losing standstill after access is admitted still faults.

The controller publishes an immutable `EnclosureGuardStatus` image. Its typed
`state` is `paused`, `initializing`, `ready` or `faulted`; feedback sampling and
completed startup confirmation are separate fields. `ready` means playback is
running and startup feedback/standstill has been confirmed, not that cutting is
permitted. Consumers use the published `canOpen`, `canClose`, `canStartCut`,
`canReset`, `canResume` and `canResetSimulation` action flags. Guard readiness
(`canStart`) and the guard's `allowCut` output remain available to deterministic
stepping clients independently of realtime playback.

Feedback images are published by the simulation owner, not reconstructed by the
UI from mutable guard inputs. Transport transitions publish a new image under
the same mutex; old images remain unchanged. Reset clears initialization and
feedback freshness. Fault messages take priority even while paused. The panel
renders the controller's messages and action flags without interpreting phase
strings or querying playback state to infer guard readiness.

Published-status acceptance: 15 contract assertions and 10,514 native/UI
assertions pass, including immediate transport publications, preserved old
snapshots, reset freshness and the open-design-pose transition.

In Simulate mode, the 3D toolbar's **Camera** button opens an inspection pane
below the Sensors/CNC tabs; Design hides it. The enclosed
and cutting scenes provide **Tool contact**, **Workpiece** and **Loading** presets.
Tool contact is a camera-only cutaway hiding the surrounding head and support.
The first two follow the cup without spinning with it; Loading frames the
mandrel. The enclosure is hidden only in this camera, leaving the main view and
guard controller unchanged. **Picture in picture** uses the same camera inside
the main viewport; **Dock** returns it to a pane. Presets are scene metadata in
`HobberInspectionViews`, with world-aligned offsets in metres.
