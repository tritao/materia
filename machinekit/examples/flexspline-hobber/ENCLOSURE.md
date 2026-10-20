# Fitted hobber enclosure

Open `hobber.scene.json` or `materia.project.json` in Materia. This is the one
complete machine scene, with detailed hob, cup, fitted enclosure and guard.
The old cutting, loading and enclosed scene variants have been retired.

The viewport controls select **Idle / doors**, **Cutting** (default), or
**Load / cut / unload** before Play. Pause before changing the scenario; a change
resets the runtime and stock while retaining the same authored assembly.
Cutting closes and locks the doors from measured feedback before starting. The
full cycle parks the head, opens for loading, closes and locks to cut, retracts
and opens to unload. An interrupted or faulted cycle requires Reset; it never
automatically restarts. Transfer and sleeve expansion remain illustrations,
not a gripper/contact simulation.

**Visible**, **Transparent** and **Hidden** change only the main camera
presentation. The enclosure bodies, door switches, guard lock and interlocks
remain active in every viewing mode. Inspection-camera cutaways are independent.
Idle runs the guard for door operation, without running a cutting program.

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

The study itself only compares sections. The raised case is now integrated in
the scene as described below. Drainage at the raised inner edge, panel
deflection, clamp loads, gasket compression stops and actual material/seal
qualification remain engineering work.

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
below the Sensors/CNC tabs; Design hides it. The unified scene provides **Tool contact**, **Workpiece** and **Loading** presets.
Tool contact is a camera-only cutaway hiding the surrounding head and support.
The first two follow the cup without spinning with it; Loading frames the
mandrel. The enclosure is hidden only in this camera, leaving the main view and
guard controller unchanged. **Picture in picture** uses the same camera inside
the main viewport; **Dock** returns it to a pane. Presets are scene metadata in
`HobberInspectionViews`, with world-aligned offsets in metres.

The enclosure frame now mounts independently of the tray. Four dry-side steel
outrigger envelopes bear against the underside of the lower side extrusions.
Each is fixed to its own 80 × 80 × 20 mm cast-in side pad, with a reserved solid
120 × 70 × 120 mm boss behind the pad rather than an anchor in the hollow skin.
The frame has one tree attachment and three closure constraints, so all four
supports are explicit in the saved assembly. The tray is absent from this load
path and the mounting envelopes remain below the wet floor. Four separate
6 mm angle envelopes on the inner lower-frame faces support the current tray
from beneath; their 25 mm shelves support its compressed gasket.

These are design envelopes, not fabrication-ready brackets: bolt bores, slots,
fastener sizing, insert anchorage, levelling and expansion allowance still need
detailing. CAD acceptance checks all four extrusion contacts, mount/shell
interference, the full machine envelope, loading/service access and save/reopen.
The scene now uses the 13 mm laminate tray and recessed casting ledge. The
13 mm raised case remains the selected baseline; the flush case is a comparison.

Mount/support acceptance: four frame contacts and four tray support contacts,
48 machine/door states with 1,008 exact near-pair checks, 21 door travel samples,
loading/service access and save/reopen passed. The raised/flush laminate study
also still passes with the new side inserts and reserved bosses.


## Integrated removable laminate tray

The tray uses six rectangular router-cut blanks in four regions: two full-depth
side strips, a front strip, and three rear blanks around the X motor. Their
outer edges have 2 mm clearance to the frame. The 2 mm joints between blanks
sit over backing strips and compressed gasket envelopes; removable wet-rated
joint sealant is still required. No bolts pass through the laminate. The
material remains generic compact laminate, not a qualified commercial grade.

The casting now has the 25 mm-wide, 4 mm-deep perimeter rebate. A 2 mm
compressed gasket under the 13 mm panels gives a top surface 11 mm above the
casting. Separate outer support angles and joint backing strips carry the
outside edges. Ten stepped clamps bear on the tray surface. Their M6 × 30 bolts
seat in separate 20 mm-deep steel insert envelopes with nominal M6 bores and
reserved solid backing. Thread helices, anchor retention and tightening loads
are not qualified by these nominal CAD bores.

The rear motor prevents a flat bridge at the ledge level. A removable stepped
steel cover spans the interrupted rebate instead: 150 × 30 mm roof, 6 mm feet,
and a roof underside 13 mm above the casting. Two taller toe clamps with M6 × 35
bolts retain this cover independently. The feet sit in 10 mm-deep pockets either
side of the drive opening. Separate compressed gasket envelopes follow the
cover flange and rear blank edges. Remove the two cover clamps and cover, then
the tray clamps and joint sealant before lifting the blanks. The enclosure
frame stays on its own casting mounts throughout.

These are nominal construction details for review. The tray is flat: drainage
falls, the inner collar transition, coolant/cleaner and chip-heat compatibility,
compression stops, bolt torque, insert anchorage and leak/strength testing
remain required before fabrication release.

Integrated tray acceptance passes: six 13 mm blanks at the -2 / +11 mm stack,
12 clamp toe contacts, all tray-part interference checks, 25 mm sampled lift-out
clearance with the clamps/cover removed, 48 machine/door states, service/loading
access and save/reopen. Repeated static posed pairs are checked once; moving
pairs retain their individual poses. The generated `build/enclosure/tray-cut-list.csv`
lists the six rectangular blanks in millimetres. These are nominal design sizes,
not a fabrication release.

## T-FIXX reference and placement review

The candidate is T-FIXX M10 × 50 A4. The DXF supplied through Cast-in Portugal
is a 2D side-view library, version 1.20 dated 2014-01-30, labelled with legacy
identifier 0020.270-00101. `inspect-tfixx-reference.py` checks the original
millimetre units, candidate labels, 50 mm metal-body length and 13.5 mm sleeve
diameter. These main dimensions agree with the manufacturer's March 2026
Demu-Fix 25-EN catalogue, current candidate order 1000200636. The drawing clip
projects 1.5 mm; the current catalogue specifies a 2 mm identification clip.
Supplier confirmation of part equivalence and seating is still required. This
is not a thread model or a supplied 3D solid. The original library is kept
unmodified outside project history.

Regenerate the reference record and placement study:

```sh
python machinekit/examples/flexspline-hobber/inspect-tfixx-reference.py /path/to/T-FIXX.dxf
./haxeon/scripts/haxeon build --compiler-only --project=machinekit/examples/flexspline-hobber/haxeon.tray-anchor.json --output=/tmp/hobber-tfixx-study.hl
# Run the resulting HashLink module with the usual native-library environment.
```

The authoritative product source is the [March 2026 manufacturer catalogue](https://www.halfen.com/cdn/Medien%20I%20Media/Druckschriften%20-%20Printed%20materials/Technische%20Produktinformationen/PB_Halfen_Demu-Fix_25_v03-2026_EN.pdf),
printed pages 14–15. The [original DXF archive](https://www.halfen.com/Softwaredaten/Downloads/Software-CAD/CAD-Bibliotheken-AutoCAD/Verankerungstechnik/demu_cad_tfixx_dxf.zip)
can be inspected without converting the DWG.

The current 25 mm ledge and individual M6 clamp seats do not accept this anchor
as a direct replacement. `HobberTrayAnchorStudy` screens ten candidate axes
55 mm from the outside casting edges, local 60 × 80 mm seat extensions, and
70 × 70 × 70 mm solid backing reservations. It excludes the old tray-clamp
inserts because those would be replaced, while retaining all precision,
lifting and enclosure-frame inserts. Anchor envelopes must clear service
voids and retained inserts after reserving the backing. Local rebate extensions
must also clear those inserts. The screening uses a conservative 24 × 24 × 50 mm
anchor envelope, not a fabricated detailed solid.

The manufacturer's concrete layout minima are 50 mm to an edge and 100 mm
between axes. These are screening references; they do not establish capacities
in epoxy mineral casting. The continuous 80 mm front rebate reaches the work well; the screened local
tabs avoid that interruption. The current rear insert geometry actually clears
an 80 mm rebate, correcting the earlier assumption about X-rail overlap.
The one-piece tray needs local tongues
and a matching gasket path instead. Flush M10 screw seating (or a specified
thread adapter), panel bearing thickness, rear-cover retention and actual anchor
qualification remain integration work. The existing working tray scene is
unchanged by this study. Outputs are `build/tfixx-anchor/cad-check.json`,
`placement.json` and `placement.svg`.

Placement acceptance: ten candidate positions; minimum outside-edge and
service-opening axis distances both 55 mm; minimum axis spacing 123.49 mm.
The conservative anchor volumes, proposed local rebates and backing reservations
clear all retained insert envelopes. No service void intersects the candidate
anchor envelopes after reserving the proposed backing. This is a placement
study, not a full moving-machine or structural qualification.

## M6 tray-anchor sourcing shortlist

Supplier candidates and enquiry leads are saved in [tray-anchor-sourcing.json](tray-anchor-sourcing.json).
Its `offers` use the existing `SupplierOffer` fields for later loading into
`SourcingPlan`; no selection is made and the candidate requirement is not yet
an installed engineering BOM item. Prices and final quantity are unknown.
Stock is unverified, not confirmed unavailable. PFEIFER M6 × 40 stainless
05.250.064.040 and COM Maleniec M6 × 40 galvanised k1110zn are documented
candidates; PCT and Cast-in Portugal remain enquiry leads without confirmed SKUs.

M6 is the preferred tray-fastener direction; the earlier T-FIXX M10 study
remains reference work rather than the selected design. Smaller threads do not
automatically permit smaller casting edge distances. Next, obtain a current
M6 drawing and check the direct-fastening layout before replacing the existing
clamps. Target one removable tray sheet, a continuous supported gasket, and
socket mouths flush with the proposed mounting surface. Bolt the sockets through the
mould during casting. Final screw seats, compression control and rear-service
access still need detailing; validate anchorage and tightening in a representative
mineral-cast sample before fabrication release.

## Rectangular direct-M6 tray proposal (current)

The current proposal supersedes the earlier tongue/tab and rear-notch drawing.
[tray-mounting-proposal.svg](tray-mounting-proposal.svg) and
[tray-mounting-proposal.json](tray-mounting-proposal.json) describe a single
approximately 1099.17 × 969.17 × 13 mm compact-laminate blank with a **905 × 580 mm
rectangular cutout**. There are no inward tabs, stepped cutouts or motor notch.
All ten M6 positions lie along the two long sides, at casting-local X
−380/−190/0/190/380 and Y ±315 mm. Minimum outside casting-edge distance is
85 mm. The rectangular opening extends to the rear casting edge to clear the
motor region without a separate notch in the sheet.

The proposed seat is now at the casting top (Z=0), replacing the earlier
4 mm recess. A 2 mm installed gasket puts the tray underside at +2 and top at
+15 mm; this clears the existing lifting-insert tops at Z=0. The proposed
casting mould needs flat support here instead of the old perimeter recess.
The rear inner gasket return lies just beyond the casting and still needs
continuous support tied into the independently supported frame. This detail
is not represented as completed hardware. The accepted working enclosure
scene still has its earlier tray construction; this review artifact is separate.

Use two simple rectangular gasket loops: 15 × 3 mm single-sided adhesive
closed-cell tape, nominally compressed to 2 mm. Net proposed route length is
7.07 m; **9 m** includes a provisional 15% cutting allowance, rounded up.
Coolant/oil compatibility, adhesive and compression force remain unselected.
Use sealed butt/scarf joints or fabricated corners; no overlapping tape layers.
The adhesive locates the gasket; the screws supply compression.

Direct screw seats remain provisional Ø16 × 7 mm counterbores, Ø10.5 mm
sleeve holes, M6 × 25 screws, 1 mm washers and removable Ø10/Ø6.6 × 8 mm
sleeves. This leaves 6 mm laminate beneath the recess. Socket mouths are
flush with the casting top, not protruding. Each screw counterbore clears the
inner gasket band by 2 mm. A single ring sheet should be installed before the
upper machine assembly; removal over the assembled machine is unverified.

### Cast-in socket model

`machinekit.structural.CastInFixingSocket` replaces the box-only review
representation with a cylindrical socket, blind cavity, semantic M6 × 1 thread
and closed, flattened/waved crimped tail. Its portable component recipe and
mouth/tail connectors keep it reusable for subsequent assembly integration.
It references [PFEIFER 05.250.064.040](https://www.pfeifer.info/out/assets/PFEIFER_SOCKET-CRIMPED-END_PPEN.PDF):
M6 × 40 mm, insertion dimension **a=25** and thread dimension **e=6 mm**.
The earlier text claiming 16 mm threaded engagement was incorrect: the screw
inserts 16 mm; the source does not establish 16 mm of thread engagement.

Outside diameter 8 mm, tail width 11 mm, thickness 2.2 mm and wave 1.4 mm are
explicit nominal modeling assumptions. The supplier document does not dimension
them; this is not manufacturer CAD or a qualified anchorage design. Material
rendering uses the existing generic stainless planning material, not a claimed
certification of a particular grade. The source PDF SHA and dimensional basis
are saved with the supplier record in `tray-anchor-sourcing.json`.

The model checks a single solid, clear blind insertion cavity, closed tail and
preserved recipe dimensions. Native Materia scene encode/decode retains all
23 parts. Component recipe registration is included for future integration.

Regenerate the proposal with:

```sh
python machinekit/examples/flexspline-hobber/plan-tray-mounting.py
```

Compile/run `haxeon.tray-proposal.json` to create
`build/tray-mounting/tray-proposal.mtrg` and `anchor-detail.mtrg`.
The tray review is exploded: sheet/sleeves lifted 40 mm and gaskets lifted
15 mm; sockets stay at their proposed installed level. The anchor detail
scene shows one socket at its actual nominal size for close inspection.

`HobberTrayMountingStudy` / `haxeon.tray-mounting.json` screen conservative
24 × 24 × 40 mm anchor envelopes with 50 × 50 × 65 mm backing reservations
against service voids and retained inserts; output is `tray-mounting-screen.json`.
`HobberTrayClearanceStudy` / `haxeon.tray-clearance.json` separately screen
the rectangular sheet against the existing machine at travel endpoints; output
is `tray-clearance-screen.json`. Neither is a continuous swept clearance,
complete enclosure acceptance, leak test or mineral-cast strength qualification.

Current revision validation: rectangular sheet clears the existing machine in
24 X/Y/Z endpoint and three-A-position samples, with 14 unique exact posed
pairs checked. The ten conservative anchor envelopes clear retained inserts
and service voids with the proposed solid backing. Socket cavity/closed-tail,
portable recipe and 23-part scene round trips pass. These samples are not a
continuous sweep or full enclosure acceptance.

### Tray proposal BOM and sourcing exports

[tray-bom.json](tray-bom.json) is the full known tray-proposal requirement list,
regenerated with the geometry proposal so quantities stay aligned. It contains
one sheet, ten sockets, ten M6 × 25 screws, ten washers, ten compression spacers
and nine metres of gasket tape (7.07 m net plus allowance, rounded up).
Quantities are engineering requirements, not supplier pack counts.

`HobberTrayBom` loads these requirements through the existing `Bom` and
`SourcingPlan` classes. Compile/run `haxeon.tray-bom.json` to export
`build/tray-mounting/tray-bom.csv` and `tray-bom-sourced.json` with units,
review status, candidate SKUs, URLs and supplier notes. It checks unique item
identities, hardware quantities, gasket quantity and supplier-to-BOM mappings.
The sourced JSON also preserves supplier enquiry leads and items still needing
design or quantities: rear gasket support/fasteners, joint/edge treatment and
mould-positioning hardware. No artificial zero-quantity purchased lines are
used for these unresolved requirements.

The spacer candidate is Ettinger **007.66.089**, stainless AISI 303,
Ø10 outside / Ø6.4 bore / length 8 mm. The model still uses nominal Ø6.6 bore;
compatibility and length tolerance must be reviewed before selection. Its
manufacturer datasheet is saved as a supplier reference. Screws, washers and
gasket grade still need actual supplier selections. No supplier is selected
and prices/stock are unverified. This replacement-design BOM stays separate
from the accepted six-piece enclosure BOM to avoid counting both designs.
