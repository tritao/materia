# Imported reference arms

Current authority: [independent UR5e/FR5 manufacturer validation](COMMERCIAL-REFERENCE-VALIDATION.md) and the [frozen ZeroErr 5 kg reference](../zeroerr-arm/REFERENCE-RECONCILIATION.md). Geometry experiments and CNC tending are deferred/archived. Older load-screen and FR5-pending statements below are historical.

The active work is the [standalone arm-design baseline](ARM-DESIGN.md).
CNC tending and cell reconfiguration are deferred; their existing results are
application-specific historical evidence. The new suite checks commercial
references before interpreting standalone ZeroErr diagnostics.

Official UR and FAIRINO robot descriptions converted to Materia's existing articulated
scene format. The pinned URDF defines imported transforms, axes, limits and declared
inertias; manufacturer evidence is checked separately. Those source values are not
a hardware qualification. There is no ROS runtime or hardware driver dependency.

Run from the repository root:

```sh
python3 -m venv machinekit/examples/reference-arms/.venv
machinekit/examples/reference-arms/.venv/bin/pip install -r machinekit/examples/reference-arms/requirements.txt
machinekit/examples/reference-arms/.venv/bin/python machinekit/examples/reference-arms/prepare.py
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json
```

The initial models are `ur5e` and `fairino5_v6`. To prepare all catalogued descriptions,
use `prepare.py --all`. It attempts every model, checkpoints `build/preparation-report.json`,
and returns nonzero if any source is unavailable or invalid. Pass model IDs after `--`
to the Haxe example to export/check other prepared descriptions.

Each model directory below `build/` contains the expanded `robot.urdf`, hashed binary
visual/collision mesh streams, a readable `manifest.json`, its generated typed wire
representation, a `<model>.scene`, and `validation.json` after successful checking.
`<model>.mtrg` is the prebuilt extension the application opens; `.scene` is also kept
as the binary validation export. `<model>-review.mtrg` starts in a review pose.
The typed wire representation is derived from the Haxe consumer schema, not separately
maintained field IDs. Scene files use metres and import through the same assembly bridge
as our authored CAD robots. Massless fixed URDF frames are retained as transforms on the
visible links rather than invented physical bodies.

The checker round-trips the saved scene, compares all source link poses against an
independent matrix FK oracle at zero, six single-axis jogs and 32 sampled configurations,
then checks IK candidates against compiled FK and limits. Multiple returned candidates
can be different physical turns of the same geometric branch. A missing EAIK solution
is checked with the existing numerical solver from a perturbed seed and counted explicitly.
This initial checker does not yet establish trajectory clearance or dynamic performance.

## Source and engineering boundaries

`sources.json` pins official repository revisions; the UR archive also has a pinned
SHA-256. `source-lock.json` pins the individual source files of prepared models.
Prepared manifests record input hashes and binary stream hashes; the consumer
rejects changed mesh streams or URDF. Generated vendor assets are ignored by Git.
UR's source notices distinguish BSD-licensed models from newer models with Graphical
Documentation terms. FAIRINO's description package currently declares `TODO: License
declaration`; redistribution permission remains unresolved and its assets are not bundled.

Visual meshes and source collision meshes are prepared separately. Imported collision
pieces currently use enclosing 26-DOPs of source collision meshes. These are conservative
convex approximations, not exact B-rep clearance evidence. Visual material regions
retain URDF colors (including named materials), or embedded DAE base colors and
metallic/roughness values. Each region is a fixed visual part; the link's declared
mass/inertia and source collision pieces are assigned once. Explicit empty
`collisionHulls` marks the additional pieces as visual-only.

The prepared UR5e has 27 visual pieces, seven physical links, six moving joints,
and four source colors (including blue and black). FR5's raw URDF declares pale grey. The user-requested
`fairino5-appearance.json` overrides it with white housings, coral-red annular
cover trim and a metallic flange, approximating the supplied product photo.
The profile clips paint boundaries across the existing triangle surfaces in
link-local coordinates, following the existing polygonal cover rim for the inner paint edge and using
128-sided clipping for the outer radius. Aluminium covers the entire terminal
head link up to its existing part seam. All 24 existing cover screw heads (four per
ring) keep a metallic finish, using screw locations measured from the pinned mesh.
The screw finish follows whole source triangles rather than adding screw geometry.
New vertices stay within the original
triangles; surface area, volume and bounds are preserved. It produces 17 visual pieces while preserving all seven physical
links, six joints, source mass/inertia and collision shapes. Its basis and hash
are recorded in the manifest diagnostics; it is not claimed as vendor material
data. Neither model's current assets contain image textures.

STL visuals reconstruct angle-weighted normals across coincident vertices,
with a 40-degree crease threshold to keep cap edges and flange steps sharp.
Paint clipping interpolates these normals, so color boundaries do not create
shading seams. DAE visual normals are retained instead of overwritten. These
are visual-only changes: collision meshes, mass/inertia and joint data are
unchanged. The underlying vendor STL surface is retained; extreme close-up
silhouettes still have the source tessellation's resolution limit. Sources with embedded image textures are rejected explicitly until UV
and image import is implemented; they are not silently converted to flat grey.
Regenerate with `prepare.py ur5e fairino5_v6`, then export with `--cartesian`.
The app integration check `--commercial-only` verifies colors, mass and playback
without running the archived cell-comparison cases.

Source mass/inertia is preserved explicitly, including inertial-frame rotation. Exterior
mesh volume is retained only as geometric metadata; its placeholder density must not be
used for material or load sizing. The imported URDF effort values are not verified drive
torque curves, and there are no invented motors or actuator capabilities.

UR uses the vendor `tool0` frame. FAIRINO descriptions end at the vendor wrist3
origin. For FR5, `interfaces.json` maps that origin to the nominal flange with a
100 mm local +Z offset, checked against the official overall drawing and V6.0 STEP.
The FR5 ROS mesh ends at 99 mm, 1 mm short of the nominal face; that discrepancy is
reported and retained. Other FAIRINO flange mappings still require model-specific review.
Source rounded angles are preserved without snapping them to perfect right angles.
The importer also reports source centres of mass outside visual mesh bounds by more
than 5 mm. Several FAIRINO source links trigger this diagnostic; their inertias must
not be treated as qualified dynamic reference data.

For source continuous joints, the checker uses an explicit +/-pi planning window for
finite IK turn enumeration; the saved scene retains the continuous source joint.
This window is not a mechanical limit. FR3-MT uses the existing numerical backend
because its source geometry has no EAIK decomposition. Numerical fallback does not
promise complete branches or persistent geometric branch labels.

Initial range verification exported 22 scenes: 14 UR models, seven standard FAIRINO
models and FR5-WML. All passed 39 source-pose checks each (858 total). FR3-WML and
FR3-WMS source URDFs reference an undeclared `tool_Link` and are rejected explicitly.
Importer tests cover source hash rejection, coordinate conventions, finite input,
safe mesh paths, material regions and generated wire-field mapping. Run them with the preparation venv:

```sh
python -m unittest discover -s machinekit/examples/reference-arms -p 'test_*.py'
```

Optional static preview (requires NumPy and Matplotlib):

```sh
python3 machinekit/examples/reference-arms/render-preview.py
```

This produces `build/reference-preview.png`; it compares scale in illustrative
configurations, not identical TCP poses or task performance.

## Application model selection and Cartesian review

After preparing the models, open **UR and FAIRINO reference arms** under Robotics &
handling on the application's Start page. Its 22 jobs switch robots at the same base
mount through the existing job selector. The project declares every generated mesh
stream as a cache input. Preparation also generates an absolute asset-location module
under ignored `build/assets/`; rerun preparation after moving the checkout.

```sh
./app/run-built.sh --project=machinekit/examples/reference-arms/materia.project.json
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json -- --cartesian
python3 machinekit/examples/reference-arms/render-preview.py --cartesian
./app/run-built.sh --project=machinekit/examples/reference-arms/build/ur5e/ur5e-cartesian.mtrg
```

The Cartesian review uses the same synthetic downward-facing TCP target for UR5e and
FR5, a 150 mm extension from each mapped flange, and a 20 mm straight line sampled
every millimetre. Existing EAIK/DLS and `PathRequest` solve it, with explicit residual,
joint-limit and jump checks. Source collision hulls are checked at all 21 configurations;
the initial review has no observed violations. This is sampled clearance with the
existing articulation pair policy, not a continuous swept proof, tool collision check
or registration to a real tending cell.

Press Play in the Cartesian prebuilt artifact to run a slow forward/return illustration.
Its joint keys are relative to the saved starting pose, as the application runtime
requires. Linear key playback is not a jerk-limited hardware trajectory. The orange
line in `build/cartesian-preview.png` marks the common TCP extension; it is not a
physical gripper or collision body.

The prepared-asset application test exercises source job selection, prebuilt motion
loading, and 200 actual simulator steps for each arm:

```sh
./haxeon/scripts/haxeon run --project=app/tests/reference-arms/haxeon.json
```

It writes `build/application-validation.json` with observed joint motion and tracking
error. These are simulator observations with source mass/inertia, not physical-arm
performance results. Run `python3 machinekit/examples/reference-arms/prepare-cad.py`
to fetch the pinned 12-model FAIRINO STEP archive and FR5 drawings into `build/cad/`.
UR's current STEP download page requires login; its STEP archive has not been fetched.

The acceptance checklist is in [PLAN.md](PLAN.md); consolidated evidence and
engineering findings are in [REVIEW.md](REVIEW.md).

## Our arm alongside UR5e and FR5

Prepare and export the straight ZeroErr variant using its README, then run:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json -- --comparison
python3 machinekit/examples/reference-arms/render-comparison.py
python3 machinekit/examples/reference-arms/render-comparison.py --closeup
./app/run-built.sh --project=machinekit/examples/reference-arms/comparison.project.json
```

The **Robot arm comparison** Start-page example opens the same source project.
Its job selector switches floor, wall and ceiling mounting. Portable prebuilt
artifacts are `build/comparison/floor.mtrg`, `wall.mtrg` and `ceiling.mtrg`.
All three robots coexist in each scene, separated by 1.2 m along display X.
That spacing is a viewing arrangement, not a registered cell or clearance study.
The panel previews remove display spacing and retain identical scale/axes.
The millimetre ZeroErr meshes, connectors, poses, volume and unit-density inertia
are converted to metres; declared physical masses/inertias remain SI.

Each robot carries the same 40 x 40 x 150 mm solid tool envelope, with a TCP
150 mm along its mapped flange +Z. This is a physical mesh/collision body with
an assumed uniform 1 kg mass and 75 mm CoG; its mounting hardware is unspecified.
FR5 uses the drawing-based nominal flange frame; its source visual mesh remains
1 mm short of that face. The envelope is not a vendor gripper, fabricated
adapter or the 4 kg workpiece. The existing sizing study's combined 5 kg/150 mm
CoG load assumption remains separate and is not silently replaced by this tool.

Existing EAIK/DLS solves a common base-relative downward TCP line from
(0.45, 0.15, 0.45) to (0.47, 0.15, 0.45) m, in 21 samples for each arm.
The task rotates with wall/ceiling mounting. Every sample is checked for FK
residual, joint limits and discontinuous branch jumps, and against the source
collision hulls plus tool envelope. The generated `*-task.json` records any
violations and numerical fallback use. Our uncommissioned joints use an
explicit +/-pi study planning window; this does not add physical stops.

`build/comparison/validation.json` records saved binary reconstruction, all
occurrence poses/rotations at both task endpoints, independent mounting/TCP
checks, mesh scale and declared mass checks. Each scene contains 82 shared
geometries, 137 instances and 18 moving joints. Our six joints remain passive;
these are static comparison views, without a powered prototype mission.
The separate UR5e/FR5 Cartesian playback remains available as described above.

Source task fingerprints reject stale prepared tasks. The ZeroErr export also
binds its artifact to its prepared layout and exporter; changed design inputs
require preparation/export again. The application project declares mesh streams,
task records and ZeroErr source/provenance files as cache inputs.
Application integration tests load all three source jobs and all three prebuilt
artifacts, verify all 137 instances/18 joints and preserve our passive joints.
No dynamic benchmark, manufacturing release or physical cell qualification is
inferred from these visual/kinematic comparisons.

## Current tending fixture inputs

Export the current mill and the existing tending dual-gripper geometry separately:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.fixtures.json
```

`build/cell-registration/mill.mtrg` contains the current loaded reference mill at
its loading-axis positions, with door and jaw open. `tool.mtrg` contains the
physical dual gripper and its modelled air supply; the supply is an external
service, not carried tool mass. `fixtures.json` records both artifact hashes,
the producing bytecode hash, stock dimensions, vise load and stock frames, and
both gripper TCP frames. Saved-frame positions are checked after decoding.
The tool mount is recorded in both MachineKit's +Y-normal convention and the
+Z-normal convention used by the imported arms; the conversion checks that the
physical normal is preserved.

This export neither reads nor regenerates the generic arm's reach certificate.
The tool uses the existing generic 50-4-M6 mounting plate; a revision-matched
ZeroErr adapter remains unverified. The candidate study below registers arm bases, proposed trays and holding
states and performs fresh transfer checks. It does not establish a complete
feasible tending cycle. The combined comparison's simple
tool envelope does not substitute for this dual-gripper geometry.

## Candidate tending registration and transfer review

The registration study uses current native mill, tray, table and dual-gripper
geometry, with explicit candidate dimensions in `tending-registration-config.json`.
It does not reuse the old Reach1300 arm poses or certificate. Generate in order:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.fixtures.json
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json -- --registration
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json -- --registration --plan-transfers
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json -- --registration --plan-transfers --registration-model=zeroerr --plan-candidate-count=9
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json -- --registration-properties
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json -- --registration-previews
python3 machinekit/examples/reference-arms/registered-load-study.py
python3 machinekit/examples/reference-arms/render-registered-cells.py
./app/run-built.sh --project=machinekit/examples/reference-arms/build/cell-registration/zeroerr-cell.mtrg
```

The scan considers nine front-mounted bases (three setbacks and three heights),
82 TCP targets per model, square-stock grasp rolls, and the production plan's
23 transfer requests. Geometry and tool frames come from the actual exported
assemblies. A face-derived grasp uses the current stock collision hull and jaw
width/finger-length limits. Unbounded prototype joints receive only a study
planning window. IK residuals are checked; numerical sampling is not exhaustive
and a failed search is not proof of physical impossibility.

Transfer endpoints and routes are checked in four physical states: open empty
grippers, A loaded, B loaded, and both loaded. Closed jaws use the travel for the
actual blank width. Payload geometry is the actual stock hull. Other tray pockets
and the mill stock remain conservatively occupied; these inventories overbound
the actual four-part production sequence and can reject otherwise usable routes.
Door geometry follows each production transfer's open/closed requirement.

Candidate priority uses routes clear in all four sampled states, then target
coverage. By default the best sampled candidate is planned. The current restricted
prototype review instead plans all nine bases with `--plan-candidate-count=9`
and selects the most continuously checked routes, then target coverage. Use
`--registration-model=zeroerr` on the scan and planning commands when only our
arm changed: unchanged other-model results are retained only if their source,
fixture and configuration hashes still match. Final planning uses a continuous quintic clearance check,
with deterministic RRT-Connect fallback capped at 256 iterations. The resulting
piecewise stop-to-stop quintics are checked again in every holding state, with
5 mm normal clearance and a 1e-6 rad lowering reserve. Adjacent-body articulation
exclusions apply. The two intended finger/blank contacts are measured with signed
distance, then excluded from independent motion inflation because their offsets
on the same flange link are invariant. The report records these specific contacts
and their measured distances. All other tool, payload and fixture pairs remain
checked. This assumes an ideal rigid grasp; force closure, slip and flexibility
are not qualified.

Current results for the selected candidates:

| Arm | Base XYZ (mm) | Continuously checked transfers | Outcome |
| --- | --- | --- | --- |
| Our restricted straight ZeroErr arm | 145, -950, 1050 | 4 / 23 | 18 endpoints and one additional route remain unproven |
| UR5e | 145, -950, 900 | 10 / 23 | Endpoints found; remaining routes unproven |
| FAIRINO FR5 | 145, -950, 1050 | 18 / 23 | Endpoints found; remaining routes unproven |

Three ZeroErr routes and four FR5 routes required RRT repairs. The earlier
unrestricted prototype study checked 5/23 routes at a different base. The
conservative J3/J4 restrictions reduce the selected result to 4/23; smaller
covers do not establish an improvement in complete task feasibility. These counts are
results for this candidate study and conservative inventory, not robot rankings,
production cycle times or complete cell certificates. In particular, our arm's
arm/table and wrist/enclosure conflicts remain design issues. Contact insertion,
release, door motion during transfers and a complete executable production cycle
are not established by this transfer review.

`build/cell-registration/registration-review.json` retains every candidate,
endpoint failure and sampled witness. `registration-planned.json` contains the
selected bases, route waypoints, continuous proof results, bounded search failures
and rigid contact evidence. Progress files checkpoint each completed candidate.
`zeroerr-cell.mtrg`, `ur5e-cell.mtrg`, and `fairino5_v6-cell.mtrg` are static review
projects with current mill geometry, proposed trays, conservative stock inventory,
and physical open grippers attached to the arm. Frozen fixtures leave six moving
arm joints; our joints stay passive. The pedestal is a clearance envelope, not a
fabrication design. Saved physical TCP placement is checked independently.
The app integration test loads all three registered projects.

The registered load study replaces the earlier point-tool assumption with the
exported tool components' declared/CAD mass, CoG and inertia in each jaw state.
The external air supply is excluded. Estimated tool mass is 2.781 kg; each current
60 x 60 x 20 mm aluminium blank is 0.1944 kg, giving 3.170 kg with two blanks.
Mass aggregation is checked against the native tool estimate; tensor symmetry/PSD
and gravity/potential-gradient identities are checked. At 37 evaluated prototype
poses/path vertices, the heaviest state's nominal shoulder gravity reaches 102.49 Nm.
The +25% arm-mass, 60 degrees/s² zero-speed acceleration envelope and design factor
reach 166.01 Nm, about 1.91 times the current shoulder's 87 Nm catalog rating.
This candidate is worse on this screen than the earlier unrestricted base's
128.94 Nm result; the current shoulder is inadequate even at nominal gravity
in some reviewed configurations.
This is a configuration load screen, not an executed duty/RMS or thermal result.
The separate full-workspace 5 kg benchmark and unresolved shoulder sizing remain.

`tool-properties.json` and `registered-load-report.json` bind their source artifacts
and assumptions. `registered-cells.png` provides the common-scale overview. A
revision-matched tool adapter, physical installation, harness and hardware testing
remain prerequisites for a real-cell claim.

After the documented native and application checks, audit current deliverables:

```sh
python3 machinekit/examples/reference-arms/audit-deliverables.py
```

This verifies artifact integrity and expected review coverage and writes
`build/completion-audit.json`. It preserves explicit false flags for physical
qualification, complete tending-cycle feasibility and a full-target shoulder
selection; a passing artifact audit does not change those engineering findings.

## Original ZeroErr reference comparison

See the [unchanged supplier evaluation](../zeroerr-arm/ORIGINAL-EVALUATION.md)
for matched original/modified loads, all nine base searches, a numerical-only
IK control and reproduction commands. `--original-reference` and
`--unrestricted-modified-reference` use isolated reports and projects; the
standard three-arm comparison and modified planning restrictions remain intact.

The [original-arm follow-up](../zeroerr-arm/ORIGINAL-REFINEMENT.md) adds
`--refine-routes`, explicit entry waypoints, paired outfeed IK branch checks,
configuration-load refresh and a source-bound supplier review. Baseline reports
are preserved; the four recovered vise transfers use additional planning work.

## Manufacturer wrist electrical interfaces

`wrist-interfaces.json` records source-backed pinouts separately from the custom
cobot's provisional 5 A collar-mounted service module. `ReferenceRobot.wristInterfaces`
loads the hash-pinned catalog with the imported UR5e/FR5 model. No hardware revision
or current-rating edition is silently selected. Vendor mesh geometry is preserved.

```sh
# Attach the catalog to already-prepared UR5e/FR5 manifests; no mesh/FK changes.
python3 machinekit/examples/reference-arms/wrist_interfaces.py
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.wrist-interfaces.json
```

Normal `prepare.py` also prepares/stamps this catalog. The wrist checks have a
separate build directory and export review scenes and structured interface
sidecars under `build/wrist-interface-review/`. Normal reference exports also write
`<model>.wrist-interfaces.json`. Source pinout image URLs and SHA-256 values are
stored per connector; the normalized catalog and typed artifact are hashed.

| Profile | Physical connections | Electrical scope |
| --- | --- | --- |
| UR5e legacy | One M8 male, 8 contacts | 0/12/24 V, digital I/O, analog/RS485 multiplexing |
| UR5e revised | One M8 female, 8 contacts; different pin numbering | Same service categories, explicit revised pinout |
| FAIRINO FR5 manual end plate | M12 female 8-contact I/O plus a separate 5-contact RS485 connector | 24 V; one shared 1.5 A tool-board budget, no independently qualified transient overload |

UR's [connector change notice and developer specifications](https://www.universal-robots.com/it/developer/hardware-and-motion/electrical-interfaces-tool-connector/)
identify the first female UR5e serial as `20245501103`. Serial selection uses strings,
not a 32-bit integer. The [published female pinout](https://a.storyblok.com/f/169662/758x337/3548b38c03/female-tool-out-orig.png)
and [male pinout](https://a.storyblok.com/f/169662/679x337/557f9befc3/male-tool-out-new.png)
are distinct: for example RS485+ is pin 2 on the female and pin 1 on the male.
Adapter-cable numbering must follow its own documented mating interface.

**UR source conflict is retained:** the captured developer table lists UR5e
1 A single-pin / 1.5 A dual-pin typical current; the linked [SW5.25 UR5e manual](https://www.universal-robots.com/manuals/EN/PDF/SW5_25/user-manual-UR5e-PDF_online/710-965-00_UR5e_User_Manual_en_Global.pdf)
section 8.7.6 lists 0.6 A for both. Both state 2 A peak, maximum 1 second, maximum
10% duty and the typical-current limit on the 10-second average. They are separate
rating-evidence options, not inferred hardware revisions. The serial connector
cutover does not resolve the current-table conflict. Confirm the applicable limits
for a purchased robot before treating either option as hardware-qualified.

FAIRINO's [manual end-plate diagram](https://manual.fairino.support/latest/_images/0413.png)
shows AI0/AO0/DI0/DI1/power/DO0/DO1/GND on the 8-contact connector, and
485-A/485-B/power/unused/GND on the separate 5-contact connector. The 1.5 A tool
supply comes from the [FR5 product specification](https://www.fairino.us/collaborative-robot/fairino/fr5),
not the controller's separate power-supply table. The 5-contact connector's thread,
coding and sex are deliberately unqualified; the schematic alone does not establish
them. A/B naming is not silently equated with UR's +/- convention.

`ReferenceWristInterfaces.checkUse` requires a selected rating edition, power and
signal connector, supply mode and load envelope. It checks continuous/shared and
transient current, peak duration/duty, window-average consistency, stated capacitive
limit, RS485/analog exclusivity and digital outputs consumed by dual-pin power.
The 2.4 A simultaneous peak of two PGC-140-50s fails every recorded wrist power option.
This is a documented-interface compatibility check, not a hardware driver or wiring
commissioning test.

**Spatial qualification remains open.** Neither the pinout drawings nor the imported
visual descriptions establish a verified installed socket mating position/key
clocking for these hardware revisions. `matingFrame`, `parentLink` and
`placementSource` remain null; `requireMatingFrame` refuses to invent a connector
frame. No generic socket is added to or substituted into the manufacturer meshes.
These references cannot yet certify mating-cable access, bend clearance or wrist
motion around those sockets. Obtain/reconcile revision-matched CAD or drawings
before qualifying those positions.

### Connector geometry confirmed in FR5 STEP

A subsequent direct STEP inspection confirms that both connector patterns are
already part of the terminal wrist geometry: five contacts on one side and eight
on a perpendicular side. They are embedded in terminal solid 36 of the 37-solid
pinned FR5 V6.0 file, rather than separate connector components. Exact cylindrical
contact faces and source-file provenance are recorded in `wrist-geometry-findings.json`.

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.wrist-geometry.json
```

The inspection exports `build/wrist-geometry-inspection/fr5-terminal-wrist.step`
and `.mtrg`, the full supplier scene, raw mesh streams and cylindrical-face records.
This confirms the geometry exists. Registering its source STEP coordinates to the
imported ROS link and assigning electrical pin numbers, key clocking and a cable
mating plane remain distinct checks. No UR STEP is present in the prepared assets.

The FR5 port records now include typed `WristGeometryBinding` observations. These
identify STEP solid 36, insert faces 117 (five contacts) and 116 (eight contacts),
contact cylinder face indices, source SHA256 and parent URDF SHA256. Preparation
checks both source files; loading checks the parent URDF and catalog hash.

The insert face centres map nominally into `wrist3_link` at (0, -38, 80) mm
and (-38, 0, 80) mm. Convert STEP mm to metres, then translate by
(0.820, -0.050, -0.102) m, with no rotation. Housing axis and rear shoulder
support this nominal mapping; the ROS mesh has simplified socket caps and a
flange face 1 mm short of the STEP. This is not a full surface registration.
Observation frame Z points outward and Y follows wrist +Z; this does not identify
the mechanical key. `requireObservedGeometry` exposes the evidence separately
from `requireMatingFrame`, which still rejects automatic cable attachment.
Cable seating depth, connector key clocking and CAD contact-to-pin numbering
remain unqualified. Earlier cylinder-wall midpoints in the findings file are
now explicitly named as such, rather than being described as contact planes.

Cable qualification follow-up (2026-10-18): the DH EN.2403 catalogue, PDF page 70,
explicitly lists robot-cable option **14 = FAIRINO** for PGC-140-50. It is an
ordering selector, not a complete cable part number or dimensioned plug. The
manufacturer setup guide describes an 8-to-5-pin adapter for the AG series:
https://www.fairino.mx/en/cobots-knowledge-base/setting-up-a-dh-robotics-ag-gripper
The documented PGC option is saved as typed `WristCableEvidence` in the FR5
profile; unknown seating/key/cable dimensions stay null. No generic M12 plug
has been substituted. `cableGeometryGaps` reports missing evidence and
`requireCableGeometry` rejects placement until those items and a qualified
socket mating frame are present. This geometry gate does not replace electrical
load, pin-mapping or actual CAD clearance validation.

Run the independent supplier-face audit with:
```
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.wrist-cable.json
```
It imports the hash-pinned STEP through native OCCT, rechecks solid/face inventory
and insert locations, and compares each insert area with a radius-4 mm disc minus
the measured contact holes. Both face boundaries are circular (two outer arcs),
with no key notch represented on the insert face. The evidence is written to
`build/wrist-cable-qualification/report.json`. This establishes a limit of the CAD,
not the absence of a key on the real connector. No plug, routed cable, or clearance
pass is emitted without the missing supplier geometry. The exact supplier data
needed to finish is listed in `fairino-cable-request.txt`; nothing has been sent.

A two-gripper harness also needs a power decision: our conservative 1.2 A peak
per PGC totals 2.4 A against the documented shared 1.5 A FR5 wrist limit. Even
the EN.2403 catalogue's 1.0 A peak per PGC would total 2.0 A. A passive splitter
cannot support those simultaneous peaks from that supply. Either qualify
nonoverlapping operation, including the other gripper's load, or use a designed
external supply arrangement that does not backfeed the robot tool power.

Commercial plug layout candidate

`wrist-plug-candidate.json` records Binder **77 3429 0000 20005-0500**, a
five-contact M12x1 A-coded straight male cordset, as a layout candidate.
`prepare_wrist_plug.py` downloads and hash-checks the actual supplier STEP and
PDF into ignored `build/cad/binder-m12-candidate/`. These assets are not bundled
for redistribution. The original supplier geometry is retained, including its
4.3 mm family-model cable stub; the selected cordset's published diameter is
5.2 mm, with minimum fixed/moving bend radii of 26/52 mm. The candidate is also
available as typed cable evidence alongside the DH catalogue option.

```
python3 machinekit/examples/reference-arms/prepare_wrist_plug.py
python3 machinekit/examples/reference-arms/wrist_interfaces.py
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.wrist-plug.json
python3 machinekit/examples/reference-arms/render_wrist_plug.py
```

The candidate entry exports the isolated plug, three wrist/plug `.mtrg` scenes,
a combined nominal STEP, exact supplier surface observations, and
`build/wrist-plug-candidate/layout-report.json`. Plug source Z=0 is its leading
CAD face, not a verified seating datum. The study offsets -8/0/+8 mm relative to
the observed FR5 insert are explicit sensitivity assumptions. Clock zero follows
the existing wrist reference; 0/90/180/270-degree orientations are tested in the
moving mesh screen without claiming the real key permits any of them.

Exact static CAD checks compare the external body (source Z >= 15.8 mm) against
all 37 source robot solids at the original STEP assembly pose. The excluded
mating/coupling region cannot be judged for fit from our simplified socket CAD;
its whole-plug overlap is recorded separately. Moving checks use actual imported
ROS visual triangles over all prepared FK probes. Surface distance is not a
signed penetration/containment test or a swept-path certificate. No grippers,
custom adapter, hub or flexible cable are included in this candidate study.

This unshielded PVC cordset is a geometric candidate, not a selected RS485 robot
harness: twisted-pair impedance, shielding needs, torsion, OEM coding/sex, key
clocking, seating, fixed lead and physical continuity remain unresolved. Its
4 A connector contact rating does not increase the robot's 1.5 A supply. The
shielded right-angle Binder 77 3527 0000 50705-0500 is a future geometry comparison;
its public product page supplied a drawing but no direct STEP asset, and no
geometry has been substituted for it.

Recorded candidate result: the three static CAD cases have minimum external-body
clearances of 6.502 / 14.109 / 21.997 mm at assumed offsets -8 / 0 / +8 mm.
There is no external-body solid overlap in that static source pose. Whole-plug
mating-region overlap at -8 mm is separately reported and cannot qualify seating.

The moving screen contains 468 cases (39 source FK probes x 3 assumed depths x
4 clockings), with 7,956 visual-mesh pair queries. Probe 12 reports forearm
surface intersection at nominal/+8 mm placement; the -8 mm placement has only
about 1.45 mm surface separation in that pose. These are sensitivity results,
not a certified operating range. `review_wrist_plug.py` independently verifies a
strict triangle crossing between the actual plug body (excluding the short
supplier cable stub) and the source forearm visual mesh at nominal offset/clock.
Its segment/triangle routine checks analytic hit/miss examples, and source mesh
hashes and probe coordinates are checked before use.

```
python3 machinekit/examples/reference-arms/review_wrist_plug.py
```

This writes `independent-mesh-review.json` and `folded-pose-interference.png` next
to the native report. `fr5-plug-candidate.png` shows the nominal candidate with
the actual supplier terminal wrist. The straight plug remains a comparison
candidate; these results do not support selecting it for the complete motion.

Preferred wrist layout: right-angle connectors

The current FR5 `layoutSelection` prefers the right-angle five-pin wrist lead,
with study clock 180 degrees directing its cable toward wrist -Z / back along the arm.
The eight-pin I/O socket remains unused; a future I/O lead should follow the
same right-angle design preference after separate fit qualification. Selection
is typed design metadata, not an assertion that socket key clocking is known.

`wrist-right-angle-candidate.json` pins the manufacturer drawing and STEP supplied
by the Rockwell product page for **889D-E5AC-1**. The linked file is the shared
**889D-E5AC-2 / 889D-E5AC-*** connector-family model; cable length is not being
inferred from this layout model. Its body projects 31.75 mm along the mating axis.
The published 6.1 mm cable and 10-diameter bend rule imply a 61 mm minimum radius,
but shielding, twisted-pair impedance and robot torsion suitability remain open.
The published unshielded PVC cordset is a geometric comparison reference.

```
python3 machinekit/examples/reference-arms/prepare_wrist_plug.py
python3 machinekit/examples/reference-arms/wrist_interfaces.py
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.wrist-right-angle.json
python3 machinekit/examples/reference-arms/render_wrist_plug.py --right-angle
```

Supplier source geometry is retained unchanged in the ignored CAD directory.
The derived connector clips off the illustrated long cable, label and wire ends
at native Y=30.226 mm, preserving the whole connector and 5.08 mm original cable
stub. Native OCCT verifies the derived part remains one valid solid. A -90-degree
rotation about source Y maps native X into the study's outward Z and native Y
into the cable-exit Y. With the existing wrist observation frame, study Y then
points toward the tool flange before clocking. The selected 180-degree clock
reverses the exit toward the arm. This reference alignment does not establish the
mechanical key or physical seating datum.

The shared `WristPlugCandidate` study supports a configurable identifier, axial
cutoff, preferred clock, and depth/clock sensitivity cases. Right-angle outputs
are isolated in `build/wrist-right-angle-candidate/`; the straight comparison and
its known folded-pose interference remain available in their original directory.
The right-angle study rechecks all 37 static STEP solids and the same 39 source
FK probes. Static pairs whose bounding boxes prove at least 5 mm separation
record that conservative lower bound; closer pairs use exact BRep distance and,
where needed, solid intersection. Each pair identifies its distance method.
The unqualified mating zone is left unevaluated. No new flexible cable route,
grippers or hub are included.

The exterior clipping box follows the imported connector bounds, preserving the
sideways elbow and strain relief. A source-vertex containment guard checks that
the axial exclusion has not silently shortened the candidate. It can be run
without the full pose study:
```
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.wrist-right-angle.json -- --exterior-check-only
```
The resulting `exterior-envelope-check.json` records the retained extents.

Recorded right-angle result: the preferred study clock is **180 degrees**, exiting
back along the arm. At the assumed nominal axial placement, all 39 source FK
samples have positive external-geometry surface separation, minimum **4.062 mm**
to the forearm at probe 12. The initial clock-zero/tool-facing exit still crosses
the forearm in that pose. Clock choice therefore matters even with an elbow.
The +8 mm seating sensitivity case remains near contact at the preferred clock;
actual seating must be established before treating this as a fitted connector.
Static supplier-solid separation lower bounds at -8/0/+8 mm are approximately
6.002/14.002/22.002 mm. These are conservative bounds, not exact gap measurements.

`review_wrist_plug.py --right-angle` reviews the limiting nominal-placement pose
using the same imported supplier and ROS triangles, checks source mesh hashes,
and runs an independent strict triangle-crossing screen. No crossing is found
there; this is not a closed-volume containment or full-motion certification.
`limiting-nominal-pose.png` and `fr5-right-angle-candidate.png` show the result.
