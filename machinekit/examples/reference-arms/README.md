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
convex approximations, not exact B-rep clearance evidence. Render colors currently use
the URDF material or a neutral fallback; embedded DAE textures are not reproduced.

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
Five importer tests cover source hash rejection, coordinate conventions, finite input,
safe mesh paths and generated wire-field mapping. Run them with the preparation venv:

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
