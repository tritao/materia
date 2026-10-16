# ZeroErr 5 kg reference arm

This is an articulated CAD variant based on ZeroErr's eRobot 5 reference:
eRob110T at J1–J3, eRob70T at J4–J6, straight hollow links, the supplied elbow
adapters/dummy housing and terminal flange. The original solid geometry,
fasteners and cable fittings are retained. It is separate from the generic
UR-dimensioned `cobot-arm` example and is not installed in the tending cell.

`reference-layout.json` pins the supplier archive and every component STEP file
with SHA-256. It records placements extracted from the supplier AP214 assembly
and the exterior solids assigned to each housing and output flange. J1 and J2
have their output flanges upstream: their housings move downstream. The other
four joints have the usual downstream output flanges. Module detail solids are
attached to their respective housing/flange, rather than duplicating a complete
motor envelope on both sides of a joint.

The 110T STEP merges its projecting output spigot into the casing solid. The
preparation step separates that portion at the front casing datum (local z=0),
checked against the V4 drawing's 98 mm output diameter. The spigot follows the
output flange, including when the joint is mounted with that flange upstream.
Both Boolean differences between the original repaired body and the reunited
partition must contain no solids. Numerical volume-integration differences are
recorded separately. This exterior partition does not model bearing internals.

## Build and review

Run from the repository root:

```sh
python3 machinekit/examples/zeroerr-arm/prepare-reference.py
./haxeon/scripts/haxeon build --project=machinekit/examples/zeroerr-arm/haxeon.json
python3 machinekit/examples/zeroerr-arm/prepare-reference.py --repair
./haxeon/scripts/haxeon run --project=machinekit/examples/zeroerr-arm/haxeon.json -- --export
```

To reuse a previously downloaded archive, pass `--archive /path/to/5kg.zip` to
the preparation script. Downloads and generated files remain in ignored `build/`.
The script checks the archive and CAD hashes, and independently verifies every
tracked occurrence placement against the original assembly's transformation
graph. A changed supplier archive requires an explicit source review.

The supplied 110T housing contains an invalid CAD face orientation. The repair
step uses the OCCT installation configured by the build (or `--occt-prefix`) to
derive a valid housing in a separate file, retaining the original STEP. The arm
caches 48 unique source-derived solids for repeated joint instances and review passes. It rejects
repairs that change bounds by more than 0.02 mm or volume by more than 0.1%,
and records before/after measurements and hashes in `build/vendor/repairs.json`.
This is an import repair, not a mechanical redesign or a manufacturing approval.

The check validates every imported solid, compares zero-pose placements and
overall bounds with the original supplier assembly, and checks all six joints
against independent world-axis Rodrigues calculations at nine poses. The
off-axis probe checks wrist roll as well as member position. `--export` writes
`build/zeroerr5.scene` and `build/zeroerr5.mesh.json` in a bent review pose.
Optionally run `python3 machinekit/examples/zeroerr-arm/render-preview.py` with
numpy and matplotlib installed to generate `build/zeroerr5-preview.png`.

Pass `--clearance` for exact structural overlap observations at the supplier
zero pose and bent review pose. It writes `build/clearance-report.json` with all
positive volumes among the named structural members and flanges. It does not
validate threads, all detail-fastener fits or the entire motion range; positive
overlaps are observations requiring review, not collision-clear certification.

The current checks pass for 127 source-derived solid instances (48 unique
geometries), exact supplier zero placements/bounds and nine kinematic poses.
All 22 checked structural members, flanges and output spigots have zero positive
overlap at both the supplier zero pose and the bent review pose.

## Hardware boundary

The reference archive is dated 2026-05-22 and contains T-shaped modules. These
are not the newer cylindrical eRob80H100I V6 selected separately for a bench
study. The reference does not identify complete reducer/brake/encoder ordering
codes. Match the chosen production revision's mounting drawing before making
parts. The source link drawing specifies 7075-T6; preview materials currently
use the library's aluminium appearance/density as an exterior-only estimate.
The scene's CAD-derived masses do not represent complete actuator internals and
must not be used to qualify payload or drive sizing.

All six joints are explicitly passive. Vendor motor signs, ratios, physical
travel limits, motor/driver parameters and brake sequencing have not been
commissioned. There is no powered simulation mission or hardware-control path.
Cable exits and fittings are present, but flexible joint-crossing cable loops
are not. The imported output-flange grouping represents visible assembly motion,
not a bearing, seal or transmission internals model.

Source: [ZeroErr reference design downloads](https://www.zeroerr.cn/download/eRob.html).

## Actuator and application engineering

The CAD implementation is committed separately from hardware selection. The
candidate matrix in `engineering-config.json` requests nominal 100 ratios,
holding brakes, EtherCAT and HM multiturn feedback for all six axes. These are
review candidates, not complete supplier ordering codes. The reference STEP
revision has not been matched to the current purchasable assembly revision.
The earlier 80H100I V6 bench selection remains documented in `cobot-arm`.
For a bench intended to validate this T-module arm, use the selected T module
once sizing resolves whether the 70T wrist has sufficient continuous capacity.
An I-module bench can validate communications but cannot validate T-module fits.

The V4 drawings list brake-equipped masses of 4.47 kg (110T) and 1.16 kg (70T).
The product pages list 4.64 kg and 1.24 kg; manual Appendix 3 also contains mass, CoG
and inertia tables. Resolve the delivered revision before freezing mass/CoG
properties. The study pins the V4 drawings and runs a +25% structural/module
mass sensitivity. Do not substitute I-module mass for T-module mass.
The 70T rated torque rises from 7 to 10 Nm at ratio 80 and remains 10 Nm at
100 and 120; a higher ratio alone does not fix continuous-load overload.

After the standard build and native preparation, run:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/zeroerr-arm/haxeon.json -- --engineering
python3 machinekit/examples/zeroerr-arm/engineering-study.py
./haxeon/scripts/haxeon run --project=machinekit/examples/zeroerr-arm/haxeon.json -- --motion-screen
```

The study requires numpy. `build/engineering-input.json` exports zero-pose
centroids, volume integrals, joint ancestry and world axes from the actual
prepared geometry. Its layout hash rejects stale geometry. The Python study
uses catalog total module masses, distributing those masses among each module's
exterior solids in proportion to volume. This is an estimated internal mass
split and inertia distribution. Other fittings use assumed aluminium density;
links use 2810 kg/m³. Supplier centroid/inertia frame conventions and moving-side
mass splits need confirmation before an inverse-dynamics model is released.

The user target is the UR5e/FAIRINO FR5 class. Their published nominal payload
is 5 kg and maximum joint speed is 180 degrees/s. UR5e reach is 850 mm;
FR5 reach is 922 mm, with published repeatabilities of 0.03 and 0.02 mm
respectively. These are comparison requirements, not achieved performance.
Payload includes the tool and workpiece, so the study now uses 4 kg workpiece
plus 1 kg tool, rather than 5 kg plus a tool. The combined CoG at 150 mm outward
along the tool-face normal is a study assumption; both products' load envelopes
and application inertia must be considered. Published peak speed is not a promise
of simultaneous maximum speed/acceleration/payload everywhere in the workspace.
Benchmark source URLs and travel specifications are in `engineering-config.json`.

The speed target is 180 degrees/s. The 60 degrees/s² acceleration is retained
only as a screening assumption; no common UR/FAIRINO acceleration or cycle-time
rating is inferred. The design factor is 1.2.
The study checks gravity and a zero-speed simultaneous-acceleration envelope,
including motor rotor inertia reflected by `(nominal ratio + 1)²` per manual
section 12. It checks gravity against the numerical gradient of potential
energy and mass-matrix symmetry/positive semidefiniteness. It separately reports
the payload-only gravity moment and records each maximum's witness pose.

`build/sizing-report.json` covers 256 deterministic Halton samples, 12 one-axis
endpoint poses about zero and the two review poses. Samples can self-collide: this is a load exploration, not an
executable trajectory. Payload/tool intrinsic inertia, velocity-dependent
coupling, friction, efficiency, cable drag and thermal derating are omitted.
The speed comparison is separate from the acceleration calculation; it does
not claim a dynamic duty cycle at 180 degrees/s. A passing screen cannot qualify
hardware. An overload identifies a candidate or application requirement to
change before purchase.

`--motion-screen` checks the same 22 structural members at 38 deterministic
poses: zero, ready, 24 Halton poses in a synthetic +/-90 degree interval, and
12 one-axis endpoint excursions from ready. This interval is an exploration
assumption, not supplier travel limits. Fixed-body pair volumes are reused;
all moving pairs are still checked for positive-volume overlap at each pose.
The generated `build/motion-screen.json` records collision witnesses. It does
not establish swept clearance between samples, minimum separation or cable
clearance. Existing tending reach certificates do not cover this arm variant.

Before a tending-cell reach check, resolve the real base mounting transform,
tool face/TCP and fixtures, then generate new IK and collision witnesses for
each approach, grasp and retract waypoint. The zero-pose assembly placement
and the old generic arm's offsets are not a valid cell mounting calibration.
The reference has no powered mission or commissioned travel limits.

## Cable and bench preparation

Route a dedicated flexible crossing at each of J1–J6, with strain relief on
both stationary and rotating sides. Use the supplied hollow links and 18 mm
module bores where the assembled route permits. Choose the actual power,
EtherCAT and tool harness before allocating loop space: minimum bend radius,
torsion per metre, connector insertion clearance and usable rotation range
remain unknown. The modeled fittings alone do not establish a viable harness.
Manual section 6.6 requires slack and prohibits packing or structures that
press on the bore/encoder region. Verify each loop at both travel endpoints
and through the complete intended cycle; record snagging, rubbing and tension.
Physical cable travel can impose limits narrower than mechanical travel.

Prepare a single joint on a rigid fixture with the module's revision-specific
pilot and mounting drawing, an independently supported test lever/load catch,
mechanical travel stops and an enclosure. Do not reuse generic CobotJoint
bolt patterns. A 100 mm lever with a 1 kg test mass applies 0.981 Nm at horizontal;
measure the lever's own mass/CoG and include it. Confirm fixture fasteners,
bearing loads and mounting torque from the exact supplier revision before use.

Use a 48 V supply with verified regenerative-energy handling, the exact EtherCAT
ESI/firmware pairing, the manufacturer commissioning tool, temperature/current
logging and an independent position measurement. Log complete ordering code,
serial, revision, ratio convention, encoder directions, reference zero and
firmware. Verify protective stop and STO behavior for that device; the passive
CAD model supplies none of these hardware functions.

Bench acceptance sequence (planned; no physical tests have been performed):

1. With load supported and drive disabled, check wiring, EtherCAT identity and
   absolute encoder values across restart. Confirm positive command direction
   against a small measured displacement before mapping model signs.
2. Verify brake sequencing unloaded. Manual V3.44 section 7 specifies at least
   300 ms from disable to re-enable and at least 500 ms after enable before a
   motion command; confirm against the delivered drive's communications manual.
3. Start with a small supported lever, +/-15 degree sweep, 5 degrees/s,
   10 degrees/s² and a conservative current/torque limit approved for the
   fixture. Measure current, following error, encoder agreement and temperature.
4. Add the calibrated 1 kg/100 mm load. Measure static holds, reversals, lost
   motion with an independent gauge, encoder drift after restart and power-off
   holding with the catch engaged. Log the result rather than assuming encoder
   resolution equals mechanical accuracy. Stop under drive control before
   applying the holding brake; do not use the brake for routine dynamic stops.
5. Only after these checks, reproduce the application torque/speed/duty profile
   within supplier limits. Log temperature to steady state, supply regeneration,
   peak current, tracking error and loaded accuracy. Acceptance tolerances must
   come from the application and the delivered module's specification.

Procurement remains pending revision compatibility, actual payload/CoG/inertia,
load/duty sizing, bearing and brake limits, cable details, ESI/firmware and
comparable delivered quotes. No supplier has been contacted and no parts ordered.

Engineering sources: [110T product page](https://www.zeroerr.cn/eRob/eRob110T.html),
[70T product page](https://www.zeroerr.cn/eRob/eRob70T.html), and the versioned
V4 drawings/manual URLs and hashes in `engineering-config.json`.

The nominal-100 candidates provide a catalog 30 rpm (180 degrees/s) at rated
load and 48 V, with no speed reserve at the benchmark. The 120/160 alternatives
are slower than the target. Ratio 80 offers 37.5 rpm but slightly less shoulder
rated torque and less wrist start/stop peak torque. Compare torque and speed
together; do not increase ratios to solve load while silently losing benchmark
speed. Published UR/FR5 travel limits also require a much wider cable/motion
review than the current synthetic +/-90 degree screen.

## Screening result and configuration decision

With the current 5 kg total/150 mm CoG assumption, 270 poses give the following
sampled nominal gravity moments before the 1.2 design factor:

| Axis | Estimated gravity moment | Catalog rated torque | Decision |
| --- | --- | --- | --- |
| J2 | 120.02 Nm | 87 Nm | Shoulder candidate fails this screen |
| J3 | 48.53 Nm | 87 Nm | No overload identified by this screen |
| J4 | 12.21 Nm | 10 Nm | Wrist candidate fails this screen |
| J5 | 11.07 Nm | 10 Nm | Wrist candidate fails this screen |

The estimated arm mass is 23.512 kg. The +25% arm-mass sensitivity raises the
J2 estimate to 138.00 Nm. These are surrogate-mass estimates, not measured
joint demands. More decisively, the sampled tool/payload-only J4/J5 moments
are 11.59/10.91 Nm: their overload is present even before arm-mass uncertainty
or the design factor. These observations apply to the assumed 150 mm combined
CoG and sampled poses; they are not a proof that every 5 kg application fails.
Generate a visual comparison with
`python3 machinekit/examples/zeroerr-arm/plot-sizing.py` (matplotlib required).

Do not freeze the reference's 110T/70T BOM as a UR5e/FR5-equivalent arm. The
preferred next wrist bench candidate is **eRob80T nominal 100, brake + EtherCAT,
HM feedback requested**, subject to exact revision confirmation. Its product
page lists 31 Nm rated and 30 rpm, but it is a different mounting envelope;
no 80T replacement has been inserted into the 70T CAD or treated as a fit.
The previous 80H100I V6 bench reference remains valid only for its separate
communications/actuator study, not as a validation of these T-module mounts.

A larger 142T shoulder is not an automatic solution: its current product page
lists 178 Nm at nominal 100 but only 20 rpm (120 degrees/s). Nominal 80 lists
153 Nm at 25 rpm; nominal 50 lists 99 Nm at 40 rpm. These T-page speeds differ
from the generic manual table, so obtain the exact versioned torque/speed curve
before selecting it. Adding larger wrist modules also increases shoulder load
and changes offsets. The shoulder remains unresolved for the combined torque,
speed and packaging target. No ordering codes or hardware purchase are released.

The supplier review must resolve continuous torque versus speed/temperature,
static holding capability versus gearbox rated/average/peak definitions, mass
and centroid frames, moving-side mass, encoder/brake ordering options and the
revision-compatible mounting/ESI/firmware set. In particular, the gearbox's
average-load allowance and momentary overload are not substituted for its
rated continuous torque in this screen. The nominal-100 speed values are
rounded catalog entries; the physical ratio-plus-one convention needs checking
against the delivered motor speed curve before claiming a 180 degrees/s rating.

Additional sources: [80T](https://www.zeroerr.cn/eRob/eRob80T.html),
[142T](https://www.zeroerr.cn/eRob/eRob142T.html),
[UR5e](https://www.universal-robots.com/manuals/EN/HTML/SW10_6/Content/prod-usr-man/hardware/arm_e-Series/UR5e/H_g5_sections/appendix_g5/tech_spec_sheet.htm),
[FR5](https://www.fairino.com/FR/4.html).

The completed motion screen reports zero positive structural overlaps at all
38 sampled poses. Evaluating gravity at those same poses still gives J2/J4/J5
maxima of 114.31/12.17/10.53 Nm, respectively, before the design factor. The torque
concerns therefore also occur within this structurally checked pose set.
This is not clearance certification for the wider UR/FR5 travel ranges.

## 80T wrist engineering revision

`revised-layout.json` keeps the 5 kg reference's proximal arm and 425 mm upper-arm
axis spacing, and substitutes the supplier 10 kg forearm/wrist chain at the
common 110T elbow interface. J4–J6 are all 80T modules, with the matching supplier
terminal flange: retaining the complete chain avoids inventing a mixed 80T/70T
adapter. The 10 kg archive, every added STEP, original assembly placements and
the rigid alignment to our elbow are pinned and verified independently.

The forearm axis spacing remains 401 mm. Its supplier mounting ends are retained;
189 mm is removed from the longer reference link. The tapered sections cannot be
joined directly, so a 40 mm hollow loft joins the retained end portions at local
Y=-130 and Y=-170 mm. The result must be one valid connected solid with one inner
and one outer section wire. This is a new link design requiring manufacturing,
wall-thickness, stiffness and fatigue review, not an unchanged supplier part.
Factory mounting features remain in the retained portions. CAD-derived aluminium
mass is still an estimate.

The 80T STEP also merges its output spigot into the housing. Its T mounting foot
crosses the front datum outside the 71 mm pilot, so a whole-plane partition would
incorrectly rotate part of the mounting foot. The preparation selects the main
connected spigot within the pilot cylinder at local Z=0; all other source
material stays in the casing. Both halves must be valid single solids and the
reunited material set must match the source. No bearing internals are inferred.

Build the baseline example and prepare its vendor files first, then:

```sh
python3 machinekit/examples/zeroerr-arm/prepare-revised.py
./haxeon/scripts/haxeon build --project=machinekit/examples/zeroerr-arm/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/examples/zeroerr-arm/haxeon.json -- --revised --engineering --export
python3 machinekit/examples/zeroerr-arm/engineering-study.py --revised
python3 machinekit/examples/zeroerr-arm/plot-sizing.py --revised
python3 machinekit/examples/zeroerr-arm/render-preview.py --revised
```

Generated files are separate in `build/revised/`; the scene is
`zeroerr5-wrist80.scene`. Baseline CAD and reports remain available.
The revised arm has 90 source-derived/engineered solid instances and six passive
joints. Validity is checked once per shared geometry; independent nine-pose FK
checks include every output spigot. Revised zero placements are checked against
the tracked engineering layout, not falsely compared to the unchanged supplier
assembly's overall bounds. The original baseline still checks its supplier bounds.

Run `--revised --motion-screen` for 33 pose observations: zero/ready, twelve
Halton samples across a synthetic +/-170 degree range, twelve one-axis endpoint
excursions from ready, and seven tending-surrogate waypoint observations
(including repeated approach poses). These are exploration inputs, not hardware
limits. `--shard=0 --shards=2` and `--shard=1 --shards=2` split the same list
between independent processes, writing separate `motion-screen-0.json` and
`motion-screen-1.json`. Fixed-body volumes are reused within each process.
The wider exploration finds folded self-collisions; observations must be used
as exclusions rather than interpreted as blanket travel approval. The structural
check includes all six output spigots but does not certify every fastener or
joint-crossing cable.

`revised-engineering-config.json` pins the 80T V5 drawing: 31 Nm rated at nominal
100, catalog 30 rpm at 48 V, and 1.75 kg with brake. The reference STEP's exact
production revision remains unconfirmed; a shared envelope is not proof that
all revision-specific mounting/encoder options match. Bench acquisition must
resolve the delivered revision and complete code.

The wider 270-pose load screen estimates a 25.277 kg arm. Nominal J2 gravity
reaches about 134 Nm; J4/J5 reach about 13.3/11.4 Nm against their new 31 Nm
rated candidates. The wrist screen improves, while the shoulder load increases.
The +25% arm-mass case and design factor still prevent release of the existing
110T shoulder for the full-workspace target.

## Shoulder decision and tending surrogate

The individually downloaded, hash-pinned drawings now resolve the generic-table
ambiguity: 142T V4 and 142I V4 list 20 rpm at ratio 100. 170I V3, in both the AX
and FX drawing packages, lists 137 Nm at 40 rpm for ratio 50, or 206 Nm at 25 rpm
for ratio 80. The latter misses 180 degrees/s; the former lacks the required
full-workspace design/sensitivity torque reserve. Neither a larger T module nor
an I-package substitution is selected on the basis of the faster generic table.
The evidence URLs and hashes are in `revised-engineering-config.json`.

The current 110T100 shoulder remains an operating-envelope candidate for a
bounded tending prototype, not a frozen UR5e/FR5-equivalent BOM. The synthetic
cycle uses eight explicitly recorded joint-space waypoints, stopping at each,
with quintic interpolation limited by the study speed and acceleration inputs.
Its calculated duration is 9.33 s; its peak joint speed is only about 43.4 degrees/s.
It is not a comparable production cycle-time or maximum-speed test. No real
fixture, tool TCP, cell registration, IK or actual tending mission is claimed.

The study includes finite-difference Christoffel velocity coupling for this cycle
and checks its kinetic-energy identity. It retains the estimated exterior mass
distribution, point payload, and diagonal reflected motor-inertia approximation;
friction, drive efficiency, flexibility, thermal limits and tool intrinsic
inertia remain unmodeled. Sampled nominal shoulder RMS/peak torque is about
57.6/66.6 Nm; the heavier-arm case gives about 67.7/78.2 Nm. These numbers support
further prototype study of the current shoulder within this cycle. They do not
qualify thermal duty, prolonged extended holds, the entire workspace, or swept
clearance between collision samples. Confirm actual cycle poses and load inertia
before using them for hardware sizing.

`bench-request.json` is a concrete, unsent acquisition/fixture review request for
one 80T100 joint with brake, EtherCAT and HM feedback requested. It pins the V5
mounting drawing, identifies the retained supplier forearm end as the fixture
interface basis, records the 1 kg/100 mm load and low-speed startup procedure,
and lists the ordering/ESI/brake/regeneration confirmations required before
purchase. It is not a fabricated fixture, confirmed ordering code, sent inquiry
or performed bench test.

The revised forearm drawing specifies AL6061-T6. Its engineering density is
therefore overridden to 2700 kg/m³; the retained upper arm uses the earlier
7075-T6/2810 kg/m³ assumption. The forearm drawing hash and density override are
recorded in `revised-engineering-config.json`.

The overlap reporter now records per-pair OCCT failures as `indeterminatePairs`
and writes a checkpoint after every pose. Such failures are never counted as
clear. `--pose-file=/path/to/probes.json` accepts an explicit array of six-coordinate
poses for witness rechecks and writes `motion-screen-custom-<shard>.json`.
Scene export precedes the long motion screen when both are requested.
`review-motion.py` merges observation files conservatively and can require a
complete probe list with `--required`; missing observations fail collection.
Completed clear observations from this review's original fail-closed run logs
were retained, while positive/error witnesses were rerun with detailed pair
reporting. The generated `build/revised/motion-report.json` records source hashes
and the resulting exclusions; the original logs remain beside it.

The completed wider review covers 30 unique configurations (33 observations
including repeated approach waypoints): 27 have no observed structural overlap
and three folded configurations collide. One of those colliding configurations
also contains an indeterminate OCCT pair. The witness pairs include the wrist
against shoulder/base housings; none is silently exempted. All five unique
waypoints of the synthetic tending cycle have no observed structural overlap.
The interpolated load-study samples are not a swept collision certificate.

## Straight forearm prototype

The separate `--straight` variant retains the existing motor packages and
425/401 mm axis spacing. The upper arm already has a constant central section,
so it stays unchanged. The forearm replaces the tapered middle with a 206 mm
long, OD70/ID62 tube and two short structural adapters. Each adapter retains
30 mm of the supplier mounting end, transitions over 15 mm and inserts 25 mm
into the tube. Twelve radial M5 screws per end and one locating dowel are
modelled separately. Two rounded sliding collars cover the service fasteners.
Cover retention, thread fits and bolt preload still need detailed review.

After preparing the baseline and 80T revision above, run from the repository
root:

```sh
python3 machinekit/examples/zeroerr-arm/prepare-straight.py
./haxeon/scripts/haxeon build --project=machinekit/examples/zeroerr-arm/haxeon.json
./machinekit/examples/zeroerr-arm/run-built.sh --straight --engineering --export
python3 machinekit/examples/zeroerr-arm/engineering-study.py --straight
python3 machinekit/examples/zeroerr-arm/structure-study.py
python3 machinekit/examples/zeroerr-arm/audit-straight.py
python3 machinekit/examples/zeroerr-arm/prototype-pack.py
python3 machinekit/examples/zeroerr-arm/render-preview.py --straight
```

`build/straight/current.json` identifies an immutable prepared snapshot keyed by
source, design and generator hashes. Cache reuse verifies every generated file.
The original baseline and tapered revision remain separate. The current variant
contains 120 solids; the generator preserves the protected source ends and
checks that the three structural pieces unite into one valid connected solid.
The independent exported-part audit reimports all 31 new STEP parts, checks a
41.998 mm axial passage through the nominal 42 mm duct, and checks cover clearance
against the other derived parts. It does not validate the complete arm harness.

The 270-pose study estimates 25.489 kg total arm mass, 0.212 kg above the tapered
80T variant. Its conservative zero-speed load bounds give about 201.7 N and
89.5 Nm at the supplier mounting section. With the explicitly assumed material
and connection allowables, tube stress utilization is 0.280, bolt shear 0.234
and local tube bearing 0.712. These are preliminary screens, not qualification.
The ideal unperforated tube deflection estimate is 0.080 mm under the study
bounds; it excludes holes, adapter/fastener compliance and all actuator/link
compliance, and must not be presented as TCP accuracy. Thread stripping, preload,
slip, fatigue and manufacturing tolerances remain open. The shoulder drive still
fails the full-workspace target; the straight tube does not solve drive sizing.

Generated review artifacts in `build/straight/` include
`zeroerr5-straight80.mtrg` (application project), `.scene`, overview/close-up PNGs,
`sizing-report.json`, `structure-report.json`, `exported-parts-audit.json`,
`prototype-bom.csv`, `interface-review.pdf`/`.svg` and `prototype-pack.json`.
The drawings are dimensioned interface reviews with symbolic protected-end
regions, not machining releases. Prepared STEP files define the actual shapes.
`bench-request.json` includes an unpowered adapter coupon review before joint
bench testing; no test, purchase or supplier contact has occurred.

Exported terminal flanges include a `flangeZ` connector whose local +Z is the
outward tool-face normal. This is a semantic frame for comparison and TCP work;
all six joints remain passive, and vendor drive signs/travel are uncommissioned.
The straight variant still needs its own diagnostic/interpolated and continuous
clearance review; the tapered revision's collision observations do not transfer.
