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
