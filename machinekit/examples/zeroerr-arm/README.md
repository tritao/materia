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
