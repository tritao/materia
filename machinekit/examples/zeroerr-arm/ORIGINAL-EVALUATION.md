# Original supplier arm evaluation

The unchanged ZeroErr 5 kg STEP assembly is a stronger starting point for this
cell than our enlarged-wrist prototype. The reference has straight hollow links
already; our modifications changed the wrist package and axis offsets as well
as the forearm construction. The earlier modified-arm failures must not be
attributed to the supplier architecture.

This review keeps the original 110T J1–J3, 70T J4–J6, mounting transforms,
425/401 mm main-link spacing and terminal flange. All 127 source-derived solid
instances pass import/placement checks and independent forward kinematics at
nine poses. The CAD repair and exterior output-spigot partition described in
[README.md](README.md) remain; no new mechanical redesign is applied.

The wrist axis origins in the supplier zero frame show why this is more than
a cover comparison (millimetres). The modified wrist also reverses the
source-zero J4–J6 axis directions:

| Axis | Original XYZ | Modified XYZ |
| --- | --- | --- |
| J4 | −0.4, 0, 938.3 | 42.1, 0, 938.3 |
| J5 | 85.6, 0, 938.3 | 97.6, 0, 980.8 |
| J6 | 85.6, 0, 1024.3 | 140.1, 0, 1036.3 |

## Matched comparison

Both arms use the same fixtures, physical dual gripper, 82 TCP targets, 23
transfer requests, nine proposed base positions, jaw/payload states, collision
policy and bounded planner. A separate unrestricted modified-arm diagnostic
removes its simulation restrictions, isolating their effect from the geometry.
The normal modified project retains its restrictions. Vendor physical travel
and drive signs remain unconfirmed for the original arm; unbounded source
joints are given the existing ±180 degree study planning window.

The load benchmark is **5 kg total at the flange**, split into 4 kg payload and
1 kg tool, with a combined 150 mm CoG offset. Both arms use 180 degrees/s,
60 degrees/s², design factor 1.2 and a +25% arm-mass sensitivity case. These are
study assumptions. The registered-cell load screen instead uses the exported
2.781 kg dual tool plus up to two 0.1944 kg blanks.

| Check | Original 110T/70T | Modified straight 110T/80T |
| --- | --- | --- |
| Estimated arm mass | 23.512 kg | 25.477 kg |
| Exact sampled CAD configurations clear | 56/60 | 55/60 |
| Configurations with observed overlap | 4 | 5 |
| Configurations also containing an indeterminate CAD pair | 2, both already colliding | 1, already colliding |
| Same 270 load poses clear under bare-arm enclosing hulls | 227 | 226 |
| Same eight-waypoint synthetic continuous cycle | Pass, 995 queries | Pass, 849 queries |
| Nominal shoulder gravity maximum, hull-clear load poses | 119.75 Nm | 135.42 Nm |
| Nominal J4 gravity maximum, hull-clear load poses | 12.31 Nm | 13.29 Nm |
| Nominal J5 gravity maximum, hull-clear load poses | 11.06 Nm | 11.39 Nm |

The exact CAD checks include structural members and output spigots; the hull
checks include visual hull inventory with standard articulation exclusions.
They have different scopes. The 0.5 mm continuous synthetic review covers the
bare arm, excluding physical tool, fixtures and harness. Its 9.332 s duration
is a synthetic stop-to-stop cycle, not a production cycle-time measurement.

Original CAD collisions occur at the diagnostic J3 and J4 +/-170 degree
excursions from the ready pose. The wrist meets the base/shoulder at deep elbow
folds, or the forearm at extreme wrist folds. Two colliding elbow configurations
also have unresolved Boolean pairs; neither is counted clear. These artificial
endpoints are not supplier travel specifications. The extra sampled folded
collision found on the modified arm is clear on the original.

Both geometries have an analytic IK backend available, with numerical fallback.
A numerical-only control on the original still finds up to 80/82 clear targets,
versus 26/82 for the unrestricted modified arm. Therefore the observed target
coverage difference is not explained solely by analytic solver availability.
A failed finite search still does not prove a target impossible.

All nine proposed bases were planned for each ZeroErr variant:

| Arm | Selected base XYZ (mm) | Clear targets / 82 | Continuous transfers / 23 |
| --- | --- | --- | --- |
| Original supplier 110T/70T | 145, −950, 900 | 80 | 17 |
| Modified, unrestricted diagnostic | 145, −950, 750 | 26 | 5 |
| Modified, current restrictions | 145, −950, 1050 | 24 | 4 |

The original has endpoints for all 23 transfers at its selected base; six routes
remain unproven after the bounded search. This does not establish a complete
mission or production cycle time. UR5e/FR5's existing 10/23 and 18/23 results
use their earlier selected candidates, not this additional nine-base comparison.

The original actual-tool study evaluates 120 poses/path vertices across the
four holding states. With both blanks held, nominal shoulder gravity reaches
87.97 Nm and the conservative mass/acceleration/design envelope reaches
142.48 Nm, versus the assumed 87 Nm rated. J4 reaches 5.70 Nm nominal and
8.55 Nm conservative against the assumed 10 Nm rated. Thus this registered
configuration screen does not itself require an enlarged wrist; shoulder sizing
and the wider 5 kg/150 mm benchmark remain unresolved. The modified restricted
study evaluates 37 configurations, reaching 102.49/166.01 Nm at the shoulder;
the unrestricted modified study evaluates 40, reaching 77.38/128.84 Nm.
These are different reviewed configuration sets; their maxima are not a matched
trajectory/duty comparison.

Selected bases, checked transfer counts and actual-tool loads are recorded in `build/original-evaluation/evaluation.json` and the
isolated `original-registration-planned.json`,
`unrestricted-registration-planned.json` and registered-load reports under
`../reference-arms/build/cell-registration/`. Every original base is planned;
selection maximizes continuously checked transfers, then clear target coverage.

## What the load result means

The nominal-100 catalog screen assumes 87 Nm rated for 110T and 10 Nm for 70T.
Even after removing bare-arm hull-colliding samples, the original shoulder and
J4/J5 have static cases above those assumed ratings. Its lower estimated mass
and more compact wrist improve the screen, but do not resolve actuator sizing.

The STEP establishes exterior geometry and placement. It does not establish the
supplier arm's exact gearbox ratios, ordering codes, actual masses/inertias,
allowable payload/CoG curve or duty. Consequently these results flag questions
about our assumptions; they do not disprove the supplier's 5 kg rating. Resolve
those data before choosing a hardware BOM or enlarging the joints again.

The next design baseline should retain the supplier kinematic chain and compact
wrist, then check revision-matched drives and a verified tool adapter. Tube,
adapter and cover refinements can be considered without silently moving axes.
No complete tending cycle, insertion/release sequence or physical qualification
is established by the transfer checks.

## Reproduce

Prepare/export the original and current straight variant using [README.md](README.md),
and prepare the reference-arm fixtures/library using its
[README](../reference-arms/README.md). Generated assets stay in ignored `build/`.
From the repository root:

```sh
./machinekit/examples/zeroerr-arm/run-built.sh --engineering --export
python3 machinekit/examples/zeroerr-arm/engineering-study.py --config=machinekit/examples/zeroerr-arm/original-comparison-config.json
python3 machinekit/examples/zeroerr-arm/prepare-motion-review.py
python3 machinekit/examples/zeroerr-arm/original-evaluation.py --prepare-load-probes
./haxeon/scripts/haxeon build --project=machinekit/examples/reference-arms/haxeon.json
./machinekit/examples/reference-arms/run-built.sh --original-review
./machinekit/examples/reference-arms/run-built.sh --straight-clearance --original-reference
./machinekit/examples/reference-arms/run-built.sh --registration --original-reference
./machinekit/examples/reference-arms/run-built.sh --registration --original-reference --numeric-only
./machinekit/examples/reference-arms/run-built.sh --registration --unrestricted-modified-reference
python3 machinekit/examples/reference-arms/plan-original-candidates.py --jobs=2
./machinekit/examples/reference-arms/run-built.sh --registration-previews --original-reference
./machinekit/examples/reference-arms/run-built.sh --registration --unrestricted-modified-reference --plan-transfers --plan-candidate-count=9
```

Do not rebuild the reference-arm bytecode during the independent candidate run.
Each worker writes its own evidence before the merger checks all nine distinct
bases. `--adopt-first-progress` optionally adopts a completed first candidate
from a stopped sequential original review; default operation runs all nine.

Run these two exact-CAD commands independently; the explicit probe files are
already split, and original outputs are separate from modified outputs:

```sh
./machinekit/examples/zeroerr-arm/run-built.sh --motion-screen --pose-file="$PWD/machinekit/examples/zeroerr-arm/build/straight/probes-0.json" --shard=0 --shards=2
./machinekit/examples/zeroerr-arm/run-built.sh --motion-screen --pose-file="$PWD/machinekit/examples/zeroerr-arm/build/straight/probes-1.json" --shard=1 --shards=2
python3 machinekit/examples/zeroerr-arm/review-motion.py --original \
  --required machinekit/examples/zeroerr-arm/build/straight/required-probes.json \
  --output machinekit/examples/zeroerr-arm/build/original-evaluation/motion-report.json \
  machinekit/examples/zeroerr-arm/build/motion-screen-custom-0.json \
  machinekit/examples/zeroerr-arm/build/motion-screen-custom-1.json
python3 machinekit/examples/reference-arms/registered-load-study.py --original-reference
python3 machinekit/examples/reference-arms/registered-load-study.py --unrestricted-modified-reference
python3 machinekit/examples/reference-arms/registered-load-study.py
./haxeon/scripts/haxeon run --project=app/tests/reference-arms/haxeon.json -- --original-reference
python3 machinekit/examples/reference-arms/render-registered-cells.py --original-reference
python3 machinekit/examples/zeroerr-arm/original-evaluation.py
python3 machinekit/examples/reference-arms/audit-deliverables.py
```

`build/zeroerr5.mtrg` is the unchanged arm;
`../reference-arms/build/cell-registration/zeroerr-original-cell.mtrg` is its
selected static cell review. Open either with `./app/run-built.sh --project=PATH`.
The generated `original-comparison.png` shows original, unrestricted modified
and restricted modified candidates at a shared scale. App validation checks
both original imports, six passive joints and absence of modified restrictions.
The evaluation audit binds the geometry, checkpoints, independent candidate
reports, load inputs and application evidence; passing it validates evidence
consistency and coverage, not hardware suitability.
