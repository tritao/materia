# Original-arm route refinement and supplier review

The unchanged 110T/70T arm now has four additional continuously checked
transfers. Explicit entry waypoints recover the four staging-to-vise transfers
that the initial bounded RRT search did not find. The geometry, selected base
(145, −950, 900 mm), target frames, ±180 degree study window, fixtures, stock
inventory and clearance margins remain unchanged.

This is a follow-up to [the matched original evaluation](ORIGINAL-EVALUATION.md).
Its nine-base 17/23 comparison remains intact in separate reports. The refined
selected-base result uses additional planning work and must not be substituted
into that equal-budget comparison without explanation.

## What changed in the motion study

The roadmap uses previously observed transfer poses and checked route vertices,
plus explicit TCP waypoints in front of the opening: world Y = −350/−500/−650 mm and heights 100 mm below, level with,
or 100 mm above the vise approach. Both gripper channels retain their vise
approach orientation. Eighteen target frames produce 41 IK configurations
that clear all four open-door holding states.

A small roadmap connects the original transfer endpoints through those poses.
Each edge uses the existing continuous quintic checker. Cached edge and node
checks are scoped to the same door state and fixed physical inventory. A
maximum of 640 distinct continuous edge checks bounds the search; invalid nodes
are rejected first. A selected concatenated path is freshly checked in all four
holding states. Every retained baseline route also receives a fresh continuous
check. The normal 5 mm clearance, 1e-6 rad lowering reserve, standard articulation
exclusions and validated rigid finger contacts are preserved.

Recovered transfers are:

| Request index | Channel | Transfer |
| --- | --- | --- |
| 3 | A | Outside staging → vise-load approach |
| 6 | A | Outside staging → vise approach |
| 7 | B | Outside staging → vise-load approach |
| 13 | B | Outside staging → vise approach |

The selected-base coverage is now **21/23**, up from 17/23.
Each recovered route uses one reviewed intermediate configuration. The original direct paths
meet the wrist/door rail and forearm/front frame; the new path routes around
those obstructions. A collision-free waypoint alone does not establish a clear
connecting path; only paths with successful continuous proofs are counted.

The two outfeed transfers (indices 15 and 19) remain unresolved in this bounded
study. Their original endpoint choices require large shoulder/elbow/base
changes; the direct paths encounter the door/window or enclosure roof. An
additional paired IK-branch test keeps each clearance and approach TCP frame
fixed, checks all candidates against the four closed-door holding states and
tries compatible transfer/descent pairs. It did not produce a clear pair.
Each outfeed target had four statically clear clearance branches and four
approach branches. Twelve continuous segment checks per request tested nearby
pairs without finding a clear transfer/descent pair. This is not an exhaustive
branch/path search or proof of impossibility. No joint window was widened to bypass the
problem, and no target or tray was moved.

`../reference-arms/build/cell-registration/original-registration-refinement-planned.json`
retains the paths, original direct-path sample diagnostics, portal IK evidence,
branch candidates/failures, fresh continuous results and producer/source hashes.
`build/original-evaluation/refinement-review.json` collects checked coverage and
the refreshed configuration load screen. It evaluates 132 pose/path-vertex
records. Worst reviewed shoulder gravity remains 87.97 Nm nominal; the
conservative mass/acceleration/design envelope remains 142.48 Nm versus the
assumed 87 Nm rated. The extra vertices do not increase those sampled maxima.
This does not bound torque everywhere between vertices and is not a timed
trajectory, thermal/RMS calculation or complete production mission.

Two contact targets were already unproven in the initial 80/82 endpoint study.
The transfer requests cover clearance/approach motions, not insertion, release,
contact descents, gripper timing, door motion or an executable four-part cycle.
The original arm's physical travel and drive command signs also remain unconfirmed.

## Supplier evidence and the remaining hardware decision

The pinned archive's `eRobot 5.pdf` labels J1–J3 as eRob110T and J4–J6 as eRob70T.
The component STEP filenames contain wildcard ratios. This confirms the families
and geometry, without establishing six complete drive ordering codes.

The public [110T](https://www.zeroerr.cn/eRob/eRob110T.html) and
[70T](https://www.zeroerr.cn/eRob/eRob70T.html) pages were reviewed and cached
with retrieval timestamps and SHA-256. Their rated-torque tables match the
pinned V4 drawing tables used in the model. Their mass figures differ:

| Family | Pinned V4 brake-equipped mass | Public-page mass |
| --- | --- | --- |
| 110T | 4.47 kg | 4.64 kg |
| 70T | 1.16 kg | 1.24 kg |

Substituting just those six masses would add 0.75 kg, with every other assumption
held fixed. That is a sensitivity observation, not a verified mass correction:
the public page does not identify the exact reference-arm revision/options.
The model's mass basis remains unchanged. The pages label speed as peak output
speed; the pinned V4 drawings additionally state the 48 V/rated-torque condition.
Those labels alone do not establish a continuous torque-speed-temperature curve.

The 110T nominal-100 rated value remains 87 Nm. Higher gearbox average/peak
allowances do not establish continuous motor or thermal capability. Resolve
physical joint travel/signs, exact codes/options, revision-matched torque/duty
curves, the arm payload/CoG/inertia envelope, and moving-side masses/inertias
before changing the shoulder package or committing to hardware.

[original-supplier-request.json](original-supplier-request.json) is a concrete
unsent request for those data. It retains the original module families, records
the latest actual-tool load report hashes and leaves ordering codes null.
The earlier 80T bench request remains a separate prototype study.

## Reproduce this follow-up

After the original evaluation artifacts exist, run from the repository root:

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/reference-arms/haxeon.json
./machinekit/examples/reference-arms/run-built.sh --registration --original-reference --refine-routes
python3 machinekit/examples/reference-arms/registered-load-study.py --original-refinement
python3 machinekit/examples/reference-arms/registered-load-study.py --original-reference
python3 machinekit/examples/reference-arms/registered-load-study.py --unrestricted-modified-reference
python3 machinekit/examples/reference-arms/registered-load-study.py
python3 machinekit/examples/zeroerr-arm/original-supplier-review.py --archive=/path/to/pinned/5kg.zip --refresh
python3 machinekit/examples/reference-arms/review-original-refinement.py
python3 machinekit/examples/zeroerr-arm/original-evaluation.py
python3 machinekit/examples/reference-arms/audit-deliverables.py
```

The supplier reviewer defaults to `build/zeroerr-5kg.zip`; `--archive` can reuse
an existing download. Its SHA must match `reference-layout.json`. After a first
public-page snapshot, omit `--refresh` for an offline rerun. It verifies both
pinned V4 PDFs, extracts the assembly drawing, records current-page differences,
and writes the draft request. No supplier communication or order is performed.

Progress is saved after every route in
`../reference-arms/build/cell-registration/original-refinement-progress.json`.
The collector verifies current native producers, baseline identity, all 23
requests, endpoint/branch consistency, retained positive proofs, refreshed load
inputs and the supplier source snapshots. A passing audit establishes evidence
consistency and review coverage; physical qualification and a complete tending
cycle remain false.

Next, coordinate the staging and outfeed IK/grasp choices across the complete
transfer sequence, using confirmed physical travel before considering joint
unwrapping. Then review contact/insertion motions and actual stock/jaw states,
and time the resulting duty for torque/RMS/thermal assessment. Resolve the
supplier data in parallel before changing actuator envelopes.
