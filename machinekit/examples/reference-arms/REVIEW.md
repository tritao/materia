# Robot model implementation and design review

The subsequent [unchanged supplier evaluation](../zeroerr-arm/ORIGINAL-EVALUATION.md)
separates the original 110T/70T assembly from our enlarged-wrist prototype.
The prototype results below must not be generalized to the supplier architecture.
The original now reaches 80/82 targets and 17/23 continuously checked transfers
after all nine base searches; the unrestricted modified control reaches 26/82
and 5/23. Its original actual-tool load screen still leaves shoulder sizing open.
A subsequent [selected-base refinement](../zeroerr-arm/ORIGINAL-REFINEMENT.md)
recovers four vise transfers, reaching 21/23 with additional waypoint planning.
All 17 prior routes are freshly checked; the two outfeed routes remain unproven.

The agreed software/model study is implemented. This is a prototype review,
not a manufacturing, procurement, hardware-performance or complete tending-cell
release. Negative results below are retained as engineering findings.

## Acceptance evidence

| Requirement | Current evidence | Result and scope |
| --- | --- | --- |
| Official model acquisition and provenance | `sources.json`, `source-lock.json`, `prepare.py` | Pinned official sources; downloaded assets remain outside Git. FAIRINO redistribution permission remains unresolved. |
| UR5e/FR5 and available ranges | Per-model `build/*/manifest.json`, `.scene`, `.mtrg`, `validation.json` | 22 models: 14 UR, seven standard FAIRINO, FR5-WML. Two malformed official FR3 long-reach URDFs are rejected explicitly. |
| FK, IK branches, limits and Cartesian movement | Final native range run; `ReferenceArmsPreview.hx`, `ReferenceCartesianReview.hx`, `ComparisonReview.hx` | 858 source-FK/IK configurations; 21-point common tool task for all three comparison arms. Numerical fallback and planning assumptions are recorded. |
| Application use | `app/tests/reference-arms/ReferenceArmsApplicationTests.hx`, `build/application-validation.json` | Reference source selection, 200 simulator steps each for UR5e/FR5 playback, all three comparison mounting jobs and all three registered-cell imports pass. |
| Shared-scale comparison | `comparison.project.json`, `build/comparison/*.mtrg`, preview PNGs | Our arm, UR5e and FR5 coexist; identical physical tool, common base-relative task, floor/wall/ceiling mounting. 82 shared geometries, 137 instances, 18 moving joints. |
| Straight links and adapters | `zeroerr-arm/straight-link-config.json`, prepared STEP snapshot and native audits | 70 x 4 x 206 mm forearm tube, short structural adapters, retained supplier ends, fastening and separate service covers. Existing straight upper arm and 425/401 mm axes retained. Tapered baseline remains separate. |
| Geometry and connectivity | `zeroerr-arm/build/straight/exported-parts-audit.json` and native preview checks | 120 valid arm solids; 31 new STEP parts independently reimported; structural pieces unite into one valid connected solid; protected ends retained; 41.998 mm tested duct; zero cover overlap. Nine-pose independent arm FK passes. |
| Mass, stiffness and connections | `zeroerr-arm/build/straight/structure-report.json` | 25.477 kg estimated arm, +0.199 kg. Preliminary assumed-allowable tube/fastener screens pass. Ideal tube stiffness excludes connection and whole-arm compliance. |
| Workspace and tending loads | Straight sizing/structure reports; `build/cell-registration/registered-load-report.json` | Benchmark and actual-tool configuration screens complete. Shoulder release fails the stated sizing assumptions; no adequate full-target drive selected. |
| Discrete and continuous motion clearance | Straight `motion-report.json`, `continuous-clearance.json` | All 60 exact CAD configurations observed: 55 clear, five colliding, one colliding pose also has an indeterminate pair. All 35 unique synthetic-cycle configurations clear. Continuous synthetic hull review passes within its stated pair policy. |
| Interference refinement and planning restrictions | `zeroerr-arm/CLEARANCE.md`, `clearance-improvement.json`, `planning-window-review.json` | Smaller OD83 x L24.5 covers; eleven conditional hull slices with distal fasteners. J3 −150/+153 and J4 ±154 degree simulation restrictions retain 54 clear observed poses and exclude all five collision witnesses. Non-cover CAD observations reused after exact STEP-record/tree equivalence; all cover pairs freshly checked at all 60 poses. |
| Fixture/TCP registration and transfer review | `fixtures.json`, `registration-review.json`, `registration-planned.json`, three `*-cell.mtrg` projects | Current CAD fixtures, physical dual tool, proposed tray/base layout, four jaw/holding states, source 23 transfers, continuous checks and bounded RRT. Full cycle remains unproven. No old Reach1300 certificate reused. |
| Prototype handoff | `zeroerr-arm/bench-request.json`, generated `prototype-bom.csv`, `interface-review.pdf`/`.svg`, `prototype-pack.json` | Draft BOM, dimensional interface review, missing supplier/manufacturing data and single-joint/coupler bench procedures. No contact, order or physical test performed. |
| Reproducibility and commits | Both example READMEs, `audit-deliverables.py`, `build/completion-audit.json` | Runnable generation/review commands, integrity/coverage audit and staged commits. Unrelated edits remain untouched. |

The final range run and importer tests passed on the current sources. The audit
checks current generated geometry, design/producer hashes, observed coverage,
prototype pack, application report, registered physical states and load inputs.
It is an artifact-consistency check; native checks and the application test must
be run as documented before auditing a fresh generation.

## Concrete engineering findings

- **Shoulder:** the separate full-workspace 5 kg benchmark is still beyond the
  current 110T candidate's design/sensitivity requirement. With the actual dual
  tool and two current blanks, 37 evaluated restricted prototype configurations give a
  conservative shoulder envelope of **166.01 Nm versus 87 Nm rated**, and
  nominal gravity reaches **102.49 Nm**. The selected base therefore also fails
  the nominal-gravity screen; the former unrestricted candidate gave 128.94 Nm
  under the conservative screen. This is
  not a thermal/RMS or executable-cycle qualification. Retain the supplier
  torque/speed request and resolve packaging, exact revision and duty before
  selecting a larger drive.
- **Cell transfers:** at the selected candidate bases, continuously checked
  routes cover **4/23 for our restricted arm, 10/23 for UR5e, 18/23 for FR5**.
  All nine proposed prototype bases were planned; the selected base is
  (145, −950, 1050) mm with 24/82 clear target poses. Our remaining
  18 endpoints and one additional route are unproven. The earlier unrestricted
  prototype study checked 5/23 routes; this restriction has a workspace cost.
  Other models have endpoints but unresolved routes.
  These are bounded-search results with conservative inventory, not proof of
  impossibility or a comparison of production robot performance. No complete
  tending mission is released.
- **Folded travel:** extreme elbow folds collide with base/shoulder housings.
  Smaller service covers recover only about 0.2–1.2 degrees at affected wrist
  hull boundaries. J3/J4 now carry conservative simulation restrictions from
  conditional slices; these do not guarantee all joint combinations clear.
  The +/-170 degree J4 diagnostic poses still collide and remain recorded.
  These restrictions are not commissioned physical joint stops. See the
  [interference review](../zeroerr-arm/CLEARANCE.md).
- **Tool/load:** estimated dual-tool mass is **2.781 kg**, excluding external
  air supply. Each current aluminium blank is **0.1944 kg**; two give **3.170 kg**
  total flange load. The old 1 kg point-tool assumption is not used for the
  registered-tool screen. CAD/catalog masses and inertias remain estimates.
- **Connection detail:** fits, bolt preload/thread stripping, local compliance,
  fatigue, cover retention, cable routing and manufacturing tolerances remain
  unqualified. The generic 50-4-M6 dual-tool plate is not a verified production
  adapter for ZeroErr. Drawings explicitly remain interface reviews.
- **Source gaps:** FR3-WML/WMS URDFs reference undeclared `tool_Link`; UR STEP
  access requires login; FAIRINO mesh redistribution terms are unresolved.
  FR5's nominal flange is 100 mm from the ROS wrist origin, while its mesh ends
  at 99 mm. Other FAIRINO nominal flange/STEP correspondence is unverified.
  Several source inertial frames/centres of mass are also unqualified.

These findings require further mechanical, layout, supplier or physical work
before hardware/cell release. Physical base calibration, mount/anchor design,
grip force/slip, insertion/release contact, moving-door interaction, harness
travel, safety controls and commissioned drive limits are not supplied by the
software model.

## Main review artifacts

Paths below are relative to `/home/joao/dev/materia`:

- `machinekit/examples/reference-arms/comparison.project.json`: selectable three-arm comparison.
- `machinekit/examples/reference-arms/build/comparison/floor-preview.png`: shared tool/task overview.
- `machinekit/examples/reference-arms/build/cell-registration/registered-cells.png`: registered candidate overview.
- `machinekit/examples/reference-arms/build/cell-registration/zeroerr-cell.mtrg`: our arm in the candidate cell.
- `machinekit/examples/zeroerr-arm/build/straight/zeroerr5-straight80.mtrg`: straight-arm project.
- `machinekit/examples/zeroerr-arm/build/straight/interface-review.pdf`: draft adapter/tube interface layout.
- `machinekit/examples/zeroerr-arm/build/straight/clearance-boundaries.png`: conditional collision map before/after.
- `machinekit/examples/reference-arms/build/completion-audit.json`: generated integrity/coverage record.

Open a project with `./app/run-built.sh --project=PATH`. Regeneration and all
review commands are in [README.md](README.md) and the
[ZeroErr README](../zeroerr-arm/README.md).

## Task commits

- `5118b0a0a`: official reference acquisition, scene import and FK/IK checks.
- `8c3c4b289`: application selection and Cartesian playback review.
- `f2bd1b8d4`: straight forearm/adapters, geometry and preliminary load audits.
- `c366aad91`: continuous synthetic hull review and checkpointed CAD probes.
- `7d6a3bbc3`: common physical tool/task and combined mounting scenes.
- `31c8accbe`: complete exact CAD review and fold exclusions.
- `744f65c06`: current tending fixture/tool export.
- `cfc5c1f34`: candidate cell registration, transfer planning and actual-tool loads.
- `8f04a91dc`: smaller covers, conditional interference map, simulation restrictions and verified cover-only CAD regression.
