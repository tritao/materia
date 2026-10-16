# Robot comparison and straight-link engineering plan

Active goal: finish the agreed plan, committing coherent changes as work completes.

## Acceptance checklist

- [x] Reproducible official UR and FAIRINO description/mesh acquisition, pinned provenance and licence records; generated vendor assets excluded from Git.
- [x] UR5e and FR5 articulated scene exports with correct scale, axes, limits, base and flange frames.
- [ ] Existing FK/IK integration checked against independent URDF FK, multiple branches and joint limits; joint jogging and Cartesian trajectories.
- [ ] Shared comparison scene with actual-scale robots, common tool/task frames, switchable mounting and overview/close-up previews.
- [x] Extend the working importer to the available UR and FAIRINO ranges; report model-specific gaps honestly.
- [ ] Separate straight-link ZeroErr variant preserving 425/401 mm axis spacing and supplier interfaces, with structural adapters, fastening/cable access and separate covers.
- [ ] Valid connected solids, mass/inertia, stiffness/connection-load estimates and comparison against tapered baseline.
- [ ] Updated workspace/tending load reports; unresolved shoulder sizing clearly retained.
- [ ] Diagnostic and interpolated tending clearance review, with collisions and failed queries recorded and long work checkpointed.
- [ ] Actual fixture/TCP registration required for real-cell feasibility; no invented cell certificate.
- [ ] Draft prototype BOM, interface drawings, missing supplier data and single-joint bench procedure updated.
- [ ] Relevant commits, runnable commands, artifact paths and final completion audit.

No purchase, supplier messages or physical tests are authorized by this software/design task.
Existing unrelated edits remain untouched. Visual/kinematic simulation does not qualify hardware dynamics.

## Current findings

Official UR description source: UniversalRobots/Universal_Robots_ROS2_Description,
revision d4042c1f5123c86645cc5dbd650b4f461f697ee9.
Official FAIRINO source: FAIR-INNOVATION/frcobot_ros2,
revision fcf0c7f0d60d949d8a9a4238f929a44d07f60379.
FAIRINO fairino_description/package.xml has `TODO: License declaration`.
Keep its fetched descriptions/meshes local and generated, with explicit provenance;
redistribution permission is unresolved. UR mesh terms vary by model.

## Implemented import stage

- 22 exported articulated scenes: 14 UR models, seven standard FAIRINO models,
  and FR5-WML. All pass saved-scene reconstruction, independent source-link FK
  at 39 poses, and IK checks (858 configurations total).
- FR3-MT uses the existing numerical solver; six standard FAIRINO models each
  need one numerical fallback at a near-singular pose. Every fallback starts
  from a perturbed seed. Source angles are preserved.
- Continuous UR joints keep their source semantics in the scene. Validation
  uses an explicitly labelled +/-pi planning window for finite turn enumeration.
- FR3-WML/FR3-WMS imports fail because their official source URDFs reference an
  undeclared tool_Link. These are source gaps, not accepted scenes.
- Official files and binary streams are hashed, vendor assets stay ignored,
  and five focused importer tests pass. A shared-scale static preview exists.
- Application selection now exposes 22 source-project jobs. UR5e and FR5 load
  through the actual app source loader and each complete 200 simulator steps
  with observed motion and no fault. Playback is relative to the saved starting
  pose; the app test checks the no-jump initial coordinate.
- Both solve the same 20 mm Cartesian TCP line with a 150 mm flange extension:
  21 configurations each, checked limits/residuals/jumps and no sampled hull
  violations. Prebuilt playback and a common-target PNG are available. No
  continuous swept or jerk-limited trajectory claim is made.
- The official FAIRINO 12-model V6.0 STEP archive and FR5 mounting drawings are
  hash-pinned and fetched. FR5's flange is 100 mm along local +Z from its URDF
  wrist3 origin; its ROS mesh face is 1 mm short of that nominal face. UR STEP
  access requires login; official URDF geometry is available without it.
- Source mass/frame diagnostics identify several FAIRINO centres of mass outside
  their mesh bounds. Those source inertias remain unqualified.
- Still pending: combined scene with our arm, a physical common tool and actual
  task/fixture registration, wider STEP correspondence, straight-variant motion/continuous-clearance review,
  and final bench deliverable audit.

## Straight-link stage

- Separate prepared variant retains all module placements and 425/401 mm axes.
  The upper arm already has a straight central section and stays unchanged.
- New forearm: 70 x 4 x 206 mm tube, two protected-end adapters, twelve M5
  screws and one dowel per end, and two separate rounded service collars.
- 120 valid solids and independent nine-pose FK, including new adapters/covers.
  Native structural union and independent reimport of all 31 new STEP parts pass;
  41.998 mm axial test duct and cover checks find zero material overlap.
- 270-pose estimated mass/load study and preliminary tube/fastener screens pass
  their stated assumed allowables; mass increases 0.212 kg to 25.489 kg.
  Thread stripping, preload, fatigue, fit/slip and cover retention remain open.
- Draft BOM and dimensioned interface-review PDF/SVG generated; unpowered coupon
  review added to the existing unsent bench request. These are not released
  manufacturing drawings. Full-workspace shoulder sizing remains unresolved.
- Next: straight-variant diagnostic/intermediate and continuous hull clearance,
  combined comparison with our arm and physical common tool, actual task/fixture
  registration, and completion audit. Previous tapered clearance does not transfer.

## Motion review in progress

- Continuous straight-arm synthetic-cycle hull review passes: eight waypoints,
  9.332 s, 849 subdivision queries, 0.5 mm margin. Source artifact/config hashes
  are recorded. Adjacent-body articulation exclusions apply; no tool, fixtures,
  harness or full CAD swept proof is claimed.
- Deterministic diagnostic plus quintic intermediate set contains 60 unique
  configurations. Two exact-CAD shards are running with per-pose checkpoints;
  full collection and collision/exclusion review remain pending.
