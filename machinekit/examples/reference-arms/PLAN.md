# Robot comparison and straight-link engineering plan

Active goal: finish the agreed plan, committing coherent changes as work completes.

## Acceptance checklist

- [ ] Reproducible official UR and FAIRINO description/mesh acquisition, pinned provenance and licence records; generated vendor assets excluded from Git.
- [ ] UR5e and FR5 articulated scene exports with correct scale, axes, limits, base and flange frames.
- [ ] Existing FK/IK integration checked against independent URDF FK, multiple branches and joint limits; joint jogging and Cartesian trajectories.
- [ ] Shared comparison scene with actual-scale robots, common tool/task frames, switchable mounting and overview/close-up previews.
- [ ] Extend the working importer to the available UR and FAIRINO ranges; report model-specific gaps honestly.
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
- Still pending: actual application comparison/jogging workflow, Cartesian
  trajectories, common tool/task registration, STEP correspondence, straight-link
  CAD redesign and subsequent engineering/clearance/bench deliverables.
