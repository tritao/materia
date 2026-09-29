# AnimKit follow-ups

- Expose ozz's `IKAimJob` next to the two-bone IK (`ak_instance_set_ik`), for
  HumanKit look-at.
- GPU skinning. Skinning runs on the CPU and republishes positions and normals
  every frame, which is fine for a few low-poly characters but not a crowd.
- Morph targets and morph-weight channels (skipped with a warning today).
- Carry and pick-up clips for the bundled worker (`assets/quaternius`); HumanKit
  carries with IK over the walk clip because there are none.
