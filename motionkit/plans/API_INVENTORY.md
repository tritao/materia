# MotionKit C API inventory

Snapshot of `motionkit/native/include/motionkit.h` on 2026-11-04. This is a
source-reference inventory, not a removal approval. The scan covers MotionKit,
RobotKit, MachineKit, ProcessKit and app sources. It excludes declarations,
implementations, generated bindings and vendor code. “Tests only” means no
literal non-test call was found in those trees. Haxe owned-handle cleanup can
invoke `*_destroy` through generated ownership bindings without a literal call.

`SIMPLIFICATION.md` S1, S3 and S4 are proposed migrations. Current production
callers remain the gate: move those callers and tests before deleting or moving
an ABI symbol. S1 explicitly names the older count/sample and ladder routes;
S3 names one-shot timing, sessions and reference timing; S4 moves trajectory
transforms to TrajectoryKit. S4 also calls out unused kinematics exports.

| Function | Observed use | Representative caller | Proposed disposition |
| --- | --- | --- | --- |
| `mk_refine_redundancy` | Production | `motionkit/robot/haxe/motionkit/robot/DriveAwareRefinement.hx` | Keep; reassess after S1–S4 |
| `mk_eaik_create` | Production | `motionkit/robot/haxe/motionkit/robot/EaikChain.hx` | Keep; reassess after S1–S4 |
| `mk_eaik_destroy` | Tests only; ownership hook | `motionkit/native/tests/eaik_spike.cpp` | Keep while EAIK handles exist; generated ownership call |
| `mk_eaik_forward` | Tests only | `motionkit/native/tests/candidate_sampling.cpp` | S4: delete after checking remaining callers |
| `mk_eaik_inverse` | Tests only | `motionkit/native/tests/eaik_spike.cpp` | Audit with EAIK family after S1 |
| `mk_eaik_inverse_labelled` | Production | `motionkit/robot/haxe/motionkit/robot/EaikAnalyticIk.hx` | Keep; reassess after S1–S4 |
| `mk_analytic_cartesian_forward` | Production | `motionkit/robot/haxe/motionkit/robot/CartesianAnalyticIk.hx` | Keep; reassess after S1–S4 |
| `mk_analytic_cartesian_inverse` | Production | `motionkit/robot/haxe/motionkit/robot/CartesianAnalyticIk.hx` | Keep; reassess after S1–S4 |
| `mk_orientation_lattice_count` | Production | `motionkit/robot/haxe/motionkit/robot/OrientationLattice.hx` | Keep; reassess after S1–S4 |
| `mk_sample_orientations` | Production | `motionkit/robot/haxe/motionkit/robot/OrientationLattice.hx` | Keep; reassess after S1–S4 |
| `mk_joint_lift_count` | Production | `motionkit/robot/haxe/motionkit/robot/JointLifts.hx` | Keep; reassess after S1–S4 |
| `mk_enumerate_joint_lifts` | Production | `motionkit/robot/haxe/motionkit/robot/JointLifts.hx` | Keep; reassess after S1–S4 |
| `mk_external_lattice_count` | Production | `motionkit/robot/haxe/motionkit/robot/ExternalAxisGrid.hx` | Keep; reassess after S1–S4 |
| `mk_sample_external_cells` | Production | `motionkit/robot/haxe/motionkit/robot/ExternalAxisGrid.hx` | Keep; reassess after S1–S4 |
| `mk_cartesian_candidate_count` | Tests only | `motionkit/native/tests/candidate_sampling.cpp` | S1: retire after unified planner migration |
| `mk_sample_cartesian_candidates` | Tests only | `motionkit/native/tests/candidate_sampling.cpp` | S1: retire after unified planner migration |
| `mk_create_eaik_candidate_batch` | Production | `motionkit/robot/haxe/motionkit/robot/SerialCandidateSampler.hx` | Keep; reassess after S1–S4 |
| `mk_create_cartesian_candidate_batch` | Production | `motionkit/robot/haxe/motionkit/robot/CartesianCandidateSampler.hx` | Keep; reassess after S1–S4 |
| `mk_candidate_batch_destroy` | Tests only; ownership hook | `motionkit/native/tests/candidate_sampling.cpp` | Keep while batch handles exist; generated ownership call |
| `mk_read_candidate_batch` | Production | `motionkit/robot/haxe/motionkit/robot/NativeCandidateBatch.hx` | Keep; reassess after S1–S4 |
| `mk_eaik_candidate_count` | Tests only | `motionkit/native/tests/candidate_sampling.cpp` | S1: retire after unified planner migration |
| `mk_sample_eaik_candidates` | Tests only | `motionkit/native/tests/candidate_sampling.cpp` | S1: retire after unified planner migration |
| `mk_sample_eaik_candidates_compact` | Tests only | `motionkit/native/tests/candidate_sampling.cpp` | S1: retire after unified planner migration |
| `mk_sample_cartesian_candidates_compact` | Tests only | `motionkit/native/tests/candidate_sampling.cpp` | S1: retire after unified planner migration |
| `mk_path_differential` | Production | `motionkit/robot/haxe/motionkit/robot/PathDifferential.hx` | Keep; reassess after S1–S4 |
| `mk_search_ladder` | Tests only | `motionkit/native/tests/structured_ladder.cpp` | S1: retire after unified planner migration |
| `mk_search_ladder_filtered` | Tests only | `motionkit/native/tests/structured_ladder.cpp` | S1: retire after unified planner migration |
| `mk_search_ladder_compact_filtered` | Production | `motionkit/robot/haxe/motionkit/robot/StructuredLadder.hx` | Keep; reassess after S1–S4 |
| `mk_select_configurations` | Tests only | `motionkit/native/tests/configuration.cpp` | S1: retire after unified planner migration |
| `mk_path_create` | Production | `motionkit/haxe/motionkit/path/NativeJointPath.hx` | Keep; reassess after S1–S4 |
| `mk_path_destroy` | Example/tool + tests; ownership hook | `machinekit/examples/sheet-router/fem/planner_sample.cpp` | Keep while path handles exist; generated ownership call |
| `mk_time_law_create` | Production | `motionkit/haxe/motionkit/path/PathTimeLaw.hx` | Keep; reassess after S1–S4 |
| `mk_time_law_destroy` | Example/tool + tests; ownership hook | `machinekit/examples/sheet-router/fem/planner_sample.cpp` | Keep while law handles exist; generated ownership call |
| `mk_path_distance_to_time` | Production | `motionkit/haxe/motionkit/path/PathTimeLaw.hx` | Keep; reassess after S1–S4 |
| `mk_path_times_to_distances` | Production | `motionkit/haxe/motionkit/path/PathTimeLaw.hx` | Keep; reassess after S1–S4 |
| `mk_time_law_binding_count` | Production | `motionkit/haxe/motionkit/path/PathTimeLaw.hx` | Keep; reassess after S1–S4 |
| `mk_time_law_get_binding` | Production | `motionkit/haxe/motionkit/path/PathTimeLaw.hx` | Keep; reassess after S1–S4 |
| `mk_path_lower` | Production | `motionkit/haxe/motionkit/path/NativeJointPath.hx` | Keep; reassess after S1–S4 |
| `mk_time_path` | Example/tool + tests | `machinekit/examples/sheet-router/fem/planner_sample.cpp` | S3: retire after stream migration |
| `mk_path_timing_session_create` | Example/tool + tests | `machinekit/examples/sheet-router/fem/planner_sample.cpp` | S3: retire after stream migration |
| `mk_path_timing_session_destroy` | Example/tool + tests | `machinekit/examples/sheet-router/fem/planner_sample.cpp` | S3: retire after stream migration |
| `mk_path_timing_session_time` | Example/tool + tests | `machinekit/examples/sheet-router/fem/planner_sample.cpp` | S3: retire after stream migration |
| `mk_path_timing_session_time_to_interval` | Tests only | `motionkit/native/tests/path_timing_session.cpp` | S3: retire after stream migration |
| `mk_path_timing_session_entry_interval` | Example/tool + tests | `machinekit/examples/sheet-router/fem/planner_sample.cpp` | S3: retire after stream migration |
| `mk_path_timing_stream_create` | Production | `motionkit/haxe/motionkit/path/NativeJointPath.hx` | Keep; reassess after S1–S4 |
| `mk_path_timing_stream_destroy` | Tests only; ownership hook | `motionkit/native/tests/path_timing_stream.cpp` | Keep while stream handles exist; generated ownership call |
| `mk_path_timing_stream_step` | Production | `motionkit/haxe/motionkit/planner/PathTimingStream.hx` | Keep; reassess after S1–S4 |
| `mk_path_timing_stream_commit` | Production | `motionkit/haxe/motionkit/planner/PathTimingStream.hx` | Keep; reassess after S1–S4 |
| `mk_path_timing_preview_destroy` | Tests only; ownership hook | `motionkit/native/tests/path_timing_stream.cpp` | Keep while preview handles exist; generated ownership call |
| `mk_time_path_reference` | Tests only | `motionkit/native/tests/path.cpp` | S3: retire after stream migration |
| `mk_project_couplings` | Production | `motionkit/haxe/motionkit/trajectory/Trajectory.hx` | S4: move to TrajectoryKit |
| `mk_merge_segments` | Production | `motionkit/haxe/motionkit/trajectory/Trajectory.hx` | S4: move to TrajectoryKit |
| `mk_merge_segments_position_budget` | Production | `motionkit/haxe/motionkit/trajectory/Trajectory.hx` | S4: move to TrajectoryKit |
| `mk_shape_trajectory` | Production | `motionkit/haxe/motionkit/trajectory/Trajectory.hx` | S4: move to TrajectoryKit |
| `mk_advance_joint` | Production | `motionkit/haxe/motionkit/trajectory/Trajectory.hx` | S4: move to TrajectoryKit |
| `mk_advance_joint_context` | Production | `motionkit/haxe/motionkit/trajectory/Trajectory.hx` | S4: move to TrajectoryKit |
| `mk_generate_state_to_state` | Production | `motionkit/haxe/motionkit/trajectory/Trajectory.hx` | Keep; reassess after S1–S4 |

## Review order

1. Land the S1 unified planner migration, then remove the listed
   test-only candidate and search entry points together with their Haxe bindings.
2. Port the router FEM example to the timing stream, then retire S3 timing
   entry points and their reports/handles where no longer needed.
3. Move the transform family to TrajectoryKit under one ownership and validation
   contract. Audit `mk_eaik_forward` and `mk_eaik_inverse` for deletion.
4. Regenerate or edit `motionkit/native/bindings/motionkit.hxi` in the same change
   as each C declaration change. Recheck callers and run the native, Haxe and
   integration suites at each migration gate.
