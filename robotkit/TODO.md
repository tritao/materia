# RobotKit follow-ups

## Perception foundations (robotkit/plans/PERCEPTION.md)

- Add `robotkit/tools/recording/foxglove-export` when Foxglove export is needed; the user explicitly said "you can ignore the foxstudio toolf or nwo" during Phase 3 of this session.
- Support ONNX exports with dynamically sized output tensors.
- Add a TCP observer test that measures subscription delivery rate under frame timing jitter.
- Separate inference and policy native dependencies from consumers that do not use them.
- Expand detector unit coverage for decode, NMS, exact drop accounting, and non-default
  camera aspect ratios through the Haxe entry point.
- Consider a second TCP connection for bulk data; UDP, QUIC, and HTTP model distribution, health, and recording download remain outside RKF1.
- Estimate cross-process clock offsets when a bounded mapping can be measured.
- Export a nanosecond-valued RKD6 mapping snapshot; test skew uncertainty and
  expiry policies against measured device drift.
- Add world fusion, tracking, and shared maps after observations are available.
- Consider `perceptiond` as a separate execution host.
- Evaluate GPU execution providers and inference backends beyond the pinned CPU ONNX Runtime.
- Reconsider OpenCV only for camera capture, calibration, undistortion, or heavier image processing; use vendored `stb_image` for JPEG decode.
- Add `jpeg` and `depth32f` detector inputs and depth/intrinsics lifting into planar `Detection`.
- Add segmentation, pose, and occupancy observations and fault/world-event robot stream cases.
- Decide on production sensorkit camera integration after the Phase 7 spike.

## Humanoid (robotkit/plans/HUMANOID.md)

- H3 sensing the policy run does not use yet: encoder quantization, IMU noise,
  bias and latency, per-foot contact force aggregation, an observation
  assembler that can inject them. `GravityEstimator` was tuned on a noise-free
  gyro; a biased one needs its accelerometer gain reconsidered.
- H5: a stand-up state before the policy (the G1 cannot hold its default pose
  on servos), fall detection, HOLD as a zero-velocity command through
  `PolicySession`, MotionGuard checks on every servo batch.
- Policy state on MCAP channels (command reference, gait phase, estimate,
  clamped-target count) and in the editor (H6). Recordings currently carry the
  snapshots and servo batches only, at IMU rate: 500 snapshots per second.
- `PolicyController` clamps servo targets to joint travel because the runtime
  rejects a target outside it. Decide whether a humanoid blueprint wants a
  soft margin there instead (`observed_limit_tolerance` has the same question).
- Run the policy under MJX's truncated solver (5 iterations, 8 line-search) to
  see what a Playground-trained policy will meet.
- Import collision meshes as convex hulls so a fall lands on the torso, not
  through it (finding 6 of the H4 log in the plan).
- ONNX Runtime pins for linux-aarch64, macOS arm64 and Windows x64 are
  unverified; add CI or a machine that builds them.
- `RuntimeRobotAdapter.sensors()` returns the frames of the last `snapshot()`.
  Either document it on `Robot.sensors()` or refresh it.

## Grippers

- `SimulatedGripper` only reports a grasp state; the object never moves with
  the tool. Closing on a detected object should hold it with
  `nksim_session_hold_object` on the tool-flange link body, and opening should
  release it, as HumanKit workers already do (see also `simkit/TODO.md` on the
  one-tick lag of link carriers).
