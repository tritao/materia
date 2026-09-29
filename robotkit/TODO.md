# RobotKit follow-ups

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
