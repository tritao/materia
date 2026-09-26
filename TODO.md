# Follow-up tasks

## Physics regression

- [ ] Add a focused MuJoCo actuator-limit test in RobotKit or SimKit. Use a
  gravity-loaded single joint with known mass, center of mass, axis, and torque
  limit. Verify that a position command stalls when available torque is below
  the gravitational load, then moves when the limit is raised above it. Check
  observed joint position and applied effort over a defined simulation interval.
  Keep this separate from Materia's runtime-overlay test, whose arm fixture is
  intentionally balanced about its joint axis.

## MotionKit buffered execution

- [ ] Send the next queued trajectory before the runtime drains the current
  one, removing the 1–2 control-cycle pause between queued moves. Completion
  and hold/resume then have to track two trajectories' chunk tags in the
  runtime queue at once; extend the hold sweep in
  `motionkit/tests/src/MotionKitBootstrapTests.hx` across the handover.
