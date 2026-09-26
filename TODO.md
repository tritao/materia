# Follow-up tasks

## Physics regression

- [ ] Add a focused MuJoCo actuator-limit test in RobotKit or SimKit. Use a
  gravity-loaded single joint with known mass, center of mass, axis, and torque
  limit. Verify that a position command stalls when available torque is below
  the gravitational load, then moves when the limit is raised above it. Check
  observed joint position and applied effort over a defined simulation interval.
  Keep this separate from Materia's runtime-overlay test, whose arm fixture is
  intentionally balanced about its joint axis.
