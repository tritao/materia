# SensorKit references

The local `gz-sensors/` checkout is intentionally ignored by Git. It is a
reference for behavior and algorithms, not a SensorKit dependency.

Current checkout:

```text
repository: https://github.com/gazebosim/gz-sensors.git
tag:        gz-sensors10_10.1.1
```

It is Apache-2.0 licensed; any reused implementation must retain the upstream
copyright and license notices. Prefer reimplementing the boundary and copying
only algorithms whose behavior has been validated against SensorKit tests.
