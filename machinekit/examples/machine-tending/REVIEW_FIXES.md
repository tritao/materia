# Machine tending remaining review fixes

MT1–MT6 and MT5b are complete. These fixes precede MT7. Each group gets focused checks,
measured baseline comparisons and a commit; the complete sequential gate closes the work.
Do not retain old-format compatibility: migrate in-repo callers and fixtures, require the
current format, and remove obsolete paths. Reference hardware data remains explicitly assumed.

| Group | Work | State | Commits |
|---|---|---|---|
| R1 | One current assembly schema and runtime layout; migrate producers/tests; delete legacy defaults and dead helpers | done | pending commit |
| R2 | Motor/driver/supply sizing; real spindle belt transmission; motor shaft and coupling engagement; explicit screw root | planned | |
| R3 | Physical Z brake and fault sequencing; full downstream inertia feedforward | planned | |
| R4 | Swept clearance for joint moves; assembled limits; independent geometry/mass/speed checks | planned | |
| R5 | Per-joint servo interpolation; consistent slip; typed channel faults; cached coupling setup; qualified channels and two-mill check | planned | |
| R6 | Rail support/load paths, bearing walls, cabinet mounts and material assumptions; remaining diagnostics and dead code | planned | |

## Verification

- Keep the 10 ms controller period and drive-owned substeps.
- Validate electrical and mechanical failures with deliberately incompatible parts.
- Observe actual motion, clearance, brake and fault behavior; avoid checks that merely repeat construction formulas.
- Compare against MT6: bare/enclosed mill 51.86/52.30 s; screw/belt router 232.2/215.2 s,
  0.05/1.76 mm deviation; CoreXY 649.6 mm/s, 71.549 m/s²; arm mission
  5.8/12/17.2/23.5 s; mobile mission/obstacle 61/65 s; welder seam 20.6/21.1 s,
  weldment 106.5 s. Record intended changes with their cause.

## R1 decisions

- MachineKit descriptions use v13 for every assembly; ProjectKit assembly definitions
  and states use v7. Optional physical features stay optional, but do not select a different schema.
- Native RobotKit blueprints and snapshots require their complete current layout. In-repo
  native and Haxe producers already emit the current layout; old-layout acceptance tests
  become rejection tests. No prefix-copy or missing-field defaults remain.


Done. What was built and decided:

- Removed old schema acceptance, native blueprint/snapshot prefix handling, and the unused coupled-limit helper. No outdated tracked assembly documents needed migration.
- ProjectKit: 144 assertions; RobotKit: 4,999 assertions; MachineKit smoke passed. Native validation and C API checks passed, including rejection of incomplete layouts.
