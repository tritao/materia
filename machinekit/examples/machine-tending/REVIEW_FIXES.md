# Machine tending remaining review fixes

MT1–MT6 and MT5b are complete. These fixes precede MT7. Each group gets focused checks,
measured baseline comparisons and a commit; the complete sequential gate closes the work.
Do not retain old-format compatibility: migrate in-repo callers and fixtures, require the
current format, and remove obsolete paths. Reference hardware data remains explicitly assumed.

| Group | Work | State | Commits |
|---|---|---|---|
| R1 | One current assembly schema and runtime layout; migrate producers/tests; delete legacy defaults and dead helpers | done | `0121118f2` |
| R2 | Motor/driver/supply sizing; real spindle belt transmission; motor shaft and coupling engagement; explicit screw root | done | `59791d003` |
| R3 | Physical Z brake and fault sequencing; full downstream inertia feedforward | in progress: inertia implemented; brake remains | `6e94de72f` |
| R4 | Swept clearance for joint moves; assembled limits; independent geometry/mass/speed checks | in progress: joint-plan clearance implemented | |
| R5 | Per-joint servo interpolation; consistent slip; typed channel faults; cached coupling setup; qualified channels and two-mill check | done | `dd0c14091` |
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

- MachineKit descriptions use the current version for every assembly (v13 in R1, v14 after the R2 hardware recipe changes); ProjectKit assembly definitions
  and states use v7. Optional physical features stay optional, but do not select a different schema.
- Native RobotKit blueprints and snapshots require their complete current layout. In-repo
  native and Haxe producers already emit the current layout; old-layout acceptance tests
  become rejection tests. No prefix-copy or missing-field defaults remain.


Done. What was built and decided:

- Removed old schema acceptance, native blueprint/snapshot prefix handling, and the unused coupled-limit helper. No outdated tracked assembly documents needed migration.
- ProjectKit: 144 assertions; RobotKit: 4,999 assertions; MachineKit smoke passed. Native validation and C API checks passed, including rejection of incomplete layouts.

## R2 decisions

- SFU1605 race root is explicitly assumed at 13 mm and shared with its nut; other ball races must state their root.
- Generic servo shafts are assumed per size (8×20, 14×30, 19×35 mm), with a 20×30 mm spindle shaft. The rotating shaft is a reusable SteppedShaft. Couplings seat both shafts to their midpoint.
- The 1:1 spindle belt uses its pulley geometry and a separate motor rotation joint. Cartridge and winding speed limits are independent, and readiness follows the drive reference through the belt relation.
- Servo winding data, amplifier efficiency and reference envelopes are assumed. AC-class named motors use a 310 V bus; integrated cobot motors keep their explicitly separate low-voltage winding. Phase RMS and DC currents have separate checks.
- Sources support simultaneous peak torque-speed electrical draw continuously, avoiding an unmodelled burst duty allowance. Integrated shunts are rated against simultaneous regeneration. Motor rated torque remains the planner thermal bound.
- MachineKit descriptions move to v14 for the changed hardware recipes; there is one accepted version, with no v13 fallback.

Done. What was built and decided:

- MachineKit smoke passed after migrating wired AC-servo fixtures; native validation passed, including coupled process ownership and command rejection.
- The app mill checks passed: bare/enclosed 51.77/52.49 s, 78 plans; X/Y/Z tracking 0.077237/0.051505/0.053502 mm at 10 ms. Rapids remain 15.0955/25/22.8666 m/min. Larger rotating shafts and couplings change X/Y/Z accelerations to 29.87/28.97/26.93 m/s². Door and clamp timings/force remain unchanged.
- Arm/welder assembly mass grows to 155.9/321 kg from sized offboard electrical hardware; moving-arm mass is unchanged. Remaining cross-example motion comparisons belong to the complete closing gate.

## R5 decisions

- Controller setpoint time belongs to each mechanically coupled drive group. Ordinary unrelated position joints keep the target actually sent; servo followers share their motor's interpolated interval end.
- Slip is a constant coordinate offset applied to both Hermite endpoints, preserving velocity and acceleration.
- Resolve coupling equality indices and physical response constants on topology rebuild; each step only applies MuJoCo's timestep floor.
- Require current complete SimKit world, joint, coupling and target layouts. Delete old prefix handling and runtime-port compatibility views; migrate their tests and callers.
- CNC channel mapping is a required per-machine binding. Nested inclusion qualifies drive channels, rebuilding strips the enclosing scope, and lowering/restart/shutdown use the same mapping. Two included mills retain distinct spindle channels.
- Typed process reads distinguish missing channels, wrong event kinds and nonfinite analog values. Fault only the owning robot and apply de-energized physical outputs while other robots advance. Ordinary stops retain pneumatic coil outputs; faults and emergency stops drop them, with valve latches and supply air remaining physical simulation state.

Done. What was built and decided:

- MachineKit smoke passed after all capability and nested-channel migrations. Toolpath motion: 18 assertions, accuracy 2, machining run 12, scenarios 2,952.
- Native runtime tests passed, including mixed ordinary/servo timing, coupled followers, typed channel errors and fault isolation. Native MuJoCo tests passed, including rejection of truncated layouts and Hermite interval-end feedforward.
- Mill application checks passed with physical readiness selected by the spindle's mechanical joint: bare/enclosed 51.77/52.49 s; X/Y/Z peaks 0.077237/0.051505/0.053502 mm. These match R2. Added full-job tracking and physical readiness checks.

## R3 articulated inertia

- The drive gain/stability estimate includes every descendant's rotated tensor and centre-of-mass parallel-axis term at the neutral configuration. It is explicitly a reference estimate.
- Physics acceleration feedforward uses MuJoCo's live full mass matrix, including articulated cross terms and native payload attachments. Cached transmission nullspace coordinates lift motor reference accelerations to the mechanism and project the resulting effort back by virtual work. Parallel drives share effort by their capacity and leverage.
- SimKit targets and the native robot blueprint no longer carry obsolete scalar reflected inertia. The estimate stays in Haxe for drive stability calculations. Native/Haxe callers and generated bindings use the new complete layouts; no previous-layout fallback remains.
- Native MuJoCo tests pass with a 3 kg downstream attachment and a two-link arm at 0 and 90 degrees, verifying both shoulder and elbow effort. Interval-end feedforward remains zero.
- Mill application checks pass without a changed baseline: bare/enclosed 51.77/52.49 s; X/Y/Z peaks 0.077237/0.051505/0.053502 mm at 10 ms. The physical brake and its sequencing remain to be implemented before R3 is complete.

## R4 joint-plan admission

- A typed plan validator checks the actual timed polynomial before submission, including its interior. A maximum-speed bound determines sample count for each segment, catching excursions whose endpoints return to the same place. This remains a sampled clearance check, not a continuous collision certificate.
- Mission joint moves snapshot fixed cell geometry and other controllers' bodies at planning start. The worker uses the same immutable clearance policy. Moving obstacles still require runtime monitoring.
- Joint moves require mechanical velocity limits and a typed mission frame; the cast and arbitrary 2 rad/s velocity fallback are removed.
- Focused interior-excursion clearance regression: 2 assertions passed. The complete gate runs the registered regression and passes MotionKit at 67,535 assertions.
- RobotKit passed at 4,999 assertions, including the downstream inertia estimate. All cobot missions still take 2.06 s; Reach500/850/900/1300 peaks become 0.004640/0.005987/0.011236/0.016598 mrad because acceleration feedforward now includes the complete articulated inertia.
- Bench-mill checks retain typed installed drive references instead of casting component entries. Screw lead is read from the part; the doorway uses posed casting bounds; a compiled G54-zero probe is checked against actual stock and spindle locating geometry.
- Cobot checks verify exported joint limits and CAD overlap at both zero and ready configurations. A full-turn periodic FK identity no longer masquerades as a joint-limit clearance check.

## Verified checkpoint: articulated inertia and joint-plan admission

- `mt-suite-review-inertia-clearance.txt`: every sequential kit, CadKit, MachineKit smoke, app build and complete project-source suite passed. Native runtime and MuJoCo checks passed against the rebuilt current layouts.
- Compared with R2, mill and all existing example cycle baselines remain unchanged. Screw/belt router 232.2/215.2 s, 128 plans, 0.05/1.76 mm worst deviation; CoreXY 649.6 mm/s and 71.549 m/s²; arm 5.8/12/17.2/23.5 s; mobile mission/obstacle 61/65 s; welder 20.6/21.1 s and whole weldment 106.5 s.
- Reach remains Reach850 on a 600 mm riser, with 15 poses and 16.2399% minimum joint margin. Door 1.50/1.79 s, seated blank clamp 0.08 s at 482.55 N. The cell has 16 disjoint joints: R2's separate spindle-motor rotation adds one to MT6's 15.
- Open work: physical Z brake and fault sequencing (R3); remaining joint-limit and independent-environment clearance work (R4); rail/support sizing, bearing-block wall and cabinet mounts (R6). No new compiler defect was found.
