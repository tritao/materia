# Machine tending remaining review fixes

MT1–MT6 and the MT5b review fixes are complete. All six groups passed the complete
sequential closing gate and focused native checks. MT7 remains a separate handoff.
The sections below retain checkpoint history; final verification closes every earlier pending item.
Do not retain old-format compatibility: migrate in-repo callers and fixtures, require the
current format, and remove obsolete paths. Reference hardware data remains explicitly assumed.

| Group | Work | State | Commits |
|---|---|---|---|
| R1 | One current assembly schema and runtime layout; migrate producers/tests; delete legacy defaults and dead helpers | done; current schemas and complete native layouts verified | `0121118f2`, `57db394b3`, `b9acdd4a5` |
| R2 | Motor/driver/supply sizing; real spindle belt transmission; motor shaft and coupling engagement; explicit screw root | done | `59791d003` |
| R3 | Physical Z brake and fault sequencing; full downstream inertia feedforward | done; actual mill brake, saved bindings, fault sequencing and full gate verified | `6e94de72f`, `ea48834a8`, `f38740676`, `b248a1301`, `9dfc9bf1f`, `a56345609`, `03838fb54`, `b9acdd4a5`, `61fd8f6af`, `144010183`, `295d426ea`, `4ff013c90` |
| R4 | Swept clearance for joint moves; assembled limits; independent geometry/mass/speed checks | done; clearance, environment objects and explicit motion policy verified | `4fc64fbee`, `8b22593be`, `4bd869718` |
| R5 | Per-joint servo interpolation; consistent slip; typed channel faults; cached coupling setup; qualified channels and two-mill check | done | `dd0c14091` |
| R6 | Rail support/load paths, bearing walls, cabinet mounts and material assumptions; remaining diagnostics and dead code | done; supports, paired guides and load admission verified | `20f0d3e16`, `56923ba3e` |

## Verification

- Keep the 10 ms controller period and drive-owned substeps.
- Validate electrical and mechanical failures with deliberately incompatible parts.
- Observe actual motion, clearance, brake and fault behavior; avoid checks that merely repeat construction formulas.
- Compare against MT6: bare/enclosed mill 51.86/52.30 s; screw/belt router 232.2/215.2 s,
  0.05/1.76 mm deviation; CoreXY 649.6 mm/s, 71.549 m/s²; arm mission
  5.8/12/17.2/23.5 s; mobile mission/obstacle 61/65 s; welder seam 20.6/21.1 s,
  weldment 106.5 s. Record intended changes with their cause.

## R1 decisions

- MachineKit descriptions use the current version for every assembly (v13 in R1, v14 after R2's hardware recipes, v15 after the paired-guide casting changes); ProjectKit assembly definitions
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

## R4 environment objects

- Planning clearance includes independently owned collision objects, using their actual simulated poses and primitive geometry. Objects held by the planning robot are attached to their actual carrier link in the clearance model; other objects remain snapshots at planning start.
- Native carrier handles stay typed throughout; there is no cast or integer sentinel conversion. Independent scene-obstacle and carried-object future-collision application regressions pass, and clear cobot missions retain their 2.06 s cycles and tracking peaks.

## R6 support geometry in progress

- A reusable, saved rectangular tube member uses the existing structural profile geometry. Four 30 × 30 × 2 mm steel legs carry the cabinet bottom to the floor, with CAD contact and hollow-section checks.
- The blank becomes 60 × 60 × 20 mm. The contour pocket leaves a 3 mm front wall and a 4 mm ligament to the centred bearing seat, with the toe clamps remaining at the back corners. Actual material witnesses check both walls after the cuts. CAM reads the revised recesses from the solid. Bare/enclosed cycles become 52.51/53.24 s because the larger contour removes another 144 mm³. X/Y/Z peaks are 0.085925/0.051505/0.053502 mm at the unchanged 10 ms tick; the larger blank increases carried mass.
- Per-size cobot housing and gearhead materials are explicitly assumed allocations of generic package mass, not claims about vendor casing metallurgy. Cobot modules retain a typed servo reference rather than recovering it with casts.
- Rail support sizing and the physical Z brake remain open.

Verified support checkpoint:

- The sequential kit gate passed all eleven kits, CadKit and MachineKit smoke. Its first app run found the old 12 × 8 mm analytical removal expectation; the revised 12 × 12 mm expectation is independent of the production solid and now also checks the enclosed job quantitatively.
- Focused mill app checks pass: 3860.62 of 3846.78 mm³ removed, 7.14 mm³ leftover, 0.618 mm³ gouge, zero rapid or holder collisions, 78 plans and the existing 55 flagged drive diagnostics (42 peak-torque, 34 rated-RMS findings). Rapids remain 15.0955/25/22.8666 m/min; accelerations are now approximately 29.86/28.95/26.93 m/s² from the increased blank mass.
- Door open/close remains 1.50/1.79 s; seated clamp 0.08 s at 482.55 N. Four cabinet legs add 5.2752 kg of stationary structure; enclosed object count becomes 122. Reach remains Reach850/600 mm with 16.2399% minimum joint margin.
- The complete app retry finished with exit 0 and no uncaught exception (`/tmp/mt-review-environment-support-app.log`). Together with the preceding unchanged kit gate, this closes verification of `20f0d3e16`. The original failed gate report remains intact; it is not presented as a passing app run. The subsequently registered guide test and native brake work are separate, still-open changes.
- Existing baselines match: screw/belt router 232.2/215.2 s, 128 plans and 0.05/1.76 mm deviation; CoreXY 649.6 mm/s and 71.549 m/s²; arm 5.8/12/17.2/23.5 s; mobile mission/obstacle 61/65 s; welder seam 20.6/21.1 s, post 29.8 s and weldment 106.5 s.

## R3 brake implementation requirements

- Replace the unused example boolean with a typed brake part and binding on the physical motor joint. Its stator and rotor belong to the mechanical assembly, with one assumed rating/specification and derived mounting geometry.
- The controller owns a digital release output and waits for released feedback before advancing motion. Hardware endpoints see coil outputs and feedback; cylinder or brake physics must never run in RobotRuntime. Missing release feedback prevents motion and faults the owning controller.
- The simulation owns the spring-applied brake state, release/application delay and bounded passive friction effort, evaluated at physics substeps. Use the drive's integration settings and actual joint inertia; do not introduce an application heuristic or position-hold shortcut.
- Ordinary stop retains release while the energized servo holds. Emergency stop and fault de-energize release; the spring applies the brake. Safety reset must require release confirmation again before motion resumes.
- Check release-before-motion, stop/fault/reset sequencing, and gravity holding with motor effort disabled. Compare the torque capacity with the complete carried Z load. Any saved field or native layout change takes one current version, with every in-repo producer/fixture migrated and old paths removed.

## R6 remaining guide implementation requirements

- Give the carriage finite support span along travel as well as across rails, with paired blocks on each rail. Derive block centre spacing from actual block length and a stated assembly gap.
- Size the saddle/head/column and rail lengths to seat every block at all travel limits. Compute overtravel from all blocks, including followers, without recovering rail types through casts.
- Verify actual mount contact, block separation, and rail room at travel extremes. Evaluate load and moment reactions from assembled mass, commanded acceleration and an explicitly assumed cutting load; compare with explicitly assumed guide capacities and reject overloads in checks.

The implementation and verification below track these requirements.

## R3 physical brake state in progress

- SimKit's `BrakeState` starts spring-applied, advances release/application delays only in simulation time, and exposes released contact feedback and passive friction effort. Repeated coil writes preserve the delay; a changed command cancels the previous transition. It does not generate motor targets or run in RobotRuntime.
- The standalone native state test passes with warnings treated as errors. It covers delayed release, delayed application, cancelled release, interrupted application, zero-delay devices and invalid values. The test is registered in CMake with assertions enabled in Release.
- Native SimKit APIs now configure one brake per joint, set its digital release coil and read its physical released contact. MuJoCo advances contact state each physics substep and adds bounded dry-friction effort to the joint's existing friction, independently of motor targets. The kinematic test backend reports this physical capability as unsupported.
- The rebuilt complete MuJoCo native suite passes, including unpowered gravity holding, delayed release/application and slipping with insufficient capacity. SimCore and its C API tests pass. All changed translation units pass syntax checks with production flags, and the state test passes with exceptions disabled. NativeKit's no-exceptions policy required replacing the initial throwing constructor with a checked factory and explicit failure returns. These native checks verify `f38740676`; the preceding app gate used the earlier libraries without brake bindings.
- The regenerated portable SimKit interface and typed Joint brake methods pass the Haxeon compile check, with all three methods reachable from the compile fixture. Typed assembly/blueprint bindings, the physical brake part, runtime release interlocks and stop/fault/reset checks are still required. The state test alone does not prove the complete brake behavior.

## R6 paired guides in progress

- Each rail now carries two blocks separated by a 4 mm assembly gap, giving 65.4 mm centre spacing. There are twelve blocks in the mill. The slide builder receives the installed rail directly and measures overtravel across all four block poses; the rail cast is removed.
- `GuideSupport` resolves a complete force/moment wrench onto the four equal-stiffness blocks. Its registered smoke test independently balances the result, verifies overload detection and checks that increased support span reduces the worst block load. An independent numerical check covered 1,000 wrenches with a maximum residual of 9.1e-13.
- Saddle width becomes 450 mm, head height 130 mm and column height 670 mm. Machine descriptions require v15 for the changed casting recipes; no v14 acceptance remains. The full smoke checks verify CAD masses at 50.9425/25.4801/33.2253 kg for column/saddle/head. The initial column expectation omitted mounting-hole ligaments exposed by the longer core.
- New CAD checks measure all block and rail bounds at every travel corner, compare their observed room with compiled overtravel, verify block separation and press each actual mounting face into its carriage to establish finite contact. These checks and the CNC geometry pass after fixing the test's projection onto connector Z instead of the mechanical +Y axis. Measured X/Y/Z overtravel is 11.6/39.6/20.6 mm; source-derived rapids are 15.0955/25/21.5218 m/min. The longer Z screw lowers its critical-speed cap. Both the mass-corrected smoke and the scoped load-check smoke pass.
- Guide capacities are explicitly assumed at 10 kN per HGR15 block and 2.6 kN per MGN12C block, with no vendor-verification claim. Load admission uses the resolved physical-part view and mate tree, posed centres of mass and drive-derived acceleration limits. It combines all acceleration signs with an explicit 500 N cutting envelope per world axis at all travel corners. With a safety factor of two, bare X/Y/Z utilization is 14.5363/48.1601/63.0236%; the enclosed mill is 20.2625/54.7884/63.0236%. A one-newton capacity fails the same admission function.
- The load study requires the already-resolved whole assembly artifact and an explicit prefix: the vise's required air ports are connected by the enclosure, so a stand-alone vise-equipped mill is not a valid physical assembly. All new study data stays typed without casts. Motion-direction normalization and metadata-derived unit conversion pass. The study lives in `machinekit/tests`, keeping its CadBridge dependency out of the example project; the corrected smoke registration passes in `/tmp/mt-guides-test-boundary.log`.
- The rebuilt app's mill filter passes: bare/enclosed 52.54/53.19 s; X/Y/Z peaks 0.085940/0.051497/0.072365 mm at 10 ms; X/Y/Z acceleration approximately 29.77/28.24/26.44 m/s². Changed carried masses and the longer Z screw alter planning and tracking. Removal is 3860.66 of 3846.78 mm³, leftover 7.13 mm³ and gouge 0.659 mm³, with zero rapid or holder collisions, 78 plans and the existing 55 flagged drive diagnostics (42 peak, 34 rated RMS). Door 1.50/1.79 s and seated clamp 0.08 s at 482.55 N remain unchanged. The enclosed scene has 128 objects. The complete closing gate remains pending.

Additional cleanup found while tracing the brake API:

- Removed obsolete prefix-layout descriptions from `simkit/sim_core/include/nativekit_sim.h` and `simkit/sim_core/src/PhysicsBackend.hpp`. The joint-coupling C entry point now requires the same complete descriptor that the world layer already required, rather than checking the obsolete prefix first. Valid callers and physical behavior are unchanged; rebuilt SimCore, C API and MuJoCo tests pass.

## R4 requested joint-motion policy

- `JointMotionPolicy` owns requested acceleration and jerk separately for angular and linear joints, in SI units. MissionPlayer accepts the typed policy and passes the same resolved rates to both validation and compilation. Defaults preserve the prior mission requests (2 rad/s² and 20 rad/s³; separately 2 m/s² and 20 m/s³), without presenting them as physical ratings.
- Every joint still requires a finite positive mechanical velocity limit. A stated acceleration ceiling caps the requested acceleration; a missing ceiling leaves the explicit planning request in force. Invalid and zero mechanical acceleration limits reject motion instead of silently substituting a fallback. Fixed and floating joints cannot use this scalar policy.
- The absolute-path app build and focused clearance suite pass (`/tmp/mt-joint-policy-app-build.log`, `/tmp/mt-joint-policy-clearance.log`). Registered checks exercise different angular/linear policies, hardware capping, missing acceleration ratings, zero-cap rejection and missing-velocity rejection. Existing independent and held-object clearance checks pass with the integrated policy. The complete closing gate remains pending.

## R3 release handshake implementation

- `BrakeReleaseInterlock` is a controller-only handshake: energize the release output, wait for a newer contact sample within its age bound, and permit motion only after released feedback. A stopped but enabled servo retains release. Disabling drops the coil. Missing feedback times out; stale, reversed-sequence or lost contact after admission faults and de-energizes. Faults latch until reset, which requires a new feedback epoch. No cylinder/brake dynamics live here.
- The registered standalone native test passes with assertions, warnings as errors and exceptions disabled. It covers stale true feedback, physical false feedback during release, fresh admission, normal hold, stale/lost feedback, fault latching, reset, disabling, missing feedback timeout, late acknowledgement, future timestamps, reversed clock and invalid policy.
- At `9dfc9bf1f` this state machine was not yet wired into `RobotRuntime`; the runtime integration below now covers coil ownership and reference-clock gating. Physical bindings and Z checks remain required.
- Removed the remaining prefix-sized memcpy and obsolete missing-field wording from `runtime_abi.hpp`. All three callers validate the current complete blueprint before copying. The native runtime rebuild, ABI validation and C API tests pass against the rebuilt library (`/tmp/mt-brake-interlock-native-build.log`, `/tmp/mt-brake-interlock-validation.log`, `/tmp/mt-brake-interlock-c-api.log`).

## R3 typed configuration work in progress

- Current-layout native and Haxe brake entries bind a mechanical joint, dedicated digital release channel and dedicated released-contact sensor, with explicit controller timeout and feedback age bounds. Physical torque and contact delays remain outside the runtime blueprint. Bindings are regenerated for the complete new layout.
- Validation rejects duplicate joint/output/feedback bindings, an energized safe value, ordinary-stop de-energization, a feedback sensor on another joint and invalid timing bounds. Program events cannot write a controller-owned release output.
- The native rebuild, ABI validation and C API checks pass (`/tmp/mt-brake-config-native-build.log`, `/tmp/mt-brake-config-validation.log`, `/tmp/mt-brake-config-c-api.log`). The absolute-path app build passes (`/tmp/mt-brake-config-app-build.log`), validating typed Haxe lowering with the regenerated interface.
- Runtime integration below executes the handshake, gates trajectory advancement and sends stationary references during release. Physical simulation/assembly binding, schema migration and Z-axis fault/reset/gravity checks remain open. Configuration validation alone does not prove motion interlocking.

## R3 runtime motion gate

- RobotRuntime owns the digital release output and executes the feedback handshake before advancing any trajectory or stop-ramp reference clock. While waiting it preserves a stationary commanded pose with zero reference velocity, interval-end velocity and feedforward. Normal stops retain release; emergency stops disable it. Explicit safety reset starts a new feedback epoch.
- New sensor samples are timestamped privately in the owner-clock domain for freshness checks. Public receipt timestamps retain their wall-clock meaning; virtual simulation and wall-clock time are not subtracted from one another.
- Dedicated native faults distinguish release timeout and contact/feedback loss. Program events cannot write the release channel; controller writes use an explicit controller event cause. Device-owned trajectory queues are rejected because the host cannot gate their independent path clock without a device-side handshake. Raw segment submission cannot bypass that restriction.
- Registered runtime checks verify frozen path time and stationary Hermite endpoints through release, fresh admission, normal stop, emergency stop, reset followed by missing-feedback timeout, stale feedback and lost contact. The rebuilt native runtime, complete existing runtime suite, ABI validation, C API and standalone handshake tests pass (`/tmp/mt-brake-gate-native-build-final.log`, `/tmp/mt-brake-runtime.log`, `/tmp/mt-brake-gate-runtime-suite.log`, `/tmp/mt-brake-gate-validation.log`, `/tmp/mt-brake-gate-c-api.log`).
- Physical simulation binding is implemented below. The physical part, assembly/model migration, actual mill Z gravity tests and complete closing gate remain required.

## R3 physical simulation integration

- A separate typed simulation descriptor supplies holding effort and contact delays. It must match every controller brake binding exactly. The simulation owns the physical model; the runtime owns only release output and feedback admission.
- Coil commands travel through the session/host command queue and contact feedback through joint snapshots. Physics callbacks never access a world from the wrong owner thread. Native and Haxe bindings use the complete new descriptor and snapshot layouts.
- Idle and normally stopped braked servos retain powered position hold. Emergency stop removes motor effort throughout the braked transmission group. Reset anchors its hold to the actual observed stopped pose before releasing again, including sag during application delay.
- Applied brake friction uses the physics timestep's two-step response floor and MuJoCo's maximum supported impedance, restoring the joint's original friction response when released. This is separate from transmission coupling stiffness. See [MuJoCo friction solver documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#friction).
- The registered integrated gravity test passes for a 10 kg prismatic load: a 150 N brake drifts 0.147 micrometres over 0.3 s after application; an undersized 50 N brake slips 693.96 mm. The 30 ms application delay allows 12.34 mm of sag before the rated brake settles. This generic fixture is not evidence for the actual mill Z assembly.
- Recompiled runtime, brake-runtime, ABI validation, C API, SimCore, SimKit MuJoCo and RobotKit MuJoCo suites pass against the rebuilt libraries. Actual mill hardware/bindings and the closing gate remain open.
- The absolute-path app build passes with the complete new simulation descriptor and joint snapshot layout (`/tmp/mt-brake-physical-app-build.log`). Additional SimCore C API and RobotKit kinematic simulation tests pass after recompilation.

## R3 authored brake model in progress

- An authored brake binds its mechanical motor joint, release output and released-contact sensor, with physical rating/delays and separate controller timing policy. Compilation produces separate controller and physical simulation records; duplicate ownership, missing contact and shared pneumatic/speed outputs are rejected.
- RobotModel moves to v12. Its remaining feature-selected v9/v10/v11 writer and older-version reader are removed. In-repo authored model fixtures and the deployment model migrate to v12; unsupported-version rejection fixtures remain unsupported. This completes a compatibility cleanup missed by R1 rather than introducing a brake-only format.
- Actual mill parts, mechanical assembly binding and Z-axis verification remain open.
- RobotKit world passes at 5,006 assertions (`/tmp/mt-brake-model-robotkit.log`), including the authored brake round trip, separate lowering, old-schema rejection, wrong-contact rejection and duplicate ownership rejection. Other suites and the complete closing gate remain pending for this schema change.

## R3 physical mill assembly in progress

- Replace the unused axis brake boolean with an installed assumed 4 N m spring-applied shaft brake. Its 10 mm hub and 2 mm disc turn with the Z screw; fixed front, shell and back sectors seat on the motor plate. Sixteen sectors per stator layer keep each convex collision hull clear of the rotating hub/disc, with 0.1 mm released axial clearance. The steel mass allocation, envelopes, press-fit hub and 50/30 ms release/application delays are assumed reference values.
- The 19 mm Z shaft seats through the brake and into a 32 mm coupling. Coupling length is derived from the shaft length, 8 mm mounting plate, brake depth and 1 mm assembly clearance; engagement must exceed the explicitly assumed 0.75 bore-diameter minimum. The front brake mounting does not hide a brake inside the existing motor casting.
- A saved motor binding references the installed brake part and separate controller timing policy. Physical rating and delays are derived anew from that part when exporting. Inclusion, nested rebuilding, flattening, document rebuilding and controller ownership preserve and qualify the release/contact names. The existing drive bus carries release and integrated-contact feedback; there is no unrelated field sensor or application-owned brake state.
- MachineKit descriptions move to v16 and ProjectKit assembly definitions/states to v8. New optional wire fields have new ids, with only the current versions accepted. No tracked old assembly-definition documents required migration.
- MachineKit smoke passes with the installed brake (`/tmp/mt-brake-assembly-smoke.log`). Focused checks pass for every part recipe, saved motor binding, nested rebuild, document round trip and two separately scoped brake-equipped modules (`/tmp/mt-brake-binding-tests.log`). The app build passes. Actual mill gravity/stop/fault/reset checks and the closing gate are still being verified.

Manual session clock correction: zero owner timestamps now select the fixed-step clock once for all participant phases. Previously control substituted simulation time but publication recorded brake-contact arrival at zero, so a healthy release always timed out. Native session phase checks and explicit/implicit-clock brake gravity tests pass. The actual mill now completes healthy release, movement, normal stop, emergency stop and reset: 4 N m brake, 0.665237 mm emergency-stop sag, 0.000020013 mm settled drift over 0.5 s. Release-failure load-hold verification remains open.

Fault-output correction: a release-timeout emergency command was staged correctly but `fail_tick` discarded it before noticing that safety was already faulted. Owner-locked failure handling now preserves staged emergency outputs for already latched faults/stops, while ordinary failed ticks retain rollback and fault conversion. The new native jammed-release regression reproduced 0.258145 N motor effort before the fix and reports exactly zero after it; its applied 150 N brake holds the 10 kg load. Session clock fix commit: `61fd8f6af`.

Current physical mill verification: healthy release, commanded Z travel, normal stop, emergency stop, reset and a jammed release all pass in `/tmp/mt-mill-brake-verified.log`. Native complete runtime checks pass in `/tmp/mt-brake-fault-runtime.log`; the gravity/jammed-release regression passes in `/tmp/mt-brake-jam-fixed.log`. The fault-output correction is commit `144010183`. The hub now has 0.1 mm axial clearance from each fixed end; role connectors and solids derive their section boundaries from one helper. Assembled CAD checks verify shaft grip, coupling gap, stator mounting contact and rotor clearance from the plate and stator. Their smoke run is pending.

Installed brake geometry smoke passed (`/tmp/mt-brake-geometry-smoke.log`), including actual stator/plate mounting, shaft seating, rotor clearance, recipe/document/nested binding round trips and the registered reach study. Rapids remain 15.095544/25/21.521827 m/min; Z guide utilization becomes 0.636247 from 0.630236 because the assembled brake contributes mass. The full app first exposed the unrelated 64-child compound limit while building the larger fixed link. `295d426ea` removes this arbitrary cap from dynamic compounds. Complete native MuJoCo checks pass with an 80-part compound, retained concave opening and a load supported by its final part (`/tmp/mt-large-compound-mujoco.log`). Mill cycle verification is rerunning after this fix. All temporary native diagnostics have been removed.

Physical mill checkpoint passed (`/tmp/mt-installed-brake-mill2.log`, exit 0): bare/enclosed cycles 52.57/53.22 s, versus paired-guide 52.54/53.19 s. Both grow 0.03 s after brake release sequencing and the changed rotating assembly. X/Y/Z tracking peaks at 10 ms are 0.085940/0.051497/0.060011 mm. Rapids remain 15.095544/25/21.521827 m/min; accelerations are 29.77/28.24/27.16 m/s² (Z grows from 26.44 because the shorter coupling reduces rotating inertia despite the added rotor). Bare removal is 3860.657927 of nominal 3846.780671 mm³, leftover 7.130270, gouge 0.657927, no rapid or holder contacts, 78 plans/55 flagged. Enclosed removal is 3860.667926 mm³. Door open/close remains 1.50/1.79 s; seated-blank clamp remains 0.08 s at 482.548632 N. The bare/enclosed artifacts contain 129/178 physical parts. With the final hub axial clearance, actual brake sag is 0.665314 mm and settled drift 0.000020016 mm over 0.5 s. All six review groups are implemented; the complete sequential closing gate and baseline audit remain.


## Final verification — 2026-10-05

Done. What was built and decided:

- `mt-suite-review-closing-brake.txt`: all eleven kits, CadKit, MachineKit smoke, the absolute-path app build and complete project-source run exit 0. The app log has no uncaught exception. Production code was unchanged during the run.
- Final kit counts: RobotKit world 5,006; MotionKit 67,535; CadBridge 157; CncKit 317; CamKit 12,311; ProjectKit 144; ProcessKit 23 plus weld-path 19. ProjectKit's reduction from the original 146 reflects intentional deletion of legacy assembly/scene compatibility checks, offset by new current-format tests; no current-format test was skipped.
- Twelve native suites were recompiled against current headers and run sequentially against the app's rebuilt libraries: runtime, brake runtime, ABI validation, runtime C API, SimCore, session, SimCore C API, MuJoCo backend, RobotKit MuJoCo, RobotKit kinematic simulation, physical brake simulation and release handshake. All exit 0 (`/tmp/mt-closing-native-summary.txt`). The gravity fixture verifies both explicit/default clocks, zero motor effort after release timeout, rated holding and insufficient-brake slip. The 80-part MuJoCo compound verifies its gap and last-child load support.
- SimKit, runtime and runtime–simulation binding checks all exit 0 (`/tmp/mt-closing-bindings-summary.txt`). Regeneration updates only the session clock documentation in the SimKit interface; ABI declarations are unchanged.
- Final bare/enclosed mill cycles are 52.57/53.22 s, with X/Y/Z peak tracking 0.085940/0.051497/0.060011 mm at 10 ms. Rapids are 15.095544/25/21.521827 m/min; programmed pocket feed is 1.2 m/min, entry feed 0.3 m/min. Brake release/fault/stop/reset checks pass; emergency-stop sag is 0.665314 mm and settled drift 0.000020016 mm over 0.5 s. Door remains 1.50/1.79 s; seated clamp remains 0.08 s at 482.548632 N.
- Existing post-main/review baselines reproduce: screw/belt router 232.2/215.2 s, 128 plans, 0.05/1.76 mm worst deviation; CoreXY 649.6 mm/s and 71.549 m/s²; arm 5.8/12/17.2/23.5 s; mobile final goTo 61 s and obstacle round 65 s; welder 20.6/21.1 s seams, 29.8 s post and 106.5 s whole weldment on both backends. Cobot classes all take 2.06 s with the recorded 0.004640/0.005987/0.011236/0.016598 mrad peaks.
- Reach remains Reach850 on a 600 mm riser: fifteen cell-derived poses, 16.239926% minimum joint margin. Two cell controllers own sixteen disjoint joints. The new physical brake and paired-guide changes intentionally alter mill mass, inertia and cycle figures as recorded above; other example baselines do not move in this closing gate.
- No new confirmed haxeon compiler defect was found, and haxeon was not patched. The section-return expression avoids a captured-local definite-assignment diagnostic; the earlier enum `Array.contains` claim remains unconfirmed.
- Current formats are MachineKit v16, ProjectKit assembly/state v8 and RobotModel v12. Old acceptance paths are removed and in-repo producers/fixtures migrated. No push or merge into main was performed. MT7–MT12 remain planned; live monitoring of moving foreign obstacles and device-owned brake queue handshakes are not introduced in this handoff.
