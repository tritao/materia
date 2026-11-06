# Hobbing simulation: spindle drive through the belt (2026-11-06)

**Symptom.** Opening the flexspline hobber and pressing Play logs "Simulation rebuild rejected: Hobbing needs one declared spindle
velocity drive".

**Cause.** `app/src/HobbingProgramPlayer.hx` (~line 123) picks the spindle velocity drive by
`drive.joint == indices[4]`, the hob joint B. Since the B belt drive landed (`HobSpindle`), the hob drive is bound to the servo's own
joint `spindle/motor-turn` (`FlexsplineHobber.B_MOTOR`, `bind("hob-drive", B_MOTOR, "spindle/motor", true, spindle.ratio)`). That
joint drives B through the transmission `hob-belt` (`Transmission.BeltReduction`, 1:1 today). So no velocity drive sits on B.

**Fix (long-term clean).** Resolve the spindle drive through the machine's transmissions: the velocity drive whose joint is B, or that
drives B through a transmission chain. Convert commanded speeds and directions through the chain's ratio and sign. Use the existing
transmission/coupled-joint machinery (MotionKit/RobotKit blueprint transmissions), not a hobber special case. Keep the
speed/direction channel naming rule. Check the other places that assume the hob drive is on B: the encoder master (the B master is
now the cartridge encoder, while `hob-motor` is the servo's own encoder), the at-speed channel, and the tooth-pass and sync timing.

**Acceptance.**
- The hobber Plays its cutting scenario in the app.
- The hobbing app tests pass, and a regression test covers a belt-coupled spindle drive.
- The existing hobbing and machine-tending simulations still pass.

## Status

- G1 — Shared spindle drive resolution: complete. Resolve the process drive in toolpath motion using MotionKit's existing
  coupling projection, including inverse traversal, chained ratios and direction. Reject absent or ambiguous drives and
  joints whose rates cannot be related by one scalar.
  Validation: `app/tests/haxeon.spindle-drive.json` passes 15 checks (direct, reversed 3:1 belt, inverse and forward
  relations, chained reduction, radians/second channels, missing/ambiguous drives, differential, cycles and channel names).
- G2 — Hobbing command binding: complete. Keep the declared speed/direction channel convention; convert spindle RPM at
  output lowering using both the mechanical relation and the drive's declared radians-per-speed-unit. Keep feed and
  synchronization calculations in spindle coordinates.
  Validation: the focused spindle-drive project passes 21 checks, including lowered start, running speed/reverse and
  stop commands through a reversed chained reduction, and the existing `.at_speed` startup barrier. Unbound CNC
  channels retain their ordinary RPM units; a bound physical drive supplies the explicit conversion. The toolpath's
  modal spindle RPM remains physical spindle RPM for work-revolution feeds.
- G3 — Encoder timing: complete. Bind the master to the encoder on the actual hob joint, independently of the motor's
  feedback encoder. The explicit controller rate schedules master sampling; follower timing remains bounded by its servo
  position loop. At-speed, tooth-pass windows and synchronization must continue to read measured B state.
  Validation: the focused project passes 21 drive/lowering checks and six encoder checks. A master with no actuator
  binds its cartridge encoder at the explicit controller rate; the table keeps its own feedback and servo ceiling.
  Equivalent encoder aliases are accepted only when their resolutions agree, conflicting master resolutions are
  rejected, and a follower cannot borrow feedback from another joint.
- G4 — Regression implemented; acceptance incomplete. Actual app Cutting succeeds, but the required tending gate
  has pre-existing joint-limit and door-jam recovery faults. It must not be reported as passing.
  - `app/tests/haxeon.json`: final full suite passed, exit 0 (`hob-drive-stable-app-tests.log`).
  - `machinekit/tests/hobber/haxeon.json`: final full suite passed, 2333 assertions
    (`hob-drive-supported-belt-hobber-tests.log`).
  - `machinekit/tests/hobbing/haxeon.json`: final repeat passed, 1161 assertions, including two stationary belt-path checks
    (`hob-drive-stable-hobbing-tests.log`).
  - `machinekit/examples/bench-mill/haxeon.frame-tests.json`: final repeat passed, 917 assertions
    (`hob-drive-stable-bench-tests.log`).
  - Full native MuJoCo tests pass. New elastic reversed 2:1 drive cases cover stopped/forward/reverse/stopped commands
    with the transmission declared in either direction, load sag, velocity accuracy and effort bounds. The new regression
    fails against the original native library, proving it detects the defect (`hob-drive-velocity-native-run.log`,
    `hob-drive-velocity-native-baseline.log`). Assembly-integrity tests also pass, including supported and unsupported
    stationary belt envelopes (`hob-drive-belt-support-tests.log`).
  - Headless app Cutting from measured initially open doors completes: 963 tooth-pass windows, 5.9725 simulated job
    seconds, 562.59 N peak modeled tangential load, 0.0108012 arcsec maximum C phase error. This passes the existing
    two-table-encoder-quanta gate. The binding resolves `machine/spindle/B` through `machine/spindle/motor-turn`,
    at `0.10471975511965977` spindle rad/s per channel unit. The final cutting-only run passes 23898 checks, exit 0 (`hob-drive-cutting-acceptance.log`).
    The optional third test argument `cutting` selects this acceptance scenario; the default unified test still runs all scenarios and their assertions.
  - UI-tree evidence: 414 rows, 10 groups and six presentation views in
    `/media/joao/DATA1/claude-hobber-tools/tmp/hob-drive-evidence/hobber-ui-tree.json`. The views are Assembled,
    Tool contact (cutaway), Workpiece, Loading, Enclosure transparent and Enclosure hidden. This worktree has views,
    rather than a separate sets schema. No scene-group/view edits are included.

  Corrections exposed by real playback:
  - Start tooth-pass sampling at measured spindle readiness, then retain every sample even if speed drops. Pre-spin
    belt settling and a negative initial phase are valid; a real reversal after readiness still fails. Four focused
    sampling checks pass. Job readiness requires spindle standstill; the absolute cartridge phase remains the gear
    master, without encoder rebasing or a B=0 requirement.
  - The complete closed belt loop previously orbited with B, including its mass and collision hull. A 16-angle audit
    found 1.4176 mm penetration into the bed (`hob-drive-collision-parts.log`). Its envelope now stays on the housing;
    both pulleys still rotate. Physical support is checked at the declared belt-path contacts. The assembly-order test
    records fitting the belt after fastening the motor flange, whose wrench access the loop obstructs.
  - Velocity inverse dynamics previously used only the motor's inertia/gravity, leaving the cartridge creeping at about
    -0.00975 rad/s and preventing measured standstill. The existing transmission-coordinate map now lifts motor
    acceleration to the coupled load and projects load inertia/bias effort back to the powered joint. Physical
    compliance/sag and amplifier effort limits remain active. No standstill or synchronization tolerance was relaxed.
    An earlier RobotKit predictor experiment was reverted completely.

  Remaining failures and decisions:
  - Full load/cut/unload repeats Cutting successfully (0.0108032 arcsec), then fails the existing measured loading-park
    X/Y/Z witness. Final measured park is X=109.9999999999 mm, Y=0.0001102694 mm, Z=-0.0103044546 mm;
    Z exceeds the existing ±0.010 mm witness by 0.304 µm (steady initial sag is about 10.7 µm). The default full-cycle test keeps this failure visible. No scale wiring or park tolerance changed.
    Evidence: `hob-drive-final-full-cycle.log` (and the earlier `hob-drive-load-compensated-playback.log`).
  - The original tending harness now compiles with the pinned compiler: two switch subjects save observable channel
    reads in locals, and a checked local preserves the nullable assembly definition. Its project manifest includes the
    missing casting-shop source root. No assertions were removed and no compiler source/pin changed.
  - The current worktree's tending certificate was stale. Revalidation certified all 96 saved poses and all production
    paths; its output differed only in the fingerprint. That revalidated fingerprint was used temporarily for testing,
    then restored; the certificate is not part of this change. The `cell` case passes (two controllers, 65 disjoint
    joints). The `interface` case faults during CNC load-position movement before `cnc.ready`: joints 33/34 exceed
    travel. The original native backend reproduces the same interface joint-limit fault. Production, run independently,
    passes emergency-stop, empty-infeed, missed-grasp and dropped-part recovery, then fails door-jam recovery
    (step 5 waits for `cnc.door_open`; request=false, door=-0.00004667176760850077 m, timeout after 6 s).
    Enforcing the original backend through a temporary loader audit reproduces that production failure exactly.
    A runtime library map verifies the original backend alone after load. Earlier preload-only comparisons were
    superseded because they could load a second library later; those runs are not baseline proof. Evidence:
    `hob-drive-tending-revalidated-tests.log`, `hob-drive-tending-production.log`,
    `hob-drive-tending-interface-original-only.log`, `hob-drive-tending-production-original-only.log`,
    `hob-drive-tending-production-original-only-maps.txt`, `hob-drive-original-backend-audit.c`,
    `hob-drive-tending-revalidation.log`. Neither tending case is counted as passing.
  - Further tending diagnosis identifies native joints 33/34 as X-right cover slides, not the controller's
    similarly indexed Z covers. A fresh saved artifact preserves retry steps through the codec and generator;
    retry metadata is not the door-recovery cause. The travel fault is independently observable during CNC startup.
    Stronger ideal-constraint impedance fixes the startup probe but breaks the existing screw-clamping regression.
    A 4 kHz integration refinement passes isolated cover-chain and unchanged native screw tests, yet the actual
    production scene still reports cover travel faults (-0.0188601/-0.0188638 m, bounds ±0.0178571 m plus the
    unchanged 1 mm safety margin). Both experiments were discarded, including their temporary native tests/helpers.
    The diagnostic production process was stopped after reproducing the travel fault; it is not a passing gate.
    Evidence: `hob-drive-tending-startup.log`, `hob-drive-tending-startup-rigid.log`,
    `hob-drive-quarter-native-tests.log`, `hob-drive-refined-tending.log`. The committed native backend was restored
    and its full unit suite passed again, exit 0 (`hob-drive-restored-native-tests.log`).
    App tests also passed again (`hob-drive-refined-app-tests.log`). A fresh full hobber replay reproduced successful
    Cutting and the same measured Z-park failure (`hob-drive-refined-full-cycle.log`); fresh UI evidence is in
    `hob-drive-refined-evidence/hobber-ui-tree.json`. No failed solver experiment is retained.
  - An additional CAM fixture check fails with `Motion program op 1: Null access`; the unchanged test copied from Git
    reproduces it before spindle execution (`hob-drive-cam-baseline.log`). It is not a passing acceptance gate.

  All log basenames above are under `/media/joao/DATA1/claude-hobber-tools/tmp`.

### Baseline and decisions

- Work is confined to `/media/joao/DATA1/materia-fe`, branch `hob-drive-sim`; unrelated dirty paths are excluded.
- The checkout's release HashLink was missing; built it with the prescribed CMake release commands. OCCT is reused from
  `/home/joao/dev/materia-cache/occt-f1dc4efb-release`.
- The app builds (3328 source files). The relative-path example launcher fails before playback (`Materia project file not
  found: null`); use the existing project-source hobbing test and absolute project paths for acceptance. The focused
  baseline test could not compile: `ProjectSourceTests.hx:7044` accesses nullable `generated.assemblyDefinition` after calls
  invalidate its earlier null check (E1005, in unrelated tending code). A checked local subsequently unblocked that
  access for the required tending gate. Absolute-path app baseline log: `/media/joao/DATA1/claude-hobber-tools/tmp/hob-drive-baseline-absolute.log`.
- Baseline reproduced in the unmodified app after generating a 59,569,269-byte hobber artifact:
  `Snapshot simulation rebuild failed: Hobbing needs one declared spindle velocity drive`.
  Set `MATERIA_ROOT=/media/joao/DATA1/materia-fe` for supplier CAD lookup (the project runs from its own directory).
  Diagnostic serial generation used `MACHINEKIT_THREADS=1`; no OCCT changes. Evidence:
  `/media/joao/DATA1/claude-hobber-tools/tmp/hob-drive-baseline-serial.log`.
- Drive resolution compares rates produced by the existing coupling projection for every independent coordinate. Only
  proportional coordinates have a scalar conversion; this handles inverse chains and shared roots without treating an
  independent input to a differential as a spindle drive. Coupling offsets never enter velocity conversion.
- No machine-specific drive names, pulley ratios, encoder fallbacks or alternate description formats will be introduced.
  A change of representation is unnecessary: the model already declares the process drive, couplings and encoders.
