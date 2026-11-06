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
- G2 — Hobbing command binding: pending. Keep the declared speed/direction channel convention; convert spindle RPM at
  output lowering using both the mechanical relation and the drive's declared radians-per-speed-unit. Keep feed and
  synchronization calculations in spindle coordinates.
- G3 — Encoder timing: pending. Bind the master to the encoder on the actual hob joint, independently of the motor's
  feedback encoder. The explicit controller rate schedules master sampling; follower timing remains bounded by its servo
  position loop. At-speed, tooth-pass windows and synchronization must continue to read measured B state.
- G4 — Regression and acceptance: pending. Cover direct, belt-coupled, reversed and chained drives, non-RPM speed units,
  and ambiguous/unsupported relations. Verify app tests, headless cutting playback, hobbing and machine-tending
  simulations, hobber tests (at least 2333 assertions), and bench-mill frame tests (917 assertions).

### Baseline and decisions

- Work is confined to `/media/joao/DATA1/materia-fe`, branch `hob-drive-sim`; unrelated dirty paths are excluded.
- The checkout's release HashLink was missing; built it with the prescribed CMake release commands. OCCT is reused from
  `/home/joao/dev/materia-cache/occt-f1dc4efb-release`.
- The app builds (3328 source files). The relative-path example launcher fails before playback (`Materia project file not
  found: null`); use the existing project-source hobbing test and absolute project paths for acceptance. The focused
  baseline test could not compile: `ProjectSourceTests.hx:7044` accesses nullable `generated.assemblyDefinition` after calls
  invalidate its earlier null check (E1005, in unrelated tending code). No change made there. The absolute-path app
  baseline is queued through heavy-run; log: `/media/joao/DATA1/claude-hobber-tools/tmp/hob-drive-baseline-absolute.log`.
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
