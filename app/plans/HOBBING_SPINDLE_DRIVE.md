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
