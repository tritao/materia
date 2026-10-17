# Electronic gearbox

A host-owned runtime can add measured master motion to an independent servo joint:

`q_target = q_planned + sum(ratio * encoder_prediction)`

The trajectory polynomial remains the additive coordinate. This is a programmable
follow relation, separate from the model's mechanical couplings. Followers must
have no mechanical coupling, and masters may themselves be measured followers,
provided the graph has no cycle. There are at most eight distinct masters per follower; follower indices are within
the existing 64-joint servo range. Masters can address any observed runtime joint.

`toolpathkit.path.ToolpathOp` has `ElectronicGear(follower, terms, provenance)` and
`ReleaseGear(follower, provenance)`. Joint names are resolved against the installed
RobotModel by `toolpathkit.motion.GearBinding.install`. It reads encoder counts and
driver position-loop rates. The caller supplies the controller owner frequency,
additional encoder transport delay and an explicit list of independent follower joints. Install the returned safe-release process channel
declarations when compiling the robot runtime. The owner must run at least as fast
as the declared gear rate; a slower runtime rejects the plan. No data changes or
model rebuild are needed when a later program changes the ratio.

Each encoder prediction quantizes the measured joint position to its count size,
then adds measured velocity times (owner sample age + declared extra latency).
Additional latency excludes time already represented by the observation timestamp.
No commanded master coordinate enters this calculation. Positions are unwrapped;
endpoints must supply continuous joint observations, not modulo-one-turn readings.
Disabling prediction is available on the runtime GearTerm for diagnostic comparisons.

The process-event ABI is unchanged. Channel `motion.gear/<runtime joint index>` has
safe value `Process("gear.release", 0)` and SAFE_ON_STOP. Engagement is a group of
KEEP events at the same path timestamp:

1. `gear.begin`, argument 0
2. `gear.loop`, argument required loop frequency in Hz
3. For each master: `gear.term/<index>/<extra latency ns>/<predict 0 or 1>`, argument ratio;
   then `gear.quantum`, argument SI joint units per count
4. `gear.engage`, argument 0

Only the last event replaces the live relation. Incomplete packets, cycles, duplicate
masters, invalid count sizes or latencies, and non-KEEP gear events are rejected.
`gear.release` removes the relation. Authors must align additive phase at engagement
and release; release does not manufacture a compensating trajectory or re-zero a joint.
The native submit path validates each complete packet before queuing it.

Each active owner tick emits a process event record on the gear channel with command
`sync` and argument measured follower minus the predicted master sum and additive
position, in SI joint units. It compares at the observation epoch (the additive part
is projected back over sample age using its velocity). These records use the existing
bounded event buffer and overflow flag; callers must drain them for long recordings.
For accelerating additive motion this diagnostic uses a first-order time projection.

Feed hold slows additive travel while the gear follows a continuing process master.
Stop, abort, faults and queue underflow release gears. Controlled stops begin from
measured physical positions and velocities, rather than the additive path's rest point.
The existing brake interlock gates outgoing geared motion. Device-owned trajectory
queues are explicitly unsupported: they need a device-side gearbox implementation.
There is no physical device gearbox upload or implicit resampling of device programs.

`PlanCheck` consumes the same timed event groups and checks both sides of event
boundaries, in addition to its ordinary sample grid. It adds master velocity and
acceleration before calculating follower speed, motor torque, RMS torque and elastic
loads. Trajectory masters use polynomial derivatives. A process master uses its
programmed speed/direction channels and a required signed acceleration declared in
`PlanCheckOptions.processAccelerations`. Zero is appropriate only after the spindle
has settled. Missing process acceleration is an error, rather than a zero-torque
assumption. Gear speed diagnostics name the follower joint. Planning does not alter
the master spindle ramp to make an invalid ratio fit.

## Reference checks

`robotkit_electronic_gear_tests` exercises the native owner, measured-master phase,
atomic packets, feed hold/stop, multiple masters, cycle rejection and per-tick records.
At 3000 hob rpm with ratio 1/200, 2^23 encoder counts/rev, 4 kHz owner/driver, and
1 ms declared additional encoder delay:

- Maximum encoder prediction error over two seconds is about 0.000309 arcsec.
- An explicitly assumed .02 kg m², 200 Hz critically damped servo at 4 kHz stays
  below 1 arcsec in the steady 15 rpm cutting interval (about 0.000020 arcsec).
- Disabling prediction gives omega * latency, 47.124 micrometres at 30 mm radius.

These are deterministic assumed reference checks, not a measured machine accuracy
claim. They omit cogging, encoder eccentricity, cutting load and structural compliance;
HB10/HB11 add those physical effects. The reference servo starts at steady velocity.

`machinekit/tests/gearing/haxeon.json` tests lowering, ratio changes, release,
combined speed, torque, event boundaries and missing acceleration declarations.
The broader ToolpathMotion test project is compiler-checked for enum exhaustiveness.
