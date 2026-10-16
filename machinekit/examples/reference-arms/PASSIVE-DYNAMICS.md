# Original scene: passive dynamics investigation

Pressing Play runs physics even though this scene has no motion program.
The six joints are passive. The converted model has zero joint damping,
dry friction and reflected motor inertia, with no actuators or brakes.
Gravity therefore drops the arm, without the dissipation or holding behavior
of the real geared, braked modules.

There is also a numerical problem. A 60-second MuJoCo test of the loaded
`build/arm-design/zeroerr-original.mtrg` state, with **all contacts disabled**,
produced the following results. Speeds are maximum absolute joint velocities
sampled every 0.1 seconds during the last ten seconds, in rad/s.

| Joint damping (Nm s/rad) | Timestep | Late maximum speed | MuJoCo instability warning |
| --- | --- | --- | --- |
| 0 | 10 ms | 362.609 | Yes, at 6.240 s |
| 1 | 10 ms | 0.00000108 | None logged |
| 5 | 10 ms | 0.002377 | None logged |
| 0 | 5 ms | 926.706 | Yes, at 14.275 s |
| 0 | 1 ms | 3440.496 | Yes, at 18.640 s |
| 0 | 0.5 ms | 379.718 | None logged; this is not evidence of accuracy |

The warnings explicitly report unstable QACC. Runtime snapshots nevertheless
return fault code zero, and no exception escapes. Finite returned positions
and a successful process exit must therefore **not** be interpreted as a
stable simulation. At the time of this diagnostic the backend called `mj_step` without promoting
MuJoCo instability warnings to a simulation fault.

These tests establish missing dissipation and reproduce numerical instability
without floor/self contacts. They do not establish its complete numerical
cause, measure energy conservation, or reproduce the app's contact behavior.
Reducing the timestep alone did not establish a reliable undamped result.
The arbitrary damping values are sensitivity probes, not vendor parameters.

The converted preview mass is 18.31055 kg; it still uses CAD-derived physical
properties. It is **not** the reconciled 23.826 kg whole-module load model (before the preview tool).
This is evidence about the preview, not a failed test of the vendor arm.

Next work should connect that whole-module mass/CoG/inertia model and the
appropriate brake/drive state to the preview, propagate numerical instability
as a fault, then validate free-decay and contacts separately. Joint friction
and damping require measurements or supplier data. Do not invent them to
declare the vendor model validated. Geometry remains frozen.

Reproduce with an existing built app (for its native physics libraries):

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/reference-arms/haxeon.json
bash machinekit/examples/reference-arms/run-passive-dynamics.sh
```

Inspect both `build/passive-dynamics/review.json` and `runtime.log`. The report
records source-scene, producer and bytecode hashes. Generated artifacts are
local build outputs; no saved scene, geometry or hardware setting is changed.

## Stationary inspection preview with brake-capable drives

Use `build/arm-design/zeroerr-braked.mtrg` for inspection with Play. It keeps
all supplier geometry and the saved pose, includes the six module brakes,
and has no motion program. The runtime automatically enables the servos and
releases the brakes after fresh feedback; the stationary result is a powered
hold. The original test checked drift and faults, but did not assert brake
contacts. Its earlier description as an unpowered brake-held test was incorrect.
The geometry-only `zeroerr-original.mtrg` remains available for reproducing the original passive behavior.

Each module's entire manual mass/CoG/central inertia follows its housing.
The other module pieces retain their visual/collision geometry with zero
additional mass; every simulated rigid body must still have positive total
mass. All 31 non-tool lumps match `reference_dynamics.py` independently in
mass, world CoG and inertia tensor to 1e-9 absolute tolerance. Arm mass is
23.826374 kg, with the existing 1 kg tool plus a 0.001 kg fixed bridge root.

Brake efforts use the existing nominal-100 diagnostic ratings (206/50 Nm); ratios remain unconfirmed.
Brake application/release delays are idealized as zero for the initial
diagnostic. Servo metadata uses existing simulator defaults to satisfy its brake interface and perform powered holding. It is not approved
powered-motion tuning or hardware commissioning. No joint friction is invented.

The backend now rejects ticks with bad-position/velocity/acceleration/control
warnings, and foreground/background app playback stops on that failure.
A forced-instability regression and the full native MuJoCo tests pass.
The actual app scene held for 60 simulated seconds with contacts enabled:
maximum joint drift 3.30e-8 rad, no reported faults.

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/reference-arms/haxeon.json
bash machinekit/examples/reference-arms/run-built.sh --braked-preview
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=machinekit/examples/reference-arms/build/arm-design/zeroerr-braked.mtrg
```

App regression: build `app/tests/reference-arms/haxeon.json` and run its
HashLink module with `--braked-reference`, using that build's native-library
paths. The exporter also writes `zeroerr-braked-inertials.json` for inspection.

## One slow J2 excursion

`build/arm-design/zeroerr-slow.mtrg` uses the same geometry, inertials and 1 kg
comparison tool. Play runs one J2 +5 degree excursion and returns, with
2 deg/s velocity and 4 deg/s² acceleration ceilings. It uses the existing
independent-axis planner, exact-stop moves and native brake interlocks.
It finishes in powered hold and does not loop.

The app regression `--slow-reference` observed 5.0206 degrees of travel,
1.9110 deg/s maximum speed and 13.6795 Nm maximum joint effort, with no
reported faults. All six brake-release contacts acknowledged. After return,
the test requested a normal stop, then exercised the existing latched
emergency-stop handoff **while stationary**: all six contacts confirmed
brakes applied, motor effort was zero, and drift over another 10 seconds
was 9.13e-10 rad. That test is not a new normal parking/disable API.

This is a simulation mechanism check using nominal-100 ratings, ideal brake
delays and simulator servo defaults, not hardware tuning or a 5 kg dynamic
qualification. The generated `zeroerr-slow-review.json` records observations.

```sh
bash machinekit/examples/reference-arms/run-built.sh --slow-preview
./app/run-built.sh --project=machinekit/examples/reference-arms/build/arm-design/zeroerr-slow.mtrg
```
