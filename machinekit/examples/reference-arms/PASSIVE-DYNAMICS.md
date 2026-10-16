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
stable simulation. The backend currently calls `mj_step` without promoting
MuJoCo instability warnings to a simulation fault.

These tests establish missing dissipation and reproduce numerical instability
without floor/self contacts. They do not establish its complete numerical
cause, measure energy conservation, or reproduce the app's contact behavior.
Reducing the timestep alone did not establish a reliable undamped result.
The arbitrary damping values are sensitivity probes, not vendor parameters.

The converted preview mass is 18.31055 kg; it still uses CAD-derived physical
properties. It is **not** the reconciled 23.826 kg whole-module load model.
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
