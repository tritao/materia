# Frozen reference: control integration and eRob stub

The frozen original model now uses the existing **AssemblySimulationBridge →
RobotKit joint model → MotionKit JointPathTiming** pipeline. Four original-arm
joint paths are timed with synchronized stopped edges, sampled, checked by the
whole-module inverse-dynamics model, and replayed through a six-axis SI eRob
transport boundary. This is command-level simulation, not a physical EtherCAT
master or a validated flexible-body plant.

## Profile and limits

`reference-control-profile.json` records the source and confidence of every
setting. It uses the manual figure 6-7 **candidate** ratios 120/120/120/100/100/100:
150°/s maximum at J1–J3 and 180°/s at J4–J6, subject to supplier confirmation for
this T-arm. Earlier all-100 assumptions remain labeled diagnostics; geometry is
unchanged.

The manual recommends acceleration/deceleration time at least 0.3 s. The profile
records the derived ceilings (requested velocity divided by that ramp time), not
an invented absolute vendor acceleration rating. This run requests 15°/s,
15°/s² and 100°/s³ jerk, below those derived ceilings. These settings are applied
to the converted RobotKit joint limits used by MotionKit.

**Vendor assembled-arm position limits, zero/signs and harness travel are not
available in the supplied documents.** Their fields remain null. The ±170° window
is a labeled simulation-only diagnostic window. It is not commissioned travel.
The code refuses physical operation; even filling the flags cannot enable a real
master because none is implemented.

## Torque-aware timing

The model uses the original geometry and six manual whole-module lumps. Nominal
ratios set reflected rotor inertia using the documented ratio-plus-one convention.
The validator includes rigid-body gravity, mass matrix and velocity coupling.
For each actual timed sample it checks:

- Velocity and acceleration against the profile/requested limits.
- Continuous curve for non-accelerating joints; acceleration curve where that
  joint accelerates. Start/stop peak is separately checked.
- Motion-only RMS output torque against the stated rated-torque proxy.
- Speed-weighted cubic average gearbox load against its separate allowance.

If a timing fails, time scales 1/1.5/2/3/4/6/8 are tried with velocity divided by
scale and acceleration by scale², recomputing inverse dynamics. A persistent
failure names a load/curve/duty assumption to investigate; it never changes
geometry or silently removes a failed check. Slowing cannot cure static overload.

The curves are **published-plot conservative inner approximations**, not confirmed
digital curves for delivered T modules. They are pinned in
`reference-drive-curves.json`, with their original H-type plot scope and revision
gaps explicit. Finite sample checks, point-load inertia, omitted friction and
unmeasured thermal behavior prevent hardware qualification. Motion-only RMS is
not falsely reduced by inventing brake-held dwell for these four paths.

Each path is an independent simulation run initialized at its certified start.
No unchecked jump from one path's end to the next path's start is treated as a
motion. Existing original-arm continuous clearance applies to the stopped joint
edges; timing does not change their geometric locus.

## Driver boundary

`machinekit.control.erob.ERobDriveInterface` accepts seconds, radians, radians/s
and output Nm; it exposes timestamped feedback, stop and close.
`ERobDriveStub` records deterministic feedback, rejects wrong dimensions,
nonfinite values and non-increasing command times. Hardware arming is unavailable.
It does not open a NIC, send packets, release a brake or simulate servo tracking
error. Native review exercises hardware denial and stale-command rejection.

The future real backend must supply:

| Boundary | Delivered evidence / implementation needed |
| --- | --- |
| EtherCAT | Dedicated master, exact slave identity/order, ESI revision, distributed clock and watchdog behavior |
| CiA402 | Validated controlword/statusword transitions and CSP support; fault/quick-stop handling |
| PDOs | Delivered mapping, widths, units and scaling; no assumed register writes in the stub |
| Encoder | Actual counts per output turn, multiturn encoding, zero/sign convention; no motor-count substitution |
| Torque/current | Firmware-specific command/feedback units and motor/gear efficiency calibration |
| Timing | Owner clock, bounded command latency, cyclic period and deadline enforcement |
| Brake/STO | Vendor enable/load-transfer/brake delays and independent safety circuit |

The example 19-bit output encoder is 524,288 counts per turn; it remains
uncommissioned. Never multiply output counts by gear ratio a second time.
The model's J1/J2 moving-housing convention must be reflected in command signs.

## Run

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/reference-arms/haxeon.json
bash machinekit/examples/reference-arms/run-built.sh --frozen-control
python3 machinekit/examples/zeroerr-arm/validate-reference-control.py
bash machinekit/examples/reference-arms/run-built.sh --frozen-control --replay-approved
```

Outputs under `build/reference-control/`: `timed-paths.json`, `validation.json`,
`approved-commands.json`, `replay.json`. The validator checks profile, source-path
and timing-producer hashes; replay checks the profile and approved timed-source
hashes. Regenerate validation whenever model, curves, profile or timing changes.
No existing CobotArm or tending edits are involved.
