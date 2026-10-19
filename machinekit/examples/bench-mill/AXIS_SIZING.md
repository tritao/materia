# Mechanical mill motor sizing

The installed generic 400 W X/Y and 750 W Z motors pass the preliminary
local-model torque and speed screen under the assumptions below. They are
still assumed motors, not selected Leadshine motor/drive combinations. This
checks the mechanical `MillFramePreview` concept, not the separate legacy
`BenchMill` motion/control model.

Run from the repository root:

```sh
./haxeon/scripts/haxeon run --project machinekit/examples/bench-mill/haxeon.axis-sizing.json
```

The report is written to `build/axis-sizing/axis-sizing.json` inside this example.
Set `MILL_AXIS_SIZING_OUTPUT` to an absolute directory to override the destination.
The same run executes analytical and model consistency checks before writing it.
The JSON includes moving-member masses, rotating inertia contributions, duty phases,
all 55 Leadshine candidate comparisons, and the three spindle alternatives.

## Assumptions and calculation

The 20 kg combined workpiece/fixture payload, 500 N cutting load per axis and
1 m/s² acceleration match `MillDesignStudy`. New explicit study assumptions are
10,000 mm/min rapid, 3,000 mm/min cutting feed, 30 N extra guide/seal drag, and
1.25 torque reserve. The cycle is a 50 mm cutting traverse, 50 mm rapid return
and one second powered dwell (about 2.52 seconds). Moves use trapezoidal or
short-move triangular profiles. These are study inputs, not user-approved machine
performance requirements or measured cutting forces.

Moving mass follows each real axis's mate subtree. Y includes the nested X stage;
the workpiece/fixture payload is added to both X and Y, and not to Z. The spindle,
carrier and guard are included in Z. CAD mass uses the local component geometry,
material density or declared mass, with their existing approximation limits.

Screw lead, nut efficiency/preload drag, X pulley ratio and belt loss come from
the installed local components. Screw, retainer, coupling, pulley and rotating
brake inertias come from CAD about their local shaft axes. Motor shaft inertia
is not added again: the motor rotor rating already includes it. Belt moving mass
is excluded. No unverified rotor mass is inferred from a housing.

For travel per motor radian `q` in metres, reflected translating inertia is
`mass*q²`. Screw-side rotating inertia is reflected by the square of screw speed
per motor speed. For each phase, torque is bounded by:

```text
(gravity + opposing cutting + guide drag + mass*abs(acceleration))*q/efficiency
+ (reflected rotary inertia/efficiency + rotor inertia)*abs(acceleration)/q
+ running rotary drag
```

The phase torque bounds are integrated as `sqrt(sum(T²*seconds)/sum(seconds))`.
Gravity, deceleration and load torque magnitudes never cancel, so this is a
conservative RMS bound rather than a signed drive simulation. Powered dwell
includes gravity but no running drag. The report also checks sustained cutting
separately, so changing production duty does not silently inherit an intermittent
rating. A candidate can pass the example duty and fail sustained cutting.

RMS and peak use the 1.25 reserve. The speed screen stays within motor rated speed;
it does not extrapolate torque above rated speed. Nameplate continuous/peak torque
through that region remains a screening assumption until the matched drive curves
and peak-duration limits are verified. An inertia ratio of 10 is an explicit study
target, not a universal servo limit or a tuning guarantee.

Screw speed is compared with the existing Euler–Bernoulli critical-speed model,
BK fixed/BF simple supports, bearing-centre span and 80% first-bending-speed margin.
Screw DN, buckling and load capacity remain separate qualifications.

## Interpretation

The numerical tables below are a snapshot from the local model, rounded for review.
Regenerate the JSON after geometry, material, motor or transmission changes.

| Axis | CAD moving mass | Payload | RMS bound | Peak bound | Rated / peak available | Load/rotor inertia |
|---|---:|---:|---:|---:|---:|---:|
| X | 32.5 kg | 20 kg | 0.347 N·m | 0.659 N·m | 1.27 / 3.82 N·m | 1.21 |
| Y | 71.1 kg | 20 kg | 0.340 N·m | 0.677 N·m | 1.27 / 3.82 N·m | 1.80 |
| Z | 48.2 kg | 0 kg | 0.719 N·m | 1.176 N·m | 2.39 / 7.16 N·m | 0.71 |

Demand columns are before the additional 1.25 reserve. All installed axes pass
both the example duty and sustained-cut torque screens with that reserve.
All need 2,000 motor/screw rpm at the assumed rapid. The calculated screw limits
are approximately 11,780 rpm (X), 5,824 rpm (Y) and 3,073 rpm (Z); Z has the
smallest screw-speed margin. These limits do not qualify bearings or screw DN.

| Spindle alternative | Z moving mass | Z RMS bound | Z peak bound |
|---|---:|---:|---:|
| ELM2L-1500LA100FT-H MS1AE | 53.4 kg | 0.762 N·m | 1.225 N·m |
| ELM2M-2000LB130F-H | 56.1 kg | 0.785 N·m | 1.251 N·m |
| ELM2M-2900LD180FT-H | 66.6 kg | 0.875 N·m | 1.352 N·m |

All three still pass with the current 750 W Z motor under these study inputs.
That does not resolve their column-clearance, motor-removal or mounting issues.

The catalog comparison finds 200 W Leadshine models passing the example X/Y
load screen and 400 W braked variants passing the baseline Z screen. This is
not a recommendation to downsize: production duty, peak-duration curves, drive
matching, mounting and brake capacities remain unverified. Each candidate is
compared in isolation; swapping the X motor would also alter Y moving mass.

## Z holding and power loss

Baseline frictionless holding demand is about 0.376 N·m, or 0.470 N·m with
reserve. The heaviest spindle arrangement needs about 0.520 N·m, or 0.650 N·m
with reserve. The modeled external brake claims 4 N·m, but that is an assumed
component rather than a verified supplier rating. A free-fall upper bound over
its assumed 30 ms application delay is 4.41 mm from rest; rotor/screw inertia
and drag would reduce this. An already moving head needs additional stopping
distance. Holding torque alone does not establish emergency-stop performance.

## Spindle output target

For the earlier 2.2 kW at 8,000 rpm target, the assumed 2.2 kW motor supplies
only 2.09 kW after the assumed 95% belt efficiency. From the stored torque and
speed ratings, the 1.5 / 2.0 / 2.9 kW Leadshine alternatives supply approximately
1.46 / 1.92 / 2.78 kW at their rated points through the proposed ratios. Only
the 2.9 kW option clears this power screen; none is continuously qualified at
the target until its matched curve, duty, drive and transmission are verified.

## Remaining selection work

Confirm required feeds, accelerations, process loads, payload and production cycle.
Then choose actual motors and matching drives, check torque-speed/thermal curves,
validate brake timing and regenerative energy, and qualify screws, bearings,
couplings and the X belt. Torque sizing does not establish positioning accuracy,
servo tuning, cutting stiffness or following error.
