# Standalone robot-arm design baseline

The active work is arm design. CNC tending, cell reconfiguration and the inherited
23 transfers are deferred application studies. Their historical results remain
available, but they do not define acceptance for this arm or rank commercial robots.

The original ZeroErr 5 kg supplier geometry is the development baseline. UR5e and
FAIRINO FR5 are commercial references for checking our software and setting design
benchmarks. A reference model is not a substitute for a calibrated delivered robot.

The [module and mass audit](../zeroerr-arm/MODULE-MASS-AUDIT.md) records the
documented mass conflicts, part material corrections and standalone payload
sensitivity. Exact drive codes and moving-side mass data remain unconfirmed.

## Requirements sheet

The machine-readable source is [arm-design-requirements.json](arm-design-requirements.json).
It separates the user's target class from provisional targets and diagnostic assumptions.

| Item | Working basis | Status / verification |
| --- | --- | --- |
| Flange load | 5 kg total, including tool and workpiece | Target class; not 5 kg plus a tool |
| Reach | At least 850 mm nominal; references span 850–922 mm | Provisional target; oriented usable reach must be checked separately |
| Joint speed | 180 degrees/s peak class benchmark | No assumption of simultaneous maximum speed, acceleration and payload |
| Repeatability | 0.03 mm aspirational benchmark | Requires external metrology; numerical FK residual is not repeatability |
| Joint travel | Not frozen | Confirm physical zero/signs, stops and harness travel; investigate self-collision |
| Duty / TCP accuracy / stiffness | Not frozen | Define motion duty, accuracy/error and deflection budgets before release |
| Tool interface | Revision-matched flange and adapter | Check pilot, bolt/dowel patterns, mating face and connector convention |
| Load screening | 4 kg payload + 1 kg tool; combined CoG 0/100/150 mm | Provisional point-load scenarios, not vendor payload-envelope claims |
| Acceleration / design reserve | 60 degrees/s², factor 1.2, +25% arm-mass case | Screening assumptions; not commercial specifications |
| Original joints | Nominal-100 110T/70T and catalog/exterior-volume mass split | Unconfirmed ordering codes and moving-side masses |

Class benchmarks are documented in the [UR5e specifications](https://www.universal-robots.com/manuals/EN/HTML/SW10_6/Content/prod-usr-man/hardware/arm_e-Series/UR5e/H_g5_sections/appendix_g5/tech_spec_sheet.htm)
and [FR5 specifications](https://www.fairino.com/FR/4.html); those nominal
figures do not qualify our prototype.

Keep the original supplier axes and mounting geometry while resolving these data.
Enlarging housings, joints or links must follow a demonstrated packaging/load need.
The earlier 80T wrist and straight-adapter variants remain separate experiments.

## Reference validation hierarchy

1. **External numerical oracle:** the published UR5e nominal DH table, implemented
   separately from the URDF importer and existing FK/IK code.
2. **Independent drawing checks:** FR5 shoulder/link/wrist dimensions and nominal
   flange face from its pinned manufacturer drawing archive.
3. **Source consistency:** independently multiply the pinned source URDF transforms
   and compare with the native imported model. For ZeroErr, independently rotate
   the STEP-derived zero axes and flange frame.
4. **Software regressions:** finite-difference Jacobians, FK/IK round trips,
   limit-respecting turn-equivalence checks and known positive basic-motion cases.
5. **Physical validation, still pending:** vendor-controller samples, serial-number
   calibration, actual collision clearances, motion duty and bench measurements.

[arm-reference-oracles.json](arm-reference-oracles.json) records the values,
frames, tolerances and source links. The [UR DH table](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)
uses the manufacturer's nominal model. Its base differs from ROS `base_link` by
Rz(pi); that explicit source-frame conversion is not fitted to test results.
This nominal oracle does not verify an individual robot's calibration.

FR5 is deliberately **not labelled as having an independent full FK oracle**.
Its available manufacturer drawing verifies dimensions, while full FK tests use
its own URDF as the source. These are different evidence levels. Vendor-controller
FK samples or an independently documented full convention remain required.

Source discrepancies are retained:

- UR's general specifications give wider travel than the pinned ROS description's
  ±180 degree elbow. The suite preserves that conservative source limit.
- FR5's nominal 395 mm forearm and 102 mm wrist dimensions differ from source
  values by 0.01 and 0.1 mm; its source rotations are rounded. Drawing checks allow
  0.2 mm, with differences reported rather than changing geometry.
- FR5's nominal flange is 100 mm from the wrist origin; the source mesh ends at
  99 mm. The nominal flange transform stays separate from the unchanged mesh.
- Source inertias and effort fields are not verified continuous drive ratings.

## Standalone evaluation

`ArmDesignReview.hx` uses no cell, trays, CNC production plan or physical dual
gripper. Each arm receives the same synthetic 40 × 40 × 150 mm tool envelope,
with a TCP 150 mm along the outward flange normal.

Per arm, it observes 141 joint configurations: zero, twelve single-axis ±30 degree
jogs and 128 deterministic Halton samples. Commercial samples use each pinned
source's limits. Original ZeroErr samples use a provisional ±170 degree range.
**Collision-clear fractions across these different domains are not rankings.**

It also evaluates:

- 48 oriented target queries: three radii, four azimuths, two heights and two
  downward-tool yaws. These are finite IK/collision searches, not a complete
  oriented workspace or proof that an unsolved target is impossible.
- Four common motions: a 40 mm line, 40 mm lift, 40 mm square and 20 degree yaw.
  Each has 21 Cartesian knots. Selected joint-space quintics receive continuous
  hull clearance checks at 0.5 mm with a 1e-6 rad lowering reserve. Cartesian
  straightness between knots, timing and dynamics are not certified.
- Jacobians compared with independent central differences. Singularity diagnostics
  use a stated 0.85 m characteristic length to scale linear versus angular rows;
  the 0.01 minimum-singular-value threshold is a diagnostic convention.
- FK equivalence for feasible joint values separated by a full turn, where source
  limits allow it. This does not approve rotating through a full turn, unwind a
  cable or establish a clear connecting path.

Commercial basic motions are positive nominal-model software regression cases;
a future failure must be investigated as a model/planner regression. They are
not manufacturer-executed or physically commissioned golden trajectories.

The collector verifies source/mesh hashes, producer dependencies, native bytecode,
input coverage, independent FK/Jacobians, positive commercial motion cases and
load calculations. It preserves missing external evidence and hardware-release
flags rather than treating all checks as physical validation.

## Mechanical screen and bench work

The standalone load study uses only the original geometry and module catalog
assumptions. It discards the previous synthetic mission configuration and does
not read cell reports. A zero-offset case is an artificial flange-centered
point-load diagnostic, not the literal common tool mass distribution. It screens
the same sampled original-arm poses that clear
the common tool envelope, at each CoG offset and both mass factors.

The zero-speed acceleration screen includes reflected rotor inertia. It checks
mass-matrix symmetry/PSD and gravity against the potential-energy gradient.
Point tool/payload inertia, friction, efficiency, flexibility, velocity coupling
and thermal duty remain unqualified. No commercial effort field is used as a
verified continuous actuator rating.

The next mechanical deliverables are a confirmed module/interface register,
measured or supplier-qualified moving masses/CoGs/inertias, bearing load limits,
and joint/link/connection error and stiffness budgets. Tube, fastener, pilot,
preload, slip, fatigue and harness decisions require those inputs. The earlier
straight-variant stiffness estimates cannot qualify the original supplier links.

[The bench plan](../zeroerr-arm/ARM-BENCH-PLAN.md) defines a focused joint and
connection test. No hardware order, supplier message or physical test is included.

## Current results

All three models pass the four basic motions with continuous joint-path clearance
checks. The suite checks 141 configurations per model. UR5e also passes the
independent manufacturer DH comparison at every configuration; FR5 passes the
drawing dimension checks, but its independent full FK oracle remains missing.
These are software regression results, not physical robot qualification.

For the original ZeroErr arm, the 150 mm combined tool/payload CoG screen finds
119.16 Nm shoulder gravity torque against an assumed 87 Nm rating. Wrist gravity
torques reach 11.99 and 10.89 Nm against assumed 10 Nm ratings. Confirm module
codes, ratios, ratings and mass properties before making a hardware decision;
these sampled estimates do not establish the supplier's payload envelope.

## Run and review

Prepare the two official commercial descriptions and original ZeroErr assembly
as documented in their READMEs. Use a Python environment containing NumPy;
optional plotting also needs Matplotlib. From the repository root:

```sh
python3 machinekit/examples/reference-arms/arm-design-study.py --prepare
./haxeon/scripts/haxeon build --project=machinekit/examples/reference-arms/haxeon.json
./machinekit/examples/reference-arms/run-built.sh --arm-design
python3 machinekit/examples/reference-arms/arm-design-study.py
python3 -m unittest discover -s machinekit/examples/reference-arms -p test_arm_design.py -v
python3 machinekit/examples/reference-arms/render-arm-design.py
```

All generated outputs are isolated under `build/arm-design/`:

- `review.json`: reference evidence levels, numerical checks, motion outcomes,
  source limits, conditioning and original load screens.
- `<model>-native.json`: all observations, collision witnesses, target results,
  paths and continuous proofs.
- `<model>.mtrg`: standalone native arm with common tool, saved in an observed
  clear pose. Open with `./app/run-built.sh --project=PATH`.
- `load-screens.png` / `.svg`: original-arm torque screens versus CoG offset.

Rerun the native suite after rebuilding its bytecode; the collector rejects stale
producer, input, geometry or bytecode hashes. The original import, old comparison
and tending studies remain preserved as historical evidence. No cell study is
needed to run this standalone suite.
