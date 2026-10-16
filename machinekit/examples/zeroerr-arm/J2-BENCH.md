# One-module J2 bench package

Purpose: test the **original 110T J2 load model**, moving-side convention and
holding/motion interpretation before purchasing six modules. No test has been
performed and no article ordered. This prepares the existing
[ARM-BENCH-PLAN.md](ARM-BENCH-PLAN.md), using the reconciled reference model.
The earlier 80T bench proposal is an archived experiment.

## Article and equipment

Request one revision-matched `eRob110H120T-BM-18EN` candidate, confirming ratio,
encoder, brake, EtherCAT ESI/firmware and curves with the supplier first. The
manual 5 kg example suggests nominal 120; the STEP does not identify its code.
Obtain its mating power/EtherCAT cables, vendor multiturn battery, commissioning
tool and approved supply/regeneration unit. No full six-module purchase is needed.

Prepare a rigid support, original 98 mm output-pilot mating fixture, documented
M4 pattern/fastening, a housing-side lever with measured CoG, calibrated weights
and hanger, an independent mechanical load catch, external angle/displacement
metrology, calibrated scale, current/voltage/temperature logging and a protected
single-axis electrical feed. Use the delivered brake/STO procedure.

## Preserve the original mounting

**Fix the output flange upstream; the housing and attached lever move.**
That matches J2 in the supplier arm. A conventional housing-fixed bench is not
equivalent. Mark the original shaft axis, output-face datum and positive motion
sense, and photograph the assembly. Confirm the module's T mounting surfaces and
fasteners against its revision drawing; no new robot geometry is proposed.

Review the fixture for the applied moment, forces, connection slip and catch
loads before test. The coupon/fixture is not released for machining by the
exterior STEP alone. Do not select screw lengths from hole depth alone.

## Measure first

1. Record code, serial, hardware/firmware/ESI revisions and supplier test procedure.
2. Weigh the complete module, lever, hanger and weights separately. Measure each
   signed horizontal CoG distance from the joint axis at every tested angle.
3. Obtain or identify the moving housing/output split without disassembling the
   sealed module. If the split is unavailable, use an external balance/reaction
   measurement to establish the assembled moving-side gravity moment, and retain
   that uncertainty explicitly.
4. Record fixture-alone compliance and metrology resolution/calibration. Measure
   ambient temperature, bus supply and emergency load support.

Use `j2-bench-measurements.template.json` for real measurements. Null means unknown,
not zero. `evaluate-j2-bench.py` computes `sum(m × g × horizontal lever)` and
compares any independent torque measurement without generating hardware commands.

At a 500 mm measured lever, 10 kg alone produces 49.033 Nm. This is an arithmetic
example, not an approved bench load: lever/hanger/moving housing add their own
moments. Reproducing the full-arm horizontal witness means approximately 119.6 Nm
**total joint moment**, not 119.6 Nm plus unaccounted fixture/self-weight.

## Test sequence

- Complete supplier-prescribed unloaded commissioning and command-sign checks
  before adding a gravity load. Keep the catch/support in place.
- With the article, fixture and supplier limits confirmed, define incremental
  loads up to the required moment. Candidate planning fractions are 10/25/50/75/100%
  of the modeled witness, but no step is authorized until the fixture capacity,
  article limits and load-transfer procedure are reviewed.
- At each approved step record brake-held rest (position drift, applied moment,
  temperature and brake/drive status). Transfer support/load only using the
  supplier enable/brake sequence. Do not dynamically stop the lever with a brake.
- Measure static deflection under loading/unloading and reversal, separating
  fixture compliance, connection slip and joint/brake behavior. A composite
  brake-held reversal trace is not pure gearbox backlash.
- Run the supplier-approved low-speed moving duty, logging timestamps, target and
  output position, velocity, motor current, bus voltage, temperature and faults.
  Increase duty only from the delivered torque–speed/temperature/overload limits.
- Repeat the load/model comparison after removal and reassembly to expose
  connection repeatability. Inspect mounting slip and harness strain.

Brake holding uses the delivered brake acceptance and gear load limits; normal
motion uses its speed-dependent continuous/intermittent limits, repeated peaks,
gear average load and stated duty. Do not use momentary shock torque as a normal
bench allowance. The published nominal-120 110-family brake value is 246 Nm and
start/stop allowance 217 Nm, **subject to revision confirmation**; fixture capacity
and test procedure can be lower.

## Evidence and acceptance

Save raw synchronized traces, calibration IDs, actual weights/CoGs, uncertainty,
load sequence, duration, temperature, status/faults, photos and code revisions.
Independent mass/lever torque and external angle measurement are the reference;
virtual torque feedback is not independent metrology. Current-to-torque comparison
needs confirmed Kt, ratio and efficiency, not a universal Nm/A constant.

The worksheet intentionally leaves torque-residual, deflection and temperature
acceptance thresholds unset. Allocate them from measurement uncertainty and
supplier limits before test. Whole-arm 0.03 mm repeatability is not a joint
acceptance threshold. Any disagreement first updates mass/frames/drive behavior,
not the frozen reference geometry.

```sh
python3 machinekit/examples/zeroerr-arm/evaluate-j2-bench.py measured-run.json --output reduced-run.json
python3 -m unittest discover -s machinekit/examples/zeroerr-arm -p test_j2_bench.py -v
```

The untouched template must fail reduction because measurements/fixture capacity
are missing. Synthetic unit fixtures check arithmetic only and are not bench results.
