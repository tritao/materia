# Frozen reference work: delivered evidence and remaining inputs

The original supplier ZeroErr 5 kg assembly is the trusted baseline: **110T
J1–J3, 70T J4–J6, unchanged STEP geometry and placements**. All modification
experiments are archived; no resize/straight/cover experiment was extended.
CNC tending and the inherited 23 transfers remain deferred.

For inspection in the app, open
`../reference-arms/build/arm-design/zeroerr-braked.mtrg`. Play holds the saved
pose with brake-capable drives and reconciled module inertials; no motion program
runs. The runtime enables servo holding and releases brakes on Play. For a
single slow J2 excursion and return, use `zeroerr-slow.mtrg` in the same folder. See [preview physics and validation](../reference-arms/PASSIVE-DYNAMICS.md#stationary-inspection-preview-with-brake-capable-drives).

## Six separately committed steps

| Step | Delivered package | Commit |
| --- | --- | --- |
| 1. Load reconciliation | [Hand calculation, whole-module dynamics, braked static and stated duty envelope](REFERENCE-RECONCILIATION.md) | `a36e119e2` |
| 2. Supplier confirmation | [Final text to copy and send; not sent](SUPPLIER-CONFIRMATION.txt) | `2bd118c33` |
| 3. Build package | [Original STEP BOM, likely codes, assembly/wiring and cost allowance](REFERENCE-BUILD.md) | `e186f7916` |
| 4. Controls | [Native timing, sampled torque/speed/duty validation and eRob interface stub](REFERENCE-CONTROLS.md) | `c5cc93174` |
| 5. Single-joint bench | [One original 110T J2, moving-housing fixture, measurements and reducer](J2-BENCH.md) | `efd02a88f` |
| 6. Commercial references | [Independent manufacturer nominal-DH checks for UR5e and FR5](../reference-arms/COMMERCIAL-REFERENCE-VALIDATION.md) | `a81a095f5` |

The supplier text was refined in step 3 after identifying nominal-120 proximal
modules in the manual's 5 kg I-module wiring example. Corresponding T-module
ordering codes remain hypotheses for confirmation, not purchase selections.

## Results

- Hand horizontal-pose shoulder gravity: **119.591 Nm**, matching the solver.
  The old 119.157 Nm was a different sampled pose. It was incorrectly compared
  with a universal 87 Nm holding limit; the manual has separate rated, repeated,
  average-load and brake limits. Braked rest is not continuous servo holding.
- Whole-module model: six manual mass/CoG/inertia lumps, about **23.826 kg**
  estimated arm mass, with no exterior-shell mass allocation to module internals.
  The housing/output movement approximation remains explicit and testable.
- Conditional braked-static 5 kg screen: **118/118** sampled clear poses at each
  0/100/150 mm CoG. CSV provides payload, actual reach/XYZ, q and torque for 2,124
  combinations. No complete vendor operating curve is inferred from samples.
- Four original-arm motions at **5 kg / 150 mm CoG** pass actual timed-sample
  inverse-dynamics checks, published-curve inner approximations, repeated peaks,
  rated-torque RMS proxy and gearbox average-load screen. Motion-only shoulder
  RMS is **63.22–69.97 Nm**. **5,949 commands** replayed in four independent
  simulation-stub runs; no unchecked inter-run transfer or hardware connection.
- Independent native FK: **141 configurations per commercial arm**. UR5e maximum
  manufacturer position difference is about 2.34×10⁻¹⁰ m. FR5 is 0.103403 mm,
  below its analytically derived 0.116893 mm source-precision bound. Eight frozen
  manufacturer-DH goldens and analytical fixtures are included. Neither is a
  serial-specific physical calibration claim.
- Ten targeted Python tests pass (four dynamics, three bench reducer, three
  commercial oracle tests); native review additionally checks hardware denial
  and stale-command rejection.

## What is still genuinely missing

The supplied STEP/manual cannot establish all delivered hardware details. These
remain explicit in the ready-to-send supplier request:

- Exact six module codes/ratios/revisions/options and moving-side mass splits.
- Assembled/harness joint limits, physical zero/signs and actual encoder scaling.
- Revision-matched digital continuous/intermittent torque–speed/temperature curves,
  overload duration and brake/load-transfer timings.
- Full vendor payload/CoG/inertia/reach/duty envelope and example commissioned cycle.
- Installed external fastener quantities/lengths/grades/preload, aviation connector
  identity, final harness lengths and approved power topology.
- ESI/PDO mapping, real EtherCAT master/backend and hardware safety commissioning.
- Supplier quotations: **$9,400–19,300** is an engineering allowance, not a quote;
  tax/shipping/labour excluded, with 25% contingency shown separately.

Accordingly, physical arming is unavailable, manufacturing/purchase release is
not claimed, and the J2 bench is prepared rather than performed. The next real
step is sending the prepared supplier text and obtaining one confirmed J2 article
for measured validation before purchasing the complete arm. No message was sent,
order placed or geometry changed. Future CNC layout work should use the shared
workspace of UR5e, FR5 and this frozen original arm.

## Main artifacts

All paths below are relative to the original-arm example directory:

- `build/reference-reconciliation/reconciliation.json`
- `build/reference-reconciliation/static-operating-envelope.csv`
- `build/reference-reconciliation/duty.json`
- `build/reference-build/bom.csv` and `bom.json`
- `build/reference-control/validation.json` and `replay.json`
- `../reference-arms/build/manufacturer-validation/review.json`

Generated vendor assets/reports remain under ignored build directories; source
code, normalized oracle facts, templates and procedures are committed. Shared
CobotArm/TendingReachCertificate edits were not staged or modified.
