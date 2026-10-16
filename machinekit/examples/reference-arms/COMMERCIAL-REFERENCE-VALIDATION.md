# Independent commercial-arm kinematic regressions

**UR5e and FR5 now both have independent manufacturer nominal-DH checks** against
fresh compiled native FK and Jacobians. This closes the previous FR5 full nominal
FK evidence gap. It does not claim serial-specific controller calibration,
physical repeatability or dynamic qualification.

## Independent sources and frames

- UR5e: Universal Robots' [official DH table](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/).
  The ROS base convention requires the documented Rz(π) mapping; terminal DH frame
  is the physical flange. No fitted transform is used.
- FR5: FAIRINO Collaborative Robot User Manual 4.0.0,
  [section 2.4, table 2.4-5](https://fairino-doc-en.readthedocs.io/latest/CobotsManual/robot_brief_introduction.html).
  It is the **FR5** table, not FR5-C or FR5-WML. Standard DH arrays in metres are
  `a=[0,-.425,-.395,0,0,0]`, `d=[.152,0,0,.102,.102,.100]`,
  `alpha=[π/2,0,0,π/2,-π/2,0]`, with zero joint offsets.
  ROS base_link matches this nominal base, and d6 includes the 100 mm physical
  flange offset. No optimized alignment or fitted joint offsets are used.

Both checks add the same explicit 150 mm TCP offset along flange Z.
FR5 source facts are preserved in `fr5-manufacturer-dh.json` with source/version,
section and capture method. Direct HTTP returned 403; the official web reader
provided the numeric table. The file is a normalized table extraction, not raw
HTML or measured controller output. Prepared inputs pin its hash.

## Results

Each model has **141 configurations**: zero, ±30° single-joint cases and 128 Halton
samples across its pinned source-description limits. These are mathematical FK
regressions; self-colliding samples are not physically commanded poses.

| Check | UR5e | FR5 |
| --- | --- | --- |
| Manufacturer DH configurations passed | 141/141 | 141/141 within explicit source-precision bound |
| Largest TCP position difference | 2.34 × 10⁻¹⁰ m | 0.103403 mm |
| Allowed position difference | 0.00001 mm | 0.116893 mm, derived analytically |
| Largest rotation operator difference | 6.11 × 10⁻¹⁰ | 1.08314 × 10⁻⁵ |
| Native/source matrix difference | <1.4 × 10⁻¹⁵ | <1.3 × 10⁻¹⁵ |
| Native Jacobian vs independent finite difference | <3.3 × 10⁻¹⁰ | <2.8 × 10⁻¹⁰ |

### Why FR5 has a larger difference

The pinned ROS V6 description contains forearm length **395.01 mm**, wrist offset
**102.1 mm** and rotations **1.5708 rad**, whereas the manufacturer nominal table
uses 395 mm, 102 mm and π/2. They are preserved and reported, not silently edited.

The tolerance is computed before checking samples:

`position bound = sum(origin translation differences) + sum(rotation operator difference × remaining chain-length bound)`

Rotation error is bounded by the sum of fixed-origin rotation operator differences.
A tiny numerical allowance is added. This follows the triangle inequality for
rigid transforms and is independent of the tested configurations. It explains
source precision/configuration differences; it is not a calibration or
repeatability tolerance. A deliberate 10 mm wrong-link test exceeds this bound
by more than 80 times and fails.

## Golden tests and evidence levels

`manufacturer-fk-goldens.json` preserves eight manufacturer-DH-derived TCP matrices
for native regression. They are not generated from ROS/native FK and are not
controller-recorded measurements. Analytical FR5 zero/base-rotation fixtures check
signs, base frame, units and tool offset. Additional tests verify that the bounded
comparison detects a wrong link.

The two trusted regression levels are:

1. **Implementation integrity:** native model versus independently evaluated source
   transforms/Jacobians at numerical tolerance.
2. **Manufacturer nominal geometry:** source/native geometry versus independently
   published manufacturer DH, with explicit source differences bounded for FR5.

Physical/serial calibration and vendor-controller recorded samples remain separate
future evidence. Neither robot's source effort/inertial fields are used to
establish a drive rating. This work does not rank robots by the old 23 transfers.

## Reproduce

Prepare the pinned commercial descriptions as in README, then:

```sh
python3 machinekit/examples/reference-arms/commercial-reference-study.py --prepare
./haxeon/scripts/haxeon build --project=machinekit/examples/reference-arms/haxeon.json
bash machinekit/examples/reference-arms/run-built.sh --manufacturer-checks
python3 machinekit/examples/reference-arms/commercial-reference-study.py
python3 -m unittest discover -s machinekit/examples/reference-arms -p test_commercial_manufacturers.py -v
```

Outputs: `build/manufacturer-validation/inputs.json`, `native.json`, `review.json`.
Checks pin source-lock, manifests, URDFs, meshes, scenes, normalized manufacturer
facts, helper/producer code and actual native bytecode. Re-run native observations
after a bytecode rebuild. The older `arm-design-study.py` report's FR5-pending
status and shell-distributed load conclusions are historical; use this report
for commercial FK and the [frozen reference reconciliation](../zeroerr-arm/REFERENCE-RECONCILIATION.md)
for ZeroErr loads.
