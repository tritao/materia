# Original-arm module and mass audit

> Historical experiment / superseded load interpretation. See [the frozen reference reconciliation](REFERENCE-RECONCILIATION.md) and [archive policy](EXPERIMENTS-ARCHIVED.md). Do not extend resize or geometry variants.

The original 110T/70T arm remains the geometry baseline. The available documents
confirm the module families and several part materials. They do **not** identify
the six delivered ordering codes, ratios or moving-side mass properties. The
current evidence does not release this assembly for a general 5 kg operating
envelope. CNC tending remains deferred.

## Module register

The pinned assembly drawing labels J1–J3 **eRob110T** and J4–J6 **eRob70T**.
Its module STEP names contain `H××T`: the ratio is unspecified. Nominal 100,
brakes, EtherCAT and HM feedback remain simulation/request assumptions.

| Item | 110T, J1–J3 | 70T, J4–J6 |
| --- | --- | --- |
| Pinned V4 drawing, brake-equipped mass | 4.47 kg | 1.16 kg |
| Pinned manual, V4 brake-equipped mass | 4.49 kg | 1.24 kg |
| Public product-page mass | 4.64 kg | 1.24 kg |
| Assumed nominal-100 rated output torque | 87 Nm | 10 Nm |
| Published nominal-100 peak output speed | 30 rpm | 30 rpm |
| Manual table 7-1, unpowered brake acceptance at nominal 100 | 206 Nm | 50 Nm |
| Manual table 17-6, bearing basic dynamic load C | 21,800 N | 5,800 N |
| Manual table 17-6, bearing basic static load C0 | 35,800 N | 8,600 N |
| Manual table 17-6, allowable static bearing moment Mc | 258 Nm | 74 Nm |

Sources: [110T product page](https://www.zeroerr.cn/eRob/eRob110T.html),
[70T product page](https://www.zeroerr.cn/eRob/eRob70T.html), the versioned drawing
URLs in `original-comparison-config.json`, and the SHA-pinned
[manual V3.44](https://download.zeroerr.cn/static/download/eRob机器人关节模组用户手册_V3.44.pdf?=20260823).
Bearing values are family-level manual data, not a completed combined-load or
life calculation for the delivered T modules. Brake acceptance is separate from
servo torque and gearbox limits. The public emergency-braking wording conflicts
with the manual's section 7 restrictions; delivered procedures must resolve it.

The audited public snapshots retain their original capture date, 2026-10-16.
Fresh direct HTML capture returned HTTP 403 during this audit; the separate web
reader confirmed the product tables. No failed refresh is presented as a new
snapshot. The pinned manual downloaded successfully and matched its recorded SHA.

## What the mass model knows

The audit lists all 127 arm body estimates, their zero-pose CoGs, central inertia
surrogates and active ancestors in `build/module-audit/audit.json`. The STEP
volume, centroid and geometric inertia are geometry evidence. Converting them to
physical mass requires materials or assembly mass data.

Complete module masses replace their exterior CAD volume masses. Within each
module, the existing model distributes mass by exterior volume. Consequently its
housing/output mass split and central inertias are estimates, not measurements.
For example, it assigns 4.255 kg of J2's own module to the moving side but only
0.215 kg of J3's own module to that joint's moving side. This follows J2's
upstream output/moving housing and J3's upstream housing/moving output arrangement.
It is an important uncertainty, rather than proof of those exact masses.

Manual Appendix 3 supplies whole-module CoGs and inertias for the T variants:

| Manual brake-equipped variant | CoG X/Y/Z from output origin, mm |
| --- | --- |
| 110T V4 | 59.08 / 4.09 / −0.02 |
| 70T V4 | 40.06 / 2.62 / −0.06 |

The output-face origin, T-frame axis convention and six inertia components are
recorded in the report. Those inertias are **about the output origin**, not the
CoG; units are g·mm². They cannot be copied into the existing per-body central
inertias or divided into housing/output bodies without additional information.
The audit records them without silently substituting them into dynamics.

## Part drawing material corrections

The source archive contains explicit material callouts:

| Body | Drawing | Callout | Density used for sensitivity, kg/m³ |
| --- | --- | --- | --- |
| Base | 2-091-001 | 7075-T6 | 2810 |
| Base cover | 3-091-001 | AL6061 | 2700 |
| Upper arm | 2-091-002 | 7075-T6 | 2810 |
| Forearm | 2-091-005 | AL6061 | 2700 |
| Elbow output adapter | 2-130-003 | 7075-T6 | 2810 |
| Elbow housing adapter | 2-130-004 | 7075-T6 | 2810 |
| Tool flange | 2-054-006 | 7075-T6 | 2810 |

The previous model used 7075 density for both links and generic aluminium for
these other parts. A separate `drawing_materials_only` scenario corrects those
choices; the baseline and historical reports remain reproducible. Density values
remain engineering approximations. Connectors, fasteners, boots and other
undocumented components still need actual masses/materials.

## Load decision

The audit uses the same 118 configurations passing standalone common-tool hull
clearance, retaining original joint axes. It separates arm self-weight and flange
load, then checks total flange load up to 5 kg at 0/100/150 mm combined CoG.
It also reports horizontal flange-reach bins and zero-speed acceleration bounds
at 30 and 60 degrees/s², with the provisional 1.2 design factor. Reach bins are
sampled pose subsets, not complete oriented workspaces or executable payload maps.

At the nominal-mass 150 mm shoulder gravity witness, the arm contributes 71.75 Nm
and the 5 kg flange load contributes 47.40 Nm, totalling 119.16 Nm versus the
assumed 87 Nm rating. Merely slowing acceleration cannot remove that gravity
load. Increasing the 110T ratio from 100 to 120 or 160 leaves the published rated
torque at 87 Nm while reducing speed. The 70T similarly remains at 10 Nm from
ratio 80 upward. A ratio-only change does not resolve these static flags.

The raw static calculation limits total point load to approximately 1.57 kg over
this entire sampled domain at 150 mm. This is **not** a usable payload rating:
it includes the tool, assumes the current mass split, and omits duty, friction,
flexibility and physical qualification. Applying the provisional 1.2 factor
already flags the unloaded arm in some sampled poses; the report returns an
explicit arm-alone failure rather than inventing a zero payload rating. Restricted
pose subsets can differ and require their own continuous motion/duty checks.

For example, at 150 mm CoG with the 1.2 factor, the baseline horizontal flange
reach bins give these diagnostic **total** load bounds (including the tool):

| Horizontal flange reach | Clear samples | Static screen | 60°/s² zero-speed screen |
| --- | --- | --- | --- |
| Below 350 mm | 58 | 3.41 kg | 2.90 kg |
| 350–600 mm | 35 | 1.43 kg | Arm-alone failure |
| At least 600 mm | 25 | Arm-alone failure | Arm-alone failure |

The last bin is capped at 2 m for bookkeeping, not a reach claim. The samples
have different heights/orientations, and these numbers are not a rated reach
curve or a commissioned operating restriction.

The next decision is to obtain revision-matched drive and moving-mass data, then
choose between a restricted operating envelope and an actuator/architecture
revision. Keep the original source geometry as the reference. Larger shoulder or
wrist candidates require new mass, interface, speed and bearing calculations;
this audit does not select a replacement based on torque alone.

At 150 mm combined CoG, the mass sensitivity results are:

| Scenario | Estimated arm mass | Peak shoulder gravity at 5 kg |
| --- | --- | --- |
| Pinned drawing baseline | 23.512 kg | 119.16 Nm |
| Manual module masses only | 23.812 kg | 121.21 Nm |
| Public-page module masses only | 24.262 kg | 121.83 Nm |
| Baseline arm masses +25% | 29.390 kg | 137.10 Nm |
| Part drawing materials only | 23.526 kg | 119.06 Nm |

Correcting the documented part materials has little net effect here and does
not clear the shoulder flag. These are separate sensitivity cases, not five
competing verified BOMs.

## Reproduce and inspect

Prepare the original model and the standalone arm suite first. With NumPy and
`pdftotext` available, run from the repository root:

```sh
python3 machinekit/examples/zeroerr-arm/module-mass-audit.py --reuse-reviewed-snapshots
python3 -m unittest discover -s machinekit/examples/zeroerr-arm -p test_module_mass_audit.py -v
```

Subsequent runs use cached verified snapshots; `--refresh` explicitly requests
new official HTML captures and fails if capture fails. The script verifies source
archive, assembly, manual, module drawings and standalone dependency hashes. Its
only generated outputs are under `build/module-audit/`; it does not depend on the
old cell registration or write to the supplier request.

The resulting `audit.json` contains the module register, drawing material checks,
per-body mass estimates, five mass scenarios, load limits, reach bins and joint
witness configurations. Missing delivered specifications and measured moving
masses remain explicit. The [bench plan](ARM-BENCH-PLAN.md) describes the physical
measurements needed next. No hardware purchase or physical test has occurred.
