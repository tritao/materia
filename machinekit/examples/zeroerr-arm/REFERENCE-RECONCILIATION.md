# Frozen ZeroErr 5 kg reference: load reconciliation

The supplier STEP assembly, 110T J1–J3 and 70T J4–J6, is the trusted 5 kg arm.
Geometry is frozen by `reference-baseline.json`. Modified wrists, straight tubes,
covers and larger-module concepts are [archived experiments](EXPERIMENTS-ARCHIVED.md).
Negative checks identify model/operating-assumption errors; they do not reject
this baseline or authorize a redesign.

## The 119 Nm hand calculation

Rotate J2 by −90°, with all other supplier zero coordinates unchanged: the two
long links are stretched horizontally. The load is 5 kg total, CoG 150 mm along
the tool normal. In this particular pose that normal is parallel to J2's axis,
so its 150 mm offset does not add a shoulder lever. The payload's perpendicular
lever is 0.912 m. For each body, shoulder torque is `mass × 9.80665 × signed lever`.

| Body | Mass, kg | Shoulder lever, m | Torque, Nm |
| --- | --- | --- | --- |
| Upper arm | 1.5350 | 0.21251 | 3.199 |
| Forearm | 1.2014 | 0.62095 | 7.316 |
| J2 whole module | 4.49 | 0.00409 | 0.180 |
| J3 whole module | 4.49 | 0.42091 | 18.533 |
| J4 whole module | 1.24 | 0.82338 | 10.013 |
| J5 whole module | 1.24 | 0.83294 | 10.129 |
| J6 whole module | 1.24 | 0.90938 | 11.058 |
| Other supplier fittings/adapters | Individual rows in report | Individual levers | 14.445 |
| Tool + workpiece | 5 | 0.912 | 44.718 |
| **Total** | | | **119.591** |

The hand sum agrees with the rigid-body solver. The earlier **119.157 Nm** was
the maximum over a different set of sampled poses, not the hand pose above.
At that exact old witness the new whole-module model gives **−121.156 Nm**
(the sign records load direction). Whole-module lumps correct the mass model;
they do not make the shoulder gravity moment disappear.

## The assumption that failed: rating interpretation

The manual section 2 defines rated torque at a specified gearbox input speed
(2000 rpm). It separately defines repeated start/stop peaks, permissible average
load, momentary shock torque and section 7 brake acceptance. Treating 87 Nm as a
universal static-holding ceiling was wrong.

Conditional on nominal-100 modules:

| Check | 110T limit | 70T limit | Use |
| --- | --- | --- | --- |
| Repeated start/stop gearbox loading | 204 Nm | 36 Nm | Normal transient/repeated-load ceiling; not unlimited servo holding |
| Unpowered static brake acceptance | 206 Nm | 50 Nm | Rest with brake actually applied; not dynamic braking |
| Rated torque | 87 Nm | 10 Nm | Continuous operation / stated duty proxy, with speed curve |
| Permissible average gearbox load | 140 Nm | 14 Nm | Speed-weighted cubic load over moving duty |
| Momentary allowance | 369 Nm | 70 Nm | Unexpected shock; never used to pass normal operation |

The 119.591 Nm shoulder load passes both the **204 Nm repeated-load** and
**206 Nm static brake** checks. It is not claimed as continuous servo holding
with the brake released. Brake sequencing and delivered hardware ratings still
need confirmation. Sources are the SHA-pinned manual V3.44 sections 2, 7, 12,
23 and 25 and the pinned 110T/70T drawings in `original-comparison-config.json`.

## Physical mass model

Each complete module is one lump using manual Appendix 3 mass, CoG and inertia.
No module mass is distributed over exterior CAD shells. Output-origin inertia is
converted from g·mm² to kg·m², translated to the CoG using the parallel-axis
theorem, then rotated into the supplier assembly. T-frame axes follow manual
X toward shaft, Y toward mounting base, Z by right-hand rule. The source STEP
output-face datums are z=−16 mm (110T) and z=47 mm (70T); their shaft/base directions
differ. Those coordinate choices are recorded explicitly in the baseline JSON.

The whole lump follows its housing: J1/J2 downstream, J3–J6 upstream. Relative
internal/output movement remains a stated approximation, with diagonal reflected
motor inertia retained for motion. Moving-side measurements are still requested;
whole-module data do not magically provide that split. Link and adapter materials
follow their supplier drawings with approximate alloy densities. No geometry,
source placement or module envelope was changed.

## Operating envelope established so far

**Braked static, conditional model:** all 118 original-arm hull-clear diagnostic
poses pass at 5 kg total for combined CoG offsets 0, 100 and 150 mm, checking both
brake acceptance and repeated gearbox loading. The report also records loads 0–5
kg, each pose index and joint torque. This is the sampled static envelope. It is
not an exhaustive vendor payload/reach curve, physical joint travel or a
continuous collision certificate.

At 5 kg, each tested CoG offset passes in the following horizontal flange-reach
groups. These are sampled coordinates, not an interpolated vendor reach curve:

| Observed reach group | Actual sampled reach | Passing / sampled |
| --- | --- | --- |
| Below 350 mm | 54.9–342.4 mm | 58/58 |
| 350–600 mm | 353.9–596.4 mm | 35/35 |
| At least 600 mm | 602.2–914.6 mm | 25/25 |

`build/reference-reconciliation/static-operating-envelope.csv` records all 2,124
load/CoG/pose combinations, actual flange XYZ/reach, joint coordinates, gravity
torques and the braked-static/repeated-gear pass flag. It must not be read as a
motion rating or a continuous collision-free workspace.

**A stated moving duty at 5 kg:** upright arm, J1 0→20°→0, four seconds each leg,
then 12 seconds braked dwell, repeated as a 20-second cycle. Peak joint speed is
9.375°/s and peak acceleration 7.217°/s². The model passes the conservative
published-curve approximations, repeated peaks and rated-torque RMS proxy. The
maximum servo RMS torque is 0.206 Nm and peak torque is 0.454 Nm. This deliberately
simple commissioning duty demonstrates a passing 5 kg motion; it does not stand
in for a full commercial-arm duty envelope or long horizontal servo holds.

The curve JSON uses conservative inner points read from manual section 25.3,
whose plots are H100I; the manual describes I/T as sharing parameters except
mounting. Exact delivered T revision, overload duration, cooling and digital
curves remain supplier questions. These curves are usable in simulation and
explicitly not marked hardware validated. Dwell servo torque is modeled as zero
only while the brake holds. Measured current/temperature qualification remains
separate from output-torque RMS.

**Conclusion:** the trusted vendor arm now passes the appropriate 5 kg static
checks and a stated motion duty. The earlier contradiction came from a specific
rating interpretation and from using an untimed, conservative screen as if it
were an operating rating. No geometry modification is required by that result.

## Reproduce

```sh
python3 machinekit/examples/zeroerr-arm/reconcile-reference.py
python3 machinekit/examples/zeroerr-arm/reference-duty.py
python3 -m unittest discover -s machinekit/examples/zeroerr-arm -p test_reference_dynamics.py -v
```

Generated reports: `build/reference-reconciliation/reconciliation.json` and
`duty.json`. Commissioned position limits, complete payload/reach/duty curves and
actual module ordering codes remain pending supplier confirmation.
