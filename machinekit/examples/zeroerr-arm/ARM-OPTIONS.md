# Retain or resize the original arm

The comparison supports a **separate, bounded-workspace prototype concept** with
an eRob142T nominal-100 shoulder (J2), eRob80T nominal-100 at J4/J5, and the original
110T at J1/J3 and 70T at J6. This is the preferred next CAD investigation if a
120°/s shoulder peak is acceptable. It is not a selected hardware BOM or a general
5 kg / 180°/s arm. No reviewed option passes both the 180°/s peak-speed benchmark
at every joint and the complete sampled 5 kg load screen, including mass sensitivity.

Keeping all original modules remains useful for software/reference work and a
restricted-load prototype. The available estimates do not support declaring the
original arm equivalent to a UR5e/FR5 operating envelope.

## What was compared

[The configuration](arm-options-config.json) defines eight isolated options. The
study retains the original supplier axes and 425/401 mm links, applies the
[drawing material corrections](MODULE-MASS-AUDIT.md), and changes module mass,
rated torque, reflected rotor inertia and a central-inertia size surrogate.
Resized module centres and housing/output fractions retain their original
estimated distribution. This is a sizing model, not new production geometry.

Each changed joint also carries an explicit **0.25 kg connection allowance**,
located 75 mm from the joint along its downstream zero-pose link direction.
This is an assumption, not a designed adapter. The separate +25% arm-mass case
scales these allowances too. The public-module-mass sensitivity uses mixed
public-page revisions and is not a matched BOM.
The 170I retains its pinned V3 mass in that sensitivity case; no refreshed public
170I mass is claimed. The manufacturer 170I page was unavailable during this review.

The load domain contains the same 118 original-arm configurations passing the
common-tool hull clearance check. The four standalone motions contribute 101
joint-path samples each. The comparisons use 5 kg total flange load, combined
CoG offsets 0/100/150 mm, simultaneous zero-speed acceleration bounds of 0/30/60
°/s², and a design factor of 1.2. All combinations are in the report.

The original arm's continuous clearance certificate applies only to its original
geometry. Enlarged modules and adapters need their own CAD, IK and collision
checks. Sampled load checks are separate from continuous load qualification.
Passing sample fractions are diagnostic, not workspace volume estimates or robot
rankings. A failed conservative acceleration bound does not establish that a
properly timed trajectory is impossible.

## Results at 150 mm combined CoG and 60°/s²

Values below 1 pass the sampled demand/rated-torque screen. “Four motions” is the
worst sampled value across line, lift, square and yaw; it is not a duty-cycle test.
All wrist upgrades in this table use nominal 100.

| Option | Estimated arm mass | J2 peak speed | 118 poses, nominal / +25% mass | Four motions, nominal / +25% mass |
| --- | --- | --- | --- | --- |
| Original modules | 23.53 kg | 180°/s | 1.90 / 2.17 | 1.21 / 1.40 |
| 80T at J4/J5 only | 25.21 kg | 180°/s | 2.11 / 2.43 | 1.37 / 1.60 |
| 142T50 shoulder + J4/J5 80T | 29.21 kg | 240°/s | 1.86 / 2.15 | 1.21 / 1.42 |
| 142T80 shoulder + J4/J5 80T | 29.21 kg | 150°/s | 1.25 / 1.43 | 0.83 / 0.96 |
| **142T100 shoulder + J4/J5 80T** | **29.21 kg** | **120°/s** | **1.10 / 1.26** | **0.74 / 0.86** |
| 142T100 + all three wrist modules 80T | 30.05 kg | 120°/s | 1.16 / 1.33 | 0.78 / 0.90 |
| 170I50 shoulder + J4/J5 80T | 30.49 kg | 240°/s | 1.35 / 1.56 | 0.88 / 1.03 |
| 170I80 shoulder + J4/J5 80T | 30.49 kg | 150°/s | 0.94 / 1.07 | 0.62 / 0.72 |

![Standalone actuator sizing comparison](build/arm-options/options-comparison.png)

The heavier wrists alone improve wrist torque margins but worsen the shoulder.
Upgrading J6 too adds another 0.84 kg, including its assumed adapter; it also pushes
J3 above its assumed rating in the heavier-mass case. The current axial point-load
screen does not justify that upgrade. Actual tool inertia, off-axis loads and
rotation duty may change the J6 decision.

The 142T80 is the faster T-module compromise and passes the four motion samples
at +25% mass, but its worst value is 0.96. The 142T100 has more reserve there,
though its full sampled-domain screen still fails. The 170I80 clears the nominal
118-pose screen but fails the +25% case, is heavier, and requires a new cylindrical-
module angle bracket. Its additional integration work does not presently justify
selecting it over the bounded 142T concept.

## Option A: retain the original modules

At 150 mm CoG, 60°/s² and factor 1.2, the **nominal-mass sampled total-load bounds**
on the existing motions are:

| Motion | Total flange load bound, including tool | With a 1 kg tool, remaining workpiece mass |
| --- | --- | --- |
| 40 mm line | 3.06 kg | 2.06 kg |
| 40 mm lift | 3.93 kg | 2.93 kg |
| 40 mm square | 2.35 kg | 1.35 kg |
| 20° yaw | 4.70 kg | 3.70 kg |

Thus 2.35 kg total is a nominal load-screen candidate common to these four motions,
not a commissioned limit. With +25% arm mass, even the unloaded square has a
sampled failure; there is no common robust load interval to release from these
assumptions. The 1 kg tool split assumes the specified combined CoG remains the
same as workpiece mass changes; actual tool/workpiece CoGs must be recomputed.

At 5 kg the nominal line/lift/yaw have zero-speed acceleration bounds of about
8/33/52°/s². The square fails even the static screen with factor 1.2. Simply slowing
the whole original arm does not recover all four 5 kg motions under this criterion.
Published joint speed remains a separate upper bound: no usable speed limit is
inferred from a zero-speed calculation. Velocity coupling, friction and duty need
a timed-path model and delivered-drive curves.

The machine report lists individually passing pose indices and load bounds per
reach bin. Those discrete poses must not be expanded into a rectangular joint
window or a continuous safe workspace without further checks.

## Option B: resize selected joints

The preferred concept adds approximately **5.68 kg** to the corrected baseline:
3.75 kg at J2, 0.59 kg each at J4/J5, plus 0.75 kg of assumed connections. It
preserves the original model separately. No source layout or existing assembly
was modified.

The 142T100 concept passes the four sampled motion load checks at 5 kg and all
three examined CoG offsets, including public-mass and +25% sensitivities. Its
wider-domain 150 mm / 60°/s² load bound is only 3.62 kg nominal and 1.46 kg with
+25% mass. These are point-load screening bounds, not hardware payload ratings.
A restricted-motion prototype is supported more strongly than a general-purpose
5 kg claim.

Mounting changes are substantial:

| Interface | Original | Candidate | Required design work |
| --- | --- | --- | --- |
| Shoulder outer diameter / length | 110T: 116 / 147.5 mm | 142T: 148 / 163 mm | Rebuild shoulder connection and check swept envelope; resolve resulting axis offsets |
| Shoulder output pilot | 98 mm | 127 mm | New upstream pilot seat and fastener pattern; J2 output remains upstream |
| Shoulder output fasteners | 15 × M4, 10 mm drawing depth | 15 × M5, 15 mm drawing depth | Revision-matched engagement, preload and access design |
| Wrist outer diameter / length | 70T: 73 / 99 mm | 80T: 85 / 111.8 mm | New forearm/wrist connections and harness routing |
| Wrist output pilot | 58 mm | 71 mm | New mating pilots and transition to retained J6 |
| Wrist output fasteners | 14 × M3, 6 mm drawing depth | 12 × M3, 6 mm drawing depth | New bolt circles/patterns; check exact drawings |

A drawing's thread depth is not a machining-ready screw engagement instruction.
The pilot, mating tolerances, actual engagement, shoulder support, fastener load
path and bearing life must be reviewed together. Straight links can remain a design
aim, but new end adapters and larger envelopes may change link-axis offsets and
invalidate this original-axis sizing screen.

## Evidence and next gate

The candidate torque, speed, mass and rotor inertia data are pinned to the
142T V4, 80T V5 and 170I V3 drawings listed in the report. The manufacturer pages
confirm the 142T/80T torque and peak-speed tables:
[142T](https://www.zeroerr.cn/eRob/eRob142T.html),
[80T](https://www.zeroerr.cn/eRob/eRob80T.html).
The 142T100 is 178 Nm rated with 20 rpm published peak output speed; the 80T100
is 31 Nm with 30 rpm. These separate numbers do not qualify their simultaneous
operation, static holding, thermal duty or a delivered controller configuration.

Before integrating the concept, obtain the exact module revisions/codes,
continuous holding and torque–speed–temperature curves, moving-side mass/CoG,
and confirmation of the inverted shoulder mount. Then build a separate shoulder
and J4/J5 CAD assembly with dimensioned adapters, recompute mass/axes, and repeat
clearance, timed-load and bearing/connection checks. Keep J6 unchanged until its
tool-inertia/duty requirement provides a reason to resize it.

If 180°/s at every joint remains mandatory, no reviewed shoulder here closes the
complete sampled load requirement. Investigate a different actuator family,
counterbalance or lower distal mass before committing to larger housings.
No supplier message, purchase or physical test was performed. CNC tending remains
deferred.

## Reproduce

Prepare the [standalone suite](../reference-arms/ARM-DESIGN.md) and
[module audit](MODULE-MASS-AUDIT.md), then run from the repository root:

```sh
python3 machinekit/examples/zeroerr-arm/arm-options-study.py
python3 -m unittest discover -s machinekit/examples/zeroerr-arm -p test_arm_options.py -v
python3 machinekit/examples/zeroerr-arm/render-arm-options.py
```

NumPy is required; plotting additionally needs Matplotlib. The pinned candidate
PDFs under `build/revision-review/` were captured during the earlier engineering
revision, whose source URLs/hashes remain in `revised-engineering-config.json`.
Generate a fresh module audit after changing any pinned dependency.

Outputs are `build/arm-options/comparison.json`, `options-comparison.png` and
`options-comparison.svg`. The report checks producer/input hashes and records
all choices, assumptions, per-joint demand ratios, original pose indices, path
load bounds and mass variants. The vectorized serial-chain mechanics solver is
checked against the original solver, an analytical pendulum and energy gradients.
