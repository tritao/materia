# Executed J6 frame/drive study — 2026-10-26

The frozen reference passes static stiffness but misses the requested 0.05 mm
braking-following target. None of the tested mechanical bounds closes that gap.
An offline reduced-timing bound is below the 0.04 mm reserve at the studied
joint-model poses, including the gross mount-mass sensitivity, but does not
establish a controller policy at nominal feeds. Frozen CAD, SR-D3 and mount/API
adoption remain unchanged. Cycle times are excluded throughout.

## Coverage and frozen reference

All 186 executed histories pass manifest, output, provider and clock-certificate
hash/limit checks: 90 pose-grid, 64 phase-grid and 32 full-feed-resume histories.
Coverage is nominal motion, executed hold/resume and ≤100% overrides; raised-feed
replans above 100% remain outside this cohort. No offline stop suffix or producer
structural-response number is substituted for a new result.

The new structural baseline covers both beds, 100/50 mm meshes, all nine full-Z
poses and 40 modes per pose (36 cases). Modal history starts at the rapid lead-in.
The anchored-floor reference retains assumed 50,000 N/mm horizontal and vertical
foot-support springs.

| Frozen reference | 2600 bed | 3150 bed |
|---|---:|---:|
| Worst fine-mesh static X, N/µm | 2.165 | 2.109 |
| Worst fine-mesh static Y, N/µm | 2.051 | 2.044 |
| Nominal braking-following, mm | 0.05024 | 0.06866 |
| Observed uncertainty braking-following, mm | **0.07870** | **0.09967** |
| Nominal geometric contour deviation, mm | 0.01513 | 0.02200 |
| Nominal physical-rest residual, mm | 0.00466 | 0.00532 |

Braking-following is distinct from geometric contour error and rest residual;
the SR-D3 criteria are not rewritten. Nominal values use computed frequencies
and assumed 1% damping. They are not uncertainty bounds or a cutting release.

The braking search evaluates every admitted history/mesh at 17 **uniform** mode-
frequency scales from 0.85–1.15 and damping ratios 0.5%, 1%, 3%. Two screened
governing histories per bed are refined at 129 scales and seven damping ratios
from 0.5–3%. This is a sampled search, not continuous uncertainty qualification
or a 129×7 sweep on every history. Peaks found with the 1 ms response are checked
again at the export timestep; the largest difference is 0.000064 mm.

Governing baseline braking poses are X800/Y0/Z200 and X800/Y3150/Z200 on the
short/long beds, on the 50 mm mesh. Histories are phase250/p3 hold/resume and
phase500/p5 hold/resume respectively. Coupled frame-X/gantry-Y modes lie near
27 Hz. Frame-X sway loads frame/feet/braces; gantry-Y roll involves the beam,
Y guides and drive. At short-bed centre the latter stores about 34% of strain
energy in the beam and 32% in the Y drive. Frequency remains a reported quantity.

## Drive and seat screens

Each mechanical variant has both meshes at the two governing braking poses
(28 additional structural cases). Its 58 matching executed histories are replayed;
these variants are not all-pose acceptance runs. No physical mount is adopted.
Except for the frozen reference and three-anchor row, every row below includes
the proposed end-anchored seat compliance. Stiffness-only rows hold mass fixed.

| Mechanical model | Dual-Y series stiffness, MN/m | Short braking, mm | Long braking, mm |
|---|---:|---:|---:|
| Frozen reference: no seat spring | 17.065 | 0.07870 | 0.09967 |
| Three-anchor seat, original drive | 8.546 | 0.10163 | 0.13602 |
| End-anchored seat, original drive | 12.047 | 0.08573 | 0.11919 |
| End anchors + reducer stiffness ×2 | 12.865 | 0.08308 | 0.11866 |
| End anchors + preload stiffness ×2 + reducer ×2 | 17.034 | 0.07877 | 0.09976 |
| End anchors + preload stiffness ×4 | 18.358 | 0.07540 | 0.09584 |
| End anchors + preload stiffness ×4 + reducer ×2 | 20.327 | 0.06947 | 0.09131 |
| Previous row + gross mount-mass sensitivity | 20.327 | 0.08098 | 0.11696 |

End anchoring improves seat stiffness from 8.56 to 20.49 MN/m per pinion and
reduces error relative to the three-anchor seat. Added local hardware is 0.0243 kg
per studied 500 mm section; whole-axis hardware/joint details remain pending.
The three-anchor case also fails static Y stiffness at the tested poses
(approximately 1.95 N/µm). End anchoring is valuable, but insufficient by itself.

The ×2/×4 preload cases change the **radial spring/pressure-angle compliance**
in `RackDriveSizing`, not merely steel motor-plate bending. A stiffer plate alone
cannot remove that term. Any real change must reconcile preload force, rack-
height variation and travel. The reducer and mesh properties remain assumed;
no replacement gearbox or mass-neutral hardware upgrade has been selected.
Module 2 is not credited: substituting its pitch height leaves a 5.08 mm foot
under a 6 mm counterbore, and permissible feed force is not a mesh-stiffness rating.

There is also a geometry/model mismatch to resolve before the mount proposal:
the nominal Y preload connectors imply pivot motion along Y (tangential), while
the sizing formula assumes radial motion. The audit records the hinge-axis cross
hinge-to-pinion-arm vector as (0, −100, 0) mm/rad on both sides/beds, with zero
radial alignment. This static nominal plate has no simulated revolute joint;
the audit describes its drawn connector layout, not a measured mechanism.
Correct the mechanism in the future proposal without silently substituting a
new stiffness into these reference results.

## Mass and joint-model policy bounds

The mass sensitivity extends gross local section volumes, not fixture totals:
a 22×20 mm receiver with its 44.9×9.5845 mm foot, a 140×8 mm pad and 140×4 mm
assumed replication layer. Existing Y pads are replaced in the mass accounting.
This adds **34.76 kg moving Y**, plus **49.77/57.70 kg fixed** on the short/long
beds. It is an illustrative lumped stock-mass penalty: X host, drilling, hardware,
joints, contact and composite stiffening remain unqualified; frozen rack masses
are retained. It is not a completed catalogue BOM or a qualified mount layout.
No composite host-stiffness credit is taken for the added stock.

The strongest zero-added-mass candidate has lowest modes 27.28/26.60 Hz and
first ≥50%-gantry-kinetic modes 28.29/27.62 Hz. With the mass penalty, lowest modes
fall to 25.55/25.02 Hz and braking error returns above the frozen reference.
Native checks confirm mass-only changes leave static cutting stiffness unchanged.
The real mount must earn its mass; these screens do not justify adding this stock.

The following offline policies are replayed **on that drive/seat/mass sensitivity**,
using its 58 matching histories and both meshes, with the same uncertainty search:

| Counterfactual load policy on mass model | Short braking, mm | Long braking, mm |
|---|---:|---:|
| Extra whole-command ZV | 0.07500 | 0.10296 |
| Extra whole-command ZVD | 0.07103 | 0.09717 |
| Uniform timing k=0.8, including stop | 0.05005 | 0.06819 |
| Uniform timing k=0.6, including stop | **0.02717** | **0.03811** |

These are transformed loads, not new executed J6 exports. Whole-command Cartesian
shaping does not establish contour-safe path-distance shaping. The k=0.6 bound
scales velocity/acceleration/jerk by 0.6/0.36/0.216, including the ready stop; it is
not an existing 60% override command, not a demonstrated pose-scheduled replan,
and not performance at the original feeds. Frozen-only policy endpoints, screened
across all 186 histories, remain in the [earlier policy record](rack-results/j6-policy-bounds-20261026.json).

Recommendation: retain the frozen frame/braces. Keep real mount/API adoption
held. Develop a lightweight mount with correct radial preload motion and end
anchoring, clear the preload/switch conflicts, and obtain new executed J6
histories for reduced ready-stop A/J at nominal contour feeds (and proper
path-distance ZVD if selected). Recheck with the real mount mass/compliance,
new publication-reserve histories, higher-mode convergence and measured modes/
damping from SR6-C. New mount weld reactions/fatigue remain unqualified here.

## Validation and retained evidence

All 64 structural cases ran serially through `heavy-run --slots 1`, with a
3,000,000,000-byte scope cap, an 8 GiB available-RAM gate and immediate pre-CCX RAM
check. Root stayed above 8 GB. Motion landing work kept priority. Maximum measured
case peak was **919,220,224 bytes**; modal replay peaked below 0.14 GB.

Native force-balance, energy and reciprocity checks pass. Maximum relative errors
are 0.0207%, 0.0061% and 0.000011% respectively. Independent equal-share series
estimates agree with native static compliance within 0.0006% of total compliance
across 24 cases. Four mass-only invariance checks pass. The 15 preparation tests
also pass. No completed case retains bulk solver output; no DATA1 writes occurred.

[Joint results and verified archive inventory](rack-results/j6-joint-options-20261026.json)
include histories, poses, meshes, uncertainty points, mode/energy summaries,
input/deck/response/source hashes and resource evidence. A hash-verified compact
archive on root retains generator/reader snapshots, packets and parsed modal
summaries; reproducible working outputs are removed. The [frozen baseline](rack-results/j6-frozen-20261026.json)
and its archive remain separately identifiable. No cycle-time acceptance or
comparison is made.


## Motion-session handoff: executed reduced ready-stop timing

Requested 2026-10-26. Export two **new executed** J7/J4/J5/J6 cohorts, retaining
nominal contour feeds and axis limits, changing the ready-stop policy only:

| Cohort | Stop acceleration cap / reference | Stop jerk cap / reference |
|---|---:|---:|
| S08 | 0.64 | 0.512 |
| S06 | 0.36 | 0.216 |

These are k²/k³ fractions motivated by the offline k=0.8/0.6 screens, not a
request to time-scale the entire move or apply an override. Apply them to each
axis's documented reference ready-stop acceleration/jerk caps; publish both
absolute caps and fractions. Where 3 m/s² is the applicable reference acceleration,
the caps are 1.92 and 1.08 m/s². Do not infer an absolute jerk from contour jerk:
record the actual ready-stop reference J and the resulting 0.512J/0.216J.
The current brake/drive limits remain ceilings. Preserve continuous position,
velocity and acceleration at hold and resume; execute the real reduced-limit
stop and full-feed resume. If the inherited acceleration cannot enter the new
cap instantaneously, export the actual jerk-limited transition and its limits;
do not clip samples or relabel a reference history.

Use **both 2600 and 3150 beds**, the same pose/phase grid and binding identities
as the existing 186-history handoff (90 pose-grid, 64 phase-grid, 32 full-feed
resume), for each cohort. Preserve the existing nominal feed settings, including
20 m/min contour and 40 m/min rapid where the original case requests them;
publish any axis-limited reductions. In particular include X800/Y0/Z200 on 2600
and X800/Y3150/Z200 on 3150, with phase250/p3 and phase500/p5 holds and the
remaining phase-grid hold locations. Retain nominal, hold/resume and ≤100%
override cases; no above-100% executor time scaling. Preserve the whole rapid
lead-in through braking, resume, cutting and final physical-rest tail so modal
history is continuous. Do not export an offline suffix replacement.

Deliver J4-compatible manifests, execution/provider identities, content hashes,
clock certificates, mapped V/A/J limits, absolute feed/stop policy metadata,
pose bindings and a handoff identifying publication-reserve revision. Export
actual position/velocity/acceleration/jerk and stop/resume phase boundaries at
the original resolution. Include an unmodified reference cohort from the same
landing/revision if execution or reserve behavior changed. If any binding cannot
be reproduced, list it explicitly rather than silently shrinking the cohort.
We will compare braking-following against 0.05 mm and report 0.04 mm reserve,
geometric contour and physical-rest residual separately. **No cycle-time frame
acceptance.** These stop-only exports may differ materially from uniform timing;
the offline results are not a promised pass.

Future-run resource decision (2026-10-26): a second shared heavy-run slot is
allowed for **one sequential router case**. Keep the 3 GB case cap, ≥8 GiB
available-RAM pre-solve gate, motion-session memory priority and ≥8 GB root floor.
No additional router concurrency or relaxed memory gate is implied.
