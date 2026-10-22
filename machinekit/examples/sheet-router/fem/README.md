# SR5 structural screen

## Evidence retention (2026-10-20)

Cited historical runs are compressed under `build/sheet-router-evidence/`;
`modal.tar.gz` and `system.tar.gz` are retained on the separate DEV filesystem at
`/media/joao/DEV/materia-fem-runs/sheet-router-evidence/` because DATA1 is full.
The brace archive is also there. Verified hashes are recorded in the relocation
record and brace provenance.
The freeze archive was moved, byte-for-byte verified, to
`/media/joao/DATA1/materia-router-evidence/freeze.tar.gz` on 2026-10-22 to keep
root above 8 GB free; see [relocation hash](brace-results/evidence-relocation.json).
`archive-index.json` maps each original directory to its archive. Loose binaries,
packets and solver output were removed after archive validation; no tracked files
were deleted. Paths below use an on-demand extraction directory. For example:

```sh
mkdir -p build/sheet-router-evidence/unpacked
tar -xzf /media/joao/DATA1/materia-router-evidence/freeze.tar.gz -C build/sheet-router-evidence/unpacked
```

Extract only the needed archive and remove its loose copy when finished. Old
embedded source/hash path names identify the original run, and remain unchanged.
New SR6 and later checks use `build/sheet-router-work/` as disposable working space.

## SR6 prepared-joint weld qualification (2026-10-20)

**132/132 weld groups pass on each bed at 100 million full-load reversals.**
This replaces the arbitrary-moment allocation for the original 38/36 open
groups: 28 per bed pass with actual reactions, while 10/8 grazing groups are
removed with deliberate non-weld cope clearances. Fabrication release remains
gated on the named shop/material/epoxy acceptances in
[the packet](../fabrication/README.md).

| Bed Y | Worst X / Y stiffness, N/µm | Joint-model first mode, Hz | Contour / rapid residual, mm | Maximum weld range, MPa |
| --- | --- | ---: | --- | ---: |
| 2600 | 2.2067 / 2.0499 | 27.4295 | 0.013237 / 0.013474 | 11.8251 |
| 3150 | 2.1712 / 2.0426 | 26.9863 | 0.020090 / 0.012841 | 12.4316 |

Limits remain ≥2 N/µm and ≤0.05 mm motion error. The conservative FAT36
cut-off is **12.6693 MPa**, with γMf=1.15 and the m=3/m=5 two-slope curve.
The governing long-bed rear-frame corner has 1.9% modelled fatigue margin.
No lifetime or fatigue category is relaxed. Maximum factored throat strength
demand is 32.233 MPa against 207.846 MPa on the assumed S235 material basis.

The model uses 68 separately prepared stocks, actual spatial weld runs and
exact rigid-offset MPCs. It removes old proximity ties and replaces the old
lumped beam-foot springs with the two accessible physical fillets per foot.
Six positive spring quadrature points preserve each contact's full line-group
stiffness; forces/moments are recovered directly from native spring reactions.
Actual 5/6/8 mm legs and open-end/full-size-tab effective lengths are used.
See [the derivation and detail corrections](../fabrication/WELD_QUALIFICATION.md).
These frequencies belong to this prepared-joint idealisation; the older
merged-joint frozen-layout results below retain their own model.

Coverage is nine full-Z static poses per bed at 50 mm mesh, plus 100 mm checks
at the governing modal, middle and fatigue-controlling poses. Both governing
modal poses use 40 modes. Dynamic checks use 20/40 modes, 0.5%/1% damping,
the configured 3 m/s² and 20 m/s³, 20 m/min contour and 40 m/min rapid, with
0.5/0.25 ms steps. Both time-step and modal-truncation checks pass: maximum
tool change is 0.0000113 mm; maximum modal-truncation weld-range change is
0.0094 MPa. All 24 static packets pass native force-balance and coupling-energy
guards (worst relative force imbalance 0.000203; artificial energy fraction
0.0000413); worst modal mass/strain-energy mismatch is below 1%.
The native wrench fixture, seven effective-length/life regressions, 33 mill
plus seven router exporter assertions, and fresh two-bed CAD exports pass.

Reproduction uses existing CCX and the retained planner sampler, without an
OCCT rebuild. Extract `/media/joao/DEV/materia-fem-runs/sheet-router-evidence/sr6-qualified.tar.gz` under
`build/sheet-router-evidence/unpacked/`, then from the repository root:

```sh
machinekit/examples/sheet-router/fem/check-welds.sh \
  build/sheet-router-evidence/unpacked/sr6-qualified/packets \
  machinekit/examples/sheet-router/fabrication \
  build/sheet-router-evidence/unpacked/sr6-qualified/previous-weld-details \
  build/sheet-router-work/weld-rerun
```

The driver publishes a review packet under its output directory and never signs
shop release. Per-group committed tables and source hashes are in
[fabrication/validation.json](../fabrication/validation.json). The retained
archive includes all native decks, reactions, modal/dynamic checks and logs.


## Approved SR-D3 rerun and structural freeze (2026-10-20)

**Pass; freeze the overall structure at `71f330894` plus `c71e0d17d`.** The
user-approved decision (`663a51bb8`, evidence `1ba79b551`) replaces the 40 Hz gate
with static X/Y ≥2 N/µm and motion-induced contour/rapid residual ≤0.05 mm.
Use jerk-limited contour timing, with per-move ZVD as fallback. Required
[MotionKit OM-R1](../../../../motionkit/plans/ONLINE_MOTION.md) implements that
execution policy; SR13/SR16 and measured-mode commissioning remain open.
No deep side girders or local plate/block/rail changes were adopted.

Full anchored rerun: **36 solves**, both beds, nine poses per bed, full Z200,
100/50 mm meshes, eight supports per bed at 50,000 N/mm XYZ. Maximum mesh
changes are 1.194% static, 1.848% for all twenty envelope modes, and 1.845% for
the extended forty modes. Reciprocity, load balance and energy checks pass.
The full native assembly/clearance sweep passes **426 assertions**; guide
coverage, the 149 mm ram gap, 367.75 mm tool lever and front sheet sweep pass.

| Bed mm | Worst X / Y N/µm | Lowest global / gantry Hz | Dynamic governing X/Y/Z mm |
|---|---:|---:|---|
| 2600 | 2.165 / 2.051 | 27.001 / 27.434 | 800 / 0 / 200 |
| 3150 | 2.109 / 2.044 | 24.239 / 26.986 | 800 / 3150 / 200 |

The short-bed governing gantry mode is predominantly X motion; the long-bed
mode is predominantly Y motion/gantry roll. Frequencies, participation and
strain-energy shares are retained in `sr-d3-results.json`; full body motion is
in the solve reports. The extra pair adds 22.18/22.34 kg fixed CAD mass,
respectively, versus the earlier gross-stock estimate of 25.4 kg; moving mass
remains 196.533/196.650 kg. The original middle pair stays in place.

Dynamic checks use all forty modes at each bed's newly governing pose. The
shaper is tuned from those results, not the former 26.05352 Hz constant.
The following maxima include nominal and half-step results, 0.5/1/2/5% damping,
and ±10/15% frequency variations. Limits are 3 m/s², configured default jerk
20 m/s³, 20 m/min contour and 40 m/min rapid.

| Bed mm | Jerk-limited contour / residual mm | With ZVD contour / residual mm | ZVD delay/move |
|---|---:|---:|---:|
| 2600 | 0.01418 / 0.01232 | 0.000873 / 0.000131 | 36.45 ms |
| 3150 | 0.01926 / 0.01206 | 0.001508 / 0.000112 | 37.06 ms |

The native TOPPRA acceleration-step policy is retained as a comparison, not
accepted as the required execution policy. A six-edge contour adds 218.7/222.3 ms
with per-move ZVD. The coarse long-bed basis contains five modes above the 50%
gantry-energy classification threshold, the fine basis six; all forty modes are
retained and checked by index, so a classification threshold does not discard
participating frame modes or become an acceptance gate.

**This is a reduced-model structural freeze, not cutting or shop release.**
Damping remains unmeasured at 1%; servo, backlash, cutting-force and cutter/spindle
compliance dynamics are excluded. SR6-C tap/impact or sine-sweep measurements at
the governing poses must replace modeled frequencies/damping, tune the shaper,
and repeat the 0.05 mm checks. The initial [SR6 fabrication packet](../fabrication/README.md)
now contains real tube-end/cope preparation and connected seam graphs on both
beds, plus the DIY relief/rail-seat/alignment route. Its preliminary weld-load
screen has unresolved groups; this is still a gated draft, not shop release.

Durable condensed evidence: [`sr-d3-results.json`](sr-d3-results.json). Full
hashed packets, all 36 solves, forty-mode runs and dynamic traces live under
`build/sheet-router-evidence/unpacked/sheet-router-freeze/`; `packets/acceptance.json` records the combined
assessment. Run `study.py`, `report.py`, `extended_modes.py`, then
`dynamic_error.py --work-y 2600` and `--work-y 3150` with the same packet/extended
paths, and finally `acceptance.py --packets … --dynamic …`. Analytical SDOF and
ZV/ZVD cancellation/DC fixtures pass. Native libraries and OCCT were reused.


## Raised Z / table / rack datum: separate clearance revision

The subsequent CAD revision raises the whole fixed-Z-block/moving-rail pattern
60 mm, with matching table, setter, rack and fork datums. X plate thickness stays
40 mm; its height grows from 395.75 to 455.75 mm. The 557.5 mm rails and 200 mm Z
travel remain. Native geometry verifies **367.75 mm beam-to-tool** and **149 mm
ram-to-sheet clearance** on both beds; rail coverage remains valid throughout
travel. The table rises to Z614.25 and the front brace upper joints drop to Z430.
A 1600 mm-wide, 40 mm-thick sheet swept through the front mouth intersects none of
the base, front/side frame or rail pads. All 426 assembly/clearance and 82
sizing/planner assertions pass. Long-bed moving Y mass is 196.650 kg.

This is a clearance revision, not a new 40 Hz qualification. Its four midpoint
50 mm structural spot checks (two X positions on each bed) give:

| Bed / X / Y mm | X / Y N/µm | Global / gantry Hz |
|---|---:|---:|
| 2600 /0 /1300 | 2.349 /2.405 | 25.842 /28.646 |
| 2600 /800 /1300 | 2.377 /2.028 | 25.857 /26.693 |
| 3150 /0 /1575 | 2.340 /2.402 | 23.764 /27.489 |
| 3150 /800 /1575 | 2.367 /2.023 | 23.764 /25.867 |

The full new pose/mesh envelope remains pending. **This predates the approved motion-error SR-D3 decision below.** The
motion-error, shaping and architecture calculations below use the preceding
f47ed13f4 geometry; they are not silently relabelled as validation of the raised
datum. Native packets and checks are in `build/sheet-router-evidence/unpacked/sheet-router-system/cad`,
`cad-spot`, `layout.log`, `clearance.log` and `sizing.log`.

The optional legacy desktop-router fingerprint check fails identically on clean
HEAD and with this patch (actual `88f4decf…`, expected `0aaf594d…`). The saved
pre-change binary still passes all 59 legacy assertions. No fingerprint fixture
was changed; the required router assembly, sizing and export checks above pass.

## Motion-error requirement and architecture screen (2026-10-20)

**SR-D3 decision, 2026-10-20:** the user replaced the ≥40 Hz gate with static X/Y ≥2 N/µm and ≤0.05 mm motion-induced contour error and rapid residual, using jerk-limited contour timing and per-move ZVD fallback. Modal frequencies are reported. Required MotionKit OM-R1 and measured-mode commissioning gate cutting acceptance. The proposed alternative below is an engineering motion-error allocation, not approval to build or a total part-tolerance claim. No local plate/block/rail iteration was adopted.

### Proposed dynamic-error allocation and actual planner limits

Use ≤ 0.05 mm tool/workpiece geometric contour deviation during finishing, and ≤ 0.05 mm residual displacement after rapid-to-cut handover, as proposed in the review. The 0.05 mm value comes from that suggested engineering allocation, not an industry-wide wood/plastics tolerance. It is now the approved motion allocation; product drawings and finishing loads still govern total part tolerance. Servo tracking, backlash, cutter/spindle compliance and cutting-force dynamics are excluded. The 300 N structural load alone gives about 0.140 mm static Y deflection; a 50 N finishing load gives 0.0234 mm. The assumed drive backlash is about 0.054 mm. Thus these motion-only results do **not** establish0.05 mm finished-part accuracy.

Planner caps are3 m/s²,20 m/min contour and 40 m/min rapid. `PlanningLimits.hx` supplies an assumed default jerk of 20 m/s³. Actual CNC FollowPath timing uses TOPPRA without a certified jerk bound; its acceleration phases have steps. Native Ruckig is therefore reported separately as the jerk-limited execution alternative, not silently substituted for today’s contour timing. `planner_sample.cpp` calls the prebuilt MotionKit libraries directly. Their reserves give39.2 m/min initial rapid and 19.960 m/min final cut in the TOPPRA transition; Ruckig reaches40/20. No OCCT/native library was rebuilt.

Frozen governing pose: long bed, X=800/Y=1575, full Z. Test contour is a 300×180 mm L-shaped panel with a 180×80 mm notch, six exact-stop edges. Transition is a 300 mm Y move from rapid to contour speed. Damping is assumed1%, with 0.5/2/5% damping and ±10/15% frequency tests. Shapers are tuned to the first **tool-participating gantry** mode,26.05352 Hz, rather than the lower mostly-common frame mode.

| Planner | Structure / shaping | Peak contour error mm | Rapid-to-cut residual mm | Added time per move ms |
|---|---|---:|---:|---:|
| toppra | 26 Hz, none | 0.07275 | 0.08528 | 0.00 |
| toppra | 26 Hz, ZV | 0.02844 | 0.00826 | 19.19 |
| toppra | 26 Hz, ZVD | 0.02551 | 0.00377 | 38.38 |
| toppra | 40 Hz stiffness-scaled, unshaped | 0.08372 | 0.03695 | 0.00 |
| ruckig | 26 Hz, none | 0.01120 | 0.00831 | 0.00 |
| ruckig | 26 Hz, ZV | 0.00098 | 0.00030 | 19.19 |
| ruckig | 26 Hz, ZVD | 0.00089 | 0.00017 | 38.38 |
| ruckig | 40 Hz stiffness-scaled, unshaped | 0.00093 | 0.00076 | 0.00 |

Table values use0.25 ms time steps; comparison with 0.5 ms changes contour peaks by at most0.00074 mm and residual peaks by 0.00105 mm. Residual is the maximum norm during0.5 s after the command/shaper tail, not just a conveniently sampled endpoint. The40 Hz case uniformly scales stiffness by(40/26.05352)² at unchanged mass/mode shapes; residual flexibility is scaled too. It is not a proposed CAD design. Frequency alone does not bound a specific acceleration-pulse response: phase changes can worsen the contour peak while reducing peak elastic following error.

ZV uses two positive impulses separated by half the damped period; ZVD is their convolution, giving three impulses and one period duration. [NASA’s input-shaping study](https://ntrs.nasa.gov/api/citations/19950024365/downloads/19950024365.pdf) gives the command-shaping basis. Each exact-stop edge drains its filter before the next edge to preserve corner geometry: add19.19 ms for ZV or38.38 ms for ZVD per move,115/230 ms for this six-edge contour. Continuous filtering would have a different time/corner-rounding tradeoff and was not substituted here. Cutting events must follow the shaped handover, not the original timestamp. No shaper has been installed in the controller.

Under the damping/frequency uncertainty screen, maximum TOPPRA contour/residual errors are:

| Case | Maximum contour mm | Maximum residual mm |
|---|---:|---:|
| 26 Hz none | 0.35803 | 0.18605 |
| 26 Hz ZV | 0.07374 | 0.04022 |
| 26 Hz ZVD | 0.03726 | 0.00948 |
| 40 Hz unshaped | 0.08892 | 0.07011 |

ZVD remains inside the proposed motion allocation in these cases; ZV does not under ±15% frequency variation. Jerk-limited current-design motion stays below0.013 mm contour and 0.010 mm residual without shaping in this screen, at a longer contour time (4.435 s versus3.554 s TOPPRA; TOPPRA+ZVD is 3.784 s). These are conditional model predictions requiring measured damping, modal tracking over travel and actual execution/servo validation.

### Tool participation and truncation check

For each mode, the tool-minus-workpiece eigenvector is multiplied by its generalized inertial-force participation for the actual X- or Y-moving mass set, divided by generalized mass. We integrate the damped modal equations against native sampled acceleration. Static residual flexibility preserves the full FE acceleration-load compliance; it is not treated as additional dynamic modes.

| Mode | Hz | X→tool-X acceleration residue | Y→tool-Y acceleration residue |
|---|---:|---:|---:|
| 1 | 24.178 | 0.091399 | 0.000037 |
| 2 | 26.054 | -0.000002 | 0.713745 |
| 3 | 33.548 | -0.000001 | 0.026930 |
| 4 | 34.048 | -0.000053 | 0.000503 |
| 5 | 34.504 | 0.000021 | -0.002983 |
| 6 | 35.509 | 0.005409 | 0.000128 |
| 8 | 41.047 | 0.002900 | 0.017678 |
| 9 | 42.934 | 0.117002 | 0.000878 |
| 13 | 52.290 | 0.925437 | 0.000007 |

The first3/6 global modes are insufficient after first-mode shaping: TOPPRA+ZVD predicts only0.008 mm contour error with six modes, versus0.026 mm with 40. The52.29 Hz X-carriage/drive mode has strong tool participation. Twenty and forty modes agree within about 0.0006 mm for the principal contour/residual results; the forty-mode result is retained. Peak **elastic following error**, which includes tangential lag rather than only contour-normal deviation, is 0.205 mm for current TOPPRA and 0.101 mm with ZVD. It must not be confused with geometric contour error.

### One architecture screen at the governing pose

Anchored floor baseline is retained. These are100/50 mm single-pose screens, not adopted CAD. Fixed-mass estimates use gross stock corrected by the baseline CAD-union/gross-stock ratio; no supplier fabrication drawing is implied. The deep-girder case uses the review’s allowed RHS 300×200 alternative, not an unmodelled claim for welded skins.

| Variant | Δgantry Hz | Δglobal Hz | ΔX / ΔY N/µm | Δmoving / Δframe kg | Beam-to-tip mm |
|---|---:|---:|---:|---:|---:|
| deep side girders | -0.083 | +1.792 | -0.002 / -0.004 | +0.00 / +308.74 | 427.75 |
| additional leg pair | +1.431 | +0.527 | +0.026 / +0.028 | +0.00 / +25.42 | 427.75 |
| beam down 10 mm | +0.043 | +0.080 | -0.003 / -0.068 | -5.26 / +0.00 | 417.75 |
| top/front X rails | -0.217 | -0.089 | -0.235 / +0.164 | +2.75 / +0.00 | 427.75 |

**deep side girders**: Replace both under-table side stringers with closed RHS 300 vertical x200x6; keep top datum fixed. Existing crossmembers/braces retained. Rigid section offsets connect real intersecting members. Gross-stock mass uses baseline CAD-union/gross ratio; local weld/wall effects excluded.

**additional leg pair**: Baseline already has a middle pair: six feet at three stations. Add a fourth pair directly beneath Y=1575 rather than counting the existing middle pair twice. Added posts/feet share the anchored 50,000 N/mm XYZ support assumption.

**beam down 10 mm**: 20 mm feet instead of 30 lower the beam 10 mm. X rails and Z ram remain at their original absolute heights: ram-to-sheet gap stays149 mm. Upper front X rail has only5 mm nominal edge margin; 60 mm beam lowering is geometrically infeasible with this rail seating and clearance. Shorter feet are not ideal rigid links.

**top/front X rails**: Upper X rail on beam top, lower rail on front: roll-couple span214 mm instead of 150 mm. Rotate block radial/lateral stiffness and component inertia; add a flexible20 mm horizontal bridge, including its2.75 kg mass. No rigid bridge bypass. Tool lever and 149 mm ram clearance unchanged; mounting is a concept, not collision-checked CAD.

The baseline already has six feet, including a middle pair at Y=1411.375; the added pair is at governing Y=1575. The top/front rail option includes rotated directional block springs and component inertia, and a flexible20 mm bridge (not a rigid shortcut). Larger beam lowering was rejected as geometrically infeasible with the current149 mm ram clearance and rail seating. The10 mm case keeps the entire Z stack at its original absolute height.

Coarse/fine changes in **frequency deltas** are below0.0042 Hz; all variants retain the same tracked gantry branch (body-motion MAC≥ 0.996). The additional leg pair is the only useful modal improvement here, but27.484 Hz is still far below40. Do not add309 kg of deep side members or adopt the split-rail bridge on this evidence.

**Recommendation:** validate a jerk-limited contour execution policy and/or per-move ZVD against measured modes and finishing requirements before redesigning the entire machine for a frequency proxy. Retain SR-D3 until the user chooses. Continue the independent clearance work: raise the complete Z-guide/ram datum 60 mm together with the table/rack, recover the tool lever to 367.75 mm while keeping149 mm clearance, and lower the front braces out of the sheet-loading path. This requires a taller X plate, not merely moving the spindle relative to its plate. No architecture variant above was selected.

Artifacts remain on root under `build/sheet-router-evidence/unpacked/sheet-router-system/`: native planner sampler, `dynamic/dynamic-error.json`, `dynamic/responses.{svg,png}`, `architecture/architecture.json`, paired coarse results and source/provenance hashes. Independent SDOF step and ZV/ZVD cancellation/DC-gain fixtures pass.

## Anchored modal diagnosis (2026-10-20)

Full 100/50 mm, nine-pose, two-bed rerun of the f47ed13f4 CAD. Six anchored supports each use 50,000 N/mm in XYZ, with no rotational clamp. Foundation values are assumptions, not anchor/slab qualification. All masses and the original 428 mm tool lever are retained.

| Bed Y | Worst X N/µm | Worst Y N/µm | Lowest global Hz | Lowest gantry Hz |
|---|---:|---:|---:|---:|
| 2600 | 2.090 | 2.142 | 26.216 (X) | 26.948 (Y) |
| 3150 | 2.044 | 2.135 | 24.177 (X) | 26.054 (Y) |

All 36 solves pass the numerical gates: maximum static mesh change 0.994%,
twenty-mode frequency change 1.741%, force-balance error 0.0210%, static energy
error 0.0042%, and artificial rigid/weld connector compliance 0.0733%. Modal
strain energy agrees with independently integrated modal kinetic energy within
1.352%; the beam/spring fixture also recovers 96.55% spring energy for its soft
rotational-joint mode. Six-gantry-mode profiles below additionally pass forty-mode
convergence. Minimum combined compressive support margin is 1991 N.

The static target passes under the anchored assumptions, but the 40 Hz gantry
target fails. At the long-bed centre, the 26.054 Hz mode is beam/head Y sway with
beam roll about X and non-rigid beam bending: beam 31.3%, frame tubes 28.2%,
Y drive 23.7%, Y blocks 6.6%, floor 3.9% of strain energy. The Y-drive spring
combines rack mesh, reducer and preload hinge. Within it, the hinge takes 64.9%
of compliance (about 15.4% of this mode’s total strain energy), mesh 17.1%
and reducer 18.0%. Its X/Z carriage
elastic energy is small; adding head stiffness or mass does not address its cause.
The lowest 24.177 Hz global mode is X frame/table sway: frame tubes 50.3%,
braces 27.4%, floor supports 17.8%.

The free-standing long-bed centre reproduces the former 23.7 Hz concern at
23.692 Hz: Y frame/beam/head sway, with floor supports 30.2%, frame tubes 24.8%,
beam 17.8% and Y drive 14.8% of strain energy. Its first global X-sway mode is
9.985 Hz, versus about 5.2–6 Hz before bracing; this free-standing number is a
single-pose sensitivity, not a full free-standing envelope. Anchored global minima
are the full-pose values in the table above. Lower frame modes are not dismissed
by the moving-mass classification.

Retain the 40 mm X plate and aluminium Z for the next geometry pass. Returning X
to 25 mm gains only 0.390 Hz while failing static stiffness (1.986 X / 1.494 Y
N/µm at the governing modal pose); steel Z adds 22.95 kg and loses 1.357 Hz.
Increasing Y block spacing adds 5.20 kg for 0.172 Hz, and HGR30 adds mass without
helping this Y mode. None of these swaps approaches 40 Hz. This is a coupled
frame/beam/Y-drive stiffness problem, rather than a defect cured by stiffening
the Z ram further. Mass reduction remains useful only while preserving the
static budget; pocketing needs its own structural check.

Recommended next geometry: recover the 60 mm tool lever through the fork/riser
and matching table/setting-datum redesign, while correcting front-brace loading
access. Keep the 40 mm X plate and aluminium ram. The subsequent modal stiffness
iteration must target the slab-to-Y-rail/frame-to-beam load path and Y drive hinge,
using the measured energy split; lever recovery alone is not claimed to reach
40 Hz. Fabrication/access decisions and machining requirements are settled in
[SOURCING.md](../SOURCING.md#fabrication-and-access-decisions-after-the-braced-frame-screen).

Energy columns below are percentages of **modal strain energy**: native CalculiX ELSE totals plus ½kΔu² for every spring. KE is moving-mass kinetic-energy share; ≥50% defines the reported gantry class. Y guides credit block springs only; Y rail bending is not resolved. X includes plate, guide and drive springs; Z includes ram, ribs, rails, guide/screw and saddle joints. Table and artificial rigid offsets are shown so each row accounts for 100%.

Shapes use global XYZ. Each body’s direction is its RMS displacement direction. Beam rotation is a least-squares rigid-body fit, normalized to 1 mm maximum physical-node motion; bend residual is the remaining normalized RMS displacement. Eigenvector sign is arbitrary. These observations distinguish translation, rotation and beam bending without pretending the entire gantry is rigid.

### 2600 mm bed: first six global modes

Pose X=1600, Y=1300, full Z. Lowest frame-dominant mode is 26.216 Hz. These are the first six modes at the lowest-global-frequency pose, not six copies of one branch at different poses.

| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 26.216 | 36.4 | frame X, table X, beam X, X carriage X, Z ram X, spindle X; beam RX +0.173 mrad/mm, bend residual 0.017 | 18.7 | 48.1 | 26.9 | 0.6 | 0.2 | 0.1 | 0.8 | 4.2 | 0.2 | 0.2 | 0.0 |
| 2 | 29.259 | 89.4 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RX +1.223 mrad/mm, bend residual 0.102 | 6.5 | 18.6 | 3.6 | 8.0 | 34.6 | 2.4 | 26.0 | 0.3 | 0.1 | 0.1 | 0.0 |
| 3 | 34.978 | 0.9 | frame Z; beam RX -0.033 mrad/mm, bend residual 0.005 | 5.4 | 90.5 | 2.8 | 0.0 | 0.1 | 0.0 | 0.2 | 0.0 | 0.0 | 0.7 | 0.1 |
| 4 | 40.298 | 81.0 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RZ +0.739 mrad/mm, bend residual 0.115 | 2.2 | 20.1 | 2.4 | 0.5 | 23.9 | 0.2 | 50.1 | 0.4 | 0.1 | 0.0 | 0.0 |
| 5 | 42.761 | 11.2 | frame Z, beam Z, X carriage Z, Z ram Z, spindle Y; beam RX +0.124 mrad/mm, bend residual 0.128 | 5.9 | 82.4 | 3.7 | 0.3 | 0.6 | 0.2 | 6.2 | 0.4 | 0.1 | 0.2 | 0.0 |
| 6 | 43.778 | 6.5 | frame X, table X, beam Z, X carriage Z, Z ram Z, spindle Z; beam RZ -0.099 mrad/mm, bend residual 0.076 | 5.6 | 74.2 | 6.3 | 3.2 | 0.7 | 0.8 | 8.7 | 0.2 | 0.1 | 0.1 | 0.0 |

### 2600 mm bed: first six gantry-dominant modes

Pose X=800, Y=1300, full Z, chosen for the lowest gantry frequency. Forty modes were extracted on both meshes to resolve six gantry-class modes; all forty frequencies converge within 1.711%.

| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2 | 26.948 | 93.2 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RX -1.771 mrad/mm, bend residual 0.123 | 4.6 | 16.2 | 2.9 | 9.7 | 29.2 | 2.7 | 34.3 | 0.3 | 0.1 | 0.1 | 0.0 |
| 4 | 40.890 | 73.5 | beam Z, X carriage Z, Z ram Z, spindle Z; beam RX +0.535 mrad/mm, bend residual 0.140 | 14.3 | 23.3 | 2.8 | 1.2 | 0.8 | 1.6 | 49.7 | 3.0 | 2.9 | 0.2 | 0.0 |
| 5 | 41.543 | 78.9 | beam Y, X carriage X, Z ram X, spindle X; beam RZ -0.842 mrad/mm, bend residual 0.125 | 2.3 | 20.9 | 2.2 | 0.4 | 19.4 | 0.1 | 51.6 | 2.9 | 0.2 | 0.0 | 0.0 |
| 12 | 53.340 | 74.6 | X carriage X, Z ram X, spindle X; beam RZ -0.079 mrad/mm, bend residual 0.020 | 5.8 | 10.8 | 5.3 | 1.2 | 1.3 | 0.3 | 6.8 | 65.6 | 2.7 | 0.3 | 0.0 |
| 30 | 90.867 | 61.5 | frame Z, table Z, beam Z, X carriage X, Z ram X, spindle X; beam RX -0.156 mrad/mm, bend residual 0.122 | 18.2 | 9.5 | 2.1 | 0.4 | 0.0 | 1.2 | 66.8 | 0.7 | 0.8 | 0.1 | 0.1 |
| 37 | 102.085 | 93.9 | beam Z, X carriage Z, Z ram Z, spindle Z; beam RX -0.546 mrad/mm, bend residual 0.051 | 0.5 | 4.8 | 0.4 | 0.5 | 0.0 | 1.6 | 12.8 | 41.8 | 37.4 | 0.0 | 0.0 |

### 3150 mm bed: first six global modes

Pose X=1600, Y=1575, full Z. Lowest frame-dominant mode is 24.177 Hz. These are the first six modes at the lowest-global-frequency pose, not six copies of one branch at different poses.

| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 24.177 | 31.9 | frame X, table X, beam X, X carriage X, Z ram X, spindle X; beam RX -0.115 mrad/mm, bend residual 0.013 | 17.8 | 50.3 | 27.4 | 0.5 | 0.1 | 0.1 | 0.5 | 3.0 | 0.1 | 0.2 | 0.0 |
| 2 | 27.920 | 71.2 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RX -1.345 mrad/mm, bend residual 0.103 | 4.9 | 36.7 | 4.5 | 4.5 | 24.9 | 1.5 | 22.6 | 0.2 | 0.1 | 0.1 | 0.0 |
| 3 | 33.603 | 3.3 | frame Z, beam Z, X carriage Y, Z ram Y, spindle Y; beam RX +0.083 mrad/mm, bend residual 0.095 | 3.0 | 88.1 | 3.9 | 0.4 | 0.2 | 0.1 | 3.9 | 0.1 | 0.0 | 0.3 | 0.0 |
| 4 | 34.005 | 2.0 | frame X, table X, beam Y, X carriage Y, Z ram Y, spindle Y; beam RZ -0.342 mrad/mm, bend residual 0.031 | 1.9 | 79.6 | 6.1 | 4.1 | 2.8 | 0.9 | 4.4 | 0.0 | 0.0 | 0.1 | 0.0 |
| 5 | 34.517 | 1.1 | frame Z; beam RX -0.061 mrad/mm, bend residual 0.008 | 5.0 | 90.0 | 3.1 | 0.2 | 0.1 | 0.1 | 0.7 | 0.0 | 0.0 | 0.8 | 0.1 |
| 6 | 35.267 | 12.2 | frame Z, beam Y, X carriage Y, Z ram Y, spindle Y; beam RZ +0.355 mrad/mm, bend residual 0.064 | 0.9 | 77.3 | 2.5 | 1.3 | 1.6 | 0.9 | 15.2 | 0.1 | 0.0 | 0.1 | 0.0 |

### 3150 mm bed: first six gantry-dominant modes

Pose X=800, Y=1575, full Z, chosen for the lowest gantry frequency. Forty modes were extracted on both meshes to resolve six gantry-class modes; all forty frequencies converge within 1.540%.

| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2 | 26.054 | 81.3 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RX -1.813 mrad/mm, bend residual 0.122 | 3.9 | 28.2 | 3.9 | 6.6 | 23.7 | 1.9 | 31.3 | 0.2 | 0.1 | 0.1 | 0.0 |
| 8 | 41.047 | 68.3 | frame Z, beam Z, X carriage Z, Z ram Z, spindle Z; beam RX +0.577 mrad/mm, bend residual 0.128 | 20.4 | 19.7 | 5.2 | 1.0 | 0.4 | 1.4 | 45.1 | 3.2 | 3.2 | 0.3 | 0.0 |
| 9 | 42.934 | 89.0 | beam Y, X carriage X, Z ram X, spindle X; beam RZ +0.654 mrad/mm, bend residual 0.107 | 2.1 | 7.8 | 1.6 | 1.0 | 24.0 | 0.8 | 57.7 | 4.8 | 0.3 | 0.0 | 0.0 |
| 13 | 52.290 | 76.5 | X carriage X, Z ram X, spindle X; beam RZ -0.087 mrad/mm, bend residual 0.013 | 5.6 | 12.9 | 4.1 | 1.4 | 1.5 | 0.3 | 6.2 | 64.7 | 2.9 | 0.4 | 0.0 |
| 30 | 88.317 | 70.2 | frame Z, table Z, beam Z, X carriage X, Z ram X, spindle X; beam RX -0.130 mrad/mm, bend residual 0.126 | 10.5 | 11.1 | 1.9 | 0.5 | 0.0 | 1.4 | 73.0 | 0.7 | 0.6 | 0.1 | 0.1 |
| 39 | 101.523 | 89.8 | beam Z, X carriage Z, Z ram Z, spindle Z; beam RX -0.540 mrad/mm, bend residual 0.049 | 1.2 | 7.3 | 1.3 | 0.5 | 0.0 | 1.5 | 11.8 | 39.6 | 36.5 | 0.1 | 0.0 |

### Single-pose sensitivities

Lowest gantry-dominant mode across both beds/all fine-mesh poses; governing target failure. Bed 3150, X=800, Y=1575, full Z, 50 mm mesh. These are analytical property/layout variants at fixed mounting interfaces, not collision-checked replacement CAD. Y-spacing includes longer steel feet and rails; HGR30 includes catalogue mass increments and ZA radial rigidity. Lateral guide stiffness remains assumed. Changing plate thickness requires reworking the actual mounting stack.

| Variant (ranked by tracked gantry frequency gain) | ΔX N/µm | ΔY N/µm | Δfirst gantry Hz | Δtracked branch Hz | Motion-signature MAC | Δmoving Y kg |
|---|---:|---:|---:|---:|---:|---:|
| X plate 25 mm | -0.272 | -0.641 | +0.390 | +0.390 | 1.000 | -6.531 |
| Y blocks 400 mm | -0.004 | +0.040 | +0.172 | +0.172 | 0.999 | +5.200 |
| welds x0.1 | -0.000 | -0.000 | -0.001 | -0.001 | 1.000 | +0.000 |
| Y HGR30 | +0.000 | +0.001 | -0.261 | -0.261 | 1.000 | +1.480 |
| steel Z ram | +0.415 | +0.309 | -1.357 | -1.357 | 1.000 | +22.946 |
| free levelling feet | -0.005 | -0.001 | -2.362 | -2.362 | 0.982 | +0.000 |

Tracked-branch MAC compares normalized body translations and rotations (200 mm rotation scale), not a consistent-mass matrix MAC. Both the classified first mode and matched branch are reported so threshold swaps are visible. Weld ×0.1 changes translation springs from 10¹⁰ to 10⁹ N/mm and rotation from 10¹³ to 10¹² N·mm/rad; these are quasi-rigid numerical connectors, not qualified weld flexibility. Free supports keep six vertical springs but only X at foot 0 and Y at feet 0/3; friction/no-slip is assumed.

HGR30 uses [HIWIN HGH25CA/HGH30CA](https://www.hiwin.com/wp-content/uploads/Linear_Guideway-E.pdf): radial ZA 539→618 N/µm, block mass increment 0.51→0.88 kg and rail increment 3.21→4.47 kg/m. Incremental mass is added to generic CAD masses; rail bending and supplier mounting-height changes are not resolved.

### Added moving mass

| Member | Previous kg | Revised kg | Added kg |
|---|---:|---:|---:|
| xCarriage | 10.886 | 17.417 | +6.531 |
| zCarriage | 6.623 | 12.030 | +5.407 |
| beamFootLeft | 5.258 | 7.886 | +2.629 |
| beamFootRight | 5.258 | 7.886 | +2.629 |
| beam | 71.934 | 72.502 | +0.569 |
| shoeLiftBracket | 0.173 | 0.292 | +0.119 |
| motorYLeftSupport | 0.084 | 0.158 | +0.074 |
| switchZpositiveLimitMount | 0.159 | 0.156 | -0.004 |
| switchZhomeMount | 0.072 | 0.068 | -0.004 |
| switchZnegativeLimitMount | 0.068 | 0.064 | -0.004 |
| motorZSupport | 0.094 | 0.084 | -0.010 |
| motorYRightSupport | 0.084 | 0.064 | -0.020 |
| **Total moving Y** | **175.860** | **193.775** | **+17.915** |

Fixed base braces are not part of moving Y mass. The beam mass also increases slightly because thicker feet change the generated span. Part-level inertia and material mass are changed in each sensitivity, not just a modal denominator.

Artifacts: `build/sheet-router-evidence/unpacked/sheet-router-modal/anchored/{results,summary}.json`, `extended/extended-modes.json`, `sensitivities/sensitivities.json`, and `diagnosis.json`. The full-run solve-time sources, packet hashes and solver hash remain with the results. Extended/sensitivity runs record their source hashes separately. No fabrication release.


### Reproduce the anchored review

Use the existing mill FEM virtualenv and prebuilt CalculiX. Keep all outputs on
root; DATA1 is only read for the earlier moving-mass comparison. The packets used
here are the exact f47ed13f4 exports copied from
`build/sheet-router-evidence/unpacked/sheet-router-stiffness/validated/y{2600,3150}` into `anchored/`.

```bash
export LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
router_python=machinekit/examples/bench-mill/build/fem/venv/bin/python
router_ccx=machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx
router_packets=build/sheet-router-evidence/unpacked/sheet-router-modal/anchored
/home/joao/dev/materia-cache/tools/heavy-run --mem 6G --need 1G --label router-modal-full \
  "$router_python" machinekit/examples/sheet-router/fem/study.py --packets "$router_packets" --ccx "$router_ccx" --floor anchored
"$router_python" machinekit/examples/sheet-router/fem/report.py "$router_packets"
/home/joao/dev/materia-cache/tools/heavy-run --mem 5G --need 1G --label router-modal-sensitivity \
  "$router_python" machinekit/examples/sheet-router/fem/sensitivity.py --packets "$router_packets" --ccx "$router_ccx" --output build/sheet-router-evidence/unpacked/sheet-router-modal/sensitivities
/home/joao/dev/materia-cache/tools/heavy-run --mem 5G --need 1G --label router-extended-modes \
  "$router_python" machinekit/examples/sheet-router/fem/extended_modes.py --packets "$router_packets" --ccx "$router_ccx" --output build/sheet-router-evidence/unpacked/sheet-router-modal/extended
"$router_python" machinekit/examples/sheet-router/fem/diagnose.py --packets "$router_packets" \
  --extended build/sheet-router-evidence/unpacked/sheet-router-modal/extended/extended-modes.json \
  --sensitivities build/sheet-router-evidence/unpacked/sheet-router-modal/sensitivities/sensitivities.json \
  --baseline /media/joao/DATA1/materia-router-sr6/final
```

The native beam/rotation/crossing gate, all 37 mill Python regressions, and both
eccentric Y-drive checks pass. Maximum reserved Y-branch force is 931.07 N; the
Hertz diagnostic is 725.79 MPa and is not the rack acceptance limit. No CAD was
changed in this analysis pass. The six sensitivity geometries and fabrication/access
decisions require their own CAD and supplier qualification before adoption.

## f47ed13f4 CAD and comparison history

The working CAD now selects a 40 mm flat X plate, 30 mm welded steel beam feet,
180 mm moving-Z rail spacing and a 20 mm Z web with two closed side sections
25 × 120 × 6 mm. Sixteen RHS 80 × 80 × 4 frame diagonals brace the sides,
ends and lower plane. The original RHS 200 × 100 × 6.3 beam and 100 mm side
allowance remain. Removing that allowance failed native clearance: the outboard
Z screw intersected the base at X=0. The shoe lift has an offset bracket, and the
right Y motor support is lowered to clear the rack. The final geometry is validated against HEAD in an isolated worktree;
the checks and remaining structural work are recorded below.

The beam model now joins welded frame members at interior intersections, rather
than only at their endpoints. A native fixture compares automatic crossing
connectivity with four explicitly joined arms: both deflect 0.007976131 mm under
300 N, with four branches at one physical node. Welded beam feet use two spatial
contacts across the beam footprint instead of a guessed single rotational spring.
That revision originally used finite levelling-foot supports; the anchored baseline
and free-standing sensitivity above supersede that support selection.
Local weld stress, tube-wall flexibility and the built-up aluminium Z joint still
require qualification.

Model correction must be separated from hardware improvement. `compare.py`
re-solves the previous exact CAD with the corrected model at two critical poses
per bed, recording packet and source hashes. At the long-bed centre, the old
hardware gives 1.751 N/µm X and 0.487 N/µm Y. The earlier export of the revised
hardware gives 2.253 X and 2.135 Y at that pose, but its lowest classified
gantry-dominant mode is only 23.688 Hz. This is a single-mesh diagnostic, not
acceptance evidence. It predates the final motor-support clearance
fix. Mixed frame/gantry modes remain relevant regardless of classification.

Working evidence is under `build/sheet-router-evidence/unpacked/sheet-router-stiffness`: `baseline/` contains
the corrected comparison, `cad/` the earlier revised CAD, and `cad-screen/` its
critical-pose screen. Fresh final CAD packets are in `validated/`, exported from an isolated detached
worktree based on HEAD with only the router patch. Both beds export 171 components
and 39 stock members using the existing OCCT libraries. This avoids the unrelated
casting refactor in the shared checkout. The full anchored two-mesh rerun is reported above; exports alone do not qualify
the structural targets.
The isolated guide gate passes both beds, including closed Z-section validity and
volume, sixteen frame braces, moving-rail reach and selected drives. The final
SR4 sizing/planner gate passes 82 assertions. Shared exporter fixtures pass
33 mill plus seven router assertions, native beam/spring/crossing fixtures pass,
and all 37 mill Python regressions pass. Both beds pass the complete assembly
and clearance sweep: 426 assertions, including travel extremes and tool-rack
approaches. Logs are retained in `build/sheet-router-evidence/unpacked/sheet-router-stiffness/isolated-validation/`.

The revised long-bed moving Y mass is 193.77 kg. The ideal drive-only estimate is
47.23 Hz, falling to 36.78 Hz with half hinge stiffness, or 45.03 Hz with 10% extra
mass. The hinge accounts for 64.9% of series drive compliance.

The historical results below retain their original model and geometry; their
results use their original support/model assumptions. No fabrication release.

## Previous guide-first candidate (SR6, 2026-10-20)

This historical candidate implements the first guide revision in [SOURCING.md](../SOURCING.md).
The initial SR5 results below describe the superseded single-block assembly.
The new candidate uses two X rails/four blocks, moving Z rails/four fixed blocks,
280 mm Y block spacing, flat 25/20 mm X/Z plates, 20 mm steel feet and a direct
two-level spindle saddle. It retains the original steel beam and nominal rack size.
Z is 1610, 200 mm travel and 10 m/min; X and braked Z are 750 W.

New exports default to `build/sheet-router-evidence/unpacked/sheet-router-sr6`. Set `SHEET_ROUTER_FEM_OUTPUT`
to place packets on another volume; pass that same path to `study.py`, `report.py`
and the eccentric-load checker. `SHEET_ROUTER_FEM_BINARY`, `SHEET_ROUTER_BINARY`
and `SHEET_ROUTER_SIZING_BINARY` relocate compiler outputs independently.
This run uses `/media/joao/DATA1/materia-router-sr6/final` for packets
and `/media/joao/DATA1/materia-router-sr6/logs` for checks, because root is nearly full.

The updated model uses per-block radial/lateral translation springs at actual CAD
connectors and zero guide rotational springs. X/Z plate grillages span the contact
locations. Moving Z rails receive a conservative 60% gross rectangular-section
reinforcement proxy; mounting stiffness remains assumed. The spindle saddle is
a rigid body supported by four translational mounting springs; clamp, bearing
and tool flexibility are excluded. Lateral guide stiffness and plate Poisson
coupling still require qualification.

The table reacts through six actual bed-stock seats rather than one nearest stock.
Floor-standing blower and cabinet devices do not contribute to frame mass. Other
fixed masses attach to nearest stock on a fixed 50 mm analysis grid. Twenty modes
are reported, including global frame modes. Gantry-dominant modes use a declared
50% moving-mass kinetic-energy criterion; this is a screening classification,
not a validated separation of frame and gantry dynamics. Static and first-six-mode
convergence gates remain 3% and 5%. Per-case raw field files are gzip-compressed
with integrity verification; decks, tables, logs and source snapshots remain.

The settled CAD passes 426 assembly/clearance assertions for both beds, 292
component assertions, 82 revised SR4 sizing/planner assertions, and the guide/reach
gate. All five legacy gantry hashes and 59 axis assertions remain unchanged.
The shared exporter/centreline fixtures pass (33 mill plus seven router checks),
as do the independent beam/spring native fixture, mill packet/STEP comparison
and 37 mill Python regressions. These checks do not establish the structural
targets. The current candidate remains a screening design, with no fabrication release. The shoe envelope is 150 mm diameter with a 40 mm brush
and a front-directed short hose neck; its lift and rack clearances are checked
with the same assembly poses.

### Settled candidate results

All 36 solves (two meshes, nine full-Z poses, two beds) pass the numerical gates.
Static compliance changes by at most 1.092%; all twenty frequencies by 0.875%.
Maximum force-balance error is 0.0292%, energy error 0.0108%, reciprocity error
0.000031%, and artificial rigid-member compliance 0.0752%. Every support retains
positive compression under the combined envelope.

| Bed Y | Worst X stiffness | Worst Y stiffness | Global first-mode range | Lowest classified gantry-dominant mode |
|---|---:|---:|---:|---:|
| 2600 mm | 1.154 N/µm | 0.481 N/µm | 5.727–6.027 Hz | 27.249 Hz |
| 3150 mm | 1.005 N/µm | 0.480 N/µm | 5.177–5.468 Hz | 30.760 Hz |

The 2 N/µm X/Y and 40 Hz gantry targets still fail. The modal classification is
only the declared ≥50% moving-mass energy screen: mixed frame/gantry modes below
that threshold remain relevant and the global frame modes are not dismissed.
The lower spindle datum necessary for rack clearance leaves approximately 428 mm
from beam centre to longest-tool tip at full Z. Shorter-tool reach remains open.

In the worst Y cut, the Z plate plus its moving-rail reinforcement account for
about 56% of compliance; the X plate contributes about 13% and the gantry beam
about 9%. The worst X case is led by the base/frame contribution. Investigate the
Z load path and frame bracing/supports before selecting a box X carriage. These
are model-based priorities, not fabrication approval.

Worst reserved eccentric Y branch demand is 874.60 N. Both drives pass torque,
reducer, preload and conditional manufacturer rack checks; the mounting-derated
rack limit is 3000 N, versus 935.77 N after including preload-flank demand.
The 715.08 MPa Hertz diagnostic remains visible without
using the old assumed 700 MPa limit to resize the rack. The specified hardened
family, mounting and application factors still require qualification.

The long-bed Y drive-only estimate is 49.58 Hz at 175.86 kg. Halving hinge
stiffness gives 38.61 Hz; 10% extra mass gives 47.27 Hz. Hinge compliance is 64.9%
of the series drive compliance. These estimates do not replace the full structure.

`summary.json`, `results.json`, `eccentric-drives.json`, posed STEP files, native
decks/tables, compressed field files and solve-time source snapshots remain in
the external packet directory named above. `summary.json` records the actual
postprocessor SHA separately from the solve-time source snapshot.

## Initial SR5 baseline (historical)

The current SR3/SR4 assembly is rejected for fabrication. The structural screen is
complete; the SR-D3 targets are not met. A redesigned carriage/guide layout must
be studied before SR6 releases drawings.

The shared `machinekit.structural.fem.AssemblyFemExport` writes posed STEP,
materials, mass, centre of mass, world centroidal inertia and bounds from any
`MachineAssembly`, with a caller-supplied solid selection. Both the mill and router
use it. Fresh mill exports before and after extraction match in all 220 component
records and every byte of the 36 STEP bodies; source/build provenance is excluded
from that comparison because the exporter source changed.

## Reproduce the current candidate

Use the preceding SR6 model description for current runs. The model description
and baseline results below this command section are historical.

Run from the repository root. Reuse the pinned mill Gmsh 4.12.2 environment and
CalculiX 2.21 installation; no native build or new Python environment is needed.
Use the project's `heavy-run` resource gate for compilation and the full study.
The example gate below is the workstation tool used for this run.

```bash
heavy_run=/home/joao/dev/materia-cache/tools/heavy-run
fem_python=machinekit/examples/bench-mill/build/fem/venv/bin/python
fem_ccx="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx"
export LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

"$heavy_run" --mem 10G --need 1G --label sr6-export -- machinekit/scripts/test-sheet-router-fem
"$fem_python" machinekit/examples/sheet-router/fem/verify.py --ccx "$fem_ccx" --output build/sheet-router-evidence/unpacked/sheet-router-sr6/verification
"$heavy_run" --mem 10G --need 1G --label sr6-study -- "$fem_python" machinekit/examples/sheet-router/fem/study.py --packets build/sheet-router-evidence/unpacked/sheet-router-sr6 --ccx "$fem_ccx"
"$fem_python" machinekit/examples/sheet-router/fem/report.py build/sheet-router-evidence/unpacked/sheet-router-sr6
"$heavy_run" --mem 10G --need 1G --label sr6-eccentric -- machinekit/scripts/test-sheet-router-fem --y-loads
```

`--checks-only` runs the Haxe checks without exporting. `--export-only`
exports a geometry iteration without repeating the unchanged exporter fixtures;
run those fixtures separately before completing a study. `--layout` checks the
new guides, moving-rail reach and selected drives; `--baseline` checks legacy
gantry hashes and axis behavior. The optional verifier
arguments `--mill-before <folder> --mill-after <folder>` compare fresh legacy mill
packets. `study.py --one` is a diagnostic centre-pose run; it writes partial output
and cannot produce a completed study. The full study publishes `results.json`
only after every solve, force/energy check and convergence gate passes. Per-case
input decks, solver logs and numerical tables remain beside the report.

## Model and limits

The packet includes all 165 posed components, seven selected STEP bodies, 24 RHS
stock descriptions, plate dimensions, nine full-Z poses and six foot locations.
Each bed length has a 3 × 3 corner/midspan grid at Z = 250 mm. At each pose:

- X/Y/Z cutting cases apply 300 N to the tool and the opposite force to the bed;
- gravity includes all authored part masses;
- X and Y acceleration cases apply 3 m/s² to the corresponding moving masses;
- six modes are solved with the same restraints and mass model.

Gmsh meshes quadratic lines. CalculiX B32R BOX sections represent the hollow
stocks; RECT sections represent full-width plate strips. The beam feet are strips
between the two Y blocks, 140 mm apart. The X plate joins the X block to the Z
column head; the Z carriage is a strip from its block to the spindle attachment.
Stiff offsets carry forces and moments to actual tool/guide positions. Each joint
uses translation and rotation reference nodes and six independent springs.
Direct SPRING2 rotational DOFs on beam nodes are unsuitable here; the analytical
fixture verifies the reference-node coupling.

Assumed inputs are steel E = 210 GPa, aluminium E = 69 GPa, nu = 0.3, guide
translation = 100 kN/mm and individual guide rotation = 10 MN·mm/rad. Bolt springs
are 1 MN/mm and 1 GN·mm/rad. Feet have 50 kN/mm vertical springs and three horizontal
restraints in a 3-2-1 arrangement. The actual SR4 rack/reducer/preload and screw
stiffnesses supply the held-axis drive springs. They include elastic compliance;
backlash, tuning and following error are excluded.

Native stock sections carry distributed mass. Base stock densities are scaled to
match the CAD weldment union mass, avoiding double-counting stock intersections.
Remaining part masses, centres and full inertia tensors are aggregated per rigid
carrier, then represented by six positive point masses. Entire cable chains move
with their carrier, matching SR4's conservative allowance. Fixed accessory masses
are transferred to nearby frame supports; separate accessory floor supports are
not modelled.

The bed reaction is a rigid link to the nearest authored bed stock member. Table
diaphagm action and distributed table attachment are excluded. Spindle mount,
spindle bearings, tool and local tube-wall deformation are rigid/omitted. No weld,
bolt/contact, stress, buckling, fatigue, damping or nonlinear qualification is
claimed. The very large calculated deflections exceed the useful precision range
of a linear model: they reject this layout rather than predict cutting accuracy.

CalculiX's authored beam/section and output definitions are documented in the
[official manual](https://www.dhondt.de/ccx_2.20.pdf). Line connectivity follows the
[Gmsh API reference](https://gmsh.info/doc/texinfo/): Gmsh start/end/mid nodes are
reordered to CalculiX start/mid/end.

## Result, 2026-10-20

| Bed Y | Worst X stiffness | Worst Y stiffness | First mode across poses | Largest gravity frame twist |
|---|---:|---:|---:|---:|
| 2600 mm | 0.006106 N/µm | 0.005849 N/µm | 2.7318–2.7329 Hz | 0.00825 mm |
| 3150 mm | 0.006100 N/µm | 0.005849 N/µm | 2.7316–2.7327 Hz | 0.00902 mm |

Targets are ≥ 2 N/µm XY and ≥ 40 Hz for the first gantry mode. The lowest mode has
substantial tool-to-bed motion, rather than being an irrelevant accessory mode.
The report retains all six frequencies and relative tool-motion indicators.

Worst full-Z X compliance is about 164 µm/N: the X guide contributes 90, the X
plate 45, and the Z guide 26 µm/N. Worst Y compliance is about 171 µm/N: the X plate
contributes 92, X guide 60 and Z guide 15 µm/N. The X plate term alone rejects the
2 N/µm target even if the guide assumptions are made much stiffer. Budgets are
strain-energy contributions for the closed tool/bed load loop, including springs.
Support reactions and acceleration displacements are retained for every case.

The eccentric Y branch sees up to 827.65 N after the 1.5 reserve, including the
300 N cutting vector envelope, Y acceleration and 15 N side drag. The servo torque
and preload still pass, but the original 20 mm engaged face reaches 706.06 MPa
against SR4's assumed 700 MPa contact limit. A **25 mm engaged face on both rack
and pinion** passes that tooth screen (about 632 MPa); its physical fit and the
redesign's changed moving mass must be checked before changing the selected parts.

Decisions for the next design iteration:

- Retain **steel RHS 200 × 100 × 6.3** as the beam baseline. Same-section aluminium
  has only 0.329 times the bending rigidity. Enlarging the beam alone cannot cure
  the dominant carriage/guide terms.
- Replace the open 12 mm aluminium X plate with a **closed box bridge**. Use spaced
  X rails and blocks to support roll as well as pitch/yaw; two blocks on the same
  X rail alone do not fix roll about that rail.
- Replace the single Z block with spaced blocks/rails and obtain supplier stiffness.
- Reject the **12 mm aluminium beam feet** for final design. **20 mm steel** is the
  next candidate, with about 14 times the strip bending rigidity. This is a design
  choice to package and recheck, not an analysed replacement assembly.
- Keep tubes **empty**. Filling the beam adds 39.31 kg/m to its 28.43 kg/m steel mass.
  Even a perfectly bonded 30 GPa fill gives only 1.37/1.24 times strong/weak bending
  rigidity; an unbonded fill gives no credited stiffness. Damping is unquantified.

These candidates are not claimed to meet SR-D3. SR6 must begin with structural
redesign and rerun SR4/SR5 before fabrication drawings are released.

## Verification

The 100/50 mm global meshes use four-times-finer plate-strip spacing. Across all
18 poses, maximum diagonal compliance change is 0.61% and maximum change of the
first six frequencies is 0.31% (gates 3% and 5%). The 2 m RHS cantilever agrees
with the stated bending/shear formula within 2%, and the rotational-spring
fixture within 0.07%. Halving its mesh changes deflection by 0.67%.

Each solve checks positive compliance, summed energy against external work, and
support force equilibrium. Postprocessing checks compliance reciprocity, negligible
artificial-member compliance and compressive foot reactions under gravity plus
the cutting vector and both acceleration envelopes. The smallest remaining foot
compression is about 75 N within this assumed foundation model. The Haxe gate
passes seven shared-exporter/frame-data checks and 33 existing mill datum checks;
the existing mill Python suite passes 37 tests.

Generated evidence is under `build/sheet-router-evidence/unpacked/sheet-router-sr5/`: `summary.json`,
`results.json`, `verification/verification.json`, `y-loads.json`, and
`eccentric-drives.json`. Packet, solver executable, study source and report hashes
are recorded. Generated files are intentionally outside project history.


## SR6 DIY foot-joint comparison

The frozen welded candidate is unchanged. Two assumed bolted/dowelled joints
were solved on both beds at the static-X, static-Y and modal governing poses
(h=50 mm, anchored baseline). The four-quadrant machined-face contact candidate
uses 1.5 MN/mm normal and 0.2 MN/mm transverse stiffness per quadrant, with no
local rotational spring. It gives long-bed governing Y **1.909 N/µm** versus
**2.044** welded, and modal-pose gantry **25.395 Hz** versus **26.986**. The
original lumped bolted spring is softer still. Retain welded feet; these assumed
spring values do not qualify a physical preloaded/dowelled joint. Hardware mass
was held unchanged in this interface-only screen.

All twelve comparisons: [diy-feet-results.json](diy-feet-results.json). Full decks
and results: `/media/joao/DEV/materia-fem-runs/sheet-router-evidence/sr6-diy.tar.gz`, extracted under
`build/sheet-router-evidence/unpacked/sr6-diy/`. The same archive retains prepared
frame STEP files and native service checks; their local weld qualification is
separate from the frozen beam-centreline model.


## Brace removal and full-history J4 screen, 2026-10-22

The frozen CAD is unchanged. The [brace study](BRACE_STUDY.md) compares lower-plane
removal, both uniform vertical-diagonal slopes, their combinations, a front-open
layout, and balanced diagonal orientations. It retains the populated ATC mass,
anchored feet, modal history through rapid entry and five planner histories.
Static stiffness passes, but no reduced layout meets the motion reserve and
weld-fatigue screen. The broader baseline itself needs planner-envelope
requalification: the finer-mesh short-bed contour reaches 0.06320 mm, and a
long-bed dynamic weld range reaches 13.03 MPa against 12.669 MPa. The report
separates the frozen macro model, prepared weld model and numerical solver
rejections. Rack seats/bolt compliance remain the next joint rerun; this screen
does not approve brace removal or release fabrication.
