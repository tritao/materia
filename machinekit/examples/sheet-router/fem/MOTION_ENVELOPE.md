# Motion envelope correction — 2026-10-22

The user accepted retaining every brace. This audit corrects the motion labels
in fdc806321 before adding rack-seat compliance. It grants no CAD release.

## Approved envelope and actual coverage

- Hold selects the published ready jerk-limited stop; resume uses normal planning.
- For 0 < k ≤ 1, a jerk-filtered executor clock scales time. At constant k this
  gives v·k, a·k², j·k³. Changing k also contributes derivative terms; the
  complete chain rule must fit the axis limits and the allocated reserve.
- Above 100%, replan the next window at raised feed, under unchanged axis V/A/J
  limits and the maximum axis feed. Never use k > 1 executor time scaling.

The J4 export names `override-1p5`, `override-3`, `hold-resume-1p5` and
`hold-resume-3` mean **requests at 1.5 or 3 seconds**. All request **75% feed**;
hold adds a 50 ms dwell. The earlier “1.5×/3×” labels were incorrect.
They are neither 150% nor 300% overspeed histories.

Their metadata identifies an **offline ordinary-compiler prefix/splice/tail**
fixture. The producer copies the prefix and calls `retimeOnPath`, or
`stopOnPath` followed by `resumeOnPath`, then certifies the joined result.
It does not export the approved publication-ready stop and filtered executor
clock. All ten pinned exports pass their immutable-manifest checks and
continuous certificates; sampled V/A/J remain within 0.666667 m/s, 3 m/s²
and each configured jerk. Thus these failures cannot be dismissed as
acceleration/jerk violations from multiplying time by three.

Exclude the eight controlled J4 exports from **J5 acceptance**, retaining them
as diagnostics. No raised-feed (>100%) export was used, and no matching
ready-stop/filtered-clock J5 history has been substituted. The jerk-timing
worktree's current ONLINE_MOTION J5 text caps >100% at 100% for its first
landing; this audit records the user's newer next-window replan requirement.
Until that export exists, raised-feed coverage is explicitly **pending**.
Do not manufacture an overspeed history by resampling.

## Governing histories and poses

Every pose below is at full Z=200 mm. All listed governing corners use
frequency ×0.85 and damping 0.005; computed frequencies remain unmeasured.
The full rapid-entry modal history is retained.

| Quantity | Bed | History | Pose X/Y mm | Mesh | Value | Status |
|---|---:|---|---|---:|---:|---|
| Previously quoted contour | 2600 | 75% request at t=3 s, offline retime | 0/1300 | 50 | 0.063195 mm | Old-control diagnostic |
| Previously quoted contour | 3150 | hold/resume at t=3 s, offline stop | 0/0 | 50 | 0.043973 mm | Old-control diagnostic |
| Maximum moving handover | 2600 | nominal; same prefix for all controls | 0/2600 | 100 | 0.047330 mm | Fails 0.04 reserve; passes 0.05 |
| Refined moving handover subset | 2600 | nominal | 800/2600 | 50 | 0.046303 mm | Fails reserve; passes budget |
| Maximum moving handover | 3150 | nominal | 800/0 | 50 | 0.027831 mm | Passes both |
| W066 dynamic weld range | 3150 | hold/resume at t=3 s, offline stop | 800/0 | 50 | 13.027551 MPa | Above 12.669282 MPa; old-control diagnostic |

W066 joins `lower1-p0` to `post13-p0`, the right lower stringer at the added
Y≈1575 leg. Half-timestep replay gives 13.029258 MPa. Keep the load and
100-million-reversal life; investigate a physical joint improvement rather
than reducing the applied range.

## Baseline under the admitted subset

This is **nominal-only coverage**, not a claim that the whole corrected
operator envelope passes. All nine poses were screened at h=100; the refined
h=50 subset contains five short-bed and four long-bed poses.

| Bed | Nominal contour h100 / h50 mm | Governing nominal pose | Handover h100 / h50 mm | Final residual max mm | Decision |
|---|---:|---|---:|---:|---|
| 2600 | 0.064081 / 0.063071 | X1600/Y1300 | 0.047330 / 0.046303 | 0.015740 | Contour exceeds 0.05; policy/drive correction required |
| 3150 | 0.044131 / 0.041219 | X800/Y0 | 0.026737 / 0.027831 | 0.022834 | Budget passes; 0.04 reserve missed |

The long-bed refined nominal dynamic weld screen at X800/Y0 is 10.816466 MPa
(W052); it is not a complete all-pose J5 fatigue qualification. The original
W066 result remains a useful counterexample for the detail screen.

A uniform slowdown does not guarantee a smaller resonant displacement at
every point: excitation frequency and phase also change. Actual histories,
including clock-filter transitions and shaping memory, must be replayed.

## Next joint study

Add both rack seats and their bolt/seat compliance, correct the right Y rack
below its pinion, retain braces, and rerun before selecting a fix. Do not edit
the frozen CAD until the user approves the proposal. Compare short-bed ZV/ZVD,
pose-scheduled V/A/J, and Y-drive stiffness changes with **whole-contour cycle
time**, including shaper/filter tails. Missing J5 coverage remains a gate.

For W066, screen a pair of 6 mm triangular gussets with approximately 100 mm
legs and wrapped load-transfer welds (about 0.47 kg of added plate per pair),
while retaining conservative FAT36 until a precise improved detail is justified.
Check gusset toes, post wall and all added welds. A full-penetration alternative
requires access/backing, weld procedure and inspection before claiming a higher
fatigue category. Neither detail is released by this audit.

## Evidence

[envelope-audit.json](brace-results/envelope-audit.json) records input hashes,
actual control parameters, sampled bounds, continuous-certificate validation,
all governing metrics and the excluded scenarios. Reproduce with
`envelope_audit.py --j4 J4 --replays REPLAYS --source J4/source --output OUT`.
Inputs/replays are retained in the compact brace archive cited in
[BRACE_STUDY.md](BRACE_STUDY.md). Maintain at least 8 GB free on root.
