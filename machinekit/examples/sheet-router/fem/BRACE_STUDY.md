> Motion control labels and acceptance scope corrected on 2026-10-22: see
> [MOTION_ENVELOPE.md](MOTION_ENVELOPE.md). Controlled J4 rows below are diagnostics.

# Brace-reduction study

Result: retain all sixteen braces on both beds. No tested reduction qualifies
under the fixed J4 policies and 100-million-reversal screen. Frozen CAD is
unchanged. Rack-seat/bolt compliance remains the next joint rerun; any later
geometry adoption requires the user's explicit approval.

The screen compares both 2600 and 3150 mm beds, anchored to the floor, retaining
the populated ATC mass. All eight feet use assumed 50,000 N/mm springs in
XYZ, with no rotational clamp. This is not a free-standing-foot result. `baseline` keeps sixteen diagonals. `no-plan` deletes the
four lower-plane diagonals. `single-a` and `single-b` retain opposite slopes of
every side/end X pair, keeping lower-plane bracing. `no-plan-single-a/b` combine
those changes. `no-plan-front-open` removes lower-plane bracing and the front end
X while retaining the side and rear end X pairs. The exploratory
`single-balanced` and `no-plan-single-balanced` layouts point both side bays
toward their upper middle post and oppose the front/rear end slopes, to screen
a more balanced load path after the uniform slopes failed. Both diagonal slopes undergo
signed cutting and acceleration envelopes; neither slope is credited as a
one-way, tension-only load path.

This is a finite layout comparison, not an optimization over all diagonal
patterns. Removing lower-plane braces improves vacuum plumbing and floor-level
service routes. Single vertical diagonals improve reach but leave triangular
obstructions. The front-open variant removes front service diagonals; it does
not increase the previously checked sheet-loading mouth or the 149 mm clearance
below the gantry. The added middle legs, chains and upper supports remain.

## Reproduction and acceptance

`brace_study.py` transforms the qualified prepared-member packets and their
weld stitches. It removes seams on deleted stock and closes only cope gaps at a
removed crossing partner. Copes and connections at retained posts remain.
`brace_macro.py` evaluates the frozen merged-joint model used for J4 motion
policy selection. The prepared-contact model separately evaluates every
remaining weld group's reactions. The two idealizations are reported separately.

The complete screen uses all nine X=0/800/1600 by Y=0/bed/2/bed poses,
including the asymmetric corner poses introduced by single diagonals. All poses retain full Z extension. Forty modes are retained
for motion reconstruction; reports include the lowest six shapes and strain
energy fractions. The prepared-contact model extracts forty modes at the three
X=800 poses for the original seven layouts; all nine poses receive the static
reversal envelope. The balanced exploratory layouts stop after static-fatigue
and motion rejection, with no dynamic weld pass credited. Static reactions include gravity, signed 300 N XYZ cutting and
signed 3 m/s² XYZ acceleration. Weld fatigue uses the existing SR6 detail
categories and 100-million-cycle cut-off, including both load reversals.

The J4 exports and source snapshots are pinned, with producer manifests and
configuration hashes retained. They include the entire rapid entry and moving
rapid-to-cut handover, nominal contours, override and hold/resume requests at t=1.5/3 s. Modal
history is never reset at a segment or handover. Policies remain 2600 ZV and
3150 unshaped with their selected axis jerks. The robustness screen varies
frequency ±15% and damping from 0.5% to 3%; a coarse frequency screen cannot
qualify a finalist. Acceptance requires the 0.04 mm reserve as well as the
0.05 mm SR-D3 ceiling. Rack-seat mass/compliance is a further joint rerun, not
implicitly included in these brace-only results.

`brace_motion.py` reconstructs contour, handover and stop errors. Its optional
weld replay uses modal reaction histories, a static residual correction, and a
conservative decaying-tail bound. It evaluates simultaneous throat-stress
vectors, enumerating all sixteen signs of the independent 300 N XYZ cutting
and 3 m/s² Z allowance. Full reversal is twice the largest vector magnitude.
Gravity remains non-reversing and carries its 1.35 design factor; variable
loads carry 1.5 for the static check. The future bound starts after the
explicitly simulated 0.5 s settling tail. The older component-interval upper
bound is retained for comparison, but peaks from different times are not
combined to claim an actual trajectory stress. `brace_checks.py` checks this
envelope against known loads and independent exhaustive enumeration. Signed cutting and Z acceleration allowances
remain included alongside the XY planner history. `brace_report.py` emits all
per-group stresses and lowest-six energy partitions, including failures.

Every structural solve must have positive diagonal compliance, reciprocal
cross compliance, force balance, cut work/strain energy agreement, and modal
mass/strain energy agreement. CalculiX runs serially. A four-thread solve of an
unchanged deck produced negative/non-reciprocal compliance; its results were
discarded and an exact-deck serial comparison is retained as a solver diagnosis.
A rejected numerical discretization is recorded explicitly and may be retried
at an independent mesh; a physical stiffness or fatigue failure is never
relabelled a numerical failure.

## Results and recommendation

All eighteen bed/layout combinations pass the static 2 N/µm target at nine
full-Z poses. None passes every motion/fatigue criterion. The following table
uses the largest observed motion errors and smallest stiffness/frequency across
the retained screens. Every layout has h=100 mm macro results; baseline and
lower-plane removal additionally have h=50 mm results at the original and newly
identified governing poses. The other reduced layouts already have clear
counterexamples, so this is a rejection screen rather than a converged release
qualification for those layouts. Frequency is reported, never used as a gate.

Motion columns include the full rapid-entry history, nominal, override requests at t=1.5/3 s
and hold/resume. The primary grid has three frequency scales × seven damping
values; lower-plane removal on the long bed also has a 129 × seven sweep at
three modal poses. The 0.04 mm reserve and 0.05 mm SR-D3 ceiling are distinct.
The stress column is the complete nine-pose quasi-static full-reversal envelope;
dynamic results follow separately. Both must qualify before adoption.

### 2600 mm bed

| Layout | Min X / Y N/µm | Lowest Hz | Contour mm | Handover / stop mm | QS range MPa | Fixed kg / weld m saved |
|---|---:|---:|---:|---:|---:|---:|
| baseline | 2.175 / 2.051 | 26.82 | 0.06815 | 0.04733 / 0.02764 | 11.82 | 0.0 / 0.00 |
| no-plan | 2.170 / 2.050 | 25.41 | 0.06615 | 0.03707 / 0.03008 | 11.75 | 113.0 / 3.55 |
| single-a | 2.134 / 2.068 | 24.33 | 0.07200 | 0.04841 / 0.02494 | 21.41 | 87.9 / 12.86 |
| single-b | 2.157 / 2.070 | 24.05 | 0.06967 | 0.05035 / 0.02510 | 15.35 | 94.5 / 11.90 |
| single-balanced | 2.137 / 2.073 | 24.09 | 0.07141 | 0.04965 / 0.02379 | 21.45 | 90.2 / 12.06 |
| no-plan-single-a | 2.119 / 2.068 | 22.25 | 0.06503 | 0.04746 / 0.02848 | 21.82 | 200.9 / 15.93 |
| no-plan-single-b | 2.148 / 2.070 | 21.92 | 0.07262 | 0.04596 / 0.02498 | 15.60 | 207.5 / 14.97 |
| no-plan-single-balanced | 2.122 / 2.073 | 21.85 | 0.07548 | 0.04571 / 0.02973 | 21.86 | 203.2 / 15.13 |
| no-plan-front-open | 2.145 / 2.074 | 21.93 | 0.06618 | 0.04597 / 0.03119 | 14.25 | 154.3 / 5.57 |

### 3150 mm bed

| Layout | Min X / Y N/µm | Lowest Hz | Contour mm | Handover / stop mm | QS range MPa | Fixed kg / weld m saved |
|---|---:|---:|---:|---:|---:|---:|
| baseline | 2.109 / 2.044 | 24.09 | 0.04413 | 0.02783 / 0.02283 | 12.43 | 0.0 / 0.00 |
| no-plan | 2.102 / 2.044 | 22.65 | 0.04408 | 0.03447 / 0.02153 | 12.48 | 119.8 / 3.53 |
| single-a | 2.075 / 2.059 | 21.72 | 0.06231 | 0.01944 / 0.03447 | 23.39 | 95.0 / 11.11 |
| single-b | 2.099 / 2.064 | 21.65 | 0.05673 | 0.02402 / 0.03229 | 25.29 | 102.8 / 11.43 |
| single-balanced | 2.078 / 2.066 | 21.58 | 0.06006 | 0.02785 / 0.03209 | 25.32 | 98.3 / 11.59 |
| no-plan-single-a | 2.058 / 2.059 | 19.74 | 0.07166 | 0.02828 / 0.02528 | 23.83 | 214.7 / 14.16 |
| no-plan-single-b | 2.087 / 2.064 | 19.62 | 0.05666 | 0.02352 / 0.02782 | 25.42 | 222.6 / 14.48 |
| no-plan-single-balanced | 2.061 / 2.066 | 19.46 | 0.05877 | 0.02282 / 0.02805 | 25.46 | 218.1 / 14.64 |
| no-plan-front-open | 2.078 / 2.067 | 19.59 | 0.06612 | 0.02294 / 0.02017 | 14.59 | 161.1 / 5.56 |

The first mode remains frame-X sway, with the gantry following the frame.
The side/end and lower-plane energy fractions, body translations/rotations and
lowest six frequencies at the minimum-global-mode pose are in
[governing-lowest-six.csv](brace-results/governing-lowest-six.csv). The archive
contains the same diagnostics at every solved pose, including the additional
finer-mesh cases. The next low gantry-dominant family moves in Y with beam roll;
Y-drive strain energy remains about 30–33% near the central pose. Removing
braces does not resolve that drive compliance. All brace mass savings are fixed;
moving-Y mass savings are zero.

### Dynamic fatigue and numerical evidence

The long-bed baseline at X800/Y0 gives **13.0276 MPa** at h=50 mm versus the
unchanged **12.6693 MPa** FAT36 cut-off (W066, `lower1-p0` / `post13-p0`, the
right lower stringer/middle-leg joint). Its 20-mode value is 13.0600 MPa;
halving the trajectory step from 0.25 to 0.125 ms gives 13.0293 MPa. Lower-plane
removal gives **16.3701 MPa** at h=50 mm (W064, `lower1-p0` / `post11-p0`),
16.2912 MPa with 20 modes and 16.3724 MPa at half step. The corresponding
h=100 mm observed ranges are 13.5319 / 17.8940 MPa. These are sampled
counterexamples, not claims of a continuous worst-case uncertainty certificate.
The controlling ranges arise during motion, rather than the unsampled tail.

The expanded J4 envelope therefore needs baseline weld requalification as well
as motion-policy qualification. The historical SR6 pass used its recorded
histories; it does not cover every new override/hold/pose. No life, category,
cutting allowance or load factor was reduced. Remaining reduced layouts fail
the static fatigue envelope or motion reserve already; they are not credited
with a dynamic fatigue pass. Dynamic fatigue is explicitly evaluated for the
long-bed baseline and lower-plane-only candidate, where it affects the decision.

Every retained piece-pair check is in
[weld-group-envelopes.csv](brace-results/weld-group-envelopes.csv), with leg,
throat, detail/category, controlling pose, mesh, stress and pass/fail. Several
prepared contacts share one fabrication group: the baseline has **132 group
IDs**, representing **160 / 162 piece-pair checks** on the short/long beds.
The full per-history dynamic contact rows are retained in the archive.

The h=100 → 50 mm baseline first-six frequency change is at most 0.72% at the
original modal poses. Timed errors are more phase-sensitive: finer-mesh observed
contour peaks are **0.06320 mm** short (X0/Y1300, 75% feed request at t=3 s (old offline suffix)) and **0.04397 mm**
long (X0/Y0, hold/resume at t=3 s (old offline stop)). The former exceeds 0.05 mm; both exceed the 0.04 mm
reserve. Lower-plane removal still gives **0.06293 / 0.04315 mm** at the refined
screened poses. This is why the old focused governing-pose policy pass cannot
be extrapolated to the whole bed.

The unchanged pre-ATC control reproduces all forty frozen short-bed frequencies
exactly and modal residues to 1.2e-16. Thirty pinned J4 source/replay corner
comparisons agree to less than 1e-11 mm. Seven independent vector-envelope
checks pass, as do all eighteen topology/reference/mass transformations.
A serial SPOOLES failure at long-bed baseline X800/Y1575/h100 is explicitly
rejected and replaced by a valid h75 solve; it is not a physical layout failure.
The four-thread compliance artefact is retained only as solver-diagnosis evidence.

### Access and next step

| Layout change | Access benefit | Remaining limit |
|---|---|---|
| Remove lower-plane X | Clears the Z≈180 mm floor-level crossings for vacuum hoses, drains and service | Side/end X remain; it fails the fixed-policy reserve/fatigue screen |
| One vertical diagonal per bay | Opens half of each side/end X for valve/plumbing reach | A triangular obstruction remains; retained joint fatigue rises materially |
| Combined | Largest floor/side service opening and fixed-mass saving | Lowest frame modes and failed motion/fatigue screens |
| Front-open plus no plan | Removes the front service diagonals | Side/rear X remain; loading mouth and gantry clearance do not increase |

The existing front sheet-loading mouth and 149 mm gantry clearance are already
checked and remain unchanged. Outboard Y-chain and beam-mounted X-chain routes
do not move in these variants. Existing service-route witnesses remain the
baseline; an unadopted layout cannot claim a new clearance certificate.

**Recommendation:** keep the frozen braces. The only modest removal worth
revisiting is the lower-plane group, after the physical rack seats/bolts are
included and full-pose timing policies and weld fatigue pass with margin.
Do not adopt single-diagonal or front-open geometry from this screen. The next
joint rack-support study must report the seat mass/compliance and the effect
on the low Y-drive/beam mode before proposing any frozen-CAD change.

## Motion-envelope correction

The request suffixes `1p5` and `3` mean **seconds**, not feed multipliers.
All four controlled J4 scenarios request 75% feed. They compile offline
stop/retime/resume suffixes and do not establish the approved J5 ready-stop
and executor-clock behaviour. These controlled histories are diagnostic only;
see [MOTION_ENVELOPE.md](MOTION_ENVELOPE.md) for the corrected coverage.
The user accepted retaining all braces.

## Evidence and reproduction

The retained packet is `build/sheet-router-evidence/brace-reduction-compact.tar.gz`,
with its SHA256 in the adjacent `.sha256` file and
[provenance.json](brace-results/provenance.json). It contains pinned SR7 mass
packets, qualified SR6 native/preparation/stitch data, J4 CSV/manifests/source,
serial solver decks/logs, selected governing compressed data, reactions, per-mode energy/body
reports, signed per-contact stress rows, numerical rejections, convergence
checks and test results. The compact archive retains all decks, logs, summaries, reactions and mode reports;
337 reproducible bulk raw DAT files were removed after verifying all 1,872 retained
regular files. Refined baseline and refined weld raw DAT are retained. The
original hash inventory and retention manifest identify every omitted file.
Loose working packets are removed after archive verification. No tracked CAD or assembly geometry is changed by this commit.

The drivers consume extracted **regular files** from that archive:

```sh
python3 brace_macro.py --inputs INPUTS --output MACRO --ccx CCX --mesh 100
python3 brace_study.py --inputs INPUTS --output WELDS --ccx CCX --mesh 100
python3 brace_motion.py --structural MACRO --j4 J4 --output MOTION --all-poses
python3 brace_motion.py --structural WELDS --j4 J4 --output WELD_MOTION --fatigue
python3 brace_checks.py
```

Use the repository heavy-run gate and serial solver, keep output under root
`build/`, and stop below 8 GB free. Exact commands/logs and mesh/pose selections
are retained. Do not write through symlinked extraction or snapshot paths.
