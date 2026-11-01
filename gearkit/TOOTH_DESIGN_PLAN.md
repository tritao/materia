# Strain-wave tooth design plan

**Goal.** Design a matched strain-wave tooth set in `gearkit`: flexspline tooth, circular-spline
tooth and wave-generator cam. Check how they mesh, and hand the result to the machines that cut
them.

1. Choose the gear: tooth counts, module, the deformation `w₀` and the cam shape. Choose a
   flexspline tooth form: involute, or double-circular-arc (Liu 2025).
2. The flexspline's deformation comes from the cam (inextensible ring theory). It gives every
   flexspline tooth's path relative to the circular spline.
3. The circular-spline tooth is the **envelope** of the flexspline tooth swept along that path,
   which makes it conjugate by construction.
4. A meshing analysis reports engaged teeth, backlash at each position, interference, and contact
   zones over a full wave-generator turn.
5. Design rules check manufacturability:
   - the flexspline tooth can be hobbed (a generating rack exists, no undercut);
   - the circular spline can be skived or shaped;
   - fillets, clearances and stress-relevant radii are within limits.
6. The output, `StrainWaveDesign`, feeds three consumers:
   - the hobber (the flexspline target and its hob);
   - the future circular-spline skiving machine;
   - CadKit parts for the cobot joints.

**Why now:** the hobber (`machinekit/examples/flexspline-hobber/PLAN.md`) proves we can cut a
tooth to class 5–6. This plan decides **which tooth is worth cutting**. Background and sources:
`machinekit/STRAIN_WAVE_RESEARCH.md` (deformation equations, Liu 2025, Ishikawa S-tooth,
full-conjugate and cycloid profiles, `pancake_swgb_designer` envelope method).

The work spans kits:
- `gearkit`: tooth forms, deformation, envelope, meshing, rules, search, design output;
- `stockkit`: none (the ring stock is reused unchanged by the hobber and the future skiving
  machine);
- `machinekit`: CadKit parts for the flexspline cup, circular spline and cam; the hobber's Phase D
  hook;
- `app`: a meshing view.

## What exists

Surveyed 2026-10-17 on branch `flexspline-hobber` (6caeb51f3).

- `gearkit` (new, dependency-free; HB1, HB4):
  - `GearSpec`: an **involute-only** spur specification (external and internal, profile shift,
    cutter-tip and tooth-tip radii, evaluation range);
  - `ToothProfile`: analytic involute, trochoidal root fillet, tip rounding, outline;
  - `ConjugateRack.of` / `envelopeAt`: numerical rack recovery and regeneration from tooth
    coordinates. Internal rack-limit restrictions are documented in the hobber plan's HB1 notes;
  - `HobDesign`: built from the recovered rack;
  - metrology: `GearMeasurement`, `GearRings`, `GearInspection`, `Iso1328`, `ErrorBudget`,
    `GearTrace`.
- The hobber cuts and grades involute flexsplines. Its metrology reads tooth flanks through
  `ToothProfile`.
- No deformation kinematics, no non-involute forms, no flexspline/circular-spline pairing, and no
  meshing analysis exist anywhere.
- External, ideas only (no licence): `brandonkxo/HarmonicGearTeethGenerator` (Liu 2025
  equations), `erdemu/pancake_swgb_designer` (swept-union envelope, `w,v,μ` kinematics). We
  implement from the papers.

## What is missing

- A tooth-form abstraction (today involute is built into `GearSpec` and `ToothProfile`).
- Double-circular-arc and sampled (envelope-derived) forms.
- Wave-generator cam shapes and flexspline deformation kinematics.
- Relative tooth motion and the circular-spline envelope.
- Meshing analysis and strain-wave design rules.
- A parameter search, the design record, CadKit parts and a meshing view.

## Decisions

**TD-D1. Kinematic meshing uses inextensible ring theory on the neutral line.**
- The cam profile `ρ(φ)` sets radial displacement `w(φ)` of the flexspline neutral line.
  Inextensibility (`w + dv/dφ = 0`) gives the tangential displacement `v`. The section rotation
  is `μ = (v − dw/dφ)/r_n` (the convention `pancake_swgb_designer` and the research notes use).
- Cams:
  - **cosine** (`w = w₀ cos 2φ`, closed form);
  - **ellipse**, **four-roller** and **Resàl**, integrated numerically. Gravagno 2016 reports the
    lowest pure kinematic error for Resàl.
- Each flexspline tooth is rigid and anchored to its neutral-line section, as in HB-D9 and the
  envelope method.
- The deflection coefficient `κ = w₀ / (m·(Z_c − Z_f)/2)` is a design parameter.
- **Rejected for the design loop:** finite-element deformation (too slow to iterate). It becomes
  a later validation step (Later).

**TD-D2. Tooth forms are a general interface, and involute is one implementation.**
- `ToothForm` gives:
  - flank curve and derivative on a parameter;
  - root and tip blends;
  - outline;
  - validity checks.
- Implementations: `Involute` (the HB1 code, ported), `DoubleCircularArc` (TD-D6), and `Sampled`
  (a smooth curve through envelope points, such as the derived circular-spline tooth).
- `GearSpec` holds a `ToothForm` instead of assuming involute. `ConjugateRack`, `HobDesign`,
  `GearInspection` and `Iso1328` work from the form's flank geometry.
- One schema, no compatibility layer (repo rule). HB1–HB4 tests must still pass after the port.
- **Rejected:** a parallel non-involute code path. Two flank models would drift, and metrology
  would double.

**TD-D3. The circular spline is derived from the flexspline (flexspline-first).**
- In the circular-spline frame, each flexspline tooth follows the TD-D1 path over one
  wave-generator turn. The circular-spline tooth space is the envelope of the swept tooth.
- It is computed numerically:
  - by the swept union of tooth placements, for robustness;
  - refined by the contact condition `n·v_rel = 0` at the envelope points, for accuracy;
  - then fitted to a `Sampled` form with a stated deviation bound.
- **Circular-spline-first** (Camus or full-conjugate, Xie 2024) is the reverse operation on the
  same machinery. It is a later option, not the default.

**TD-D4. Meshing is analysed kinematically first, under no load.**
- For a designed pair, over one full wave-generator turn, per tooth position:
  - the minimum normal clearance (backlash);
  - penetration (interference);
  - contact status (clearance under a threshold);
  - and the contact point positions.
- Summary: flank-engaged and tip-edge-contact fractions (apart; tip contact is not engagement), backlash distribution, the contact zone in angle around the
  major axis, and interference count (must be zero).
- Loaded meshing is later. Torque deflects the flexspline and changes the engagement; that needs
  a compliance model.

**TD-D5. Design rules are explicit and name the parameter that breaks them.**
- Geometry:
  - no interference;
  - minimum backlash ≥ a stated value (manufacturing tolerance);
  - tip/root clearances;
  - tooth tip thickness ≥ a minimum;
  - root fillet radius ≥ a minimum (fatigue proxy).
- Manufacturability:
  - the flexspline tooth must have a valid generating rack for hobbing (reuse `ConjugateRack`
    checks; undercut fails);
  - the circular-spline tooth must be producible by internal generation. The rack-limit check
    exists for internal teeth now; a skiving-tool check comes with the skiving machine.
- Flexspline structure, as a first proxy:
  - bending strain at the major axis `ε ≈ 3·w₀·t/(r_n²)` (assumed form, check against a source)
    against a stated fatigue strain limit for the steel;
  - full stress and fatigue analysis is later.

**TD-D6. Double-circular-arc teeth follow Liu 2025's parametrisation, implemented from the
paper.**
- Flank = convex arc (radius `r₁`, centre offsets `c₁, e₁`), a common tangent line, and a concave
  arc (`r₂`, `c₂, e₂`). Addendum `ha`, dedendum `hf`. Wall coefficients: ring `s = μ_s·m·Z_f`,
  cup `t = μ_t·s`.
- Regression: Liu's published defaults (m 0.5, Z_f 100, Z_c 102, w₀ 0.5) produce a valid tooth.
  Our meshing analysis is compared with the paper's reported behaviour where the paper gives
  numbers.
- The open-access paper is the source. The unlicensed GitHub implementation is read for ideas
  only.

**TD-D7. Design search is small, deterministic and transparent.**
- Search a few parameters (`κ`, cam type, and arc radii and offsets for double-circular-arc)
  with a grid then Nelder–Mead. Seeds are fixed.
- Objectives: maximise flank-engaged fraction and backlash uniformity, and maximise root fillet
  radius, under the TD-D5 constraints.
- Every candidate's metrics are reported, so a person can see the trade-off rather than only the
  winner.
- **Rejected:** heavy optimiser dependencies. They are not needed at this size.

**TD-D8. `StrainWaveDesign` is the single output.**
- It records:
  - flexspline `GearSpec` (form, counts, module, shift);
  - circular-spline `GearSpec` (`Sampled` envelope form, fitted);
  - cam profile;
  - `w₀` and `κ`;
  - cup and ring wall dimensions;
  - the meshing report;
  - rule results;
  - provenance (inputs, tolerances).
- Its consumers:
  - the **hobber**, where `HobDesign` takes the flexspline form (Phase D target tooth);
  - the future **skiving machine** (the circular spline);
  - **CadKit parts** (flexspline cup with teeth, circular-spline ring, cam) for cobot joints;
  - the later **behaviour model**.

**TD-D9. Claims stay kinematic until loaded models exist.** Reports say "unloaded kinematic
meshing". Published figures (e.g. about 30 % engaged teeth for S-tooth and double-circular-arc
units) are compared only as plausibility, with the source and the load condition stated.

**TD-D10. Generic names in code;** vendors and patents appear only as sources in docs.

## Test designs

- **D1, the hobber reference:** involute 20°, m 0.3, Z_f 200, Z_c 202, cosine cam, κ = 1. The
  baseline, and the same flexspline the hobber already cuts.
- **D2, Liu 2025 defaults:** double-circular-arc, m 0.5, Z_f 100, Z_c 102, w₀ 0.5.
- **D3, involute 30°:** m 0.3, Z_f 200, κ swept from 0.8 to 1.2. The common lower-cost alternative.
- **D4, a deliberately bad design:** κ too large, so the derived circular spline interferes or the
  flexspline can't be hobbed. It must be rejected, naming the parameter.

## Reference numbers (assumed until checked against a source or measurement)

| Quantity | Value | Source |
|---|---|---|
| Deflection coefficient κ | 0.8–1.2, nominal 1 | research notes; literature range |
| Minimum backlash (unloaded) | ≈ 5 µm | assumed, the manufacturing tolerance class |
| Engaged-tooth plausibility | ≈ 30 % for S-tooth and double-circular-arc units | Harmonic Drive and literature figures, load condition unstated |
| Flexspline steel fatigue strain limit | to be sourced | 40CrNiMo, check |
| Liu 2025 defaults | m 0.5, Z_f 100, Z_c 102, w₀ 0.5, r₁ 0.685, c₁ 0.332, e₁ 0.155, r₂ 0.785, c₂ 0.330, e₂ 0.134 | Liu et al., *Machines* 13 (2025) 535 |

## Baseline (TD0)

Recorded 2026-11-01 on `main` 6e041094c, worktree `tooth-design`, toolchain from the main
checkout (`haxeon build --compiler-only`; the worktree's submodule directories are symlinks to the
main checkout's). Run with the main checkout's `app/build/host/native/*` on `LD_LIBRARY_PATH`.

| Gate | Baseline |
|---|---|
| gearkit suite (`gearkit/tests`) | green: GeometryTests 1034568 assertions (rack error 1.32e-14 mm, re-envelope 2.51e-14 mm), MetrologyTests 246, GearKitTests 18 |
| hob rack output (`supplier-packages`, 1543 rack points) | JSON md5 `017ec06361ce3f4663e9ee765544712f` |
| stockkit ring / metrology / hobbing-stream / hobbing-errors / hobbing-forces | green (345 / 11 assertions / HB3 difference 0 um / 24 / 27 assertions) |
| camkit `tests/hobbing` | fails at HobbingCamTests.hx:219 ("Motion program op 1: Null access", motionkit `ProgramCompiler`) on unchanged main too; pre-existing, environment (native motionkit library) |
| `app/tests` | does not compile on unchanged main with this toolchain (`State.updateQuietly` unknown): the app source is ahead of the committed haxeon submodule; pre-existing |
| machinekit `tests/hobbing`, `tests/hobber` | compile; full runs stop on the known "Collision hulls disagree with recipe" save/load bug (pre-existing, not chased) |

D1 to D4 are written as inputs in `gearkit/tests/src/ToothDesigns.hx` (`StrainWaveInput` in
`gearkit/haxe/gearkit`), validated by `ToothDesignInputTests`. **D4's kappa (2.0) is assumed**
until TD5 measures where the derived circular spline interferes.

## TD2 notes: double-circular-arc form

Code: `DoubleCircularArc` (the flat construction, validity, `liu(module)`), `DoubleArcProfile` (wrapping,
blends, `ToothProfile`), tests in `DoubleArcTests`. Source: Liu et al. 2025, *Machines* 13, 535, section 2.1,
rebuilt from the definitions (Figs 2-3), not from the printed equations.

Construction (flat frame, x from the tooth axis, y from the **pitch line**, so `ds` cancels):
O1 = (-c1, -e1), O2 = (pi m/2 + c2, e2); addendum line y = ha, dedendum line y = -hf. The common tangent is the
internal tangent: with outward normal n = (cos d, sin d) at B, (O2 - O1).n = r1 + r2, so d = angle(O1->O2) +
acos((r1+r2)/D). B = O1 + r1 n, C = O2 - r2 n. A = (-c1 + r1 cos a, ha) with sin a = (ha+e1)/r1; D =
(x2 - r2 cos b, -hf) with sin b = (e2+hf)/r2. The flank is monotone in y, x grows towards the root.
Checked in `DoubleArcTests`: B, C on their circles, tangent perpendicular to O1B and O2C, slope continuity at B
and C (1e-9), C0 at B and C, A and D on their lines, pitch-line crossing on BC.

Deviations from the printed equations:
- Eq 4: `x1` in y_BC should be `y1`, and the (l - l1) term is subtracted in y: y_BC = r1 sin d + y1 - (l-l1) cos d
  (the tangent runs downward). x_BC as printed is right.
- Eq 6: y_CD printed with `r1 sin d` and a `+ r2(..)`; correct is x = x2 - r2 cos(d + (l-l2)/r2),
  y = y2 - r2 sin(d + (l-l2)/r2).
- Eq 11 (d = eps + sigma) is **correct** when sigma is the angle of O1->O2 (positive when O2 is above O1) and
  eps = acos((r1+r2)/D). The other root (sigma - eps) is the crossing tangent with B below C (h1 < 0) and is
  rejected. The printed sigma = arctan((y1-y2)/(x1-x2)) has the same ratio as (y2-y1)/(x2-x1), so it agrees.
- Eqs 2, 3, 5, 7, 12, 13: no change (l1 = r1 (alpha - d), h1 = yB - yC).
- **Table 2 swaps c2 and e2** relative to Table 1: it lists c2 = 0.1344, e2 = 0.3299, but Table 1 gives c2 in
  [0.329, 0.331] and e2 in [0.130, 0.140]. Used: c2 = 0.32994565, e2 = 0.13439061 (unswapped). Parameter set:
  r1 0.68517449, c1 0.33183981, e1 0.15535155, r2 0.78482551, c2 0.32994565, e2 0.13439061, ha 0.29316387,
  hf 0.35425825 (m 0.5). With these: d = 16.448 deg, alpha 40.89, beta 38.51, h1 0.1265 mm, A half-thickness
  0.1861, D at x 0.5012, tooth thickness on the pitch line 0.6734 of pi m = 1.5708 (the tooth is thin, the space wide).
- The paper gives no root or tip blend. **Assumed:** circular blends, internally tangent to the arc and the
  dedendum/addendum line. Root radius = `GearSpec.cutterTipRadius` (default 0.38 m), tip radius =
  `tipFilletRadius` (0.05 m). This is not a generated trochoid.
- **Assumed wrapping:** the flat profile is wrapped on the **pitch circle** (radius Rp + y, angle x/Rp) so the
  angular pitch is exactly 2 pi/Z. The paper puts XR tangent to the neutral layer; wrapping there makes the
  pitch inconsistent with Z (4 % at Z 100). The effect on the shape is about 1 % of tooth height.
- Pressure angle of the form = d (16.45 deg), the flank angle where the pitch line crosses BC.
  Profile shift and internal gears are rejected. Parameters belong to one module; `liu(m)` scales every length by
  m / 0.5 (the shape in module units is the paper's), and a gear of another module is rejected.
- Validity (each throws naming the parameter): ha/hf too large for the arcs, overlapping arcs (no tangent),
  tangent not running downward, pointed tip (c1, r1, e1, ha), D past mid-space (c2, r2, e2, hf), blend radii
  that do not fit (cutterTipRadius, tipFilletRadius).
- Scaled to m 0.3, Z 200 the set is geometrically valid (the shape is scale invariant). Largest circular root
  fillet that fits: 0.743 m.
- Circular-spline nominal tip and root (assumed) are now pitch -/+ the flexspline form's own addendum and
  dedendum: 0.586 m / 0.708 m for D2, unchanged (1.0 / 1.25 m) for involutes.
- `ToothOutline.close` was factored out of `InvoluteProfile.outline` (identical output; GeometryTests unchanged).

## TD3 notes

- Cams (`gearkit/haxe/gearkit/*Cam.hx`, `WaveDeformation`, `StrainWaveKinematics`): `w0` is the
  major-axis radial displacement. Ellipse and four-roller are physical polar profiles with semi-axes
  `rn +/- w0`, minus their mean radius so the ring keeps its length (needed for `v` to close).
- **Assumed:** the four-roller is a four-arc oval (lobe arc radius = half the minor semi-axis,
  larger arc on the minor axis, tangent at the join); no roller geometry was sourced. The neutral
  radius in tests (29.325 mm, the one `StrainWavePair` builds for D1) is the root circle (29.625 mm) less half an assumed
  0.6 mm wall (0.01 m Z_f). An earlier version used 29.7 mm, above D1's root; it is corrected.
- "Neutral-line length preserved to 1e-9" is tested as the linear inextensibility condition
  `w + dv/dphi = 0` (measured 7e-13) and closure of `v` (1.4e-14). The exact arc length of the
  deformed line differs from `2 pi rn` at second order in `w0/rn` (about 1e-4 for D1); tested
  against a `3 (w0/rn)^2` bound, not 1e-9.
- Cosine numerics against the closed form: 1.4e-16.

### Resal cam and the cam comparison

- `ResalCam`: Gravagno, Mucino, Pennestri 2016, eq 18 (the extracted text garbles the fraction; the form used is
  `u = delta (1/pi - theta sin theta/4 - cos theta/4) / (1/pi - pi/8)`, theta from the minor axis, rollers at +-pi/2,
  checked by its zero mean and `u(pi/2) = delta`). Cam convention: delta = w0, theta = (phi mod pi) - pi/2. Tests:
  `w(0) = w0`, minor axis -0.918 w0, zero slope at both axes, mean zero (1e-12), even, period pi, slope matches
  the derivative, and the general inextensibility/closure checks all pass.
- TD5 cam comparison (`ToothStudy.camStudy`, kappa 1, each cam with its own derived circular spline). **Transmission
  error is zero by construction and is not measured**: every cam gets a circular spline derived from its own motion, so
  no cam can show a ratio error against a fixed one. Only clearances and how closely the other teeth follow that
  envelope differ. Flank = teeth whose closest point to the circular spline (within 5 / 20 um) is on the working
  flank; tip = tip edge contact (closest point on the tip blend or land), reported apart, not as engagement:

| Cam | D1 minor-tip clr | D1 flank 5 / 20 um | D1 tip edge 5 / 20 um | D2s minor-tip clr | D2s flank 5 / 20 um | D2s tip edge 5 / 20 um | max abs w difference from cosine |
|---|---|---|---|---|---|---|---|
| cosine | 0 | 1.6 % / 19.9 % | 46.5 % / 65.9 % | 248 um | 16.3 % / 40.7 % | 1.2 % / 1.2 % | 0 |
| ellipse | -4.6 um | 1.6 % / 19.8 % | 46.5 % / 65.7 % | 244 um | 16.1 % / 40.2 % | 1.2 % / 1.2 % | 4.6 um |
| Resal | -24.5 um | 1.5 % / 21.7 % | 46.5 % / 63.5 % | 224 um | 14.6 % / 38.7 % | 1.3 % / 1.3 % | 24.5 um |
| four-roller | -127 um | 2.8 % / 19.3 % | 39.9 % / 40.0 % | 121 um | 2.8 % / 40.3 % | 2.3 % / 3.4 % | 127 um |

  What these numbers can and cannot say:
  - **Cam differences of a few um are inside model error.** `ShapedCam` maps the cam to the neutral line by taking the
    radius at the deformed angle as `w` at the material angle. That drops a second-order term of order
    `w0^2 / r_n` (0.09 / 29.3, about 3 um for D1). So the ellipse "interference" of -4.6 um at the D1 minor axis is not
    a result; neither is the 0 to 3 point spread among cosine, ellipse and Resal engagement. The Resal minor axis
    (-24.5 um) is outside that error but is arithmetic on the assumed circular-spline tip circle (below), not a mesh
    result. Nothing here ranks cosine, ellipse and Resal.
  - **The four-roller geometry is unsourced.** It is an assumed four-arc oval (see TD3 notes); no roller layout was taken
    from a source. Its -127 um and its different engagement describe that assumed shape only.
  - Gravagno's kinematic error (transmission-ratio error) is not modelled; it needs a fixed circular spline meshing with
    the cam. Dwell "caterpillar" cams (cycloidal, 3-4-5, 8th-degree polynomial) are a possible later TD, not implemented.

## TD4 notes

- `CircularSplineEnvelope` sweeps one tooth boundary (exact chord-circle crossings, 400 radii,
  2400 time steps, then twice that) and keeps the largest angular extent per radius within half a
  circular pitch of the space axis. `SweptSpace` is the result, a sampled tooth space with a
  deviation bound (4 x the coarse-to-fine difference; the factor is a measured allowance, since
  re-sweep penetration reached 3.6 x the raw difference).
- **Deviation from the plan:** the contact-condition refinement (`n.v_rel = 0`) is not done; the
  swept union is refined by resolution instead. `SweptSpace` is a sampled space profile, not yet a
  `ToothForm` with root and tip blends (that fit belongs to TD8).
- Measured: rigid-limit envelope (rolling pinion, no deformation) matches the standard internal
  involute space to 1.1e-6 mm; D1 re-sweep penetration 0.20 um against a 0.69 um bound; D3 (30 deg,
  kappa 1) 0.35 um against 1.21 um.
- Circular-spline nominal radii are assumed full depth: tip `r_pc - m`, root `r_pc + 1.25 m`.

## TD5 notes and the D1 / D3 comparison

`MeshingAnalysis` scans one circular pitch of wave-generator angle (24 or 48 samples; the ring state
repeats every pitch) with every flexspline tooth against the derived circular spline
(`SweptSpace`). Run with `ToothStudy` (see "Running the tooth study" below). **All numbers below are model output:
unloaded, kinematic, rigid teeth, zero-backlash conjugate circular spline.** Assumptions: full-depth
teeth (addendum 1.0 m, dedendum 1.25 m) on both gears and a nominal full-depth circular spline
(tip `r_pc - m`, root `r_pc + 1.25 m`); neutral line half a wall (0.01 m Z_f) under the root;
contact threshold 5 um (and 20 um); thinnest acceptable circular-spline tooth 0.05 m.

**Which numbers are meshing results, and which are not.**
- The root clearance `(1.25 - kappa) m` (`rRoot - reach`) and the minor-axis tip clearance `(kappa - 1) m`
  (`rTip - minorTip`) are **arithmetic on the assumed nominal circular-spline radii** (`MeshingAnalysis`, using
  `StrainWavePair.circularTipRadius` / `circularRootRadius`). They restate the tooth depth against the assumed
  circle; they are not meshing results. The derived circular spline is built from the flexspline's own motion, so it
  cannot disagree with that motion.
- **Transmission error is zero by construction and is not measured**: the circular spline is derived from each
  flexspline-and-cam pair. A fixed circular spline meshing with a cam is needed to measure it (the planned fixed-circular-spline redesign, not done).
- The engagement metric (changed 2026-11-01). The first version counted a tooth as engaged when any outline point was
  within the threshold, which includes the tip rounding and the tip land. That reported 48 % for D1 at kappa 1, but
  it was almost entirely tip edge contact. Each sample point is now classified by region (`ToothProfile.regionAt`:
  flank strictly between the root join and the tip-blend start; tip = tip blend and land; root blend), and a tooth
  within the threshold is attributed to the region of its closest point. **Flank engaged** and **tip edge contact**
  are reported apart; tip contact is edge contact, not engagement. The attribution matters: counting every tooth with
  any flank point within 5 um (joins excluded) gives 32 % for D1 at kappa 1, because flank points a few samples below
  the edge lie within 5 um of the edge's own trace.

m 0.3, Z 200/202, cosine cam. "CS tip trim" is how far the circular-spline tooth tip must be cut
back from the nominal tip for the derived tooth to be 0.05 m thick. Minimum clearance on both flank and tip is
0 to -0.2 um (inside the 0.7 um fit bound) in every interference-free row, by construction.

| Case | kappa | Interference | Root clr | Tip-tip clr (minor axis) | CS tip trim | Flank engaged 5 um / 20 um | Tip edge contact 5 um / 20 um | Root fillet R min | Hobbable |
|---|---|---|---|---|---|---|---|---|---|
| D1 20 deg, hob corner 0.38 m | 0.8 | yes (tips meet) | 135 um | -60 um | 88 um | 0 % / 0 % | 84.3 % / 84.4 % | 0.116 mm | yes |
| D1 | 1.0 | no | 75 um | 0 | 28 um | 1.6 % / 19.9 % | 46.5 % / 65.9 % | 0.116 mm | yes |
| D1 | 1.1 | no | 45 um | 30 um | 0 | 10.8 % / 19.3 % | 22.6 % / 31 % | 0.116 mm | yes |
| D1 | 1.2 | no | 15 um | 60 um | 0 | 11.7 % / 22.8 % | 12.6 % / 12.7 % | 0.116 mm | yes |
| D3 30 deg, hob corner 0.1 m | 0.8 | yes | 135 um | -60 um | 68 um | 1.3 % / 21 % | 52.5 % / 71 % | 0.034 mm | yes |
| D3 | 0.9 | yes | 105 um | -30 um | 37 um | 0 % / 4.8 % | 35 % / 46.6 % | 0.034 mm | yes |
| D3 | 1.0 | no | 75 um | 0 | 8 um | 0 % / 2.2 % | 22.5 % / 33 % | 0.034 mm | yes |
| D3 | 1.1 | no | 45 um | 30 um | 0 | 9.1 % / 15.9 % | 1.1 % / 1.1 % | 0.034 mm | yes |
| D3 | 1.2 | no | 15 um | 60 um | 0 | 10.1 % / 16.4 % | 1.8 % / 1.8 % | 0.034 mm | yes |

- D3 with the reference hob's 0.38 m corner radius cannot be cut: "rack corner leaves no cutter tip
  land". At full depth the largest corner radius is 0.47 m at 20 deg but 0.11 m at 30 deg
  (0.033 mm at m 0.3), so the root fillet of a 30 deg flexspline is about 3.5 times smaller.
  At the same 0.1 m corner radius, 20 and 30 deg have the same root fillet.
- Both forms are interference-free for kappa 1.0 to 1.2 and interfere below 1 (tooth tip meets
  circular-spline tooth tip at the minor axis) because the tooth depth is 1.0 m; kappa above 1.25
  bottoms in the root (D4, kappa 2: tips 225 um past the root, tooth 0, wave generator 0.0039 rad).
  **This window follows from the assumed nominal circular-spline radii** (see above): it restates tooth depth
  against the assumed circle, so it is arithmetic, not a meshing finding.
- **Flank engagement is small and rises with kappa for the involute forms.** At kappa 1 the D1 flank is engaged for
  1.6 % of teeth (5 um) and the D3 flank for 0 %, while 46.5 % (D1) and 22.5 % (D3) of teeth touch only with the tip
  edge. At kappa 1.1 and 1.2 flank engagement is 9 to 12 % for both and tip edge contact falls. The earlier claim that
  engagement falls as kappa rises, and that 20 deg engages far more than 30 deg, came from counting tip edge contact;
  with the flank counted alone it is not supported. At 20 um the D1 flank figure is 20 % at kappa 1 and 23 % at 1.2
  and D3 2 % at kappa 1, so the pressure-angle difference at kappa 1 is small in absolute terms and sensitive to the
  threshold. Do not compare any of these percentages with published (loaded) figures: the derived circular spline
  makes every tooth pass through its envelope, so the flank fraction measures how closely the other teeth follow
  the envelope (an upper bound on real unloaded engagement), not load sharing.
- **No pressure-angle recommendation follows from engagement.** D1 is the pick for other reasons: the reference hob is
  a catalogue hob, the involute has direct metrology (the base circle and involute traces; the hobber's Fr and flank
  grading exist for it), and it has the larger root-fillet room (0.472 m against 0.11 m at 30 deg). It is not the pick
  because it "engages more".
- Not done: a fixed standard circular-spline tooth instead of the derived one,
  loaded meshing, root stress.

### D2 (double-circular-arc) against D1 and D3

`D2s` = Liu's arcs scaled to m 0.3, Z 200/202; `D2` = the unscaled paper case (m 0.5, Z 100/102, w0 0.5). Both are
valid. Same assumptions, thresholds and metric as above, root fillet 0.38 m for D2 (circular, assumed).

| Case | kappa | Interference | Root clr | Tip-tip clr (minor axis) | Flank engaged 5 um / 20 um | Tip edge contact 5 um / 20 um | Min clr flank / tip | Flank zone | Root fillet R |
|---|---|---|---|---|---|---|---|---|---|
| D2s | 0.8 | no | 97 um | 188 um | 48.8 % / 48.8 % | 7.8 % / 7.9 % | 0 / -0.1 um | +-50 deg | 0.113 mm |
| D2s | 1.0 | no | 37 um | 248 um | 16.3 % / 40.7 % | 1.2 % / 1.2 % | 0 / -0.1 um | +-16 deg | 0.113 mm |
| D2s | 1.1 | no | 7 um | 278 um | 10.9 % / 22.7 % | 1.0 % / 1.0 % | 0 / -0.1 um | +-11 deg | 0.113 mm |
| D2s | 1.2 | yes (root bottoming) | -23 um | 308 um | 2.7 % / 10.3 % | 11.8 % / 11.8 % | -12.9 / -23.3 um | +-13 deg | 0.113 mm |
| D2 (paper) | 0.8 | no | 161 um | 314 um | 47.8 % / 47.8 % | 8.4 % / 8.8 % | 0 / -0.1 um | +-50 deg | 0.186 mm |
| D2 (paper) | 1.0 | no | 61 um | 414 um | 14.2 % / 34.7 % | 1.8 % / 1.8 % | 0 / -0.1 um | +-14 deg | 0.186 mm |
| D2 (paper) | 1.2 | yes | -39 um | 514 um | 0.5 % / 6.5 % | 11.9 % / 11.9 % | -21.5 / -38.9 um | +-11 deg | 0.186 mm |

- D2 is shorter (addendum 0.586 m, dedendum 0.708 m), so its root clearance is hf - ha = 0.122 m (37 um) at kappa 1
  and it bottoms above kappa about 1.12; it never has tip-tip interference (minor-axis tips clear by 190 to 310 um).
  Both clearances are arithmetic on the assumed nominal circular-spline radii (see TD5 notes), not meshing results:
  they are the form's own tooth depth restated.
- Flank engagement at kappa 1 (5 um / 20 um): D2s 16.3 % / 40.7 %, D2 (paper) 14.2 % / 34.7 %, against D1 1.6 % / 19.9 %
  and D3 0 % / 2.2 %. The D2 flank is engaged for roughly ten times as many teeth as the D1 flank at 5 um in this
  unloaded, zero-backlash model, and its tooth tip is almost never the contact (1 to 2 %); D1 and D3 touch mostly
  with the tip edge. This is a statement about how closely the other teeth follow the derived envelope in a
  kinematic model, not a load-sharing result, and not the published ~30 % (loaded). The earlier "D2 engages fewer
  teeth than D1" came from tip edge contact and is withdrawn.
- Backlash/clearance along the mesh: with the derived (conjugate) circular spline the clearance is zero on the
  envelope by construction; the fractions above are how many teeth stay within 5 / 20 um of it.
- Fillet: D2's root fillet is a design choice here (circular, equal to the hob corner radius). The geometric
  maximum is 0.743 m (D1's trochoid reaches 0.472 m at 20 deg, D3 0.11 m at 30 deg), so D2 allows the largest fillet.
- D2's curved flank cannot be cut with the existing hob (next section), so the choice between D1 and D2 is a
  hob and metrology choice as much as a meshing one. Meshing evidence here is kinematic and unloaded only.

### Hob implications for D2 (rack recovered by `ConjugateRack` from the D2s tooth)

`ToothStudy.hobStudy` prints these; rack heights are in module units, flank angle from the tooth axis.

| Rack | Flank | Datum half-thickness | Tooth depth below datum | Tip corner radius (curvature) |
|---|---|---|---|---|
| D1 reference hob | straight 20 deg | 0.2356 mm | 1.25 m | 0.114 mm (0.38 m) |
| D3 | straight 30 deg | 0.2356 mm | 1.25 m | 0.030 mm |
| D2s | curved, 29.5 / 16.4 / 38.4 deg (low end / pitch / root end) | 0.202 mm | 0.709 m | 0.112 to 0.113 mm |

- **Verdict: D2 can be hobbed, but not with the existing hob.** `HobCheck.problem` returns none, the rack has no root fold
  and no flank reversal, and a `HobDesign` builds. The hob needs its own profile.
- Flank: the conjugate rack flank is gently curved. The best straight line is 23.5 deg and stays within +-8.8 um of it;
  a straight 20 deg flank stays within +-13.5 um, but only when placed at a datum half-thickness of 0.1927 mm. The
  reference hob (20 deg, 0.2356 mm) differs from the D2 rack flank by up to 56 um as it stands. A straight-flank
  hob ground to about 23.5 deg with the right thickness would approximate D2 to about 9 um (profile form error, to be set against
  the class 5 to 6 target); a form-ground hob follows it exactly.
- Depth and thickness: D2's tooth is 0.709 m deep against the reference hob's 1.25 m, and its pitch-line tooth is
  0.404 mm (vs 0.471 mm): the hob tooth must be 14 % fatter on the datum. Sinking the reference hob to fix the
  thickness (about 0.12 mm) would cut the root about 0.4 m too deep.
- Corner: the hob tip corner radius needed to cut a circular root fillet of radius rho is about rho (0.112 mm for
  0.38 m). So the reference hob's 0.38 m corner radius is the right order and would cut a root fillet near 0.38 m,
  but its exact trochoid shape (not circular) was not computed. Any corner up to 0.7 m is hobbable (no fold).
- Not checked: the real hob's relief and gash, and the trochoidal root actually generated by a given corner.


### Running the tooth study

The D1 / D2 / D3 comparison, the cam comparison and the D2 hob study are not part of the gearkit test run. They are
`ToothStudy` in its own project, `gearkit/studies/tooth-design/` (it depends on `gearkit` and has no other inputs; the
study-only helper `maxCutterTipRadius` lives there, not in `HobCheck`). From the repository root,
with the main checkout's native libraries on `LD_LIBRARY_PATH` as for the other suites:

```
haxeon run --project gearkit/studies/tooth-design/haxeon.json
```

It prints one line per case (about 40 s). The meshing assertions that guard the analysis stay in `gearkit/tests`
(`MeshingTests`, with `ToothDesigns` as their fixture).

## Steps

### Phase A: forms and kinematics

**TD0. Baseline and test designs.**
- Record the gearkit and hobber gates on the starting commit. Write D1–D4 as inputs.
- Tests: inputs validate; the existing gearkit suite passes unchanged.

**TD1. Tooth-form interface (TD-D2).**
- `ToothForm`; `Involute` from the HB1 code; `GearSpec` holds a form. Port `ConjugateRack`,
  `HobDesign`, `GearInspection` and `Iso1328` to the form.
- Tests:
  - every HB1–HB4 assertion passes after the port;
  - the hobber's quick checks pass;
  - a form-agnostic test asserts the rack, hob and metrology produce identical numbers for the
    involute through the interface.

**TD2. Double-circular-arc form (TD-D6).**
- Arcs, common tangent, root and tip blends, validity (tangency, no self-intersection, positive
  tip thickness). Conjugate rack for hobbing.
- Tests:
  - tangent continuity at both joins to 1e-9;
  - Liu defaults give a valid tooth;
  - a hob can be designed for D2's flexspline;
  - an invalid arc set is rejected, naming the parameter.

**TD3. Cams and deformation kinematics (TD-D1).**
- Cosine, ellipse, four-roller and Resàl cams; neutral-line `w, v, μ`; flexspline tooth pose as a
  function of wave-generator angle.
- Tests:
  - the cosine cam matches the closed form to 1e-12;
  - neutral-line length is preserved (inextensibility) to 1e-9 for every cam;
  - pose symmetry about the major axis;
  - the four-roller cam reproduces its circle-arc contact segments.

### Phase B: pairing and meshing

**TD4. Circular-spline envelope (TD-D3).**
- Relative path in the circular-spline frame, swept union, contact-condition refinement, fit to a
  `Sampled` form with a deviation bound.
- Tests:
  - in the rigid limit (`w₀ → 0` with a rigid internal-gear pair), the envelope reproduces the
    standard conjugate internal gear;
  - the derived circular spline is symmetric;
  - re-sweeping the flexspline against the fitted circular spline gives zero penetration within
    the fit bound, and at least one contact.

**TD5. Meshing analysis (TD-D4).**
- Per wave-generator angle and tooth: clearance, penetration, contact; summary metrics.
- Tests:
  - D1 and D2 have zero interference;
  - D4 is reported with the tooth index and wave-generator angle where interference occurs;
  - flank-engaged and tip-edge-contact fractions are reported apart, with the minimum clearance of each region;
  - no ordering claim (rise with κ, or D2 above D1) is asserted: the measured values are in the TD5 notes, and they are unloaded kinematic numbers.

**TD6. Design rules and manufacturability (TD-D5).**
- Backlash, clearance, tip thickness, fillet, hobbability, internal generation and the bending
  strain proxy.
- Tests: each rule fires on a crafted case and names the parameter; D1–D3 pass at nominal; D4
  fails for its known reason.

### Phase C: search, output, use

**TD7. Design search (TD-D7).**
- Grid then Nelder–Mead over κ, cam and arc parameters, with a candidate table.
- Tests:
  - deterministic results for fixed seeds;
  - the best D2 variant satisfies all rules;
  - the candidate table includes every evaluated point.

**TD8. `StrainWaveDesign` and CadKit parts (TD-D8).**
- The record. Flexspline cup (teeth on the rim, wall and diaphragm from the coefficients),
  circular-spline ring, cam. Valid solids.
- Tests:
  - the record round-trips through its codec;
  - solids are valid and their tooth counts match;
  - the flexspline solid's tooth section matches the form within the fit bound.

**TD9. Meshing view in the app.**
- Animated wave-generator rotation with the flexspline visibly deformed (exaggeration factor
  shown), backlash colouring per tooth, report panel.
- Tests: the app check loads D2 and steps a full wave-generator turn without errors.

**TD10. Acceptance and handoff.**
- D1, D2 and D3 produce complete designs with meshing reports. D4 is refused with the right
  parameter named.
- **Hobber handoff:** the hobber's Phase D takes D2's flexspline. `HobDesign` builds its hob, the
  hobbing sim cuts it, and metrology grades it against the double-circular-arc form.
- A short report: every assumed number, and the kinematic-only scope.

### Later

- **Cup coning:** a cup flexspline deforms more at the open end than near the diaphragm, so
  deformation varies along the face width. This needs 3D tooth geometry and longitudinal
  modification (He 2024). It is significant for real cup-type units; the first candidate after
  TD10.
- Loaded meshing: flexspline compliance under torque (ring model, then FE). Engagement and
  stiffness versus torque.
- Root stress and fatigue (Goodman, Sensors 2026 review methods). Rated torque derived instead of
  stated.
- S-tooth (Ishikawa), full-conjugate circular-spline-first (Xie 2024), cycloid forms
  (*Actuators* 2025).
- Pancake type (two internal splines).
- The behaviour model (stiffness, hysteresis, efficiency, ripple) for the cobot simulations.

## Order

```
TD0 ── TD1 ── TD2 ──┐
         └── TD3 ───┴── TD4 ── TD5 ── TD6 ── TD7 ── TD8 ── TD9 ── TD10
```

- TD2 and TD3 can run in parallel after TD1. TD4 needs both.
- **TD1 changes `GearSpec`, which the hobber uses.** Start it only after the hobber's HB12 has
  landed, or coordinate with the session running the hobber, so the two don't edit `gearkit` at
  the same time.

## Working rules

- Start from the hobber branch once HB12 is done (or from local main once the hobber has merged),
  in its own worktree.
- Dependencies count only once they are on local `main`, or on the base branch named above.
- A blocked step is marked `blocked: <what>`.
- Commit per step, e.g. `Tooth design: deformation kinematics (TD3)`, with Progress updated.
- Diagnose before working around. Numbers not from a source or a measurement are marked assumed.
- No legacy compatibility. Heavy suites go through the heavy-run wrapper.

## Progress

| Step | State | Commits |
|---|---|---|
| TD0 | done | Tooth design: baseline and test-design inputs (TD0) |
| TD1 | done | Tooth design: tooth-form interface (TD1) |
| TD2 | done: `DoubleCircularArc` / `DoubleArcProfile` (flat construction rebuilt from the definitions, wrapped on the pitch circle; circular root and tip blends assumed); deviations from the paper in the TD2 notes | Tooth design: double-circular-arc form (TD2) |
| TD3 | done: cosine, ellipse, four-roller, Resal (Gravagno 2016 eq 18) | Tooth design: deformation kinematics (TD3) |
| TD4 | done for involute forms (D1, D3); not fitted to a full `Sampled` ToothForm (root/tip blends), and no contact-condition refinement: accuracy is by convergence | Tooth design: circular-spline envelope (TD4) |
| TD5 | done for D1, D2, D3, D4 (unloaded kinematic), metric corrected 2026-11-01: flank engagement and tip edge contact are reported apart. D1 kappa 1 flank 1.6 % (tip edge 46.5 %); D2s 16.3 %; D3 0 %. Root and minor-tip clearances are arithmetic on the assumed circular-spline radii, transmission error is zero by construction, and cam differences of a few um are inside model error (see TD5 notes) | Tooth design: meshing analysis and D1/D3 comparison (TD5); Tooth design: double-circular-arc form (TD2) (D2 comparison) |
| TD6 | planned | |
| TD7 | planned | |
| TD8 | planned | |
| TD9 | planned | |
| TD10 | planned | |
| Later | later | |
