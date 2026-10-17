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
- Summary: engaged-tooth fraction, backlash distribution, the contact zone in angle around the
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
- Objectives: maximise engaged-tooth fraction and backlash uniformity, and maximise root fillet
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
  - the engaged fraction rises monotonically with κ over a feasible range;
  - D2 engages more teeth than D1 at equal module scaling (stated margin, plausibility only).

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
| TD0 | planned | |
| TD1 | planned | |
| TD2 | planned | |
| TD3 | planned | |
| TD4 | planned | |
| TD5 | planned | |
| TD6 | planned | |
| TD7 | planned | |
| TD8 | planned | |
| TD9 | planned | |
| TD10 | planned | |
| Later | later | |
