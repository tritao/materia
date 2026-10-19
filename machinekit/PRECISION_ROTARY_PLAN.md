# Precision rotary-axis plan

**Goal.** Design, source, build and prove our own direct-drive precision rotary axis. The first
unit is the flexspline hobber's C table. Every step is modelled in materia, and the real unit
is measured on our own bench, so the model and the hardware check each other.

1. **Design in materia.** A built-up direct-drive table:
   - axial-radial bearing, frameless torque motor, ring encoder with two read heads;
   - labyrinth seal with air purge, clamp, rotary union, cooling;
   - our own housing, rotor and table top.

   The table's own **error budget** predicts runout, angular accuracy, speed ripple and
   stiffness from part grades, fits and assembly errors, before parts are ordered.
2. **Source** components (Chinese and Western options, from `machinekit/STRAIN_WAVE_RESEARCH.md`,
   *Suppliers and prices*). Outsource the precision grinding of the housing and rotor seats.
3. **Build** with a written assembly procedure: staged bearing bolting, encoder ring centring,
   head alignment, and **grinding the table face and centring bore in place on the table's own
   bearing**.
4. **Prove** it on a rotary-axis test bench:
   - error motion (ISO 230-7 method: capacitive probes on a master ball or cylinder);
   - angular accuracy (against a reference);
   - speed ripple at 15 rpm;
   - stiffness;
   - thermal drift.

   The measured values go back into the model as Measured.
5. **Compare** against a bought table (Demat RTV210-050 class), measured on the same bench. The
   hobber's HB11 budget predicts the gear grade each table would give.

**Why build our own.** Not cost at first (parts ≈ US$5,000–20,000, similar to or above a Chinese
complete table). The reasons:
- **control:** a non-contact seal, low cogging, a large through-bore, low profile, and the encoder
  wired straight into our gearbox board (RKD6);
- **a path to class 3**, with ≤ 1 µm runout from selected bearings and in-place grinding;
- **reuse.** The same competence and bench serve:
  - the hobber's A swivel;
  - the skiver's high-speed work spindle (`examples/circular-spline-skiver/PLAN.md`, harder);
  - the lathe spindle (`examples/lathe/PLAN.md`);
  - a gear-measuring station;
  - the end-of-line gearbox test bench.

**Not replacing the purchase.** The hobber prototype uses a bought table first (hobber Phase D).
This plan runs in parallel and joins the hobber when its unit is proven.

The work spans:
- `machinekit.motion` (bearing, motor, encoder, seal, union, cooling parts) and
  `machinekit.hobbing.RotaryTable`, generalised to `machinekit.rotary` if other machines adopt it;
- `machinekit/examples/precision-rotary/` (the table as its own example and its test bench);
- `gearkit.ErrorBudget`, or a shared error-budget module;
- `processkit` / `manufacturingkit` for the procedures and inspection records;
- the RKD6 device core (encoder interface, shared with the device gearbox: skiver PS-D4).

## What exists

Surveyed 2026-10-19 on branch `flexspline-hobber` (9c1d2dbbd).

- `machinekit.hobbing.RotaryTable`: a built-up C package with housing, `TorqueMotor`,
  `CrossedRollerBearing`, `RotaryEncoderRing`, `RotaryClamp` and platen, a continuous C joint
  and an encoder binding.
- Part fields today:
  - `CrossedRollerBearing`: outside, bore, height, precision class, radial runout, axial runout,
    moment stiffness;
  - `RotaryEncoderRing`: counts per revolution, angular accuracy, eccentricity;
  - `TorqueMotor`: torques, speeds, rotor inertia, cogging torque, pole count, cogging order,
    torque and back-EMF constants, phase resistance;
  - `RotaryClamp`: holding torque, release time.
- The hobber's HB11 error sources include table radial and axial runout, table tilt, encoder
  eccentricity and graduation error, and cogging (HB13).
- A **bought-table form** was requested for HB5 (catalogue or measured values with
  ValueBasis). This plan's measurements feed it.
- The electronic gearbox (`robotkit/runtime/ELECTRONIC_GEAR.md`) and the device-gearbox plan
  (skiver PS-D4).
- Supplier research and the C-table purchase specification: `STRAIN_WAVE_RESEARCH.md`.

**Missing.**
- Bearing mounting physics: fits, preload, bolt sequence, seat form errors and their effect on
  runout.
- One versus two encoder heads, and interpolation error.
- Seal drag and labyrinth geometry; rotary union and cooling parts.
- A table-level error budget; in-place finishing as a modelled step.
- Assembly and inspection procedures as data.
- A rotary-axis test bench (machine and measurement model); analysis of measured data.

## Decisions

**RA-D1. The table is designed from the requirement, not from a catalogue.**
- The requirement comes from the consumer. For the hobber: class 5–6 → radial and axial
  non-repeatable runout ≤ 1–2 µm; angular accuracy ±1–2″; speed ripple at 15 rpm inside the HB7
  sync budget; tilt stiffness at a mandrel stack of 100–150 mm.
- The table error budget (RA-D5) allocates it to bearing, seats, assembly, encoder, motor and
  seal.
- Bearing size ≈ Ø200–250 class (HB-D11), with a through-bore for a rotary union.

**RA-D2. The bearing seats are ground, and the table face is ground in place.**
- Housing and rotor seats follow the bearing maker's mounting guide (form, size, squareness).
  Outsourced to a precision grinding shop, with inspection reports.
- After assembly, the **table face and centring bore are finish-ground or hard-turned on the
  table's own bearing**, driven by its own motor. This removes the stack-up of every part below
  the face.
- The model has a "finished in place" step that resets the face and bore errors to the finishing
  process's capability.

**RA-D3. Two encoder heads, 180° apart.**
- Averaging cancels first-order ring eccentricity and most mounting errors.
- Residuals: graduation error and the heads' **interpolation error** (sub-divisional error at
  the line pitch), which beats against the tooth frequency into pitch error. Both are stated per
  encoder and are error sources.
- One head is allowed for the budget build, with eccentricity as a source.

**RA-D4. Non-contact sealing.**
- A labyrinth between rotor and housing with a positive-pressure air purge. No contact lip seal
  on the rotating interface, because contact-seal drag variation spoiled rotation accuracy on the
  GE15FRplus.
- Seal drag is a parameter: about zero for the labyrinth, a stated value and variation for
  contact seals (used for bought tables).
- Coolant exposure is checked against the hobber's chip and coolant layout.

**RA-D5. The table has its own error budget, shared with the machine budgets.**
- Sources, each on its part:
  - bearing runout by grade, and preload tilt;
  - seat roundness and squareness;
  - bolt-induced distortion (a staged-tightening residual);
  - encoder graduation, interpolation and eccentricity (one or two heads);
  - motor cogging and torque ripple through the servo loop (speed ripple at 15 rpm);
  - seal drag;
  - thermal growth from motor losses with and without cooling;
  - clamp release offset.
- Outputs:
  - radial and axial error motion (synchronous and asynchronous, ISO 230-7 terms);
  - angular error vs angle;
  - speed ripple spectrum;
  - tilt, axial and radial stiffness.
- The same quantities the bench measures. The hobber's HB11 table sources consume them
  directly.
- Analytic or lumped where possible. A fine simulation only where coupling matters (servo
  ripple through the gearbox).

**RA-D6. Procedures are data.**
- Assembly: clean bench; bearing bolts in stages crosswise to torque; runout measured between
  stages; encoder ring centred with a 0.1 µm indicator; heads aligned to spec; preload and
  thermal soak before final measurement.
- In-place finishing step; final inspection.
- Each step: what to measure, the limit, what to do on failure.
- Generated as a document from the model, like the casting plan's CT-D10. It later becomes
  input for assembly planning.

**RA-D7. Measurement follows ISO 230-7 and is reproducible.**
- **Error motion:** capacitive probes (or 0.1 µm-class indicators) against a precision master
  ball or cylinder, using reversal or multi-probe methods to separate artefact error from
  spindle error. Synchronous and asynchronous radial, axial and tilt error.
- **Angular accuracy:** autocollimator plus optical polygon, or a calibrated reference encoder
  (rented, or a calibration service).
- **Speed ripple:** encoder data at 10–20 rpm under the gearbox loop.
- **Stiffness:** a known load and an indicator. **Thermal:** a probe drift run over a soak.
- Analysis code in materia turns raw bench data into the RA-D5 quantities. Measured values get
  ValueBasis Measured, with instrument uncertainty.

**RA-D8. Bought and built tables share one interface.**
- Both satisfy the same rotary-axis description (the bought form from HB5, the built-up
  `RotaryTable`). Machines don't care which.
- The bench measures either.
- **The decision rule:** adopt the built table for a machine when its measured values give a
  better predicted grade (HB11) at acceptable cost and risk.

**RA-D9. Generic names in code;** vendors only as sources in docs.

## Reference numbers (assumed until checked against a datasheet or a measurement)

| Quantity | Value | Source |
|---|---|---|
| Bearing | YRT/YRTS-class, ≈ Ø200 bore, P4 (P2 selected for class 3) | assumed; grades per maker's tables |
| Bearing runout | P4 ≈ 2–3 µm, P2 ≈ ≤ 1–1.5 µm | maker tables / Chinese claim ≈ 1.5 µm, check |
| Torque motor | ≈ 50–60 Nm continuous, ~200 Nm peak (DDF195-50 / RTV210 class) | assumed from HB loads + acceleration |
| Encoder | ring, two heads, ≥ 2²³ counts, ±1–2″ | assumed |
| Seal | labyrinth + air purge, drag ≈ 0 | RA-D4 |
| Target error motion | ≤ 1–2 µm (class 5–6), ≤ 0.5–1 µm (class 3) | hobber HB-D6 / HB13 |
| Target speed ripple | inside the HB7 sync budget at 15 rpm | hobber HB7 |
| Parts cost | ≈ US$5,000–20,000 | `STRAIN_WAVE_RESEARCH.md` |
| Bench instruments | capacitive probes + amplifier, master ball/cylinder, 0.1 µm indicators; autocollimator + polygon rented | assumed |

## Steps

### Phase A: model and design (can start on the hobber branch)

**RA0. Baseline and requirement.**
- Record the gate. Write the hobber C requirement from HB-D6 and HB11 as an RA-D1 requirement
  record.
- Tests: the requirement record round-trips; its limits match the hobber's class-6 allocations.

**RA1. Part fields and new parts.**
- Extend `CrossedRollerBearing` (or add an axial-radial bearing type): preload, fits, seat-form
  sensitivity, speed limit.
- `RotaryEncoderRing`: head count, interpolation error amplitude and order.
- New parts: `LabyrinthSeal` (drag, purge flow), `RotaryUnion` (channels, drag), `CoolingJacket`
  (heat removal).
- Every field carries a ValueBasis.
- Tests: recipes, BOM rows, validation errors naming the field.

**RA2. Built-up table assembly.**
- Generalise `RotaryTable` to hold the RA1 parts, the through-bore, and a finished-in-place
  face/bore step.
- `examples/precision-rotary/` as its own project and Start-page entry.
- Checks: fits, clearance, union passage, cooling routing, insert/mount rules as in the hobber.

**RA3. Table error budget (RA-D5).**
- Analytic and lumped models per source, plus a servo-ripple simulation through the existing
  gearbox loop.
- Tests:
  - each source shows its signature (one head: once-per-revolution angular error from
    eccentricity; two heads: cancelled to the stated residual; cogging: ripple at the cogging
    order × speed);
  - superposition is reported;
  - the budget for the reference design meets RA0 or names the limiting part.

**RA4. Design iteration.**
- Run RA3 over bearing grade, one vs two heads, seal type and motor choice, and pick the design.
  Record the chosen configuration, its predicted values and cost.
- Tests: the chosen design meets the hobber C requirement at P4 with two heads (or the plan
  records that it doesn't and why).

### Phase B: sourcing, manufacturing, assembly

**RA5. Sourcing (RA-D9).**
- Requests for quotation for bearing, motor, encoder, union and drive (two suppliers each where
  possible). A grinding shop for the seats. Results recorded in `STRAIN_WAVE_RESEARCH.md` with
  prices and lead times.
- Done when: an order list with prices, plus inspection-report requirements for the ground parts.

**RA6. Drawings and procedures (RA-D6).**
- Manufacturing drawings of housing, rotor and table top from the model, with seat tolerances
  from the bearing guide.
- The assembly, finishing and inspection procedure document, generated.
- Tests: the drawings carry every RA-D2 seat tolerance; the procedure lists each step's
  measurement and limit.

**RA7. Build.**
- Physical assembly per RA6, with in-place finishing of the face and bore. Every intermediate
  measurement is recorded.
- Done when: the table runs under its drive, and the records are complete.

### Phase C: proof

**RA8. Rotary-axis test bench (RA-D7).**
- The bench as a materia machine: base, probe nest, master artefact, reference encoder or
  autocollimator mount, load device, thermal sensors.
- The analysis code: error motion separation, angular error, ripple spectrum, stiffness.
- Tests: analysis code validated on synthetic data with known errors (the reversal method
  recovers an injected spindle error to its stated tolerance).

**RA9. Measure.**
- The built table, and the bought table (Demat RTV210-050 class), on the same bench. Results
  entered as Measured values with uncertainties.
- Done when: both tables have measured error motion, angular error, ripple at 15 rpm, stiffness
  and thermal drift.

**RA10. Compare and decide (RA-D8).**
- The hobber's HB11 table sources run with each table's measured values: predicted gear grade
  per table. Model vs measurement agreement for the built table, per RA3 quantity.
- **Acceptance:**
  - the built table meets the hobber C requirement, or the report names the limiting part;
  - model and measurement agree within stated bounds, or the model is corrected;
  - a recommendation of which table goes into the hobber.

### Phase D: later

- Class-3 version: selected P2 bearing, in-place grinding, thermal control.
- The skiver's high-speed work spindle (bearing behaviour at speed, balancing).
- The A swivel (small, clamped).
- A rotary axis for a gear-measuring station (an air-bearing option).
- Lathe spindle reuse.
- Encoder wired to the RKD6 device gearbox with hardware timestamps (skiver PS-D4).

## Order

```
RA0 ── RA1 ── RA2 ── RA3 ── RA4 ── RA5 ── RA6 ── RA7 ──┐
                       └──────── RA8 ───────────────────┴── RA9 ── RA10
```

- Phase A is model-only and can start on the hobber branch (it touches `machinekit.motion` and
  `hobbing.RotaryTable`; coordinate with the hobber session).
- RA8 (bench model and analysis) can be built while parts are on order.
- RA9 needs the physical table and bench, plus the bought table from the hobber's Phase D
  purchase.

## Working rules

- Phase A on the hobber branch or after it lands, in its own worktree. Coordinate edits to
  `RotaryTable` with the hobber session.
- Dependencies count only once they are on local `main`.
- A blocked step is marked `blocked: <what>` in Progress.
- Commit per step, e.g. `Precision rotary: table error budget (RA3)`, with Progress updated in
  the same commit.
- Numbers not from a datasheet or a measurement are marked assumed. Measured values carry
  instrument uncertainty.
- No legacy compatibility.

## Progress

| Step | State | Commits |
|---|---|---|
| RA0 | planned | |
| RA1 | planned | |
| RA2 | planned | |
| RA3 | planned | |
| RA4 | planned | |
| RA5 | planned | |
| RA6 | planned | |
| RA7 | planned | |
| RA8 | planned | |
| RA9 | planned | |
| RA10 | planned | |
| Phase D | later | |
