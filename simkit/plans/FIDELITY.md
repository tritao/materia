# Simulation fidelity levels

Every simulation in Materia currently runs at full physical fidelity: step-level motor counters,
motor holds, compliant couplings, MuJoCo contacts and pneumatic substeps. That is right when a
check depends on it (seam tracking, force search, brake holding, frame stretch) and wasted when it
does not (deposition geometry of a long print, reach and collision sweeps, scrubbing a preview).
This plan adds explicit fidelity levels over one execution plan.

Packages involved: `simkit` (backends and model layers), `robotkit` (runtime/device simulation),
`app` (players, project-source cases, editor), `machinekit` (example acceptance).

## Levels

| Level | Evaluates | Typical checks |
|---|---|---|
| Kinematic | The plan's timed joint states replayed exactly; no forces, steps or contacts | Toolpath and deposition geometry, reach and collision sweeps, long prints, plan timing |
| Ideal | Rigid bodies, ideal drives and transmissions, contacts | Grasp and handoff, pneumatics, contact-dependent cycle time |
| Physical | Ideal plus compliance, motor holds, step rounding, encoder placement, sag | Tracking accuracy, force search, holding and brakes, printer frame stretch, shaping residuals |

## Rules

- One plan for all levels. Planning, certification, streaming and device compilation never change
  with the level; only the simulator chooses which effects to evaluate. A lower level cannot hide a
  planning defect.
- Each check declares the minimum level it requires. Requesting a result the level does not
  model (seam error from a Kinematic run) is an error, not an ideal answer. Tests run each check
  at its declared level, so a pass at a lower level is never reported for a physical property.
- Levels are whole model layers. Physical adds the compliance/motor-hold/quantization layer; Ideal
  omits it; Kinematic bypasses dynamics. No scattered fast-mode branches inside models, and the
  models themselves are identical across levels.
- Results record the level they ran at. Engineering numbers (bead volume, seam error, cycle time)
  are reported with it.
- Relation to `app/plans/REALTIME_PREVIEW.md` (sheet-metal branch): that contract requires the
  real physics at real time and forbids animation-only playback presented as the preview. Kinematic
  playback here is a separately labelled mode for geometry and scrubbing, never the real-time physical
  preview. The two plans share the level vocabulary; RP keeps its no-substitutes rule.

## Steps

**SF0. Survey and baseline.** Inventory where each effect is switched on today (MuJoCo vs
deterministic backend, motor holds, couplings, step observation, pneumatic substeps) and which
project-source/MachineKit checks depend on which effects. Record suite wall time and peak memory per
case on local main as the baseline.

**SF1. Level vocabulary and model layers.** A `SimulationFidelity` value owned by the simulation
session; effects grouped into layers selected once at world construction. Physical reproduces today's
results bit-for-bit.

**SF2. Kinematic backend.** Replays the ExecutionPlan's timed joint states, with deposition, stock
removal and collision queries driven from them. Identical joint trajectories to the plan at sample
times; no device stepping.

**SF3. Declared check levels.** Each acceptance check declares its minimum level; the harness refuses
mismatches. Migrate project-source and MachineKit cases; cases whose checks are all geometric move to
Kinematic, contact-only ones to Ideal.

**SF4. Cross-level consistency.** For each reference scene, a periodic check that Kinematic geometry
and Physical geometry agree within the physical model's derived deviation (for example belt stretch
m·a/K), so the levels cannot drift apart unnoticed.

**SF5. Editor.** Level shown and selectable in the editor; scrubbing and long-job preview use
Kinematic, clearly labelled; Play keeps the real-time physical contract of REALTIME_PREVIEW.

**SF6. Acceptance.** Suite wall time and peak memory against the SF0 baseline; full printer cube and
the pellet printer's large parts checked at Kinematic for geometry and at Physical on short
representative sections; no check's verdict changes.

**Order.** SF0 -> SF1 -> SF2 -> SF3 -> SF4 -> SF5 -> SF6. Before or alongside pellet PL1; PL8 and
PL12 rely on Kinematic for full-size parts.
