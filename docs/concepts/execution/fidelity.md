---
title: Fidelity levels
description: "Kinematic, Ideal and Physical simulation: what each proves and what it does not."
status: draft
---

# Fidelity levels

> **Planned, not implemented.** Today every simulation in Materia runs at full
> physical detail. This page describes the level model the project has adopted
> (`simkit/plans/FIDELITY.md`, steps SF0–SF6) so that pages, tests and results
> can already use the same words.

## The problem

A simulation answers a question, and different questions need different detail.

- Whether a 3D print deposits the right geometry over six hours of moves needs
  only the plan's timed positions.
- Whether a gripper holds a part during a handoff needs rigid bodies and contacts.
- Whether a router follows its path within 0.05 mm at full feed needs belt
  stretch, motor holds and step rounding.

Running everything at the highest detail is correct but slow: long prints and
reach sweeps spend most of their time on effects nobody is checking. Running at
a lower detail is fast but dangerous if a result is then read as if it were
physical.

## The model

Three levels, each a strict superset of the one before:

| Level | Models | Leaves out | Answers |
|---|---|---|---|
| **Kinematic** | The execution plan's timed joint states, replayed exactly | Forces, contacts, steps, compliance | Toolpath and deposition geometry, reach, collision sweeps, plan timing |
| **Ideal** | Rigid bodies, ideal drives and transmissions, contacts | Compliance, motor holds, step rounding, sag | Grasps and handoffs, pneumatics, contact-dependent cycle time |
| **Physical** | Ideal plus compliance, motor holds, step rounding, encoder placement, sag | — | Tracking accuracy, force search, holding and brakes, frame stretch |

Five rules make the levels safe to use:

1. **One plan for every level.** Planning, plan checks, streaming and device
   compilation never change with the level; only the simulator chooses which
   effects to evaluate. A lower level cannot hide a planning defect.
2. **Checks declare their minimum level.** Asking a Kinematic run for a seam
   error is an error, not an optimistic answer.
3. **Levels are whole model layers**, selected once when the world is built:
   Physical adds the compliance/motor-hold/quantization layer, Ideal omits it,
   Kinematic bypasses dynamics. The models themselves are identical across
   levels; there are no scattered "fast mode" branches.
4. **Results record their level.** A cycle time or bead volume is reported
   together with the level that produced it.
5. **The levels are cross-checked.** For reference scenes, Kinematic and
   Physical geometry must agree within the deviation the physical model
   predicts (for example belt stretch, m·a/K), so the levels cannot silently
   drift apart.

## In Materia

- **Today:** SimKit runs MuJoCo for the editor's Play, with every physical
  effect on. The editor can also switch to a deterministic test backend
  (`ApplicationSimulation`). There is no `SimulationFidelity` type yet.
- **Planned:** a `SimulationFidelity` value owned by the simulation session
  (SF1), a Kinematic backend that replays the execution plan (SF2), declared
  check levels in the test harness (SF3), cross-level checks (SF4), and the
  level shown and selectable in the editor (SF5).
- **Results today:** checks do not yet record a level. When SF3 lands, each
  check declares the minimum level it needs, and the harness refuses to run it
  at a lower one.

## What it isn't

- **Not a quality slider.** A level is a statement about which effects exist in
  the model, not a trade of accuracy for speed inside the same effects.
- **Not a different plan.** The plan is computed once; levels only change how
  it is evaluated.
- **Not the real-time preview.** Kinematic playback is a separately labelled
  mode for geometry and scrubbing. Pressing Play keeps the real physics at real
  time.
- **Not about hardware.** Real hardware is always "Physical"; levels exist only
  in simulation.

## See also

- [The life of a machine](../start/life-of-a-machine.md), step 8
- [Execution plans](../motion/execution-plans.md)
- [Simulation worlds](simulation-worlds.md)
- [Determinism](../foundations/determinism.md)
