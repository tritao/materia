---
title: Execution plans
description: The one immutable, validated contract between planning and execution.
status: draft
---

# Execution plans

## The problem

Motion in Materia comes from many planners: straight moves, free moves, CNC
toolpaths, welding seams, process paths. It goes to many executors: the
editor's simulation, the robot runtime, a controller over a serial link. If
every planner talked to every executor in its own format, each pair would need
its own validation, and a plan that was safe in simulation could behave
differently on hardware.

So there is one contract in the middle. Planners produce it; executors consume
it; nothing else crosses.

> **MotionKit produces plans. RobotKit executes them.**

## The model

### From program to plan

```
MotionProgram ──► ProgramCompiler ──► ExecutionPlan ──► runtime ──► simulation
 (what to do)       (planning)        (the contract)       │
                                                           └──► device segments ──► controller
```

A **motion program** says what is wanted: move linearly here, move freely
there, switch this output, wait for that input. Programs refer to frames and
poses. CNC and process code compile into programs; neither submits plans
directly.

An **execution plan** says exactly how joints move over time:

- **Joint space only.** No frames, poses or Cartesian data; every value is a
  joint coordinate in SI units (metres or radians).
- **Bounded polynomials.** Motion is a sequence of segments, each a polynomial
  of degree 0 to 5 per joint over its own time interval. Times are integer
  nanoseconds from the start of the plan.
- **Explicit start state.** Position, velocity and acceleration the plan
  assumes at its start, each with a tolerance. An executor that is not in that
  state refuses the plan.
- **Timed events.** Output changes (digital, analog, or a process command with
  an argument) at a time in the plan, with a policy for what happens to them if
  the machine is held.
- **Identity.** A plan id, plus the model and calibration revisions it was
  planned for. A plan for another revision of the machine is refused.

### Barriers split plans

Two program steps are not motion: a **dwell** (pause in place) and **waiting
for an input**, whose end cannot be known in advance. These are **barriers**:
they end one plan, and the next starts after them. A plan never waits for
anything; it only runs.

### Validated where it is made, and again where it runs

Validation happens in layers, and each layer checks what it is responsible for:

1. **At creation**, the trajectory core finds the **exact extremes** of every
   polynomial and compares them with the limits: position, velocity,
   acceleration, jerk and continuity between segments. A limit is only checked
   if it is claimed; an unclaimed limit is reported as *unchecked*, never
   silently passed. **Any failed check refuses the plan.**
2. **The planner certifies what only it can know**: task-space deviation (does
   the tool stay on the path?) and clearance. A failure refuses the plan.
3. **A plan check** asks whether the drives can actually do it: does a stepper
   stay under its pull-out torque at every point, does a servo stay under its
   peak and rated torque, does the drive stretch or lag beyond tolerance?
   Findings are attached to the plan as warnings by default; a caller can ask
   for rejection instead.
4. **At submission**, the runtime checks the plan against the machine it is
   about to move: revisions, capabilities, start state against where the
   machine actually is, coupled joints consistent, events on declared channels,
   and the limits again.
5. **For a controller**, the plan is lowered to per-actuator segments in the
   controller's own number format and tick, and that lowered motion is
   validated again, so what is checked is what will execute.

The report from step 1 travels with the plan, so anyone can see which
guarantees were proven exactly, which were sampled, and which were not checked.

### Coupled joints

A plan carries only the joints a planner chooses (the axes). Joints coupled to
them (screws, pulleys, motors) are filled in by the executor from the coupling
equations (see [Conventions](../foundations/conventions.md#couplings-and-transmissions)),
and the runtime refuses a plan whose followers disagree with their leaders.

### Streaming

A long plan is not sent in one piece. The executor streams it in bounded
chunks with a lead of a fraction of a second, and only the last chunk of a
block ends at rest. Events fire when the plan's own clock crosses their time,
never on wall time, so they stay where the plan put them even when the machine
is slowed or held.

## In Materia

| Piece | Where |
|---|---|
| Trajectory core: polynomial evaluation, validation, immutable plans (C ABI) | `trajectorykit` (`trajectory_core`) |
| The plan in Haxe (native-backed, read in place) | `motionkit.trajectory.ExecutionPlan` |
| Program compiler: TOPP-RA for paths, Ruckig for free moves, events, barriers | `motionkit.robot.ProgramCompiler`, `PlanCertification` |
| Drive feasibility | `motionkit.robot.PlanCheck`, `PlanCheckOptions` (`rejects`), findings on `ExecutionPlan.checked` |
| Streaming to the runtime | `robotkit.execution.PlanExecutor`, `ExecutionPlanSubmission`, `SegmentArrays` |
| Submission checks and per-tick evaluation | `RobotRuntime` (`robotkit/runtime`) |
| Lowering for a controller | `compile_device_segments6` (see [The controller boundary](../execution/controller-boundary.md)) |
| Decisions | `motionkit/plans/CONTRACTS.md`, `robotkit/RESTRUCTURE_PLAN.md` |

Plan-check findings have stable names used in logs and tests:
`stepper-stall`, `servo-peak-torque`, `servo-rated-torque`, `accuracy` and
`electronic-gear-speed`.

## What it isn't

- **Not a path.** A path is geometry; the planner times it and lowers it to
  polynomials before anything is sent (see
  [Paths vs. trajectories](paths-and-trajectories.md)).
- **Not a program.** The program is the request; the plan is the result.
- **Not G-code.** G-code is compiled into a program and then a plan on the
  host; executors never interpret it.
- **Not device commands.** Controllers receive a separate per-actuator
  lowering, validated again.
- **Not interactive control.** Jogging and direct joint targets are separate
  commands, not plan operations.

## See also

- [The life of a machine](../start/life-of-a-machine.md), step 6
- [Paths vs. trajectories](paths-and-trajectories.md)
- [The controller boundary](../execution/controller-boundary.md)
- [Fidelity levels](../execution/fidelity.md): one plan for every level
- [Immutability and snapshots](../foundations/immutability.md)

## Open questions

- **Two plan types.** `motionkit.trajectory.ExecutionPlan` (the validated
  native plan) and `robotkit.execution.ExecutionPlanSubmission` (a streamed
  chunk) are different types, and one plan becomes many submissions. The
  concept is one contract; the names should make the relationship obvious.
- **Some producers skip plan creation.** Jogs, homing moves, servo sessions and
  output-only programs build submissions directly, so for them the runtime's
  checks (position, velocity, acceleration, continuity) are the only gate:
  no exact jerk check, no task-space certificate, no plan check.
- **"Immutable" has exceptions.** On the Haxe plan, the plan-check findings,
  the ends-at-rest flag and the task-space and clearance report entries are set
  after creation.
- **Unfinished restructuring.** The raw-segment submission path still exists
  (RESTRUCTURE R7b), and the C ABI and Haxe types still carry MotionKit names
  rather than TrajectoryKit ones (R7e).
- **Stale text.** The MotionKit README describes a 1–2 control-cycle pause
  between queued trajectories, which streaming no longer has; CONTRACTS P0.1
  says the runtime rejects plans with events, which it now executes.
