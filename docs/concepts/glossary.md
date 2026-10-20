---
title: Glossary
description: One line per Materia term, linked to the page that explains it.
status: draft
---

# Glossary

Each term links to the concept page that explains it. When two kits use
different words for one idea, the glossary names the preferred term and lists
the other as an alias.

| Term | Meaning | Explained in |
|---|---|---|
| `A_T_B` | Transform mapping coordinates in frame `B` into frame `A`. | [Conventions](foundations/conventions.md) |
| Actuator | A motor attached to a joint (for a belt axis, the pulley joint, not the axis). | [Transmissions](modelling/transmissions.md) |
| Assembly | Components connected by joints (which move) and mates (which position). | [Assemblies and mates](modelling/assemblies-and-mates.md) |
| BOM line | The bill-of-materials entry every machine component carries. | [Machine components](modelling/machine-components.md) |
| Closure | A connection that closes a mechanical loop: a condition two connectors must meet, with no coordinate of its own. | [Assemblies and mates](modelling/assemblies-and-mates.md) |
| Connector | A named frame on a component; components attach only through connectors. | [Machine components](modelling/machine-components.md) |
| Coupling | A joint driven by others: `follower = leader × ratio + offset`. | [Transmissions](modelling/transmissions.md) |
| Derived limits | Speed and acceleration caps computed from motors, transmissions and loads. | [Mechanical truth](modelling/mechanical-truth.md) |
| Device binding | The map from a model's actuators to a controller's channels. | [The robot runtime](execution/robot-runtime.md) |
| Driven joint | A joint that is an input of a mechanism; loop solves never move it. | [Assemblies and mates](modelling/assemblies-and-mates.md) |
| Execution plan | Immutable, validated, timed joint motion; the contract between planning and execution. | [Execution plans](motion/execution-plans.md) |
| Fidelity level | Kinematic, Ideal or Physical simulation detail (planned). | [Fidelity levels](execution/fidelity.md) |
| Grounded | An occurrence fixed to the world. | [Assemblies and mates](modelling/assemblies-and-mates.md) |
| Joint | A degree of freedom between a parent and a child occurrence: fixed, prismatic, revolute or continuous. | [Assemblies and mates](modelling/assemblies-and-mates.md) |
| Leader | The coordinate that is planned; followers are coupled to it. | [Transmissions](modelling/transmissions.md) |
| Machine binding | How a toolpath program meets a machine: axes, rapid speed, travel, I/O channels. | [Toolpaths](process/toolpaths.md) |
| Machine component | Geometry generator + connectors + BOM line (`MachineComponent`). | [Machine components](modelling/machine-components.md) |
| Mate | A placement constraint between two connectors (coincident, coaxial, planar, …). Mates position parts; joints move them. | [Assemblies and mates](modelling/assemblies-and-mates.md) |
| Mission | A declarative list of steps (pick, place, weld, …) a scene asks a robot to do. | [The life of a machine](start/life-of-a-machine.md) |
| Occurrence | One placed copy of a component definition. | [Assemblies and mates](modelling/assemblies-and-mates.md) |
| Path | Geometry only: where to go, not when. | [Paths vs. trajectories](motion/paths-and-trajectories.md) |
| Persistent name | A face or edge name that survives recompute. | [Identity](foundations/identity.md) |
| Plan check | Validation of an execution plan against the hardware, such as stall prediction. | [Execution plans](motion/execution-plans.md) |
| Provenance | Where an operation came from: a G-code line and column, or a CAM operation. | [Toolpaths](process/toolpaths.md) |
| Robot model | The compiled links, joints, couplings and limits that planning and the runtime use. | [The kinematic forest](motion/kinematic-forest.md) |
| Runtime | The owner of a robot that routes commands and executes plans, simulated or real. | [The robot runtime](execution/robot-runtime.md) |
| Scene artifact | The encoded scene an example produces and the editor loads (`SceneArtifact`). | [Data vs. runtime](foundations/data-and-runtime.md) |
| Setup | A named work-coordinate origin in a toolpath program, optionally with its stock and fixtures. | [Toolpaths](process/toolpaths.md) |
| Skill | A reusable process behaviour (welding, finishing, handling a part). | [Process vs. robot](process/process-and-robots.md) |
| Stock | The in-process material a tool cuts. | [Stock and material removal](process/stock.md) |
| Toolpath program | Ordered machining, printing or gearing operations with tools and setups (`ToolpathProgram`). | [Toolpaths](process/toolpaths.md) |
| Trajectory | A path with timing. | [Paths vs. trajectories](motion/paths-and-trajectories.md) |
| Transmission | A mechanical link between joints: lead screw, belt, rack, gear mesh, chain. | [Transmissions](modelling/transmissions.md) |
