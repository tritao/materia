# Materia concepts

These pages explain the ideas you need to understand Materia: what each layer
is for, how the layers fit together, and where the boundaries are. They do not
teach the API (that is the reference) or walk through tasks (those are the
tutorials and how-to guides).

Start with the two pages in **Start here**, then read whichever section matches
what you are working on. Every page links back to its step in
[The life of a machine](start/life-of-a-machine.md).

## Start here

- [Materia in one page](start/overview.md)
- [The life of a machine](start/life-of-a-machine.md): one gantry followed from
  geometry to the controller

## Foundations

- [Conventions](foundations/conventions.md): frames, axes, units, quaternions
- [Frames and transforms](foundations/frames.md)
- [Identity](foundations/identity.md): stable identities and persistent names
- [Data vs. runtime](foundations/data-and-runtime.md)
- [Immutability and snapshots](foundations/immutability.md)
- [Determinism](foundations/determinism.md)
- [Truth vs. observation](foundations/truth-and-observation.md)
- [Diagnostics](foundations/diagnostics.md)

## Modelling: describing a machine

- [CAD documents](modelling/cad-documents.md)
- [Assemblies and mates](modelling/assemblies-and-mates.md)
- [Machine components](modelling/machine-components.md)
- [Mechanical truth](modelling/mechanical-truth.md)
- [Transmissions](modelling/transmissions.md)
- [Collision model](modelling/collision.md)
- [Specialized models](modelling/specialized-models.md): sheet metal, gears, BIM,
  humans

## Motion: making it move

- [The kinematic forest](motion/kinematic-forest.md)
- [Paths vs. trajectories](motion/paths-and-trajectories.md)
- [Execution plans](motion/execution-plans.md)
- [Machine-axis views](motion/machine-axes.md)

## Process: making it do work

- [Process vs. robot](process/process-and-robots.md)
- [Toolpaths](process/toolpaths.md)
- [Stock and material removal](process/stock.md)
- [Manufacturing outputs](process/manufacturing-outputs.md)

## Simulation and execution

- [Fidelity levels](execution/fidelity.md)
- [Simulation worlds](execution/simulation-worlds.md)
- [The robot runtime](execution/robot-runtime.md)
- [The controller boundary](execution/controller-boundary.md)
- [Facilities and fleets](execution/facilities-and-fleets.md)

## Platform: what it runs on

- [Haxeon](platform/haxeon.md)
- [NativeKit and UIKit](platform/nativekit-and-ui.md)
- [SceneKit rendering](platform/rendering.md)
- [The editor and projects](platform/editor-and-projects.md)
- [Web vs. native](platform/web-and-native.md)

## Reference

- [Glossary](glossary.md)

---

## Writing these pages

Every concept page uses the same five sections:

1. **The problem**: why the concept exists, as the question a reader has
   before reading.
2. **The model**: the core idea and one diagram. No API detail.
3. **In Materia**: the kit that owns it, the key types, links into the API
   reference.
4. **What it isn't**: boundaries and common confusions.
5. **See also**: the step in *The life of a machine*, then neighbouring
   concepts.

Rules:

- Explain each concept once, here. Kit READMEs link to the concept page instead
  of restating it.
- Name real types and files, but keep the page true when signatures change: a
  concept page should survive a refactor that the API reference does not.
- When explaining something exposes an inconsistency (two kits disagree on
  units, two names for one idea, a boundary the code does not follow), record it
  under **Open questions** at the bottom of the page and file it. Do not write
  around it.
- Diagrams are Mermaid code blocks (GitHub and the site both render them);
  the few key diagrams may be hand-made SVG using the site's colour variables.
  Never ASCII art. Data figures are computed by `docs/figures`, never drawn.
- Plans, TODOs and handoffs stay where they are. Distil their decisions into
  these pages; do not link readers into them.

Each stub lists the sources to distil in a comment at the bottom. Pages carry
`status: stub | draft | reviewed` in their front matter.

Writing order: *The life of a machine* → Conventions → Execution plans →
Fidelity levels → Assemblies and mates → The controller boundary.
