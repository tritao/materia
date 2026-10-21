---
title: Product
description: What Materia does, from a machine's parts to its controller, with the example machines, the component catalogue and how it is licensed.
status: draft
---

# Product

Materia is a 0.x open-source project: one stack, written mostly in Haxe, to
design, simulate, plan and run machines. A machine is code built from real
parts; its motion limits come from those parts; its work is planned and
checked; and the same plans run in simulation and on a controller. This page
is a short tour. [Features](features.md) lists everything that is built today,
and [Materia in one page](../concepts/start/overview.md) explains how the
pieces fit together.

<div class="product-stages">

<section>

<img src="/screens/press-brake.webp" width="640" height="400" alt="A sheet metal press brake in the editor" loading="lazy">

<div>

## Model

Machines are assembled from catalogue parts that know their geometry,
mounting points, mass and bill of materials, on parametric CAD with
persistent face names. Joints are the real mechanism: a screw turns, a nut
slides.

[Modelling features](features.md#modelling-describing-a-machine) ·
[Every component, in 3D](/components/)

</div>

</section>

<section>

<img src="/screens/robot-arm.webp" width="640" height="400" alt="A six-axis robot arm in the editor" loading="lazy">

<div>

## Move

Speed and acceleration limits are derived from the motors, belts and
screws, never typed in. Paths are timed against those limits
into execution plans that are checked before they run.

[Motion features](features.md#motion-making-it-move) ·
[Transmissions](../concepts/modelling/transmissions.md)

</div>

</section>

<section>

<img src="/screens/cnc-router.webp" width="640" height="400" alt="A desktop CNC router in the editor" loading="lazy">

<div>

## Make

Toolpaths from CAM or G-code, material removal on the stock, welding,
printing, sheet metal and nesting, with shop outputs such as cut lists,
drawings and packets, all from the same model.

[Process features](features.md#process-making-it-do-work) ·
[Toolpaths](../concepts/process/toolpaths.md)

</div>

</section>

<section>

<img src="/screens/gantry-picker.webp" width="640" height="400" alt="A gantry picker in the editor" loading="lazy">

<div>

## Run

The same plans run in a physics simulation and on a real controller, through
one robot runtime and an open device protocol. The editor runs on the desktop
and in the browser.

[Simulation and execution](features.md#simulation-and-execution) ·
[The controller boundary](../concepts/execution/controller-boundary.md)

</div>

</section>

</div>

## More

- [Gallery](gallery.md): every example machine, what it shows, and how to
  open it.
- [Components](/components/): every parametric part, with its parameters and
  where its catalogue data comes from.
- [Roadmap](roadmap.md): what is being built now and what is planned (draft).
- [Licensing](licensing.md) and [open core](../OPEN_CORE.md): what is open,
  under which licence, and what will be commercial later.

Release notes will be added once the first version is decided. Until then,
the commit history is the record of changes.

<!--
Sources: docs/DOCS_PLAN.md (D9), docs/OPEN_CORE.md, docs/product/features.md.
-->
