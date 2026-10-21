---
title: Materia
description: One open stack to design, simulate, plan and run machines, from CAD to the controller.
template: splash
hero:
  title: Machines, from CAD to the controller
  tagline: Materia is one open stack to design a machine from real parts, derive its motion from its motors and drives, plan and simulate its work, and run it on hardware.
  actions:
    - text: Build your first machine
      link: /tutorials/gantry-picker/
      icon: right-arrow
    - text: How it fits together
      link: /concepts/start/overview/
      variant: minimal
    - text: GitHub
      link: https://github.com/tritao/materia
      icon: github
      variant: minimal
---

# Materia

<img class="landing-shot" src="/screens/editor-gantry-picker.webp" width="1600" height="857" alt="The Materia editor with the gantry picker open: a belt-driven gantry over an infeed table and a pallet, with the scene tree on the left and the machine's axis controls on the right.">

<div class="landing-cards">

<section>

<p class="landing-card-title">Model</p>

Build machines from catalogue parts that know their geometry, mounting points
and bill of materials. Joints are the real mechanism: a screw turns, a nut
slides.

[Assemblies](concepts/modelling/assemblies-and-mates.md) ·
[Components](concepts/modelling/machine-components.md)

</section>

<section>

<p class="landing-card-title">Move</p>

Speed and acceleration limits are derived from the motors, belts and screws,
never typed in. Paths are timed into checked execution plans.

[Transmissions](concepts/modelling/transmissions.md) ·
[Execution plans](concepts/motion/execution-plans.md)

</section>

<section>

<p class="landing-card-title">Make</p>

Toolpaths from CAM or G-code, material removal on the stock, welding, printing,
sheet plans and shop packets, all from the same model.

[Toolpaths](concepts/process/toolpaths.md) ·
[Manufacturing outputs](concepts/process/manufacturing-outputs.md)

</section>

<section>

<p class="landing-card-title">Run</p>

The same plans run in simulation and on a real controller, through one runtime
and an open device protocol.

[Robot runtime](concepts/execution/robot-runtime.md) ·
[Controller boundary](concepts/execution/controller-boundary.md)

</section>

</div>

## Try it: limits that come from the parts

Pick a carriage, a motor and a pulley. RobotKit, running in your browser,
builds the axis and derives how fast it can go and how hard it can accelerate.

<figure class="materia-figure materia-widget" data-widget="belt">

| Pulley | Carriage | Motor | Speed | Acceleration |
|---|---|---|---|---|
| 20 teeth | 2 kg | NEMA 17, 0.45 N·m | 1.53 m/s | 31.4 m/s² |

<figcaption>A belt-driven axis, computed live by Materia's own code. <a href="/concepts/modelling/transmissions/">How transmissions work</a>.</figcaption>
</figure>

## Machines built with it

Every example is code in the repository, opened in the same editor. Screenshots
are taken from the browser build of the editor.

<div class="landing-machines">
<a href="/product/gallery/#gantry-picker"><img src="/screens/gantry-picker.webp" width="640" height="400" alt="Gantry picker" loading="lazy"><span>Gantry picker</span></a>
<a href="/product/gallery/#six-axis-robot-arm"><img src="/screens/robot-arm.webp" width="640" height="400" alt="Six-axis robot arm" loading="lazy"><span>Six-axis robot arm</span></a>
<a href="/product/gallery/#full-sheet-cnc-router"><img src="/screens/sheet-router.webp" width="640" height="400" alt="Full-sheet CNC router" loading="lazy"><span>Full-sheet CNC router</span></a>
<a href="/product/gallery/#desktop-cnc-router"><img src="/screens/cnc-router.webp" width="640" height="400" alt="Desktop CNC router" loading="lazy"><span>Desktop CNC router</span></a>
<a href="/product/gallery/#corexy-3d-printer"><img src="/screens/printer.webp" width="640" height="400" alt="CoreXY 3D printer" loading="lazy"><span>CoreXY 3D printer</span></a>
<a href="/product/gallery/#robot-welder"><img src="/screens/robot-welder.webp" width="640" height="400" alt="Robot welder" loading="lazy"><span>Robot welder</span></a>
<a href="/product/gallery/#sheet-metal-press-brake"><img src="/screens/press-brake.webp" width="640" height="400" alt="Sheet metal press brake" loading="lazy"><span>Press brake</span></a>
<a href="/product/gallery/#mobile-base"><img src="/screens/mobile-base.webp" width="640" height="400" alt="Mobile base" loading="lazy"><span>Mobile base</span></a>
</div>

[All examples](product/gallery.md) · [Every component, in 3D](/components/)

## Where to start

<div class="landing-cards landing-paths">

<section>

<p class="landing-card-title">You build machines in code</p>

Follow [the tutorial](tutorials/gantry-picker.md), then read
[the life of a machine](concepts/start/life-of-a-machine.md) and the
[API reference](/reference/).

</section>

<section>

<p class="landing-card-title">You use the editor</p>

The [editor manual](manual/README.md) covers projects, the workspace, the 3D
view and simulation.

</section>

<section>

<p class="landing-card-title">You want to contribute</p>

[Contributing](https://github.com/tritao/materia/blob/main/CONTRIBUTING.md),
the [concepts](concepts/README.md) and the [roadmap](product/roadmap.md).

</section>

</div>

## Open, and honest about where it is

Materia is open source: the code and machine designs under Apache-2.0, this
documentation under CC BY 4.0. A commercial controller and finished machines
will come later, on the other side of a published interface
([open core](OPEN_CORE.md), [licensing](product/licensing.md)).

It is early, 0.x software. No kit is stable yet, and the
[roadmap](product/roadmap.md) says what is being built now.
