---
title: Licensing
description: What is open and under which licence, the third-party components Materia uses, and what will be commercial.
status: draft
---

# Licensing

Everything in this repository is open source. This page summarises the
licences; the full decision, with the reasons, is in [Open core](../OPEN_CORE.md).
Where this page and a `LICENSE` file disagree, the `LICENSE` file wins.

## Materia's own work

| What | Licence |
|---|---|
| Code in this repository and in Haxeon (compiler, runtime, packages) | [Apache-2.0](../../LICENSE) |
| NativeKit (windows, input, GPU, audio), a Haxeon submodule | Apache-2.0 |
| Machine designs in `machinekit/examples/` and their generated outputs (BOMs, cut lists, drawings, weld sequences) | Apache-2.0, the same as the code |
| Documentation in `docs/` | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) |

Apache-2.0 lets you use, change and sell what you build with Materia,
including your own machines and controllers, as long as you keep the licence
and the notices in the [`NOTICE`](../../NOTICE) file. It also includes a patent
grant from contributors.

That covers the whole open stack: the kits, the editor on desktop and in the
browser, the example machines, the controller protocol and the reference
controller (the Rust device core, the virtual device and the bench board
support). See [The controller boundary](../concepts/execution/controller-boundary.md).

If a hardware design is ever published on its own, outside this repository,
it would use one of the CERN open-hardware licences (CERN-OHL-W-2.0 or
CERN-OHL-P-2.0). None has been published that way yet.

## Contributing

Contributions are accepted under the same licences: Apache-2.0 for code and
designs, CC BY 4.0 for documentation. Every commit carries a
`Signed-off-by:` line certifying the
[Developer Certificate of Origin](https://developercertificate.org/)
(`git commit -s` adds it). There is no contributor licence agreement. See
[`CONTRIBUTING.md`](../../CONTRIBUTING.md).

## Third-party components

Each submodule and vendored dependency keeps its own licence file. The kits
that vendor native code describe what they use, which files are compiled and
which are left out, in their `THIRD_PARTY.md` files. The table lists the
licences the repository records today.

| Component | Used by | Licence |
|---|---|---|
| Open CASCADE Technology 8.0.1 | CadKit | LGPL-2.1 with the OCCT exception 1.0 |
| Eigen | MotionKit, KinematicsKit, CollisionKit | MPL-2.0 |
| MuJoCo (Materia's fork) | SimKit | Apache-2.0 |
| OpenCV 4.14 (six modules) | VisionKit | Apache-2.0 |
| Descartes Light (core headers) | MotionKit | Apache-2.0 |
| Ruckig | MotionKit | MIT |
| TOPP-RA (Materia's fork) | MotionKit | MIT |
| MCAP | RobotKit | MIT |
| cgltf, ozz-animation | Haxeon animation package | MIT |
| HashLink, Haxe standard library | Haxeon | MIT |
| coal (Materia's fork) | CollisionKit | BSD-3-Clause |
| V-HACD | CollisionKit | BSD-3-Clause |
| EAIK and IK-Geo | MotionKit | BSD-3-Clause |
| ProxSuite (ProxQP) | KinematicsKit | BSD-2-Clause |
| LZ4 (the `lib/` sources only) | RobotKit | BSD-2-Clause |
| stb_image | Haxeon animation package | Public domain or MIT |

MuJoCo's own dependencies (libccd, lodepng, MarchingCubeCpp, miniz, qhull,
tinyobjloader, tinyxml2) are vendored with their licence files under
`simkit/vendor/mujoco-deps/`. NativeKit's text, audio and networking libraries
are submodules of NativeKit with their own licences. Neither group has been
added to a single inventory yet.

**Open CASCADE** is the only copyleft (LGPL) library found so far. It does not
restrict the open stack, whose source is available anyway. The browser build
links it statically, which is fine while the web editor is open source. A
closed product that ships CadKit would have to allow relinking under LGPL-2.1;
the plan is that the commercial controller does not link a CAD kernel at all.

**Eigen** is MPL-2.0: changes to Eigen's own files must be shared, while code
that only uses it is not affected.

**Supplier data.** Some examples use supplier STEP files and datasheets (robot
descriptions, grippers, rails, motors). These are not part of Materia's
licence. Several are downloaded by a preparation script with a pinned checksum
rather than committed, and their READMEs say when no redistribution licence is
assumed.

### Not done yet

Before the first release, the project will:

- generate one `THIRD_PARTY_NOTICES` file covering every submodule and
  vendored dependency, including the groups above that are not inventoried;
- scan the dependencies for anything under the GPL. One example of why: the
  LZ4 repository is partly GPL-2.0-or-later, and RobotKit compiles only its
  BSD-licensed `lib/` sources;
- check the terms of third-party assets such as supplier CAD and catalogue
  data.

## What will be commercial

Nothing commercial exists yet. The plan is to sell, later and from separate
private repositories:

- **A Materia controller**: controller hardware, production firmware built on
  the open device core, safety functions (emergency stop, guard interlocks,
  safe torque off), drive tuning and commissioning tools, and certified
  builds;
- **selected machine designs as products**, with the technical file, risk
  assessment and instructions for use that selling a machine requires;
- possibly hosted services and support.

These will depend on the open stack through its published interfaces. If one
needs a change in an open kit, that change is made in the open kit, in public.
The rule of thumb: what you need to understand, build and run a machine is
open; what you pay for is a finished, certified, supported product.

Product names and a trademark policy are not decided yet.

<!--
Sources: docs/OPEN_CORE.md; LICENSE, NOTICE, CONTRIBUTING.md; haxeon/LICENSE;
haxeon/vendor/nativekit/LICENSE (Apache-2.0, nativekit 2489d27f);
cadkit/third_party/occt/README.md and OCCT_LGPL_EXCEPTION.txt; cadkit/README.md
(OCCT V8_0_1); motionkit/native/THIRD_PARTY.md; kinematicskit/native/THIRD_PARTY.md;
collisionkit/native/THIRD_PARTY.md; visionkit/native/THIRD_PARTY.md;
haxeon/packages/animation/native/THIRD_PARTY.md; simkit/vendor/mujoco/LICENSE;
robotkit/vendor/mcap/LICENSE, robotkit/vendor/lz4/LICENSE and robotkit/CMakeLists.txt;
.gitmodules.
-->
