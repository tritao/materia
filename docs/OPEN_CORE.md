---
title: Open core
description: What is open, what is commercial, where the line sits, and which licenses apply.
status: accepted (licenses in force; names and commercial repos deferred)
---

# Open core

> **Accepted 2026-10-20.** Materia and Haxeon are licensed under Apache-2.0
> (`LICENSE`, `NOTICE`), docs under CC BY 4.0, and contributions use the DCO
> (`CONTRIBUTING.md`). Product names and the commercial repositories are
> deferred (section 8).

## 1. Principle

The open stack is complete and useful on its own: you can design, simulate,
plan and run a machine with it, on hardware you build. The commercial products
add what turns that into a product you can buy and certify: controller
hardware, production firmware with safety functions, and finished machines with
their compliance paperwork.

The line between them is a **published interface**, never a private hook. If a
commercial product needs a change in an open kit, that change is made in the
open kit, in public.

## 2. What is open

Everything in this repository and its submodules:

- **Haxeon**: compiler, runtime, CLI and packages, including NativeKit and
  UIKit.
- **Every kit**: ProjectKit, SceneKit, SimKit, SensorKit, CollisionKit,
  KinematicsKit, TrajectoryKit, MotionKit, RobotKit, ProcessKit, AutomationKit,
  CadKit, MachineKit, GearKit, StockKit, CamKit, ToolpathKit, CncKit,
  ManufacturingKit, VisionKit, AnimKit, HumanKit, BimKit.
- **The editor**, desktop and web.
- **The machine examples** and their fabrication outputs (BOMs, cut lists,
  weld sequences, shop packets).
- **The controller interface**, as an open, versioned specification:
  - the RKD device protocol (records, frame codec, versioning);
  - the deployment and device-layout formats;
  - `DeviceBinding`: how a model's actuators map to controller channels.
- **The reference controller**: the RKD6 device core
  (`robotkit/device_protocol`: a `no_std` scheduled executor, step generator
  and board trait), the virtual device, and the bench board support (Nucleo
  G474RE). This is enough to build and run your own machine controller.

## 3. What is commercial (later)

Kept in **separate, private repositories** from the first commit, depending on
the open stack by pinned version, and never vendored back into it:

- **The Materia controller**: controller hardware, production board support,
  real-time firmware built on the open device core, safety functions
  (e-stop, guard interlocks, safe torque off), drive tuning and commissioning
  tools, and certified builds.
- **Selected machine designs** sold as products: production-ready variants with
  their technical file, risk assessment and instructions for use.
- Possibly later: hosted services (sharing, collaboration) and support.

Rule of thumb: what someone needs in order to *understand, build and run* a
machine is open. What someone pays for is *a finished, certified, supported
product*.

## 4. Licenses (proposed)

| What | License | Why |
|---|---|---|
| Code in this repository and in Haxeon | **Apache-2.0** | Permissive, so it gets adopted; includes a patent grant, which matters in industrial and robotics work |
| Machine designs in this repository | **Apache-2.0** (same as the code) | Here a design *is* source code (`machinekit/examples/*`) plus generated outputs; one license per directory avoids split licensing |
| Standalone hardware releases (if any are published outside the repo) | **CERN-OHL-W-2.0** or **CERN-OHL-P-2.0** | The standard open-hardware licenses, written for manufactured products |
| Documentation (`docs/`) | **CC-BY-4.0** | The usual license for prose and figures |
| Commercial products | Proprietary | Separate repositories (section 3) |

Contributions: **DCO** (a `Signed-off-by:` line on every commit), not a CLA.
Because the commercial parts are separate code that you own, you never need to
relicense contributors' work, so a CLA would only add friction.

## 5. Third-party constraints

Each submodule and vendored dependency keeps its own license, including the
`tritao/*` forks (MuJoCo, coal, TOPP-RA, HashLink, NativeKit, nanovg, sokol and
others). Licenses already recorded in the repo:

| Dependency | License | Constraint |
|---|---|---|
| Open CASCADE (CadKit) | LGPL-2.1 + OCCT exception 1.0 | The only copyleft dependency found so far; see below |
| Eigen | MPL-2.0 | File-level copyleft; changes to Eigen's own files must be shared |
| MuJoCo, OpenCV 4.14, Descartes Light | Apache-2.0 | None |
| Ruckig, TOPP-RA, ozz-animation, cgltf, HashLink, Haxe stdlib | MIT | Keep notices |
| coal, V-HACD, EAIK | BSD-3-Clause | Keep notices |
| ProxSuite | BSD-2-Clause | Keep notices |
| stb | Public domain or MIT | None |

**Open CASCADE** is fine for the open stack: the source is open anyway, so
anyone can relink. For **commercial binaries**, prefer not linking OCCT at all.
The controller should not need a CAD kernel. If a commercial product must link
it, link it dynamically, or ship what LGPL-2.1 §6 requires for relinking. The
web build links OCCT statically, which is fine while the web editor is open
source.

**Still to do before release:**

- A full inventory of every submodule and vendored dependency (NativeKit's
  text, audio and networking stack, MuJoCo's dependencies, robotkit's `mcap` and
  `lz4`), generated into one `THIRD_PARTY_NOTICES` file.
- A scan for anything GPL.
- A check of third-party **assets**: supplier STEP files, datasheet figures,
  catalog data. These often carry their own terms, and some research notes
  already flag licence caveats.

## 6. Name and trademark

In open core, the trademark is what stops someone shipping "Materia Controller"
built on your open kits. Before release:

- Run a clearance search. "Materia" is a common word and likely taken in some
  classes; decide the product names with that in mind.
- Add a short trademark policy: forks and modified builds must not use the
  product names; "compatible with Materia" is allowed.

## 7. Regulation (commercial products, EU)

Applies to the products in section 3, not to the open stack. Verify dates and
scope with the official texts.

- **Machinery Regulation (EU) 2023/1230** (applies from January 2027):
  selling a machine requires CE marking, a technical file, a risk assessment
  and instructions for use. Product manuals become a legal deliverable; the
  docs plan reserves a section for them.
- **Cyber Resilience Act**: the commercial controller is a "product with
  digital elements", with security-update and vulnerability-reporting duties.
  Open-source stewards have lighter obligations.
- **Functional safety**: the controller's safety functions fall under standards
  such as ISO 13849-1, which shapes how they are designed, validated and
  documented.

## 8. Decisions

Taken (2026-10-20):

1. **Licenses:** Apache-2.0 for code and designs, CC BY 4.0 for docs.
2. **The device core stays open:** `robotkit/device_protocol` and the bench
   board support are the open reference controller.
3. **Contributions:** DCO.

Done: `LICENSE` and `NOTICE` in Materia and Haxeon, `CONTRIBUTING.md`.

Deferred:

4. **Names:** product names for the controller and machines, after a clearance
   search.
5. **Commercial repositories:** where they live, and how they pin the open
   stack.
6. **NativeKit** (`tritao/nativekit`, own code) needs its own `LICENSE`.
7. **Full third-party inventory** into one `THIRD_PARTY_NOTICES`, the GPL scan
   and the asset check from section 5.
