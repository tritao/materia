# Move machines out of machinekit/examples

**Goal.** `machinekit/examples` mixes two kinds of thing: real machine projects (plans, FE, supplier and casting packages,
dozens of files) and small library demos. Machines move to a top-level `machines/<name>/` in this repository, with their own
tests, depending on the kits like any other consumer; library demos stay in `machinekit/examples`. Status: plan, 2026-11-10.

**Why.** Library tests depend on products today (`machinekit/tests/hobber` builds the hobber; 20-30 test manifests point into
`cnc-router`, `bench-mill`, `box-mold`, `casting-shop`), machine decisions live inside a library package, and parallel sessions
on different machines collide in one tree. Same repository, not separate repos: worktrees, haxeon pins and atomic cross-kit
changes stay simple.

## Classification (draft, to confirm)

Counts: files outside `build/`, and test manifests under `machinekit/tests` that reference the example.

**Machines** (move to `machines/`):

| Example | Files | Tests | Notes |
|---|---:|---:|---|
| flexspline-hobber | 203 | 88 | first to move; refactored (RF1-RF7), no session on it |
| sheet-router | 258 | 8 | |
| bench-mill | 170 | 25 | several old branches touch it |
| machine-tending | 71 | 11 | |
| cnc-router | 9 | 28 | many library tests use it as a fixture: see "Fixtures" |
| robot-welder | 19 | 18 | |
| robot-screwdriver | 36 | 7 | |
| mobile-base | 7 | 18 | |
| press-brake | 21 | 7 | |
| printer | 13 | 7 | |
| picking-station | 23 | 14 | |
| gantry-picker, gantry-welder, corexy | 6 / 4 / 4 | 11 / 5 / 15 | small machines |
| lathe, foam-cutter, circular-spline-skiver | 1 each | 0 | plans only |

**Library demos and references** (stay in `machinekit/examples`): linear-module, linear-rail-reference, linear-stage, xyz-stage,
dh-gripper, leadshine-servos, supplier-fasteners, supplier-racks, enclosure-kit, eoat, tool-service, cable-harness, panel-tooling,
spindle-drive-study.

**Decided (2026-11-10):**
- Directory: `machines/`.
- Arms: zeroerr-arm (the physical arm project and its J2 bench), cobot-arm (Reach850 wrist prototype), reference-arms (that arm's
  design baseline and commercial-reference validation), track-arm and overhead-arm (complete cells) are **machines**; robot-arm
  (the generic six-axis demo built from MachineKit parts) stays a **demo**.
- pendant: a **product**, moves to `machines/`.
- casting-shop and box-mold: shared casting tooling, move to a **library home** (`manufacturingkit`, or `machinekit.tooling` where
  they depend on MachineKit parts), not to `machines/`.

## Fixtures

A library test should not build a product. Where `machinekit/tests` builds a machine (hobber, cnc-router, bench-mill, ...),
either the test moves with the machine into `machines/<name>/tests`, or the library test gets its own small fixture in
`machinekit/examples` or `machinekit/tests`. Decided per test while moving its machine.

## Steps (one machine per commit series, hobber first)

- **MM1. Hobber.** `git mv machinekit/examples/flexspline-hobber machines/flexspline-hobber`; move `machinekit/tests/hobber` and
  `machinekit/tests/hobbing`'s hobber-only parts to `machines/flexspline-hobber/tests`; fix manifest dependency paths, docs links,
  the project-source case registry and heavy-run case lists, and anything that names the old path (memory notes, scripts).
  Gate: the same suites pass and the same scene and inventory come out (421 members, 154 BOM lines).
- **MM2...** The other machines one by one, each when no session has open work on it (check worktrees first: many October
  branches still touch bench-mill, cnc-router, machine-tending, press-brake and the casting examples; git follows renames, but
  merge them or retire them first where possible).
- **MM-last.** `machinekit/examples` holds only demos; a check fails if a `machinekit/tests` manifest depends on `machines/`.

## Log
- 2026-11-10 casting shop (fc1ac6365): moved to `machines/casting-shop` as a shared shop project (it needs MachineKit, CAM, CNC
  and toolpath kits; no kit sits above them); its aggregate review moved into the hobber's studies to break the cycle. 50
  manifests rewritten. Shop recipe 41, joint fasteners, bench mill, hobber scene pass; aggregate-review, hobber-source and
  motionkit tests fail identically on main.
- 2026-11-10 MM1 hobber (bec8ada3b): `machines/flexspline-hobber` with `tests/hobber` and `tests/hobbing`; 149 manifests, 21 doc
  links, root-relative references and script root climbs updated. Hobber 3303, hobbing 1185, enclosure end to end, joint
  fasteners, integrity, ProjectKit 426 pass; app suite stops at the existing CosmeticThreadTests failure.
