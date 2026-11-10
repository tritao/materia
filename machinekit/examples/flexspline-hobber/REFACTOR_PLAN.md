# Hobber refactor plan

**Goal.** The same machine and the same scene, built from code that is easy to change: one frame, one layout source, the
machine composed from subassemblies with named interfaces, one class per real part, and a tidy example directory. Pure
restructuring: no design change. Status: plan, 2026-11-10.

**Start after** the linear-module branch (Y module v2, ground bars, Z bellows) has landed on main: it touches the same files.

## Gate for every step

- The hobber scene (`haxeon.enclosure.json`, `build/hobber/hobber.mtrg`) is **byte-identical** to the commit before the step
  (the open-time cache made the export deterministic: one digest, cache on or off). Any intended difference is a separate,
  explained commit, never mixed into a refactor step.
- Suites: `machinekit/tests/hobber`, `machinekit/tests/hobbing`, `haxeon.joint-fasteners`, `haxeon.assembly-integrity`,
  `HobberEnclosureChecks`, the linear-module checks, and the casting and machined packages regenerate unchanged.
- One commit per step; delete what a step replaces (no shims, no aliases).

## Today

- `FlexsplineHobber.hx` (~770 lines) builds everything in one constructor: sections marked with `group(...)`, a hand-kept
  `zero` pose map, and `fixAt`/`fixExisting` mates between any two parts.
- Two frames: placement mixes the machine frame and the work frame (`MACHINE_FROM_WORK`); a free-standing part placed with a
  work-frame height ended up 637 mm under the floor (fixed in 1715adaf7).
- Layout spread over about eight static classes (`FoundationLayout`, `HobberXLayout`, `HobberZLayout`, `HobberYLayout`,
  `HobberSwivelLayout`, `HobberOutboardLayout`, `HobberZOffset`, `HobberRailPlacement`) with implicit cross-dependencies.
- `HobberSaddle` holds about twenty unrelated machined parts (carriage, saddles, pads, seats, cradle, electrical panel, the
  enclosure envelope) selected by a design-id string in one `switch`.
- `machinekit.hobbing` mixes reusable hobbing parts with this machine's layouts and plates.
- The example directory mixes the machine, scene entry points, checks, one-off studies (some stale: `HobberTrayMountingStudy`
  does not compile), procedures, docs and 12 manifests.
- `HobberEnclosurePreview` fits the enclosure, assembles, exports and adds motion, navigation and inspection views in one place,
  and the fit builds casting solids for their bounding boxes.

## Steps

- **RF1. One frame.** All placement in the machine frame (floor at z 0). The work frame lives only in hobbing kinematics and
  cutting checks, behind one conversion. Remove the remaining work-frame placement paths.
- **RF2. One layout record.** `HobberLayout`, derived once: datums and the height stack from `ALIGNMENT.md` (feet, X rail plane,
  X carriage, column sole, Z rails, swivel, Y module, hob centre, table). Parts read positions from it; the eight layout classes
  collapse into it (constants that are really part dimensions move to their parts).
- **RF3. Subassemblies with named interfaces.** Foundation (bed, column, feet), table and work, X carriage, Z saddle, A swivel
  head, Y module (already a component), B spindle, electrical rack: each its own `MachineAssembly` exposing named connectors
  (column sole, Z rail seats, eRob output, spindle seat, ...). `FlexsplineHobber` becomes a short composition with
  `includeAsGroup` and mates at those connectors; groups follow from the structure; the `zero` map and ad-hoc cross-part mates
  go. Each subassembly gets a small preview like `examples/linear-module`.
- **RF4. One class per part.** Split `HobberSaddle` into a part class (or recipe family) per real part, each owning its geometry,
  faces, fasteners and manufacturing route (as `HobberCarriageRib` and `HobberYPart` started). Part numbers and the machined
  RFQ package must not change.
- **RF5. Generic vs hobber.** Keep reusable parts (`Hob`, `HobSolid`, `HobMesh`, `HobSpindle`, `RotaryTable`,
  `ExpandingMandrel`, ...) in `machinekit.hobbing`; move this machine's layout and plates to a `hobber` package beside the
  example.
- **RF6. Example directory.** `machine/`, `scenes/`, `checks/`, `studies/`, `docs/`; fix or delete stale studies; fewer
  manifests via project variants; machine-level checks consolidated under `machinekit/tests/hobber`.
- **RF7. Thin scene entry.** Three explicit functions: machine, cell (machine + enclosure), scene (export + motion +
  presentation). The enclosure fit uses declared envelopes, not built solids.

Order: RF1, RF2, RF3, RF4, then RF5/RF6, then RF7.

## Log
