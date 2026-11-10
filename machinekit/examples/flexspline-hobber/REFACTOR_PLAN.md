# Hobber refactor plan

**Goal.** The same machine and the same scene, built from code that is easy to change: one frame, one layout source, the
machine composed from subassemblies with named interfaces, one class per real part, and a tidy example directory. Pure
restructuring: no design change. Status: plan, 2026-11-10.

**Start after** the linear-module branch (Y module v2, ground bars, Z bellows) has landed on main: it touches the same files.

## Gate for every step

- All suites named in the prompt pass; the machine keeps the same member IDs, groups, part numbers, BOM lines and derived fastener counts.
- Floating-point round-off in poses, connectors, holes and meshes is allowed; no design changes. One commit per step; delete replacements, no shims or aliases.

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

## Recorded baseline

At `a825f2976` on `hobber-refactor`, with haxeon at the main pin
`2a09270ea4fd4b5e88aa3b349cc53bc2edacc7c1`, the enclosure manifest produced
`f6e54e3520d9b493ba5063555c0f7e4d2cd72881a0c80520aa2afb45cd864484` (SHA-256).
A second build after restoring the original source reproduced that digest exactly.

## Log

- RF1 — Complete: all assembly rest poses, mates, covers and service directions use the machine frame; `HobberFrame` owns native-part mounting and the process conversion. Inventory unchanged (421 members, 154 BOM lines, 328 screws, 42 pins); all six suites PASS (hobber 3303, hobbing 1185, linear module 230; joint fasteners, assembly integrity, enclosure checks). Scene IDs/groups unchanged (544 occurrences including enclosure). `FlexsplineHobber.hx`: 762 lines; `HobberFrame.hx`: 35. Earlier byte-only diagnostics superseded by the relaxed gate.

- RF2 — Complete: one lazy `HobberLayout` replaces all eight layout classes plus bellows/electrical placement; the shared height stack feeds guides, carriage, column, swivel, Y module, hob and table. Intrinsic dimensions move to the bed, column, saddle, outboard and arbor classes. All six suites PASS; inventory unchanged (421 members, 154 BOM lines, 328 screws, 42 pins). `FlexsplineHobber.hx`: 764 lines; `HobberLayout.hx`: 1300. No open RF2 items.

- RF3 — Complete: eight `HobberUnit` assemblies own foundation, table/work, X, Z, A, Y, B and electrical parts; named interfaces compose through `includeAsGroup` while preserving physical IDs. `MachineAssembly.constructionPose` replaces the zero map; controls and service paths have dedicated owners. All six suites PASS (hobber 3303, hobbing 1185, linear 230; joint fasteners, assembly integrity, enclosure 371/48/66); inventory unchanged (421/154/328/42). Scene and isolated-unit preview build. Lines: FlexsplineHobber 117; HobberUnit 165; Foundation 80; TableWork 152; XAxis 66; ZAxis 130; AHead 27; YStage 28; BSpindle 73; ElectricalRack 49; Controls 32; ServiceAccess 95; UnitPreview 21. No open RF3 items.

- RF4 — Complete: `HobberSaddle` is deleted; twelve concrete parts/families own their geometry, faces, holes and manufacturing routes, with shared catalogue identity in `HobberMadePart` and primitive operations in `HobberPartGeometry`. All six suites PASS; inventory unchanged (421/154/328/42), scene digest unchanged from RF3, machined package JSON (44 parts), tooling CSV and bed casting JSON byte-identical. Lines: FlexsplineHobber 117; MadePart 114; PartGeometry 85; XCarriage 94; ZSaddle 106; DriveBar 105; ScaleSeat 46; ZScaleArm 40; ChipTray 47; EnclosurePanel 22; CoolantPort 22; SpindleSeat 102; MotorCradle 245; OutboardCarrier 54; ElectricalPanel 31. Open: the unchanged column exporter fails before and after at `cut-x-145` (ambiguous section junction); no casting design change included.

- RF5 — Complete: 32 machine-specific layout, foundation, made-part and FE/budget files move from `machinekit.hobbing` to the example's `hobber` package (5403 lines including imports); reusable hobbing components stay in the library. `HobberComponents` owns application registration, and saved-machine callers use its registry; the library has no example imports or source-root dependency. All six suites PASS; inventory unchanged (421/154/328/42), scene digest unchanged, machined JSON/tooling CSV and bed casting JSON byte-identical to RF4. Lines: FlexsplineHobber 119; HobberComponents 23; HobberLayout 1302; HobberFrame 37; other unit/part splits change only imports. Open: the same baseline column-export section error remains.
