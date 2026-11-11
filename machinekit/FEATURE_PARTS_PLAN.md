# Feature parts plan (FP0–FP9)

Every modelled part becomes a **recorded feature document in its own
frame**, placed from an **explicit machine skeleton**. Code stays the only
source of a machine; the editor gets a real feature tree, named design intent,
persistent face identity for drawings/CAM/inspection, and (last) parameter
overrides.

**End state: one modelling API, and it records.** `DocumentBuilder` (passed
explicitly, never ambient) is how parts are authored; `cadkit.modeling.Part`
remains only as the evaluated result type and as kernel helpers inside feature
evaluators. The immediate `Part`/`Solids`/`PosedParts` authoring API is deleted
when its last part moves.

## Why

The 2026-11-11 review of how machines are written (flexspline hobber as the
sample):

- **What works and stays.** Dimensions are derived from engineering rules
  (`HobberXCarriage.xCarriageHalfWidth()` sizes the plate from the guide
  block's bolt spacing, edge rule and stock increment), parts carry
  connectors, BOM lines, mass basis, routes and bolted faces, checks run
  on every build, and a machine is a readable diff. A Fusion/FreeCAD-style
  feature tree as source of truth would lose all of that and bring back hidden
  dependencies and fragile references.
- **What doesn't.**
  1. Geometry carries no intent: 72 `HobberPartGeometry.box(x0,x1,y0,y1,z0,z1)`
     calls in `machine/hobber/`, i.e. six unexplained numbers per solid.
  2. Parts are authored in one shared "native" frame (`HobberFrame.PART_DATUM_Z`,
     `partAt(0,0,PART_DATUM_Z)`), so a part's shape is also its position in the
     machine; nothing is reusable or readable in isolation.
  3. `HobberLayout` is a lazy global (53 public fields, `HobberLayout.get()`
     in 47 files): every part reaches into it, so dependencies are invisible.
  4. The editor can't show what a part is made of, only the final solid.

## What already exists

- `cadkit.parametric.recording.DocumentBuilder`: code that records a
  serializable feature document (named dimensions, sketches, extrude, pocket,
  face-placed holes incl. counterbore/countersink, patterns, booleans,
  fillets, groups and labels). Used today only by `cadkit/examples/modeling`
  and the pendant.
- The pendant (`machinekit.pendant.PendantDocuments`, `PendantCasePlate`)
  already ships recorded documents as scene `histories` plus a
  `SceneSourceMap` of top-level steps; the editor (`app.editor.PartHistories`,
  `TimelineModel`) shows the rows, highlights each step's faces and rolls a
  part back. That wiring is pendant-specific.
- Definitions/instances with input **overrides**, and `MachineKitRecipes`
  exposing every `ComponentType` as an editable recipe.

So this plan is mostly **adoption and generalisation**, not a new CAD system.

## Two APIs today, one at the end

| | Immediate (`Part`, `Solids`, `PosedParts`) | Recorded (`DocumentBuilder`) |
|---|---|---|
| A call | runs OCCT now, returns a solid | adds a feature node with parameters |
| Result | a shape | a feature graph + the shape it evaluates to |
| Knows its steps / which step made a face | no | yes (timeline, topological names) |
| Editable without code / saved | no | yes (`DocumentCodec`, recompute) |
| Users | all of MachineKit and the machines | CadKit examples, pendant |

Code reruns either way, so the document is not needed for regeneration. It is
needed for intent (tree, face highlights), for **stable references** that
drawings, CAM setups, mates and inspection points must find again, and for
editor overrides. Keeping both APIs means two caching paths, two face-identity
stories and parts that silently lack history; an ambient recorder under `Part`
(build123d style) would be a hidden global again and would record the
coordinate boxes rather than intent. Hence: one explicit recording API.

## Principles

- **Code is the source.** The editor never authors new topology on a machine
  part. Free-form feature authoring stays for one-off parts (the sketcher).
- **Derivation stays in code; geometry is recorded.** A derived value enters
  the document as a named dimension whose value code computed. It is shown
  as derived (with its rule), not as a free number.
- **Each part in its own frame**, at a datum that means something for that part
  (a plate's seat face, a bar's bore axis). The skeleton says where that datum
  sits in the machine.
- **No hidden globals.** The skeleton is a value passed in; a recipe that
  builds a part without its machine passes the reference skeleton explicitly.
- **Behaviour-preserving until the editor phase.** FP3–FP6 change structure,
  not the machine: the solids, holes, BOM and checks must match the baseline.
- **No escape hatch.** An operation with no feature type gets one; the
  builder's `unsupported(...)` is a test hook, not an authoring path.
- **No ambient builder.** The builder is an argument, like the skeleton.
- No back-compat layers: old helpers are deleted when their last user goes.

## Phases

### FP0 — Baseline and gap audit

- Snapshot every hobber made part (the 12 `HobberMadePart`s, `HobberCarriageRib`,
  `HobberChipCover`, `HobberYPart`, `HobberZBellowsPart`, both mineral castings):
  machine-frame volume, bounds, face count, mass, route holes, fastening faces,
  BOM line. Store as a checked-in fixture plus a comparison test
  (`FeaturePartEquivalence`).
- Record the current hobber open time (cold and warm, open-time cache on) and
  check-suite wall time, so later phases have a budget.
- Audit `HobberPartGeometry` / `Solids` operations against `DocumentBuilder`:
  box from bounds, `cylinderSpan`/`boreX`/`boreY`, `prism`/`radialPlate`,
  `couplingWindow`, `holeTool` along an arbitrary direction, `withHoles`
  over a `RouteHole` list, many-tool `Solids.cut`/`union`. List each gap.

**Gate:** equivalence test green on today's code; gap list written into this file.

**Results (2026-11-11, branch `feature-parts`, from main 0fa7f2c16):**

- `tests/hobber/haxeon.feature-parts.json` → `FeaturePartEquivalence`;
  fixture `tests/hobber/fixtures/feature-parts-baseline.tsv` (`--write` to
  regenerate). 1188 records: 421 member poses, 121 designed parts (15 made,
  2 castings, 31 cast inserts, 4 ribs, 7 chip covers, 2 Y parts, 60 Z bellows
  sections), 462 route holes, 156 fastening faces, 28 pinned faces. All in
  the machine frame. Compare mode is repeatable (0 differences, 0 topology
  notes). Spot checks: X carriage 21.43 kg, bed casting 1171.17 kg, column
  72.83 kg (match the casting package).
- Budget: `new FlexsplineHobber()` 23–39 s depending on machine load (open-time
  cache warm), snapshot 4–10 s, whole compare run ~51 s incl. compile.
  Hobber check suite at the start: 3303 assertions pass, 18m55s wall incl.
  compile under shared load. Editor open time is not measured here; FP1f adds the per-part timing line
  that the suite checks.
- Gap audit (detail in FP1):
  - builder: box from bounds, cylinder on an axis, frame placement, n-ary
    booleans (kernel already has `cad_cut_many`), holes by frame with fit;
  - `DocumentCodec` has a closed list of feature types: domain features
    (castings, joint holes) need a registry (FP1a);
  - castings already have a named, serialised feature model
    (`materia.casting.CastingFeature`: lightening, service opening,
    interface/insert recess, clearance; Cut box/cylinder, Prism, Insert):
    one registered step each, not boxes (D3);
  - holes have three descriptions (`RouteHole`, `BoltedFace`/`PinnedFace`,
    `HoleFeature`) (FP1d); parameters have two systems (FP1c);
  - size of the full migration: about 2,600 immediate-API calls in about 420
    files outside CadKit (MachineKit library ~1,950 / 304 files, MachineKit
    examples ~457 / 70, machines ~185 / 44). FP8/FP9 are a long tail.

### FP1 — Foundations

Make the base layers right before any part moves, because every later part is
written against them. Order: FP1a → FP1b → FP1c; FP1d–FP1g in parallel after
FP1a. Not CadKit-only: FP1c and FP1d change MachineKit (`ComponentType`,
`BoltedFace`, `ManufacturingRoute`).

**FP1a — Open feature-type registry.** Done: `FeatureTypeRegistry` +
`FeatureCodec`; a registered type saves as `{id, type, data, references}`
with the common fields; unknown types are rejected, naming them. Sheet metal
moved onto it (`cadkit.sheetmetal.SheetFeatureTypes`, registered by CadKit on
first use); its payload moved from `sheet` to `data`, so documents are now
schema v17 (v16 rejected). SheetMetalSmoke 7643 assertions, unchanged.
The unlanded sheet branches (`sheet-metal`, `sheet-metal-sm13-integration`)
now need this when they are rebased (decision deferred 2026-11-11).

**FP1b — Frames in documents.** Named datum frames already existed
(`ReferencePlaneElement`: a full `Plane`, saved, undoable, a datum
dependency that invalidates downstream features when it moves). Added
`PlaceFeature`: one step that moves a solid from its own frame into a frame,
optionally relative to a datum; origin x/y/z are length parameters bindable
to named dimensions, orientation is exact x-direction and normal vectors (no
angles: placements are mostly axis permutations, and an angle round trip
would break exact equivalence). Builder: `datum(name, plane)`,
`place(source, x, y, z, xDirection, normal, ?datum)`, `placeIn(source,
plane, ?datum)`; `Dimension` also takes `Int` literals. `PosedParts.moved`
maps 1:1 (same `Location` placement operation). Deferred: datum frames whose
own origin is bound to named parameters (needs element-owned parameters);
not needed while skeleton inputs live in code (D2), needed for FP7.

**FP1c — One parameter system.** `ComponentType` recipes
(`ComponentParameter`/`ComponentValues`) and documents (`NamedParameter`) are
two systems bridged by `MachineKitRecipes`. A recorded part's recipe inputs
*are* its document's named parameters. Each is tagged **editable**,
**derived** (with its rule as text) or **fixed** (stock, catalog) and carries
its `ValueBasis`. FP7 reads only these tags.

**FP1d — One hole description.** Done (first half): `cadkit.parametric.HoleSpec`
describes the hole, not the tool: mouth at x/y on a face frame, depth,
through, kind, thread, fit, nominal, a stable name (`J3.h2.clearance`) and an
optional head-side recess (counterbore) with its own name. `HolesFeature`
owns the cutter rule (1 mm lead-in, 1 mm run-out past a through exit).
`MachinedCut` is gone; joints derive `HoleSpec`s, and
`MachineComponent.machined` cuts with `HolesFeature.tools`, the tools a
recorded `holes` feature cuts. Geometry unchanged; the hole table no longer
counts the run-out on 28 tapped through holes of the hobber (16 -> 15). Builder: `holes(target, specs)`. Left for FP2: the
route table (`RouteHole`, `derivedRouteHoles`), drawings and
`MachinedPackageExport` read the specs back from the part's document;
slots join `HoleSpec` when a recorded part first needs one.

**FP1h — Clean-ups found in review (2026-11-11).**
- One feature vocabulary: retire `box`, `cylinder`, `transform` and
  `rotation` in favour of `box-between`, `cylinder-along` and `place`
  (editor and example uses included; old documents are not kept), and move
  the remaining built-ins out of `DocumentCodec`'s branch chain onto
  `FeatureCodec`s in `CadKitFeatureTypes`, so every type is one class and
  one codec.
  Done 2026-11-11 (first half): `box`, `cylinder` and `transform` retired,
  each an exact subset of `block` (minimum-corner), `cylinder-along` (+Z)
  and `place` (no turn); the builder keeps `box`/`cylinder`/`translate` as
  shortcuts that record those, and the features have matching factories
  (`BlockFeature.corner`, `CylinderAlongFeature.upZ`, `PlaceFeature.offset`).
  `rotation` stays: its editable angle is not something `place` (exact
  vectors) does. Documents are schema v18. Face names are unchanged (same
  primitives), so the naming goldens hold. Built-ins onto codecs, first
part: wire, polyline, loft, sweep, offset, shell, project, grid, linear and
polar patterns and hole are registry codecs (`ModelingFeatureTypes`); their
payload is under `data` like every registered type. Second part: the rest
(sketches incl. constrained, extrude, pocket, revolve, fillet, chamfer, face,
mirror, rotation, boolean, datum and level features, definition output, tool
collection, imported STEP) are `SolidFeatureTypes` codecs, so `DocumentCodec`
has no type branches and no notion of built-in types: every type, CadKit's
own included, is one class and one registered codec. Done.
- BOM: one helper for the "Assumed made part" engineering metadata the
  hobber parts repeat (done: `EngineeringMetadata.assumed(source)`, 88 uses
  across MachineKit, its examples and the hobber); `ScrewLinearModulePart`'s stock and route as a
  function of its recipe values, so its requirements do not build a part
  (done: `routeFor(spec, role)`).

**FP1e — Placed primitives and n-ary booleans.** From the hobber count:

| Hobber uses | ~Count | Builder today | FP1e adds |
|---|---|---|---|
| `box(x0,x1,y0,y1,z0,z1)`, `Part.box` | 45 | `box(w,d,h)` + `translate` | **box from bounds**, one feature, named extents |
| `cylinderSpan`, `cylinderAlong*`, `boreX/Y` | 18 | `cylinder` + `translate`/`rotate` | **cylinder between two stations on an axis** |
| `Solids.cut(body,[tools])`, `Solids.union([...])` | 33 | binary booleans; `ToolCollectionFeature` in codec only | **cut/union over a labelled tool collection**, one step, one `cad_cut_many` |
| `prism`/`radialPlate`, `revolve` | 4 | `polyline`+`extrude`, `revolve` | nothing |

**FP1f — Evaluation cost.** A recorded part's result cached in the open-time
cache keyed by its encoded document; per-feature results reused within a
recompute when inputs are unchanged; one timing line per part, so the FP0
open-time budget is checked by the suite, not by hand.

**FP1g — Face-naming contract.** Every feature type (built-in and registered)
declares stable names for the faces it creates (`front-window.floor`,
`blockL1.h3.bore`). Recorded parts fail the naming audit on unnamed faces
(today 34 weak recipes). This is what drawings, CAM setups and inspection
points reference.

**Gate:** CadKit and MachineKit suites; every new feature and registered type
round-trips through `DocumentCodec` and recomputes after a dimension change;
sheet metal unchanged on the registry; pendant and hobber equivalence (FP0)
unchanged.

### FP2 — Recording on `MachineComponent`

Done (first half, 2026-11-11): recording is part of the component contract,
not a subclass or a facet. `MachineComponent.record(builder, detail)`
returns the part's body (null = not recorded yet); `geometry()`,
`machinable()` and `hasGeometry()` come from it, the derived holes are a
last `Fasteners` step (`PartRecordings`), and the recipe's numeric inputs
must be the document's editable named parameters (and the other way round).
`AssemblyPreview.scene` attaches every recorded part's history and one
source row per top-level step with its faces. Any existing class
(`HobberMadePart`, castings, catalogue parts) adopts it by overriding one
method. `PartRecordingTests` covers it.

Done since: caching (FP1f: solid, history and datums per recording key);
the pendant records its plate, shell and board (PanelSolid recorder,
PanelPartSource), and its parallel immediate-API case builders and
PendantDocuments are gone. Left: one history per shared definition rather
than per first occurrence.
Until FP9 a component either records or overrides `geometry()`; at FP9 the
`geometry()` override point goes and `record` is the only way to make
geometry (enforcement needs that rename, not reflection).

### FP3 — Hobber skeleton

Done (structural half, 2026-11-11): `HobberLayout` is `HobberSkeleton`, its
global accessor `reference()` (the reference machine's skeleton).
`FlexsplineHobber` takes an optional skeleton (`skeleton`, reference by
default) and passes it to every `HobberUnit`, which no longer fetches the
global; the units use their own. Equivalence unchanged.

Per part, not up front: about 170 `reference()` calls remain in part
classes' static helpers and tests. Each part takes the skeleton explicitly
(and its mount frame from it) when it is recorded in its own frame (FP4,
FP5), because recording rewrites those helpers anyway; converting them now
would mean rewriting every part twice. Gate moved to the end of FP5: zero
`HobberSkeleton.reference()` outside `FlexsplineHobber` and recipe
construction.

### FP4 — Pilot: `HobberXCarriage`

Done 2026-11-11 (waits for the review point below):
- Records in its own frame, `HobberSkeleton.xCarriageSeat()` (deck
  underside, centred between the rails and along the deck); the skeleton
  gained `xDeckFront` (the bare 60). Steps: Stock group (plate.length
  derived from the skeleton, plate.halfWidth derived from the block bolt
  rule, plate.thickness fixed by 20 mm ground stock, front/rear/right as
  expressions; "Ground plate"), "Front window" (one cut-many step), and
  "Fasteners · Derived holes" from the joints.
- Its own recipe (`machinekit.hobbing.x-carriage`, no inputs: every
  dimension is skeleton or stock), part number unchanged; `HobberMadePart`
  takes `ownRecipe` so it carries none of the shared recipe's values, and
  defers to the recording for geometry.
- Its bolted and pinned faces are still worked out by the shared helpers in
  the part datum frame and moved into its own frame (`movedBy`); those
  helpers become skeleton-frame functions in FP5.
- Equivalence: only the carriage's own pose changed (its origin moved to the
  seat); its solid, mass, holes, faces and BOM line, and every other part,
  match in the machine frame (inertia round-off ~1e-12 on parts placed
  through it).
- Interfaces from the recording: the carriage declares datum frames in
  `record` (four guide-block lands, the column-sole pads) and builds those
  bolted faces from them (`MachineComponent.recordedDatums`); the nut-block,
  rib, chip-cover and bellows faces come from other parts' helpers and keep
  `movedBy` until those parts are recorded.
- Recordings are cached: solid, history (with step rows) and datums per
  `recordingKey` (recipe key, material, detail, machining and the part's
  `recordingContext`, e.g. its skeleton).
- The equivalence test also asserts the carriage's history steps and their
  faces (a silent override mismatch once left it unrecorded).
- Not yet seen in the editor (the review step below).

**Review point:** you look at the new `HobberXCarriage.hx` and the editor tree
and decide go/no-go for FP5. If it doesn't read better, stop here and keep only
FP1–FP3.

### FP5 — Roll out across the hobber

Pattern per part (from the pilot): its own recipe (inputs = its own design
choices; layout- or stock-given values recorded as derived/fixed), recorded
in its own frame at a frame the layout gives, datums for its interfaces, and
faces whose holes and roles come from the shared helpers placed with
`BoltedFace.at(datum)` / `PinnedFace.at(datum)`. `HobberRecording` bridges
layout shapes still given in the part datum frame. Gate per part: only its
own pose changes in the equivalence, and its history steps are asserted.

Done 2026-11-11: group 1 (drive bars, scale seats, Z scale arm, coolant
port, carriage ribs). Timeline: a union owns its parts as sub-steps.
Group 2: chip tray (inputs = the table it fits; CadKit `polygon` profile),
electrical panel (cut-plate route added); the unused GE-enclosure panel
removed (shared recipe v7). The X chip covers (sides, lids, skirt) record
as CadKit sheet metal: a mild-steel 2 mm sheet base from a fixed polygon
sketch, edge flanges with R2 bends; tool ports and the skirt's nut
clearance are sheet cuts in the blank (so the flat pattern carries them),
and the skirt's far lip is three flanges either side of its two notches
(the old box cut left an unmakeable sliver of bend there: −118 mm³, the
only geometry change). Everything else matches the old solids exactly.
CadKit gained named edge selection for flanges and hems (wall + outline
identity, `DocumentBuilder.sheetFlange`, saved as `edges`) so recording
code never picks faces, and `DocumentBuilder.polygonSketch` /
`circlesSketch`. Joint holes too are cut in the blank: CadKit `sheet-holes`
takes the same `HoleSpec`s and finds, at evaluation, each wall a hole
crosses square (a hole stopping inside a wall is an error); the shared
derived-holes step uses it whenever a part's body is sheet metal.
Group 3 and 4: outboard carrier, spindle seat, Y module base and carriage,
B motor cradle, Z saddle. Along the way: CadKit `cut-all` is its own
feature on the native multi-tool cut (a boolean against a compound of
overlapping tools silently cut nothing); the timeline credits a face to the
top-level step containing its creator (nested sub-steps); the eRob lists
its mounting cutout as named cylinders and can record them in a host
(`recordMountingCutout`); the skeleton's sweptDeckRelief and the parts'
static geometry builders are removed as their parts record them.

Step 5 done: mineral castings record one step per casting feature
(MachineKit registers casting-body, casting-cavity, casting-anchors,
casting-receivers with CadKit), shared by every casting in MachineKit.

Z bellows done: MachineKit's `WayCoverSection` records (plate, skins, wiper
pad and slide lips as rings; flanges get the shared derived-holes step), so
every way cover in every machine records; the hobber's custom flanges add a
"Trimmed to host face" step. `PartRecordings.recipeInputs` declares a part's
numeric recipe inputs as its editable parameters for parts whose shape comes
from a plan. Exact equivalence (60 sections).

Left in FP5: faces some parts still take from
other parts' helpers in the part datum frame (the carriage's nut-block, rib,
cover and bellows receivers) move to datums as those helpers are reworked.

In order, simplest first, one commit per part, each gated by FP0 equivalence:

1. drive bars, scale seats, Z scale arm, coolant port, carriage ribs;
2. chip tray, chip covers, enclosure panel, electrical panel;
3. spindle seat, motor cradle, outboard carrier, Y parts, Z bellows parts;
4. Z saddle;
5. mineral bed and column: each `CastingFeature` (lightening, service
   opening, interface/insert recess, clearance) becomes one registered step
   (FP1a) evaluated by the existing casting geometry, in `MineralFoundation`,
   so every casting in MachineKit gains a tree at once.

### FP6 — Delete the hobber native frame; freeze the immediate API

- Once nothing uses them: `HobberFrame.partAt/sourcePose/PART_DATUM_Z`, the
  coordinate-box helpers in `HobberPartGeometry`, `HobberPartGeometry.shifted`.
- The immediate authoring API is **frozen**: no new users. A source check
  (test or lint) lists remaining `Part.box`/`Solids.*`/`PosedParts.moved`
  call sites outside CadKit evaluators; the list may only shrink.
- Update `machinekit/README.md`, `docs/concepts/modelling/cad-documents.md`
  and `machine-components.md` ("In Materia" tables) and add a short
  "writing a part" guide.

### FP7 — Editor parameter overrides

- A recorded dimension is **editable** (an input with a range), **derived**
  (read-only, shows its rule) or **fixed** (stock, catalog).
- Editing an editable dimension writes an instance override into the project
  file; the part restages through the existing definition/instance path, and
  a failed recompute or a failed check shows the error and keeps the
  previous geometry.
- Skeleton inputs are editable the same way and restage the whole machine.

**Gate:** edit carriage thickness and a skeleton rail centre in the editor;
undo/redo; reopen the project and the overrides are there; invalid input is
rejected with the feature named.

### FP8 — MachineKit library and other machines

- MachineKit's own parts (catalog components, shared bases, structural,
  motion, covers) move to `RecordedPart`, base classes first so their
  subclasses follow. Catalog parts get trees and named faces too.
- Machines move when next worked on (sheet router, lathe, gantries, cnc
  router), not as a sweep. New machines and parts start recorded.
- Each move shrinks the FP6 call-site list.

### FP9 — Delete the immediate authoring API

When the FP6 list is empty: remove the authoring surface of
`cadkit.modeling` (`Part` constructors/booleans used for authoring,
`Solids`, `PosedParts.moved` for geometry, `BuildPart`/`BuildSketch` if
unused), keeping `Part` as the evaluated shape type and the kernel helpers
feature evaluators call. Update the CadKit docs and examples.

## Open decisions

- **D1. Overrides at all?** FP7 lets the editor change values that otherwise
  live in code, so the file plus overrides becomes the source. Alternative: the
  editor only shows the tree and every change goes through code.
  Recommendation: do FP0–FP6, then decide.
- **D2. Where skeleton inputs live:** Haxe constants (recommended; derivation
  next to them) or a data file the editor writes.
- **D3. Castings:** settled by FP0: castings already have a named feature
  model (`materia.casting.CastingFeature`); each becomes one registered
  recorded step (FP1a), not re-expressed as boxes. In scope for FP5.

## Status

- 2026-11-11: plan written; reframed the same day to one recording API
  (FP1 gap table, FP2 as the MachineKit standard, FP6 freeze, FP8 library,
  FP9 deletion). Nothing started.
- 2026-11-11: FP1 rewritten as foundations FP1a–FP1g; D3 settled.
- 2026-11-11: FP0 done on branch `feature-parts` (worktree
  /media/joao/DATA1/materia-feature-parts): equivalence test + fixture.
- 2026-11-11: FP1d first half (HoleSpec, holes feature, MachinedCut alias);
  FP1e done (box-between, cylinder-along, cutAll, unionAll); FP1c CadKit half
  (parameter roles). Known on main: MachineAssemblyDescriptionTests fails
  with "Group path must not be empty".
- 2026-11-11: FP1a done (registry, sheet metal on it, document v17); FP1b
  done (`PlaceFeature`, builder datum/place). Pre-existing failures seen on
  main, not caused here: CadKit AssemblyNestingSmoke and
  AssemblyDocumentsSmoke; MachineKitSmoke stops in BomExampleTests on
  HOBBER-X-RIB-LEFTOUTER (HobberCarriageRib has no engineering maturity).
