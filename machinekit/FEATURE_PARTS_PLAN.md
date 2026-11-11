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

### FP1 — CadKit builder gaps

Close the gaps in `DocumentBuilder` (each a serializable feature with codec
and recompute tests in CadKit, no MachineKit knowledge). From the 2026-11-11
count of hobber operations against the builder:

| Hobber uses | ~Count | Builder today | FP1 adds |
|---|---|---|---|
| `box(x0,x1,y0,y1,z0,z1)`, `Part.box` | 45 | `box(w,d,h)` + `translate` | **placed box from bounds**, one feature, named extents |
| `cylinderSpan`, `cylinderAlong*`, `boreX/Y` | 18 | `cylinder` + `translate`/`rotate` | **cylinder between two stations along an axis** |
| `PosedParts.moved(part, frame)` | 18 | translation, or one axis-angle rotation | **frame placement** (origin + rotation stored in the document) |
| `Solids.cut(body,[tools])`, `Solids.union([...])` | 33 | binary booleans only; `ToolCollectionFeature` in the codec but not the builder | **n-ary cut/union over a labelled tool collection**, one step and one OCCT boolean |
| `withHoles`/`holeTool` from `RouteHole` | most holes | `HoleFeature` on a selected face at 2D x/y; has thread | **hole at origin + axis** (plain/tapped/counterbore/dowel) resolving its mouth face by a **face-by-plane selection recipe**; **fit/tolerance** on the hole or a reference to the route hole id |
| `prism`/`radialPlate`, `revolve` | 4 | `polyline`+`extrude`, `revolve` | nothing |

Not yet sized (FP0 decides): casting operations in `MineralFoundation`
(`CastingFeature`, inserts, pockets) and the shared bases (`WayCoverSection`,
`NutBlock`, `PanelSolid`, …) that hobber parts extend.

**Gate:** CadKit tests; each new feature round-trips through `DocumentCodec`
and recomputes after a dimension change.

### FP2 — `RecordedPart` in MachineKit

Generalise the pendant pattern into **the** way MachineKit parts are modelled
(not a hobber pattern):

- `machinekit.component.RecordedPart` (a `MachineComponent`): the subclass
  implements `record(builder)`; `geometry()` is the document's output,
  collision pieces and route holes come from the same recording, so faces
  match the history the scene carries.
- Holes come from the part's own `fasteningFaces()`/`pinnedFaces()` through
  one helper (`RecordedHoles.cut(builder, body, faces)`), grouped under
  "Fasteners" in the tree. Today's `derivedRouteHoles()` stays the single
  hole derivation.
- Generic scene export: `MachineAssembly` scene generation emits `histories`
  and `SceneSourceMap` rows for every `RecordedPart` occurrence. The pendant
  moves onto it and `PendantDocuments.histories()/sources()` go.
- Open-time cache keys the recorded part by its encoded document.

**Gate:** pendant scene unchanged (same histories rows and face highlights);
open time within the FP0 budget.

### FP3 — Hobber skeleton

Replace `HobberLayout` with `HobberSkeleton`:

- **Inputs**: today's constructor constants, grouped (travels, rail centres,
  table, Z column, swivel/spindle) with a doc line each.
- **Derived**: the `heights()` stack and the other derived values.
- **Datums**: one named `AssemblyFrame` per part mount (`xCarriage.seat`,
  `zSaddle.seat`, …), replacing `partAt(0,0,PART_DATUM_Z)` and
  `sourcePose(...)` call sites.
- Passed explicitly: `FlexsplineHobber` builds one and hands it to each
  `HobberUnit` and part constructor; `HobberMadePart.create` passes
  `HobberSkeleton.reference()`. No `get()`, no cache singleton.

Pure refactor: parts keep the native frame in this phase.

**Gate:** zero hits for `HobberLayout.get(`; FP0 equivalence identical;
hobber checks green.

### FP4 — Pilot: `HobberXCarriage`

- Own frame: origin at the plate underside, centred between the X rails at the
  carriage's mid-length, +Z up through the plate, +X towards the motor
  (pinned down in the part's doc comment).
- Recorded features with names: `stock plate` (thickness = ground-stock 20,
  half-width = derived from the block bolt rule), `front window`,
  `guide block lands` (4 × `blockFace`), `column sole` holes, `nut block`,
  `rib joints`, `chip cover / bellows receivers`.
- `HobberXAxis` places it at `skeleton.xCarriage.seat`; the guide-block and
  rib poses read the same skeleton datums.

**Gate:**
- machine-frame symmetric difference against the baseline ≤ 1e-3 mm³; route
  holes, fastening faces, mass and BOM line identical;
- editor: the carriage shows the named rows, each highlights its faces, and
  rollback works (checked by hand in the running editor, screenshots in the
  commit or plan);
- open time within budget.

**Review point:** you look at the new `HobberXCarriage.hx` and the editor tree
and decide go/no-go for FP5. If it doesn't read better, stop here and keep only
FP1–FP3.

### FP5 — Roll out across the hobber

In order, simplest first, one commit per part, each gated by FP0 equivalence:

1. drive bars, scale seats, Z scale arm, coolant port, carriage ribs;
2. chip tray, chip covers, enclosure panel, electrical panel;
3. spindle seat, motor cradle, outboard carrier, Y parts, Z bellows parts;
4. Z saddle;
5. mineral bed and column: their casting features (`CastingFeature`, inserts,
   pockets) become recorded steps in `MineralFoundation`, so this is a
   MachineKit change shared with other castings. Scope it separately if FP0
   shows the casting pipeline needs its own feature types.

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
- **D3. Castings in scope for FP5** or a separate plan after FP0 shows the
  size.

## Status

- 2026-11-11: plan written; reframed the same day to one recording API
  (FP1 gap table, FP2 as the MachineKit standard, FP6 freeze, FP8 library,
  FP9 deletion). Nothing started.
