# Topological naming — persistent names for faces, edges and vertices (handoff)

**Goal:** a reference to a face, edge or vertex keeps pointing at the same
element after any edit that does not remove it: parameter changes, features
inserted upstream, sketch dimension edits, and changes to recipe parts. When
the element really is gone or split, the reference says so and lists the
candidates. It never silently picks a different element.

**Why today's references fail.** `TopologyReference.prepareRemap` /
`resolveFor` (and `EditorScene.remapSelectedCadFace`) map the reference's
*previous* element through the producer's *new* operation history. That
history's sources are the new operation's inputs. The previous element is one
of them only when the producer's inputs were not rebuilt. After any upstream
parameter change they were rebuilt, so the reference falls through to
`TopologyFingerprint`. The fingerprint is exact by design (relative size
1e-6, position 1e-8), so it fails whenever the edit moved or resized the
element: a fillet on a box edge is lost when the box gets wider.
`GeometricConnectors` (C4.4) adds a relaxed match (the one face with the same
kind, direction and radius), which works for one bore and fails for two.
`SelectionRecipe` works, but only for intent written down as a query.

History is only meaningful inside one operation. The fix is to consume it
there to carry **names** forward, and to compare names across recomputes.

| Consumer | Holds today | Where |
|---|---|---|
| Fillet / chamfer on selected edges | `TopologyReference` per edge of the source | `parametric/features/FilletFeature.hx`, `ChamferFeature.hx` |
| Sketch on a face | `TopologyReference` (support face) | `parametric/features/ConstrainedSketchFeature.hx` |
| Face profile | fingerprint captured from an index on first evaluation | `parametric/features/FaceFeature.hx` |
| Shell, hole | `SelectionRecipe` (queries; unchanged by this plan) | `ShellFeature.hx`, `HoleFeature.hx` |
| Mates on faces and edges | fingerprint + feature, direction, radius; face descriptors in scene artifacts (C4.5a) | `parametric/GeometricConnectors.hx`, `projectkit/.../SceneArtifact.hx` |
| Editor selection | fingerprint + producer history | `app/src/EditorScene.hx`, `CadModelTopology.hx`, `CadPartModel.hx`, `CadDocumentSession.hx` |

Read first:
1. Code: the files in the table; `parametric/TopologyReference.hx`,
   `TopologyFingerprint.hx`, `TopologyResolver.hx`, `TopologyHistoryMap.hx`,
   `Feature.hx`, `Document.hx` (`recompute`); `sketch/SketchProfile.hx`;
   `modeling/Part.hx`; `core/src/cadkit.cpp` (`collect_operation_history`,
   `evaluate_boolean`, the extrude/revolve/loft/sweep/shell operations).
2. `cadkit/ARCHITECTURE.md` (ABI contract) and `plans/CONSTRAINT_SOLVING.md`
   (C4.4–C4.5: geometric connectors, face descriptors).
3. Background (reference only, nothing to copy):
   - FreeCAD 1.0 topological naming (LGPL): `src/App/ElementMap.cpp`,
     `src/Mod/Part/App/TopoShapeExpansion.cpp` (`makeShapeWithElementMap`),
     and the per-operation mappers in `src/Mod/Part/App/`. Same overall
     shape as this plan. This plan deliberately differs on modified faces
     (TN-D3) and on edges (TN-D4).
   - OCCT's own naming, `TNaming` (vendored under
     `third_party/occt/src/ApplicationFramework/TKCAF/TNaming/`). Its name
     types (generation, intersection, filter by neighbours, modified-until)
     are the same building blocks as below. It is not used (TN-D11).
   - Literature: Capoyleas, Chen, Hoffmann, "Generic naming in generative,
     constraint-based design", CAD 28(1), 1996; Kripac, "A mechanism for
     persistently naming topological entities in history-based parametric
     solid models", CAD 29(2), 1997; Marcheix, Pierra, "A survey of the
     persistent naming problem", SMA 2002.

Work in `../materia-worktrees/topological-naming` on branch
`topological-naming`, stacked on `constraint-solving` (geometric connectors,
document version 10). Rebase onto `constraint-solving` while it is unmerged
(C4.5 is in progress there and touches `GeometricConnectors`; TN4 waits for
it). Once it lands, merge each green stage to `main`. Failing test first, one
commit per item, append to the Progress log, and stop and log whenever this
plan turns out to be wrong.

Environment:
- Core changes need this worktree's own cadkit build, against the shared
  prebuilt OCCT (never rebuild OCCT):
  `CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release cmake -S cadkit -B cadkit/build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug && ninja -C cadkit/build/debug`.
- Haxe suites: `CADKIT_BUILD_ROOT=<worktree>/cadkit/build/debug` and
  `LD_LIBRARY_PATH=<worktree>/cadkit/build/debug/core:<occt prefix>/lib`.
- Submodules: only `haxeon` (with its nested libs, `.tools` and runtime) is
  populated. Populate the others from a local clone at the pinned sha when
  a MachineKit or app suite needs them (TN4).
- ABI changes: regenerate `haxe/abi/CadKit.hxi` with
  `scripts/generate-haxeon-hxi`.

## Decisions

- **TN-D1 — Names belong to shapes and are computed in cadkit-core.** Each
  shape carries an immutable element map, shared by its clones. Every path
  that makes geometry gets names with no per-path work: features, subgraph
  definitions, MachineKit recipes built with `cadkit.modeling.Part`, and the
  editor. Propagation needs every input and output subshape, IsSame maps,
  adjacency and full history. In C++ that is linear with OCCT's maps; across
  the handle ABI it costs one round trip per entry, and today's
  `TopologyHistoryMap` is already references × entries. The core stays free
  of documents: names are text, and tags are opaque strings passed in.
- **TN-D2 — History is consumed inside one operation, never across
  recomputes.** Identity across recomputes is equality of names. The
  cross-recompute use of `TopologyHistoryMap` (in `TopologyReference` and
  `EditorScene`) is deleted, not patched. `Operation` stays as a low-level
  API.
- **TN-D3 — Identity survives modification.** An element that an operation
  leaves unchanged or modifies one-to-one keeps its name. A face trimmed by a
  hole is still the same face. FreeCAD instead appends a marker and the
  feature's tag on every modification, so inserting a feature upstream
  renames the faces downstream references sit on. Only splits, merges and
  collisions change a name (see Naming rules).
- **TN-D4 — Faces are tracked; edges and vertices are derived.**
  - An edge with two or more adjacent faces is named from those faces' names.
  - A vertex is named from the set of its adjacent faces.
  - Only boundary edges (one face), wire edges (no face) and their vertices
    carry tracked names, from starting names and history.
  - Derived names are computed lazily from the final face names, so they
    can never disagree with them.
- **TN-D5 — Tags are stamped after the fact.** One core call,
  `stamp(shape, tag, inputs)`, prefixes `tag:` to every name not found in
  any input.
  - `Document.recompute` calls it after each feature's `evaluate`, with the
    feature's id (`f12`) and its dependency shapes, so feature classes do not
    change.
  - Starting names and generated names get the feature's tag. Names that pass
    through keep their origin's tag.
  - The same call is `Part.named("head")` in the modeling layer, and the
    instance stamp (`i2.0`) on pattern and mirror copies.
- **TN-D6 — Starting names come from how an element was built, never from
  enumeration order.**
  - Primitives are named by role, through the OCCT builder's own face
    accessors.
  - Sketch edges are named by entity id and sketch vertices by point id.
  - Template sketches and polylines are named by authored role or segment.
  - Index names (`#k`) are used only where nothing better exists (STEP
    import, projection), and they are weak (TN-D8).
- **TN-D7 — A name is a structured value with one canonical text form,
  owned by the core.**
  - Haxe treats names as opaque strings.
  - The full text is what gets persisted, never a hash: it stays readable in
    documents and diffs, and the resolution rules (TN-D9) read its structure.
  - The grammar is versioned as the *naming scheme* (TN-D13).
- **TN-D8 — Every name is strong or weak.** A name is weak when it depends on
  enumeration order or geometric ordering: index starting names, ordinals for
  symmetric splits, and input-slot suffixes. A weak name never resolves a
  reference on its own; the fingerprint must agree.
- **TN-D9 — Resolution: names first, fingerprints as tie-breaker and last
  resort.**
  1. An exact name or alias that is unique resolves.
  2. Otherwise, structural relatives (split pieces, the unsplit ancestor),
     tie-broken by fingerprint.
  3. Otherwise, today's exact fingerprint match over the whole shape. This
     keeps a floor: nothing that resolves today stops resolving.

  Never choose among ties: report `Ambiguous` with the candidates.
- **TN-D10 — Mechanics in the core, policy in Haxe.**
  - The core finds the candidates for a name text and grades each match:
    `exact`, `alias`, `relative`, `weak` or `none`.
  - Haxe maps the grades to `ReferenceState` and applies the fingerprint
    tie-break.
  - Matching on texts alone is also a core function, so the editor can match
    scene-artifact face descriptors without a B-rep.
- **TN-D11 — OCCT as shipped; per-operation adapters in the core.**
  - No `TNaming`/OCAF: it needs an OCAF document mirroring the feature graph
    inside the core, against the ABI contract.
  - No OCCT patches: OCCT is vendored at a release and built once as a
    shared prefix (and for the web build), and every gap found so far has an
    OCCT call or a local workaround.
  - A real history bug gets a workaround in the adapter (geometric matching,
    weak names) and an upstream report with a reproducer. Patch the vendored
    OCCT only when no workaround exists.
- **TN-D12 — Every shape-producing entry point names its result, always.**
  There is no unnamed fast path, so names cannot go missing halfway through a
  chain. Plain (non-`_operation`) variants compute the history they need
  internally (booleans fill history unconditionally). Revisit only if the TN2
  benchmark shows a cost that matters to a real caller.
- **TN-D13 — Names are document format.**
  - Stored references carry name and fingerprint. Documents record the
    naming scheme that produced their names.
  - A golden-name corpus test pins the names of a set of models, so a change
    to the naming rules is always deliberate.
  - A rule change bumps the scheme. Old names then resolve through the
    fallbacks (TN-D9) and are recaptured on save, the same path as legacy
    fingerprint-only documents.
- **TN-D14 — Fingerprints and queries stay.** Fingerprints remain as
  tie-breakers, for weak names, for legacy documents and for data-only
  matching. `SelectionRecipe` remains the way to state intent ("the top
  face, whatever it is called"). Recipe-declared connectors
  (`DefinitionConnectorEvaluator`) remain preferred over picked faces for
  authored parts.
- **TN-D15 — Out of scope:** naming solids and shells (later, for body
  picking), references across documents (copy/paste re-ids features), names
  shown verbatim to users (the editor renders friendly labels from them),
  and STEP-carried names before TN6.

## Naming rules

### Text form

```
name   := [tag ":"] body suffix*
body   := role                               starting name:  box.+z  cyl.side  e.l3  p.p2  seg.4
        | role "(" name ("," name)* ")"      generated:      side(f5:e.l3)  fillet(E(f3:box.+x|f3:box.+z))
        | "E(" name "|" name ")"             derived edge, faces sorted
        | "V(" name ("|" name)* ")"          derived vertex, faces sorted
suffix := "{" name ("," name)* "}"           split piece: faces bounding only this piece, sorted
        | "@" n                              collision between inputs: input slot          (weak)
        | "~" n                              ordinal among otherwise identical pieces      (weak)
        | "#" n                              index starting name                           (weak)
tag, role, id := [A-Za-z0-9_.+-]+            other bytes %XX-escaped (sketch ids are free text)
```

Sorting is bytewise on the canonical text. Examples:
- top face of box feature 3: `f3:box.+z`;
- the edge where it meets the +x face: `E(f3:box.+x|f3:box.+z)`;
- the fillet face on that edge from feature 7: `f7:fillet(E(f3:box.+x|f3:box.+z))`;
- a side face of an extrusion (feature 6) of sketch 5's line `l3`:
  `f6:side(f5:e.l3)`;
- the left piece of that top face after slot feature 9 splits it:
  `f3:box.+z{f9:cyl.side}`.

### Starting names

| Construction | Names |
|---|---|
| box | `box.±x/±y/±z` from `BRepPrimAPI_MakeBox`'s face accessors (map each accessor to its axis once, in TN1) |
| cylinder | `cyl.side/top/bottom` from `BRepPrim_Cylinder` (`LateralFace`, `TopFace`, `BottomFace`) |
| sphere | `sphere` |
| line, arc, circle, spline | an edge name passed by the caller (sketch entity `e.<id>`); otherwise weak `curve#0` |
| polyline | `seg.<k>` in authored point order (strong: authored, not enumerated) |
| wire, planar face | input edge names carried through, matched by IsSame, else by shared underlying curve (`BRepBuilderAPI_MakeWire` may copy edges and has no history). The face takes a caller-given name (sketch region `r(<outer loop entity ids>)`) |
| sketch vertex | `p.<point id>` |
| STEP import, projection | `step#k`, `proj#k` (weak); TN6 reads STEP names |

### Through an operation

For each input face (and each tracked edge or vertex), using the adapter's
history:
1. **Unchanged** (the same subshape is in the result): keeps its name.
2. **Modified one-to-one:** keeps its name.
3. **Modified one-to-many (split):** each piece gets
   `parent{faces bounding only this piece}`.
   - Neighbours are compared by their names before any split suffix, so two
     split faces that touch cannot make the rule circular.
   - Pieces still identical after that get `~k` in a deterministic geometric
     order, and are weak.
4. **Many-to-one (merge):** the smallest source name is canonical; the others
   become aliases (lookup finds them).
5. **Generated:** `role(source)`, with the role defined by the adapter
   (table below).
6. **Collision:** two inputs bring the same name (copies of one shape in a
   compound or fuse). Every colliding name gets `@slot`, weak. Pattern and
   mirror features stamp instances first (TN-D5), so this is a safety net.
7. **Leftover:** a result face no rule names is a history gap. It gets a weak
   index name, and it fails the TN2 completeness test for that operation.

### Edges and vertices

- An edge with two faces is named `E(a|b)` from its faces.
  - Seam edges: `E(a|seam)`.
  - Degenerate edges (poles, apexes): `E(a|degenerate)`.
  - When the same pair of faces shares several edges, add the edge's
    vertices' names as a split-style suffix; if that is still tied, `~k`
    (weak).
- A vertex is named `V(...)` from its adjacent faces. Below three faces, add
  its edges.
- Boundary and wire edges and vertices keep tracked names. Extracting a face
  (`subshape`) freezes its edges' derived names as tracked names, because
  they become boundary edges.

### Per-operation adapters (TN2)

| Operation | OCCT history used | Names |
|---|---|---|
| fuse / cut / common | `BRepAlgoAPI` history (`Modified`, `Generated`, `IsDeleted`; unchanged subshapes are shared) | rules 1–4, 6; intersection edges are derived |
| extrude | `BRepPrimAPI_MakePrism`: `Generated(edge)` → side face; `FirstShape(s)` / `LastShape(s)` → caps (not in `Generated`; the current capture misses them); `Generated(vertex)` → lateral edge | `side(e)`, `start(f)`, `end(f)`; wire profiles: tracked boundary edges `start(e)`, `end(e)`, `lateral(v)` |
| revolve | `BRepPrimAPI_MakeRevol`: same, plus `Degenerated()` | as extrude; a full turn has no caps |
| loft | `BRepOffsetAPI_ThruSections`: `GeneratedFace(edge)`, `FirstShape` / `LastShape` | `side(edge of section 0)`, `start`, `end` |
| sweep | `BRepOffsetAPI_MakePipe`: `Generated(spineEdge, profileEdge)` (the one-argument form lumps all faces along the path) | `side(profileEdge,spineEdge)`, `start`, `end` |
| fillet / chamfer | `Generated(edge)` → blend faces; `Generated(vertex)` → corner faces; `Modified(face)` → trimmed faces | `fillet(E…)`, `chamfer(E…)`, `corner(V…)`; rules 1–3 for faces |
| shell | `BRepOffsetAPI_MakeThickSolid` (`Modified`, `Generated`, `IsDeleted`) | kept faces keep names; offset faces `inner(f)`. Confirm OCCT's reporting with the completeness test first |
| wire offset | `BRepOffsetAPI_MakeOffset::Generated(edge)` | tracked `offset(e)` |
| translate / rotate / mirror / place / clone | location moves keep subshape order; `BRepBuilderAPI_Transform` copies report `Modified` one-to-one | names kept |
| compound | inputs' maps | union; rule 6 on collisions |
| subshape | parent's map | restricted; boundary edges frozen |

When one native call runs several OCCT steps, the steps' histories are
chained with `BRepTools_History::Merge` before naming.

### Resolution (TN-D9, TN-D10)

`findElement(kind, name)` returns graded candidates:
- **exact / alias**, unique and strong → `Resolved`.
- **relative:** same name once split suffixes, ordinals and slot suffixes are
  removed.
  - A reference to a split piece picks the piece whose `{…}` set overlaps its
    own best (Jaccard), if that piece is clearly best.
  - A reference to an unsplit face that is now split → `Ambiguous`, listing
    the pieces, unless the fingerprint picks exactly one.
  - Edges compare their faces as relatives.
- **weak:** accepted only if the fingerprint agrees within today's tolerances.
- **none:** today's whole-shape fingerprint match (`Remapped`, reported as
  geometric). Else `Deleted` when the name's tag names a feature that no
  longer exists, and `Unresolved` otherwise.

`TopologyRemapReport` counts how each reference resolved (exact, relative,
weak, geometric), so tests and the editor can tell.

## Layout

```
cadkit/core/src/naming/                new (internal C++)
  element_name.{hpp,cpp}   interned name nodes, canonical text, parse, weak flag
  element_map.{hpp,cpp}    per-shape map: faces + tracked edges/vertices, aliases, lazy derived names
  propagate.{hpp,cpp}      rules 1–7 from a history; stamp; text matching (grades)
  adapters.{hpp,cpp}       per-operation history adapters and starting names
cadkit/core/include/cadkit.h           ABI additions (below)
cadkit/core/src/cadkit.cpp             ShapeEntry gains the map; every producer names its result
cadkit/tests/naming_smoke.cpp          golden names, per-operation completeness
cadkit/haxe/src/cadkit/
  Shape.hx                 elementName(s), findElement, stamped, withElementNames
  ElementMatch.hx          candidates + grade
  modeling/Part.hx         named(tag)
  sketch/SketchProfile.hx  entity/point/region starting names
  parametric/              TopologyReference (name-first), Document (stamp), DocumentCodec (v11),
                           FaceFeature, patterns/mirror (instance stamps), GeometricConnectors
cadkit/haxe/tests/
  NamingRobustnessSmoke.hx TN0 suite (models × edits × references, oracle per reference)
  NamingGoldenSmoke.hx     pinned names for the corpus (TN-D13)
app/src/                   EditorScene, CadModelTopology, CadPartModel: selection by name (TN3)
```

ABI additions (names are UTF-8 text):
- `cad_shape_element_name(shape, kind, index, …)`.
- `cad_shape_copy_element_names_bytes(shape, kind, …)`: all names of a kind
  in one buffer, like the mesh byte copies.
- `cad_shape_find_element(shape, kind, name, …)`: candidates and grades.
- `cad_element_name_match(reference, candidates, …)`: text-only grading for
  descriptors.
- `cad_shape_stamp_names(shape, tag, inputs, count, out_shape)`.
- `cad_shape_seed_names(shape, kind, names, count, out_shape)`.
- `cad_naming_scheme_version()`.

## TN0 — Tests first: robustness suite and baseline

Do:
- `NamingRobustnessSmoke`: each case is a model, a reference, a list of
  edits, and an oracle: a geometric predicate that says which element the
  reference *should* be after each edit (for example "the edge where the top
  face meets the +x face"). Each outcome is classed as **correct**,
  **reported** (`Ambiguous`/`Unresolved`/`Deleted` with the right
  candidates), or **wrong**.
- Cases:
  1. Box, fillet on one edge, chamfer on another; edit width, depth, height.
  2. Plate from a constrained sketch, sketch on its top face, pocket; edit
     thickness and width, and add a hole that does not touch the sketch.
  3. The same plate with a slot that moves from outside the plate to across
     it: the top face splits. Expected: reported (or correct where the
     fingerprint picks the piece), never wrong.
  4. Revolve, fillet on a rim edge; edit profile dimensions.
  5. Linear pattern of bosses, a reference on instance (1,0); edit spacing,
     and count 3 → 4.
  6. A part built with `cadkit.modeling.Part`, connector on one of two
     bores of equal radius; widen the plate, move the bores (relaxed matching
     is ambiguous here today).
  7. Delete the sketch line under a referenced side face (expected:
     reported). Redraw it in the same place (expected: correct, geometric).
  8. A version-10 document with fingerprint-only references loads and
     resolves as before.
- Record each outcome per case in the Progress log as the baseline, and
  record recompute times of cases 1–6 for TN2's budget.

Done when: the suite runs on today's code and the baseline is logged. Every
later stage keeps **zero wrong** and does not lower the correct count.

## TN1 — Element maps in the core

Do:
- Interned name nodes with canonical text and parsing; the weak flag.
- `ShapeEntry` holds `shared_ptr<const ElementMap>`. Maps are indexed like
  `cad_shape_subshape_at` (`TopExp::MapShapes` order), and derived edge and
  vertex names are computed once, on first use.
- Starting names for box, cylinder, sphere, curves and polyline; weak index
  names for import and projection.
- Names kept through clone, place, translate, rotate, mirror and compound;
  restricted through subshape.
- The ABI above (except `find` grades beyond exact/alias, which TN3 needs);
  regenerate `CadKit.hxi`.
- `naming_smoke.cpp`: golden names of box and cylinder; transforms keep
  names; compound collisions get `@slot`; derived edge names of a box.

Done when: the core smoke and the C header smoke pass, and existing suites
are unchanged.

## TN2 — Operation adapters

Do:
- The adapters in the table and rules 3–7; booleans fill history in the plain
  variants too (TN-D12).
- **Completeness test:** for every operation over a fixture set, every result
  face is accounted for by rules 1–5. A leftover is a test failure naming the
  operation and face.
- Golden names for one model per operation.
- Benchmark: naming time against operation time on the TN0 models. Budget:
  naming at most 10% of recompute. Over budget → log it and decide on
  TN-D12 before continuing.
- Log the maximum and mean name length on the TN0 models (deep histories
  nest names).

Done when: completeness passes for every operation, or each gap has a logged
adapter workaround and an upstream report draft, and the budget holds.

## TN3 — Parametric references by name

Do:
- `Shape` accessors and `ElementMatch`.
- `Document.recompute` stamps each staged result with `f<id>` and the
  feature's dependency shapes (`FeatureSubgraphEvaluator` gets this through
  the same recompute).
- Starting names from `SketchProfile` (entity and point ids, regions),
  `SketchFeature` templates and `PolylineFeature`.
- Linear and polar patterns and grids stamp instances `i<i>.<j>`; mirror
  stamps its reflected copy `m`.
- `TopologyReference` holds producer, kind, name and fingerprint, and
  resolves per TN-D9. `FaceFeature` captures a name instead of an index.
  Fillet, chamfer and sketch support use it unchanged.
- `DocumentCodec` version 11:
  - references store `name`;
  - the document stores `namingScheme`;
  - version ≤ 10 references resolve by fingerprint and capture their name.
- Delete the cross-recompute history use (TN-D2) and `TopologyHistoryMap` if
  nothing else uses it.
- Editor selection keeps a name instead of a fingerprint:
  `EditorScene.remapSelectedCadFace`, `CadModelTopology`, `CadPartModel`,
  `CadDocumentSession.selectedTopology`.
- `NamingGoldenSmoke` for the corpus.
- Update `ARCHITECTURE.md` and `MODELING.md`.

Done when: TN0 cases 1–5, 7 and 8 have zero wrong, and cases 1, 2, 4 and 5
are all correct.

## TN4 — Parts and connectors

Starts after C4.5 merges into `constraint-solving` (rebase first).

Do:
- `Part.named(tag)` in the modeling layer. MachineKit recipes name the
  bodies whose faces users pick: plates, bores, shafts, flanges.
- Geometric connectors: reference format `cadkit.geometric-connector/2` adds
  `name`. The relaxed property match stays only for weak or missing names.
- `describeFaces` adds each face's name (the descriptor JSON is opaque to the
  scene artifact, so its version is unchanged). `captureDescribed` and data-only
  reframing match by name through `cad_element_name_match`.

Done when: TN0 case 6 is correct, and a mate made in the editor survives a
recipe parameter edit by name.

## TN5 — Repairing references in the editor

Do: show `Ambiguous` and `Unresolved` references with their candidates
(split pieces highlighted), let the user pick one (rebind), and show how each
reference resolved (exact, relative, geometric).

## TN6 — Names through STEP (optional)

Do: STEP import and export through OCCT's XDE reader and writer
(`STEPCAFControl`; `read.stepcaf.subshapes.name` /
`write.stepcaf.subshapes.name`). Exported models carry their face and edge
names, and imports use authored names as strong starting names when present.

## Risks to check early

- OCCT's reporting for shell, full revolutions and lofts may not match the
  table. The TN2 completeness test is there to find this before references
  depend on it.
- Split-piece names change when a piece gains or loses a neighbour. TN0
  case 3 and the Jaccard rule cover the common case; anything else is
  reported, not guessed.
- Name length grows with the depth of history along a lineage. TN2 measures
  it; there is no hashing unless it becomes a real cost (TN-D7).
- Tags are feature ids, so copying features into another document renames
  their output (TN-D15).
- The naming code is plain C++ over OCCT and must also build for the web
  (Emscripten) target.

## Progress log

### 2026-10-01 — Plan

Worktree and branch created from `constraint-solving` at 82383eeb (C4.5a).
Decisions TN-D1..D15 recorded above; nothing implemented yet. Next: TN0.

### 2026-10-01 — TN0 done: robustness suite and baseline

- `haxe/tests/NamingRobustnessSmoke.hx` (in `HaxeonSmoke`; alone with
  `CADKIT_SMOKE_ENTRY=NamingFocused scripts/test-haxeon`, which now takes
  an entry). Each row rebuilds its model, captures the reference, applies
  one edit and classes the result. The suite also checks that every oracle
  singles out exactly one element, so "correct" cannot be vacuous. `EXPECTED`
  pins every row; raise rows there as stages land.
- `NamingLegacyFixtures.BOX_FILLET_V10`: a version-10 document generated
  once and kept verbatim. Never regenerate it.
- Cases as planned, except case 7 (sketch edits): "move line" (both
  endpoints of the line under the face move 2 mm) is added, and "delete
  the line" is replaced by "split line". Deleting a line opens the profile,
  so nothing evaluates.
- Baseline: **10 correct, 17 reported, 0 wrong.** Today's references never
  pick a wrong element, but lose almost every edit:

  | Case | control | edits correct | edits reported |
  |---|---|---|---|
  | 1 box fillet / chamfer | correct | — | width, depth, height (Unresolved) |
  | 2–3 plate sketch on top face | correct | — | thickness, width, hole inside (Unresolved); slot across (Ambiguous, the right answer) |
  | 4 revolve rim fillet | correct | — | width, height |
  | 5 pattern boss rim | correct | spacing (instance 1 does not move) | count |
  | 6 part connector, two equal bores | correct | — | widen, move bore (ambiguous) |
  | 7 sketch-made side face | correct | redraw line (identical geometry) | move line; split line (the right answer) |
  | 8 legacy v10 | correct | | |

  TN3's target from this: every row correct except "slot across" and
  "split line" (reported), and case 6 correct after TN4.
- Timing: 62 ms in all edited recomputes together; the slowest rows are the
  connector rows (~12 ms, three boolean builds each). This is the
  pre-naming reference for TN2's budget.

### 2026-10-01 — TN1 done: element maps in the core

- `core/src/naming.{hpp,cpp}`: `ElementMap`, which holds face names and
  tracked edge and vertex names. Derived names are computed once with
  `call_once`, and the maps are shared by clones.
  - Rules 1, 2, 6 and 7 live in `propagate` over a `History`
    (`AlgorithmHistory` wraps any OCCT `Modified`/`IsDeleted` algorithm).
    Splits still fall to weak names until TN2.
  - Also there: `restrict_to`, `seed`, `stamp` and the weak `index_names`.
  - `SharedCurveHistory` matches a copied edge by its underlying curve, for
    wire and face building, which has no history.
- `ShapeEntry` and `OperationData` carry names. A shape from an operation
  without an adapter gets weak `face#k` names the first time they are asked
  for (`copy_named_shape`), so every shape is named (TN-D12).
- Named so far: box, cylinder, sphere; clone, translate, rotate, mirror,
  place (plain and `_operation`); compound (`@slot` on collisions); subshape
  extraction (boundary edges frozen); polyline (`seg.k`, `pt.k`); wire and
  planar face (edge names carried; the face stays weak until seeded).
- ABI: `cad_naming_scheme_version`, `cad_shape_copy_element_names_bytes`
  (newline-separated), `cad_shape_seed_names` (ids, escaped by the core),
  and `cad_shape_stamp_names`. `cad_shape_find_element` is still TN3's job.
  Haxe: `Shape.elementNames`, `elementName`, `withElementNames`, `stamped`,
  `namingScheme`.
- Tests: `tests/naming_smoke.cpp` (golden box and cylinder names, moves keep
  names, extraction, collisions, stamping, seeds through wire and face) and
  `haxe/tests/NamingSmoke.hx` (the same across the ABI). The full Haxe smoke
  and the TN0 suite are unchanged, as expected: nothing resolves by name yet.
- Deviations from the plan:
  - Box faces are named from their outward normals, not from
    `BRepPrimAPI_MakeBox`'s accessors. That is equally independent of order
    and avoids mapping OCCT's Left/Front/... to axes.
  - Names are plain `std::string`; interning waits for the TN2 benchmark.
  - Names sort bytewise, so `+` comes before `-`: `E(box.+z|box.-x)`.

### 2026-10-01 — TN2 done: operation adapters

- `propagate` covers all seven rules:
  - **Generated elements** are collected across kinds first (an edge
    sweeps a face) and named `role(sources)`, with sources sorted and
    deduplicated.
  - **Splits** name each piece `parent{neighbours bounding only this
    piece}`, compared by unsuffixed names.
  - **Merges** keep the smallest name. Aliases are not stored yet; TN3's
    lookup will need them only if merges show up in practice.
  - Faces still alike after all that get a weak `~k` by centroid.
  - `LambdaHistory` lets each adapter be written at its call site, and
    `with_face_roles` names loft caps.
- Adapters in `cadkit.cpp`:
  - booleans (plain variants now always fill history);
  - extrude and revolve (`side`, `lateral`, and `start`/`end` from
    `FirstShape`/`LastShape`);
  - fillet and chamfer (`fillet(E…)`, `chamfer(E…)`, `corner(V…)`;
    faces split or trimmed);
  - loft (`side` from section 0, caps `start`/`end`);
  - sweep (`side(profileEdge,spineEdge)` via two-argument `Generated`,
    caps from a face profile);
  - shell (kept faces keep their names, offset faces are `inner(f)`);
  - wire offset (`offset(e)`).
- **Completeness:** `naming_smoke.cpp` `check_operations` runs every
  operation on a fixture and requires every face to have a strong, unique
  name: cut with a hole, cut with a slot that splits faces, fuse, common,
  extrude, partial and full revolve, fillet and chamfer on one edge, fillet
  on all edges (corners), shell, loft and sweep. All pass, so no OCCT
  history gaps were found and there is nothing to report upstream.
  `CADKIT_NAMING_VERBOSE=1` prints every name.
- Golden examples, pinned in the test:
  - `f1:box.+z{f1:box.-x,f2:cyl.side,f3:box.-x}` and
    `f1:box.+z{f1:box.+x,f3:box.+x}`: the top face split by a slot, with
    the hole in the left piece;
  - `start(r)`, `end(r)`, `side(seg.k)`: an extrusion;
  - `fillet(E(f1:box.+x|f1:box.+z))`;
  - `side(f8:seg.1,f9:seg.2)`: a sweep.
- **Observation:** shelling a box with its top face removed gives the top
  rim (the wall-thickness ring) the removed face's name, `f1:box.+z`,
  because OCCT reports it as that face modified. That is reasonable: it is
  what is left of the top.
- **Benchmark** (`benchmarks/naming_benchmark.cpp`, target
  `cadkit-naming-benchmark`, compiles `naming.cpp` directly so OCCT and
  naming are timed apart). A 200 × 200 plate drilled with 64 holes one
  boolean at a time, then every top edge filleted (138 faces, 340 edges):
  - Release: naming is **4.5% of OCCT time**, and derived edge and vertex
    names 0.2%. Debug: 7.9%.
  - Edge names are 33 bytes on average, 52 at most.

  The budget (≤ 10%) holds, so TN-D12 stands (no unnamed fast path) and
  names stay plain strings (no interning).
- The TN0 suite is unchanged (10 / 17 / 0). References start using names
  in TN3.

### 2026-10-01 — TN3 done: references by name

- **TN0 now: 23 correct, 4 reported, 0 wrong** (from 10 / 17 / 0).
  - Box fillet and chamfer after width, depth and height edits; the plate
    sketch after thickness, width and an upstream hole; the revolve rim
    after profile edits; the pattern boss after count 3 → 4; the
    sketch-made face after its line moves: all correct.
  - Still reported, as they should be: the slot that splits the top face
    (`Ambiguous`), the sketch line split in two (`Unresolved`), and the two
    part-connector edits (TN4).
- **Names travel inside fingerprint records** (deviation: the plan had a
  separate reference field).
  - `TopologyFingerprint.capture` reads the element's own name. A face or
    edge extracted from a named shape keeps it (TN1's `restrict_to`).
  - So every holder of a fingerprint carries names with no new plumbing:
    fillet and chamfer edges, sketch support faces, `FaceFeature`, editor
    selection, and geometric connectors and face descriptors (TN4).
  - The codec writes `name` into fingerprint records and `namingScheme` at
    the document root; `DocumentCodec.VERSION` is 11 (MachineKit's pin
    updated).
- **One policy, in `TopologyResolver`** (`resolve` and `resolveAmong`):
  1. One exact name resolves.
  2. Several exact names, weak names, and tied relatives are decided by
     geometry among those candidates only.
  3. A weak name the geometry does not confirm falls back to geometry.
  4. Named candidates still tied are `Ambiguous`, unless an exact
     whole-shape geometric match exists (the floor of TN-D9).

  The core grades the text (`cad_element_name_match_bytes`,
  `ElementNames.match`): 3 exact, 2 weak, [1, 2) relative by split-suffix
  overlap. Haxe reads names in bulk, so no `cad_shape_find_element` was
  needed. After every successful remap the reference re-captures its
  element, so its name follows it, and a legacy reference gets one on its
  first resolution (tested: the v10 fixture saves with its name at v11).
- **Cross-recompute history removed** (TN-D2): from `TopologyReference` and
  `EditorScene.remapSelectedCadFace`. `TopologyHistoryMap` and
  `TopologyRemapResult` are deleted. HaxeonSmoke's history check now reads
  `Operation` directly.
- **Stamping** (`Document.recompute` → `EvaluationResult.stamped`): each
  staged result is stamped `f<id>` against its active dependencies' shapes.
  - Rule refinement found by TN0: a split piece, ordinal or slot of an
    input's element is not stamped, because it keeps that identity. Without
    this, the slot's pieces became `f8:f1:box.+z{…}` and were no longer
    relatives of `f1:box.+z`. Tested in the core smoke.
- **Instances:** linear pattern `i<k>` (`i<k>.<j>` in 2D), polar `i<k>`, grid
  `i<column>.<row>`, mirror's reflected copy `m` (`Shape.instance`).
- **Starting names:**
  - `SketchProfile` names each profile edge `e.<entity>` and each region
    `r.<sorted outer-loop entities joined by +>`.
  - `Sketch.circle` names its edge `circle`; `Sketch.slot` names
    `slot.bottom/right/top/left`.
  - `Curve.named` and `Sketch.named` seed a single edge or face.
  - The core now names a planar face `face` (strong) unless it is seeded.
- **Golden corpus:** `NamingGoldenSmoke` pins the face names of four
  documents (box fillet, plate with hole and slot, sketch extrusion,
  pattern). `CADKIT_NAMING_GOLDEN=print` reprints them.
- **HaxeonSmoke checks 98 and 90** (two coincident cylinders) now test that
  *geometry alone* is ambiguous; with names the rim resolves. That is the
  improvement, not a regression.
- **Not done, or done differently:**
  - The `Deleted` state is no longer produced. History was its only source,
    and "the name's tag names a missing feature" is not implemented, so a
    lost element is `Unresolved`.
  - Merge aliases are not stored.
  - The app compiles (`haxeon build --compiler-only --project
    app/haxeon.json`), but its suites were not run here: they need the
    native UI submodules.
  - `nativekit` (31 MB, no vendor libraries) is now populated in the
    worktree for MachineKit's suite, which passes.

### 2026-10-01 — TN4 done: parts and connectors

- Rebased onto `constraint-solving` 06cd2079 (C4.5b, C4.5c) with no
  conflicts. The worktree's haxeon moved to the new pin `d11c48fb`.
- **TN0 now: 25 correct, 2 reported, 0 wrong.** The part connector on one of
  two equal bores is now found after widening the plate and after moving the
  bore. Left reported, both rightly: the split top face and the split
  sketch line.
- `Part.named(tag)` (modeling) and `Solids.named(part, tag)` (MachineKit,
  consuming) prefix every name of a body. Recipes should name the bodies
  whose faces users pick. The names must mean the same body for every
  parameter value, never a shifting index (see the doc comment).
- **Automatic positional names for recipe bodies were considered and
  rejected.** If an optional body comes or goes, every later index shifts
  and a name would silently point at a different body. As weak names they
  would add nothing over `@slot`.
- **Geometric connectors:** a connector whose stored name is strong is found
  by that name or reported. The relaxed kind/direction/radius match now
  runs only for connectors without a strong name (legacy or unnamed
  shapes), where it can only guess among look-alikes.
- **Names in connector records:** names ride in the fingerprint record
  (TN3), so the connector reference format stays
  `cadkit.geometric-connector/1` with one additive field. That is a
  deviation from the planned `/2`: older readers simply ignore it.
  `describeFaces` and `captureDescribed` carry names the same way (tested in
  `NamingSmoke`), so the editor's data-only mate matching is name-first
  through `TopologyResolver.resolveAmong`.
- **MachineKit:**
  - `NemaStepper` names its bodies `body`, `bolt1..4` (like its connectors),
    `pilot` and `shaft`.
  - `MachineKitSmoke.namedStepperFaces` captures a connector on
    `shaft:cyl.side` of a NEMA 17 and finds it on a NEMA 23, whose shaft is
    thicker, so only the name can find it.
  - Other recipes are unnamed for now. They keep authored connectors
    (TN-D14) and fall back to geometry.
- **Not done:** the editor end-to-end check ("a mate made in the editor
  survives a recipe parameter edit") was not run. The app suites need the
  native UI submodules. The mechanism under it (descriptors with names →
  `resolveAmong`) is tested.
- **Found on the way:** a haxeon incremental-build failure ("IR
  verification failed for $equality…: Unknown IR call") after a one-line
  test edit. A clean build passes. The reproducer is kept in
  `../materia-worktrees/haxeon-incremental-repro/` (cache, source, notes).

### 2026-10-01 — TN4.5 done: hardening before the repair UI

Review after TN4: the architecture held, so eight changes were made
before TN5 builds on it.

1. **Resolution says how and between what.** `TopologyResolution` carries
   `method` (`ResolutionMethod`: name, relative, identity, geometry,
   selection, not found) and, when ambiguous, `candidates` (indices, best
   first). `TopologyReference.resolvedBy()` and `candidates()` keep the last
   result. `TopologyRemapReport` counts `byName`, `byRelative`,
   `byIdentity`, `byGeometry`, `bySelection`. TN0 prints the method for
   every row: edits resolve by name; the redrawn line and the legacy
   document by geometry; the split face is ambiguous between exactly its
   two pieces (asserted).
2. **The naming scheme guards loading.** Each stored name now carries the
   scheme that made it (`naming` next to `name` in the fingerprint record;
   the document-root field is gone). A name from other rules, or with no
   scheme, is dropped on load. The reference then resolves by geometry and
   re-captures, so a renamed rule can never make an old name match a
   different element. Tested in `NamingSmoke`.
3. **Every built-in feature names every face strongly and uniquely.**
   `NamingFeaturesSmoke` covers 26 feature types: primitives, booleans,
   template-sketch extrusions, revolve, loft, sweep, shell, pocket, the
   three hole styles, patterns, grid, mirror, rotation and tool
   collections. Only the counterbore failed: its bore and recess cylinders
   collided as `cyl.side@0/@1`. `HoleFeature` now names its bodies `bore`,
   `counterbore` and `countersink`.
4. **Names are cached per shape** in Haxe (`Shape.elementNames`; shapes
   are immutable), so resolving many references against one shape copies
   its names across the ABI once.
5. **Dead history plumbing removed:** the `Operation` parameters of
   `resolveFor` and `prepareRemap`, and `EvaluationContext.operation` and
   its staged operations (no remaining users).
6. **`Deleted` means something again:** an element whose name's creator
   tag is a feature the document no longer has, or that is suppressed. A
   lost element of a feature that still exists stays `Unresolved`. The core
   reads the tag (`cad_element_name_tag_bytes`, `ElementNames.creatorTag`),
   so Haxe still does not parse names.
7. **Typed match records.** `cad_element_name_match_bytes` returns one
   16-byte record per candidate (`cad_name_match_grade`, reserved,
   overlap), exposed as `ElementMatch`. This replaces the double that
   encoded the grade.
8. **Adapters in their own file:** `core/src/naming_adapters.hpp`,
   including the loft, sweep, wire-offset and shell adapters that were
   inline lambdas.

Gotcha: `CadKit.NameMatchGrade` has `None` and `Relative` values, which
haxeon resolves ahead of unqualified `ResolutionMethod` values. That is why
the "not found" method is `NotFound`, and why `ResolutionMethod` values are
always written qualified.

Results: TN0 is still 25 / 2 / 0. All suites pass: the core naming, core
and modeling smokes, the full CadKit Haxe smoke, MachineKit (clean build),
and the app compile.

### 2026-10-01 — TN5 done: repairing references in the editor

- **Candidates survive the failure.** A failed recompute discards the
  shape the ambiguity was found in, so `TopologyReference.candidates()`
  holds fingerprints (geometry plus name) captured at resolution time, not
  indices. `TopologyFingerprint.describe()` gives people a short summary
  ("planar face at (44, 20, 10), 1040 mm²"). The ambiguity error names its
  candidates too, so an edit that the editor rolls back on this error still
  says what it split.
- **Repair is a retarget.** `TopologyReference.retarget(fingerprint)` is one
  undoable document change (`TopologyReferenceChange`). The reference is
  pending until the next recompute finds the element, by its name first.
  No geometry has to be resolved at repair time, which matters because the
  candidates exist only in the discarded result. `Document.brokenReferences()`
  lists every reference that needs a decision.
- **Editor:**
  - `EditorScene.selectedReferenceIssues()` covers every topology reference
    of the selected feature (sketch support faces, fillet and chamfer
    edges): ambiguous ("now matches 2 elements … choose the one it means"),
    lost, deleted, and, as a warning only, found again by shape alone.
  - The inspector shows one "Use <candidate>" button per candidate, wired
    to `repairSelectedReference(reference, candidate)`, a project-level
    undoable edit.
  - The sketch's old support-face status line now shows only when there is
    no such issue; its "pick a replacement face" repair is unchanged.
- **Where broken references come from in the editor.**
  `CadDocumentSession.perform` rolls back an edit whose recompute fails, so a
  parameter edit that splits a referenced face is undone and reported (with
  the candidates in the message). It does not leave a broken reference.
  Broken references persist when a document *loads* broken (older saves,
  regenerated recipes), and that is what this UI repairs. **Follow-up:**
  offer the candidates at edit time ("this edit splits the sketch's face:
  which piece?") and re-apply the edit with the choice.
- **Tests:**
  - `NamingRepairSmoke` (CadKit): split, candidates, `describe`, retarget,
    resolve by name, undo, redo.
  - `CadPlateWorkflowTests.splitSupportRepairWorkflow` (app, real editor
    session): the inspector issue lists two pieces, choosing the right-hand
    one moves the sketch onto it, and project undo/redo restore and reapply
    the choice.
  - All app suites pass (`app/tests/haxeon.json`).
- **Environment:** the worktree now has every native submodule (MuJoCo,
  UI, animation and motion vendors) and its own app native build, which took
  about 2 minutes. Disk: 7.0 GB free afterwards. Run the app tests with
  `CADKIT_OCCT_DIR=… haxeon/scripts/haxeon run --project
  app/tests/cad-plate/haxeon.json` (CAD only) or `app/tests/haxeon.json`
  (all).

## Follow-up stages (agreed 2026-10-01 after TN5)

- **TN7:** offer the choice at edit time.
- **TN8:** name the bodies of MachineKit recipes.
- **TN9:** the smaller gaps: merge aliases, picking edges to repair edge
  references, solid names, readable labels.

TN6 (STEP names) stays optional and last.

### 2026-10-01 — TN7 done: choosing while editing

- **How an edit becomes a choice.** Every editor CAD edit goes through
  `EditorScene.applyCadEdit`, which now wraps its redo (`offeringChoice`).
  If the edit fails and leaves a reference *newly* ambiguous with
  candidates (references already broken before the edit don't count), the
  scene keeps a `PendingReferenceChoice`: the edit's redo and undo, the
  reference (feature id and index), its identity before the edit, the
  candidates, and a message ("Edit transform.x makes the sketch's support
  face match 2 elements. Choose the one it means.").
- **Inspector:** shows the pending message, one "Use <candidate>" button
  per candidate, and "Keep the previous model".
- **Choosing:** `resolvePendingReferenceChoice(k)` clears what the failed
  attempt left (the edit's undo), then applies the edit as one undoable
  operation. Its redo runs the edit; where the reference is ambiguous it
  retargets it to the candidate and recomputes. Its undo runs the edit's
  undo, then puts the reference back to its identity before the edit.
  Project undo and redo restore and reapply both the edit and the choice.
- **Cancelling:** `cancelPendingReferenceChoice()` runs the edit's undo and
  republishes, so the previous model stays.
- New generic edit `setCadFeatureParameter(id, featureId, parameter,
  value)`, which drove the test.
- **Test:** `CadPlateWorkflowTests.splitDuringEditWorkflow` moves a slot
  across a plate under a sketch. The edit stops with two candidates;
  cancel keeps slot x = −50 and a resolved sketch; the second attempt plus
  "choose the right-hand piece" applies x = 28 with the sketch on that
  piece; undo puts back both the slot and the original face; redo
  reapplies both.

### 2026-10-01 — TN8 done: MachineKit recipes name their bodies

- `MachineKitNamingAudit` (in `MachineKitSmoke`) builds every component
  type's default geometry and requires strong, unique face names.
  `MACHINEKIT_NAMING_AUDIT=print` lists the offenders. A component in its
  `UNNAMED` list must still fail, so the list can only shrink.
- **Before:** 15 component types had weak or repeated faces, all from
  same-kind primitives colliding (`cyl.side@0/@1`): deep-groove bearing,
  socket-head cap screw, flat washer, shaft collar, bushing, linear
  bearing, pillow block, flange bearing housing, lead-screw nut, robot
  flange, end-effector plate, pedestal, arm link, shaft coupling, carriage.
- **After:** they name their bodies with `Solids.named`:
  - by role: `body`, `bore`, `head`, `shank`, `socket`, `rings`,
    `cavity`, `flange`, `pilot`, `pin`, `barrel`, `base`, `column`,
    `tube`, `collar`, `guide.left/right`, `rim.front/back`,
    `seal.front/back`;
  - by position in an authored hole pattern: `bolt<k>`, `toolbolt<k>`,
    `nutbolt<k>`, `anchor<k>`, `foot<k>`, `gusset<k>`, `setscrew<k>`
    (1-based like the `bolt<k>` connectors; the same convention as pattern
    instances).
- **Cutting tools too:** the mounting cutouts (`RobotFlange` and
  `NemaStepper`), so the faces a cutout leaves in a mating plate (the
  end-effector plate, the pedestal's top) stay distinguishable.
- **Two remain, by design** (`UNNAMED`, documented): the pillow block's
  barrel side and the coupling's set-screw hole ends are each split into
  mirror-image pieces with identical neighbours. They get weak ordinals by
  position, resolved only when the geometry agrees.
- MachineKit's full script (examples and the robot-arm motion check) passes.
