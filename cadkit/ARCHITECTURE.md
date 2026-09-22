# CadKit architecture

CadKit is a headless CAD kernel boundary. Its core must remain usable from a
CLI, test process, server worker, simulator, or GUI host without linking to a
window system, renderer, NativeKit, or Haxeon.

## Dependency rule

```text
Haxeon ───────► Haxeon CAD API ─────► cadkit-core ───────► OCCT
   │
   └────────► NativeKit (UI / GPU / platform)
```

`cadkit-core` may depend on:

- C and C++;
- OCCT;
- the C/C++ standard libraries.

`cadkit-core` must not depend on NativeKit, Haxeon, a renderer, a window
system, a simulator, or the parametric document model.

## ABI contract

1. The public ABI is C, never C++.
2. OCCT classes never cross the ABI.
3. Public CAD objects are opaque, generation-checked handles.
4. Ownership and destruction are explicit.
5. Native exceptions never escape the C boundary.
6. Large geometry crosses the ABI in bulk rather than item-by-item.
7. CAD topology and render meshes are separate concepts.
8. Parametric documents live above `cadkit-core`.
9. ABI additions are backwards-compatible once public.

The initial implementation proves the handle table, primitive construction,
bounds queries, immutable transforms, and bulk tessellation into owned mesh
buffers with CAD-face index ranges, unique topology enumeration through
generic shape handles, and basic
boolean/measurement operations. Haxeon additionally receives mesh streams as
bulk byte buffers and projects lazy typed face/edge/vertex collections through
value types. Face area/normals, edge curve kinds/tangents, and OCCT `IsSame`
topology identity are the first selector-ready interrogation operations. More
operations should preserve an explicit generated/modified/deleted operation
history, which is now available through an owning operation result handle and
indexed C/Haxeon accessors. Profile extrusion and revolution plus constant
all-edge and selected-edge fillet and chamfer operations are also headless core
operations. Selected finishing crosses the ABI as a counted list of
generation-checked edge handles, validated against the source shape before
OCCT is called. More operations should be added only after this boundary is
tested from a language-neutral caller.

The first parametric document layer is Haxeon-only. It owns feature IDs,
parameters, dependency ordering, staged recompute, and parameter transactions;
it consumes `cadkit.Shape` and `cadkit.Operation` but is not visible to the
native core. `TopologyReference` keeps an owning topology snapshot and remaps
it after recompute using OCCT operation history first, then a conservative
geometric fingerprint fallback. Haxeon selectors require deterministic
selection: `unique` reports empty and ambiguous matches as typed errors, and
ambiguous topology-reference fallback is preserved as an explicit state rather
than guessed. Operation history mapping is centralized so generated and
modified targets are deduplicated, multiple targets become ambiguous, and
deleted sources become explicit `Deleted` references. Document recompute
returns aggregate remap counts through the Haxeon document layer, including
selected-edge failures discovered while evaluating staged results. A failed
recompute records `Deleted` or `Ambiguous` before raising `RecomputeError`,
so callers can inspect the failure and recover through undo/redo.
`DocumentCodec` serializes the feature graph, parameters, and those
fingerprints as versioned JSON; native handles and meshes never enter the
document format. Profile extrusion and revolution are headless core
operations; their Haxeon feature wrappers consume face or wire profiles.
`FaceFeature` uses an index only for initial selection, then captures
a geometric fingerprint and persists it for remapping across recompute.
`FilletFeature` and `ChamferFeature` can own persistent selected-edge
references while remapping those references against their source feature.
Geometric sketches are supplied by the modeling layer described below. The
Haxeon-owned `cadkit.sketch` layer adds constrained authored geometry, a pure
Haxeon numerical solver, diagnostics, and validated conversion to native
profiles without adding solver policy to the native core.

STEP file transfer is also a core boundary operation: `cad_step_import` and
`cad_step_export` use OCCT's `TKDESTEP` provider without introducing a UI,
renderer, NativeKit, or Haxeon dependency. The initial boundary transfers one
shape and intentionally excludes application metadata, assemblies, and
parametric document semantics.

## OCCT policy

OCCT is vendored under `third_party/occt` from released version `V8_0_1`,
commit `b8f597c677811d1f9f4d8a97f5ae2825c0353a42`. Updating OCCT is an explicit
maintenance change that must replace the vendored source and update the
documented version together; builds must never silently follow an OCCT branch.

## Immediate modeling layer

`cadkit.modeling` sits above the CadKit shape/operation façade. Immutable
vectors, axes, workplanes, and placements provide coordinate semantics;
Curve/Sketch/Part and explicit callback builders provide modeling semantics.
Native line/arc/circle/spline, wire/face, compound, placement, loft, sweep,
planar-wire offset, shell, and directional projection operations cross the
same generation-checked C boundary. Profile constructors validate connectivity,
planarity, containment, and resulting topology. TKOffset is a headless core
dependency for the additional sweep/offset operations.

Selections own deduplicated topology handles, support geometric sorting and
nested selection, and preserve ambiguity at extrema. Modeling scopes and
builders release owned intermediates on failures. Operation variants retain
OCCT history for placement, loft, sweep, offset, and shell; projection and
profile construction currently return shapes without history.

The optional parametric `SketchFeature` adapter consumes the modeling layer
and persists rectangle/circle/slot parameters and a workplane through
`DocumentCodec`. The modeling layer does not depend on the document model.
See `haxe/MODELING.md` for ownership rules and operation limits. Geometric
sketch construction and constrained sketch solving are available; interactive
inference, dragging, and large-sketch optimization remain later layers.

The document adapter additionally serializes wire extraction, editable
polyline paths, solid loft, sweep, planar-wire offset, selected-face shell,
directional projection, and centered XY grid features. They consume staged
results through `EvaluationContext` and inherit document recompute atomicity
and parameter transactions. Loft/sweep/offset/shell retain native operation
history; projection and grid assembly return shapes without history.

`SelectionRecipe` stores a geometric query with an expected cardinality for
shell/fillet/chamfer. It evaluates against each staged source and rejects missing
or changed-cardinality selections, independently of persistent topological
reference remapping. Both selection mechanisms are serialized by the codec;
documents without a recipe retain their original reference semantics.
The editable mounting-plate example exercises parameter edits, selection
reevaluation, transactional failure recovery, persistence, and undo/redo.

`cadkit.parametric.recording.DocumentBuilder` constructs this same feature DAG
through an explicit builder API. `NamedParameter` records one-to-many bindings
to stable feature parameter names; edits route through a document transaction,
so all bound slots undo together. The codec stores these bindings and the
selected output feature. Immediate modeling builders remain independent and
never create hidden document state.

Documents also own persistent `DocumentId` and `ElementId` values. An element
records a name and one output feature, and several elements can expose separate
committed shapes from the same feature graph. Element creation, removal,
renaming, duplication, and output reassignment use document undo/redo without
deleting features. Format version 2 persists this registry. Version 1 input is
migrated by exposing its effective output as a `Model` element. These IDs
describe document-level objects and do not replace topology remapping for faces
or edges.
