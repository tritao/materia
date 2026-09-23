# CadKit Haxeon integration

This directory contains the Haxeon projection of CadKit's C ABI. It is not a
generic Haxe binding and does not add a dependency from `cadkit-core` to
Haxeon.

The checked-in `abi/CadKit.hxi` is generated for
`x86_64-linux-gnu`. Regenerate it for another Haxeon target with:

```sh
HAXEON_TARGET=arm64-apple-darwin ./scripts/generate-haxeon-hxi
```

The projection uses Haxeon value types for `cad_vec3`, `cad_bounds`, and
`cad_mesh_options`, and closeable owned handles for `cad_shape` and `cad_mesh`.
`cadkit.Shape` and `cadkit.Geometry` are the first small semantic façade above
the generated `CadKit` module. `Shape.faces()`, `Shape.edges()`, and
`Shape.vertices()` provide lazy indexed collections of typed `Face`, `Edge`,
and `Vertex` wrappers. These wrappers expose selector-ready surface/curve
metadata without making one native call per collection construction.
Each collection also exposes a typed query builder: face queries can filter by
surface, parallel normal, area, and center; edge queries by curve, length,
parallel tangent, and endpoint; vertex queries by position. Terminal query
operations (`all`, `first`, `unique`, and `count`) consume the query and close
the temporary owner; an unused query can be released with `close()`. Returned
topology wrappers are caller-owned and must be closed. `unique` throws
`SelectionError` with `Empty`, `Ambiguous`, or `Invalid` kind rather than
silently choosing a topology element.

`Shape.tessellate()` returns immutable bulk vertex, normal, and index streams
plus `MeshFaceRange` entries. A range uses the same face index as
`Shape.faces().at(index)`, so `mesh.rangeFor(face)` identifies the index
subrange belonging to that CAD face. Tessellation results are cached per
immutable shape and exact deflection options; closing the shape releases the
cache while already returned mesh values remain valid.
`Shape.massProperties()` reports solid volume, surface area, and center of mass
in the shape's current length unit; density can be supplied to calculate mass.

Modeling methods with an `Operation` suffix retain OCCT result history:
`Operation.resultShape()` returns the new shape, while `Operation.history()`
indexes generated, modified, and deleted topology entries.
`Shape.extrude()` and `Shape.revolve()` sweep a face or wire profile; their
`extrudeOperation()` and `revolveOperation()` variants preserve the same
history information. Revolve axes use an origin, a direction, and a radian
angle.
`Shape.fillet()` and `Shape.chamfer()` apply constant finishing operations to
all edges of a solid in the initial API; their operation variants retain
history in the same way. `Shape.filletEdges()` and `Shape.chamferEdges()` take
an explicit array of selected `Edge` wrappers; their operation variants retain
the same history information and reject edges that do not belong to the source
shape.

The Haxeon-only `cadkit.parametric` package owns the first document layer:
`Document` manages a dependency DAG of box, cylinder, face, transform,
extrude, revolve, fillet, chamfer, and boolean features. `FaceFeature` extracts
an indexed face profile from a source shape, captures its fingerprint after
the first evaluation, and uses that fingerprint for subsequent remapping;
old documents without a fingerprint still use their stored index initially.
Extrude and revolve features consume that profile (or any custom feature
evaluating to a face or wire). Recompute stages dirty feature results before
replacing committed shapes, and `Transaction` groups
parameter edits for undo/redo. A
`TopologyReference` can own a selected face, edge, or vertex and remap it
after recompute: operation history is preferred, with a geometric fingerprint
fallback. Ambiguous fingerprint matches remain explicitly `Ambiguous` instead
of being guessed. `DocumentCodec` persists the feature graph and topology
fingerprints as versioned JSON and reconstructs a fresh document by
recomputing it; native handles are never serialized. NativeKit is not involved
in this layer. `ReferenceState` distinguishes initially `Resolved` references,
history/fingerprint `Remapped` references, `Deleted` topology, unresolved
references, and ambiguous matches. `Document.lastRemapReport` aggregates those
outcomes for each recompute, including selected-edge failures during staged
evaluation; `RecomputeError.referenceState` identifies a deleted or ambiguous
selection. `FilletFeature` and `ChamferFeature` optionally
own persistent edge references; those references remap against their source
feature while the finishing feature evaluates against staged source shapes.

`Shape.importStep(path)` and `Shape.exportStep(path)` provide the first
headless STEP file boundary. M14 transfers one OCCT shape and intentionally
does not preserve STEP application metadata, assemblies, or parametric
documents; those belong in later layers.

`examples/nativekit/MeshUpload.hx` is intentionally outside the CadKit package
boundary. It shows an application importing both packages and uploading the
three CadKit mesh byte streams through NativeKit GPU buffers.

After configuring CadKit with its default shared-library build, run the
compile-and-runtime smoke test with:

```sh
./scripts/test-haxeon
```

Set `HAXEON_ROOT` when Haxeon is not in the sibling directory `../haxeon`, or
`CADKIT_BUILD_ROOT` when the native build is not `build/debug`.

## Modeling layer

`cadkit.modeling` adds workplanes, curves, sketches, parts, explicit builders,
patterns, richer selectors, and resource scopes. See [MODELING.md](MODELING.md)
and the [mounting plate example](../examples/modeling/MountingPlate.hx).
`SketchFeature` connects rectangle, circle, and slot profiles to the existing
parametric document and its JSON codec.

The document layer also includes loft, sweep, offset, shell, projection, wire,
polyline, and grid features. `SelectionRecipe` persists geometric selection
intent for finishing and shell operations. See the
[editable mounting plate](../examples/modeling/EditableMountingPlate.hx) for
parameter editing, recompute, JSON persistence, and undo/redo.

`cadkit.parametric.recording.DocumentBuilder` is the explicit bridge from
builder-style construction to a serializable feature graph. Named dimensions
can drive multiple feature parameters and retain their bindings after reload.

`Document.createSubgraphDefinition()` snapshots another authored feature
document as a reusable definition. Typed definition inputs bind to named
parameters in that graph, and each named output selects a feature result.
Instances apply their overrides to the graph's parameters and retain their own
placement; the first geometry output is the instance's primary shape. These
definitions round-trip through `DocumentCodec`. `InstanceElement.makeUnique()`
copies the definition's inputs and outputs, then reassigns only that instance;
its identity, overrides, and placement remain intact, and the reassignment can
be undone. Registered recipe evaluators
remain available for domain-owned procedural definitions such as BimKit's
window, using the same typed-input and named-output contract.

## Constrained sketches

`cadkit.sketch` provides an OCCT-independent two-dimensional constraint model
with stable string IDs, construction geometry, explicit workplanes and units,
and separate authored and solved coordinates. `ConstrainedSketch.solve()` uses
bounded damped nonlinear least squares and commits a new solution snapshot only
after convergence. Diagnostics distinguish invalid input, local under- or
full-constraint rank, redundant constraints, and nonconvergent/conflicting
constraints; the latter reports the constraint IDs with significant residuals.

The initial constraint set includes fixed points, coincidence, horizontal and
vertical lines, distance and radius dimensions, equal length/radius, parallel,
perpendicular, angle, concentricity, point-on-entity, tangency, and reflection
symmetry. Tangency supports line-circle, line-arc, circle-circle, circle-arc,
and arc-arc combinations. Arcs retain their authored direction.

`SketchProfile.build()` converts a successful solution through the existing
native edge, wire, and planar-face constructors. Construction entities are
excluded. Open, branching, self-intersecting, empty, and native-invalid
boundaries raise `ProfileError` with entity IDs. Nested loops become holes.

`ConstrainedSketchFeature` integrates constrained profiles with staged document
recompute. Distance, radius, and angle constraints become bindable feature
parameters named `constraint.<id>`. The document codec persists entities,
constraints, solver settings, workplanes, and named bindings. The recording
builder exposes `constrainedSketch()`. See
`examples/modeling/ConstrainedMountingPlate.hx` for one centered, filleted plate
whose width, height, edge clearance, hole radius, and thickness survive
recompute, undo/redo, and JSON reload.
