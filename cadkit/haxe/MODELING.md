# CadKit modeling API

`cadkit.modeling` is a headless, build123d-inspired Haxeon layer over the CadKit
C ABI. It uses fluent methods and explicit callback builders. It does not
require a document or NativeKit. The executable
[mounting plate example](../examples/modeling/MountingPlate.hx) is included in
the Haxeon smoke test.

## Coordinates and dimensions

`Vector`, `Axis`, `Plane`, and `Location` describe immutable geometry.
Angles are radians. Choose one consistent length unit for a model; millimeters
are conventional. No automatic unit conversion is performed.

A `Plane` is right-handed, with an origin, an X direction, and a normal. Its Y
direction is normal × X. `XY()`, `XZ()`, and `YZ()` provide standard frames.
`toWorld()` and `toLocal()` convert coordinates. `Location.compose(child)`
applies the child first, then the parent; `inverse()` reverses the placement.
Rotation about an `Axis` respects its origin.

Sketch rectangles and part boxes default to centered X/Y alignment; boxes
start at Z=0. `Align.Min`, `Center`, and `Max` select alignment per dimension.
Sketch polygon points, builder points, circle centers, and sketch-builder
patterns are local coordinates. `Curve` constructors use world coordinates.
`placed(location)` transforms an existing object's world geometry; it also
transforms a sketch's workplane. Extrusion follows that workplane's normal.

## Modeling objects

| Type | Construction and operations |
| --- | --- |
| `Curve` | Line, three-point arc, circle, interpolating spline, open/closed polyline, connected wire, rigid placement, planar wire offset, directional projection |
| `Sketch` | Planar face from outer wire and holes, rectangle, circle, polygon, capsule slot, planar booleans, placement, extrusion, revolution, sweep |
| `Part` | Box, cylinder, sphere, solid loft through wires, placement, booleans, selected-edge fillet/chamfer, selected-face shell, volume and STEP export |

Closed polylines must not repeat the first point. Wires must connect in input
order. A face requires a closed planar outer wire; holes must be closed,
coplanar, inside the boundary, and non-overlapping. Hole winding is normalized
by planar subtraction. `Sketch.face` requires its explicit workplane to match
the face; the default is XY. Native constructors validate topology before
returning handles.

`Part` results can have zero, one, or several solids. Inspect `isEmpty()` and
`solidCount()` when a workflow requires a single solid. An empty builder
returns an empty compound. The first operation in a nonempty builder must be
`Add`; subsequent operations support `Add`, `Subtract`, and `Intersect`.

`BuildLine.build`, `BuildSketch.build`, and `BuildPart.build` take callbacks and
release their owned intermediates on success and exceptions. Their `add()`
methods borrow existing models. `Locations.grid()` produces centered grids by
default. `Locations.polar()` avoids a duplicate endpoint for a full turn and
includes both endpoints for a partial arc. `BuildPart.pattern()` combines
placed copies, which can remain disconnected solids.

## Ownership

Every returned model owns its native shape and, when available, its operation
history. Inputs are borrowed. Transformations and boolean operations return
new models and leave inputs usable. Call `close()` on models when done.

`Scope.run()` supplies an explicit scope for fluent work:

```haxe
var result = Scope.run(function(scope:Scope) {
    var base = scope.own(Part.box(20, 20, 4));
    var hole = scope.own(Part.cylinder(2, 4));
    return scope.own(base.subtract(hole));
});
// result has been transferred out of the scope; its inputs were closed.
result.close();
```

Register each intermediate with `scope.own()`. The scope does not automatically
capture arbitrary temporaries in expressions. Its return value is transferred
to the caller; other registered models are released even if the callback
throws. Avoid separately closing a model's exposed `shape` or `operation`
while using the model.

Selections have separate ownership: close each selection, including selections
returned by `children()` and every group returned by `groupBy()`. `all()`,
`at()`, and `unique()` return independently owned shapes. They do not consume
the selection. Predicate callbacks receive borrowed shapes and must not close
them. Builder callbacks must close selections they create, as the example does.

## Selection and history

`faces()`, `edges()`, `vertices()`, `wires()`, and `solids()` return owning,
deduplicated `Selection` values. Refinements mutate the selection and release
rejected topology:

- `curve(kind)` and `surface(kind)` filter geometry types.
- `parallel(axis)` matches straight-edge directions or planar-face normals.
- `filter(predicate)`, `sortBy(key)`, `sortByAxis()`, and `sortBySize()` support
  custom selection rules.
- `extreme(axis)` keeps all tied maxima; pass `false` for minima. `unique()`
  reports empty or ambiguous matches instead of guessing.
- `groupBy(key, tolerance)` creates independently owned groups.
- `children(kind)` selects nested topology, such as the edges of selected faces.

Axis positions use face area centroids, vertex positions, or bounding-box
centers for other topology. Grouping compares each sorted value to the first
value in its group. Keys and predicates should be pure functions.

Extrusion, revolution, booleans, finishing, rigid placement, loft, sweep,
offset, and shell retain OCCT generated/modified/deleted history.
`generated(kind)` and `modified(kind)` expose deduplicated targets from the
last operation. These report OCCT history, not an inferred persistent identity
or an exact emulation of build123d's `LAST`/`NEW` selectors. Primitive creation,
wire/face assembly, and projection do not currently attach operation history.
Results without history return empty history selections. Document reference
remapping continues to distinguish ambiguous and deleted references.

## Parametric documents

`cadkit.parametric.features.SketchFeature` adds rectangle, circle, and slot
profiles to the document DAG. Width/height are editable parameters; circle
uses width as radius. Each feature has an immutable workplane. Compose with
existing `BooleanFeature`, `TransformFeature`, `ExtrudeFeature`,
`RevolveFeature`, and finishing features. `DocumentCodec` persists the profile,
parameters, and plane; recompute recreates geometry. Transactions support
parameter undo/redo. Existing version-1 documents remain readable.

This adapter is optional: immediate modeling does not create a feature graph.
General automatic recording of builder scripts is not implemented.

## Current operation limits

Splines interpolate nonperiodic point lists. Loft builds a solid through closed
wires without holes. Sweep uses OCCT's default pipe frame and requires a single
face or wire profile and a connected wire spine; it does not expose guide
rails or frame controls. Offset operates on planar wires with round joins and
may return several wires. Shell removes selected faces from one solid and
uses a signed thickness (negative inward). Projection is directional onto an
existing target shape and returns curves. These are initial operation APIs,
not full build123d feature parity. Sketch constraints, assemblies, variable
fillets, and operator-overloaded syntax remain separate future work.

## Validation

`ctest --test-dir build/debug --output-on-failure` exercises the C boundary,
including profile validation and all new native operations.
`./scripts/test-haxeon` compiles the generated FFI and the modeling example,
then checks dimensions, topology, finishing, STEP round trips, workplanes,
patterns, selectors, history, cleanup, and profile document recompute/persistence.
