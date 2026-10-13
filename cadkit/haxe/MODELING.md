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
parameter undo/redo. Persistence writes version 2 and reads version 1 for
element-registry migration.

This adapter is optional: immediate modeling does not create a feature graph.
Immediate builders do not silently create feature graphs. Use the explicit
recording builder below when the result must remain editable.

## Recording builder and named dimensions

`cadkit.parametric.recording.DocumentBuilder` creates the same serializable
features described below while using builder-style construction. It owns its
document until `finish()` succeeds. `DocumentBuilder.build(callback)` closes
the partial document when construction or initial recompute fails.

```haxe
var document = DocumentBuilder.build(function(builder) {
    var width = builder.dimension("plate.width", 80);
    var depth = builder.dimension("plate.depth", 50);
    var thickness = builder.dimension("plate.thickness", 6);
    var profile = builder.rectangle(width, depth);
    var solid = builder.extrude(profile, thickness);
    builder.output(solid);
});

document.parameter("plate.width").set(100);
document.recompute();
var shape = document.result(); // borrowed committed output
```

Named dimensions can bind several scalar feature slots. Editing the dimension,
or any bound feature parameter, changes every binding as one undoable edit.
The codec saves dimension names, current values, bindings, and the selected
document output. Reloaded documents preserve those relationships. JSON from
older documents is rejected; current v11 documents without an explicit output
use the last feature. Persistence saves current model state,
not the undo/redo stacks.

Named parameters may be declared as `length`, `angle`, `count`, `area`, or
`volume`. Canonical values use millimetres, radians, integer counts, square
millimetres, and cubic millimetres. Explicit conversions support
`mm/cm/m/in/ft`, `rad/deg`, and the corresponding squared and cubed length
units. `valueIn(unit)` converts for display and `set(value, unit)` accepts an
explicit input unit. `dimension()` values remain unitless scalars.

`Document.defineExpression` and `DocumentBuilder.expression` create read-only
derived parameters. Expressions support named references, parentheses, unary
signs, and `+`, `-`, `*`, and `/`; addition requires identical quantity types,
while multiplication and division derive length, area, and volume types.
Dependencies are evaluated before feature recompute. Unknown references,
dimension mismatches, non-finite results, fractional counts, and dependency
cycles fail without replacing committed geometry. Expression definitions,
types, units, dependencies, and bindings survive document serialization.
`ExpressionDrivenWall.hx` implements `wall.height = upperLevel.elevation -
baseLevel.elevation` and updates the bound solid from level edits.

The recorder covers rectangle/circle/slot/box/cylinder primitives, constrained sketches, wires and polylines,
grids, linear and polar patterns, planar and solid booleans, extrusion, translation, revolution,
loft, sweep, offset, shell, projection, fillet, and chamfer. Operations whose
editable meaning is not represented must be rejected explicitly with
`unsupported(name)`. Recorded extrusion uses a scalar amount and a fixed unit
direction. When no direction is supplied, sketch features use their workplane
normal and other features use positive Z. Explicit directions may be arbitrary;
reversed mode negates the direction, and symmetric mode centers the extrusion
on the source profile. Editable XYZ extrusion components remain supported.

`LinearPatternFeature` generates a centered compound of independent instances
along an arbitrary unit direction and, optionally, a second nonparallel axis.
Counts and spacing are editable named parameters; counts must remain positive
integers. The pattern does not perform a Boolean operation. Pass its compound
to `BooleanFeature` or the recording builder's `add`/`subtract` methods when
the instances should modify a target solid.

`PolarPatternFeature` places instances at an editable radius around an explicit
axis and radial direction. Count, radius, start angle, and angular span are
normal feature parameters. A span of positive or negative `2π` omits the
duplicate endpoint; partial spans include both ends. Instance orientation may
rotate with the orbit or remain fixed. `ParametricFlange.hx` demonstrates a
bolt-circle cut, while `VentilatedEnclosure.hx` patterns an attached profile
before applying one through-all pocket operation.

`RotationFeature` applies a rigid rotation around an explicit pivot and
arbitrary axis. Its angle is an editable parameter and may be bound to a named
dimension through `DocumentBuilder.rotate`. Rotation uses the placement
operation, preserving generated/modified history where OCCT reports it.

`MirrorFeature` reflects profiles or solids across an explicit plane. `copy`
returns only the reflection and retains transform history, `both` returns an
unfused compound containing the source and reflection, and `fuse` joins both
halves through a Boolean operation. Mirrored cutting tools remain independent
features and can be subtracted later. `MirroredMountingBracket.hx` demonstrates
a fused half-model followed by a mirrored pair of counterbore tools.

## Current operation limits

Splines interpolate nonperiodic point lists. Loft builds a solid through closed
wires without holes. Sweep uses OCCT's default pipe frame and requires a single
face or wire profile and a connected wire spine; it does not expose guide
rails or frame controls. Offset operates on planar wires with round joins and
may return several wires. Shell removes selected faces from one solid and
uses a signed thickness (negative inward). Projection is directional onto an
existing target shape and returns curves. These are initial operation APIs,
not full build123d feature parity. Assemblies, variable fillets, interactive
constraint inference and dragging, and operator-overloaded syntax remain separate future work.

## Constrained sketches

`cadkit.sketch.ConstrainedSketch` stores stable point, line, circle, arc, and
constraint IDs separately from solved coordinates. Construction entities take
part in solving but do not produce profile edges. The pure Haxeon solver uses
bounded damped least squares, seeds edits from the last successful solution,
and reports convergence, residual, local rank-based degrees of freedom,
redundancy, nonconvergence, and locally conflicting stationary residuals.
`SketchSession` applies edits to isolated drafts and solves them without
requiring a closed profile. It exposes the current diagnostic and remaining
degrees of freedom; after a conflicting edit, the previous solution remains
available while the draft can be repaired. Profile construction remains an
explicit later step.

Sketch coordinates, radii, and distance dimensions are authored in the
sketch's selected length unit (`mm`, `cm`, `m`, `in`, or `ft`). Solving keeps
those values in that unit; profile construction converts local geometry to
canonical millimetres before passing it to OCCT. The workplane origin stays in
world millimetres. Feature dimension parameters use canonical millimetres
(angles use radians), and the codec converts dimensional constraints back to
their authored sketch unit when saving.

`SketchProfile.build()` validates line and curved boundaries, rejects overlap,
and classifies arbitrary nesting by parity so islands within holes become
material again. Exact curves are passed to the native wire and face builders;
sampling is used only for intersection and containment validation.

`ConstrainedSketchFeature` exposes dimensional constraints as normal feature
parameters and supports controlled entity and constraint edits. These edits
participate in document transactions and undo/redo. The document codec stores
the current authored model, workplane, solver settings, IDs, and parameter
bindings. A constrained sketch may also depend on a supporting solid and derive
its workplane from exactly one planar `SelectionRecipe`. The attached plane
uses an explicit projected X direction and supports normal flipping and signed
offset. It is recomputed from the staged supporting shape, while authored point
coordinates and dimensions remain local to the plane. See `ConstrainedMountingPlate.hx` and
`ConstrainedSlottedBracket.hx` for complete editable examples.

`PocketFeature` subtracts an attached planar profile along its face normal.
Blind pockets expose a named positive depth. Through-all pockets derive their
cut length from the target's current projected extent, so they continue through
the solid after dimensional recompute. Both target and profile are document
dependencies, and failed attachment or boolean evaluation leaves the previously
committed result intact. `AttachedPocketEnclosure.hx` combines a blind recess on
the top face with through-holes attached to a side face.

`HoleFeature` generates a face-attached cutting tool at editable local X/Y
coordinates. Plain holes support blind depth and adaptive through-all modes.
Counterbores add an explicit larger recess diameter and depth, and reject a
recess no larger than the bore or a blind recess extending through the bore.
The tool remains separate from its Boolean cut and may be passed through linear
or polar pattern features first. `CounterboredEnclosureLid.hx` demonstrates a
four-hole patterned cut that follows lid size and thickness changes.

Countersinks use explicit bore diameter, mouth diameter, and included angle.
Their conical depth is derived from those dimensions and must be positive and
shallower than the blind depth or current through-all target extent. Included
angles must lie strictly between zero and `π` radians. `CountersunkFlange.hx`
patterns a face-attached countersink tool around a bolt circle and keeps the
through cut valid when flange radius and thickness change.

## Sheet metal

`cadkit.sheetmetal.Sheet` owns an exact line/arc wall graph, permanent wall/bend ids,
thickness through `SheetMaterial`, and an explicit bend rule. Angles are radians and lengths
are millimetres. Side A is local z = 0; the material grows toward negative Z. A missing
bend rule is diagnosed, never supplied implicitly.

`evaluate(flat())` and `evaluate(folded())` derive wall placements and bend strips without
native geometry. `SheetState` also supports partial angles, preserving the bend allowance's
neutral length. Evaluations keep immutable wall snapshots. Rips cut connectivity at the
child-side bend seam and retain the strip on the parent; closure failures name the measured gap.

`SheetSolid.build(sheet, state)` returns an owning `Shape`: wall extrusions and bend-strip
revolutions fused into one result. Face identities come from wall/loop/segment and bend roles,
seeded before fusing. The native naming grammar escapes their logical identifiers. Close the
returned shape when done. Native history is used within the extrusion to match side faces to
their exact input edges; graph identity supplies persistence across rebuilds.

`FlatPattern.of(sheet)` derives an exact developed blank without native geometry. `regions()`
returns its outer contours and holes; `domains()` retains typed wall/bend ownership for checks.
`bends()` returns a table sorted by permanent bend id, including signed angle, inner radius,
allowance and the material-covered fold-line intervals. `length()` sums those intervals;
`nominalWidth` preserves the attachment width before cuts. Returned arrays are isolated copies.

When `SheetMaterial.grainAngle` is authored, it is an undirected rolling angle in radians relative
to the root wall frame. The flat pattern's `plane` aligns its X axis with that grain; without
grain it uses the root plane. Bounds and all contour coordinates refer to this exported plane.
The document saves the material snapshot, including grain, with its current schema version.

`cadkit.drawing.Drawing2D.of(flat)` supplies exact contours, bend markings, annotations and
an optional grain arrow to both exporters. `Dxf.write(drawing)` returns R12 ASCII with only
LINE, ARC, CIRCLE and TEXT entities; clockwise contours retain their winding through negative-Z
object coordinates. `Svg.write(drawing)` returns a millimetre SVG with the same manufacturing
layers. Both return text for the caller to save and perform no native operation. DXF annotations
must be printable ASCII; SVG annotations escape XML text.

`SheetChecks.of(sheet)` returns a developed snapshot plus named manufacturability findings.
Each finding has a typed kind, measurement, assumption flag and exact geometry in the report's
flat plane. Checks cover overlaps, minimum straight flange legs, cut/hole clearance, internal
bend endings, explicit material radius minima and grain direction. The provisional `4T` tooling
minimum and `2.5T + R` cut clearance are marked assumed. Relief boundaries carry an explicit
purpose, so checks do not infer it from edge names.

## Validation

`ctest --test-dir build/debug --output-on-failure` exercises the C boundary,
including profile validation and all new native operations.
`./scripts/test-haxeon` compiles the generated FFI and the modeling example,
then checks dimensions, topology, finishing, STEP round trips, workplanes,
patterns, selectors, history, cleanup, and profile document recompute/persistence.

## Editable feature graphs

The document layer also provides these serializable features:

| Feature | Inputs and editable parameters |
| --- | --- |
| `WireFeature` | Extracts the sole wire of a profile; rejects profiles with holes or multiple wires |
| `PolylineFeature` | World-space point coordinates (`coordinate(point, axis)`); point count and closure are fixed |
| `LoftFeature` | Ordered wire-feature dependencies; solid loft, with fixed ruled/smooth option |
| `SweepFeature` | Face/wire profile and connected wire path |
| `OffsetFeature` | Planar wire and signed offset distance |
| `ShellFeature` | One solid, a face-selection recipe, and signed thickness |
| `ProjectFeature` | Edge/wire source, target shape, and editable projection direction |
| `GridFeature` | Source, fixed row/column counts, editable positive XY spacing |

Loft, sweep, offset, and shell retain native operation history. Projection and
grid compounds currently do not. Grids contain translated copies rather than
boolean unions and use world XY coordinates. The native operation limits above
also apply to feature evaluation. Zero offset/thickness/direction and invalid
geometry fail staged recompute, preserving committed shapes. Undo restores
parameters; call `recompute()` after undo or redo.

`SelectionRecipe` stores geometric intent for shell, fillet, and chamfer. It
supports face/edge kinds, plane/line/circle filters, optional parallel
directions, positional minima/maxima/both ends, and an explicit expected
count. The recipe is evaluated against the staged source on every recompute.
A missing match reports `Unresolved`; a nonzero count different from the
expected count reports `Ambiguous`. These states propagate through
`RecomputeError`. A recipe does not claim that a regenerated edge is the same
topological entity as an old edge. Existing persistent reference/fingerprint
selection remains available and keeps its original remapping semantics.

For example, the plate's finishing feature uses:

```haxe
var corners = new SelectionRecipe(
    "edge", "line", Vector.Z(), "ends", Vector.X(), 4);
var finish = document.add(new FilletFeature(extrusion, 2, null, null, corners));
```

The [editable mounting plate](../examples/modeling/EditableMountingPlate.hx)
contains six features: rectangle, circle, hole grid, planar subtraction,
extrusion, and selected-edge fillet. Its `resize()` groups width, depth, and
hole-spacing edits in a transaction, checks hole clearance, and recomputes
before committing. Invalid edits cancel the parameter transaction. The example
writes JSON and STEP, exercises undo/redo, reloads the JSON, and exports the
reloaded shape. The JSON stores the feature graph and current parameters,
including the selection recipe; it does not store the undo/redo session stack.
After reload, subsequent edits create a new undo/redo history.

### Sheet corners

`SheetCornerEdge.fromFace(source.sheet(), face)` resolves a picked straight side
face into a permanent wall id and authored boundary identity. A corner closes
two flange ends on a common parent. `CornerJoin.Gap` gives each end the stated
perpendicular stand-off from its mate's full thickness slab; `OverlapFirst` or
`OverlapSecond` extends that flange past the mate's far face by the stated overlap.
The selected supporting edges must run along the folded planes' intersection.
Parallel planes and missing selections receive named diagnoses.

```haxe
var gap = builder.length("corner.gap", 0.5);
var relief = builder.length("corner.relief", 8);
var corner = builder.sheetCorner(source,
    SheetCornerEdge.fromFace(source.sheet(), firstSide),
    SheetCornerEdge.fromFace(source.sheet(), secondSide),
    gap, CornerRelief.Round, relief);
```

Round relief size is diameter; square size is side length, with its first axis
along the first parent bend seam. One exact blank-space tool trims adjoining
walls and bend strips. Contour changes and relief publish together after seam
validation, retaining wall/bend ids and nominal datums. A gap that needs more
seam trimming than the relief provides fails material validation. Saved corners
contain authored picks and dimensions; development and solids are rebuilt.
The current document format is 16 and rejects previous versions.

### Sheet manufacturing blanks

`SheetBlank.requirement(FlatPattern.of(sheet), order)` derives material, thickness
and rectangular stock dimensions from the current sheet. Supply a typed
`SheetBlankOrder` with requirement and part ids, their positive revisions, quantity,
and `SheetTolerance(minus, plus)` values in mm for thickness, width and height.
The exported X axis follows authored grain; grained stock permits rotation 0,
while ungrained stock permits 0/90 degrees. The result is ManufacturingKit
`SheetPartRequirement`, ready for its current project codec and stock planning.

### Editing sheets in the app

In an empty CAD part, create and apply a constrained sketch. Select its feature,
choose a sheet grade, thickness in mm and rolling direction in the sketch frame,
then choose **Create sheet base**. Leave the K-factor override blank to use the
grade table; an explicit material identity without sheet defaults needs an ANSI K factor. Pick a straight wall-side face in
the folded viewport, enter the straight flange length, signed bend angle in
degrees and inner radius in mm, then choose **Create edge flange**.

The selected sheet feature exposes editable dimensions and the bend table.
**Flat pattern** switches the viewport to the derived blank and shows up/down
bend intervals in blue/orange. This is view state; it does not edit the saved
feature graph. Flanges are created in folded view. **Export flat DXF** and
**Export flat SVG** export the exact current blank, bend marks and annotations.

### Air-bending predictions

`cadkit.sheetmetal.process.BendProcess` accepts typed stock and tooling, a signed
bend angle in radians and an effective bend length in mm. `AirBending` requires
UTS, yield and Young's modulus in MPa. Tool included angles are radians; tool
radii and V opening are mm. Predictions distinguish the loaded and released
inner radii, conserve neutral length, and return signed overbend, punch-bottom
depth below the die top, force in kN and an assumed 90-degree minimum flange.
The initial formulas are assumed estimates; tool angle/contact failures are
named diagnoses.

```haxe
var stock = new SheetMaterial("mild-steel", 1.5, null,
    {uts: 400, yield: 250, youngs: 205000});
var tools = new ProcessTooling("acute-punch", "v12", 0.6,
    Math.PI / 6, 12, Math.PI / 6, 0.5);
var process:BendProcess = new AirBending();
var prediction = process.predict(stock, tools, Math.PI / 2, 120);
```

`CalibrationEntry` supplies an explicit material lot, thickness, tool ids,
half-open angle-magnitude band, depth offset and measured final inner radius.
Pass an immutable `CalibrationTable` to `AirBending` and the lot as the fifth
prediction argument to apply it. The model reconstructs loaded radius and depth
from the measured final radius before adding the offset. Overlapping entries
and measurements incompatible with the punch are refused. Without a matching
lot/key, `calibrated` is false and depth remains analytic. Use `BendRule.Process(new ProcessBendRule(tools, calibration, lot))` on stock
with mechanical properties to develop sheets for that setup. A flange with an
omitted radius derives its released radius from the process on every rebuild;
an explicit radius must match the prediction. Stock, tools, lot and calibration
are saved; derived predictions are rebuilt. `BendDimensions.allowance(stock,
radius, angle, bendLength)` and `deduction` consume the same stock object.
Air bending refuses hems and tool-limited angles. The sheet checks use the
assumed 0.7V flange minimum, with V=8T as the default for stock up to 3 mm;
thicker stock requires selected tooling.
