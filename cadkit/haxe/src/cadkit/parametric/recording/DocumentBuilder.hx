package cadkit.parametric.recording;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.Feature;
import cadkit.parametric.NamedParameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.GridFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.PolarPatternFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.parametric.features.LoftFeature;
import cadkit.parametric.features.OffsetFeature;
import cadkit.parametric.features.PolylineFeature;
import cadkit.parametric.features.ProjectFeature;
import cadkit.parametric.features.RevolveFeature;
import cadkit.parametric.features.ShellFeature;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.SweepFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.parametric.ElementReference;
import cadkit.parametric.ReferencePlaneElement;
import cadkit.parametric.features.PlaceFeature;
import cadkit.parametric.features.RotationFeature;
import cadkit.parametric.features.MirrorFeature;
import cadkit.parametric.features.WireFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetFeature;
import cadkit.sheetmetal.SheetEdgeFlangeFeature;
import cadkit.sheetmetal.SheetHemFeature;
import cadkit.sheetmetal.SheetRipFeature;
import cadkit.sheetmetal.SheetCutFeature;
import cadkit.sheetmetal.SheetCornerFeature;
import cadkit.sheetmetal.SheetCornerEdge;
import cadkit.sheetmetal.CornerJoin;
import cadkit.sheetmetal.CornerRelief;
import cadkit.sheetmetal.HemSpec;
import cadkit.sheetmetal.LengthSpec;
import cadkit.sheetmetal.FlangePosition;
import cadkit.sheetmetal.ReliefType;
import cadkit.sheetmetal.ReliefSizing;
import cadkit.Shape;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetSide;

/** Explicit recorder for the serializable document feature set.
 * Methods borrow features and dimensions and return document-owned features.
 */
class DocumentBuilder {
  private final document:Document;
  private var finished:Bool;

  public function new() {
    document = new Document();
    finished = false;
  }

  private function check():Void {
    if (finished) throw new ParametricError("document builder is closed");
  }

  public function dimension(name:String, value:Float):NamedParameter {
    check();
    return document.defineParameter(name, value);
  }

  public function typedParameter(name:String, value:Float, kind:String, unit:String):NamedParameter {
    check();
    return document.defineTypedParameter(name, value, kind, unit);
  }

  public function length(name:String, value:Float, unit:String = "mm"):NamedParameter {
    return typedParameter(name, value, ParameterKind.Length, unit);
  }

  public function angle(name:String, value:Float, unit:String = "rad"):NamedParameter {
    return typedParameter(name, value, ParameterKind.Angle, unit);
  }

  public function count(name:String, value:Int):NamedParameter {
    return typedParameter(name, value, ParameterKind.Count, "count");
  }

  public function area(name:String, value:Float, unit:String = "mm2"):NamedParameter {
    return typedParameter(name, value, ParameterKind.Area, unit);
  }

  public function volume(name:String, value:Float, unit:String = "mm3"):NamedParameter {
    return typedParameter(name, value, ParameterKind.Volume, unit);
  }

  public function expression(name:String, kind:String, unit:String, source:String):NamedParameter {
    check();
    return document.defineExpression(name, kind, unit, source);
  }

  private function bind(dimension:Dimension, feature:Feature, slot:String):Void {
    var named = dimension.named();
    if (named == null) return;
    if (named.document != document) throw new ParametricError("named parameter belongs to another builder");
    named.bind(feature.parameter(slot));
  }

  /** Features added inside `body` are put in `name` unless they already have a group. */
  public function group(name:String, body:() -> Void):Void {
    check();
    var previous = document.currentGroup;
    document.currentGroup = name;
    try body() catch (error:Dynamic) {
      document.currentGroup = previous;
      throw error;
    }
    document.currentGroup = previous;
  }

  /** Name a feature for history views. */
  public function label<T:Feature>(feature:T, text:String):T {
    check();
    feature.label = text;
    return feature;
  }

  public function rectangle(width:Dimension, height:Dimension, ?plane:Plane):SketchFeature {
    check();
    var feature = document.add(new SketchFeature("rectangle", width.number(), height.number(), plane));
    bind(width, feature, "sketch.width");
    bind(height, feature, "sketch.height");
    return feature;
  }

  public function roundedRectangle(width:Dimension, height:Dimension, radius:Dimension, ?plane:Plane):SketchFeature {
    check();
    var feature = document.add(new SketchFeature("rounded-rectangle", width.number(), height.number(), plane, radius.number()));
    bind(width, feature, "sketch.width");
    bind(height, feature, "sketch.height");
    bind(radius, feature, "sketch.radius");
    return feature;
  }

  public function circle(radius:Dimension, ?plane:Plane):SketchFeature {
    check();
    var feature = document.add(new SketchFeature("circle", radius.number(), 1, plane));
    bind(radius, feature, "sketch.width");
    return feature;
  }

  public function slot(length:Dimension, width:Dimension, ?plane:Plane):SketchFeature {
    check();
    var feature = document.add(new SketchFeature("slot", length.number(), width.number(), plane));
    bind(length, feature, "sketch.width");
    bind(width, feature, "sketch.height");
    return feature;
  }

  /** Base stock from one closed constrained-sketch region, with an optional shared thickness dimension. */
  public function sheetBase(
    source:ConstrainedSketchFeature,
    stock:SheetMaterial,
    ?thickness:Dimension,
    side:SheetSide = A
  ):SheetBaseFeature {
    check();
    var selectedStock = thickness == null ? stock : new SheetMaterial(
      stock.id,
      thickness.number(),
      stock.bendRule,
      stock.mechanical,
      stock.defaultRadiusRatio,
      stock.minRadiusRatio,
      stock.grainAngle
    );
    var feature = document.add(new SheetBaseFeature(source, selectedStock, side));
    if (thickness != null) bind(thickness, feature, "sheet.thickness");
    return feature;
  }

  /** Through-wall cut from an exact constrained sketch on the chosen sheet side. */
  public function sheetCut(source:SheetFeature,
    profile:ConstrainedSketchFeature, wall:Int, side:SheetSide = A):SheetCutFeature {
    check();
    return document.add(new SheetCutFeature(source, profile, wall, side));
  }

  /** Cut the child-side seam of one permanent bend without deleting its allowance strip. */
  public function sheetRip(source:SheetFeature, bend:Int):SheetRipFeature {
    check();
    return document.add(new SheetRipFeature(source, bend));
  }

  /** Close two graph-owned flange ends, with shared gap, relief and optional overlap dimensions. */
  public function sheetCorner(
    source:SheetFeature,
    first:SheetCornerEdge,
    second:SheetCornerEdge,
    gap:Dimension,
    relief:CornerRelief,
    reliefSize:Dimension,
    join:CornerJoin = Gap,
    ?overlap:Dimension
  ):SheetCornerFeature {
    check();
    var feature = document.add(new SheetCornerFeature(
      source,
      first,
      second,
      join,
      gap.number(),
      overlap == null ? 0 : overlap.number(),
      relief,
      reliefSize.number()
    ));
    bind(gap, feature, "sheet.corner.gap");
    bind(reliefSize, feature, "sheet.corner.relief-size");
    if (overlap != null) bind(overlap, feature, "sheet.corner.overlap");
    return feature;
  }

  /** Return hem on selected wall-side faces, with one shared straight-return dimension. */
  public function sheetHem(
    source:SheetFeature,
    faces:Array<Shape>,
    spec:HemSpec,
    length:Dimension,
    side:SheetSide = A
  ):SheetHemFeature {
    check();
    var feature = document.add(new SheetHemFeature(source, faces, spec, length.number(), side));
    bind(length, feature, "sheet.hem.length");
    return feature;
  }

  /** Flange selected wall-side faces, with shared length/angle dimensions and an optional explicit radius. */
  public function sheetEdgeFlange(
    source:SheetFeature,
    faces:Array<Shape>,
    length:Dimension,
    angle:Dimension,
    ?radius:Dimension,
    lengthSpec:LengthSpec = Leg,
    position:FlangePosition = BendOutside,
    offset:Float = 0,
    gap1:Float = 0,
    gap2:Float = 0,
    extend1:Float = 0,
    extend2:Float = 0,
    reliefType:ReliefType = None,
    reliefSizing:ReliefSizing = Absolute,
    reliefWidth:Float = 0,
    reliefDepth:Float = 0
  ):SheetEdgeFlangeFeature {
    check();
    var feature = document.add(new SheetEdgeFlangeFeature(
      source,
      faces,
      length.number(),
      angle.number(),
      radius == null ? null : radius.number(),
      lengthSpec,
      position,
      offset,
      gap1,
      gap2,
      extend1,
      extend2,
      null,
      null,
      null,
      reliefType,
      reliefSizing,
      reliefWidth,
      reliefDepth
    ));
    bind(length, feature, "sheet.length");
    bind(angle, feature, "sheet.angle");
    if (radius != null) bind(radius, feature, "sheet.radius");
    return feature;
  }

  /** Record a constrained sketch and bind dimension constraint IDs to named parameters. */
  public function constrainedSketch(sketch:ConstrainedSketch,
    dimensions:Map<String, NamedParameter>):ConstrainedSketchFeature {
    check();
    var feature = document.add(new ConstrainedSketchFeature(sketch));
    for (constraintId in dimensions.keys()) {
      var dimension = dimensions.get(constraintId);
      if (dimension == null
        || dimension.document != document) throw new ParametricError("named parameter belongs to another builder");
      dimension.bind(feature.dimension(constraintId));
    }
    return feature;
  }

  public function attachedConstrainedSketch(
    sketch:ConstrainedSketch,
    support:Feature,
    selection:SelectionRecipe,
    xDirection:Vector,
    dimensions:Map<String, NamedParameter>,
    offset:Float = 0,
    flip:Bool = false
  ):ConstrainedSketchFeature {
    check();
    var feature = document.add(new ConstrainedSketchFeature(sketch, support, selection, xDirection, offset, flip));
    for (constraintId in dimensions.keys()) {
      var dimension = dimensions.get(constraintId);
      if (dimension == null
        || dimension.document != document) throw new ParametricError("named parameter belongs to another builder");
      dimension.bind(feature.dimension(constraintId));
    }
    return feature;
  }

  public function box(width:Dimension, depth:Dimension, height:Dimension):BoxFeature {
    check();
    var feature = document.add(new BoxFeature(width.number(), depth.number(), height.number()));
    bind(width, feature, "box.width");
    bind(depth, feature, "box.depth");
    bind(height, feature, "box.height");
    return feature;
  }

  public function cylinder(radius:Dimension, height:Dimension):CylinderFeature {
    check();
    var feature = document.add(new CylinderFeature(radius.number(), height.number()));
    bind(radius, feature, "cylinder.radius");
    bind(height, feature, "cylinder.height");
    return feature;
  }

  public function wire(source:Feature):WireFeature {
    check();
    return document.add(new WireFeature(source));
  }

  public function polyline(points:Array<Vector>, closed:Bool = false):PolylineFeature {
    check();
    return document.add(new PolylineFeature(points, closed));
  }

  public function grid(source:Feature,
    columns:Int, rows:Int, spacingX:Dimension, spacingY:Dimension):GridFeature {
    check();
    var feature = document.add(new GridFeature(source, columns, rows, spacingX.number(), spacingY.number()));
    bind(spacingX, feature, "grid.spacingX");
    bind(spacingY, feature, "grid.spacingY");
    return feature;
  }

  public function linearPattern(
    source:Feature,
    count:Dimension,
    spacing:Dimension,
    direction:Vector,
    ?secondCount:Dimension,
    ?secondSpacing:Dimension,
    ?secondDirection:Vector
  ):LinearPatternFeature {
    check();
    if ((secondCount == null) != (secondSpacing == null) || (secondCount == null) != (secondDirection
      == null)) throw new ParametricError("linear pattern second axis needs count, spacing, and direction");
    var feature = document.add(new LinearPatternFeature(
      source,
      count.number(),
      spacing.number(),
      direction,
      secondCount == null ? 1 : secondCount.number(),
      secondSpacing == null ? 1 : secondSpacing.number(),
      secondDirection
    ));
    bind(count, feature, "linear.count");
    bind(spacing, feature, "linear.spacing");
    if (secondCount != null) {
      bind(secondCount, feature, "linear.secondCount");
      bind(secondSpacing, feature, "linear.secondSpacing");
    }
    return feature;
  }

  public function polarPattern(
    source:Feature,
    count:Dimension,
    radius:Dimension,
    angularSpan:Dimension,
    axisOrigin:Vector,
    axisDirection:Vector,
    radialDirection:Vector,
    orientInstances:Bool = true,
    ?startAngle:Dimension
  ):PolarPatternFeature {
    check();
    var feature = document.add(new PolarPatternFeature(
      source,
      count.number(),
      radius.number(),
      angularSpan.number(),
      axisOrigin,
      axisDirection,
      radialDirection,
      orientInstances,
      startAngle == null ? 0 : startAngle.number()
    ));
    bind(count, feature, "polar.count");
    bind(radius, feature, "polar.radius");
    bind(angularSpan, feature, "polar.angularSpan");
    if (startAngle != null) bind(startAngle, feature, "polar.startAngle");
    return feature;
  }

  public function blindHole(
    target:Feature,
    selection:SelectionRecipe,
    xDirection:Vector,
    x:Dimension,
    y:Dimension,
    diameter:Dimension,
    depth:Dimension,
    offset:Float = 0,
    flipped:Bool = false
  ):HoleFeature {
    check();
    var feature = document.add(HoleFeature.plain(
      target,
      selection,
      xDirection,
      "blind",
      x.number(),
      y.number(),
      diameter.number(),
      depth.number(),
      offset,
      flipped
    ));
    bindHolePosition(feature, x, y, diameter);
    bind(depth, feature, "hole.depth");
    return feature;
  }

  public function throughHole(
    target:Feature,
    selection:SelectionRecipe,
    xDirection:Vector,
    x:Dimension,
    y:Dimension,
    diameter:Dimension,
    offset:Float = 0,
    flipped:Bool = false
  ):HoleFeature {
    check();
    var feature = document.add(HoleFeature.plain(
      target,
      selection,
      xDirection,
      "through-all",
      x.number(),
      y.number(),
      diameter.number(),
      1,
      offset,
      flipped
    ));
    bindHolePosition(feature, x, y, diameter);
    return feature;
  }

  public function blindCounterbore(
    target:Feature,
    selection:SelectionRecipe,
    xDirection:Vector,
    x:Dimension,
    y:Dimension,
    diameter:Dimension,
    depth:Dimension,
    recessDiameter:Dimension,
    recessDepth:Dimension,
    offset:Float = 0,
    flipped:Bool = false
  ):HoleFeature {
    check();
    var feature = document.add(HoleFeature.counterbore(
      target,
      selection,
      xDirection,
      "blind",
      x.number(),
      y.number(),
      diameter.number(),
      depth.number(),
      recessDiameter.number(),
      recessDepth.number(),
      offset,
      flipped
    ));
    bindHolePosition(feature, x, y, diameter);
    bind(depth, feature, "hole.depth");
    bind(recessDiameter, feature, "hole.recessDiameter");
    bind(recessDepth, feature, "hole.recessDepth");
    return feature;
  }

  public function throughCounterbore(
    target:Feature,
    selection:SelectionRecipe,
    xDirection:Vector,
    x:Dimension,
    y:Dimension,
    diameter:Dimension,
    recessDiameter:Dimension,
    recessDepth:Dimension,
    offset:Float = 0,
    flipped:Bool = false
  ):HoleFeature {
    check();
    var feature = document.add(HoleFeature.counterbore(
      target,
      selection,
      xDirection,
      "through-all",
      x.number(),
      y.number(),
      diameter.number(),
      1,
      recessDiameter.number(),
      recessDepth.number(),
      offset,
      flipped
    ));
    bindHolePosition(feature, x, y, diameter);
    bind(recessDiameter, feature, "hole.recessDiameter");
    bind(recessDepth, feature, "hole.recessDepth");
    return feature;
  }

  public function blindCountersink(
    target:Feature,
    selection:SelectionRecipe,
    xDirection:Vector,
    x:Dimension,
    y:Dimension,
    diameter:Dimension,
    depth:Dimension,
    mouthDiameter:Dimension,
    includedAngle:Dimension,
    offset:Float = 0,
    flipped:Bool = false
  ):HoleFeature {
    check();
    var feature = document.add(HoleFeature.countersink(
      target,
      selection,
      xDirection,
      "blind",
      x.number(),
      y.number(),
      diameter.number(),
      depth.number(),
      mouthDiameter.number(),
      includedAngle.number(),
      offset,
      flipped
    ));
    bindHolePosition(feature, x, y, diameter);
    bind(depth, feature, "hole.depth");
    bind(mouthDiameter, feature, "hole.recessDiameter");
    bind(includedAngle, feature, "hole.includedAngle");
    return feature;
  }

  public function throughCountersink(
    target:Feature,
    selection:SelectionRecipe,
    xDirection:Vector,
    x:Dimension,
    y:Dimension,
    diameter:Dimension,
    mouthDiameter:Dimension,
    includedAngle:Dimension,
    offset:Float = 0,
    flipped:Bool = false
  ):HoleFeature {
    check();
    var feature = document.add(HoleFeature.countersink(
      target,
      selection,
      xDirection,
      "through-all",
      x.number(),
      y.number(),
      diameter.number(),
      1,
      mouthDiameter.number(),
      includedAngle.number(),
      offset,
      flipped
    ));
    bindHolePosition(feature, x, y, diameter);
    bind(mouthDiameter, feature, "hole.recessDiameter");
    bind(includedAngle, feature, "hole.includedAngle");
    return feature;
  }

  private function bindHolePosition(feature:HoleFeature,
    x:Dimension, y:Dimension, diameter:Dimension):Void {
    bind(x, feature, "hole.x");
    bind(y, feature, "hole.y");
    bind(diameter, feature, "hole.diameter");
  }

  public function add(first:Feature, second:Feature):BooleanFeature {
    check();
    return document.add(new BooleanFeature(first, second, BooleanOperation.Fuse));
  }

  public function subtract(first:Feature, second:Feature):BooleanFeature {
    check();
    return document.add(new BooleanFeature(first, second, BooleanOperation.Cut));
  }

  public function intersect(first:Feature, second:Feature):BooleanFeature {
    check();
    return document.add(new BooleanFeature(first, second, BooleanOperation.Common));
  }

  public function extrude(
    source:Feature,
    amount:Dimension,
    ?direction:Vector,
    reversed:Bool = false,
    symmetric:Bool = false
  ):ExtrudeFeature {
    check();
    if (direction == null) {
      if (Std.isOfType(source, ConstrainedSketchFeature)) {
        var constrained:ConstrainedSketchFeature = cast source;
        direction = constrained.sketch().plane.normal;
      } else if (Std.isOfType(source, SketchFeature)) {
        var primitive:SketchFeature = cast source;
        direction = primitive.plane.normal;
      } else {
        direction = Vector.Z();
      }
    }
    var feature = document.add(ExtrudeFeature.along(source, amount.number(), direction, reversed, symmetric));
    bind(amount, feature, "extrude.amount");
    return feature;
  }

  public function pocket(target:Feature, profile:Feature, depth:Dimension):PocketFeature {
    check();
    var feature = document.add(PocketFeature.blind(target, profile, depth.number()));
    bind(depth, feature, "pocket.depth");
    return feature;
  }

  public function pocketThroughAll(target:Feature, profile:Feature):PocketFeature {
    check();
    return document.add(PocketFeature.throughAll(target, profile));
  }

  public function translate(source:Feature, x:Float, y:Float, z:Float):TransformFeature {
    check();
    return document.add(new TransformFeature(source, x, y, z));
  }

  /** A named datum frame (a reference-plane element) that placed features can be relative to. */
  public function datum(name:String, frame:Plane):ReferencePlaneElement {
    check();
    return document.createReferencePlane(name, frame);
  }

  /**
   * `source` moved rigidly from its own frame into `frame` (relative to `datum` when given). The origin may be named
   * dimensions, so they drive where the body sits; the orientation is exact.
   */
  public function place(source:Feature, x:Dimension, y:Dimension, z:Dimension, xDirection:Vector, normal:Vector,
      ?datum:ReferencePlaneElement):PlaceFeature {
    check();
    var reference = datum == null ? null : new ElementReference(document.id, datum.id);
    var feature = document.add(new PlaceFeature(source, new Plane(new Vector(x.number(), y.number(), z.number()), xDirection, normal),
      reference));
    bind(x, feature, "place.x");
    bind(y, feature, "place.y");
    bind(z, feature, "place.z");
    return feature;
  }

  /** `source` moved rigidly into `frame` (relative to `datum` when given), all values fixed. */
  public function placeIn(source:Feature, frame:Plane, ?datum:ReferencePlaneElement):PlaceFeature
    return place(source, frame.origin.x, frame.origin.y, frame.origin.z, frame.xDirection, frame.normal, datum);

  public function rotate(source:Feature, pivot:Vector, axis:Vector, angle:Dimension):RotationFeature {
    check();
    var feature = document.add(new RotationFeature(source, pivot, axis, angle.number()));
    bind(angle, feature, "rotation.angle");
    return feature;
  }

  public function mirror(source:Feature, planeOrigin:Vector, planeNormal:Vector, mode:String = "copy"):MirrorFeature {
    check();
    return document.add(new MirrorFeature(source, planeOrigin, planeNormal, mode));
  }

  public function revolve(source:Feature, origin:Vector, axis:Vector, angle:Dimension):RevolveFeature {
    check();
    var feature = document.add(new RevolveFeature(
      source,
      origin.x,
      origin.y,
      origin.z,
      axis.x,
      axis.y,
      axis.z,
      angle.number()
    ));
    bind(angle, feature, "revolve.angle");
    return feature;
  }

  public function loft(sections:Array<Feature>, ruled:Bool = false):LoftFeature {
    check();
    return document.add(new LoftFeature(sections, ruled));
  }

  public function sweep(profile:Feature, path:Feature):SweepFeature {
    check();
    return document.add(new SweepFeature(profile, path));
  }

  public function offset(source:Feature, distance:Dimension):OffsetFeature {
    check();
    var feature = document.add(new OffsetFeature(source, distance.number()));
    bind(distance, feature, "offset.distance");
    return feature;
  }

  public function shell(source:Feature, thickness:Dimension, selection:SelectionRecipe):ShellFeature {
    check();
    var feature = document.add(new ShellFeature(source, thickness.number(), selection));
    bind(thickness, feature, "shell.thickness");
    return feature;
  }

  public function project(source:Feature, target:Feature, direction:Vector):ProjectFeature {
    check();
    return document.add(new ProjectFeature(source, target, direction));
  }

  public function fillet(source:Feature, radius:Dimension, selection:SelectionRecipe):FilletFeature {
    check();
    var feature = document.add(new FilletFeature(source, radius.number(), null, null, selection));
    bind(radius, feature, "fillet.radius");
    return feature;
  }

  public function chamfer(source:Feature, distance:Dimension, selection:SelectionRecipe):ChamferFeature {
    check();
    var feature = document.add(new ChamferFeature(source, distance.number(), null, null, selection));
    bind(distance, feature, "chamfer.distance");
    return feature;
  }

  public function output(feature:Feature):Void {
    check();
    document.setOutput(feature);
  }

  public function unsupported(operation:String):Feature {
    check();
    throw new ParametricError("operation cannot be recorded: " + operation);
  }

  /** Recompute and transfer ownership to the caller. */
  public function finish():Document {
    check();
    try {
      document.outputFeature();
      document.recompute();
      finished = true;
      return document;
    } catch (error:Dynamic) {
      finished = true;
      document.close();
      throw error;
    }
  }

  public function close():Void {
    if (finished) return;
    finished = true;
    document.close();
  }

  public static function build(callback:DocumentBuilder -> Void):Document {
    var builder = new DocumentBuilder();
    try {
      callback(builder);
      return builder.finish();
    } catch (error:Dynamic) {
      builder.close();
      throw error;
    }
  }
}
