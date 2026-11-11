package cadkit.parametric.features;

import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.ElementReference;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;
import cadkit.parametric.ReferencePlaneElement;

/**
 * A solid moved rigidly from its own frame into a frame: the frame's origin is three length parameters (bindable to named
 * parameters, so dimensions can drive where a body sits), its orientation exact x-direction and normal vectors.
 *
 * With a datum (a reference-plane element), the frame is relative to the datum, and moving the datum moves the solid.
 * Orientation is stored as vectors, not angles: machine placements are mostly exact axis permutations, and an angle round
 * trip would put floating-point error into geometry that must match its source exactly.
 */
class PlaceFeature extends Feature {
  public final source:Feature;
  public final x:Parameter;
  public final y:Parameter;
  public final z:Parameter;
  public final xDirection:Vector;
  public final normal:Vector;
  public final datum:Null<ElementReference>;

  public function new(source:Feature, frame:Plane, ?datum:ElementReference) {
    super();
    this.source = source;
    // Plane has normalised the axes and checked they are perpendicular.
    xDirection = frame.xDirection;
    normal = frame.normal;
    this.datum = datum;
    x = new Parameter(this, "place.x", frame.origin.x, -1e300, false, 1e300, ParameterKind.Length);
    y = new Parameter(this, "place.y", frame.origin.y, -1e300, false, 1e300, ParameterKind.Length);
    z = new Parameter(this, "place.z", frame.origin.z, -1e300, false, 1e300, ParameterKind.Length);
  }

  /** `source` moved by (`x`, `y`, `z`) without turning. */
  public static function offset(source:Feature, x:Float, y:Float, z:Float):PlaceFeature
    return new PlaceFeature(source, new Plane(new Vector(x, y, z), Vector.X(), Vector.Z()));

  override public function serializationType():String return "place";

  override public function dependencies():Array<FeatureId> return [source.id];

  override public function dependencyFeatures():Array<Feature> return [source];

  override public function datumDependencies():Array<String> return datum == null ? [] : [datum.elementId.value];

  override public function elementReferences():Array<ElementReference> return datum == null ? [] : [datum];

  /** The frame relative to the datum (or the document when there is none). */
  public function frame():Plane return new Plane(new Vector(x.value, y.value, z.value), xDirection, normal);

  /** The frame in document coordinates. */
  public function worldFrame(context:EvaluationContext):Plane {
    var local = new Location(frame());
    if (datum == null) return local.plane;
    var element = context.owner().resolveElement(datum, "reference-plane");
    var reference = Std.downcast(element, ReferencePlaneElement);
    if (reference == null) throw new ParametricError("place datum must be a reference plane");
    return new Location(reference.plane).compose(local).plane;
  }

  override public function evaluate(context:EvaluationContext):EvaluationResult
    return EvaluationResult.fromOperation(new Location(worldFrame(context)).applyOperation(context.shape(source)));
}
