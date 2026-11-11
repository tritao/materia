package cadkit.parametric.features;

import cadkit.Shape;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;

/**
 * A cylinder on an axis line between two stations along it: radius, `from` and `to` are length parameters; the axis (a point
 * and a direction) is exact. Bores, bosses and hole tools along any direction are one feature.
 */
class CylinderAlongFeature extends Feature {
  public final origin:Vector;
  public final axis:Vector;
  public final radius:Parameter;
  public final from:Parameter;
  public final to:Parameter;

  public function new(origin:Vector, axis:Vector, radius:Float, from:Float, to:Float) {
    super();
    if (axis.length() < 1e-12) throw new ParametricError("cylinder axis must not be zero");
    this.origin = origin;
    this.axis = axis.normalized();
    this.radius = new Parameter(this, "cylinder.radius", radius, 0.0, false, 1e300, ParameterKind.Length);
    this.from = new Parameter(this, "cylinder.from", from, -1e300, false, 1e300, ParameterKind.Length);
    this.to = new Parameter(this, "cylinder.to", to, -1e300, false, 1e300, ParameterKind.Length);
  }

  /** A cylinder up +Z from the origin to `height`. */
  public static function upZ(radius:Float, height:Float):CylinderAlongFeature return new CylinderAlongFeature(new Vector(), Vector.Z(), radius, 0, height);

  override public function serializationType():String return "cylinder-along";

  /** A unit vector perpendicular to the axis, fixed by the axis alone so the frame is reproducible. */
  public function crossAxis():Vector {
    var helper = Math.abs(axis.z) < 0.9 ? Vector.Z() : Vector.X();
    return helper.subtract(axis.scale(helper.dot(axis))).normalized();
  }

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var length = to.value - from.value;
    if (!(radius.value > 0 && length > 0)) throw new ParametricError('cylinder needs a positive radius and to > from: r ${radius.value}, $length');
    var cylinder = Shape.cylinder(radius.value, length);
    try {
      var start = origin.add(axis.scale(from.value));
      var placed = new Location(new Plane(start, crossAxis(), axis)).apply(cylinder);
      cylinder.close();
      return EvaluationResult.fromShape(placed);
    } catch (error:Dynamic) {
      cylinder.close();
      throw error;
    }
  }
}
