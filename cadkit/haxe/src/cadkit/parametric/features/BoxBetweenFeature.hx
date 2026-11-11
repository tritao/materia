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

/** An axis-aligned box between two corners, each extent a length parameter (bindable to named dimensions). */
class BoxBetweenFeature extends Feature {
  public final x0:Parameter;
  public final x1:Parameter;
  public final y0:Parameter;
  public final y1:Parameter;
  public final z0:Parameter;
  public final z1:Parameter;

  public function new(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float) {
    super();
    this.x0 = extent("box.x0", x0);
    this.x1 = extent("box.x1", x1);
    this.y0 = extent("box.y0", y0);
    this.y1 = extent("box.y1", y1);
    this.z0 = extent("box.z0", z0);
    this.z1 = extent("box.z1", z1);
  }

  function extent(name:String, value:Float):Parameter return new Parameter(this, name, value, -1e300, false, 1e300, ParameterKind.Length);

  override public function serializationType():String return "box-between";

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var dx = x1.value - x0.value, dy = y1.value - y0.value, dz = z1.value - z0.value;
    if (!(dx > 0 && dy > 0 && dz > 0)) throw new ParametricError('box needs each upper bound above its lower: $dx x $dy x $dz');
    var box = Shape.box(dx, dy, dz);
    try {
      var placed = new Location(new Plane(new Vector(x0.value, y0.value, z0.value), Vector.X(), Vector.Z())).apply(box);
      box.close();
      return EvaluationResult.fromShape(placed);
    } catch (error:Dynamic) {
      box.close();
      throw error;
    }
  }
}
