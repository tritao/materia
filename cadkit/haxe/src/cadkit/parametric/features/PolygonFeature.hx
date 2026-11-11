package cadkit.parametric.features;

import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.ParametricError;

/** A closed polygon face: `points` (in the plane's own x/y) joined in order and closed, on `plane`. A profile to revolve or extrude. */
class PolygonFeature extends Feature {
  public final points:Array<Vector>;
  public final plane:Plane;

  public function new(points:Array<Vector>, plane:Plane) {
    super();
    if (points.length < 3) throw new ParametricError("a polygon needs at least three points");
    for (p in points) if (Math.abs(p.z) > 1e-10) throw new ParametricError("polygon points lie in the plane's x/y");
    this.points = points.copy();
    this.plane = plane;
  }

  override public function serializationType():String return "polygon";

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var sketch = Sketch.polygon(points, plane);
    try {
      var face = sketch.shape.cloneShape();
      sketch.close();
      return EvaluationResult.fromShape(face);
    } catch (error:Dynamic) {
      sketch.close();
      throw error;
    }
  }
}
