package cadkit.parametric.features;

import cadkit.parametric.ParameterKind;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;

/** Serializable profile primitive. Use BooleanFeature for holes and combined regions.
 * Rectangle/slot use width and height; circle uses width as its radius; rounded-rectangle adds a corner radius.
 */
class SketchFeature extends Feature {
  public final profile:String;
  public final plane:Plane;
  public final width:Parameter;
  public final height:Parameter;
  /** Only the rounded-rectangle profile has a corner radius. */
  public final radius:Null<Parameter>;

  public function new(profile:String, widthValue:Float, heightValue:Float = 1, ?plane:Plane, radiusValue:Float = 0) {
    super();
    if (profile != "rectangle" && profile != "circle" && profile != "slot"
      && profile != "rounded-rectangle") throw new ParametricError("unsupported sketch profile: " + profile);
    this.profile = profile;
    this.plane = plane == null ? Plane.XY() : plane;
    width = new Parameter(this, "sketch.width", widthValue, 0, false, 1e300, ParameterKind.Length);
    height = new Parameter(this, "sketch.height", heightValue, 0, false, 1e300, ParameterKind.Length);
    radius = profile == "rounded-rectangle" ? new Parameter(this, "sketch.radius", radiusValue, 0, false, 1e300, ParameterKind.Length) : null;
  }

  override public function serializationType():String {
    return "sketch";
  }

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var sketch:Sketch;
    if (profile == "rectangle") sketch = Sketch.rectangle(width.value, height.value, plane);
    else if (profile == "rounded-rectangle") {
      var corner = radius;
      if (corner == null) throw new ParametricError("rounded rectangle has no radius");
      sketch = Sketch.roundedRectangle(width.value, height.value, corner.value, plane);
    }
    else if (profile == "circle") sketch = Sketch.circle(width.value, plane);
    else sketch = Sketch.slot(width.value, height.value, plane);
    try {
      var result = EvaluationResult.fromShape(sketch.shape.cloneShape());
      sketch.close();
      return result;
    } catch (error:Dynamic) {
      sketch.close();
      throw error;
    }
  }
}
