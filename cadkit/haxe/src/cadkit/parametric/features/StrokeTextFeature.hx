package cadkit.parametric.features;

import cadkit.modeling.StrokeText;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;

/**
 * Lettering as a solid (`StrokeText`): `text` `height` high in strokes `stroke` wide, from z0 to z1, its baseline centre at
 * (x, y) and turned by `rotation` about Z; `mirror` reads it from below. A raised label or the tool of an engraved one.
 */
class StrokeTextFeature extends Feature {
  public final text:String;
  public final height:Parameter;
  public final stroke:Parameter;
  public final z0:Parameter;
  public final z1:Parameter;
  public final mirror:Bool;
  public final x:Parameter;
  public final y:Parameter;
  public final rotation:Parameter;

  public function new(text:String, height:Float, stroke:Float, z0:Float, z1:Float, mirror:Bool = false, x:Float = 0, y:Float = 0, rotation:Float = 0) {
    super();
    if (text.length == 0) throw new ParametricError("stroke text needs characters");
    this.text = text;
    this.mirror = mirror;
    this.height = new Parameter(this, "text.height", height, 0.0, false, 1e300, ParameterKind.Length);
    this.stroke = new Parameter(this, "text.stroke", stroke, 0.0, false, 1e300, ParameterKind.Length);
    this.z0 = new Parameter(this, "text.z0", z0, -1e300, false, 1e300, ParameterKind.Length);
    this.z1 = new Parameter(this, "text.z1", z1, -1e300, false, 1e300, ParameterKind.Length);
    this.x = new Parameter(this, "text.x", x, -1e300, false, 1e300, ParameterKind.Length);
    this.y = new Parameter(this, "text.y", y, -1e300, false, 1e300, ParameterKind.Length);
    this.rotation = new Parameter(this, "text.rotation", rotation, -1e300, false, 1e300, ParameterKind.Angle);
  }

  override public function serializationType():String return "stroke-text";

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var part = StrokeText.solid(text, height.value, stroke.value, z0.value, z1.value, mirror, x.value, y.value, rotation.value);
    try {
      var shape = part.shape.cloneShape();
      part.close();
      return EvaluationResult.fromShape(shape);
    } catch (error:Dynamic) {
      part.close();
      throw error;
    }
  }
}
