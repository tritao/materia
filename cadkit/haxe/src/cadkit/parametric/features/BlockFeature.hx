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
 * An axis-aligned block by its sizes, placed against the origin: on each axis it starts at the origin (`min`), is centred on
 * it (`centre`) or ends at it (`max`). A plate centred in x and y on its own frame, resting on z = 0, is
 * `block(length, width, thickness)` with the default alignment; the sizes are length parameters.
 */
class BlockFeature extends Feature {
  public static inline final MIN = "min";
  public static inline final CENTRE = "centre";
  public static inline final MAX = "max";

  public final x:Parameter;
  public final y:Parameter;
  public final z:Parameter;
  public final align:Array<String>;

  public function new(x:Float, y:Float, z:Float, ?align:Array<String>) {
    super();
    this.x = new Parameter(this, "block.x", x, 0.0, false, 1e300, ParameterKind.Length);
    this.y = new Parameter(this, "block.y", y, 0.0, false, 1e300, ParameterKind.Length);
    this.z = new Parameter(this, "block.z", z, 0.0, false, 1e300, ParameterKind.Length);
    this.align = align == null ? [CENTRE, CENTRE, MIN] : align.copy();
    if (this.align.length != 3) throw new ParametricError("a block needs one alignment per axis");
    for (a in this.align) if (a != MIN && a != CENTRE && a != MAX) throw new ParametricError("unknown block alignment: " + a);
  }

  override public function serializationType():String return "block";

  static function start(size:Float, align:String):Float return align == MIN ? 0 : align == CENTRE ? -size / 2 : -size;

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    if (!(x.value > 0 && y.value > 0 && z.value > 0)) throw new ParametricError('block needs positive sizes: ${x.value} x ${y.value} x ${z.value}');
    var box = Shape.box(x.value, y.value, z.value);
    try {
      var corner = new Vector(start(x.value, align[0]), start(y.value, align[1]), start(z.value, align[2]));
      var placed = new Location(new Plane(corner, Vector.X(), Vector.Z())).apply(box);
      box.close();
      return EvaluationResult.fromShape(placed);
    } catch (error:Dynamic) {
      box.close();
      throw error;
    }
  }
}
