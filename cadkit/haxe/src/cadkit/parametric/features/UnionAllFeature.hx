package cadkit.parametric.features;

import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;

/**
 * Several solids fused into one in a single step. Fused as a balanced tree, so each boolean joins two halves of similar size
 * (as `Part.fuseAll` does), instead of growing one result through every step.
 */
class UnionAllFeature extends Feature {
  public final parts:Array<Feature>;

  public function new(parts:Array<Feature>) {
    super();
    if (parts.length < 2) throw new ParametricError("union needs at least two solids");
    this.parts = parts.copy();
  }

  override public function serializationType():String return "union-all";

  override public function dependencies():Array<FeatureId> return [for (part in parts) part.id];

  override public function dependencyFeatures():Array<Feature> return parts.copy();

  override public function evaluate(context:EvaluationContext):EvaluationResult
    return EvaluationResult.fromShape(fuseRange(context, 0, parts.length));

  function fuseRange(context:EvaluationContext, from:Int, to:Int):Shape {
    if (to - from == 1) return context.shape(parts[from]).cloneShape();
    var middle = from + ((to - from) >> 1);
    var left = fuseRange(context, from, middle), right:Null<Shape> = null;
    try {
      right = fuseRange(context, middle, to);
      var result = left.fuse(right);
      left.close();
      right.close();
      return result;
    } catch (error:Dynamic) {
      left.close();
      if (right != null) right.close();
      throw error;
    }
  }
}
