package cadkit.parametric.features;

import cadkit.modeling.Part;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;

/**
 * `target` less several tools in one step. The tools may overlap each other (a bore and the slit that opens it): they go to
 * one native multi-tool cut, which intersects every tool with the target and with each other, unlike one boolean against a
 * compound of tools. In the history the tools are this step's sub-steps.
 */
class CutAllFeature extends Feature {
  public final target:Feature;
  public final tools:Array<Feature>;

  public function new(target:Feature, tools:Array<Feature>) {
    super();
    if (tools.length == 0) throw new ParametricError("a cut needs at least one tool");
    this.target = target;
    this.tools = tools.copy();
  }

  override public function serializationType():String return "cut-all";

  override public function dependencies():Array<FeatureId> return [target.id].concat([for (tool in tools) tool.id]);

  override public function dependencyFeatures():Array<Feature> return [target].concat(tools);

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var body = new Part(context.shape(target).cloneShape());
    var cutters:Array<Part> = [];
    try {
      for (tool in tools) cutters.push(new Part(context.shape(tool).cloneShape()));
      var cut = body.subtractAll(cutters);
      var shape = cut.shape.cloneShape();
      cut.close();
      body.close();
      for (c in cutters) c.close();
      return EvaluationResult.fromShape(shape);
    } catch (error:Dynamic) {
      body.close();
      for (c in cutters) c.close();
      throw error;
    }
  }
}
