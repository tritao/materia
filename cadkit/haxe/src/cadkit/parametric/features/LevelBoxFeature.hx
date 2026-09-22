package cadkit.parametric.features;

import cadkit.Shape;
import cadkit.parametric.ElementReference;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;

/** Wall-local box whose height is the span between two levels and offsets. */
class LevelBoxFeature extends Feature {
	public final width:Parameter;
	public final depth:Parameter;
	public final base:ElementReference;
	public final top:ElementReference;
	public final baseOffset:Parameter;
	public final topOffset:Parameter;

	public function new(width:Float, depth:Float, base:ElementReference, top:ElementReference, baseOffset:Float = 0, topOffset:Float = 0) {
		super();
		this.width = new Parameter(this, "level-box.width", width, 0, false, 1e300, ParameterKind.Length);
		this.depth = new Parameter(this, "level-box.depth", depth, 0, false, 1e300, ParameterKind.Length);
		this.base = base;
		this.top = top;
		this.baseOffset = new Parameter(this, "level-box.baseOffset", baseOffset, -1e300, false, 1e300, ParameterKind.Length);
		this.topOffset = new Parameter(this, "level-box.topOffset", topOffset, -1e300, false, 1e300, ParameterKind.Length);
	}

	override public function serializationType():String
		return "level-box";

	override public function datumDependencies():Array<String>
		return [base.elementId.value, top.elementId.value];

	public function baseElevation(document:cadkit.parametric.Document):Float
		return document.levelElevation(base) + baseOffset.value;

	public function height(document:cadkit.parametric.Document):Float {
		var value = document.levelElevation(top) + topOffset.value - baseElevation(document);
		if (value <= 0)
			throw new ParametricError("top level must be above base level");
		return value;
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult
		return EvaluationResult.fromShape(Shape.box(width.value, depth.value, height(context.owner())));
}
