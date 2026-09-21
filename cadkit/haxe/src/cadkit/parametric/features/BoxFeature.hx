package cadkit.parametric.features;

import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;

class BoxFeature extends Feature {
	public final width:Parameter;
	public final depth:Parameter;
	public final height:Parameter;

	public function new(widthValue:Float, depthValue:Float, heightValue:Float) {
		super();
		width = new Parameter(this, "box.width", widthValue, 0.0);
		depth = new Parameter(this, "box.depth", depthValue, 0.0);
		height = new Parameter(this, "box.height", heightValue, 0.0);
	}

	override public function serializationType():String {
		return "box";
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		return EvaluationResult.fromShape(Shape.box(width.value, depth.value, height.value));
	}
}
