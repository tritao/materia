package cadkit.parametric.features;

import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;

class CylinderFeature extends Feature {
	public final radius:Parameter;
	public final height:Parameter;

	public function new(radiusValue:Float, heightValue:Float) {
		super();
		radius = new Parameter(this, "cylinder.radius", radiusValue, 0.0);
		height = new Parameter(this, "cylinder.height", heightValue, 0.0);
	}

	override public function serializationType():String {
		return "cylinder";
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		return EvaluationResult.fromShape(Shape.cylinder(radius.value, height.value));
	}
}
