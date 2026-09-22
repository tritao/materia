package cadkit.parametric.features;

import cadkit.parametric.ParameterKind;
import cadkit.Geometry;
import cadkit.Operation;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;

class TransformFeature extends Feature {
	public final source:Feature;
	public final x:Parameter;
	public final y:Parameter;
	public final z:Parameter;

	public function new(source:Feature, xValue:Float, yValue:Float, zValue:Float) {
		super();
		this.source = source;
		// Haxeon has no POSITIVE_INFINITY constant; this finite floor is far
		// outside practical CAD coordinates while preserving unrestricted motion.
		x = new Parameter(this, "transform.x", xValue, -1.0e300, false, 1e300, ParameterKind.Length);
		y = new Parameter(this, "transform.y", yValue, -1.0e300, false, 1e300, ParameterKind.Length);
		z = new Parameter(this, "transform.z", zValue, -1.0e300, false, 1e300, ParameterKind.Length);
	}

	override public function serializationType():String {
		return "transform";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var operation = context.shape(source).translateOperation(
			Geometry.vec3(x.value, y.value, z.value));
		return EvaluationResult.fromOperation(operation);
	}
}
