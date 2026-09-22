package cadkit.parametric.features;

import cadkit.parametric.ParameterKind;
import cadkit.Geometry;
import cadkit.Operation;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;

/** Revolves a face or wire feature around an arbitrary axis. */
class RevolveFeature extends Feature {
	public final source:Feature;
	public final originX:Parameter;
	public final originY:Parameter;
	public final originZ:Parameter;
	public final axisX:Parameter;
	public final axisY:Parameter;
	public final axisZ:Parameter;
	public final angle:Parameter;

	public function new(
		source:Feature,
		originXValue:Float,
		originYValue:Float,
		originZValue:Float,
		axisXValue:Float,
		axisYValue:Float,
		axisZValue:Float,
		angleValue:Float) {
		super();
		this.source = source;
		// Axis components may be zero; OCCT validates that the complete axis is non-zero.
		originX = new Parameter(this, "revolve.originX", originXValue, -1.0e300, false, 1e300, ParameterKind.Length);
		originY = new Parameter(this, "revolve.originY", originYValue, -1.0e300, false, 1e300, ParameterKind.Length);
		originZ = new Parameter(this, "revolve.originZ", originZValue, -1.0e300, false, 1e300, ParameterKind.Length);
		axisX = new Parameter(this, "revolve.axisX", axisXValue, -1.0e300);
		axisY = new Parameter(this, "revolve.axisY", axisYValue, -1.0e300);
		axisZ = new Parameter(this, "revolve.axisZ", axisZValue, -1.0e300);
		angle = new Parameter(this, "revolve.angle", angleValue, -1.0e300, false, 1e300, ParameterKind.Angle);
	}

	override public function serializationType():String {
		return "revolve";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var operation:Operation = context.shape(source).revolveOperation(
			Geometry.vec3(originX.value, originY.value, originZ.value),
			Geometry.vec3(axisX.value, axisY.value, axisZ.value),
			angle.value);
		return EvaluationResult.fromOperation(operation);
	}
}
