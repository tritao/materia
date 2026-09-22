package cadkit.parametric.features;

import cadkit.parametric.ParameterKind;
import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;

/** Rigid rotation around an arbitrary axis and pivot. */
class RotationFeature extends Feature {
	public final source:Feature;
	public final pivot:Vector;
	public final axis:Vector;
	public final angle:Parameter;

	public function new(source:Feature, pivot:Vector, axis:Vector, angleValue:Float) {
		super();
		this.source = source;
		this.pivot = pivot;
		this.axis = axis.normalized();
		angle = new Parameter(this, "rotation.angle", angleValue, -1e300, false, 1e300, ParameterKind.Angle);
	}

	override public function serializationType():String {
		return "rotation";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var placement = Location.rotation(new Axis(pivot, axis), angle.value);
		return EvaluationResult.fromOperation(placement.applyOperation(context.shape(source)));
	}
}
