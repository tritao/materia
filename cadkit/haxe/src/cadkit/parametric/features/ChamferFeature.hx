package cadkit.parametric.features;

import cadkit.Operation;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;

/** Applies a symmetric chamfer to every edge of a source solid. */
class ChamferFeature extends Feature {
	public final source:Feature;
	public final distance:Parameter;

	public function new(source:Feature, distanceValue:Float) {
		super();
		this.source = source;
		distance = new Parameter(this, "chamfer.distance", distanceValue, 0.0);
	}

	override public function serializationType():String {
		return "chamfer";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var operation:Operation = context.shape(source).chamferOperation(distance.value);
		return EvaluationResult.fromOperation(operation);
	}
}
