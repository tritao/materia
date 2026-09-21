package cadkit.parametric.features;

import cadkit.Operation;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;

/** Applies a constant fillet to every edge of a source solid. */
class FilletFeature extends Feature {
	public final source:Feature;
	public final radius:Parameter;

	public function new(source:Feature, radiusValue:Float) {
		super();
		this.source = source;
		radius = new Parameter(this, "fillet.radius", radiusValue, 0.0);
	}

	override public function serializationType():String {
		return "fillet";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var operation:Operation = context.shape(source).filletOperation(radius.value);
		return EvaluationResult.fromOperation(operation);
	}
}
