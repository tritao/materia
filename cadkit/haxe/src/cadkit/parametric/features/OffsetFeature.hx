package cadkit.parametric.features;

import CadKit;
import cadkit.Operation;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;

/** Signed planar-wire offset. Zero is rejected during staged evaluation. */
class OffsetFeature extends Feature {
	public final source:Feature;
	public final distance:Parameter;

	public function new(source:Feature, distance:Float) {
		super();
		this.source = source;
		this.distance = new Parameter(this, "offset.distance", distance, -1e300);
	}

	override public function serializationType():String {
		return "offset";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		return EvaluationResult.fromOperation(new Operation(CadKit.wireOffsetOperationChecked(context.shape(source).borrowHandle(), distance.value)));
	}
}
