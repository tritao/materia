package cadkit.parametric.features;

import CadKit;
import cadkit.Operation;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;

class SweepFeature extends Feature {
	public final profile:Feature;
	public final path:Feature;

	public function new(profile:Feature, path:Feature) {
		super();
		this.profile = profile;
		this.path = path;
	}

	override public function serializationType():String {
		return "sweep";
	}

	override public function dependencies():Array<FeatureId> {
		return [profile.id, path.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [profile, path];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		return EvaluationResult.fromOperation(new Operation(CadKit.sweepOperationChecked(context.shape(profile).borrowHandle(),
			context.shape(path).borrowHandle())));
	}
}
