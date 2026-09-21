package cadkit.parametric.features;

import cadkit.Operation;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;

class BooleanFeature extends Feature {
	public final first:Feature;
	public final second:Feature;
	public final operation:BooleanOperation;

	public function new(first:Feature, second:Feature, operation:BooleanOperation) {
		super();
		this.first = first;
		this.second = second;
		this.operation = operation;
	}

	override public function serializationType():String {
		return "boolean";
	}

	override public function dependencies():Array<FeatureId> {
		return [first.id, second.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [first, second];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var left = context.shape(first);
		var right = context.shape(second);
		var result:Operation = switch operation {
			case Fuse: left.fuseOperation(right);
			case Cut: left.cutOperation(right);
			case Common: left.commonOperation(right);
		};
		return EvaluationResult.fromOperation(result);
	}
}
