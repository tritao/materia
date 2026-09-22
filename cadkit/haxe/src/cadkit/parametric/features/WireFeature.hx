package cadkit.parametric.features;

import CadKit;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;

/** Select the sole wire of a profile; holes/multiple regions are intentionally ambiguous. */
class WireFeature extends Feature {
	public final source:Feature;

	public function new(source:Feature) {
		super();
		this.source = source;
	}

	override public function serializationType():String {
		return "wire";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var shape = context.shape(source);
		if (shape.subshapeCount(CadKit.ShapeKind.Wire) != 1)
			throw new ParametricError("wire feature requires exactly one wire");
		return EvaluationResult.fromShape(shape.subshape(CadKit.ShapeKind.Wire, 0));
	}
}
