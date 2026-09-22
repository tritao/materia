package cadkit.parametric.features;

import CadKit;
import cadkit.Shape;
import cadkit.Operation;
import cadkit.modeling.Model;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;

class LoftFeature extends Feature {
	private final sections:Array<Feature>;

	public final ruled:Bool;

	public function new(sections:Array<Feature>, ruled:Bool = false) {
		super();
		if (sections.length < 2)
			throw new ParametricError("loft requires two or more wire features");
		this.sections = sections.copy();
		this.ruled = ruled;
	}

	override public function serializationType():String {
		return "loft";
	}

	override public function dependencies():Array<FeatureId> {
		var ids:Array<FeatureId> = [];
		for (section in sections)
			ids.push(section.id);
		return ids;
	}

	override public function dependencyFeatures():Array<Feature> {
		return sections.copy();
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var shapes:Array<Shape> = [];
		for (section in sections)
			shapes.push(context.shape(section));
		return EvaluationResult.fromOperation(new Operation(CadKit.loftOperationChecked(Model.refs(shapes), 1, ruled ? 1 : 0)));
	}
}
