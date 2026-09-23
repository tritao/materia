package cadkit.parametric.features;

import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.ParametricError;

/** Self-contained imported STEP source retained as authored document data. */
class ImportedShapeFeature extends Feature {
	public final stepText:String;

	public function new(stepText:String) {
		super();
		if (stepText == null || StringTools.trim(stepText) == "")
			throw new ParametricError("imported STEP text must not be empty");
		this.stepText = stepText;
	}

	override public function serializationType():String
		return "imported-step";

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		context.checkCancelled();
		var imported = Shape.importStepText(stepText);
		try {
			context.checkCancelled();
			return EvaluationResult.fromShape(imported);
		} catch (error:Dynamic) {
			imported.close();
			throw error;
		}
	}
}
