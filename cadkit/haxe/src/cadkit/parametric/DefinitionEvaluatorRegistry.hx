package cadkit.parametric;

import cadkit.Shape;

/** Registry boundary that lets higher-level packages provide definition recipes. */
class DefinitionEvaluatorRegistry {
	private static var evaluators:Map<String, DefinitionEvaluator> = new Map();

	public static function register(recipe:String, evaluator:DefinitionEvaluator):Void {
		if (recipe == null || StringTools.trim(recipe) == "")
			throw new ParametricError("definition recipe name must not be empty");
		if (evaluator == null)
			throw new ParametricError("definition evaluator must not be null");
		if (evaluators.exists(recipe))
			throw new ParametricError("definition evaluator is already registered: " + recipe);
		evaluators.set(recipe, evaluator);
	}

	public static function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape {
		var evaluator = evaluators.get(definition.recipe);
		if (evaluator == null)
			throw new ParametricError("unsupported definition recipe: " + definition.recipe);
		return evaluator.evaluate(definition, instance, output);
	}
}
