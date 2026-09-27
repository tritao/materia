package cadkit.parametric;

import cadkit.Shape;

/** Registry boundary that lets higher-level packages provide definition recipes. */
class DefinitionEvaluatorRegistry {
	private static var evaluators:Map<String, DefinitionEvaluator> = new Map();
	private static var connectorEvaluators:Map<String, DefinitionConnectorEvaluator> = new Map();

	public static function register(recipe:String, evaluator:DefinitionEvaluator,
			?connectors:DefinitionConnectorEvaluator):Void {
		if (recipe == null || StringTools.trim(recipe) == "")
			throw new ParametricError("definition recipe name must not be empty");
		if (evaluator == null)
			throw new ParametricError("definition evaluator must not be null");
		var existing = evaluators.get(recipe);
		if (existing == evaluator) {
			if (connectors != null) {
				var previous = connectorEvaluators.get(recipe);
				if (previous != null && previous != connectors)
					throw new ParametricError("definition connector evaluator is already registered: " + recipe);
				connectorEvaluators.set(recipe, connectors);
			}
			return;
		}
		if (existing != null)
			throw new ParametricError("definition evaluator is already registered: " + recipe);
		evaluators.set(recipe, evaluator);
		if (connectors != null) connectorEvaluators.set(recipe, connectors);
	}

	public static function isRegistered(recipe:String):Bool return evaluators.exists(recipe);

	public static function connector(definition:Definition, instance:InstanceElement, output:String):Placement {
		var evaluator = connectorEvaluators.get(definition.recipe);
		if (evaluator == null)
			throw new ParametricError("definition recipe has no connector evaluator: " + definition.recipe);
		if (evaluator.connectorNames(definition, instance).indexOf(output) < 0)
			throw new ParametricError("unknown instance connector: " + output);
		return evaluator.connector(definition, instance, output);
	}

	public static function connectorNames(definition:Definition, instance:InstanceElement):Array<String> {
		var evaluator = connectorEvaluators.get(definition.recipe);
		return evaluator == null ? [] : evaluator.connectorNames(definition, instance);
	}

	public static function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape {
		if (definition.subgraph != null)
			return new FeatureSubgraphEvaluator().evaluate(definition, instance, output);
		var evaluator = evaluators.get(definition.recipe);
		if (evaluator == null)
			throw new ParametricError("unsupported definition recipe: " + definition.recipe);
		return evaluator.evaluate(definition, instance, output);
	}
}
