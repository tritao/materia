package cadkit.parametric;

/** Optional local connector frames for a definition evaluator. */
interface DefinitionConnectorEvaluator {
	public function connector(definition:Definition, instance:InstanceElement, output:String):Placement;
}
