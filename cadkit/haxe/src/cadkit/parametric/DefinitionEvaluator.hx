package cadkit.parametric;

import cadkit.Shape;

/** Domain-owned implementation for one registered reusable definition recipe. */
interface DefinitionEvaluator {
	public function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape;
}
