package cadkit.parametric;

import cadkit.Shape;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyRecord.AssemblyConnector;

/** Geometry placeholder and connector frames for non-recipe assembly members. */
class AssemblyMemberEvaluator implements DefinitionEvaluator implements DefinitionConnectorEvaluator {
	public static inline var RECIPE:String = "cadkit.assembly-member";
	static var registered:Bool = false;

	public static function register():Void {
		if (registered) return;
		var evaluator = new AssemblyMemberEvaluator();
		DefinitionEvaluatorRegistry.register(RECIPE, evaluator, evaluator);
		registered = true;
	}

	public function new() {}

	public function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape
		return Shape.box(0.001, 0.001, 0.001);

	public function connectorNames(definition:Definition, instance:InstanceElement):Array<String>
		return [for (connector in connectors(definition)) connector.name];

	public function connector(definition:Definition, instance:InstanceElement, output:String):Placement {
		for (candidate in connectors(definition)) if (candidate.name == output)
			return PlacementFrames.fromAssemblyFrame(candidate.frame);
		throw 'Unknown assembly connector "$output"';
	}

	static function connectors(definition:Definition):Array<AssemblyConnector> {
		var property = definition.property("cadkit.assembly.connectors");
		if (property == null) return [];
		var encoded:String = cast property.value;
		return JsonWire.decode(encoded);
	}
}
