package cadkit.parametric;

import cadkit.Shape;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyRecord.AssemblyConnector;

/** Evaluates nested assembly instances from their scoped child instances. */
class AssemblyMemberEvaluator implements DefinitionEvaluator implements DefinitionConnectorEvaluator {
	public static inline var RECIPE:String = "cadkit.assembly-member";
	public static inline var NESTED_RECIPE:String = "cadkit.assembly-nested";
	static var registered:Bool = false;

	public static function register():Void {
		if (registered) return;
		var evaluator = new AssemblyMemberEvaluator();
		DefinitionEvaluatorRegistry.register(RECIPE, evaluator, evaluator);
		DefinitionEvaluatorRegistry.register(NESTED_RECIPE, evaluator, evaluator);
		registered = true;
	}

	public function new() {}

	public function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape {
		if (definition.recipe != NESTED_RECIPE) return Shape.box(0.001, 0.001, 0.001);
		var scopeProperty = definition.property("cadkit.assembly.subdefinition");
		var ownerProperty = instance.property("cadkit.assembly.owner");
		if (scopeProperty == null || ownerProperty == null)
			return Shape.box(0.001, 0.001, 0.001);
		var scope:String = cast scopeProperty.value;
		var owner:PersistentReference = cast ownerProperty.value;
		var result:Null<Shape> = null;
		try {
			for (element in instance.document.allElements()) if (element.kind == "instance") {
				var childScope = element.property("cadkit.assembly.scope");
				var childOwner = element.property("cadkit.assembly.owner");
				if (childScope == null || childOwner == null || childScope.value != scope) continue;
				var reference:PersistentReference = cast childOwner.value;
				if (reference.documentId != owner.documentId || reference.targetId != owner.targetId) continue;
				var child:InstanceElement = cast element;
				var childDefinition = instance.document.definition(child.definitionId);
				var geometry = instance.document.definitionOutput(child,
					childDefinition.primaryGeometryOutput().name);
				var placed = child.localPlacement.location.apply(geometry);
				if (result == null) result = placed;
				else {
					var combined:Shape;
					try combined = result.fuse(placed) catch (error:Dynamic) {
						placed.close();
						throw error;
					}
					result.close();
					placed.close();
					result = combined;
				}
			}
			if (result == null) throw new ParametricError('nested assembly "$scope" has no child instances');
			return result;
		} catch (error:Dynamic) {
			if (result != null) result.close();
			throw error;
		}
	}

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
