package machinekit.document;

import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionConnectorEvaluator;
import cadkit.parametric.DefinitionEvaluator;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.Placement;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.MachineKitComponents;

/** CadKit evaluator registration for editable MachineKit single-part recipes. */
class MachineKitRecipes {
	static var evaluators:Map<String, MachineKitRecipeEvaluator> = [];
	static var components:Map<String, {key:String, component:MachineComponent}> = [];

	public static function register():Void {
		for (type in MachineKitComponents.all()) {
			var evaluator = evaluators.get(type.id);
			if (evaluator == null) {
				evaluator = new MachineKitRecipeEvaluator(type);
				evaluators.set(type.id, evaluator);
			}
			DefinitionEvaluatorRegistry.register(type.id, evaluator, evaluator);
		}
	}

	public static function typeOrNull(id:String):Null<ComponentType> {
		for (type in MachineKitComponents.all()) if (type.id == id) return type;
		return null;
	}

	public static function component(instance:InstanceElement):MachineComponent {
		var type = typeOrNull(instance.document.definition(instance.definitionId).recipe);
		if (type == null) throw 'Instance is not a MachineKit recipe: ${instance.id.value}';
		var identity = instance.document.id.value + ":" + instance.id.value;
		var key = instance.document.resolvedInputKey(instance);
		var cached = components.get(identity);
		if (cached != null && cached.key == key) return cached.component;
		var values = new ComponentValues();
		for (parameter in type.parameters()) {
			var raw = instance.resolvedValue(parameter.name);
			switch parameter.type {
				case Length | Angle: values.setNumber(parameter.name, cast raw);
				case Count: values.setInteger(parameter.name, cast raw);
				case Bool: values.setBoolean(parameter.name, cast raw);
				case Choice(_) | CatalogDesignation(_): values.setToken(parameter.name, cast raw);
			}
		}
		var built = type.create(values);
		components.set(identity, {key: key, component: built});
		return built;
	}
}

private class MachineKitRecipeEvaluator implements DefinitionEvaluator implements DefinitionConnectorEvaluator {
	final type:ComponentType;

	public function new(type:ComponentType) this.type = type;

	public function evaluate(definition:Definition, instance:InstanceElement, output:String):Shape {
		var component = MachineKitRecipes.component(instance);
		var part:Part;
		if (output == "body") {
			var detail = instance.resolvedToken("detail") == "envelope" ? ComponentDetail.Envelope : ComponentDetail.Preview;
			part = component.geometry(detail);
		} else {
			part = component.tool(output, 0);
		}
		var result = part.shape.cloneShape();
		part.close();
		return result;
	}

	public function connector(definition:Definition, instance:InstanceElement, output:String):Placement {
		var component = MachineKitRecipes.component(instance);
		for (connector in component.connectors())
			if (connector.name == output)
				return MachineKitDocuments.placement(connector.frame);
		throw 'Unknown connector "$output" for ${type.id}';
	}

	public function connectorNames(definition:Definition, instance:InstanceElement):Array<String>
		return [for (connector in MachineKitRecipes.component(instance).connectors()) connector.name];

}
