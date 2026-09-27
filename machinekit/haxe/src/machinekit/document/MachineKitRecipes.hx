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
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.HexBolt;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.motion.FlangeBearingHousing;
import machinekit.motion.NemaStepper;
import machinekit.robotics.RobotFlange;

/** CadKit evaluator registration for editable MachineKit single-part recipes. */
class MachineKitRecipes {
	static var evaluators:Map<String, MachineKitRecipeEvaluator> = [];

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

	public static function toolNames(component:MachineComponent):Array<String>
		return MachineKitRecipeEvaluator.toolNames(component);

	public static function component(instance:InstanceElement):MachineComponent {
		var type = typeOrNull(instance.document.definition(instance.definitionId).recipe);
		if (type == null) throw 'Instance is not a MachineKit recipe: ${instance.id.value}';
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
		return type.create(values);
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
			part = tool(component, output);
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

	public static function toolNames(component:MachineComponent):Array<String> {
		if (Std.isOfType(component, DeepGrooveBearing) || Std.isOfType(component, FlangeBearingHousing))
			return ["bearingSeat"];
		if (Std.isOfType(component, SocketHeadCapScrew) || Std.isOfType(component, HexBolt))
			return ["clearanceHole", "tapHole", "counterboreHole"];
		if (Std.isOfType(component, NemaStepper) || Std.isOfType(component, RobotFlange))
			return ["mountingCutout"];
		return [];
	}

	function tool(component:MachineComponent, output:String):Part {
		if (Std.isOfType(component, DeepGrooveBearing) && output == "bearingSeat")
			return cast(component, DeepGrooveBearing).housingSeat();
		if (Std.isOfType(component, FlangeBearingHousing) && output == "bearingSeat")
			return cast(component, FlangeBearingHousing).bearing.housingSeat();
		if (Std.isOfType(component, SocketHeadCapScrew)) {
			var screw:SocketHeadCapScrew = cast component;
			var depth = Math.max(screw.length, screw.spec.counterboreDepth + 1);
			return switch output {
				case "clearanceHole": screw.clearanceHole(depth);
				case "tapHole": screw.tapHole(depth);
				case "counterboreHole": screw.counterboreHole(depth);
				default: throw 'Unknown tool "$output" for ${type.id}';
			};
		}
		if (Std.isOfType(component, HexBolt)) {
			var bolt:HexBolt = cast component;
			var depth = Math.max(bolt.length, bolt.spec.headHeight + 1);
			return switch output {
				case "clearanceHole": bolt.clearanceHole(depth);
				case "tapHole": bolt.tapHole(depth);
				case "counterboreHole": bolt.counterboreHole(depth);
				default: throw 'Unknown tool "$output" for ${type.id}';
			};
		}
		if (Std.isOfType(component, NemaStepper) && output == "mountingCutout")
			return cast(component, NemaStepper).mountingCutout(10);
		if (Std.isOfType(component, RobotFlange) && output == "mountingCutout")
			return cast(component, RobotFlange).mountingCutout(10);
		throw 'Unknown tool "$output" for ${type.id}';
	}
}
