package machinekit.document;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.Document;
import cadkit.parametric.Placement;
import cadkit.parametric.TypedProperty;
import machinekit.component.Bom;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValue;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentParameter;
import machinekit.component.ToolSpec;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Definition construction and BOM extraction for MachineKit document instances. */
class MachineKitDocuments {
	public static function define(document:Document, type:ComponentType, ?values:ComponentValues):Definition {
		MachineKitRecipes.register();
		var resolved = type.resolve(values);
		var component = type.create(resolved);
		var inputs:Array<DefinitionInput> = [];
		for (parameter in type.parameters()) {
			var value = resolved.get(parameter.name);
			inputs.push(definitionInput(parameter.name, parameter, value));
		}
		for (tool in component.toolSpecs()) {
			var defaults = tool.defaults();
			for (parameter in tool.parameters())
				inputs.push(definitionInput(ToolSpec.inputName(tool.name, parameter.name), parameter,
					defaults.get(parameter.name)));
		}
		inputs.push(DefinitionInput.token("detail", "preview", ["preview", "envelope"]));
		var outputs = [new DefinitionOutput("body", DefinitionOutput.Geometry)];
		for (tool in component.toolSpecs()) outputs.push(new DefinitionOutput(tool.name, DefinitionOutput.Tool));
		var definition = document.createDefinition(type.label(), type.id, inputs, outputs);
		definition.restoreProperty("machinekit.type", TypedProperty.text("machinekit.type", type.id));
		return definition;
	}

	public static function partNumber(instance:cadkit.parametric.InstanceElement):String
		return MachineKitRecipes.component(instance).bom.partNumber;

	public static function material(instance:cadkit.parametric.InstanceElement):String
		return MachineKitRecipes.component(instance).materialId;

	public static function catalogSource(instance:cadkit.parametric.InstanceElement):Null<String> {
		var type = MachineKitRecipes.typeOrNull(instance.document.definition(instance.definitionId).recipe);
		if (type == null) return null;
		for (parameter in type.parameters()) switch parameter.type {
			case CatalogDesignation(index):
				var designation:String = cast instance.resolvedValue(parameter.name);
				var source = index.metadata(designation).source;
				if (source != null) return source;
			default:
		}
		return null;
	}

	public static function bom(document:Document):Bom {
		var result = new Bom();
		for (element in document.allElements()) if (element.kind == "instance") {
			var instance:cadkit.parametric.InstanceElement = cast element;
			var definition = document.definition(instance.definitionId);
			if (MachineKitRecipes.typeOrNull(definition.recipe) != null)
				result.addComponent(MachineKitRecipes.component(instance));
		}
		return result;
	}

	/** Convert the kinematics frame convention to a CadKit local placement. */
	public static function placement(frame:AssemblyFrame):Placement {
		var x = AssemblyFrames.transformVector(frame, 1, 0, 0);
		var z = AssemblyFrames.transformVector(frame, 0, 0, 1);
		return new Placement(new Plane(new Vector(frame.x, frame.y, frame.z),
			new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z)));
	}

	static function number(value:ComponentValue):Float return switch value {
		case Number(v): v;
		default: throw "Expected numeric component input";
	};
	static function integer(value:ComponentValue):Int return switch value {
		case Integer(v): v;
		default: throw "Expected integer component input";
	};
	static function boolean(value:ComponentValue):Bool return switch value {
		case Boolean(v): v;
		default: throw "Expected boolean component input";
	};
	static function token(value:ComponentValue):String return switch value {
		case Token(v): v;
		default: throw "Expected token component input";
	};

	static function definitionInput(name:String, parameter:ComponentParameter, value:ComponentValue):DefinitionInput
		return switch parameter.type {
			case Scalar: new DefinitionInput(name, "scalar", "1", number(value));
			case Length: new DefinitionInput(name, "length", parameter.unit, number(value));
			case Angle: new DefinitionInput(name, "angle", parameter.unit, number(value));
			case Count: DefinitionInput.integer(name, integer(value));
			case Bool: DefinitionInput.boolean(name, boolean(value));
			case Text: new DefinitionInput(name, "text", "1", token(value));
			case Choice(options): DefinitionInput.token(name, token(value), options);
			case CatalogDesignation(index): DefinitionInput.token(name, token(value), index.designations());
		};

}
