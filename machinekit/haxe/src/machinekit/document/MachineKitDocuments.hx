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
			var input = switch parameter.type {
				case Length: new DefinitionInput(parameter.name, "length", parameter.unit, number(value));
				case Angle: new DefinitionInput(parameter.name, "angle", parameter.unit, number(value));
				case Count: DefinitionInput.integer(parameter.name, integer(value));
				case Bool: DefinitionInput.boolean(parameter.name, boolean(value));
				case Choice(options): DefinitionInput.token(parameter.name, token(value), options);
				case CatalogDesignation(index):
					DefinitionInput.token(parameter.name, token(value), index.designations());
			};
			inputs.push(input);
		}
		inputs.push(DefinitionInput.token("detail", "preview", ["preview", "envelope"]));
		var outputs = [new DefinitionOutput("body", DefinitionOutput.Geometry)];
		for (name in MachineKitRecipes.toolNames(component)) outputs.push(new DefinitionOutput(name, DefinitionOutput.Tool));
		for (connector in component.connectors()) outputs.push(new DefinitionOutput(connector.name, DefinitionOutput.Connector));
		var definition = document.createDefinition(component.designation, type.id, inputs, outputs);
		definition.restoreProperty("machinekit.type", TypedProperty.text("machinekit.type", type.id));
		definition.restoreProperty("machinekit.partNumber", TypedProperty.text("machinekit.partNumber", component.bom.partNumber));
		definition.restoreProperty("machinekit.material", TypedProperty.text("machinekit.material", component.materialId));
		for (parameter in type.parameters()) switch parameter.type {
			case CatalogDesignation(index):
				var designation = resolved.token(parameter.name);
				var source = index.metadata(designation).source;
				if (source != null) {
					definition.restoreProperty("machinekit.catalog.source",
						TypedProperty.text("machinekit.catalog.source", source));
					definition.restoreProperty("machinekit.catalog.designation",
						TypedProperty.text("machinekit.catalog.designation", designation));
				}
			default:
		}
		return definition;
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

}
