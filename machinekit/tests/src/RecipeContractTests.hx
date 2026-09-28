import cadkit.modeling.Part;
import machinekit.component.Bom.BomItem;
import machinekit.component.ComponentDetail.*;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentParameterType.*;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.MachineKitComponents;
import materia.project.MaterialLibrary;

/** Cross-recipe checks for identity, generated solids, tools, and BOM part numbers. */
class RecipeContractTests {
	public static function run():Void {
		for (type in MachineKitComponents.all()) {
			var seenValues:Map<String, Bool> = [];
			var partNumbers:Map<String, String> = [];
			for (values in cases(type)) {
				var component:MachineComponent;
				try component = type.create(values) catch (error:Dynamic)
					throw '${type.id} cannot build ${type.key(values)}: $error';
				var key = type.key(component.values());
				if (seenValues.exists(key)) continue;
				seenValues.set(key, true);

				var rebuilt = type.create(component.values());
				check(type.key(rebuilt.values()) == key, '${type.id}: values changed after rebuild');
				check(rebuilt.designation == component.designation,
					'${type.id}: designation changed after rebuild (${component.designation} -> ${rebuilt.designation})');
				check(sameBom(component.bom, rebuilt.bom), '${type.id}: BOM changed after rebuild');

				var partNumber = component.bom.partNumber;
				var prior = partNumbers.get(partNumber);
				if (prior != null && prior != key)
					throw '${type.id}: part number "$partNumber" aliases different values';
				partNumbers.set(partNumber, key);

				checkSolid(component.geometry(Envelope), '${type.id} $key Envelope');
				checkSolid(component.geometry(Preview), '${type.id} $key Preview');
				for (tool in component.toolSpecs())
					checkValid(component.tool(tool.name, tool.defaults()), '${type.id} $key tool "${tool.name}"');
			}

			var baseline = type.create();
			var materialOptions = MaterialLibrary.specs();
			for (parameter in type.parameters()) if (parameter.name == "material")
				materialOptions = switch parameter.type {
					case Choice(options): options;
					default: materialOptions;
				};
			for (material in materialOptions) {
				if (material == baseline.materialSpec()) continue;
				var variant = type.create(baseline.values().copy().setToken("material", material));
				check(variant.materialSpec() == material, '${type.id}: material input was ignored');
				var rebuilt = type.create(variant.values());
				check(rebuilt.materialSpec() == material && sameBom(variant.bom, rebuilt.bom),
					'${type.id}: material changed after rebuild');
				check(variant.bom.partNumber != baseline.bom.partNumber,
					'${type.id}: material variants alias the same part number');
			}
		}
	}

	static function cases(type:machinekit.component.ComponentType):Array<ComponentValues> {
		var result = [type.defaults()];
		for (parameter in type.parameters()) switch parameter.type {
			case CatalogDesignation(index):
				for (designation in index.designations())
					result.push(type.defaults().setToken(parameter.name, designation));
			default:
		}
		return result;
	}

	static function sameBom(a:machinekit.component.BomItem, b:machinekit.component.BomItem):Bool
		return a.partNumber == b.partNumber && a.description == b.description && a.material == b.material &&
			a.typeId == b.typeId && a.valuesKey == b.valuesKey && a.quantity == b.quantity;

	static function checkSolid(part:Part, label:String):Void {
		try {
			check(part.valid(), '$label: invalid shape');
			check(part.solidCount() == 1, '$label: expected one solid, got ${part.solidCount()}');
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
		part.close();
	}

	static function checkValid(part:Part, label:String):Void {
		try {
			check(part.valid(), '$label: invalid shape');
			check(part.solidCount() > 0, '$label: expected at least one solid');
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
		part.close();
	}

	static function check(condition:Bool, message:String):Void {
		if (!condition) throw message;
	}
}
