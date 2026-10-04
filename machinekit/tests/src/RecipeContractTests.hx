import cadkit.modeling.Part;
import haxeon.Equality;
import cadkit.modeling.Vector;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.SteppedShaft;
import machinekit.pneumatic.RoutedHose;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.component.PortInterface;
import machinekit.component.Bom;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;
import machinekit.standard.ParallelKey;
import machinekit.component.Bom.BomItem;
import machinekit.component.ComponentDetail.*;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentParameterType.*;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.MachineKitComponents;
import materia.project.MaterialLibrary;

/** Cross-recipe checks for identity, generated solids, tools, and BOM part numbers. */
private class UnregisteredCup extends SuctionCup {
	public function new() super(20, 10);
}

class RecipeContractTests {
	public static function run():Void {
		check(new UnregisteredCup().componentType() == null,
			"Unregistered cup subclass must remain code-only");
		var hoseBom = new Bom();
		hoseBom.addComponent(new RoutedHose("STOCK-L100", [new Vector(), new Vector(0, 0, 100)], 4, 2, 0.011));
		hoseBom.addComponent(new RoutedHose("STOCK-L100", [new Vector(), new Vector(50, 0, 0),
			new Vector(50, 50, 0)], 4, 2, 0.011));
		check(hoseBom.lines().length == 1 && hoseBom.lines()[0].quantity == 2,
			"Equal-length hose routes must share one BOM line");
		var cupBom = new Bom();
		cupBom.addComponent(new SuctionCup(40, 18));
		cupBom.addComponent(new SuctionCup(40, 18, null, null, null, PushIn(4)));
		check(cupBom.lines().length == 2, "Different cup fittings must have different BOM identities");
		checkRebuild(new SuctionCup(40, 18, 900, 0.2, "CUP-CUSTOM", Thread("G1/8"), "Custom cup"));
		checkRebuild(new VacuumGenerator(80, "GEN-CUSTOM", PushIn(4), Thread("G1/4"), "Custom generator"));
		for (type in MachineKitComponents.defaultRegistry().all()) {
			for (parameter in type.parameters()) switch parameter.type {
				case CatalogDesignation(index):
					for (designation in index.designations()) {
						var selection = type.create(type.defaults().setToken(parameter.name, designation));
						check(selection.values().token(parameter.name) == designation,
							'${type.id}: ${parameter.name} lost catalog entry $designation');
						check(type.create(selection.values()).values().token(parameter.name) == designation,
							'${type.id}: ${parameter.name} changed catalog entry $designation on rebuild');
					}
				default:
			}
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
				check(Equality.equals(component.connectors(), rebuilt.connectors()),
					'${type.id}: connectors changed after rebuild');
				check(Equality.equals(component.ports(), rebuilt.ports()),
					'${type.id}: ports changed after rebuild');
				check(Equality.equals(facetWords(component), facetWords(rebuilt)),
					'${type.id}: facets changed after rebuild');

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
		checkRebuild(new RoutedHose("TEST-ROUTE", [new Vector(), new Vector(0, 0, 100),
			new Vector(100, 0, 100)], 4, 2, 0.011));
		checkRebuild(new SchmalzVacuumHose("10.07.09.00001", [new Vector(),
			new Vector(0, 0, 100), new Vector(100, 0, 100)]));
		checkRebuild(new ShaftCoupling(5, 8, null, null, [{z: 6, angle: 0}]));
		checkRebuild(new SteppedShaft([{diameter: 8, length: 51.5}, {diameter: 6, length: 8.5}],
			[{name: "bearing", z: 10}], [{name: "key", z0: 52, key: ParallelKey.forShaft(6, 6)}],
			[{name: "ring", z0: 50, width: 1.2, diameter: 7.6}]));
		checkRebuild(new SteppedShaft([{diameter: 12, length: 20}, {diameter: 8, length: 20}],
			null, null, null, {inputChamfer: 1, outputThread: {diameter: 6, pitch: 1, length: 5}}));
		}
	}

	static function checkRebuild(component:MachineComponent):Void {
		var type:machinekit.component.ComponentType = cast component.type;
		check(type != null, '${component.designation}: recipe is missing');
		var rebuilt = type.create(component.values());
		check(component.designation == rebuilt.designation, '${component.designation}: custom designation changed');
		check(Equality.equals(component.connectors(), rebuilt.connectors()),
			'${component.designation}: custom connectors changed');
		check(Equality.equals(component.ports(), rebuilt.ports()),
			'${component.designation}: custom ports changed');
		check(Equality.equals(facetWords(component), facetWords(rebuilt)),
			'${component.designation}: custom facets changed');
	}

	static function facetWords(component:machinekit.component.MachineComponent):Array<String>
		return [for (facet in component.facets()) facet.describe()];

	static function cases(type:machinekit.component.ComponentType):Array<ComponentValues> {
		var result = [type.defaults()];
		for (parameter in type.parameters()) switch parameter.type {
			case CatalogDesignation(index):
				for (designation in index.designations())
					result.push(type.defaults().setToken(parameter.name, designation));
			case Choice(options) if (parameter.name == "servo"):
				// Servo selections are complete catalog configurations. Other choices can
				// depend on companion values, such as a thread's interface name and size.
				for (option in options) result.push(type.defaults().setToken(parameter.name, option));
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
