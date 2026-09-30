package eoat;

import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.component.BomItem;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import haxeon.Equality;
import haxeon.wire.JsonWire;

private class ExampleAirSource extends MachineComponent {
	public function new() {
		super("EXAMPLE-AIR-SOURCE", "Test cell air source", "steel", true);
		addPort({name: "air", kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
	}

	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(10, 10, 10);
}

/** Smoke checks for both generic changer configurations. */
class EndEffectorExampleChecks {
	static function check(value:Bool, message:String):Void if (!value) throw message;

	static function sortedLines(lines:Array<BomItem>):Array<BomItem> {
		var result = lines.copy();
		result.sort((a, b) -> Reflect.compare(a.partNumber, b.partNumber));
		return result;
	}

	public static function run():Void {
		var set = EndEffectorExample.build();
		var rebuiltSet = EndEffectorSet.fromDescription(JsonWire.decode(set.encode()));
		check(Equality.equals(set.describe(), rebuiltSet.describe()),
			"EOAT changer description changed after save and rebuild");
		var short = set.configuration("short");
		var long = set.configuration("long");
		check(short.components().length == 7 && long.components().length == 7,
			"EOAT configuration should include only one tool");
		check(short.billOfMaterials().quantity("VACUUM-GENERATOR") == 1 &&
			long.billOfMaterials().quantity("VACUUM-GENERATOR") == 1,
			"EOAT BOM should include one generator");
		check(short.billOfMaterials().quantity("ISO9409-STYLE-50-4-M6") == 0,
			"Robot flange must not be in the EOAT BOM");
		check(short.massPropertiesAtMount().mass > 0 &&
			long.massPropertiesAtMount().mass > short.massPropertiesAtMount().mass,
			"Longer EOAT should have greater mass");
		var shortContact = short.mountTFrame("contact"), longContact = long.mountTFrame("contact");
		var dx = longContact.raw().x - shortContact.raw().x, dy = longContact.raw().y - shortContact.raw().y,
			dz = longContact.raw().z - shortContact.raw().z;
		check(Math.abs(Math.sqrt(dx * dx + dy * dy + dz * dz) - 50) < 1e-6,
			"Tool contact should move with bar length");
		for (configuration in [short, long]) {
			var rebuilt = EndEffector.fromDescription(JsonWire.decode(configuration.encode()));
			check(Equality.equals(configuration.describe(), rebuilt.describe()),
				"EOAT configuration description changed after save and rebuild");
			// A saved description lists members by sorted id, so a rebuild adds them in that order
			// rather than the order the original was assembled in.
			check(Equality.equals(sortedLines(configuration.billOfMaterials().lines()),
				sortedLines(rebuilt.billOfMaterials().lines())),
				"EOAT configuration BOM changed after save and rebuild");
			check(Equality.equals(configuration.mountTFrame("contact"), rebuilt.mountTFrame("contact")),
				"EOAT contact frame changed after save and rebuild");
			var source = configuration.upstream("tool/cup", "vacuum");
			check(source.port.instanceId == "robot/master" && source.port.portName == "airIn1" && source.external,
				"Cup vacuum should trace to robot-side air");
			var signal = configuration.upstream("tool/changer", "signalIn");
			check(signal.port.instanceId == "robot/master" && signal.port.portName == "signalIn" && signal.external,
				"Tool signal should trace to robot-side signal");
		}
		var cell = new MachineAssembly();
		cell.include("tool", short);
		cell.addComponent("airSource", new ExampleAirSource());
		cell.connectPorts("feed", "airSource", "air", "tool/robot/master", "airIn1");
		cell.exposePort("robotSignal", "tool/robot/master", "signalIn");
		cell.exposePort("lock", "tool/robot/master", "lock");
		check(cell.validate().length == 0, "Cell with an air source validates");
		var supplied = cell.upstream("tool/tool/cup", "vacuum");
		check(!supplied.external && supplied.port.instanceId == "airSource" &&
			supplied.port.portName == "air", "Cell vacuum trace reaches its air source");

		var unfed = new MachineAssembly();
		unfed.include("tool", short);
		unfed.exposePort("robotSignal", "tool/robot/master", "signalIn");
		unfed.exposePort("lock", "tool/robot/master", "lock");
		var failure = "";
		try unfed.validate() catch (error:Dynamic) failure = Std.string(error);
		check(failure.indexOf("tool/tool/cup/vacuum") >= 0 &&
			failure.indexOf("tool/robot/master/airIn1") >= 0 &&
			failure.indexOf("is not supplied") >= 0,
			"Unfed cell reports the complete cup service chain");
	}

	public static function main():Void {
		run();
		trace("EOAT example passed");
	}
}
