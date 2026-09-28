package eoat;

/** Smoke checks for both generic changer configurations. */
class EndEffectorExampleChecks {
	static function check(value:Bool, message:String):Void if (!value) throw message;

	public static function run():Void {
		var set = EndEffectorExample.build();
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
		var dx = longContact.x - shortContact.x, dy = longContact.y - shortContact.y,
			dz = longContact.z - shortContact.z;
		check(Math.abs(Math.sqrt(dx * dx + dy * dy + dz * dz) - 50) < 1e-6,
			"Tool contact should move with bar length");
		for (configuration in [short, long]) {
			var source = configuration.upstream("tool/cup", "vacuum");
			check(source.instanceId == "robot/master" && source.portName == "airIn1",
				"Cup vacuum should trace to robot-side air");
			var signal = configuration.upstream("tool/changer", "signalIn");
			check(signal.instanceId == "robot/master" && signal.portName == "signalIn",
				"Tool signal should trace to robot-side signal");
		}
	}

	public static function main():Void {
		run();
		trace("EOAT example passed");
	}
}
