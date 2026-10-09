import machinekit.component.ComponentDetail;
import machinekit.component.MachineKitComponents;
import machinekit.motion.NemaStepper;
import machinekit.printing.Extruder;
import machinekit.printing.Fan;
import machinekit.printing.HeatedBed;
import machinekit.printing.HeaterCartridge;
import machinekit.printing.Hotend;
import machinekit.printing.Nozzle;
import machinekit.printing.Thermistor;
import machinekit.transmission.SpurGear;

/** The printing parts: thermistor curve, heater power, thermal and flow values, extruder rotation, masses and connectors. */
class PrintingPartTests {
	static function close(actual:Float, expected:Float, message:String, tolerance:Float = 1e-9):Void {
		if (!(Math.abs(actual - expected) <= tolerance * Math.max(1.0, Math.abs(expected)))) throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		// The 100k beta-3950 curve: 100k at 25 C, and the usual round numbers at 60, 100 and 210 C.
		var thermistor = new Thermistor();
		close(thermistor.resistance(25), 100000, "100k at 25 C");
		close(thermistor.resistance(60), 100000 * Math.exp(3950 * (1 / 333.15 - 1 / 298.15)), "the beta model at 60 C", 1e-9);
		close(thermistor.resistance(60), 24861.65, "about 24.9k at 60 C", 1e-5);
		close(thermistor.resistance(100), 6975.2, "about 7.0k at 100 C", 1e-4);
		close(thermistor.resistance(210), 6975.2 * Math.exp(3950 * (1 / 483.15 - 1 / 373.15)), "the beta model at 210 C", 1e-4);
		for (celsius in [-10.0, 0.0, 25.0, 60.0, 100.0, 210.0, 285.0])
			close(thermistor.temperature(thermistor.resistance(celsius)), celsius, 'the curve inverts at $celsius C', 1e-9);
		var rejected = false;
		try thermistor.resistance(-300) catch (_:Dynamic) rejected = true;
		if (!rejected) throw "A temperature below absolute zero has no resistance";

		// A 24 V, 40 W cartridge is 14.4 ohms and draws 40 W on 24 V, and a quarter of that on 12.
		var cartridge = new HeaterCartridge();
		close(cartridge.resistance(), 14.4, "24 V / 40 W is 14.4 ohm");
		close(cartridge.power(24), 40, "40 W on its rating");
		close(cartridge.power(12), 10, "a quarter on half the voltage");
		var bed = new HeatedBed();
		close(bed.resistance(), 24 * 24 / 300.0, "the bed heater is V^2 / P");
		close(bed.thickness(), 5.5, "plate and pad");

		// The hotend: an assumed block that melts at most 15 mm^3/s, losing more with the part fan on.
		var hotend = new Hotend();
		close(hotend.loss(210, 25, 0), 0.12 * 185, "heat lost to still air");
		close(hotend.loss(210, 25, 1), 0.17 * 185, "and with the fan at full duty");
		if (!(hotend.loss(210, 25, 0) < cartridge.spec.watts)) throw "the cartridge can hold 210 C in still air";
		close(hotend.maxSpeed(0.45e-3 * 0.2e-3), 15e-9 / 9e-8, "melt rate over a bead area is the speed limit", 1e-9);
		if (!(hotend.spec.minExtrusionTemperature < 210)) throw "210 C is above the cold-extrusion limit";

		// The extruder: a 7.3 mm hob feeds pi * 7.3 mm of filament a turn.
		var extruder = new Extruder();
		close(extruder.rotationPerMillimetre(), 2 / 7.3, "radians of hob per millimetre of filament");
		close(extruder.filamentPerRadian() * extruder.rotationPerMillimetre(), 1.0, "and back");
		close(2 * Math.PI * extruder.filamentPerRadian(), Math.PI * 7.3, "one turn feeds pi times the hob diameter");
		var pinion = new SpurGear(0.5, 18, 6, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 0, 5), gear = new SpurGear(0.5, 54, 6, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 0, 5);
		close(gear.teeth / pinion.teeth, 3, "a 3:1 reduction");

		// The pancake stepper and the nozzle.
		var pancake = NemaStepper.model("17HS08-1004S");
		close(pancake.bodyLength, 20, "a 20 mm pancake");
		var rating:machinekit.motion.StepperMotorRating = cast pancake.rating();
		close(rating.holdingTorque, 0.13, "13 N cm");
		var nozzle = new Nozzle();
		close(nozzle.orificeArea(), Math.PI * 0.04, "a 0.4 mm orifice");

		// Fans: the blower has an outlet connector, the axial fan does not.
		var blower = new Fan("BLOWER-5015-24V"), axial = new Fan("FAN-4010-24V");
		function has(component:machinekit.component.MachineComponent, name:String):Bool {
			for (connector in component.connectors()) if (connector.name == name) return true;
			return false;
		}
		if (!has(blower, "outlet") || has(axial, "outlet")) throw "only the blower has an outlet";

		// Connector frames: the nozzle's tip is at its origin, the hotend seats it at its own origin and carries the other parts.
		close(nozzle.connector("tip").frame.z, 0, "the nozzle tip is at its origin");
		close(nozzle.connector("mount").frame.z, 7, "and its mount 7 mm above");
		close(hotend.connector("filamentEntry").frame.z, 11.5 + 30, "the filament enters at the top of the heatsink");
		close(hotend.connector("cartridgeSeat").frame.y, -10, "the cartridge lies along the block");
		close(extruder.connector("filamentGrip").frame.z, 0, "the filament grip is the extruder's origin");
		close(bed.connector("surface").frame.z, 0, "the bed's surface is its origin");
		close(bed.connector("mount").frame.z, -5.5, "its mount is under the pad");

		// Every part builds, has volume, a mass, and rebuilds from its recipe.
		var registry = MachineKitComponents.defaultRegistry();
		for (component in [cast(thermistor, machinekit.component.MachineComponent), cartridge, hotend, nozzle, extruder, bed, blower, axial]) {
			var part = component.geometry(ComponentDetail.Preview);
			if (!(part.volume() > 0)) throw '${component.designation} has no volume';
			part.close();
			var mass = component.massProperties().mass;
			if (!(mass > 0 && mass < 5)) throw '${component.designation} has an unreasonable mass: $mass kg';
			var recipe = component.componentType();
			if (recipe == null) throw '${component.designation} has no recipe';
			var rebuilt = recipe.create(component.values());
			if (rebuilt.designation != component.designation) throw '${component.designation} did not rebuild from its recipe';
			if (!registry.has(recipe.id)) throw '${recipe.id} is not registered';
		}
		close(bed.massProperties().mass, 0.235 * 0.235 * 0.004 * 2700 + 0.235 * 0.235 * 0.0015 * 2700, "the bed weighs its plate and pad (aluminium density)", 0.05);
		Sys.println("Printing part tests passed");
	}
}
