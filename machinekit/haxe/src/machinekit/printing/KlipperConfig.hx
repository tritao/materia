package machinekit.printing;

/** One stepper of a Klipper printer: what its `[stepper_x]` section says. */
typedef KlipperStepper = {
	/** The section's suffix: `x`, `y`, `z`, `z1`, `z2`. */
	var name:String;
	/** Millimetres of travel for one turn of the motor's shaft (or of the part the gear ratio turns). */
	var rotationDistance:Float;
	var microsteps:Int;
	/** The driver's RMS run current, amperes. */
	var runCurrent:Float;
	/**
	 * Whether this stepper owns its axis's travel and switch. A stepper that only turns with another on the same axis (the
	 * bed's second and third screws) has just its drive: the rest of its fields are ignored.
	 */
	var leads:Bool;
	var positionMin:Float;
	var positionMax:Float;
	/** Where the axis stands when its endstop trips, millimetres. */
	var positionEndstop:Float;
	/** The endstop is at the far end of the axis, so homing runs towards it in the positive direction. */
	var homesPositive:Bool;
	var homingSpeed:Float;
}

/** A probe beside the nozzle: where it stands relative to it and the nozzle's height above the bed when it trips. */
typedef KlipperProbe = {
	var xOffset:Float;
	var yOffset:Float;
	var zOffset:Float;
}

/** A heater: Klipper's PID gains are the controller's, scaled by 255 (see `scaled`). */
typedef KlipperHeater = {
	/** The thermistor's table: `Generic 3950` is a 100k NTC of beta 3950. */
	var sensorType:String;
	var minTemp:Float;
	var maxTemp:Float;
	var kp:Float;
	var ki:Float;
	var kd:Float;
}

/** The extruder: its stepper's gearing, the filament and nozzle it feeds and the heater that melts it. */
typedef KlipperExtruder = {
	/** Millimetres of filament for one turn of the hobbed gear. */
	var rotationDistance:Float;
	/** The gear train between motor and hobbed gear, motor teeth to gear teeth: `54:18` is a 3:1 reduction. */
	var gearRatio:String;
	var microsteps:Int;
	var runCurrent:Float;
	var nozzleDiameter:Float;
	var filamentDiameter:Float;
	var minExtrudeTemp:Float;
	var pressureAdvance:Float;
	var pressureAdvanceSmoothTime:Float;
	var heater:KlipperHeater;
}

/** Everything a CoreXY printer's `printer.cfg` states that the machine's model knows. */
typedef KlipperPrinter = {
	var name:String;
	/** Limits in millimetres per second and per second squared, as the model's drives allow. */
	var maxVelocity:Float;
	var maxAccel:Float;
	var maxZVelocity:Float;
	var maxZAccel:Float;
	/** The two belt motors (`x`, `y`) and the bed's screws (`z`, `z1`, `z2`), in that order. */
	var steppers:Array<KlipperStepper>;
	var extruder:KlipperExtruder;
	var bed:KlipperHeater;
	var probe:KlipperProbe;
}

/**
 * A Klipper `printer.cfg` for a printer, as text, and a reader of what it says. The pins are placeholders (`PIN_X_STEP`,
 * ...) that name what a board must wire; `pins` lists them with their meaning, and Klipper refuses the file until each
 * is replaced, which is what a config for a board that has not been chosen should do. Lengths are millimetres.
 */
class KlipperConfig {
	/** Klipper keeps PID gains times 255, as a duty of 0 to 255. */
	public static inline var PID_SCALE:Float = 255.0;

	/** A number as the config writes it: no more digits than mean anything, none trailing. */
	public static function number(value:Float):String {
		if (!Math.isFinite(value)) throw "A config value must be finite";
		// Rounded to six decimals in floating point: Math.round returns an Int, which a limit in mm/s^2 times 1e6 overflows.
		var magnitude = Math.abs(value) * 1e6 + 0.5;
		var rounded = (value < 0.0 ? -1.0 : 1.0) * (magnitude - magnitude % 1.0) / 1e6;
		var text = Std.string(rounded == 0.0 ? 0.0 : rounded);
		return StringTools.endsWith(text, ".0") ? text.substr(0, text.length - 2) : text;
	}

	/** The controller's gain as Klipper writes it. */
	public static function scaled(gain:Float):Float return gain * PID_SCALE;

	/** The pins a board must supply, each with what it is for, in the order the file uses them. */
	public static function pins(printer:KlipperPrinter):Array<{pin:String, meaning:String}> {
		var result:Array<{pin:String, meaning:String}> = [{pin: "PIN_MCU_SERIAL", meaning: "the board's serial device"}];
		for (stepper in printer.steppers) {
			var upper = stepper.name.toUpperCase();
			result.push({pin: 'PIN_${upper}_STEP', meaning: 'step of stepper_${stepper.name}'});
			result.push({pin: 'PIN_${upper}_DIR', meaning: 'direction of stepper_${stepper.name}'});
			result.push({pin: 'PIN_${upper}_ENABLE', meaning: 'enable of stepper_${stepper.name}'});
			if (stepper.leads) result.push({pin: 'PIN_${upper}_ENDSTOP', meaning: 'home switch of stepper_${stepper.name}'});
			result.push({pin: 'PIN_${upper}_UART', meaning: 'driver UART of stepper_${stepper.name}'});
		}
		for (suffix in [["STEP", "step of the extruder"], ["DIR", "direction of the extruder"], ["ENABLE", "enable of the extruder"],
				["UART", "driver UART of the extruder"], ["HEATER", "the hotend's cartridge"], ["SENSOR", "the hotend's thermistor"]])
			result.push({pin: 'PIN_EXTRUDER_${suffix[0]}', meaning: suffix[1]});
		result.push({pin: "PIN_BED_HEATER", meaning: "the bed's heater"});
		result.push({pin: "PIN_BED_SENSOR", meaning: "the bed's thermistor"});
		result.push({pin: "PIN_PROBE", meaning: "the probe beside the nozzle"});
		result.push({pin: "PIN_PART_FAN", meaning: "the part-cooling blower"});
		result.push({pin: "PIN_HOTEND_FAN", meaning: "the heatsink fan"});
		return result;
	}

	public static function render(printer:KlipperPrinter):String {
		var lines:Array<String> = ["# Klipper configuration for " + printer.name + ", generated from its model.",
			"# Every PIN_* below is a placeholder for the board's pin; see the pin report.", ""];
		function section(title:String, entries:Array<Array<String>>):Void {
			lines.push("[" + title + "]");
			for (entry in entries) lines.push(entry[0] + ": " + entry[1]);
			lines.push("");
		}
		section("mcu", [["serial", "PIN_MCU_SERIAL"]]);
		section("printer", [["kinematics", "corexy"], ["max_velocity", number(printer.maxVelocity)],
			["max_accel", number(printer.maxAccel)], ["max_z_velocity", number(printer.maxZVelocity)],
			["max_z_accel", number(printer.maxZAccel)]]);
		for (stepper in printer.steppers) {
			var upper = stepper.name.toUpperCase();
			var entries = [["step_pin", 'PIN_${upper}_STEP'], ["dir_pin", 'PIN_${upper}_DIR'], ["enable_pin", '!PIN_${upper}_ENABLE'],
				["microsteps", Std.string(stepper.microsteps)], ["rotation_distance", number(stepper.rotationDistance)]];
			if (stepper.leads) {
				entries.push(["endstop_pin", 'PIN_${upper}_ENDSTOP']);
				entries.push(["position_endstop", number(stepper.positionEndstop)]);
				entries.push(["position_min", number(stepper.positionMin)]);
				entries.push(["position_max", number(stepper.positionMax)]);
				entries.push(["homing_speed", number(stepper.homingSpeed)]);
				if (stepper.homesPositive) entries.push(["homing_positive_dir", "true"]);
			}
			section("stepper_" + stepper.name, entries);
			section("tmc2209 stepper_" + stepper.name, [["uart_pin", 'PIN_${upper}_UART'], ["run_current", number(stepper.runCurrent)]]);
		}
		var extruder = printer.extruder;
		section("extruder", [["step_pin", "PIN_EXTRUDER_STEP"], ["dir_pin", "PIN_EXTRUDER_DIR"], ["enable_pin", "!PIN_EXTRUDER_ENABLE"],
			["microsteps", Std.string(extruder.microsteps)], ["rotation_distance", number(extruder.rotationDistance)],
			["gear_ratio", extruder.gearRatio], ["nozzle_diameter", number(extruder.nozzleDiameter)],
			["filament_diameter", number(extruder.filamentDiameter)], ["heater_pin", "PIN_EXTRUDER_HEATER"],
			["sensor_type", extruder.heater.sensorType], ["sensor_pin", "PIN_EXTRUDER_SENSOR"], ["control", "pid"],
			["pid_Kp", number(scaled(extruder.heater.kp))], ["pid_Ki", number(scaled(extruder.heater.ki))],
			["pid_Kd", number(scaled(extruder.heater.kd))], ["min_temp", number(extruder.heater.minTemp)],
			["max_temp", number(extruder.heater.maxTemp)], ["min_extrude_temp", number(extruder.minExtrudeTemp)],
			["pressure_advance", number(extruder.pressureAdvance)],
			["pressure_advance_smooth_time", number(extruder.pressureAdvanceSmoothTime)]]);
		section("tmc2209 extruder", [["uart_pin", "PIN_EXTRUDER_UART"], ["run_current", number(extruder.runCurrent)]]);
		var bed = printer.bed;
		section("heater_bed", [["heater_pin", "PIN_BED_HEATER"], ["sensor_type", bed.sensorType], ["sensor_pin", "PIN_BED_SENSOR"],
			["control", "pid"], ["pid_Kp", number(scaled(bed.kp))], ["pid_Ki", number(scaled(bed.ki))], ["pid_Kd", number(scaled(bed.kd))],
			["min_temp", number(bed.minTemp)], ["max_temp", number(bed.maxTemp)]]);
		var probe = printer.probe;
		section("probe", [["pin", "PIN_PROBE"], ["x_offset", number(probe.xOffset)], ["y_offset", number(probe.yOffset)],
			["z_offset", number(probe.zOffset)]]);
		section("fan", [["pin", "PIN_PART_FAN"]]);
		section("heater_fan hotend_fan", [["pin", "PIN_HOTEND_FAN"], ["heater", "extruder"], ["heater_temp", "50.0"]]);
		// What Moonraker needs of Klipper to run a print: a place for the files, pause and resume, object exclusion.
		section("virtual_sdcard", [["path", "~/printer_data/gcodes"]]);
		section("pause_resume", []);
		section("exclude_object", []);
		return lines.join("\n");
	}

	/** The pin report as text: one line per placeholder, what it is and where the file uses it. */
	public static function pinReport(printer:KlipperPrinter):String
		return [for (entry in pins(printer)) entry.pin + "  " + entry.meaning].join("\n") + "\n";

	/**
	 * A config as its sections and their keys (a Klipper section header is `[name]` or `[kind name]`; keys are
	 * `key: value` or `key = value`; comments start with `#` or `;`). Enough to read back what `render` writes.
	 */
	public static function parse(text:String):Map<String, Map<String, String>> {
		var result = new Map<String, Map<String, String>>();
		var current:Null<Map<String, String>> = null;
		for (raw in text.split("\n")) {
			var line = StringTools.trim(raw);
			if (line.length == 0 || line.charAt(0) == "#" || line.charAt(0) == ";") continue;
			if (line.charAt(0) == "[") {
				if (line.charAt(line.length - 1) != "]") throw 'Config section header "$line" is not closed';
				var name = line.substr(1, line.length - 2);
				current = result.exists(name) ? result.get(name) : new Map<String, String>();
				result.set(name, current);
				continue;
			}
			var section = current;
			if (section == null) throw 'Config line "$line" is outside any section';
			var colon = line.indexOf(":"), equals = line.indexOf("=");
			var at = colon < 0 ? equals : (equals < 0 ? colon : Std.int(Math.min(colon, equals)));
			if (at <= 0) throw 'Config line "$line" has no key';
			section.set(StringTools.trim(line.substr(0, at)), StringTools.trim(line.substr(at + 1)));
		}
		return result;
	}
}
