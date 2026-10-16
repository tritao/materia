import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import haxe.io.Bytes;
import machinekit.printing.KlipperConfig;
import motionkit.robot.MotorSpaceConstraints;
import materia.project.SceneArtifact;
import robotkit.device.DeviceBinding;
import robotkit.model.SteadyLoads;
import sys.FileSystem;
import sys.io.File;

/**
 * The printer's Klipper config against the model it was made from: the rotation distances of its belts, screws and
 * extruder gears, the limits its drives give, the heaters' gains and the pins it leaves for a board; read back from the
 * text; and the export directory it is written to with the demo bracket's G-code.
 */
class PrinterKlipperTests {
	static var assertions = 0;

	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok) throw message;
	}

	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-9):Void
		check(Math.abs(actual - expected) <= tolerance, '$message: expected $expected, got $actual');

	public static function run():Void {
		assertions = 0;
		var printer = new CoreXyPrinter();
		var customFrontEnd = new machinekit.control.ThermistorInput(2200,new machinekit.power.DcVoltage(5.0),2,30,20,4);
		var material = new machinekit.printing.PrintMaterialSpec("Custom feedstock / calibrated setup",0.00173,180,0.065);
		var customPrinter = new CoreXyPrinter(customFrontEnd,material);
		var customJob = CoreXyPrinterPreview.CoreXyPrinterJob.job(customPrinter,"G90\n");
		near(customJob.filamentDiameter,0.00173,"job uses the selected feedstock diameter");
		near(customJob.minExtrusionTemperature,180,"job uses the material's cold-extrusion threshold");
		near(customJob.meltLag,0.065,"job uses the material/setup response lag");
		near(customJob.pressureAdvance,0.065,"default advance follows the selected response lag");
		var materialJob = CoreXyPrinterPreview.CoreXyPrinterJob.smallTab(customPrinter);
		var materialController = new cnckit.CncController(Printer,null,new cnckit.PrinterSettings(material.filamentDiameter));
		var materialRead = cnckit.CncCompiler.compileDetailed(materialJob.source,materialController);
		var materialPart = camkit.additive.TestParts.smallTab();
		var materialSlice = camkit.additive.Slicer.slice(materialPart,new camkit.additive.SlicerSettings());
		materialPart.close();
		check(Math.abs(materialRead.program.depositedVolume()/materialSlice.program.depositedVolume()-1)<0.001,
			"Material-aware job G-code preserves the sliced volume");
		for (heater in customJob.heaters) {
			near(heater.pullupOhm,2200,"job reads the model's pull-up");
			near(heater.vRef,5.0,"job reads the model's ADC reference");
		}
		check(customJob.heaters[0].slot==CoreXyPrinter.HOTEND_THERMISTOR_CHANNEL &&
			customJob.heaters[1].slot==CoreXyPrinter.BED_THERMISTOR_CHANNEL,"job slots match the circuit wiring");
		var scene = SceneArtifact.decode(CoreXyPrinterPreview.printer());
		var job = scene.printing;
		if (job == null || scene.assemblyDefinition == null) throw "the printer example carries no print job";
		var model = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition, AssemblyPhysicalPartView.fromSceneArtifact(scene),
			scene.assemblyState).model;
		var planning = DeviceBinding.planningModel(model, CoreXyPrinter.STEP_TICK_HZ);
		var limits = [for (id in ["x", "y", "z"]) {
			// The motors' full rate (the plan bounds a move by both motors' rows, `MotorSpaceConstraints`).
			var found = planning.coupledLimits(id, new SteadyLoads(), true);
			{speed: found.requireVelocity(), acceleration: found.requireAcceleration()};
		}];
		// Klipper has one speed for every direction: the motors' rows give it, at the 45 degree move that turns one motor flat out.
		var rows = MotorSpaceConstraints.of(planning, ["x", "y"]);
		var cartesian = rows.inscribedSpeed();
		// And one acceleration: the motors' torques on the worst direction, from the axes' masses.
		var masses = [for (load in robotkit.model.DriveLoads.of(planning, new SteadyLoads(), ["x", "y"])) load.mass];
		var inscribed = rows.inscribedAcceleration(masses);
		check(inscribed > 0 && inscribed < Math.POSITIVE_INFINITY, 'the motors give an acceleration in the worst direction: $inscribed');
		// Every direction keeps both motors inside their torque at that acceleration, and the worst one loads a motor to its limit.
		var peak = 0.0;
		var motors = [for (i in 0...rows.rows.length) if (rows.efforts[i] != null) i];
		check(motors.length == 2, 'the two belt motors have torque limits (${motors.length})');
		// W = R^-T for the square CoreXY rows: the torques that do the work of an axis force.
		var r = [for (i in motors) rows.rows[i]];
		var determinant = r[0][0] * r[1][1] - r[0][1] * r[1][0];
		var inverseTransposed = [[r[1][1] / determinant, -r[1][0] / determinant], [-r[0][1] / determinant, r[0][0] / determinant]];
		for (degree in 0...360) {
			var d = [Math.cos(degree * Math.PI / 180.0), Math.sin(degree * Math.PI / 180.0)];
			for (k in 0...2) {
				var torque = inscribed * Math.abs(inverseTransposed[k][0] * masses[0] * d[0] + inverseTransposed[k][1] * masses[1] * d[1]);
				var effort = rows.efforts[motors[k]];
				if (effort != null) peak = Math.max(peak, torque / effort);
			}
		}
		check(peak <= 1.0 + 1e-9 && peak > 0.999, 'at the inscribed acceleration the worst direction loads a motor to its limit: ${peak}');
		Sys.println('Printer Klipper: max_velocity ${Math.round(cartesian * 1000)} mm/s and max_accel ${Math.round(inscribed * 1000)} mm/s^2 in the worst direction (the axes alone: ${Math.round(Math.min(limits[0].acceleration, limits[1].acceleration) * 1000)})');
		var klipper = CoreXyPrinterKlipper.printer(printer, job, limits[0], limits[1], limits[2], cartesian, inscribed);
		check(klipper.maxAccel <= Math.min(limits[0].acceleration, limits[1].acceleration) * 1000.0 + 1e-9 && klipper.maxAccel <= inscribed * 1000.0 + 1e-9,
			'max_accel is no more than the weaker axis or the motors in the worst direction: ${klipper.maxAccel}');
		var text = KlipperConfig.render(klipper);
		var config = KlipperConfig.parse(text);
		var customConfig = KlipperConfig.parse(KlipperConfig.render(CoreXyPrinterKlipper.printer(customPrinter,customJob,
			limits[0],limits[1],limits[2],cartesian,inscribed)));
		var customExtruder = customConfig.get("extruder");
		if (customExtruder == null) throw "Material config lost its extruder";
		var materialDiameter = customExtruder.get("filament_diameter"), materialThreshold = customExtruder.get("min_extrude_temp");
		if (materialDiameter == null || materialThreshold == null) throw "Material config lost feedstock properties";
		near(Std.parseFloat(materialDiameter),1.73,"Klipper filament diameter comes from the job in millimetres");
		near(Std.parseFloat(materialThreshold),180,"Klipper cold-extrusion guard comes from the selected material");
		for (name in ["extruder","heater_bed"]) {
			var heaterSection = customConfig.get(name);
			if (heaterSection == null) throw "Custom config lost a heater";
			var pullup = heaterSection.get("pullup_resistor");
			if (pullup == null) throw "Custom config lost its pull-up";
			near(Std.parseFloat(pullup),2200,"custom circuit survives Klipper export");
		}

		// The belts, the screws and the extruder's gears, in Klipper's terms.
		function section(name:String):Map<String, String> {
			var found = config.get(name);
			if (found == null) throw 'the config has no section [$name]';
			return found;
		}
		function value(name:String, key:String):Float return Std.parseFloat(section(name).get(key));
		for (name in ["x", "y"]) near(value("stepper_" + name, "rotation_distance"), 40.0, 'a GT2 belt on a 20-tooth pulley moves 40 mm a turn ($name)', 1e-5);
		for (name in ["z", "z1", "z2"]) near(value("stepper_" + name, "rotation_distance"), 8.0, 'a Tr8x2 screw of four starts lifts 8 mm a turn ($name)', 1e-9);
		near(value("extruder", "rotation_distance"), Math.PI * 7.3, "the hobbed gear moves pi times its diameter of filament a turn", 1e-5);
		check(section("extruder").get("gear_ratio") == "54:18", "the extruder gears are a 54:18 pair");
		near(value("extruder", "nozzle_diameter"), 0.4, "the nozzle");
		near(value("extruder", "filament_diameter"), 1.75, "the filament");
		for (name in ["x", "y", "z", "z1", "z2"]) check(section("stepper_" + name).get("microsteps") == "16", 'sixteen microsteps ($name)');
		check(section("stepper_z").get("homing_positive_dir") == "true" && section("stepper_x").get("homing_positive_dir") == null,
			"the bed homes at the bottom of its travel, the belts at the low ends");
		near(value("stepper_x", "position_max"), 220.0, "the belts' travel");
		near(value("stepper_z", "position_endstop"), 220.0, "the bed's switch is at the bottom of its travel");
		for (name in ["z1", "z2"]) check(section("stepper_" + name).get("endstop_pin") == null && section("stepper_" + name).get("position_max") == null,
			'the second and third screws turn with the first and have no switch of their own ($name)');
		check(section("stepper_z").get("endstop_pin") == "PIN_Z_ENDSTOP" && section("stepper_x").get("endstop_pin") == "PIN_X_ENDSTOP", "one switch for each axis");
		// The drivers' run current is the model's driver setting (an RMS value already), not a fraction of it.
		for (name in ["x", "y", "z", "z1", "z2"]) near(value("tmc2209 stepper_" + name, "run_current"), 1.68, 'the drivers are set to 1.68 A RMS ($name)');
		near(value("tmc2209 extruder", "run_current"), 1.0, "the extruder's driver is set to 1 A RMS");
		// The probe stands where the model put it, and trips with the nozzle a millimetre above the bed (the probe's face is
		// a millimetre above the nozzle's tip and senses two).
		near(value("probe", "x_offset"), 38.0, "the probe is 38 mm to the right of the nozzle");
		near(value("probe", "y_offset"), -4.0, "and 4 mm in front");
		near(value("probe", "z_offset"), 1.0, "and trips with the nozzle a millimetre above the bed");
		check(config.exists("virtual_sdcard") && config.exists("pause_resume") && config.exists("exclude_object"), "what Moonraker needs to run a print");
		check(config.exists("tmc2209 stepper_x") && config.exists("tmc2209 extruder") && config.exists("heater_fan hotend_fan"), "drivers and fans");

		// The limits are the drives'.
		check(rows.rows.length >= 2 && cartesian < limits[0].speed, "two motors bound x and y together, below an axis's own speed");
		near(cartesian, limits[0].speed / Math.sqrt(2.0), "a diagonal turns one motor at the rate both axes' sum asks: the axes' speed over the square root of two", 1e-9);
		var speed = Math.min(Math.min(limits[0].speed, limits[1].speed), cartesian) * 1000.0, accel = Math.min(limits[0].acceleration, limits[1].acceleration) * 1000.0;
		near(value("printer", "max_velocity"), speed, "max_velocity is the motors' on the worst direction", 1e-5);
		near(value("printer", "max_accel"), accel, "max_accel is the weaker axis's", 1e-5);
		near(value("printer", "max_z_velocity"), limits[2].speed * 1000.0, "max_z_velocity is the bed's", 1e-5);
		near(value("printer", "max_z_accel"), limits[2].acceleration * 1000.0, "max_z_accel is the bed's", 1e-5);
		check(section("printer").get("kinematics") == "corexy", "a CoreXY machine");
		Sys.println('klipper: belts 40 mm, screws 8 mm, extruder ${Math.round(value("extruder", "rotation_distance") * 100) / 100} mm at 54:18; ' +
			'max_velocity ${Math.round(speed * 10) / 10} mm/s, max_accel ${Math.round(accel)} mm/s^2, z ${Math.round(limits[2].speed * 10000) / 10} mm/s ${Math.round(limits[2].acceleration * 1000)} mm/s^2');

		// The heaters' gains are the controller's, times 255; the extruder takes the job's pressure advance and minimum temperature.
		var hotend = job.heaters[0], bed = job.heaters[1];
		near(value("extruder","pullup_resistor"),hotend.pullupOhm,"hotend pull-up follows the job");
		near(value("heater_bed","pullup_resistor"),bed.pullupOhm,"bed pull-up follows the job");
		near(value("extruder", "pid_Kp"), hotend.kp * 255.0, "the hotend's proportional gain", 1e-5);
		near(value("extruder", "pid_Ki"), hotend.ki * 255.0, "its integral gain", 1e-5);
		near(value("heater_bed", "pid_Kp"), bed.kp * 255.0, "the bed's proportional gain", 1e-5);
		near(value("extruder", "min_extrude_temp"), job.minExtrusionTemperature, "the coldest the hotend extrudes");
		near(value("extruder", "pressure_advance"), job.pressureAdvance, "the pressure advance");
		check(section("extruder").get("sensor_type") == "Generic 3950" && section("heater_bed").get("sensor_type") == "Generic 3950", "100k beta 3950 thermistors");
		near(value("extruder", "max_temp"), hotend.maxTemperature, "the hotend's over-temperature limit");

		// The pins are placeholders, each of them in the report and each of them in the file.
		var pins = KlipperConfig.pins(klipper);
		var used = new Map<String, Bool>();
		for (name in config.keys()) {
			var keys = config.get(name);
			for (key in keys.keys()) {
				var word = keys.get(key);
				var pin = StringTools.startsWith(word, "!") ? word.substr(1) : word;
				if (StringTools.startsWith(pin, "PIN_")) used.set(pin, true);
			}
		}
		var reported = new Map<String, Bool>();
		for (entry in pins) reported.set(entry.pin, true);
		var count = 0;
		for (pin in used.keys()) { count++; check(reported.exists(pin), 'the pin report lists $pin'); }
		for (entry in pins) check(used.exists(entry.pin), 'the file uses ${entry.pin}');
		check(count == pins.length && pins.length == 1 + 3 * 5 + 2 * 4 + 6 + 2 + 1 + 2, 'the printer wants ${pins.length} pins');
		check(KlipperConfig.pinReport(klipper).split("\n").length == pins.length + 1, "one line of the report for each pin");

		// The reader refuses what is not a config.
		var refused = false;
		try KlipperConfig.parse("step_pin: PA0") catch (_:Dynamic) refused = true;
		check(refused, "a key outside any section is refused");

		// The export directory: the config, the pins and the bracket as Klipper G-code.
		var directory = "build/export";
		var written = CoreXyPrinterExport.write(directory, printer, job, limits[0], limits[1], limits[2], cartesian, inscribed);
		check(written.length == 3, "the export writes three files");
		for (path in written) check(FileSystem.exists(path) && File.getContent(path).length > 100, 'the export wrote $path');
		var gcode = File.getContent(directory + "/" + CoreXyPrinterExport.BRACKET);
		var materialBracket = CoreXyPrinterExport.bracketProgram(customJob);
		var defaultBracket = cnckit.CncCompiler.compileDetailed(gcode,new cnckit.CncController(Printer));
		var customBracket = cnckit.CncCompiler.compileDetailed(materialBracket,materialController);
		check(Math.abs(customBracket.program.depositedVolume()/defaultBracket.program.depositedVolume()-1)<0.001,
			"Bracket export uses the job material instead of a default filament");
		check(gcode.indexOf("M83") >= 0 && gcode.indexOf("EXCLUDE_OBJECT_DEFINE") >= 0 && gcode.indexOf("M109 S210") >= 0,
			"the bracket is Klipper G-code: relative extrusion, excluded objects, a wait for the hotend");
		check(File.getContent(directory + "/" + CoreXyPrinterExport.CONFIG) == text, "the written config is the rendered one");
		Sys.println('klipper export: ${written.length} files in $directory, bracket ${gcode.split("\n").length} lines');
		Sys.println('Printer Klipper tests passed ($assertions assertions)');
	}
}
