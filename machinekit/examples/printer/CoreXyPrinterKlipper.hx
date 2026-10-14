import machinekit.motion.ProximitySwitch;
import machinekit.printing.KlipperConfig;
import machinekit.printing.KlipperConfig.KlipperHeater;
import machinekit.printing.KlipperConfig.KlipperPrinter;
import machinekit.printing.KlipperConfig.KlipperStepper;
import materia.project.SceneArtifact.SceneArtifactHeater;
import materia.project.SceneArtifact.SceneArtifactPrinting;

/** An axis's speed and acceleration as the drives allow them (`RobotModel.coupledLimits`): metres per second and per second squared. */
typedef PrinterAxisLimit = {speed:Float, acceleration:Float};

/**
 * The CoreXY printer as a Klipper `printer.cfg` model: what the model knows of its belts, screws, extruder gears,
 * heaters and drives, in Klipper's terms. The numbers come from the parts (a GT2 belt on a 20-tooth pulley moves 40 mm
 * a turn, a Tr8x2 screw of four starts 8 mm, the hobbed gear turns 7.3 mm x pi of filament a turn behind an 18:54
 * pair) and from the print job (the heaters' controller gains, the pressure advance, the minimum extrusion
 * temperature). The pins are placeholders for the board that is chosen.
 */
class CoreXyPrinterKlipper {
	/** Homing speed as a fraction of an axis's speed, and its cap, in millimetres per second. */
	static inline var HOMING_FRACTION:Float = 0.2;
	static inline var HOMING_CAP:Float = 50.0;
	/** Klipper's own default corner velocity, mm/s: written out, see `printer`. */
	static inline var SQUARE_CORNER_VELOCITY:Float = 5.0;

	/**
	 * `x` and `y` are the axes' limits at the motors' full rate, so a pure x or y move can be faster than a diagonal;
	 * Klipper has one `max_velocity` for every direction, which is `cartesianSpeed`, the speed the motors give on the
	 * worst direction (`MotorSpaceConstraints.inscribedSpeed`), at most the axes' own. Its `max_accel` is one Euclidean
	 * bound too: `cartesianAcceleration`, what the motors' torques give on the worst direction
	 * (`MotorSpaceConstraints.inscribedAcceleration`), at most the weaker axis's own. `square_corner_velocity` is written out:
	 * Klipper jumps velocity at a corner by up to it, and this stack plans corners with no velocity jump (AM-D4), so the
	 * smallest value Klipper accepts keeps its planner as near to that as it can be.
	 */
	public static function printer(source:CoreXyPrinter, job:SceneArtifactPrinting, x:PrinterAxisLimit, y:PrinterAxisLimit,
			z:PrinterAxisLimit, cartesianSpeed:Float, cartesianAcceleration:Float):KlipperPrinter {
		function homing(limit:PrinterAxisLimit):Float return Math.min(HOMING_CAP, HOMING_FRACTION * limit.speed * 1000.0);
		var belt = 2.0 * Math.PI * source.pulleyRadius;
		var screw = CoreXyPrinter.zThread().lead;
		var current = CoreXyPrinter.DRIVER_CURRENT;
		function stepper(name:String, distance:Float, limit:PrinterAxisLimit, max:Float, positive:Bool, leads:Bool = true):KlipperStepper
			return {name: name, rotationDistance: distance, microsteps: CoreXyPrinter.MICROSTEPS, runCurrent: current, leads: leads,
				positionMin: 0.0, positionMax: max, positionEndstop: positive ? max : 0.0, homesPositive: positive, homingSpeed: homing(limit)};
		function heater(name:String):SceneArtifactHeater {
			for (item in job.heaters) if (item.name == name) return item;
			throw 'The print job has no heater "$name"';
		}
		function klipper(item:SceneArtifactHeater):KlipperHeater {
			// A 100k NTC of beta 3950 is Klipper's "Generic 3950"; another thermistor needs its own table.
			if (item.beta != 3950.0 || item.r0Ohm != 100000.0) throw 'Heater "${item.name}" has a thermistor Klipper has no table for';
			return {sensorType: "Generic 3950", minTemp: 0.0, maxTemp: item.maxTemperature, kp: item.kp, ki: item.ki, kd: item.kd};
		}
		var travel = [for (spec in source.specs) spec.upper];
		var steppers = [stepper("x", belt, x, travel[0], false), stepper("y", belt, y, travel[1], false)];
		// One screw to each corner of the bed's carrier: three motors that turn together, the first with the switch.
		for (name in ["z", "z1", "z2"]) steppers.push(stepper(name, screw, z, travel[2], true, name == "z"));
		var hotend = source.hotend.spec;
		return {name: "CoreXY printer", maxVelocity: Math.min(Math.min(x.speed, y.speed), cartesianSpeed) * 1000.0,
			maxAccel: Math.min(Math.min(x.acceleration, y.acceleration), cartesianAcceleration) * 1000.0, maxZVelocity: z.speed * 1000.0,
			maxZAccel: z.acceleration * 1000.0, squareCornerVelocity: SQUARE_CORNER_VELOCITY, steppers: steppers,
			extruder: {rotationDistance: Math.PI * source.extruder.spec.hobDiameter,
				gearRatio: '${source.bigGear.teeth}:${source.pinion.teeth}', microsteps: CoreXyPrinter.MICROSTEPS,
				runCurrent: CoreXyPrinter.EXTRUDER_DRIVER_CURRENT, nozzleDiameter: source.nozzle.spec.orifice,
				filamentDiameter: hotend.filamentDiameter, minExtrudeTemp: job.minExtrusionTemperature,
				pressureAdvance: job.pressureAdvance, pressureAdvanceSmoothTime: job.pressureAdvanceWindow,
				heater: klipper(heater("hotend"))},
			bed: klipper(heater("bed")),
			probe: {xOffset: CoreXyPrinter.PROBE_X, yOffset: CoreXyPrinter.PROBE_Y,
				zOffset: new ProximitySwitch().spec.sensingDistance - CoreXyPrinter.PROBE_FACE_ABOVE_TIP}};
	}

	/** The config as text. */
	public static function config(source:CoreXyPrinter, job:SceneArtifactPrinting, x:PrinterAxisLimit, y:PrinterAxisLimit,
			z:PrinterAxisLimit, cartesianSpeed:Float, cartesianAcceleration:Float):String
		return KlipperConfig.render(printer(source, job, x, y, z, cartesianSpeed, cartesianAcceleration));
}
