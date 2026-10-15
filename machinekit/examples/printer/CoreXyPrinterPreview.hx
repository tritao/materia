import cadkit.modeling.AssemblyModel;
import camkit.additive.Slicer;
import camkit.additive.SlicerSettings;
import camkit.additive.TestParts;
import cnckit.CncController;
import cnckit.printer.PrinterFlavour;
import cnckit.printer.PrinterWriter;
import cadkit.modeling.AssemblyState;
import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.PosedParts;
import machinekit.motion.LinearRail;
import machinekit.motion.NemaStepper;
import machinekit.printing.Thermistor.ThermistorSpec;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the CoreXY printer. */
class CoreXyPrinterPreview {
	public static inline var ASSEMBLY_ID:String = "corexy-printer";

	/** The printer printing the slicer's calibration cube (20 mm with a 6 mm hole through it) at its default infill: the slow dimensional check. */
	public static function cube():Bytes {
		var printer = new CoreXyPrinter();
		var scene = AssemblyPreview.scene(printer, ASSEMBLY_ID);
		return withPrinting(scene,CoreXyPrinterJob.calibrationCube(printer, 20.0, 0.15));
	}

	/** The same block 4 mm tall and solid (20 layers): the regular dimensional check, a few minutes of printing. */
	public static function cubeShort():Bytes {
		var printer = new CoreXyPrinter();
		var scene = AssemblyPreview.scene(printer, ASSEMBLY_ID);
		return withPrinting(scene,CoreXyPrinterJob.calibrationCube(printer, 4.0, 1.0));
	}

	/**
	 * Geometry, joints and initial pose of the printer (parts with equal designations share geometry), and the job it
	 * prints: the slicer's small tab.
	 */
	public static function printer():Bytes {
		var printer = new CoreXyPrinter();
		var scene = AssemblyPreview.scene(printer, ASSEMBLY_ID);
		return withPrinting(scene,CoreXyPrinterJob.smallTab(printer));
	}
	/** Tune once from the complete physical travel range, then keep the kernel fixed through queued motion. */
	static function withPrinting(scene:materia.project.SceneArtifact.SceneArtifactData,job:materia.project.SceneArtifact.SceneArtifactPrinting):Bytes {
		var model = cadbridge.AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,
			cadbridge.AssemblyPhysicalPartView.fromSceneArtifact(scene),scene.assemblyState).model;
		var soft = robotkit.model.DriveLoads.of(model,null,["x","y"]);
		var hard = robotkit.model.DriveLoads.stiffest(model,null,["x","y"]);
		var damping = 1.0;
		for (coupling in model.couplings) if (coupling.stiffness>0.0) damping=Math.min(damping,coupling.dampingRatio);
		job.shaping = [for (load in soft) {
			var upper = [for (candidate in hard) if (candidate.axis == load.axis) candidate][0];
			var lowerHz = Math.sqrt(load.stiffness/load.mass)/(2*Math.PI), upperHz = Math.sqrt(upper.stiffness/upper.mass)/(2*Math.PI);
			var shaper = motionkit.trajectory.InputShaper.forFrequencyBand(motionkit.trajectory.InputShaper.InputShaperKind.EI,lowerHz,upperHz,damping);
			{axis:Std.string(load.axis),kind:"ei",frequency:shaper.frequency,damping:damping};
		}];
		scene.printing = job;
		return SceneArtifact.encode(scene);
	}

}

/**
 * The print job of the printer example (see SceneArtifactPrinting): the slicer's program as Klipper G-code, and the
 * printer's own numbers for the extruder, the hotend and the bed, taken from the parts that are in it.
 */
class CoreXyPrinterJob {
	/** The step rate of the controller the steppers are wired to, as the router's. */
	public static inline var STEP_TICK_HZ:Int = 40000;
	/** The room, degrees Celsius. */
	public static inline var AMBIENT:Float = 22.0;

	/** The slicer's `SmallTab` (10 x 10 x 1.2 mm, six layers) with its default settings. */
	public static function smallTab(printer:CoreXyPrinter):SceneArtifactPrinting {
		var sliced = Slicer.slice(TestParts.smallTab(), new SlicerSettings());
		return job(printer, PrinterWriter.write(sliced.program, new CncController(Printer), Klipper));
	}

	/** The slicer's calibration block (20 mm square, `height` millimetres tall, with a 6 mm hole through it) at infill `density`. */
	public static function calibrationCube(printer:CoreXyPrinter, height:Float, density:Float):SceneArtifactPrinting {
		var settings = new SlicerSettings();
		settings.infillDensity = density;
		var sliced = Slicer.slice(TestParts.calibrationCube(height), settings);
		return job(printer, PrinterWriter.write(sliced.program, new CncController(Printer), Klipper));
	}

	/** `source` (printer G-code in machine coordinates) as a job on this printer. */
	public static function job(printer:CoreXyPrinter, source:String):SceneArtifactPrinting {
		// The heaters are the printer's own parts, not new ones: a part changed in the printer changes the job.
		var hotend = printer.hotend.spec, cartridge = printer.cartridge, thermistor = printer.hotendThermistor;
		var hotendHeater = heater("hotend", "thermistor", 0, "fan.part", true, thermistor.spec,
			hotend.heatCapacity, cartridge.power(cartridge.spec.volts), hotend.lossCoefficient, hotend.fanLossCoefficient,
			Math.min(hotend.maxTemperature, Math.min(cartridge.spec.maxTemperature, thermistor.spec.maxTemperature)) - 10.0);
		var bed = printer.bed.spec;
		var bedHeater = heater("bed", "bedThermistor", 1, null, false, printer.bedThermistor.spec,
			bed.heatCapacity, bed.watts, bed.lossCoefficient, 0.0, bed.maxTemperature);
		return {source: source, axes: ["x", "y", "z"], extruder: "e",
			filamentDiameter: hotend.filamentDiameter * 1e-3, gearRadius: printer.extruder.spec.hobDiameter * 0.5e-3,
			maxFlow: hotend.maxVolumetricFlow * 1e-9, pressureAdvance: hotend.pressureLag, pressureAdvanceWindow: 0.04, meltLag: hotend.pressureLag, rampLength: 0.0005,
			minExtrusionTemperature: hotend.minExtrusionTemperature, heaters: [hotendHeater, bedHeater],
			controller: {stepTickHz: STEP_TICK_HZ}, deposit: "printed"};
	}

	/**
	 * A heater on the controller whose plant is a block of `capacity` J/K, heated by `power` W at full duty and losing
	 * `loss` W/K to the room (`fanLoss` more with the part fan on): a first-order lag of time constant
	 * `tau = capacity / loss`. The PI closes the loop in a twentieth of `tau` (`kp = capacity / (power tau / 20)`) with
	 * an integral time of an eighth of it, so the integral has made up the duty the block needs to hold its target
	 * (`loss (target - room) / power`) before the watchdog's window is over; there is no derivative, as the plant has no
	 * dead time to lead. The output is at full power outside the PID range and let down gently inside it. The watchdog
	 * expects 2 degrees of gain in a quarter of `tau` while heating (never less than 20 s).
	 */
	static function heater(name:String, mount:String, slot:Int, fan:Null<String>, feedsExtruder:Bool, sensor:ThermistorSpec,
			capacity:Float, power:Float, loss:Float, fanLoss:Float, maxTemperature:Float):SceneArtifactHeater {
		var timeConstant = capacity / loss;
		var lambda = timeConstant / 20.0;
		var heater:SceneArtifactHeater = {name: name, sensor: name, slot: slot, mount: mount, target: 'heater.$name.target',
			feedsExtruder: feedsExtruder, r0Ohm: sensor.ohmsAt25, t0Celsius: 25.0, beta: sensor.beta, pullupOhm: 4700.0, vRef: 3.3,
			kp: capacity / (power * lambda), ki: capacity / (power * lambda) / (timeConstant / 8.0), kd: 0.0, maxDuty: 1.0, maxTemperature: maxTemperature,
			bandCelsius: 3.0, pidRangeCelsius: 10.0, gainWindowSeconds: Math.max(20.0, timeConstant / 4.0), gainMinCelsius: 2.0,
			hysteresisCelsius: 15.0, hysteresisWindowSeconds: 10.0, heatCapacity: capacity, power: power, loss: loss,
			fanLoss: fanLoss, ambient: AMBIENT};
		if (fan != null) heater.fan = fan;
		return heater;
	}
}

/** A pulley's coefficients on x and y: it turns `x * x + y * y` pitch radii per millimetre. */
class PrinterPulleyTerm {
	public final id:String;
	public final x:Int;
	public final y:Int;

	public function new(id:String, x:Int, y:Int) {
		this.id = id;
		this.x = x;
		this.y = y;
	}
}

/** Geometry builds, the axes put the nozzle where machine coordinates say, belts, screws and gears agree, and nothing collides. */
class CoreXyPrinterChecks {
	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	/** Each pulley's angle per millimetre of x and of y, in pitch radii. */
	static final TERMS:Array<PrinterPulleyTerm> = [
		new PrinterPulleyTerm("pulleyA", 1, -1), new PrinterPulleyTerm("idlerAFront", 1, -1), new PrinterPulleyTerm("idlerARear", 1, -1),
		new PrinterPulleyTerm("pulleyB", 1, 1), new PrinterPulleyTerm("idlerBFront", 1, 1), new PrinterPulleyTerm("idlerBRear", 1, 1),
		new PrinterPulleyTerm("idlerAStart", 1, 0), new PrinterPulleyTerm("idlerAEnd", -1, 0),
		new PrinterPulleyTerm("idlerBStart", 1, 0), new PrinterPulleyTerm("idlerBEnd", -1, 0)
	];

	public static function run():Void {
		PosedParts.scope(posed -> {
			clearance = posed;
			try runChecks() catch (error:Dynamic) { clearance = null; throw error; }
			clearance = null;
		});
	}

	/** The members posed by this run, with their geometry built once each; closed when the run ends. */
	static var clearance:PosedParts;

	static function runChecks():Void {
		var scene = SceneArtifact.decode(CoreXyPrinterPreview.printer());
		var definition = scene.assemblyDefinition;
		var printer = new CoreXyPrinter();
		if (definition == null || definition.occurrences.length != printer.components().length)
			throw "CoreXY printer preview has the wrong number of occurrences";
		for (part in scene.parts) if (part.volume == null || part.volume <= 0 || part.inertia == null)
			throw 'CoreXY printer part "${part.id}" has no mass properties';
		var prismatic = [for (joint in definition.joints) if (Std.string(joint.type) == "prismatic") joint.id];
		prismatic.sort(Reflect.compare);
		if (prismatic.join(",") != "x,y,z") throw 'CoreXY printer should have prismatic joints x, y, z, got $prismatic';
		var turning = [for (joint in definition.joints) if (Std.string(joint.type) == "continuous") joint.id];
		turning.sort(Reflect.compare);
		var expected = [for (term in TERMS) term.id + "-turn"].concat(["e", "motorE-turn", "screwZFront-turn", "screwZRearLeft-turn", "screwZRearRight-turn"]);
		expected.sort(Reflect.compare);
		if (turning.join(",") != expected.join(",")) throw 'CoreXY printer turning joints should be ${expected.join(",")}, got ${turning.join(",")}';
		var motors = [for (entry in printer.components()) if (Std.isOfType(entry.component, NemaStepper)) entry.id];
		motors.sort(Reflect.compare);
		if (motors.join(",") != "motorA,motorB,motorE,motorZFront,motorZRearLeft,motorZRearRight")
			throw 'CoreXY printer should have six steppers, got $motors';

		// Every pulley follows its belt: one coupling per axis it sees, of the pulley's pitch radius.
		var radius = printer.pulleyRadius;
		var couplings = definition.couplings;
		if (couplings == null) throw "CoreXY printer has no couplings";
		var beltCouplings = 0;
		for (term in TERMS) beltCouplings += (term.x != 0 ? 1 : 0) + (term.y != 0 ? 1 : 0);
		var screwCouplings = 3, gearCouplings = 1;
		if (couplings.length != beltCouplings + screwCouplings + gearCouplings)
			throw 'CoreXY printer should have ${beltCouplings + screwCouplings + gearCouplings} couplings, got ${couplings.length}';
		var leaders = new Map<String, Int>();
		for (coupling in couplings) leaders.set(coupling.target, (leaders.exists(coupling.target) ? leaders.get(coupling.target) : 0) + 1);
		for (term in TERMS) {
			var count = leaders.get(term.id + "-turn");
			var wanted = (term.x != 0 ? 1 : 0) + (term.y != 0 ? 1 : 0);
			if (count != wanted) throw '${term.id} should follow $wanted axes, got $count';
		}
		for (coupling in couplings) {
			var drive = printer.transmissionFor(coupling.id);
			if (drive == null) throw 'coupling ${coupling.id} has no transmission';
			switch drive.source {
				case TimingBelt(_, _), BeltIdler(_, _):
					near(Math.abs(coupling.ratio), 1 / radius, '${coupling.id} turns one pitch radius per radian', 1e-12);
					// The belts state their damping, and it rides on the coupling a belt's stiffness does (a carriage rings on it).
					switch drive.source {
						case TimingBelt(_, _):
							if (coupling.dampingRatio == null) throw '${coupling.id} carries no damping ratio from its belt';
							near(coupling.dampingRatio, CoreXyPrinter.SHAPER_DAMPING, '${coupling.id} carries its belt\'s damping ratio', 1e-12);
						default:
					}
				case LeadScrew(_, _): near(Math.abs(coupling.ratio), 2 * Math.PI / 8, '${coupling.id}: a T8 screw turns 2 pi per 8 mm of bed travel', 1e-12);
				case GearMesh(_, _): near(coupling.ratio, -3, "the extruder gears turn the motor three times as far as the hobbed gear", 1e-12);
				default: throw 'unexpected transmission for ${coupling.id}';
			}
		}
		// The three screws all follow z; the extruder's hobbed gear turns the motor, so `e` leads.
		for (id in ["screwZFront", "screwZRearLeft", "screwZRearRight"]) {
			var found = [for (coupling in couplings) if (coupling.target == id + "-turn") coupling];
			if (found.length != 1 || found[0].source != "z") throw '$id should follow z alone';
		}
		var gear = [for (coupling in couplings) if (coupling.target == "motorE-turn") coupling];
		if (gear.length != 1 || gear[0].source != "e") throw "the extruder motor should follow the hobbed gear's joint e";
		near(printer.extruder.rotationPerMillimetre() * 1.0, 2 / 7.3, "the hobbed gear turns 2 / hob diameter radians per millimetre of filament", 1e-12);

		// The switches: X and Y at the low ends, Z at the bottom, with Z first.
		var homes = definition.switches == null ? [] : definition.switches;
		var byJoint = new Map<String, materia.assembly.AssemblyDefinition.AssemblySwitch>();
		for (contact in homes) byJoint.set(contact.joint, contact);
		function contactOn(joint:String):materia.assembly.AssemblyDefinition.AssemblySwitch {
			var contact = byJoint.get(joint);
			if (contact == null) throw 'the printer has no home switch on $joint';
			return contact;
		}
		if (homes.length != 3 || contactOn("x").side != -1 || contactOn("y").side != -1 || contactOn("z").side != 1)
			throw "the printer homes X and Y at their low ends and Z at the bottom";
		var after = function(joint:String):String {
			var list = contactOn(joint).homeAfter;
			return list == null ? "" : list.join(",");
		};
		if (after("z") != "" || after("x") != "z" || after("y") != "z")
			throw "Z homes first; X and Y wait for the bed to be down";

		// The nozzle goes where machine coordinates say, and the bed where z says.
		var model = new AssemblyModel("mm");
		printer.addTo(model, "");
		var state = new AssemblyState(model.definition(CoreXyPrinterPreview.ASSEMBLY_ID));
		var frame = ["sideBottomLeft", "railYRight", "basePlate", "motorZFront", "crossTopFront"];
		var framePoses = [for (id in frame) state.worldPose(id)];
		for (at in [[110.0, 110, 10], [0.0, 0, 0], [220.0, 220, 220], [0.0, 220, 100], [220.0, 0, 5], [37.5, 112, 12.5]]) {
			state.setJoint("x", at[0]); state.setJoint("y", at[1]); state.setJoint("z", at[2]);
			state.forwardKinematics();
			var label = 'at machine ${at.join(", ")}';
			var tip = state.worldConnector("nozzle", "tip");
			near(tip.x, CoreXyPrinter.MACHINE_ZERO_X + at[0], '$label: nozzle x');
			near(tip.y, CoreXyPrinter.MACHINE_ZERO_Y + at[1], '$label: nozzle y');
			near(tip.z, CoreXyPrinter.NOZZLE_Z, '$label: nozzle height');
			var surface = state.worldConnector("bed", "surface");
			near(surface.x, 0, '$label: the bed does not move in x'); near(surface.y, 0, '$label: the bed does not move in y');
			near(surface.z, CoreXyPrinter.NOZZLE_Z - at[2], '$label: the bed is z below the nozzle tip');
			for (i in 0...frame.length) {
				var pose = state.worldPose(frame[i]), start = framePoses[i];
				near(pose.x + pose.y + pose.z, start.x + start.y + start.z, '${frame[i]} stays put', 1e-9);
			}
			checkBlocksOnRails(printer, state);
			for (term in TERMS)
				near(state.joint(term.id + "-turn"), (term.x * at[0] + term.y * at[1]) / radius, '$label: ${term.id}', 1e-9);
			near(state.joint("pulleyA-turn"), (at[0] - at[1]) / radius, '$label: motor A', 1e-9);
			near(state.joint("pulleyB-turn"), (at[0] + at[1]) / radius, '$label: motor B', 1e-9);
			for (id in ["screwZFront", "screwZRearLeft", "screwZRearRight"])
				near(Math.abs(state.joint(id + "-turn")), 2 * Math.PI / 8 * at[2], '$label: $id turns with the bed', 1e-9);
		}
		// Clearances: the gantry and the carriage clear the frame, the motors and the pulleys at the corners of travel;
		// the bed clears the frame, the motors and the screws' hardware at both ends of its travel, and the nozzle meets it only at z = 0.
		var gantry = ["blockYLeft", "blockYRight", "bracketLeft", "bracketRight", "crossbar", "railX", "idlerAStartPin", "idlerAEndPin",
			"idlerBStartPin", "idlerBEndPin", "idlerAStart", "idlerAEnd", "idlerBStart", "idlerBEnd", "homeTriggerY", "homeXMount", "homeX"];
		var carriage = ["blockX", "carriagePlate", "clamp", "toolheadMount", "hotend", "nozzle", "cartridge", "thermistor", "hotendFan", "partBlower",
			"extruderMountPlate", "extruder", "motorE", "extruderGear", "extruderPinion", "probeMount", "probe", "homeTriggerX"];
		var fixed = ["sideTopLeft", "sideTopRight", "crossTopFront", "crossTopRear", "postLeftFront", "postRightFront", "postLeftRear", "postRightRear",
			"railYLeft", "railYRight", "motorA", "motorB", "motorPlateA", "motorPlateB", "pulleyA", "pulleyB", "idlerAFront", "idlerARear", "idlerBFront",
			"idlerBRear", "idlerAFrontPin", "idlerARearPin", "idlerBFrontPin", "idlerBRearPin", "homeYMount", "homeY"];
		for (at in [[0.0, 0, 0], [220.0, 0, 0], [0.0, 220, 0], [220.0, 220, 0], [110.0, 110, 0]]) {
			checkClear(printer, state, at, gantry.concat(carriage), fixed);
			checkClear(printer, state, at, carriage, gantry.filter(id -> id != "railX" && id != "homeTriggerY" && id != "homeXMount" && id != "homeX"));
		}
		// The toolhead meets the bed only at z = 0, and the bed's carrier clears the base, the motors and their couplings at both ends.
		var bedParts = ["bed", "bedSupport", "carrier", "wingLeft", "wingRight", "bedThermistor", "homeTriggerZ"];
		var toolhead = carriage.filter(id -> id != "homeTriggerX");
		for (at in [[0.0, 0, 0], [220.0, 220, 0], [110.0, 110, 0], [0.0, 220, 0], [220.0, 0, 0]]) checkClear(printer, state, at, toolhead, bedParts);
		var below = ["basePlate", "motorZFront", "motorZRearLeft", "motorZRearRight", "screwZFrontCoupling", "screwZRearLeftCoupling", "screwZRearRightCoupling",
			"powerSupply", "motorADriver", "homeZ", "homeZMount", "postLeftFront", "postRightFront", "postLeftRear", "postRightRear",
			"sideBottomLeft", "sideBottomRight", "crossBottomFront", "crossBottomRear"];
		for (z in [0.0, 100, 220]) {
			checkClear(printer, state, [110.0, 110, z], bedParts.filter(id -> id != "homeTriggerZ"), below);
			checkClear(printer, state, [110.0, 110, z], ["blockZLeft", "blockZRight"], ["postLeftRear", "postRightRear", "crossTopRear", "crossBottomRear"]);
		}
		checkClear(printer, state, [110.0, 110, 220], ["homeTriggerZ"], ["basePlate", "motorZFront", "powerSupply"]);
		// The check has teeth: three millimetres further and the nozzle is in the bed.
		var widened = printer.definition();
		for (joint in widened.joints) if (joint.id == "z" && joint.limits.lower != null) joint.limits.lower -= 5;
		var beyond = new AssemblyState(widened);
		beyond.setJoint("x", 110); beyond.setJoint("y", 110); beyond.setJoint("z", -3);
		beyond.forwardKinematics();
		if (!(clearance.volume(printer, beyond, "CoreXY printer", "nozzle", "bed") > 1)) throw "the nozzle should meet the bed when z goes below zero";
		// The gear pair meshes at its pitch circles' distance.
		moveTo(state, [110.0, 110, 10]);
		var gearPose = state.worldPose("extruderGear"), pinionPose = state.worldPose("extruderPinion");
		near(Math.sqrt(Math.pow(gearPose.x - pinionPose.x, 2) + Math.pow(gearPose.y - pinionPose.y, 2) + Math.pow(gearPose.z - pinionPose.z, 2)),
			(printer.pinion.pitchDiameter + printer.bigGear.pitchDiameter) / 2, "the extruder gears sit at their pitch circles' distance", 1e-9);
		// At the joints' zero the belts are clamped by the carriage's pad on exactly their one strand, and clear of the rest.
		var clamp = 24.0 * 1.38 * CoreXyPrinter.BELT_WIDTH;
		for (letter in ["A", "B"]) {
			near(overlap(printer, state, [0, 0, 10], "belt" + letter, "clamp"), clamp, 'the clamp holds belt $letter\'s one strand', 0.5);
			// The belt wraps its own pulleys; everything else it must clear.
			for (other in gantry.concat(carriage).concat(fixed)) {
				var wrapped = other.indexOf("pulley") == 0 || (other.indexOf("idler") == 0 && other.indexOf("Pin") < 0);
				if (other != "clamp" && !wrapped && overlap(printer, state, [0, 0, 10], "belt" + letter, other) > 1e-3)
					throw 'belt $letter runs through $other';
			}
		}
		Sys.println('corexy printer: ${scene.parts.length} definitions, ${definition.occurrences.length} occurrences, ' +
			'${printer.billOfMaterials().lines().length} BOM lines, ${Math.round(printer.massProperties().mass * 10) / 10} kg');
	}

	/** No member of `moving` intersects a member of `others` at machine position `at` (x, y, z). */
	static function checkClear(printer:CoreXyPrinter, state:AssemblyState, at:Array<Float>, moving:Array<String>, others:Array<String>):Void {
		moveTo(state, at);
		clearance.checkClear(printer, state, "CoreXY printer", moving, others,
			(a, b, volume) -> '$a collides with $b at machine ${at.join(", ")}: ${Math.round(volume)} mm³');
	}

	static function moveTo(state:AssemblyState, at:Array<Float>):Void {
		state.setJoint("x", at[0]); state.setJoint("y", at[1]); state.setJoint("z", at[2]);
		state.forwardKinematics();
	}

	static function overlap(printer:CoreXyPrinter, state:AssemblyState, at:Array<Float>, a:String, b:String):Float {
		moveTo(state, at);
		return clearance.volume(printer, state, "CoreXY printer", a, b);
	}

	/** Every rail block stays within its rail's usable length. */
	static function checkBlocksOnRails(printer:CoreXyPrinter, state:AssemblyState):Void {
		for (pair in [["railYLeft", "blockYLeft"], ["railYRight", "blockYRight"], ["railX", "blockX"], ["railZLeft", "blockZLeft"], ["railZRight", "blockZRight"]]) {
			var local = AssemblyFrames.compose(AssemblyFrames.inverse(state.worldPose(pair[0])), state.worldPose(pair[1]));
			var rail = [for (entry in printer.components()) if (entry.id == pair[0]) cast(entry.component, LinearRail)][0];
			var reach = rail.spec.railEndMargin + rail.spec.blockLength / 2;
			near(local.x, 0, '${pair[1]} centred on ${pair[0]}', 1e-6);
			near(local.y, 0, '${pair[1]} seated on ${pair[0]}', 1e-6);
			if (!(local.z >= reach - 1e-6 && local.z <= rail.length - reach + 1e-6))
				throw '${pair[1]} runs off ${pair[0]}: ${local.z} of ${rail.length} mm';
		}
	}
}

/** Standalone check of the printer example. */
function main():Void CoreXyPrinterChecks.run();
