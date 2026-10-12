package app;

import cadkit.modeling.AssemblyState;
import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.CncDiagnostic.CncSeverity;
import materia.project.SceneArtifact.SceneArtifactPrinting;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.event.EventValue;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.robot.PlanCheck;
import motionkit.robot.PlanCheck.PlanCheckOptions;
import motionkit.robot.StepperSlip;
import motionkit.trajectory.InputShaper;
import nativekit.sim.SimSession;
import robotkit.device.DeviceBinding;
import robotkit.runtime.EncoderMonitor;
import robotkit.runtime.Simulation;
import robotkit.runtime.VirtualHeaters;
import robotkit.runtime.VirtualHeaters.HeaterSetup;
import robotkit.tool.HeaterSensor;
import robotkit.tool.HeaterSensor.HeaterReading;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.PrinterBinding;
import toolpathkit.motion.ToolpathChannels;
import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.Point3;
import toolpathkit.path.PrintFeature;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;

/**
 * Prints a project's job on its simulated printer (see SceneArtifactPrinting): the G-code is compiled against the
 * machine's axes, the extruder is a joint planned with them, and the printer's heaters run on its virtual device, whose
 * reports the program's waits read. A heater that faults stops the print. The machine starts where its initial pose has
 * it, referenced.
 */
class PrintProgramPlayer extends ToolpathPlayer {
	/** Two time-sync replies establish the board's host clock before the device is used, simulated seconds. */
	static inline final SYNC_DELAY = 0.15;
	/** The printed material goes to the scene at most this often, in simulated seconds. */
	static inline final STOCK_REFRESH = 0.1;
	/**
	 * The printer's link to its controller: the board's native USB (full speed, 12 Mbit/s), not a plain UART. A sliced
	 * curve plans to thousands of segments a second (see AM5b) and each carries a polynomial per motor, which the
	 * endpoint's link qualification refuses to refill in time at the welding device's 3 Mbaud or a UART's 921600.
	 */
	public static inline final LINK_BAUD = 12000000;
	/** Ray spacing of the printed material, in metres: a fifth of a bead's height; coarser only for a bed full of it. */
	static inline final STOCK_SPACING = 0.0001;
	/** Most rays in the Z grid of the printed material before its spacing grows. */
	static inline final STOCK_RAYS = 2000000.0;
	/** How far an axis may stand from where a plan ends, metres, before the plan is not complete: about four microsteps. */
	static inline final POSITION_TOLERANCE = 5e-5;

	/** The virtual heaters a job asks for, with the extruder's motor on device channel `extruderChannel`. */
	public static function heaterSetups(job:SceneArtifactPrinting, extruderChannel:Int):Array<HeaterSetup> {
		var result:Array<HeaterSetup> = [];
		for (heater in job.heaters) {
			var setup = new HeaterSetup(heater.name, heater.sensor, heater.slot, heater.target);
			setup.fanChannel = heater.fan;
			if (heater.feedsExtruder) {
				setup.extruderActuator = extruderChannel;
				setup.minExtrusionTemperature = job.minExtrusionTemperature;
			}
			setup.r0Ohm = heater.r0Ohm; setup.t0Celsius = heater.t0Celsius; setup.beta = heater.beta;
			setup.pullupOhm = heater.pullupOhm; setup.vRef = heater.vRef;
			setup.kp = heater.kp; setup.ki = heater.ki; setup.kd = heater.kd; setup.maxDuty = heater.maxDuty;
			setup.maxTemperature = heater.maxTemperature; setup.bandCelsius = heater.bandCelsius;
			setup.pidRangeCelsius = heater.pidRangeCelsius; setup.gainWindowSeconds = heater.gainWindowSeconds;
			setup.gainMinCelsius = heater.gainMinCelsius; setup.hysteresisCelsius = heater.hysteresisCelsius;
			setup.hysteresisWindowSeconds = heater.hysteresisWindowSeconds;
			setup.heatCapacity = heater.heatCapacity; setup.power = heater.power; setup.loss = heater.loss;
			setup.fanLoss = heater.fanLoss; setup.ambient = heater.ambient;
			result.push(setup);
		}
		return result;
	}

	final job:SceneArtifactPrinting;
	final project:ProjectDocumentSession;
	final heaters:VirtualHeaters;
	/** The printer's controller channels: a printer is a machine of its own, in the scene's unscoped namespace. */
	final channels:ToolpathChannels;
	final readings = new Map<String, HeaterReading>();
	/** Each layer's first G-code line, in order, and the layer count. */
	var layerLines:Array<Int> = [];
	var readyAt:Float;
	var synchronised = false;
	/** Whether the printed material is contoured for the scene as it grows; a measurement that only reads the stock turns it off. */
	public var showPrint:Bool = true;
	/** The ray spacing of the printed material, in metres; read when the print lays its first bead. */
	public var stockSpacing:Float = STOCK_SPACING;
	/** The printed material, once the print has laid some, and the object that shows it. */
	public var stock(default, null):Null<PrintingStock> = null;
	final newStock:Null<Void->PrintingStock>;
	final stockObject:Null<String>;
	var stockShownAt = Math.NEGATIVE_INFINITY;
	final extruderIndex:Int;
	final gearRadius:Float;
	/** What the program says of each G-code line: the layer height in force and the feature printed. */
	var heightLines:Array<Int> = [];
	var heights:Array<Float> = [];
	var featureLines:Array<Int> = [];
	var features:Array<PrintFeature> = [];
	var lastHeight = 0.0;
	/** Wall-clock seconds spent laying and meshing the printed material, for profiling. */
	public var depositSeconds(default, null):Float = 0.0;
	public var meshingSeconds(default, null):Float = 0.0;
	/** The layer the machine is printing (1 for the first, 0 before it), and the layers in the job. */
	public var layer(default, null):Int = 0;
	public var layerCount(default, null):Int = 0;

	public function new(job:SceneArtifactPrinting, robot:AssemblyRobot, simulation:Simulation,
			project:ProjectDocumentSession, session:SimSession, ?check:PlanCheckOptions) {
		super(robot, session, job.source, false, check);
		var definition = project.projectAssemblyDefinition, physical = project.projectPhysical;
		var heaters = robot.heaters;
		if (definition == null || physical == null) throw "A print job needs the project's printer";
		if (heaters == null) throw "A print job needs the heaters of the printer's virtual device";
		this.job = job;
		this.project = project;
		this.heaters = heaters;
		channels = new ToolpathChannels("");
		readyAt = session.simulationTime() + SYNC_DELAY;
		var metresPerUnit = physical.metresPerUnit;
		var placement = new AssemblyState(definition, project.projectAssemblyState);
		var steady = checkOptions.steady;
		// The controller's step tick caps the stepper axes; the planner holds each joint to its own limits.
		var machineModel = DeviceBinding.planningModel(robot.model, job.controller.stepTickHz);
		var axes:Array<MotionAxisBlueprint> = [];
		var start:Array<Float> = [];
		var rapid = 0.0;
		for (id in job.axes) {
			var joint = [for (joint in definition.joints) if (joint.id == id) joint];
			if (joint.length != 1 || Std.string(joint[0].type) != "prismatic") throw 'Print axis "$id" must be a prismatic joint of the printer';
			var limits = joint[0].limits;
			var lower = limits.lower, upper = limits.upper;
			if (lower == null || upper == null) throw 'Print axis "$id" needs travel limits';
			var initial = placement.joint(id) * metresPerUnit;
			start.push(initial);
			// The axis is as fast as the joints turning with it and their motors allow: belts, screws and the motors.
			var coupled = machineModel.coupledLimits(id, steady);
			if (coupled.velocity == null || coupled.maxAcceleration == null || !(coupled.velocity > 0) || !(coupled.maxAcceleration > 0))
				throw 'Print axis "$id" needs velocity and acceleration limits, its own or its motors\'';
			rapid = Math.max(rapid, coupled.velocity);
			axes.push(new MotionAxisBlueprint(id, [id], lower * metresPerUnit, upper * metresPerUnit,
				coupled.velocity, coupled.maxAcceleration, initial, [1.0], [-initial]));
		}
		var blueprint = new MotionSystemBlueprint(machineModel, robot.blueprint, axes, session.fixedTimestep());
		var machine = new MachineBinding("machine", job.axes[0], job.axes[1], job.axes[2], rapid, channels, 1e-4);
		machine.setRapidsAtFeed(true);
		var printer = new PrinterBinding(job.extruder, job.filamentDiameter, job.gearRadius, job.maxFlow, job.rampLength,
			job.pressureAdvance, job.pressureAdvanceWindow);
		if (job.shaping != null) for (shaper in job.shaping)
			printer.shapers.set(shaper.axis, InputShaper.named(shaper.kind, shaper.frequency, shaper.damping));
		machine.setPrinter(printer);
		var binding = new ToolpathMotionBinding(machine, blueprint);
		// Every plan is checked against the drives: the plan's joints are all of the machine's, the axes' and the extruder's motors are loaded.
		binding.compiler.planCheck = new PlanCheck(machineModel, [for (joint in machineModel.joints) joint.id], checkOptions);
		var controller = new CncController(Printer);
		// A program runs from wherever the machine is, so each pass compiles it from there.
		compileFrom = position -> {
			var compiled = CncCompiler.compileDetailed(job.source, controller, position, machine.travel);
			var errors = [for (diagnostic in compiled.diagnostics) if (diagnostic.severity == CncSeverity.Error) diagnostic.toString()];
			if (errors.length > 0) throw 'The print program: ${errors.join("; ")}';
			var lowered = ToolpathMotion.lower(compiled.program, machine);
			if (lowered.program == null)
				throw 'The print program: ${[for (diagnostic in lowered.diagnostics) diagnostic.message].join("; ")}';
			program = lowered.program;
			sourceMap = lowered.sourceMap;
			return compiled.program;
		};
		solver = binding.solver;
		var joints = machineModel.joints.length;
		axisJoints = [for (index in 0...joints) index];
		for (_ in axisJoints) axisTrackingPeak.push(0.0);
		// Compiling from the starting pose now reports a bad program before the simulation starts.
		var compiled = compileFrom(new Point3(start[0], start[1], start[2]));
		var lines:Array<Int> = [];
		for (op in compiled.ops) switch op {
			case Layer(_, _, provenance): lines.push(provenance.line);
			case _:
		}
		layerLines = lines;
		layerCount = lines.length;
		var extent = scanProgram(compiled);
		extruderIndex = printer.extruderIndex;
		gearRadius = job.gearRadius;
		var shown = job.deposit;
		if (shown == null) {
			newStock = null;
			stockObject = null;
		} else {
			var occurrence = [for (item in definition.occurrences) if (item.id == shown) item];
			var center = occurrence.length == 1 ? project.assemblyPreviewCenter(occurrence[0].definition) : null;
			if (center == null) throw 'The print is shown on "$shown", which is not a part of the printer';
			var centerMetres = [for (coordinate in center) coordinate * metresPerUnit];
			var filament = job.filamentDiameter, lag = job.meltLag;
			var margin = 0.001;
			var low = [for (axis in 0...3) extent.low[axis] - margin], high = [for (axis in 0...3) extent.high[axis] + margin];
			// The Z grid has a ray a spacing; a bed's worth of beads at a fifth of their height would be too many.
			newStock = () -> new PrintingStock(low, high, centerMetres,
				Math.max(stockSpacing, Math.sqrt((high[0] - low[0]) * (high[1] - low[1]) / STOCK_RAYS)), filament, lag, featureAt);
			stockObject = "project:" + shown;
		}
		var robotIndex = robot.parts[0].robotIndex;
		var axisJoint = new Map<String, Int>();
		for (index in 0...job.axes.length)
			for (joint in 0...joints) if (machineModel.joints[joint].id == job.axes[index]) axisJoint.set(job.axes[index], joint);
		encoders = new EncoderMonitor(robot.model, [for (_ in robot.model.joints) 0.0]);
		slip = new StepperSlip(robot.robot.description().couplings, axisJoint,
			(joint, offset) -> simulation.setJointSlip(robotIndex, joint, offset));
		// The printer starts referenced: there is no homing to run.
		newHoming = () -> null;
		homingComplete = true;
		var tolerances = completionTolerances(machineModel, job);
		newMotion = () -> {
			var state = robot.runtime.snapshot();
			var currentBinding = new ToolpathMotionBinding(machine, binding.blueprint, state.modelRevision, state.calibrationRevision);
			currentBinding.compiler.planCheck = binding.compiler.planCheck;
			var made = new ManipulatorMotion(robot.robot, currentBinding.compiler, channel -> input(channel),
				() -> robot.runtime.pollEvents(), axisJoints, tolerances);
			made.slip = slip;
			return made;
		};
		motion = newMotion();
	}

	/**
	 * How far each joint may stand from where a plan ends before the plan is not done: a few microsteps of an axis,
	 * and what that comes to on the joints that turn with the axes (a joint's tolerance is the sum over its
	 * couplings of the coupling's ratio times its leader's) and on the extruder (the same length of filament).
	 */
	static function completionTolerances(model:robotkit.model.RobotModel, job:SceneArtifactPrinting):Array<Float> {
		var ids = [for (joint in model.joints) joint.id];
		var known = [for (_ in ids) -1.0];
		for (id in job.axes) known[ids.indexOf(id)] = POSITION_TOLERANCE;
		known[ids.indexOf(job.extruder)] = POSITION_TOLERANCE / job.gearRadius;
		for (_ in ids) for (joint in 0...ids.length) if (known[joint] < 0) {
			var sum = 0.0, ready = true, any = false;
			for (coupling in model.couplings) if (coupling.follower == ids[joint]) {
				any = true;
				var leader = known[ids.indexOf(coupling.leader)];
				if (leader < 0) ready = false; else sum += Math.abs(coupling.ratio) * leader;
			}
			if (any && ready) known[joint] = sum;
		}
		// A joint nothing moves holds where it is, and ends a plan where it began.
		return [for (value in known) value < 0 ? POSITION_TOLERANCE : value];
	}

	/** A digital input of the program: whether a heater is at its target, from its sensor's last frame. */
	function input(channel:String):Null<EventValue> {
		for (heater in job.heaters) if (channel == channels.heaterAtTemperature(heater.name)) {
			var frames = robot.robot.snapshot().sensors;
			for (index in 0...frames.length) {
				var frame = frames.get(index);
				if (frame.sensorId == heater.sensor)
					return EventValue.Digital(frame.values.length > HeaterSensor.AT_TARGET && frame.values.get(HeaterSensor.AT_TARGET) >= 0.5);
			}
			return EventValue.Digital(false);
		}
		return null;
	}

	override function waiting():Bool {
		if (session.simulationTime() < readyAt) return true;
		if (!synchronised) {
			// A virtual device counts its steps from zero, which is the machine's origin: once it has taken over the joints
			// the machine stands there, wherever the assembly's initial pose was, and the program runs from that position.
			synchronised = true;
			try compileFromRobot() catch (error:Dynamic) { fail('The print program: $error'); return true; }
		}
		return false;
	}

	override function beforeStep():Void {
		// The device's reports reach the runtime's sensors before the program reads them.
		var seen = heaters.observe();
		for (index in 0...job.heaters.length) {
			readings.set(job.heaters[index].name, seen[index]);
			var message = HeaterSensor.faultMessage(seen[index].fault);
			if (message != null) fail('$message (${job.heaters[index].name})');
		}
	}

	override function afterStep(at:{op:Int, distance:Float}, provenance:Null<Provenance>, passFinished:Bool):Void {
		layer = countThrough(layerLines, currentLine);
		if (newStock == null || extruderIndex < 0) return;
		if (stock == null) stock = newStock();
		var clock = Sys.time();
		// What the machine did, not what was asked: the joints where the simulation has them.
		var positions = robot.robot.snapshot().positions;
		var pose = solver.forward([for (index in axisJoints) positions.get(index)]);
		var fed = positions.get(extruderIndex) * gearRadius;
		stock.follow(new Point3(pose.x, pose.y, pose.z), fed, heightAt(currentLine), session.fixedTimestep(), at.op,
			provenance == null ? new Provenance(0, 0, 0) : provenance);
		depositSeconds += Sys.time() - clock;
	}

	/** Shows the printed material as it grows: a few times a second it is contoured on its own thread and the chunks that changed are shown. */
	override public function present():Void {
		var laid = stock, id = stockObject;
		if (laid == null || id == null || !showPrint) return;
		var clock = Sys.time();
		var chunks = laid.takePreview();
		if (chunks != null) project.scene.setRuntimeGeometryParts(id, laid.previewChunks(), chunks);
		var now = session.simulationTime();
		if (laid.hasChanged() && (now - stockShownAt >= STOCK_REFRESH || now < stockShownAt) && laid.refreshPreview())
			stockShownAt = now;
		meshingSeconds += Sys.time() - clock;
	}

	/** Stops printing and gives the printed part's object its own geometry back. */
	public function dispose():Void {
		if (stock != null) stock.dispose();
		stock = null;
		if (stockObject != null) project.scene.clearRuntimeGeometry(stockObject);
	}

	/**
	 * The program's heights and features by G-code line, and the box its beads lie in (machine coordinates, metres):
	 * the stock for the print is as big as the print.
	 */
	function scanProgram(compiled:ToolpathProgram):{low:Array<Float>, high:Array<Float>} {
		var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, 0.0];
		var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, 0.0];
		var depositing = false, width = 0.0, height = 0.0;
		heightLines = []; heights = []; featureLines = []; features = [];
		function include(point:Point3):Void {
			low[0] = Math.min(low[0], point.x - width / 2.0); high[0] = Math.max(high[0], point.x + width / 2.0);
			low[1] = Math.min(low[1], point.y - width / 2.0); high[1] = Math.max(high[1], point.y + width / 2.0);
			high[2] = Math.max(high[2], point.z);
		}
		for (op in compiled.ops) switch op {
			case Deposit(_, beadWidth, beadHeight, provenance):
				depositing = true; width = beadWidth; height = beadHeight;
				heightLines.push(provenance.line); heights.push(beadHeight);
			case DepositOff(_): depositing = false;
			case Feature(kind, provenance): featureLines.push(provenance.line); features.push(kind);
			case Move(Cut, geometry, _, _, _) if (depositing):
				var length = GeometryTools.length(geometry);
				for (step in 0...Std.int(Math.ceil(length / 0.0005)) + 1)
					include(GeometryTools.pointAt(geometry, Math.min(length, step * 0.0005)));
			case _:
		}
		if (!(low[0] <= high[0])) { low = [0.0, 0.0, 0.0]; high = [0.001, 0.001, 0.001]; }
		return {low: low, high: high};
	}

	/** How many of `lines` (ascending) are at or before `line`. */
	static function countThrough(lines:Array<Int>, line:Int):Int {
		var low = 0, high = lines.length;
		while (low < high) {
			var middle = (low + high) >> 1;
			if (lines[middle] <= line) low = middle + 1; else high = middle;
		}
		return low;
	}

	/** The feature a G-code line is printed as, or null before any is named. */
	function featureAt(line:Int):Null<PrintFeature> {
		var count = countThrough(featureLines, line);
		return count == 0 ? null : features[count - 1];
	}

	/** The height of the bead the program lays at a G-code line: the last it set at or before it. */
	function heightAt(line:Int):Float {
		var count = countThrough(heightLines, line);
		if (count > 0) lastHeight = heights[count - 1];
		return lastHeight;
	}

	/** A robot that faults while the print runs is told with the heaters' last reports: the controller's fault shows there first. */
	override function failureReason(reason:String):String {
		var seen = [for (name in heaterNames()) {
			var reading = readings.get(name);
			reading == null ? '$name (no report)' : '$name ${Math.round(reading.celsius)}/${Math.round(reading.targetCelsius)} C duty ${Math.round(reading.duty * 100)} % fault ${reading.fault}';
		}];
		return '$reason (${seen.join(", ")}; line $currentLine, layer $layer)';
	}

	override function resetJob():Void {
		if (stock != null) {
			stock.dispose();
			stock = null;
			stockShownAt = Math.NEGATIVE_INFINITY;
		}
		if (stockObject != null) project.scene.clearRuntimeGeometry(stockObject);
		lastHeight = 0.0;
		heaters.reset();
		readings.clear();
		layer = 0;
		readyAt = session.simulationTime() + SYNC_DELAY;
		synchronised = false;
	}

	/** The latest report of heater `name` (`hotend`, `bed`), or null before the device has reported. */
	public function heaterReading(name:String):Null<HeaterReading> return readings.get(name);

	public function heaterNames():Array<String> return [for (heater in job.heaters) heater.name];

	/** The program's G-code is a print: the panel shows its layer and temperatures. */
	override public function statusDetail():String {
		var text = ' · layer $layer/$layerCount';
		for (name in heaterNames()) {
			var reading = readings.get(name);
			if (reading != null) text += ' · $name ${Math.round(reading.celsius)}/${Math.round(reading.targetCelsius)} °C';
		}
		return text;
	}

	override public function title():String return "PRINT PROGRAM";
}
