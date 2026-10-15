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

	/**
		The device actuator that drives the extruder joint `joint`, and which way its steps count as it pushes filament: the
		motor may turn either way (the printer's turns backwards, through one gear mesh).
	**/
	public static function extruderOnDevice(binding:robotkit.device.DeviceBinding, joint:String):robotkit.runtime.VirtualHeaters.HeaterExtruder {
		for (channel in binding.channels) if (binding.model.joints[channel.jointIndex].id == joint)
			return robotkit.runtime.VirtualHeaters.HeaterExtruder.onChannel(channel.channel, channel.ratio);
		throw 'The printer\'s extruder joint "$joint" has no motor on the device';
	}

	/** The virtual heaters a job asks for, with the extruder's motor on the device as `extruder` says. */
	public static function heaterSetups(job:SceneArtifactPrinting, extruder:robotkit.runtime.VirtualHeaters.HeaterExtruder):Array<HeaterSetup> {
		var result:Array<HeaterSetup> = [];
		for (heater in job.heaters) {
			var setup = new HeaterSetup(heater.name, heater.sensor, heater.slot, heater.target, {
				r0Ohm: heater.r0Ohm, t0Celsius: heater.t0Celsius, beta: heater.beta, pullupOhm: heater.pullupOhm, vRef: heater.vRef,
				kp: heater.kp, ki: heater.ki, kd: heater.kd, maxDuty: heater.maxDuty, maxTemperature: heater.maxTemperature,
				bandCelsius: heater.bandCelsius, pidRangeCelsius: heater.pidRangeCelsius, gainWindowSeconds: heater.gainWindowSeconds,
				gainMinCelsius: heater.gainMinCelsius, hysteresisCelsius: heater.hysteresisCelsius,
				hysteresisWindowSeconds: heater.hysteresisWindowSeconds, heatCapacity: heater.heatCapacity, power: heater.power,
				loss: heater.loss, fanLoss: heater.fanLoss, ambient: heater.ambient});
			setup.fanChannel = heater.fan;
			if (heater.feedsExtruder) {
				setup.extruder = extruder;
				setup.minExtrusionTemperature = job.minExtrusionTemperature;
			}
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
	/** The machine's axes and the extruder where physics has them (the links), not where the device reconstructs them. */
	final physicalJoints:PhysicalJoints;
	/** The joints (indices into the robot's) `physicalJoints` reads, in its order. */
	final physicalSlots:Array<Int>;
	/** Each spring coupling (a stiffness): its follower's and leaders' indices among the joints, with the leaders' ratios and the offset. */
	final springFollowers:Array<Int> = [];
	final springLeaders:Array<Array<Int>> = [];
	final springRatios:Array<Array<Float>> = [];
	final springOffsets:Array<Float> = [];
	/**
	 * The most any drive spring was ever stretched, as the distance the nozzle's axis is from where the spring's own pulley
	 * puts it (the follower's physical value against its leaders' through the coupling, over the leaders' largest ratio), in
	 * metres, and the time of it. Where the controller's view of the nozzle lags the physical one by the device's reporting
	 * latency (a speed times the interval), the stretch is time-aligned: all of it is physics.
	 */
	public var nozzleDeviation(default, null):Float = 0.0;
	public var nozzleDeviationAt(default, null):Float = 0.0;
	/** The G-code line under way when the largest stretch happened, and the stretch of each spring coupling then (metres). */
	public var nozzleDeviationLine(default, null):Int = 0;
	public var nozzleDeviationDetail(default, null):String = "";
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
			// The axis is as fast as the joints turning with it and their motors allow: belts, screws and the motors at their full rate.
			// What both motors of a CoreXY ask together is bounded by their rows in the plan (the pulleys' columns), not by a share here.
			var coupled = machineModel.coupledLimits(id, steady, true);
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
		if (job.shaping != null) {
			var soft = robotkit.model.DriveLoads.of(machineModel,null,[for (shaper in job.shaping) shaper.axis]);
			var hard = robotkit.model.DriveLoads.stiffest(machineModel,null,[for (shaper in job.shaping) shaper.axis]);
			for (shaper in job.shaping) {
				var shaped = InputShaper.named(shaper.kind,shaper.frequency,shaper.damping);
				for (load in soft) if (load.axis == shaper.axis && load.stiffness>0.0 && load.mass>0.0) {
					var upper = [for (candidate in hard) if (candidate.axis == load.axis) candidate][0];
					var lowerHz = Math.sqrt(load.stiffness/load.mass)/(2*Math.PI), upperHz = Math.sqrt(upper.stiffness/upper.mass)/(2*Math.PI);
					if (shaped.impulses.residualBound(lowerHz,upperHz,shaper.damping)>0.2)
						throw 'Fixed shaper for ${shaper.axis} does not cover physical travel: $lowerHz..$upperHz Hz';
				}
				printer.shapers.set(shaper.axis,shaped);
			}
		}
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
		var wanted = job.axes.concat([job.extruder]);
		function indexOf(id:String):Int return [for (index in 0...robot.model.joints.length) if (robot.model.joints[index].id == id) index][0];
		var springs = new Map<String, Int>();
		for (coupling in robot.model.couplings) if (coupling.stiffness > 0.0) {
			var at = springs.get(coupling.follower);
			if (at == null) {
				at = springFollowers.length;
				springs.set(coupling.follower, at);
				springFollowers.push(indexOf(coupling.follower)); springLeaders.push([]); springRatios.push([]); springOffsets.push(0.0);
				if (wanted.indexOf(coupling.follower) < 0) wanted.push(coupling.follower);
			}
			springLeaders[at].push(indexOf(coupling.leader));
			springRatios[at].push(coupling.ratio);
			springOffsets[at] += coupling.offset;
			if (wanted.indexOf(coupling.leader) < 0) wanted.push(coupling.leader);
		}
		physicalSlots = [for (id in wanted) indexOf(id)];
		physicalJoints = new PhysicalJoints(simulation, robotIndex, robot.model, physicalSlots);
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
		var frequencies = springFrequencies(machineModel);
		newMotion = () -> {
			var state = robot.runtime.snapshot();
			var currentBinding = new ToolpathMotionBinding(machine, binding.blueprint, state.modelRevision, state.calibrationRevision);
			currentBinding.compiler.planCheck = binding.compiler.planCheck;
			var made = new ManipulatorMotion(robot.robot, currentBinding.compiler, channel -> input(channel),
				() -> robot.runtime.pollEvents(), axisJoints, tolerances, frequencies);
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

	function indexOfJoint(id:String):Int {
		for (index in 0...robot.model.joints.length) if (robot.model.joints[index].id == id) return index;
		throw 'The printer has no joint "$id"';
	}

	/** The frequency (rad/s) each joint hangs on its drive's spring at, sqrt(K / m) of the axis it is, or 0 for a joint with none. */
	static function springFrequencies(model:robotkit.model.RobotModel):Array<Float> {
		var frequencies = [for (_ in model.joints) 0.0];
		for (load in robotkit.model.DriveLoads.of(model, new robotkit.model.SteadyLoads()))
			if (load.stiffness > 0.0 && load.mass > 0.0)
				for (index in 0...model.joints.length) if (model.joints[index].id == load.axis) frequencies[index] = Math.sqrt(load.stiffness / load.mass);
		return frequencies;
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
		// What the machine did, not what was asked, and not what the controller believes: the axes and the extruder where
		// physics has them (the links), so a belt's stretch or a carriage's swing reaches the deposit.
		var snapshot = robot.robot.snapshot();
		var reported = [for (index in axisJoints) snapshot.positions.get(index)];
		var actual = reported.copy();
		var values = physicalJoints.read([for (slot in physicalSlots) reported[slot]], session.simulationTime());
		for (slot in 0...physicalSlots.length) actual[physicalSlots[slot]] = values[slot];
		var pose = solver.forward(actual);
		var fed = actual[extruderIndex] * gearRadius;
		for (spring in 0...springFollowers.length) {
			var put = springOffsets[spring], scale = 0.0;
			for (leader in 0...springLeaders[spring].length) {
				put += springRatios[spring][leader] * actual[springLeaders[spring][leader]];
				scale = Math.max(scale, Math.abs(springRatios[spring][leader]));
			}
			var stretch = Math.abs(actual[springFollowers[spring]] - put) / scale;
			if (stretch > nozzleDeviation) {
				nozzleDeviation = stretch; nozzleDeviationAt = session.simulationTime(); nozzleDeviationLine = currentLine;
				nozzleDeviationDetail = [for (other in 0...springFollowers.length) {
					var sum = springOffsets[other], largest = 0.0;
					for (leader in 0...springLeaders[other].length) {
						sum += springRatios[other][leader] * actual[springLeaders[other][leader]];
						largest = Math.max(largest, Math.abs(springRatios[other][leader]));
					}
					robot.model.joints[springFollowers[other]].id + " " + Math.round((actual[springFollowers[other]] - sum) / largest * 1e6) + " um";
				}].join(", ") + ", reported speeds " + [for (axis in job.axes) Math.round(snapshot.velocities.get(indexOfJoint(axis)) * 1e3) + " mm/s"].join(" ");
			}
		}
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
	public function dispose(?releases:Array<Void->Void>):Void {
		var retiredStock = stock;
		if (retiredStock != null) {
			if (releases == null) retiredStock.dispose(); else releases.push(function() retiredStock.dispose());
		}
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
		nozzleDeviation = 0.0;
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
