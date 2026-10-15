package app;

import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import nativekit.scene.GeometryData;
import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.CncDiagnostic.CncSeverity;
import cnckit.controller.RobotInterface;
import cnckit.controller.RobotInterfaceAlarm;
import cnckit.controller.RobotInterfaceChannels;
import cnckit.controller.RobotInterfaceInput;
import cnckit.controller.RobotInterfaceOutputs;
import robotkit.model.Joint;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import robotkit.model.SteadyLoads;
import robotkit.runtime.Simulation;
import app.SimulationAssemblyParts.SimulationAssemblyPart;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;
import toolpathkit.motion.ToolpathSourceMap;
import toolpathkit.path.MoveKind;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.ToolpathFrame;
import toolpathkit.path.GeometryTools;
import toolpathkit.tool.Tool;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.event.EventValue;
import motionkit.program.MotionProgram;
import motionkit.robot.AxisKinematics;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.robot.MotionSystem;
import motionkit.robot.PlanCheck;
import motionkit.robot.PlanningLimits;
import motionkit.robot.PlanCheck.PlanCheckOptions;
import motionkit.robot.PlanCheckSummary;
import motionkit.robot.StepperSlip;
import robotkit.runtime.EncoderMonitor;
import nativekit.sim.SimSession;
import nativekit.sim.SimPose;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathChannels;
import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.motion.MachiningContinuation;
import toolpathkit.motion.MachiningRecipe;
import toolpathkit.motion.MachiningRun;
import motionkit.robot.SessionState;
import toolpathkit.path.Point3;

/**
 * A project's machining job, as its generator describes it (see SceneArtifactMachining), with
 * lengths in metres. Joint ids name the machine's X, Y and Z axes, whose positions are machine
 * coordinates of the spindle's gauge line: the origin of the `spindle` part, whose +Z runs up the
 * spindle axis. Each tool hangs its length below it.
 */
typedef CncJobStock = {
	var occurrence:String;
	var minimum:Array<Float>;
	var maximum:Array<Float>;
	var center:Array<Float>;
	var density:Float;
	var mesh:{positions:Array<Float>, indices:Array<Int>};
}

typedef CncJob = {
	@:optional var powerUpSideOffsets:Array<materia.project.SceneArtifact.SceneArtifactPowerUpSideOffset>;
	@:optional var powerUpOffsets:Array<materia.project.SceneArtifact.SceneArtifactPowerUpOffset>;
	var source:String;
	var axes:Array<String>;
	var spindle:String;
	/** Explicit process-channel namespace; independent of the spindle occurrence path. */
	var channelScope:String;
	/** G54 in machine coordinates. */
	var workOffset:Array<Float>;
	/** The tool table; G43 H numbers are tool numbers. */
	var tools:Array<Tool>;
	var loop:Bool;
	/** All stock occurrences the job may cut, selected by the machine's part-presence input. */
	var stocks:Array<String>;
	/** Each stock occurrence's raw geometry and CAD bounds in metres. */
	var stockSources:Array<CncJobStock>;
	/** The finished part, a closed mesh in the stock part's frame: the cut stock is compared with it. */
	@:optional var target:{positions:Array<Float>, indices:Array<Int>};
	/** The part showing the tool in the spindle; each loaded tool's shape replaces its geometry. */
	@:optional var toolPart:String;
	/** The tool in the spindle when the job starts. */
	@:optional var loadedTool:Int;
	/** The controller the machine's steppers are nominally wired to; its step rate caps the axes. */
	@:optional var controller:{stepTickHz:Int};
	@:optional var controllerId:String;
}

/**
 * Runs a project's CNC job on its simulated assembly robot: the G-code is compiled against the
 * machine's axes and streamed to the robot runtime as trajectory segments by MotionKit, before each
 * simulation tick. A looping job starts again once the program completes.
 */
class CncProgramPlayer extends ToolpathPlayer {
	/** Stock geometry goes to the scene at most this often, in simulated seconds. */
	static inline final STOCK_REFRESH = 0.05;
	/** Ray spacing of the simulated stock, in metres. */
	static inline final STOCK_SPACING = 0.0005;
	/** Assumed encoder deadband: the cell treats the spindle as stopped below 5 rpm. */
	static inline final SPINDLE_STANDSTILL_RPM = 5.0;
	static final INTERFACE_ALARM_CODE_BITS = RobotInterfaceChannels.alarmCodeBits();

	/** Spindle and coolant channels a CNC program's events need on the machine's runtime. */
	public static function channelsFor(job:CncJob):ToolpathChannels {
		return new ToolpathChannels(job.channelScope);
	}

	public static function processChannels(job:CncJob, interfaceManaged:Bool = false):Array<ProcessChannelDeclaration> {
		var channels = channelsFor(job);
		var result = [
			new ProcessChannelDeclaration(channels.spindleSpeed, ProcessEventValue.Analog(0.0)),
			new ProcessChannelDeclaration(channels.spindleDirection, ProcessEventValue.Analog(0.0)),
			new ProcessChannelDeclaration(channels.coolantMist, ProcessEventValue.Digital(false)),
			new ProcessChannelDeclaration(channels.coolantFlood, ProcessEventValue.Digital(false))
		];
		if (interfaceManaged) result.push(new ProcessChannelDeclaration(
			RobotInterfaceChannels.alarmCodeChannel(job.channelScope), ProcessEventValue.Analog(0.0)));
		return result;
	}

	final project:ProjectDocumentSession;
	final job:CncJob;
	final simulation:Simulation;
	final assemblyParts:SimulationAssemblyParts;
	var kindBySource = new haxe.ds.ObjectMap<Provenance, MoveKind>();
	public var rapidDisplacement(default, null):Array<Float> = [];
	final stockSources:Map<String, {part:SimulationAssemblyPart, source:CncJobStock}> = new Map();
	var stockObject:Null<String>;
	var activeStockOccurrence:Null<String>;
	var activeRawVolume:Float = 0.0;
	final spindlePart:SimulationAssemblyPart;
	final machineRobotIndex:Int;
	var activeStock:Null<MachiningStock> = null;
	final allStocks:Array<MachiningStock> = [];
	public final completedStocks:Array<{occurrence:String, removedVolume:Float, rawVolume:Float,
		finishedMass:Float, targetMass:Float, massError:Float, leftover:Float, gouge:Float,
		rapidContacts:Int, collisions:Int}> = [];
	final channels:ToolpathChannels;
	final toolsByNumber = new Map<Int, Tool>();
	/** The tool in the spindle at the start, and now; -1 for none. */
	final initialTool:Int;
	public var loadedTool(default, null):Int;
	/** The tool the scene shows, and how to draw one in the spindle, when the job names its part. */
	var shownTool = -1;
	final toolShape:Null<Tool->GeometryData>;
	final toolObject:Null<String>;
	/** The stock the job is cutting, when it has one. */
	public var stock(default, null):Null<MachiningStock> = null;
	var stockShownAt = Math.NEGATIVE_INFINITY;
	/** Wall-clock seconds spent cutting and meshing the stock, for profiling. */
	public var cuttingSeconds(default, null):Float = 0.0;
	public var meshingSeconds(default, null):Float = 0.0;
	/** Fraction of active CNC-cycle time with the spindle turning. */
	public var spindleUtilization(default, null):Float = 0.0;
	var spindleIntervalSeconds:Float = 0.0;
	var spindleTurningSeconds:Float = 0.0;
	var commandedKind:MoveKind = MoveKind.Rapid;
	var commandedOp = -1;
	var commandedProvenance = new Provenance(0, 0, 0);
	final interfaceManaged:Bool;
	var robotInterface:Null<RobotInterface>;
	final interfaceAlarmCodeChannel:Null<String>;
	final doorCoils:Null<{a:String, b:String}>;
	final viseCoils:Null<{a:String, b:String}>;
	final interfaceSensorIds:Null<{doorOpen:String, doorClosed:String, viseOpen:String, viseClamped:String, presence:String,
		robotZone:String}>;
	var loadPosition:Array<Float> = [];
	var parkingProgram:Null<MotionProgram> = null;
	var parkingStarted = false;
	var parkingComplete = false;
	var startRequested = false;
	var pendingToolChange = false;
	var lastInterfaceOutputs:Null<RobotInterfaceOutputs> = null;
	/** The interlock alarm currently published by the machine's robot interface. */
	public var interfaceAlarm(default, null):RobotInterfaceAlarm = RobotInterfaceAlarm.None;
	/** The machine's zone scanner reading at the last interface update: a robot body is in its zone. */
	public var robotInZone(default, null):Bool = false;
	final recipe:MachiningRecipe;
	/** Sensor selected from the assembled machine; null on routers without at-speed feedback. */
	public final atSpeedSensorId:Null<String>;
	final spindleVelocityJoint:Int;
	final cellInput:Null<String->Null<EventValue>>;
	/** The restart the operator asked for, until the machine can take it. */
	var restartRequest:Null<{op:Int, distance:Float}> = null;
	var pendingContinuation:Null<MachiningContinuation> = null;
	/** The restarted program running now, which maps back to the program's lines. */
	var continuation:Null<MachiningContinuation> = null;

	/**
	 * `robot` is `project`'s assembly, simulated in `simulation`. The simulated
	 * joints read from the assembly's starting pose, so each axis maps machine coordinates onto them
	 * with that pose as its offset.
	 */
	public function new(job:CncJob, robot:AssemblyRobot, simulation:Simulation,
			project:ProjectDocumentSession, session:SimSession, assemblyParts:SimulationAssemblyParts,
			?cellInput:String->Null<EventValue>,
			?check:PlanCheckOptions, interfaceManaged:Bool = false) {
		super(robot, session, job.source, job.loop, check);
		if (job.axes.length != 3) throw "A CNC job needs its X, Y and Z axes";
		var definition = project.projectAssemblyDefinition, physical = project.projectPhysical;
		if (definition == null || physical == null) throw "A CNC job needs the project's machine";
		var metresPerUnit = physical.metresPerUnit, state = project.projectAssemblyState;
		channels = channelsFor(job);
		this.job = job;
		this.simulation = simulation;
		this.assemblyParts = assemblyParts;
		this.cellInput = cellInput;
		this.project = project;
		this.interfaceManaged = interfaceManaged;
		robotInterface = interfaceManaged ? new RobotInterface() : null;
		interfaceAlarmCodeChannel = interfaceManaged ? RobotInterfaceChannels.alarmCodeChannel(job.channelScope) : null;
		var placement = new AssemblyState(definition, state);
		var steady = checkOptions.steady;
		// Apply the controller pulse ceiling to stepper planning.
		var machineModel = robot.model;
		var wiring = job.controller;
		if (wiring != null)
			machineModel = DeviceBinding.planningModel(robot.model, wiring.stepTickHz);
		var axes:Array<MotionAxisBlueprint> = [];
		var start:Array<Float> = [];
		// Rapids ask for the fastest axis speed; the planner still holds each joint to its own limit.
		var rapid = 0.0;
		for (id in job.axes) {
			var joint = [for (joint in definition.joints) if (joint.id == id) joint];
			if (joint.length != 1 || Std.string(joint[0].type) != "prismatic")
				throw 'CNC axis "$id" must be a prismatic joint of the machine';
			var limits = joint[0].limits;
			var lower = limits.lower, upper = limits.upper;
			if (lower == null || upper == null) throw 'CNC axis "$id" needs travel limits';
			var initial = placement.joint(id) * metresPerUnit;
			start.push(initial);
			// The axis is as fast as the joints turning with it and their motors allow, such as its
			// lead screw and the motor turning it.
			// Gravity, rail friction and the drives' drag come off what the motors can accelerate with.
			var coupled = machineModel.coupledLimits(id, steady);
			if (coupled.velocity == null || coupled.maxAcceleration == null ||
					!(coupled.velocity > 0) || !(coupled.maxAcceleration > 0))
				throw 'CNC axis "$id" needs velocity and acceleration limits, its own or its motors\'';
			rapid = Math.max(rapid, coupled.velocity);
			axes.push(new MotionAxisBlueprint(id, [id], lower * metresPerUnit, upper * metresPerUnit,
				coupled.velocity, coupled.maxAcceleration, initial, [1.0], [-initial]));
		}
		// Plan over the three axes alone: the machine's other joints are fixed mounts, and planning
		// them all made compiling a short program take many seconds.
		// A restart climbs to just below the top of Z travel before crossing to where it resumes.
		var top = [for (joint in definition.joints) if (joint.id == job.axes[2]) joint][0].limits.upper;
		if (top == null) throw "The machine's Z axis needs an upper limit";
		recipe = new MachiningRecipe(0.01, 0.0, top * metresPerUnit - 0.001);
		var planning = planningModel(machineModel, job.axes, steady);
		var machine = new MachineBinding("machine", job.axes[0], job.axes[1], job.axes[2], rapid, channels);
		rapidDisplacement = interfaceManaged ? [0.0, 0.0, 0.0] : robotkit.model.DriveLoads.rapidDisplacement(machineModel, job.axes, steady);
		var clearanceStocks:Array<toolpathkit.motion.RapidClearance.ClearanceStock> = [];
		// Fixed stock has a known machine frame at compile time. Robot-tended blanks move before their CNC cycle;
		// those retain their existing cell planning until its active-stock pose is available to the compiler.
		if (!interfaceManaged) for (source in job.stockSources) {
			var relative = AssemblyFrames.compose(AssemblyFrames.inverse(placement.worldPose(job.spindle)), placement.worldPose(source.occurrence));
			var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
			var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
			for (x in [source.minimum[0], source.maximum[0]]) for (y in [source.minimum[1], source.maximum[1]])
				for (z in [source.minimum[2], source.maximum[2]]) {
					var point = AssemblyFrames.transformPoint(relative, x/metresPerUnit, y/metresPerUnit, z/metresPerUnit);
					var coordinates = [point.x*metresPerUnit+start[0], point.y*metresPerUnit+start[1], point.z*metresPerUnit+start[2]];
					for (axis in 0...3) { low[axis] = Math.min(low[axis], coordinates[axis]); high[axis] = Math.max(high[axis], coordinates[axis]); }
				}
			clearanceStocks.push({minimum:low, maximum:high});
		}
		var binding = new ToolpathMotionBinding(machine,
			new MotionSystemBlueprint(planning.model, robot.blueprint, axes, session.fixedTimestep()));
		// Every plan the compiler makes is checked against the motors' drives: torque along the plan and
		// the drives' stretch. The findings are on the plan, and summed in `planChecks()`.
		binding.compiler.planCheck = new PlanCheck(machineModel, job.axes, checkOptions);
		var controller = new CncController();
		for (tool in job.tools) {
			controller.toolLibrary.set(tool);
			toolsByNumber.set(tool.number, tool);
		}
		controller.setWorkOffset(54, job.workOffset[0], job.workOffset[1], job.workOffset[2]);
		// A program runs from wherever the machine is, so each pass compiles it from there.
		compileFrom = position -> {
			var compiled = CncCompiler.compileDetailed(job.source, controller, position, machine.travel);
			var sourceLines = job.source.split("\n");
			var errors = [for (diagnostic in compiled.diagnostics) if (diagnostic.severity == CncSeverity.Error) {
				var message = diagnostic.toString();
				if (diagnostic.span.line > 0 && diagnostic.span.line <= sourceLines.length)
					message += ' [${sourceLines[diagnostic.span.line - 1]}]';
				message;
			}];
			if (errors.length > 0) throw 'The machining program from [${position.x}, ${position.y}, ${position.z}] with G54 ${job.workOffset}: ${errors.join("; ")}';
			loadPosition = finalMachinePosition(compiled.program);
			var protectedProgram = toolpathkit.motion.RapidClearance.apply(compiled.program, clearanceStocks, rapidDisplacement,
				job.loadedTool == null ? -1 : job.loadedTool);
			kindBySource = new haxe.ds.ObjectMap<Provenance, MoveKind>();
			for (op in protectedProgram.ops) switch op {
				case Move(kind, _, _, _, provenance), MachineMove(kind, _, _, _, provenance): kindBySource.set(provenance, kind);
				case _:
			}
			var lowered = ToolpathMotion.lower(protectedProgram, machine);
			if (lowered.program == null)
				throw 'The machining program: ${[for (diagnostic in lowered.diagnostics) diagnostic.message].join("; ")}';
			program = lowered.program;
			sourceMap = lowered.sourceMap;
			return compiled.program;
		};
		if (interfaceManaged) {
			if (job.controllerId == null || cellInput == null)
				throw "A robot-tended CNC job needs controller ownership and cell signals";
			var separator = job.controllerId.lastIndexOf("/");
			var machineRoot = separator < 0 ? "" : job.controllerId.substr(0, separator);
			function path(id:String):String return machineRoot.length == 0 ? id : machineRoot + "/" + id;
			var door = pneumaticDriveFor(robot, path("door"));
			var vise = pneumaticDriveFor(robot, path("mill/vise/jaw"));
			var doorB = door.channelB, viseB = vise.channelB;
			if (doorB == null || viseB == null)
				throw "The door and vise need double-solenoid cylinder outputs";
			doorCoils = {a:door.channelA, b:doorB};
			viseCoils = {a:vise.channelA, b:viseB};
			interfaceSensorIds = {doorOpen:path("door-open"), doorClosed:path("door-closed"),
				viseOpen:path("vise-open"), viseClamped:path("vise-clamped"), presence:path("stock-present"),
				robotZone:path("robot-zone")};
			for (id in [interfaceSensorIds.doorOpen, interfaceSensorIds.doorClosed,
				interfaceSensorIds.viseOpen, interfaceSensorIds.viseClamped, interfaceSensorIds.presence,
				interfaceSensorIds.robotZone])
				if (!hasSensor(id)) throw 'Robot-tended CNC machine is missing field sensor "$id"';
		} else {
			doorCoils = null;
			viseCoils = null;
			interfaceSensorIds = null;
		}
		var spindleLink = robot.part("project:" + job.spindle);
		this.spindlePart = assemblyParts.get("project:" + job.spindle);
		machineRobotIndex = spindleLink.robotIndex;
		var sensorId:Null<String> = null;
		var sensorJoint = -1;
		for (sensor in robot.blueprint.sensors) if (sensor.kind == "at_speed" && sensor.joint >= 0 &&
				robot.blueprint.joints[sensor.joint].childLink == spindleLink.linkIndex) {
			if (sensorId != null) throw 'Spindle "${job.spindle}" has multiple readiness sensors';
			sensorId = sensor.id;
			sensorJoint = sensor.joint;
		}
		var processOwned = robot.blueprint.processJoints();
		for (joint in robot.blueprint.joints) if (joint.childLink == spindleLink.linkIndex &&
				processOwned[joint.joint] && sensorId == null)
			throw 'Process spindle "${job.spindle}" needs its own readiness sensor';
		atSpeedSensorId = sensorId;
		spindleVelocityJoint = sensorJoint;
		solver = binding.solver;
		axisJoints = planning.indices;
		// Compiling from the starting pose now reports a bad program before the simulation starts.
		var compiled = compileFrom(new Point3(start[0], start[1], start[2]));
		parkingComplete = !interfaceManaged;
		if (interfaceManaged) {
			var parkSource = 'G21 G90 G53 G0 X${machineMillimetres(loadPosition[0])} ' +
				'Y${machineMillimetres(loadPosition[1])} Z${machineMillimetres(loadPosition[2])}\nM2';
			var parked = CncCompiler.compileDetailed(parkSource, controller,
				new Point3(start[0], start[1], start[2]), machine.travel);
			var parkErrors = [for (diagnostic in parked.diagnostics) if (diagnostic.severity == CncSeverity.Error) diagnostic.toString()];
			if (parkErrors.length > 0) throw 'Machine load-position move: ${parkErrors.join("; ")}';
			var loweredPark = ToolpathMotion.lower(parked.program, machine);
			if (loweredPark.program == null) throw "Machine load-position move did not lower";
			var parkProgram = loweredPark.program;
			if (parkProgram == null) throw "Machine load-position move did not lower";
			parkingProgram = parkProgram;
		}
		initialTool = job.loadedTool == null ? -1 : job.loadedTool;
		loadedTool = initialTool;
		stockObject = null;
		activeStockOccurrence = null;
		if (job.stocks == null || job.stockSources == null || job.stockSources.length != job.stocks.length)
			throw "CNC job stock occurrences and generated stock geometry do not match";
		for (source in job.stockSources) {
			if (job.stocks.indexOf(source.occurrence) < 0 || stockSources.exists(source.occurrence))
				throw 'CNC stock geometry has an unexpected occurrence "${source.occurrence}"';
			var part = assemblyParts.get("project:" + source.occurrence);
			if (interfaceManaged && !part.freeObject)
				throw 'Robot-tended CNC stock "${source.occurrence}" must be a free dynamic part';
			stockSources.set(source.occurrence, {part: part, source: source});
		}
		toolObject = job.toolPart == null ? null : "project:" + job.toolPart;
		toolShape = job.toolPart == null ? null : toolShapeIn(job.toolPart, job.spindle, placement, project, metresPerUnit);
		// Spindle-speed handshakes read the assembled machine's at-speed sensor. A router with no
		// spindle feedback keeps the existing ready behavior; tool changes remain operator inputs.
		var robotIndex = spindleLink.robotIndex;
		var axisJoint = new Map<String, Int>();
		for (index in 0...job.axes.length) axisJoint.set(job.axes[index], planning.indices[index]);
		for (_ in axisJoints) axisTrackingPeak.push(0.0);
		encoders = new EncoderMonitor(robot.model, [for (_ in robot.model.joints) 0.0]);
		slip = new StepperSlip(robot.robot.description().couplings, axisJoint,
			(joint, offset) -> simulation.setJointSlip(robotIndex, joint, offset));
		newHoming = () -> {
			var hasHomes = false;
			for (contact in robot.blueprint.switches) if (contact.role == "home") hasHomes = true;
			if (!hasHomes) return null;
			var view = new MotionSystem(robot.robot,
				new MotionSystemBlueprint(machineModel, robot.blueprint, axes, session.fixedTimestep()));
			view.configureRuntimeHoming(robot.runtime, () -> {
				slip.rebaseAfterHoming();
				encoders.reset(robot.runtime.snapshot().q.toArray());
			}, simulation.homingSides(robotIndex));
			return view;
		};
		homing = newHoming();
		homingComplete = homing == null;
		newMotion = () -> {
			var state = robot.runtime.snapshot();
			var currentBinding = new ToolpathMotionBinding(machine, binding.blueprint, state.modelRevision, state.calibrationRevision);
			currentBinding.compiler.planCheck = binding.compiler.planCheck;
			var made = new ManipulatorMotion(robot.robot, currentBinding.compiler,
				channel -> {
					if (channel == channels.spindleAtSpeed) return spindleAtSpeed();
					if (!StringTools.startsWith(channel, channels.toolChangePrefix)) return cellInput == null ? null : cellInput(channel);
					var number = Std.parseInt(channel.substr(channels.toolChangePrefix.length));
					if (number == null || !toolsByNumber.exists(number)) throw 'The machining program loads unknown tool $channel';
					if (number != loadedTool) {
						if (interfaceManaged) {
							pendingToolChange = true;
						return EventValue.Digital(false);
						}
						loadedTool = number;
						if (stock != null) stock.load(toolsByNumber.get(number));
					}
					return EventValue.Digital(true);
				},
				() -> robot.runtime.pollEvents(), planning.indices,
					[for (_ in planning.indices) 1e-5]);
			made.slip = slip;
			return made;
		};
		motion = newMotion();
	}

	function spindleAtSpeed():EventValue {
		var sensorId = atSpeedSensorId;
		if (sensorId == null) return EventValue.Digital(true);
		var frames = robot.robot.snapshot().sensors;
		for (index in 0...frames.length) {
			var frame = frames.get(index);
			if (frame.sensorId == sensorId)
				return EventValue.Digital(frame.values.length > 0 && frame.values.get(0) >= 0.5);
		}
		return EventValue.Digital(false);
	}

	/**
	 * Geometry of a tool in the spindle for part `toolPart`'s scene object: the tool's profile hung its
	 * length below the gauge line, in the part's frame shifted to its preview centre, in metres.
	 */
	static function toolShapeIn(toolPart:String, spindle:String, placement:AssemblyState, project:ProjectDocumentSession,
			metresPerUnit:Float):Tool->GeometryData {
		var definition = project.projectAssemblyDefinition;
		var occurrence = definition == null ? [] : [for (item in definition.occurrences) if (item.id == toolPart) item];
		var center = occurrence.length == 1 ? project.assemblyPreviewCenter(occurrence[0].definition) : null;
		if (center == null) throw 'CNC tool part "$toolPart" is not a part of the machine';
		var fromSpindle = AssemblyFrames.compose(AssemblyFrames.inverse(placement.worldPose(toolPart)),
			placement.worldPose(spindle));
		return tool -> CutterGeometry.revolved(tool.profile(), (x, y, z) -> {
			var local = AssemblyFrames.transformPoint(fromSpindle, x / metresPerUnit, y / metresPerUnit,
				(z - tool.length) / metresPerUnit);
			return [(local.x - center[0]) * metresPerUnit, (local.y - center[1]) * metresPerUnit,
				(local.z - center[2]) * metresPerUnit];
		}, (x, y, z) -> {
			var turned = AssemblyFrames.transformVector(fromSpindle, x, y, z);
			return [turned.x, turned.y, turned.z];
		});
	}

	/** A tended machine first moves to its load position, where the robot can reach the vise. */
	override function prepare():Bool {
		if (!interfaceManaged || parkingComplete) return true;
		var park = parkingProgram;
		if (park == null) throw "Robot-tended CNC machine has no load-position program";
		if (!parkingStarted) {
			motion.run(park);
			parkingStarted = true;
		}
		motion.update(session.fixedTimestep());
		if (motion.failure != null) {
			fail('Machine load-position move: ${motion.failure}');
			return false;
		}
		if (motion.completed) {
			parkingComplete = true;
			compileFrom(machinePositionAtMotionEnd());
			motion = newMotion();
			if (speedOverride != 1.0) motion.setSpeedOverride(speedOverride);
		}
		return false;
	}

	override function beforeStep():Void takeRestart();

	override function exchangeSignals(passFinished:Bool):Void {
		if (interfaceManaged) updateRobotInterface(passFinished);
	}

	/** A tended machine starts a pass when the cell asks for one; a looping one waits for an operator's restart to take. */
	override function startsPass():Bool {
		if (interfaceManaged)
			return startRequested && (!started || (motion.completed && !motion.running && passCounted));
		return super.startsPass() && (!started || pendingContinuation == null);
	}

	override function beginPass():Void {
		if (job.stocks.length > 0) {
			if (interfaceManaged) activateStock(stockAtPresenceInput());
			else if (activeStock == null) activateStock(job.stocks[0]);
		}
		continuation = null;
		startRequested = false;
		pendingToolChange = false;
	}

	/** A restart aborts the program on purpose; that is not a failure. */
	override function stoppedOnPurpose():Bool return pendingContinuation != null || restartRequest != null;

	override function originalAt(op:Int, distance:Float):{op:Int, distance:Float}
		return continuation == null ? {op: op, distance: distance} : continuation.originalAt(op, distance);

	override function afterStep(at:{op:Int, distance:Float}, provenance:Null<Provenance>, passFinished:Bool):Void {
		if (encoders.readings.length > 0) {
			var spindleWindow = if (interfaceManaged)
				lastInterfaceOutputs != null && lastInterfaceOutputs.inCycle
			else started && motion.running;
			if (spindleWindow) {
				var velocities = robot.robot.snapshot().velocities;
				var elapsed = session.fixedTimestep();
				spindleIntervalSeconds += elapsed;
				if (spindleVelocityJoint >= 0 && spindleVelocityJoint < velocities.length &&
					Math.abs(velocities.get(spindleVelocityJoint)) > SPINDLE_STANDSTILL_RPM * (2.0 * Math.PI / 60.0))
					spindleTurningSeconds += elapsed;
				spindleUtilization = spindleTurningSeconds / spindleIntervalSeconds;
			}
		}
		var cutting = activeStock;
		if (cutting == null) return;
		var clock = Sys.time();
		var kind = provenance == null ? null : kindBySource.get(provenance);
		var now = kind == null ? MoveKind.Rapid : kind;
		var nowProvenance = provenance == null ? new Provenance(0, 0, 0) : provenance;
		// The tool moved on the last tick under the move commanded before it, so the segment it
		// just cut belongs to that move; the move under way now labels the next segment. A tick
		// that runs from a rapid into a cutting move cut under the cutting move: the rapid's part
		// of it is at most one tick, and a rapid that does enter the stock cuts on the ticks after.
		if ((commandedKind == MoveKind.Rapid || commandedKind == MoveKind.Link || commandedKind == MoveKind.Retract) &&
				now != MoveKind.Rapid && now != MoveKind.Link && now != MoveKind.Retract)
			cutting.follow(now, at.op, nowProvenance);
		else
			cutting.follow(commandedKind, commandedOp, commandedProvenance);
		commandedKind = now;
		commandedOp = at.op;
		commandedProvenance = nowProvenance;
		cuttingSeconds += Sys.time() - clock;
		if (interfaceManaged && passFinished) finishActiveStock();
	}

	function updateRobotInterface(passFinished:Bool):Void {
		var logic = robotInterface;
		var ids = interfaceSensorIds;
		var door = doorCoils;
		var vise = viseCoils;
		if (logic == null || ids == null || door == null || vise == null) return;
		var sensors = robot.robot.snapshot().sensors;
		function readSensor(id:String):Bool {
			for (index in 0...sensors.length) {
				var frame = sensors.get(index);
				if (frame.sensorId == id) return frame.values.length > 0 && frame.values.get(0) >= 0.5;
			}
			return false;
		}
		robotInZone = readSensor(ids.robotZone);
		var input = new RobotInterfaceInput(
			cellDigital(RobotInterfaceChannels.REQUEST_DOOR_OPEN),
			cellDigital(RobotInterfaceChannels.REQUEST_DOOR_CLOSE),
			cellDigital(RobotInterfaceChannels.REQUEST_CLAMP),
			cellDigital(RobotInterfaceChannels.REQUEST_UNCLAMP),
			cellDigital(RobotInterfaceChannels.CYCLE_START),
			// The robot is clear when the machine's own zone scanner sees no robot body in its zone.
			!robotInZone,
			cellDigital(RobotInterfaceChannels.FAULT_RESET),
			readSensor(ids.doorOpen), readSensor(ids.doorClosed),
			readSensor(ids.viseClamped), readSensor(ids.viseOpen), readSensor(ids.presence),
			spindleStopped(), axesAtLoad(), passFinished, pendingToolChange);
		var outputs = logic.update(input, session.fixedTimestep());
		interfaceAlarm = outputs.alarm;
		if (outputs.startCycle) startRequested = true;
		if (outputs.holdCycle && motion.running) motion.hold();
		writeDigital(RobotInterfaceChannels.CNC_READY, outputs.ready);
		writeDigital(RobotInterfaceChannels.DOOR_OPEN, outputs.doorOpen);
		writeDigital(RobotInterfaceChannels.DOOR_CLOSED, outputs.doorClosed);
		writeDigital(RobotInterfaceChannels.CLAMPED, outputs.clamped);
		writeDigital(RobotInterfaceChannels.UNCLAMPED, outputs.unclamped);
		writeDigital(RobotInterfaceChannels.PART_PRESENT, outputs.partPresent);
		writeDigital(RobotInterfaceChannels.IN_CYCLE, outputs.inCycle);
		writeDigital(RobotInterfaceChannels.CYCLE_COMPLETE, outputs.cycleComplete);
		writeDigital(RobotInterfaceChannels.ALARM, outputs.alarm != RobotInterfaceAlarm.None);
		var alarmCode:Int = outputs.alarm;
		var previousAlarmCode:Int = lastInterfaceOutputs == null ? -1 : lastInterfaceOutputs.alarm;
		for (index in 0...INTERFACE_ALARM_CODE_BITS.length) {
			var mask = 1 << index;
			if (previousAlarmCode < 0 || (previousAlarmCode & mask) != (alarmCode & mask))
				robot.runtime.writeDigitalOutput(INTERFACE_ALARM_CODE_BITS[index], (alarmCode & mask) != 0);
		}
		var alarmChannel = interfaceAlarmCodeChannel;
		if (alarmChannel != null)
			robot.runtime.writeOutput(alarmChannel, ProcessEventValue.Analog(outputs.alarm));
		robot.runtime.writeDigitalOutput(door.a, outputs.doorCoilA);
		robot.runtime.writeDigitalOutput(door.b, outputs.doorCoilB);
		robot.runtime.writeDigitalOutput(vise.a, outputs.viseCoilA);
		robot.runtime.writeDigitalOutput(vise.b, outputs.viseCoilB);
		lastInterfaceOutputs = outputs;
	}

	function writeDigital(channel:String, value:Bool):Void {
		var prior = lastInterfaceOutputs;
		var unchanged = if (prior == null) false else switch channel {
			case RobotInterfaceChannels.CNC_READY: prior.ready == value;
			case RobotInterfaceChannels.DOOR_OPEN: prior.doorOpen == value;
			case RobotInterfaceChannels.DOOR_CLOSED: prior.doorClosed == value;
			case RobotInterfaceChannels.CLAMPED: prior.clamped == value;
			case RobotInterfaceChannels.UNCLAMPED: prior.unclamped == value;
			case RobotInterfaceChannels.PART_PRESENT: prior.partPresent == value;
			case RobotInterfaceChannels.IN_CYCLE: prior.inCycle == value;
			case RobotInterfaceChannels.CYCLE_COMPLETE: prior.cycleComplete == value;
			case RobotInterfaceChannels.ALARM: (prior.alarm != RobotInterfaceAlarm.None) == value;
			default: false;
		};
		if (!unchanged) robot.runtime.writeDigitalOutput(channel, value);
	}

	function cellDigital(channel:String):Bool {
		var source = cellInput;
		if (source == null) return false;
		return switch source(channel) {
			case Digital(value): value;
			case _: false;
		};
	}

	/** Selects the unique free blank inside the same oriented volume sampled by the vise sensor. */
	function stockAtPresenceInput():String {
		var ids = interfaceSensorIds;
		if (ids == null) throw "Stock selection needs the machine's part-presence sensor";
		var sensor:Null<robotkit.runtime.RobotRuntimeSensorBlueprint> = null;
		for (candidate in robot.blueprint.sensors) if (candidate.id == ids.presence) sensor = candidate;
		if (sensor == null || sensor.kind != "presence_input" || sensor.link < 0)
			throw 'CNC part-presence input "${ids.presence}" has no live sensor mount';
		var range = sensor.maxRange;
		if (!Math.isFinite(range) || range <= 0.0)
			throw 'CNC part-presence input "${ids.presence}" has no positive sensing range';
		var mount = AssemblyRobot.compose(simulation.linkPose(machineRobotIndex, sensor.link), {
			x: sensor.position.get(0), y: sensor.position.get(1), z: sensor.position.get(2),
			qx: sensor.rotation.get(0), qy: sensor.rotation.get(1),
			qz: sensor.rotation.get(2), qw: sensor.rotation.get(3)});
		var direction = AssemblyRobot.rotate(mount.rotation, [0.0, 0.0, 1.0]);
		var center = [for (axis in 0...3) mount.position[axis] + direction[axis] * range * 0.5];
		var volume = new SimPose(center[0], center[1], center[2], mount.rotation[0], mount.rotation[1],
			mount.rotation[2], mount.rotation[3]);
		var halfExtents = [range, range, range * 0.5];
		var selected:Null<String> = null;
		for (occurrence in job.stocks) {
			var candidate = stockSources.get(occurrence);
			if (candidate == null) throw 'CNC stock "$occurrence" has no simulated body';
			if (candidate.part.overlapsVolume(volume, halfExtents)) {
				if (selected != null)
					throw 'More than one CNC blank overlaps the vise part-presence volume: "$selected" and "$occurrence"';
				selected = occurrence;
			}
		}
		if (selected == null) throw 'No CNC blank overlaps the vise part-presence volume "${ids.presence}"';
		return selected;
	}

	function activateStock(occurrence:String):Void {
		if (activeStock != null) throw 'CNC stock "${activeStockOccurrence}" is still active at the next cycle start';
		var selected = stockSources.get(occurrence);
		if (selected == null) throw 'CNC job selected undeclared stock "$occurrence"';
		var source = selected.source;
		var cut = new MachiningStock(source.minimum, source.maximum, source.center, STOCK_SPACING,
			selected.part, spindlePart, job.target, source.mesh);
		if (loadedTool >= 0) cut.load(toolsByNumber.get(loadedTool));
		stock = cut;
		activeStock = cut;
		activeStockOccurrence = occurrence;
		stockObject = "project:" + occurrence;
		stockShownAt = Math.NEGATIVE_INFINITY;
		activeRawVolume = cut.stock.volume();
		allStocks.push(cut);
		commandedKind = MoveKind.Rapid;
		commandedOp = -1;
		commandedProvenance = new Provenance(0, 0, 0);
	}

	/** Persists the finished contour and the matching inertial mass on the blank occurrence. */
	function finishActiveStock():Void {
		var cut = activeStock, occurrence = activeStockOccurrence;
		if (cut == null || occurrence == null) return;
		var selected = stockSources.get(occurrence);
		if (selected == null) throw 'Finished CNC stock "$occurrence" lost its physical definition';
		var geometry = cut.geometry();
		project.scene.setRuntimeGeometry("project:" + occurrence, geometry);
		var finishedMass = cut.stock.volume() * selected.source.density;
		selected.part.setMass(finishedMass);
		var targetMass = cut.target == null ? 0.0 : cut.target.volume() * selected.source.density;
		var massError = targetMass <= 0.0 ? 0.0 : Math.abs(finishedMass - targetMass) / targetMass;
		var deviation = cut.target == null ? {leftover: 0.0, gouge: 0.0} : cut.deviation();
		completedStocks.push({occurrence: occurrence, removedVolume: cut.removed,
			rawVolume: activeRawVolume, finishedMass: finishedMass, targetMass: targetMass,
			massError: massError, leftover: deviation.leftover, gouge: deviation.gouge,
			rapidContacts: cut.rapidContacts, collisions: cut.collisions});
		activeStock = null;
		activeStockOccurrence = null;
		activeRawVolume = 0.0;
	}

	function spindleStopped():Bool {
		var stoppedCommand = switch robot.runtime.channelValue(channels.spindleSpeed) {
			case Analog(value): Math.abs(value) <= SPINDLE_STANDSTILL_RPM;
			case _: false;
		};
		if (!stoppedCommand || spindleVelocityJoint < 0) return false;
		var velocities = robot.robot.snapshot().velocities;
		if (spindleVelocityJoint >= velocities.length) return false;
		return Math.abs(velocities.get(spindleVelocityJoint)) <= SPINDLE_STANDSTILL_RPM * (2.0 * Math.PI / 60.0);
	}

	function axesAtLoad():Bool {
		if (loadPosition.length != 3) return false;
		var positions = robot.robot.snapshot().positions;
		var machine = solver.forward([for (index in axisJoints) positions.get(index)]);
		return Math.abs(machine.x - loadPosition[0]) <= 0.001 &&
			Math.abs(machine.y - loadPosition[1]) <= 0.001 && Math.abs(machine.z - loadPosition[2]) <= 0.001;
	}

	function hasSensor(id:String):Bool {
		for (candidate in robot.blueprint.sensors) if (candidate.id == id) return true;
		return false;
	}

	function pneumaticDriveFor(robot:AssemblyRobot,
			jointId:String):robotkit.runtime.RobotRuntimeCylinderDriveBlueprint {
		for (drive in robot.blueprint.cylinderDrives)
			if (drive.control == robotkit.runtime.CylinderControl.PressureEffort && drive.joint >= 0 && drive.joint < robot.model.joints.length &&
				robot.model.joints[drive.joint].id == jointId) return drive;
		throw 'Robot-tended CNC machine is missing its pneumatic actuator on joint "$jointId"';
	}

	function finalMachinePosition(program:ToolpathProgram):Array<Float> {
		var frame = ToolpathFrame.of(program), result:Array<Float> = null;
		var lastWasMachineMove = false;
		for (op in program.ops) {
			frame.advance(op);
			switch op {
				case Move(_, geometry, _, _, _):
					var point = GeometryTools.pointAt(geometry, GeometryTools.length(geometry));
					var offset = frame.toMachine();
					result = [point.x + offset[0], point.y + offset[1], point.z + offset[2]];
					lastWasMachineMove = false;
				case MachineMove(_, geometry, _, _, _):
					var point = GeometryTools.pointAt(geometry, GeometryTools.length(geometry));
					result = [point.x, point.y, point.z];
					lastWasMachineMove = true;
				case _:
			}
		}
		if (result == null) throw "CNC program has no final machine position";
		if (interfaceManaged && !lastWasMachineMove)
			throw "Robot-tended CNC program must end with a G53 move to its load position";
		return result;
	}

	static function machineMillimetres(metres:Float):String
		return Std.string(Math.round(metres * 1000.0 * 1e6) / 1e6);

	override public function canRestart():Bool return true;

	override public function title():String return "CNC PROGRAM";

	override public function statusDetail():String
		return ' · tool $loadedTool' +
			(spindleUtilization > 0.0 ? ' · spindle ${Math.round(spindleUtilization * 1000) / 10}%' : "");

	/**
	 * Restarts the program at G-code line `line`, or the first line after it that moves: the machine
	 * stops, climbs clear of the work, loads that line's tool, starts the spindle as the program had it
	 * and carries on from there. Returns false when no later line moves.
	 */
	override public function restartFromLine(line:Int):Bool {
		var best:Null<toolpathkit.motion.ToolpathSourceMap.ToolpathSourceMapEntry> = null;
		for (entry in sourceMap.entries) {
			var at = entry.provenance.line;
			if (at < line || entry.endDistance <= entry.startDistance) continue;
			if (best == null || at < best.provenance.line ||
					(at == best.provenance.line && entry.opIndex < best.opIndex))
				best = entry;
		}
		if (best == null) return false;
		restartRequest = {op: best.opIndex, distance: best.startDistance};
		clearFailure();
		return true;
	}

	/** Moves an operator's restart along: hold, then plan from where the machine stopped, then run once it has. */
	function takeRestart():Void {
		var request = restartRequest;
		if (request != null) {
			var run = new MachiningRun(recipe, program, motion, channels);
			if (motion.running && motion.sessionState() != SessionState.Held) motion.hold();
			else if (motion.running) {
				pendingContinuation = run.prepareRestart(request.op, request.distance);
				restartRequest = null;
			} else {
				pendingContinuation = run.continuationFrom(request.op, request.distance);
				restartRequest = null;
			}
		}
		var next = pendingContinuation;
		if (next == null || motion.running || motion.sessionState() != SessionState.Idle) return;
		var snapshot = robot.robot.snapshot();
		if (snapshot.trajectoryActive || snapshot.trajectoryQueueDepth > 0 || snapshot.safety != 0) return;
		new MachiningRun(recipe, program, motion, channels).resumePrepared(next);
		continuation = next;
		pendingContinuation = null;
		started = true;
		passCounted = false;
	}

	/** The session is back at its start, and the robot with it: run the program again on fresh stock. */
	override function resetJob():Void {
		restartRequest = null;
		pendingContinuation = null;
		continuation = null;
		loadedTool = initialTool;
		startRequested = false;
		pendingToolChange = false;
		lastInterfaceOutputs = null;
		parkingStarted = false;
		parkingComplete = !interfaceManaged;
		if (interfaceManaged) {
			robotInterface = new RobotInterface();
			interfaceAlarm = RobotInterfaceAlarm.None;
		}
		spindleUtilization = 0.0;
		spindleIntervalSeconds = 0.0;
		spindleTurningSeconds = 0.0;
		for (cut in allStocks) cut.dispose();
		allStocks.splice(0, allStocks.length);
		for (occurrence in job.stocks) project.scene.clearRuntimeGeometry("project:" + occurrence);
		stock = null;
		activeStock = null;
		activeStockOccurrence = null;
		activeRawVolume = 0.0;
		stockObject = null;
		stockShownAt = Math.NEGATIVE_INFINITY;
		completedStocks.splice(0, completedStocks.length);
	}

	/**
	 * Shows the tool in the spindle, and the stock as cut so far: a few times a second the stock is
	 * re-contoured on its own thread, where it changed, and the chunks it rebuilt are shown when ready.
	 */
	override public function present():Void {
		var shape = toolShape, toolId = toolObject;
		if (shape != null && toolId != null && shownTool != loadedTool) {
			var tool = toolsByNumber.get(loadedTool);
			if (tool != null) project.scene.setRuntimeGeometry(toolId, shape(tool));
			shownTool = loadedTool;
		}
		var cut = stock, id = stockObject;
		if (cut == null || id == null) return;
		var clock = Sys.time();
		var chunks = cut.takePreview();
		if (chunks != null) project.scene.setRuntimeGeometryParts(id, cut.previewChunks(), chunks);
		var now = session.simulationTime();
		if (cut.hasChanged() && (now - stockShownAt >= STOCK_REFRESH || now < stockShownAt) && cut.refreshPreview())
			stockShownAt = now;
		meshingSeconds += Sys.time() - clock;
	}

	/** Stops cutting and gives the stock and tool parts their own geometry back. */
	public function dispose(?releases:Array<Void->Void>):Void {
		for (cut in allStocks) {
			if (releases == null) cut.dispose(); else releases.push(function() cut.dispose());
		}
		allStocks.splice(0, allStocks.length);
		stock = null;
		activeStock = null;
		activeStockOccurrence = null;
		for (occurrence in job.stocks) project.scene.clearRuntimeGeometry("project:" + occurrence);
		if (toolObject != null) project.scene.clearRuntimeGeometry(toolObject);
		shownTool = -1;
	}

	/**
	 * A serial chain of just the machine's axis joints, with their types, axes and limits, for the
	 * planner, and each joint's index in the full model, where plans are executed. Joints coupled
	 * to an axis, such as its lead screws, are left out: the runtime turns them with their axis,
	 * and their limits are folded into the axis's.
	 */
	static function planningModel(model:RobotModel, axes:Array<String>, steady:SteadyLoads):{model:RobotModel, indices:Array<Int>} {
		var planning = new RobotModel(model.name + ".axes");
		var parent = planning.addLink(new Link("machine.base"));
		var indices:Array<Int> = [];
		var limits = PlanningLimits.of(model, axes, steady);
		for (slot in 0...axes.length) {
			var id = axes[slot];
			var index = -1;
			for (candidate in 0...model.joints.length) if (Std.string(model.joints[candidate].id) == id) index = candidate;
			if (index < 0) throw 'CNC axis "$id" is not a joint of the machine';
			var source = model.joints[index];
			var child = planning.addLink(new Link(id + ".carriage"));
			var joint = planning.addJoint(new Joint(id, source.type, parent, child, source.id));
			joint.axis = source.axis.copy();
			joint.limits = limits.bounds[slot].copy();
			indices.push(index);
			parent = child;
		}
		return {model: planning, indices: indices};
	}
}
