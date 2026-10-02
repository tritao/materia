package app;

import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import nativekit.scene.GeometryData;
import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.CncDiagnostic.CncSeverity;
import robotkit.model.Joint;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.runtime.Simulation;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;
import toolpathkit.motion.ToolpathSourceMap;
import toolpathkit.path.MoveKind;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.tool.Tool;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.event.EventValue;
import motionkit.program.MotionProgram;
import motionkit.robot.AxisKinematics;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MotionSystemBlueprint;
import nativekit.sim.SimSession;
import toolpathkit.motion.MachineBinding;
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
typedef CncJob = {
	var source:String;
	var axes:Array<String>;
	var spindle:String;
	/** G54 in machine coordinates. */
	var workOffset:Array<Float>;
	/** The tool table; G43 H numbers are tool numbers. */
	var tools:Array<Tool>;
	var loop:Bool;
	/** The part the job machines, cut as the tool moves; null for a job that cuts nothing. */
	@:optional var stock:String;
	/** The stock part's mesh in its own frame, the raw stock. */
	@:optional var stockMesh:{positions:Array<Float>, indices:Array<Int>};
	/** The finished part, a closed mesh in the stock part's frame: the cut stock is compared with it. */
	@:optional var target:{positions:Array<Float>, indices:Array<Int>};
	/** The part showing the tool in the spindle; each loaded tool's shape replaces its geometry. */
	@:optional var toolPart:String;
	/** The tool in the spindle when the job starts. */
	@:optional var loadedTool:Int;
}

/**
 * Runs a project's CNC job on its simulated assembly robot: the G-code is compiled against the
 * machine's axes and streamed to the robot runtime as trajectory segments by MotionKit, before each
 * simulation tick. A looping job starts again once the program completes.
 */
class CncProgramPlayer implements SessionMember {
	/** Stock geometry goes to the scene at most this often, in simulated seconds. */
	static inline final STOCK_REFRESH = 0.05;
	/** Ray spacing of the simulated stock, in metres. */
	static inline final STOCK_SPACING = 0.0005;

	/** Spindle and coolant channels a CNC program's events need on the machine's runtime. */
	public static function processChannels():Array<ProcessChannelDeclaration>
		return [
			new ProcessChannelDeclaration("spindle.speed", ProcessEventValue.Analog(0.0)),
			new ProcessChannelDeclaration("spindle.direction", ProcessEventValue.Analog(0.0)),
			new ProcessChannelDeclaration("coolant.mist", ProcessEventValue.Digital(false)),
			new ProcessChannelDeclaration("coolant.flood", ProcessEventValue.Digital(false))
		];

	final session:SimSession;
	final project:ProjectDocumentSession;
	final compileFrom:Point3->ToolpathProgram;
	final robot:AssemblyRobot;
	/** Machine coordinates from the axis joints' positions, and those joints' indices on the robot. */
	final solver:AxisKinematics;
	final axisJoints:Array<Int>;
	var program:MotionProgram;
	var sourceMap:ToolpathSourceMap;
	final kindByLine = new Map<Int, MoveKind>();
	final newStock:Null<Void->MachiningStock>;
	final stockObject:Null<String>;
	static inline final TOOL_CHANGE = "cnc.tool_change.";
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
	public var motionSeconds(default, null):Float = 0.0;
	public var runSeconds(default, null):Float = 0.0;
	public var slowestUpdate(default, null):Float = 0.0;
	var commandedKind:MoveKind = MoveKind.Rapid;
	var commandedOp = -1;
	var commandedProvenance = new Provenance(0, 0, 0);
	final newMotion:Void->ManipulatorMotion;
	var motion:ManipulatorMotion;
	final loop:Bool;
	var started = false;
	/** Passes of the program completed since the session started. */
	public var passes(default, null):Int = 0;
	var passCounted = false;
	/** Why the program stopped, when it failed. */
	public var failure(default, null):Null<String> = null;
	/** The G-code line the machine is executing; 0 between lines. */
	public var currentLine(default, null):Int = 0;
	/** Speed of every move, as a fraction of the program's. */
	public var speedOverride(default, null):Float = 1.0;
	final source:String;
	final recipe:MachiningRecipe;
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
			project:ProjectDocumentSession, session:SimSession) {
		if (job.axes.length != 3) throw "A CNC job needs its X, Y and Z axes";
		var definition = project.projectAssemblyDefinition, physical = project.projectPhysical;
		if (definition == null || physical == null) throw "A CNC job needs the project's machine";
		var metresPerUnit = physical.metresPerUnit, state = project.projectAssemblyState;
		this.session = session;
		this.project = project;
		this.loop = job.loop;
		this.source = job.source;
		var placement = new AssemblyState(definition, state);
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
			var coupled = robot.model.coupledLimits(id);
			if (!(coupled.velocity > 0) || !(coupled.maxAcceleration > 0))
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
		var planning = planningModel(robot.model, job.axes);
		var machine = new MachineBinding("machine", job.axes[0], job.axes[1], job.axes[2], rapid);
		var binding = new ToolpathMotionBinding(machine,
			new MotionSystemBlueprint(planning.model, robot.blueprint, axes, session.fixedTimestep()));
		var controller = new CncController();
		for (tool in job.tools) {
			controller.toolLibrary.set(tool);
			toolsByNumber.set(tool.number, tool);
		}
		controller.setWorkOffset(54, job.workOffset[0], job.workOffset[1], job.workOffset[2]);
		// A program runs from wherever the machine is, so each pass compiles it from there.
		compileFrom = position -> {
			var compiled = CncCompiler.compileDetailed(job.source, controller, position, machine.travel);
			var errors = [for (diagnostic in compiled.diagnostics) if (diagnostic.severity == CncSeverity.Error) diagnostic.toString()];
			if (errors.length > 0) throw 'The machining program: ${errors.join("; ")}';
			var lowered = ToolpathMotion.lower(compiled.program, machine);
			if (lowered.program == null)
				throw 'The machining program: ${[for (diagnostic in lowered.diagnostics) diagnostic.message].join("; ")}';
			program = lowered.program;
			sourceMap = lowered.sourceMap;
			return compiled.program;
		};
		this.robot = robot;
		solver = binding.solver;
		axisJoints = planning.indices;
		// Compiling from the starting pose now reports a bad program before the simulation starts.
		var compiled = compileFrom(new Point3(start[0], start[1], start[2]));
		for (op in compiled.ops) switch op {
			case Move(kind, _, _, _, provenance), MachineMove(kind, _, _, _, provenance): kindByLine.set(provenance.line, kind);
			case _:
		}
		initialTool = job.loadedTool == null ? -1 : job.loadedTool;
		loadedTool = initialTool;
		var spindleLink = robot.part("project:" + job.spindle);
		var stockPart = job.stock;
		if (stockPart == null) {
			newStock = null;
			stockObject = null;
		} else {
			var stockLink = robot.part("project:" + stockPart);
			var occurrence = [for (item in definition.occurrences) if (item.id == stockPart) item];
			var part = [for (item in physical.parts) if (occurrence.length == 1 && item.id == occurrence[0].definition) item];
			var center = occurrence.length == 1 ? project.assemblyPreviewCenter(occurrence[0].definition) : null;
			var hull = part.length == 1 ? part[0].collisionHull : null;
			if (hull == null || center == null) throw 'CNC stock part "$stockPart" has no shape to cut';
			var minimum = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
			var maximum = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
			for (index in 0...hull.length) {
				minimum[index % 3] = Math.min(minimum[index % 3], hull[index] * metresPerUnit);
				maximum[index % 3] = Math.max(maximum[index % 3], hull[index] * metresPerUnit);
			}
			var centerMetres = [for (coordinate in center) coordinate * metresPerUnit];
			newStock = () -> {
				var stock = new MachiningStock(minimum, maximum, centerMetres, STOCK_SPACING, simulation, stockLink, spindleLink,
					job.target, job.stockMesh);
				stock.load(toolsByNumber.get(loadedTool));
				return stock;
			};
			stockObject = "project:" + stockPart;
		}
		toolObject = job.toolPart == null ? null : "project:" + job.toolPart;
		toolShape = job.toolPart == null ? null : toolShapeIn(job.toolPart, job.spindle, placement, project, metresPerUnit);
		// Spindle-speed handshakes are always ready. A tool change is the operator loading that tool,
		// which the stock then cuts with and the spindle shows.
		newMotion = () -> new ManipulatorMotion(robot.robot, binding.compiler,
			channel -> {
				if (channel == "spindle.at_speed") return EventValue.Digital(true);
				if (!StringTools.startsWith(channel, TOOL_CHANGE)) return null;
				var number = Std.parseInt(channel.substr(TOOL_CHANGE.length));
				if (number == null || !toolsByNumber.exists(number)) throw 'The machining program loads unknown tool $channel';
				if (number != loadedTool) {
					loadedTool = number;
					if (stock != null) stock.load(toolsByNumber.get(number));
				}
				return EventValue.Digital(true);
			},
			() -> robot.runtime.pollEvents(), planning.indices);
		motion = newMotion();
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

	public function feed():Void {
		if (failure != null) return;
		var clock = Sys.time();
		takeRestart();
		if (started && motion.completed && !passCounted) {
			passes++;
			passCounted = true;
		}
		if (!started || (loop && motion.completed && !motion.running && pendingContinuation == null)) {
			continuation = null;
			if (started) {
				// A program runs from wherever the machine is: the position the planner last
				// commanded, or where the robot stands when there is none.
				var commanded = motion.commandedPositions();
				var joints = commanded != null ? commanded : {
					var actual = robot.robot.snapshot().positions;
					[for (index in axisJoints) actual.get(index)];
				};
				var machine = solver.forward(joints);
				compileFrom(new Point3(machine.x, machine.y, machine.z));
			}
			motion.run(program);
			started = true;
			passCounted = false;
			runSeconds += Sys.time() - clock;
			clock = Sys.time();
		}
		motion.update(session.fixedTimestep());
		var spent = Sys.time() - clock;
		motionSeconds += spent;
		slowestUpdate = Math.max(slowestUpdate, spent);
		// A restart aborts the program on purpose; that is not a failure.
		if (motion.failure != null && pendingContinuation == null && restartRequest == null) failure = motion.failure;
		var progress = motion.progress();
		var at = continuation == null ? {op: progress.op, distance: progress.pathDistance} :
			continuation.originalAt(progress.op, progress.pathDistance);
		var provenance = at.op < 0 ? null : sourceMap.provenanceAt(at.op, at.distance);
		currentLine = provenance == null ? 0 : provenance.line;
		if (newStock != null) {
			if (stock == null) stock = newStock();
			clock = Sys.time();
			var kind = provenance == null ? null : kindByLine.get(provenance.line);
			var now = kind == null ? MoveKind.Rapid : kind;
			var nowProvenance = provenance == null ? new Provenance(0, 0, 0) : provenance;
			// The tool moved on the last tick under the move commanded before it, so the segment it
			// just cut belongs to that move; the move under way now labels the next segment. A tick
			// that runs from a rapid into a cutting move cut under the cutting move: the rapid's part
			// of it is at most one tick, and a rapid that does enter the stock cuts on the ticks after.
			if (commandedKind == MoveKind.Rapid && now != MoveKind.Rapid)
				stock.follow(now, at.op, nowProvenance);
			else
				stock.follow(commandedKind, commandedOp, commandedProvenance);
			commandedKind = now;
			commandedOp = at.op;
			commandedProvenance = nowProvenance;
			cuttingSeconds += Sys.time() - clock;
		}
	}

	/** The program's G-code, one entry per line. */
	public function sourceLines():Array<String> return source.split("\n");

	/** Whether the machine is stopped by a feed hold. */
	public function held():Bool return motion.sessionState() == SessionState.Held;

	/** Brings the machine to a controlled stop on its path. */
	public function hold():Void motion.hold();

	public function resume():Void motion.resume();

	/** Sets the speed of every move, from 5% to 200% of the program's; it takes effect from motion not yet planned, about a second ahead. */
	public function setSpeedOverride(scale:Float):Void {
		motion.setSpeedOverride(scale);
		speedOverride = scale;
	}

	/**
	 * Restarts the program at G-code line `line`, or the first line after it that moves: the machine
	 * stops, climbs clear of the work, loads that line's tool, starts the spindle as the program had it
	 * and carries on from there. Returns false when no later line moves.
	 */
	public function restartFromLine(line:Int):Bool {
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
		failure = null;
		return true;
	}

	/** Moves an operator's restart along: hold, then plan from where the machine stopped, then run once it has. */
	function takeRestart():Void {
		var request = restartRequest;
		if (request != null) {
			var run = new MachiningRun(recipe, program, motion);
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
		new MachiningRun(recipe, program, motion).resumePrepared(next);
		continuation = next;
		pendingContinuation = null;
		started = true;
		passCounted = false;
	}

	/** The session is back at its start, and the robot with it: run the program again on fresh stock. */
	public function beforeReset():Void {}

	public function reset():Void {
		motion = newMotion();
		if (speedOverride != 1.0) motion.setSpeedOverride(speedOverride);
		restartRequest = null;
		pendingContinuation = null;
		continuation = null;
		currentLine = 0;
		loadedTool = initialTool;
		started = false;
		failure = null;
		passes = 0;
		passCounted = false;
		if (stock != null) {
			stock.dispose();
			stock = null;
			stockShownAt = Math.NEGATIVE_INFINITY;
		}
		// The fresh stock shows as the raw part until its first contour arrives.
		if (stockObject != null) project.scene.clearRuntimeGeometry(stockObject);
	}

	/**
	 * Shows the tool in the spindle, and the stock as cut so far: a few times a second the stock is
	 * re-contoured on its own thread, where it changed, and the chunks it rebuilt are shown when ready.
	 */
	public function present():Void {
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
	public function dispose():Void {
		if (stock != null) stock.dispose();
		stock = null;
		for (id in [stockObject, toolObject]) if (id != null) project.scene.clearRuntimeGeometry(id);
		shownTool = -1;
	}

	/**
	 * A serial chain of just the machine's axis joints, with their types, axes and limits, for the
	 * planner, and each joint's index in the full model, where plans are executed. Joints coupled
	 * to an axis, such as its lead screws, are left out: the runtime turns them with their axis,
	 * and their limits are folded into the axis's.
	 */
	static function planningModel(model:RobotModel, axes:Array<String>):{model:RobotModel, indices:Array<Int>} {
		var planning = new RobotModel(model.name + ".axes");
		var parent = planning.addLink(new Link("machine.base"));
		var indices:Array<Int> = [];
		for (id in axes) {
			var index = -1;
			for (candidate in 0...model.joints.length) if (Std.string(model.joints[candidate].id) == id) index = candidate;
			if (index < 0) throw 'CNC axis "$id" is not a joint of the machine';
			var source = model.joints[index];
			var child = planning.addLink(new Link(id + ".carriage"));
			var joint = planning.addJoint(new Joint(id, source.type, parent, child, source.id));
			joint.axis = source.axis.copy();
			joint.limits = model.coupledLimits(source.id);
			indices.push(index);
			parent = child;
		}
		return {model: planning, indices: indices};
	}
}
