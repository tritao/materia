package app;

import cadkit.modeling.AssemblyState;
import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.CncDiagnostic.CncSeverity;
import robotkit.model.Joint;
import robotkit.model.JointLimits;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.runtime.Simulation;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;
import toolpathkit.motion.ToolpathSourceMap;
import toolpathkit.path.MoveKind;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.Tool;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.event.EventValue;
import motionkit.program.MotionProgram;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MotionSystemBlueprint;
import nativekit.sim.SimSession;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.path.Point3;

/**
 * A project's machining job: a G-code program run on the project's own machine. Joint ids name the
 * machine's X, Y and Z axes, whose assembly coordinates are machine coordinates; the work offset is
 * G54 in the assembly's length unit.
 */
typedef CncJob = {
	var programPath:String;
	var source:String;
	var axes:Array<String>;
	var workOffset:Array<Float>;
	var loop:Bool;
	/** The part the job machines, cut as the tool moves; null for a job that cuts nothing. */
	@:optional var stockPart:String;
	/** The part whose origin is the tool tip and whose +Z is the tool axis. */
	@:optional var toolPart:String;
	/** The controller's tool table; the first entry is the tool in the spindle. */
	var tools:Array<CncJobTool>;
	/** Ray spacing of the simulated stock, in the assembly's length unit. */
	var stockSpacing:Float;
}

/** One tool of a CNC job's tool table, in the assembly's length unit. */
typedef CncJobTool = {
	var number:Int;
	var diameter:Float;
	var fluteLength:Float;
	/** Length out of the holder. */
	var length:Float;
	var holderDiameter:Float;
	var holderLength:Float;
}

/**
 * Runs a project's CNC job on its simulated assembly robot: the G-code is compiled against the
 * machine's axes and streamed to the robot runtime as trajectory segments by MotionKit, before each
 * simulation tick. A looping job starts again once the program completes.
 */
class CncProgramPlayer implements SessionMember {
	/** Stock geometry goes to the scene at most this often, in simulated seconds. */
	static inline final STOCK_REFRESH = 0.05;

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
	final sourceMap:ToolpathSourceMap;
	final kindByLine = new Map<Int, MoveKind>();
	final newStock:Null<Void->MachiningStock>;
	final stockObject:Null<String>;
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
	final program:MotionProgram;
	final newMotion:Void->ManipulatorMotion;
	var motion:ManipulatorMotion;
	final loop:Bool;
	var started = false;
	/** Why the program stopped, when it failed. */
	public var failure(default, null):Null<String> = null;

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
			var lower = limits.lower, upper = limits.upper, velocity = limits.velocity, acceleration = limits.acceleration;
			if (lower == null || upper == null || velocity == null || acceleration == null)
				throw 'CNC axis "$id" needs travel, velocity and acceleration limits';
			var initial = placement.joint(id) * metresPerUnit;
			start.push(initial);
			rapid = Math.max(rapid, velocity * metresPerUnit);
			axes.push(new MotionAxisBlueprint(id, [id], lower * metresPerUnit, upper * metresPerUnit,
				velocity * metresPerUnit, acceleration * metresPerUnit, initial, [1.0], [-initial]));
		}
		// Plan over the three axes alone: the machine's other joints are fixed mounts, and planning
		// them all made compiling a short program take many seconds.
		var planning = planningModel(robot.model, job.axes);
		var machine = new MachineBinding("machine", job.axes[0], job.axes[1], job.axes[2], rapid);
		var binding = new ToolpathMotionBinding(machine,
			new MotionSystemBlueprint(planning.model, robot.blueprint, axes, session.fixedTimestep()));
		var controller = new CncController();
		controller.setWorkOffset(54, job.workOffset[0] * metresPerUnit, job.workOffset[1] * metresPerUnit,
			job.workOffset[2] * metresPerUnit);
		var compiled = CncCompiler.compileDetailed(job.source, controller, new Point3(start[0], start[1], start[2]),
			machine.travel);
		var errors = [for (diagnostic in compiled.diagnostics) if (diagnostic.severity == CncSeverity.Error) diagnostic.toString()];
		if (errors.length > 0) throw '${job.programPath}: ${errors.join("; ")}';
		var lowered = ToolpathMotion.lower(compiled.program, machine);
		if (lowered.program == null)
			throw '${job.programPath}: ${[for (diagnostic in lowered.diagnostics) diagnostic.message].join("; ")}';
		program = lowered.program;
		sourceMap = lowered.sourceMap;
		for (op in compiled.program.ops) switch op {
			case Move(kind, _, _, _, provenance), MachineMove(kind, _, _, _, provenance): kindByLine.set(provenance.line, kind);
			case _:
		}
		var stockPart = job.stockPart;
		if (stockPart == null) {
			newStock = null;
			stockObject = null;
		} else {
			var toolPart = job.toolPart;
			if (toolPart == null || job.tools.length == 0) throw "A CNC job that cuts stock needs its tool part and tool table";
			var stockLink = robot.part("project:" + stockPart), toolLink = robot.part("project:" + toolPart);
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
			var tool = cutter(job.tools[0], metresPerUnit), spacing = job.stockSpacing * metresPerUnit;
			// The stock is the stock part's bounding box, which is exact for a block of stock.
			newStock = () -> new MachiningStock(tool, minimum, maximum, centerMetres, spacing, simulation,
				stockLink, toolLink);
			stockObject = "project:" + stockPart;
		}
		// Spindle and tool-change handshakes are always ready: the simulation models neither.
		newMotion = () -> new ManipulatorMotion(robot.robot, binding.compiler,
			channel -> channel == "spindle.at_speed" || StringTools.startsWith(channel, "cnc.tool_change.") ?
				EventValue.Digital(true) : null,
			() -> robot.runtime.pollEvents(), planning.indices);
		motion = newMotion();
	}

	public function feed():Void {
		if (failure != null) return;
		var clock = Sys.time();
		if (!started || (loop && motion.completed && !motion.running)) {
			motion.run(program);
			started = true;
			runSeconds += Sys.time() - clock;
			clock = Sys.time();
		}
		motion.update(session.fixedTimestep());
		var spent = Sys.time() - clock;
		motionSeconds += spent;
		slowestUpdate = Math.max(slowestUpdate, spent);
		if (motion.failure != null) failure = motion.failure;
		if (newStock != null) {
			if (stock == null) stock = newStock();
			clock = Sys.time();
			// The tool moved on the last tick under the move commanded before it, so the segment it
			// just cut belongs to that move; the move under way now labels the next segment.
			stock.follow(commandedKind, commandedOp, commandedProvenance);
			var progress = motion.progress();
			var provenance = progress.op < 0 ? null : sourceMap.provenanceAt(progress.op, progress.pathDistance);
			var kind = provenance == null ? null : kindByLine.get(provenance.line);
			commandedKind = kind == null ? MoveKind.Rapid : kind;
			commandedOp = progress.op;
			commandedProvenance = provenance == null ? new Provenance(0, 0, 0) : provenance;
			cuttingSeconds += Sys.time() - clock;
		}
	}

	/** The session is back at its start, and the robot with it: run the program again on fresh stock. */
	public function reset():Void {
		motion = newMotion();
		started = false;
		failure = null;
		if (stock != null) {
			stock.dispose();
			stock = null;
			stockShownAt = Math.NEGATIVE_INFINITY;
		}
	}

	/** Shows the stock as cut so far, re-contouring only what changed, a few times a second. */
	public function present():Void {
		var cut = stock, id = stockObject;
		if (cut == null || id == null || !cut.hasChanged()) return;
		var now = session.simulationTime();
		if (now - stockShownAt < STOCK_REFRESH && now >= stockShownAt) return;
		stockShownAt = now;
		var clock = Sys.time();
		project.scene.setRuntimeGeometry(id, cut.geometry());
		meshingSeconds += Sys.time() - clock;
	}

	public function dispose():Void {
		if (stock != null) stock.dispose();
		stock = null;
	}

	/**
	 * A serial chain of just the machine's axis joints, with their types, axes and limits, for the
	 * planner, and each joint's index in the full model, where plans are executed.
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
			var limits = source.limits;
			joint.limits = new JointLimits(limits.lower, limits.upper, limits.velocity, limits.effort, limits.maxAcceleration);
			joint.limits.overtravel = limits.overtravel;
			indices.push(index);
			parent = child;
		}
		return {model: planning, indices: indices};
	}

	/** A flat end mill with its shank and the holder above it, in metres. */
	static function cutter(tool:CncJobTool, metresPerUnit:Float):Tool {
		var diameter = tool.diameter * metresPerUnit, flutes = tool.fluteLength * metresPerUnit;
		var length = tool.length * metresPerUnit;
		var profile = CutterProfile.flat(diameter, flutes);
		if (length > flutes) profile = profile.withShank(diameter, length - flutes);
		if (tool.holderLength > 0)
			profile = profile.withHolder(tool.holderDiameter * metresPerUnit, tool.holderLength * metresPerUnit);
		return Tool.shaped(tool.number, length, profile);
	}
}
