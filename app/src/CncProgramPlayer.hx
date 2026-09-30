package app;

import cadkit.modeling.AssemblyState;
import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.CncDiagnostic.CncSeverity;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
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
}

/**
 * Runs a project's CNC job on its simulated assembly robot: the G-code is compiled against the
 * machine's axes and streamed to the robot runtime as trajectory segments by MotionKit, before each
 * simulation tick. A looping job starts again once the program completes.
 */
class CncProgramPlayer implements SessionMember {
	final session:SimSession;
	final program:MotionProgram;
	final newMotion:Void->ManipulatorMotion;
	var motion:ManipulatorMotion;
	final loop:Bool;
	var started = false;
	/** Why the program stopped, when it failed. */
	public var failure(default, null):Null<String> = null;

	/**
	 * `metresPerUnit` converts the assembly's lengths. The simulated joints read from the assembly's
	 * starting pose, so each axis maps machine coordinates onto them with that pose as its offset.
	 */
	public function new(job:CncJob, robot:AssemblyRobot, definition:AssemblyDefinition, state:AssemblyStateRecord,
			metresPerUnit:Float, session:SimSession) {
		if (job.axes.length != 3) throw "A CNC job needs its X, Y and Z axes";
		this.session = session;
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
		var machine = new MachineBinding("machine", job.axes[0], job.axes[1], job.axes[2], rapid);
		var binding = new ToolpathMotionBinding(machine,
			new MotionSystemBlueprint(robot.model, robot.blueprint, axes, session.fixedTimestep()));
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
		// Spindle and tool-change handshakes are always ready: the simulation models neither.
		newMotion = () -> new ManipulatorMotion(robot.robot, binding.compiler,
			channel -> channel == "spindle.at_speed" || StringTools.startsWith(channel, "cnc.tool_change.") ?
				EventValue.Digital(true) : null,
			() -> robot.runtime.pollEvents());
		motion = newMotion();
	}

	public function feed():Void {
		if (failure != null) return;
		if (!started || (loop && motion.completed && !motion.running)) {
			motion.run(program);
			started = true;
		}
		motion.update(session.fixedTimestep());
		if (motion.failure != null) failure = motion.failure;
	}

	/** The session is back at its start, and the robot with it: run the program again from there. */
	public function reset():Void {
		motion = newMotion();
		started = false;
		failure = null;
	}

	public function present():Void {}
}
