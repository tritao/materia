package app;

import motionkit.robot.AxisKinematics;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MotionSystem;
import motionkit.robot.PlanCheckSummary;
import motionkit.robot.PlanCheck.PlanCheckOptions;
import motionkit.robot.SessionState;
import motionkit.program.MotionProgram;
import nativekit.sim.SimSession;
import robotkit.runtime.EncoderMonitor;
import motionkit.robot.StepperSlip;
import toolpathkit.motion.ToolpathSourceMap;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathProgram;

/**
 * What a G-code job on a simulated machine shares, whatever the machine does with it (cut stock, deposit filament):
 * the program is compiled from wherever the machine stands, homing runs on the machine's switches first when it has
 * them, the plans stream to the robot before each tick, a looping job starts again when it completes, and the
 * operator's speed override, feed hold, plan checks and stepper slip act on it. A subclass builds the machine's
 * motion (`newMotion`, `newHoming`, `compileFrom`, the solver and its axis joints) and says what happens around each
 * step through the hooks.
 */
class ToolpathPlayer implements SessionMember {
	final session:SimSession;
	final robot:AssemblyRobot;
	final source:String;
	final loop:Bool;
	/** Compiles the program from the machine's position into `program` and `sourceMap`, and returns the compiled toolpath. */
	var compileFrom:Point3->ToolpathProgram;
	/** Machine coordinates from the axis joints' positions, and those joints' indices on the robot. */
	var solver:AxisKinematics;
	var axisJoints:Array<Int>;
	var program:MotionProgram;
	var sourceMap:ToolpathSourceMap;
	var newMotion:Void->ManipulatorMotion;
	var motion:ManipulatorMotion;
	var newHoming:Void->Null<MotionSystem>;
	var homing:Null<MotionSystem> = null;
	var homingStarted:Bool = false;
	var homingComplete:Bool = false;
	var started = false;
	var passCounted = false;
	/** Peak observed following error per machine axis, metres, aligned with the drive interval end. */
	public final axisTrackingPeak:Array<Float> = [];
	/** Passes of the program completed since the session started. */
	public var passes(default, null):Int = 0;
	/** Why the program stopped, when it failed. */
	public var failure(default, null):Null<String> = null;
	/** What the plan check assumes and allows, as the player was given it. */
	public final checkOptions:PlanCheckOptions;
	/**
	 * Steps the machine's steppers lose where the plan check finds them over their pull-out curve: the
	 * simulated axis falls behind its command and keeps the error until the session resets. Plans the
	 * check passes lose nothing.
	 */
	public var slip:StepperSlip;
	/**
	 * What the machine's encoders read against the commanded positions: a motor-side encoder names the steps a
	 * stepper lost, a load-side one reports the path error. Monitoring only; it has nothing to read when the
	 * machine has no encoders.
	 */
	public var encoders:EncoderMonitor;
	/** Physical homing time, excluded from planning/execution timings. */
	public var homingSeconds(default, null):Float = 0.0;
	/** Wall-clock seconds spent running the machine, for profiling. */
	public var motionSeconds(default, null):Float = 0.0;
	/**
	 * Whether the program's first plan is made on the planner thread while the simulation goes on, as it is in the
	 * editor, rather than waited for, as a deterministic stepper (a test) does so that its results do not depend on how
	 * long planning takes. Set by whoever drives the simulation.
	 */
	public var planAhead:Bool = false;
	public var runSeconds(default, null):Float = 0.0;
	public var slowestUpdate(default, null):Float = 0.0;
	/** The G-code line the machine is executing; 0 between lines. */
	public var currentLine(default, null):Int = 0;
	/** Speed of every move, as a fraction of the program's. */
	public var speedOverride(default, null):Float = 1.0;

	function new(robot:AssemblyRobot, session:SimSession, source:String, loop:Bool, check:Null<PlanCheckOptions>) {
		this.robot = robot;
		this.session = session;
		this.source = source;
		this.loop = loop;
		checkOptions = check == null ? new PlanCheckOptions() : check.copy();
	}

	// ---- what a subclass says about its machine ----------------------------------------------------------------------

	/** The job cannot be fed yet (a device is still synchronising its clock). */
	function waiting():Bool return false;

	/**
	 * After homing, before the program runs: false while the machine is still getting ready (a tended mill moving to
	 * its load position), which takes the step.
	 */
	function prepare():Bool return true;

	/** Before each step, after homing: whatever the subclass must do to keep the machine ready (operator restarts, sensors). */
	function beforeStep():Void {}

	/** Before a pass may start: `passFinished` says the running pass completed on this step (a cell's signals see it). */
	function exchangeSignals(passFinished:Bool):Void {}

	/** Whether a pass starts on this step: the first one, and again whenever a looping job has completed. */
	function startsPass():Bool return !started || (loop && motion.completed && !motion.running);

	/** The motion stopped with `reason`; the player adds what it knows of why. */
	function failureReason(reason:String):String return reason;

	/** The job stops for `reason`. */
	function fail(reason:String):Void {
		if (failure == null) failure = reason;
	}

	/** The operator's restart is over, or the job starts again: the failure it had is cleared. */
	function clearFailure():Void failure = null;

	/** A pass is about to be (re)compiled and run. */
	function beginPass():Void {}

	/** The program stopped for a reason the operator gave, which is not a failure. */
	function stoppedOnPurpose():Bool return false;

	/** Where the program is, as the original program's op and distance (a restarted program maps back to it). */
	function originalAt(op:Int, distance:Float):{op:Int, distance:Float} return {op: op, distance: distance};

	/**
	 * After each step: where the machine is in the program, the source provenance of that place, and whether the
	 * pass completed on this step.
	 */
	function afterStep(at:{op:Int, distance:Float}, provenance:Null<Provenance>, passFinished:Bool):Void {}

	/** The session is back at its start, and the robot with it: return the job to its start too. */
	function resetJob():Void {}

	/** Compiles the program from where the robot stands. */
	function compileFromRobot():Void {
		var positions = robot.robot.snapshot().positions;
		var pose = solver.forward([for (index in axisJoints) positions.get(index)]);
		compileFrom(new Point3(pose.x, pose.y, pose.z));
	}

	/**
	 * A successfully completed move plans its successor from the exact terminal setpoint. Use that
	 * same machine pose when compiling the next program; measured feedback can differ by a few
	 * micrometres while still satisfying the motion completion tolerance. Where the robot stands when
	 * nothing has been commanded.
	 */
	function machinePositionAtMotionEnd():Point3 {
		var joints = motion.commandedPositions();
		if (joints == null) {
			var actual = robot.robot.snapshot().positions;
			joints = [for (index in axisJoints) actual.get(index)];
		}
		var pose = solver.forward(joints);
		return new Point3(pose.x, pose.y, pose.z);
	}

	// ---- the machine ---------------------------------------------------------------------------------------------------

	public function feed():Void {
		if (failure != null || waiting()) return;
		var homeView = homing;
		if (!homingComplete && homeView != null) {
			try {
				if (!homingStarted) {
					// feed runs before physics; wait for the first actual switch observations.
					var seen = robot.robot.snapshot(), available = new Map<String, Bool>();
					for (frame in seen.sensors.toArray()) available.set(frame.sensorId, true);
					for (contact in robot.blueprint.switches)
						if (contact.role == "home" && !available.exists(contact.id)) return;
					homeView.home();
					homingStarted = true;
					return;
				}
				homingSeconds += session.fixedTimestep();
				homeView.update(session.fixedTimestep());
				if (homeView.homingStatus() != "Complete") return;
				homingComplete = true;
				// Latching changed the calibration revision used by every plan.
				motion = newMotion();
				if (speedOverride != 1.0) motion.setSpeedOverride(speedOverride);
				compileFromRobot();
			} catch (error:Dynamic) {
				failure = 'Machine homing: $error';
				return;
			}
		}
		if (!prepare()) return;
		var clock = Sys.time();
		beforeStep();
		var passFinished = started && motion.completed && !passCounted;
		exchangeSignals(passFinished);
		if (passFinished) {
			passes++;
			passCounted = true;
		}
		if (startsPass()) {
			beginPass();
			// A program runs from wherever the machine is: the position the planner last
			// commanded, or where the robot stands when there is none.
			if (started) compileFrom(machinePositionAtMotionEnd());
			motion.run(program, !planAhead);
			started = true;
			passCounted = false;
			runSeconds += Sys.time() - clock;
			clock = Sys.time();
		}
		motion.update(session.fixedTimestep());
		if (encoders.readings.length > 0) {
			var seen = robot.robot.snapshot();
			encoders.observe(seen.positions.toArray(), seen.setpointPositions.toArray(), session.simulationTime());
			for (axis in 0...axisJoints.length) {
				var joint = axisJoints[axis];
				axisTrackingPeak[axis] = Math.max(axisTrackingPeak[axis],
					Math.abs(seen.positions.get(joint) - seen.setpointPositions.get(joint)));
			}
		}
		var spent = Sys.time() - clock;
		motionSeconds += spent;
		slowestUpdate = Math.max(slowestUpdate, spent);
		if (motion.failure != null && !stoppedOnPurpose()) fail(failureReason(motion.failure));
		var progress = motion.progress();
		var at = originalAt(progress.op, progress.pathDistance);
		var provenance = at.op < 0 ? null : sourceMap.provenanceAt(at.op, at.distance);
		currentLine = provenance == null ? 0 : provenance.line;
		afterStep(at, provenance, passFinished);
	}

	/** What the plan checks have found in the plans the machine has started: stall and accuracy findings and how near the drives came. */
	public function planChecks():PlanCheckSummary return motion.checks;

	/** A finding in words, with the G-code line of its op when the program knows it. */
	public function describe(diagnostic:motionkit.trajectory.PlanDiagnostic):String {
		var line = 0;
		if (diagnostic.opIndex >= 0 && sourceMap != null) {
			var found = sourceMap.provenanceAt(diagnostic.opIndex, Math.max(0.0, diagnostic.pathDistance));
			if (found != null) line = found.line;
		}
		return diagnostic.describe(line);
	}

	/** What the machine does besides move, for the panel's status line (a tool, a layer and temperatures). */
	public function statusDetail():String return "";

	/** The panel's heading. */
	public function title():String return "PROGRAM";

	/** Restarts the program at G-code line `line` or the first line after it that moves; false when the job cannot restart (or no later line moves). */
	public function restartFromLine(line:Int):Bool return false;

	/** The job can be restarted from a line the operator picks. */
	public function canRestart():Bool return false;

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

	public function beforeReset():Void {
		if (homing != null && homing.isMoving()) homing.abort();
	}

	public function reset():Void {
		slip.reset();
		encoders.reset([for (_ in robot.model.joints) 0.0]);
		motion = newMotion();
		homing = newHoming();
		homingStarted = false;
		homingComplete = homing == null;
		homingSeconds = 0.0;
		if (speedOverride != 1.0) motion.setSpeedOverride(speedOverride);
		currentLine = 0;
		started = false;
		failure = null;
		passes = 0;
		passCounted = false;
		resetJob();
	}

	public function present():Void {}
}
