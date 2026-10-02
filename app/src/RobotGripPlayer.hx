package app;

import app.SimulatedTools.GripObject;
import nativekit.sim.SimSession;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulatedSuctionTool;

/** A vacuum command resolved to the robot and link it acts on. */
typedef ResolvedGrip = {time:Float, robotIndex:Int, linkIndex:Int, grip:Bool};

/**
 * Plays the project's timed vacuum commands on the robot's simulated suction tools: a tool link grips
 * the free object it is touching, carries it, and lets go. Commands fire from the session clock, once
 * per cycle of the motion they go with, so a reset session starts them over by itself. The motion they
 * go with streams joint positions and runs no trajectory plan to carry channel events, so the tools
 * are actuated directly.
 */
class RobotGripPlayer implements SessionMember {
	final session:SimSession;
	final robots:Simulation;
	/** The runtime of each robot, by robot index. */
	final runtimes:Array<RobotRuntime>;
	final tools:SimulatedTools;
	/** Vacuum commands in time order. */
	final grips:Array<ResolvedGrip>;
	/** Seconds after which the commands repeat; zero when they run once. */
	final period:Float;
	var next:Int = 0;
	var cycle:Int = 0;
	var lastTime:Float = 0.0;
	/** The tool on each gripping link, keyed `robot:link`. */
	final byLink:Map<String, SimulatedSuctionTool> = new Map();

	public function new(session:SimSession, robots:Simulation, runtimes:Array<RobotRuntime>, tools:SimulatedTools,
			grips:Array<ResolvedGrip>, period:Float) {
		this.session = session;
		this.robots = robots;
		this.runtimes = runtimes;
		this.tools = tools;
		this.grips = grips;
		this.period = period;
		for (event in grips) {
			var key = event.robotIndex + ":" + event.linkIndex;
			if (byLink.exists(key)) continue;
			var tool = tools.add(new SimulatedSuctionTool(robots, runtimes[event.robotIndex], event.robotIndex, event.linkIndex,
				null, [for (entry in tools.objectsByScene()) entry.object]));
			robots.addStepObserver(tool);
			byLink.set(key, tool);
		}
	}

	/** Fires every vacuum command whose time has come. */
	public function feed():Void {
		var now = session.simulationTime();
		// Time only runs backward when the session was reset: start the commands over.
		if (now < lastTime) restart();
		lastTime = now;
		while (next < grips.length) {
			var event = grips[next];
			if (now < cycle * period + event.time) break;
			byLink.get(event.robotIndex + ":" + event.linkIndex).actuate(event.grip);
			next++;
			// A motion that repeats also repeats its commands; one that runs once is done.
			if (next >= grips.length && period > 0) {
				next = 0;
				cycle++;
			}
		}
	}

	public function reset():Void restart();

	public function present():Void {}

	function restart():Void {
		next = 0;
		cycle = 0;
		lastTime = 0.0;
	}
}
