package app;

import nativekit.sim.SimSession;
import robotkit.model.RobotModel;
import robotkit.world.JointTarget;
import robotkit.world.RobotCommand;
import robotkit.world.RobotWorld;

/**
 * Plays the document's robot motion tracks: before each tick, every tracked
 * joint is commanded to its track's position at the session's time. Tracks are
 * sampled from the session clock, so a reset session restarts them by itself.
 */
class RobotMotionPlayer implements SessionMember {
	final world:RobotWorld;
	final session:SimSession;
	final tracks:Array<RobotMotionTrack>;

	public function new(world:RobotWorld, session:SimSession, tracks:Array<RobotMotionTrack>) {
		this.world = world;
		this.session = session;
		this.tracks = tracks;
	}

	/**
	 * Checks each track against the robots it names and returns the tracks with
	 * their joint indexes resolved. Throws for a robot or joint that does not
	 * exist, or a key outside the joint's limits.
	 */
	public static function resolve(tracks:Array<RobotMotionTrack>, robotIds:Array<String>,
			models:Array<RobotModel>):Array<RobotMotionTrack> {
		var resolved:Array<RobotMotionTrack> = [];
		for (track in tracks) {
			var index = robotIds.indexOf(track.robotId);
			if (index < 0) throw 'Robot motion names unknown robot "${track.robotId}"';
			var joints = models[index].joints;
			var jointIndex = track.joint;
			if (track.jointId != null) {
				jointIndex = -1;
				for (i in 0...joints.length) if (joints[i].id == track.jointId) { jointIndex = i; break; }
				if (jointIndex < 0)
					throw 'Robot motion names unknown joint "${track.jointId}" of "${track.robotId}"';
			}
			if (jointIndex >= joints.length)
				throw 'Robot motion joint $jointIndex is missing from "${track.robotId}"';
			var limits = joints[jointIndex].limits;
			if (limits.lower < limits.upper) for (key in track.keys)
				if (key.position < limits.lower || key.position > limits.upper)
					throw 'Robot motion exceeds joint $jointIndex limits';
			resolved.push(jointIndex == track.joint ? track :
				new RobotMotionTrack(track.robotId, jointIndex, track.loop, track.keys, track.jointId));
		}
		return resolved;
	}

	public function feed():Void {
		var commands = new Map<String, Array<JointTarget>>();
		for (track in tracks) {
			var targets = commands.get(track.robotId);
			if (targets == null) { targets = []; commands.set(track.robotId, targets); }
			targets.push(JointTarget.position(track.joint, track.sample(session.simulationTime())));
		}
		for (id in commands.keys()) {
			var robot = world.robot(id);
			if (robot != null) robot.submit(RobotCommand.JointTargets(commands.get(id), null));
		}
	}

	public function beforeReset():Void {}

	public function reset():Void {}

	public function present():Void {}
}
