package app;

import nativekit.sim.MotionType;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import robotkit.runtime.RobotContact;
import robotkit.runtime.RobotContactOtherKind;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.Simulation;
import robotkit.spatial.Vec3;

/** A vacuum command resolved to the robot and link it acts on. */
typedef ResolvedGrip = {time:Float, robotIndex:Int, linkIndex:Int, grip:Bool};

/** A scene object the simulation owns, by scene id. */
typedef GripObject = {id:String, object:SimObject};

/**
 * Plays the project's vacuum commands: a tool link grips the free object it is touching, carries it,
 * and lets go. Commands fire from the session clock, once per cycle of the motion they go with, so a
 * reset session starts them over by itself.
 */
class RobotGripPlayer implements SessionMember {
	/** How far from a tool link an object may be for its vacuum to seal on it, in metres. */
	static inline var GRIP_REACH:Float = 0.004;
	/**
	 * The gap left between the cup and the workpiece it holds. A held object is driven to follow its
	 * carrier and cannot yield, so a contact between them would push back on the arm; a gap far larger
	 * than the contact's own tolerance keeps that force at exactly zero and is invisible at this scale.
	 */
	static inline var GRIP_CLEARANCE:Float = 0.001;

	final session:SimSession;
	final robots:Simulation;
	/** The runtime of each robot, by robot index. */
	final runtimes:Array<RobotRuntime>;
	final objects:Array<GripObject>;
	/** Vacuum commands in time order. */
	final grips:Array<ResolvedGrip>;
	/** Seconds after which the commands repeat; zero when they run once. */
	final period:Float;
	var next:Int = 0;
	var cycle:Int = 0;
	var lastTime:Float = 0.0;
	/** What each gripping link holds, keyed `robot:link`. */
	final held:Map<String, GripObject> = new Map();

	public function new(session:SimSession, robots:Simulation, runtimes:Array<RobotRuntime>,
			objects:Array<GripObject>, grips:Array<ResolvedGrip>, period:Float) {
		this.session = session;
		this.robots = robots;
		this.runtimes = runtimes;
		this.objects = objects;
		this.grips = grips;
		this.period = period;
	}

	/** Scene ids of the objects the tool links are holding right now. */
	public function heldIds():Array<String> return [for (entry in held) entry.id];

	/** Fires every vacuum command whose time has come. */
	public function feed():Void {
		var now = session.simulationTime();
		// Time only runs backward when the session was reset: start the commands over.
		if (now < lastTime) restart();
		lastTime = now;
		while (next < grips.length) {
			var event = grips[next];
			if (now < cycle * period + event.time) break;
			apply(event);
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
		held.clear();
	}

	function apply(event:ResolvedGrip):Void {
		var key = event.robotIndex + ":" + event.linkIndex;
		if (!event.grip) {
			var holding = held.get(key);
			if (holding == null) return;
			held.remove(key);
			session.releaseObject(holding.object);
			return;
		}
		if (held.exists(key)) return;
		var touched = candidate(event);
		// Nothing under the cup: the vacuum finds no seal, so nothing is held.
		if (touched == null) return;
		var link = robots.linkPose(event.robotIndex, event.linkIndex);
		var frame = session.capture();
		var pose:SimPose;
		try pose = frame.objectPose(touched.entry.object) catch (failure:Dynamic) {
			frame.dispose();
			throw failure;
		}
		frame.dispose();
		// Ease the object to the clearance along the contact normal, away from the cup.
		var away = touched.contact.normal;
		var toObject = new Vec3(pose.x - link.position[0], pose.y - link.position[1], pose.z - link.position[2]);
		if (away.dot(toObject) < 0.0) away = new Vec3(-away.x, -away.y, -away.z);
		var shift = Math.max(0.0, GRIP_CLEARANCE - touched.contact.distance);
		var seated = new SimPose(pose.x + away.x * shift, pose.y + away.y * shift, pose.z + away.z * shift,
			pose.qx, pose.qy, pose.qz, pose.qw);
		session.holdObject(touched.entry.object, robots.linkBody(event.robotIndex, event.linkIndex),
			relativePose(link.position, link.rotation, seated));
		held.set(key, touched.entry);
	}

	/** The free object nearest the link within reach, that no other link already holds. */
	function candidate(event:ResolvedGrip):Null<{entry:GripObject, contact:RobotContact}> {
		var best:Null<{entry:GripObject, contact:RobotContact}> = null;
		var bestDistance = GRIP_REACH;
		for (contact in robots.robotContacts(runtimes[event.robotIndex])) {
			if (contact.linkIndex != event.linkIndex || contact.otherKind != RobotContactOtherKind.Object ||
					contact.distance > bestDistance) continue;
			for (entry in objects) {
				if (entry.object.handle.rawValue() != contact.otherObject || entry.object.motion != MotionType.Dynamic)
					continue;
				var taken = false;
				for (holding in held) if (holding.object == entry.object) taken = true;
				if (taken) continue;
				best = {entry: entry, contact: contact};
				bestDistance = contact.distance;
			}
		}
		return best;
	}

	/** The object's pose in the frame of the link that carries it. */
	static function relativePose(linkPosition:Array<Float>, linkRotation:Array<Float>, object:SimPose):SimPose {
		var inverse = [-linkRotation[0], -linkRotation[1], -linkRotation[2], linkRotation[3]];
		var offset = ApplicationSimulation.rotateOffset(object.x - linkPosition[0], object.y - linkPosition[1],
			object.z - linkPosition[2], inverse);
		var ax = inverse[0], ay = inverse[1], az = inverse[2], aw = inverse[3];
		return new SimPose(offset[0], offset[1], offset[2],
			aw * object.qx + ax * object.qw + ay * object.qz - az * object.qy,
			aw * object.qy - ax * object.qz + ay * object.qw + az * object.qx,
			aw * object.qz + ax * object.qy - ay * object.qx + az * object.qw,
			aw * object.qw - ax * object.qx - ay * object.qy - az * object.qz);
	}
}
