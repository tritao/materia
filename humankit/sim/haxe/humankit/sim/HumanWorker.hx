package humankit.sim;

import humankit.HumanBody;
import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanJob;
import humankit.HumanLimb;
import humankit.Pick;
import humankit.Place;
import nativekit.sim.SimFrame;
import nativekit.sim.SimActor;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import NativeKitSim;

/** A job-driven animated worker and its physical capsule actor. */
class HumanWorker {
	static inline var LEAD_TICKS:Int = 3;
	public final session:SimSession;
	public final body:HumanBody;
	public final actor:HumanActor;
	final placeAnchor:SimActor;
	public final zones:Array<HumanZone> = [];
	public var onTick:Null<HumanWorker->HumanWorkerSignals->Void>;
	var job:Null<HumanJob>;
	var bindings:Array<{action:Dynamic, object:SimObject, hand:HumanLimb, grasp:Array<Float>}> = [];
	var grasps:Array<{object:SimObject, offset:Array<Float>, initial:SimPose}> = [];
	var links:Array<{id:String, body:nksim_body, radius:Float}> = [];
	var pending:Array<{time:Float, object:SimObject, hand:HumanLimb, kind:Int, goal:Null<SimPose>}> = [];
	var lastAction:Dynamic;
	var lastGrip:Bool = false;
	var lastAligned:Bool = false;
	var animationTime:Float = 0.0;
	var observer:Int;
	var disposed:Bool = false;

	public function new(session:SimSession, character:HumanCharacter, proxy:HumanBodyProxy, startPose:SimPose) {
		this.session = session;
		body = new HumanBody(character);
		body.walker.place(startPose.x, startPose.y,
			Math.atan2(2 * (startPose.qw * startPose.qz + startPose.qx * startPose.qy),
				1 - 2 * (startPose.qy * startPose.qy + startPose.qz * startPose.qz)));
		body.advance(0.0);
		actor = new HumanActor(session, proxy, character.pose, body.rootTransform());
		placeAnchor = session.createActor([SimShape.sphere(0.001)], [new SimPose(0, 0, -10)]);
		observer = session.addStepObserver(publishSignals);
	}

	/** Associates a Pick with a dynamic object and its world grasp point. */
	public function bindPick(action:Pick, object:SimObject, graspPoint:Array<Float>, hand:HumanLimb = ArmR):Void
		bind(action, object, graspPoint, hand);

	/** Associates a Place with the object to release at its world target. */
	public function bindPlace(action:Place, object:SimObject, graspPoint:Array<Float>, hand:HumanLimb = ArmR):Void
		bind(action, object, graspPoint, hand);

	function bind(action:Dynamic, object:SimObject, graspPoint:Array<Float>, hand:HumanLimb):Void {
		if (graspPoint.length < 3 || (hand != ArmL && hand != ArmR)) throw "A binding needs a grasp point and hand";
		if (Std.isOfType(action, Pick)) {
			var frame = session.capture();
			var pose = frame.objectPose(object);
			frame.dispose();
			grasps.push({object: object, offset: [pose.x - graspPoint[0], pose.y - graspPoint[1],
				pose.z - graspPoint[2]], initial: pose});
		}
		bindings.push({action: action, object: object, hand: hand, grasp: graspPoint.copy()});
	}

	public function run(job:HumanJob):Void {
		if (disposed) throw "Worker is disposed";
		if (this.job != null && !this.job.isDone()) throw "Worker already has a running job";
		job.bind(body);
		this.job = job;
		lastAction = null;
		lastGrip = false;
		lastAligned = false;
	}

	public function addZone(zone:HumanZone):Void
		zones.push(zone);

	/** A robot link's collision bound is approximated by a sphere around its body origin. */
	public function addRobotLink(id:String, link:nksim_body, radius:Float):Void {
		if (radius < 0.0) throw "Robot link radius must be non-negative";
		links.push({id: id, body: link, radius: radius});
	}

	/** Call before stepping the session. Jobs and actor poses run three ticks ahead. */
	public function advance():Void {
		if (disposed) throw "Worker is disposed";
		var now = session.simulationTime();
		var dt = session.fixedTimestep();
		flushPending(now + dt);
		var target = now + LEAD_TICKS * dt;
		if (target + 1e-9 < animationTime) {
			animationTime = now;
			pending.resize(0);
		}
		var elapsed = Math.max(0.0, target - animationTime);
		if (elapsed > 1e-9) {
			if (job != null) job.advance(elapsed); else body.advance(elapsed);
			animationTime = target;
			actor.pushPose(target, body.character.pose, body.rootTransform());
			var current = job == null ? null : job.currentAction();
			for (binding in bindings) {
				if (binding.action == current) {
					var grip = Std.isOfType(current, Pick) ? (cast current:Pick).grip :
						Std.isOfType(current, Place) ? (cast current:Place).grip : false;
					if (current != lastAction) {
						lastGrip = Std.isOfType(current, Place);
						lastAligned = false;
					}
					if (Std.isOfType(current, Place)) {
						var place:Place = cast current;
						if (place.atTarget && !lastAligned) {
							var grasp:Null<{object:SimObject, offset:Array<Float>, initial:SimPose}> = null;
							for (entry in grasps) if (entry.object == binding.object) grasp = entry;
							if (grasp == null) throw "Place needs a bound Pick for its object";
							pending.push({time: target, object: binding.object, hand: binding.hand, kind: 2,
								goal: new SimPose(place.target[0] + grasp.offset[0], place.target[1] + grasp.offset[1],
									place.target[2] + grasp.offset[2], grasp.initial.qx, grasp.initial.qy,
									grasp.initial.qz, grasp.initial.qw)});
							lastAligned = true;
						}
					}
					if (grip != lastGrip) pending.push({time: target, object: binding.object, hand: binding.hand,
						kind: grip ? 1 : 3, goal: null});
					lastGrip = grip;
					lastAction = current;
				}
			}
		}
	}

	function flushPending(until:Float):Void {
		while (pending.length > 0 && pending[0].time <= until + 1e-9) {
			var event = pending.shift();
			if (event.kind == 2) {
				var goal = event.goal;
				if (goal == null) throw "Place anchor requires a goal pose";
				placeAnchor.pushKeyframe(event.time, [new SimPose(goal.x, goal.y, -10)]);
				session.holdObject(event.object, placeAnchor.partBody(0),
					new SimPose(0, 0, goal.z + 10, goal.qx, goal.qy, goal.qz, goal.qw));
			} else if (event.kind == 1) {
				var frame = session.capture();
				var index = handIndex(event.hand);
				var carrierPose = frame.actorPose(actor.actor, index);
				var objectPose = event.goal == null ? frame.objectPose(event.object) : event.goal;
				frame.dispose();
				session.holdObject(event.object, actor.actor.partBody(index), relative(carrierPose, objectPose));
			} else session.releaseObject(event.object);
		}
	}

	function handIndex(hand:HumanLimb):Int {
		var suffix = hand == ArmL ? ".L" : ".R";
		for (name in ["hand" + suffix, "forearm" + suffix])
			for (index in 0...actor.proxy.capsules.length)
				if (actor.proxy.capsules[index].name == name) return index;
		throw 'No capsule for $hand';
	}

	function publishSignals():Void {
		var listener = onTick;
		if (disposed || listener == null) return;
		var frame = session.capture();
		var occupied:Array<String> = [];
		var distances:Map<String, Float> = new Map();
		for (index in 0...actor.proxy.capsules.length) {
			var capsule = actor.proxy.capsules[index];
			var pose = frame.actorPose(actor.actor, index);
			var axis = axisZ(pose);
			var half = capsule.length * 0.5;
			for (zone in zones) {
				if (occupied.indexOf(zone.id) >= 0) continue;
				if (zone.contains(pose.x, pose.y) ||
					zone.contains(pose.x + axis[0] * half, pose.y + axis[1] * half) ||
					zone.contains(pose.x - axis[0] * half, pose.y - axis[1] * half)) occupied.push(zone.id);
			}
			for (link in links) {
				var other = frame.bodyState(link.body).pose;
				var along = (other.x - pose.x) * axis[0] + (other.y - pose.y) * axis[1] +
					(other.z - pose.z) * axis[2];
				along = Math.max(-half, Math.min(half, along));
				var dx = other.x - pose.x - axis[0] * along;
				var dy = other.y - pose.y - axis[1] * along;
				var dz = other.z - pose.z - axis[2] * along;
				var distance = Math.max(0.0, Math.sqrt(dx * dx + dy * dy + dz * dz) - capsule.radius - link.radius);
				var previous = distances.get(link.id);
				if (previous == null || distance < previous) distances.set(link.id, distance);
			}
		}
		var signals = new HumanWorkerSignals(frame.simulationTime(), occupied, distances);
		frame.dispose();
		listener(this, signals);
	}

	public function dispose():Void {
		if (disposed) return;
		session.removeStepObserver(observer);
		// A sealed session owns its actor until the session itself is disposed.
		if (!session.isSealed()) actor.dispose();
		if (!session.isSealed()) placeAnchor.dispose();
		disposed = true;
	}

	static function axisZ(p:SimPose):Array<Float>
		return [2 * (p.qx * p.qz + p.qw * p.qy), 2 * (p.qy * p.qz - p.qw * p.qx),
			1 - 2 * (p.qx * p.qx + p.qy * p.qy)];

	static function rotate(q:SimPose, x:Float, y:Float, z:Float):Array<Float> {
		var tx = 2 * (q.qy * z - q.qz * y), ty = 2 * (q.qz * x - q.qx * z),
			tz = 2 * (q.qx * y - q.qy * x);
		return [x + q.qw * tx + q.qy * tz - q.qz * ty,
			y + q.qw * ty + q.qz * tx - q.qx * tz,
			z + q.qw * tz + q.qx * ty - q.qy * tx];
	}

	static function relative(parent:SimPose, child:SimPose):SimPose {
		var inverse = new SimPose(0, 0, 0, -parent.qx, -parent.qy, -parent.qz, parent.qw);
		var point = rotate(inverse, child.x - parent.x, child.y - parent.y, child.z - parent.z);
		return new SimPose(point[0], point[1], point[2],
			inverse.qw * child.qx + inverse.qx * child.qw + inverse.qy * child.qz - inverse.qz * child.qy,
			inverse.qw * child.qy - inverse.qx * child.qz + inverse.qy * child.qw + inverse.qz * child.qx,
			inverse.qw * child.qz + inverse.qx * child.qy - inverse.qy * child.qx + inverse.qz * child.qw,
			inverse.qw * child.qw - inverse.qx * child.qx - inverse.qy * child.qy - inverse.qz * child.qz);
	}
}
