package humankit.sim;

import humankit.HumanBody;
import humankit.HumanAction;
import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanJob;
import humankit.HumanJobSpec;
import humankit.HumanJobBuilder;
import humankit.HumanJobBuildResult;
import humankit.Wait;
import humankit.HumanJobTargets;
import humankit.HumanLimb;
import humankit.Pick;
import humankit.Place;
import nativekit.sim.SimFrame;
import nativekit.sim.MotionType;
import nativekit.sim.SimActor;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import NativeKitSim;

/** A job-driven animated worker and its physical capsule actor. */
class HumanWorker {
	static inline var LEAD_TICKS:Int = 3;
	/** Pin a part where it rests: a reach toward it has started, or the hand has just opened on it. */
	static inline var PIN:Int = 0;
	/** Move the part to the hand, at the hand's grip-time pose. */
	static inline var HOLD:Int = 1;
	/** Let a pinned part go, once the hands are clear of it. */
	static inline var FREE:Int = 2;
	/** Re-hold a part at a new offset from the same hand (its carrier field is the offset). */
	static inline var REHOLD:Int = 3;
	/** How fast a Place turns a tilted part upright in the hand, in rad/s. */
	static inline var LEVEL_RATE:Float = 1.5;
	/** Where the stabiliser that pins parts during a reach sits, out of the way. */
	static final STABILISER_POSE = new SimPose(0.0, 0.0, -10.0);
	public final session:SimSession;
	public final body:HumanBody;
	public final actor:HumanActor;
	public final zones:Array<HumanZone> = [];
	public var onTick:Null<HumanWorker->HumanWorkerSignals->Void>;
	/** Farthest an object's origin may be from the hand's grip point when a hold starts, in metres. */
	public var maxHoldDistance:Float = 0.15;
	/**
	 * A fixed, tiny body that pins a part exactly where it is while the hand
	 * works on it: from the start of a Pick's reach until the hand closes, and
	 * from the moment a Place opens the hand until the hand has withdrawn. The
	 * hand's collision capsule covers the fingers coarsely and overlaps the part
	 * at those moments; it would otherwise knock or fling it.
	 */
	final stabiliser:SimActor;
	final twoHandCarrier:SimActor;
	var pinned:Array<SimObject> = [];
	/** The Place whose part stays pinned until the action ends. */
	var placing:Null<{action:Dynamic, object:SimObject, hand:HumanLimb, both:Bool}> = null;
	/** Placed parts that stay pinned until no hand or forearm capsule touches them. */
	var releasing:Array<{object:SimObject, hand:HumanLimb, both:Bool}> = [];
	/** Parts in a hand, with their offset from the hand capsule. */
	var held:Array<{object:SimObject, hand:HumanLimb, both:Bool, offset:SimPose}> = [];
	var job:Null<HumanJob>;
	var actionSteps:Array<Int> = [];
	var loopSpec:Null<HumanJobSpec>;
	var loopTargets:Null<HumanJobTargets>;
	var loopObjects:Null<Map<String, SimObject>>;
	var bindings:Array<{action:Dynamic, object:SimObject, hand:HumanLimb, both:Bool, grasp:Array<Float>}> = [];
	var lastPick:Null<Pick> = null;
	var lastPlace:Null<Place> = null;
	var links:Array<{id:String, pose:Void->SimPose, radius:Float}> = [];
	var pending:Array<{time:Float, object:SimObject, hand:HumanLimb, both:Bool, kind:Int, carrier:Null<SimPose>,
		touch:Null<Array<Float>>}> = [];
	var lastAction:Dynamic;
	var lastGrip:Bool = false;
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
		stabiliser = session.createActor([SimShape.sphere(0.001)], [STABILISER_POSE]);
		twoHandCarrier = session.createActor([SimShape.sphere(0.001)], [STABILISER_POSE]);
		observer = session.addStepObserver(publishSignals);
	}

	/** Associates a Pick with a dynamic object and its world grasp point. */
	public function bindPick(action:Pick, object:SimObject, graspPoint:Array<Float>, hand:HumanLimb = ArmR):Void
		bind(action, object, graspPoint, hand, false);

	public function bindBothPick(action:Pick, object:SimObject, graspPoint:Array<Float>):Void
		bind(action, object, graspPoint, ArmL, true);

	/** Associates a Place with the object to release at its world target. */
	public function bindPlace(action:Place, object:SimObject, graspPoint:Array<Float>, hand:HumanLimb = ArmR):Void
		bind(action, object, graspPoint, hand, false);

	public function bindBothPlace(action:Place, object:SimObject, graspPoint:Array<Float>):Void
		bind(action, object, graspPoint, ArmL, true);

	function bind(action:Dynamic, object:SimObject, graspPoint:Array<Float>, hand:HumanLimb, both:Bool):Void {
		if (graspPoint.length < 3 || (hand != ArmL && hand != ArmR)) throw "A binding needs a grasp point and hand";
		if (Std.isOfType(action, Pick)) lastPick = cast action;
		if (Std.isOfType(action, Place)) lastPlace = cast action;
		bindings.push({action: action, object: object, hand: hand, both: both, grasp: graspPoint.copy()});
	}

	public function pickError():Null<Float> return lastPick == null ? null : lastPick.pickError;
	public function placementError():Null<Float> return lastPlace == null ? null : lastPlace.placementError;

	public function run(job:HumanJob):Void {
		if (disposed) throw "Worker is disposed";
		if (this.job != null && !this.job.isDone()) throw "Worker already has a running job";
		job.bind(body);
		this.job = job;
		loopSpec = null;
		lastAction = null;
		lastGrip = false;
	}

	/** Resolves a document job and binds its dynamic parts to the session. */
	public function runSpec(spec:HumanJobSpec, targets:HumanJobTargets, objectsById:Map<String, SimObject>):Void {
		if (this.job != null && !this.job.isDone()) throw "Worker already has a running job";
		bindings = [];
		lastPick = null;
		lastPlace = null;
		var built:HumanJobBuildResult;
		try built = HumanJobBuilder.build(spec, targets, body) catch (error:Dynamic) {
			var failed = new HumanJob(body).add(new Wait(1.0));
			run(failed);
			failed.abort('Cannot resolve job targets: $error');
			return;
		}
		var seen:Array<HumanAction> = [];
		actionSteps = built.actionSteps;
		var failure:Null<String> = null;
		for (hold in built.holds) {
			if (seen.indexOf(hold.action) >= 0) continue;
			seen.push(hold.action);
			var both = false;
			for (other in built.holds) if (other.action == hold.action && other.hand != hold.hand) both = true;
			var object = objectsById.get(hold.objectId);
			if (object == null || object.motion != Dynamic) {
				failure = 'Job object "${hold.objectId}" is missing or not a dynamic SimObject';
				break;
			}
			if (Std.isOfType(hold.action, Pick)) {
				if (both) bindBothPick(cast hold.action, object, hold.grasp);
				else bindPick(cast hold.action, object, hold.grasp, hold.hand);
			} else {
				if (both) bindBothPlace(cast hold.action, object, hold.grasp);
				else bindPlace(cast hold.action, object, hold.grasp, hold.hand);
			}
		}
		run(built.job);
		if (failure != null) built.job.abort(failure);
		loopSpec = spec.loop && failure == null ? spec : null;
		loopTargets = targets;
		loopObjects = objectsById;
	}

	public function currentJobDone():Bool
		return job != null && job.isDone();

	public function currentJobFailure():Null<String>
		return job == null ? null : job.failure();

	/** Zero-based document step currently running, or null for an empty or finished job. */
	public function currentStep():Null<Int> {
		if (job == null || job.isDone()) return null;
		var index = job.currentIndex();
		return index < actionSteps.length ? actionSteps[index] : null;
	}

	public function addZone(zone:HumanZone):Void
		zones.push(zone);

	/** A robot link's collision bound is approximated by a sphere around its body origin. */
	public function addRobotLink(id:String, link:nksim_body, radius:Float):Void {
		addRobotLinkPose(id, function() {
			var frame = session.capture();
			var pose = frame.bodyState(link).pose;
			frame.dispose();
			return pose;
		}, radius);
	}

	/** Samples a robot runtime link's collision center after each physics tick. */
	public function addRobotLinkPose(id:String, pose:Void->SimPose, radius:Float):Void {
		if (radius < 0.0) throw "Robot link radius must be non-negative";
		links.push({id: id, pose: pose, radius: radius});
	}

	/** Call before stepping the session. Jobs and actor poses run three ticks ahead. */
	public function advance():Void {
		if (disposed) throw "Worker is disposed";
		if (loopSpec != null && job != null && job.isDone()) {
			if (job.failure() != null) loopSpec = null;
			else {
				var spec = loopSpec, targets = loopTargets, objects = loopObjects;
				if (spec != null && targets != null && objects != null) runSpec(spec, targets, objects);
			}
		}
		var now = session.simulationTime();
		var dt = session.fixedTimestep();
		flushPending(now + dt);
		// A failed or cancelled job leaves nothing pinned in mid-reach.
		// A job stopped early (failed or cancelled) still has a current action.
		if (job != null && job.isDone() && job.currentAction() != null && pinned.length > 0) {
			for (object in pinned) session.releaseObject(object);
			pinned = [];
			releasing = [];
		}
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
			var carryWithBoth = false;
			for (entry in held) if (entry.both) carryWithBoth = true;
			if (Std.isOfType(current, Pick) && (cast current:Pick).hands.length == 2 ||
				Std.isOfType(current, Place) && (cast current:Place).hands.length == 2)
				carryWithBoth = true;
			twoHandCarrier.pushKeyframe(target, [carryWithBoth ? twoHandPose() : STABILISER_POSE]);
			var placed = placing;
			if (placed != null && placed.action != current) {
				releasing.push({object: placed.object, hand: placed.hand, both: placed.both});
				placing = null;
			}
			for (waiting in releasing.copy())
				if (handsClear(waiting.object)) {
					pending.push({time: target, object: waiting.object, hand: waiting.hand, both: waiting.both, kind: FREE, carrier: null,
						touch: null});
					releasing.remove(waiting);
				}
			for (binding in bindings) {
				if (binding.action == current) {
					var grip = Std.isOfType(current, Pick) ? (cast current:Pick).grip :
						Std.isOfType(current, Place) ? (cast current:Place).grip : false;
					if (current != lastAction) {
						lastGrip = Std.isOfType(current, Place);
						if (Std.isOfType(current, Pick))
							pending.push({time: target, object: binding.object, hand: binding.hand, both: binding.both, kind: PIN,
								carrier: null, touch: null});
					}
					// The session carries a held object on the hand capsule's pose at
					// the event time, which is the animation pose right now.
					if (grip != lastGrip) {
						pending.push({time: target, object: binding.object, hand: binding.hand, both: binding.both,
							kind: grip ? HOLD : PIN, carrier: grip ? (binding.both ? twoHandPose() : handPlacement(binding.hand)) : null,
							touch: grip ? body.gripPoint(binding.hand) : null});
						if (!grip) placing = {action: current, object: binding.object, hand: binding.hand, both: binding.both};
					}
					lastGrip = grip;
					lastAction = current;
					if (grip && Std.isOfType(current, Place)) level(binding.object, elapsed, target);
				}
			}
		}
	}

	/**
	 * Turns a held part toward upright (keeping its heading) about its own
	 * centre, at LEVEL_RATE, while a Place lowers it: the hand rolls during a
	 * carry, and a part set down tilted would tip over on release.
	 */
	function level(object:SimObject, seconds:Float, time:Float):Void {
		for (entry in held) {
			if (entry.object != object) continue;
			var hand = entry.both ? twoHandPose() : handPlacement(entry.hand);
			var carrier = [hand.qx, hand.qy, hand.qz, hand.qw];
			var offset = [entry.offset.qx, entry.offset.qy, entry.offset.qz, entry.offset.qw];
			var world = multiply(carrier, offset);
			var yaw = Math.atan2(2 * (world[3] * world[2] + world[0] * world[1]),
				1 - 2 * (world[1] * world[1] + world[2] * world[2]));
			var upright = [0.0, 0.0, Math.sin(yaw * 0.5), Math.cos(yaw * 0.5)];
			var dot = world[0] * upright[0] + world[1] * upright[1] + world[2] * upright[2] + world[3] * upright[3];
			if (dot < 0.0) {
				upright = [for (value in upright) -value];
				dot = -dot;
			}
			var angle = 2 * Math.acos(Math.min(1.0, dot));
			if (angle < 1e-4) return;
			var t = Math.min(1.0, LEVEL_RATE * seconds / angle);
			var turned = [for (axis in 0...4) world[axis] + (upright[axis] - world[axis]) * t];
			var length = Math.sqrt(turned[0] * turned[0] + turned[1] * turned[1] + turned[2] * turned[2] +
				turned[3] * turned[3]);
			var next = multiply([-carrier[0], -carrier[1], -carrier[2], carrier[3]], [for (value in turned) value / length]);
			entry.offset = new SimPose(entry.offset.x, entry.offset.y, entry.offset.z, next[0], next[1], next[2], next[3]);
			pending.push({time: time, object: object, hand: entry.hand, both: entry.both, kind: REHOLD, carrier: entry.offset,
				touch: null});
			return;
		}
	}

	static function multiply(a:Array<Float>, b:Array<Float>):Array<Float>
		return [
			a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
			a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
			a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
			a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]
		];

	function forgetHeld(object:SimObject):Void
		for (entry in held.copy()) if (entry.object == object) held.remove(entry);

	function flushPending(until:Float):Void {
		while (pending.length > 0 && pending[0].time <= until + 1e-9) {
			var event = pending.shift();
			if (event.kind == PIN) {
				var frame = session.capture();
				var objectPose = frame.objectPose(event.object);
				frame.dispose();
				session.holdObject(event.object, stabiliser.partBody(0), relative(STABILISER_POSE, objectPose));
				if (pinned.indexOf(event.object) < 0) pinned.push(event.object);
				body.setHeldPoint(event.hand, null);
				if (event.both) body.setHeldPoint(ArmR, null);
				forgetHeld(event.object);
			} else if (event.kind == REHOLD) {
				var offset = event.carrier;
				if (offset != null) session.holdObject(event.object,
					event.both ? twoHandCarrier.partBody(0) : handBody(event.hand), offset);
			} else if (event.kind == HOLD) {
				var index = handIndex(event.hand);
				var carrierPose = event.carrier;
				if (carrierPose == null) throw "A hold needs the hand's pose at its grip time";
				var frame = session.capture();
				var objectPose = frame.objectPose(event.object);
				frame.dispose();
				var offset = relative(carrierPose, objectPose);
				var touch = event.touch;
				if (touch == null) throw "A hold needs the hand's grip point at its grip time";
				var reach = Math.sqrt(Math.pow(objectPose.x - touch[0], 2) + Math.pow(objectPose.y - touch[1], 2) +
					Math.pow(objectPose.z - touch[2], 2));
				if (reach > maxHoldDistance) {
					if (job != null)
						job.abort('the object is $reach m from the hand, beyond the $maxHoldDistance m hold limit');
					pending.resize(0);
					return;
				}
				if (event.both) {
					var right = body.gripPoint(ArmR);
					var rightReach = Math.sqrt(Math.pow(objectPose.x - right[0], 2) +
						Math.pow(objectPose.y - right[1], 2) + Math.pow(objectPose.z - right[2], 2));
					if (rightReach > maxHoldDistance) {
						if (job != null) job.abort('the object is $rightReach m from the right hand');
						pending.resize(0);
						return;
					}
				}
				// Holding again moves the part from the stabiliser to the hand.
				session.holdObject(event.object,
					event.both ? twoHandCarrier.partBody(0) : actor.actor.partBody(index), offset);
				pinned.remove(event.object);
				forgetHeld(event.object);
				held.push({object: event.object, hand: event.hand, both: event.both, offset: offset});
				var hand = event.hand;
				var both = event.both;
				body.setHeldPoint(hand, function() {
					var carrier = both ? twoHandPose() : handPlacement(hand);
					var point = rotate(carrier, offset.x, offset.y, offset.z);
					var root = body.rootTransform();
					var side = both ? 0.08 : 0.0;
					return [carrier.x + point[0] + root[4] * side,
						carrier.y + point[1] + root[5] * side, carrier.z + point[2]];
				});
				if (event.both) body.setHeldPoint(ArmR, function() {
					var carrier = twoHandPose();
					var point = rotate(carrier, offset.x, offset.y, offset.z);
					var root = body.rootTransform();
					return [carrier.x + point[0] - root[4] * 0.08,
						carrier.y + point[1] - root[5] * 0.08, carrier.z + point[2]];
				});
			} else if (pinned.remove(event.object)) {
				session.releaseObject(event.object);
			}
		}
	}

	/**
	 * True when no hand or forearm capsule, at the current animation pose, comes
	 * within the object's bounding sphere (plus 5 mm).
	 */
	function handsClear(object:SimObject):Bool {
		var frame = session.capture();
		var pose = frame.objectPose(object);
		frame.dispose();
		var reach = object.shape.boundingRadius() + 0.005;
		var placements = actor.proxy.place(body.character.pose, body.rootTransform());
		for (index in 0...actor.proxy.capsules.length) {
			var capsule = actor.proxy.capsules[index];
			if (capsule.name.indexOf("hand") < 0 && capsule.name.indexOf("forearm") < 0) continue;
			var placement = placements[index];
			var centre = new SimPose(placement.center[0], placement.center[1], placement.center[2],
				placement.rotation[0], placement.rotation[1], placement.rotation[2], placement.rotation[3]);
			var axis = axisZ(centre), half = capsule.length * 0.5;
			var along = (pose.x - centre.x) * axis[0] + (pose.y - centre.y) * axis[1] + (pose.z - centre.z) * axis[2];
			along = Math.max(-half, Math.min(half, along));
			var dx = pose.x - centre.x - axis[0] * along, dy = pose.y - centre.y - axis[1] * along,
				dz = pose.z - centre.z - axis[2] * along;
			if (Math.sqrt(dx * dx + dy * dy + dz * dz) < capsule.radius + reach) return false;
		}
		return true;
	}

	/** The session body of a hand's collision capsule, which carries what that hand holds. */
	public function handBody(hand:HumanLimb):nksim_body
		return actor.actor.partBody(handIndex(hand));

	/** The hand capsule's world pose for the current animation pose. */
	function handPlacement(hand:HumanLimb):SimPose {
		var placement = actor.proxy.place(body.character.pose, body.rootTransform())[handIndex(hand)];
		return new SimPose(placement.center[0], placement.center[1], placement.center[2], placement.rotation[0],
			placement.rotation[1], placement.rotation[2], placement.rotation[3]);
	}

	/** A kinematic midpoint carrier driven by the left and right hand capsules. */
	function twoHandPose():SimPose {
		var left = handPlacement(ArmL), right = handPlacement(ArmR);
		var root = body.rootTransform();
		var yaw = Math.atan2(root[1], root[0]) * 0.5;
		return new SimPose((left.x + right.x) * 0.5, (left.y + right.y) * 0.5,
			(left.z + right.z) * 0.5, 0.0, 0.0, Math.sin(yaw), Math.cos(yaw));
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
				var other = link.pose();
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
		if (!session.isSealed()) { stabiliser.dispose(); twoHandCarrier.dispose(); }
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
