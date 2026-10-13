package robotkit.collision;

import collisionkit.CollisionBuild;
import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionMargins;
import collisionkit.CollisionMarginClass;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionWorld;
import collisionkit.kinematics.ModelBodies;
import robotkit.collision.RobotCollision;
import robotkit.collision.RobotCollision.RobotCollisionOptions;
import robotkit.manipulation.KinematicGroup;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Ownership is independent of a hull's name and its pose in a snapshot. */
enum CollisionOwner {
	Mechanism(controller:String, link:String);
	Fixture;
	Loose;
	Held(controller:String, link:String);
}

/** Ownership and tool identity supplement the collision description; geometry does not. */
typedef CollisionOwnership = {
	var owner:CollisionOwner;
	var tool:Bool;
}

/**
 * One cell description for static, process and conservative motion checks.
 * Poses and ownership are supplied by the current CAD or simulation snapshot;
 * fixtures and loose stock never become part of an articulation merely by
 * intersecting its reference pose. Named pair data owns contact permissions.
 */
class CollisionSceneBuilder {
	public final description:CollisionDescription;
	public final bodies:ModelBodies;
	public final arm:KinematicGroup;
	public final reference:Array<Float>;
	public final owner:String;
	public final ownership:Map<String, CollisionOwnership> = new Map();
	public final objects:Map<String, Int> = new Map();
	final memberBodies:Map<String, Int> = new Map();
	var sealed = false;

	public function new(arm:KinematicGroup, reference:Array<Float>, owner:String, margin:Float,
		?observed:kinematicskit.KinematicState, ?hulls:Array<robotkit.collision.RobotCollision.RobotLinkShape>) {
		if (owner == null || owner.length == 0 || !Math.isFinite(margin) || margin < 0)
			throw "Cell collision scene needs an owner and finite nonnegative margin";
		this.arm = arm; this.reference = reference.copy(); this.owner = owner;
		description = new CollisionDescription(margin);
		if (observed != null) {
			if (observed.model.dofCount() != arm.model.dofCount()) throw "Observed state belongs to another mechanism";
			for (dof in 0...arm.model.dofCount()) if (observed.model.dofId(dof) != arm.model.dofId(dof))
				throw "Observed state uses a different DOF ordering";
		}
		var state = arm.stateOf(reference, observed == null ? null : observed.q);
		var options = new RobotCollisionOptions(owner);
		options.reference = state.q;
		options.linkGroup = CollisionMarginClass.Installed;
		options.referenceName = "cell snapshot";
		options.declareArticulation = false;
		if (hulls != null) options.linkShapes = hulls;
		var converted = RobotCollision.describe(description, arm.robot, options);
		bodies = converted.bodies;
		// Preserve the source's stable object ids; the adapter owns all geometry and adjacency.
		if (hulls != null) for (i in 0...hulls.length) {
			var index = converted.sourceObjects[i], before = description.objects[index];
			description.objects[index] = new collisionkit.DescribedObject(hulls[i].name, before.body, before.offset,
				before.geometry, before.inflation, before.source, before.member, before.tool);
		}
		// The other coordinates are held by this snapshot. In particular a
		// stopped mobile wheel is not a moving obstacle to its support floor.
		for (index in bodies.all()) {
			var before = description.bodies[index];
			var fixed = !arm.moves(bodies.model.bodyIds[bodies.carrier(index)]);
			if (before.fixed != fixed)
				description.bodies[index] = new collisionkit.DescribedBody(before.name, before.group, fixed, before.pose);
		}
		description.note('Controller "$owner" holds coordinates outside the selected motion at the supplied snapshot');
		for (i in 0...description.objects.length) objects.set(description.objects[i].name, i);
	}

	/** Ownership facts for an object already emitted by a canonical source adapter. */
	public function identify(id:String, owner:CollisionOwner, tool:Bool):Void {
		var index = object(id);
		ownership.set(id, {owner: owner, tool: tool});
		var object = description.objects[index];
		description.objects[index] = new collisionkit.DescribedObject(object.name, object.body, object.offset, object.geometry, object.inflation, object.source, object.member, tool);
		memberBodies.set(description.objects[index].member, description.objects[index].body);
	}

	public function add(id:String, owner:CollisionOwner, vertices:Array<Float>, ?at:Transform3, tool:Bool = false,
		?member:String):Void {
		if (vertices == null || vertices.length < 12 || vertices.length % 3 != 0) throw "Cell hull needs xyz vertices";
		for (value in vertices) if (!Math.isFinite(value)) throw 'Cell hull "$id" has a nonfinite vertex';
		addGeometry(id, owner, CollisionGeometry.Convex(vertices.copy()), at, tool, member);
	}

	/** Primitives and CAD hulls share the description's ownership and pair-policy path. */
	public function addGeometry(id:String, owner:CollisionOwner, geometry:CollisionGeometry, ?at:Transform3,
		tool:Bool = false, ?member:String):Void {
		if (sealed) throw "Cell collision scene is already built";
		if (objects.exists(id)) throw 'Duplicate cell collision id "$id"';
		var offset = at == null ? Transform3.identity() : at;
		var group:Int = switch owner {
			case Held(_, _): CollisionMarginClass.Held;
			case Mechanism(controller, _): controller == this.owner ? (tool ? CollisionMarginClass.Tool : CollisionMarginClass.Installed) : CollisionMarginClass.Air;
			case Loose: CollisionMarginClass.Work;
			case Fixture: CollisionMarginClass.Air;
		};
		var body:Int;
		var physical = member == null ? id : member;
		var existing = memberBodies.get(physical);
		if (existing != null) {
			body = existing;
			var placed = description.bodies[body].pose;
			var bodyPose = Transform3.fromArrays([placed.x, placed.y, placed.z], [placed.qx, placed.qy, placed.qz, placed.qw]);
			switch owner {
				case Mechanism(controller, _) | Held(controller, _):
					if (controller != this.owner) offset = bodyPose.inverse().compose(offset);
				case Fixture | Loose: offset = bodyPose.inverse().compose(offset);
			}
		} else switch owner {
			case Mechanism(controller, link) | Held(controller, link):
				if (controller == this.owner) {
					var index = bodies.model.bodyIndex(link);
					if (index < 0) throw 'Hull "$id" has unknown link "$link"';
					body = switch owner {
						case Held(_, _): bodies.carry(index, physical, group);
						case _: bodies.follow(index, physical, group);
					};
				} else {
					body = description.addBody(physical, group, true, pose(offset));
					offset = Transform3.identity();
				}
			case Fixture | Loose:
				body = description.addBody(physical, group, true, pose(offset));
				offset = Transform3.identity();
		}
		memberBodies.set(physical, body);
		objects.set(id, description.addObject(id, body, pose(offset), geometry, 0,
			"cell CAD/simulation hull", physical, tool));
		ownership.set(id, {owner: owner, tool: tool});
	}

	public function pair(first:String, second:String, rule:CollisionPairRule, reason:CollisionPairStatus,
		?margin:Float, ?contactMargin:Float):Void {
		if (sealed) throw "Cell collision scene is already built";
		var a = object(first), b = object(second);
		description.setObjectRule(a, b, rule, reason);
		description.setObjectMargins(a, b, margin, contactMargin);
	}

	/** Held parts cannot inherit structural or reference-overlap exemptions. */
	public function payload(id:String, contacts:Array<String>, rule:CollisionPairRule,
		reason:CollisionPairStatus, contactMargin:Float):Void {
		var held = object(id);
		description.setHeldContacts(description.objects[held].body, [for (contact in contacts) object(contact)], rule, reason);
		for (other in description.objects) if (other.body != description.objects[held].body) {
			var contact = contacts.indexOf(other.name) >= 0;
			pair(id, other.name, contact ? rule : CollisionPairRule.Check,
				contact ? reason : CollisionPairStatus.Checked, contact ? contactMargin : description.defaultMargin);
		}
	}

	public function contacts(members:Array<String>):Array<String> {
		var result:Array<String> = [];
		for (object in description.objects) {
			if (members.indexOf(object.member) >= 0) result.push(object.name);
		}
		return result;
	}

	/** Every convex piece of the supplied physical CAD members, without interpreting object names. */
	public function memberObjects(members:Array<String>):Array<Int>
		return [for (name in contacts(members)) object(name)];

	public function toolObjects():Array<Int> {
		var result:Array<Int> = [];
		for (i in 0...description.objects.length) {
			if (description.objects[i].tool) result.push(i);
		}
		return result;
	}

	public function object(id:String):Int {
		var found = objects.get(id);
		if (found == null) throw 'Cell collision scene has no object "$id"';
		return found;
	}

	/** Register identities emitted by a source adapter; the description retains all geometry. */
	public function register(indices:Array<Int>, owner:CollisionOwner, tool:Bool):Void {
		if (sealed) throw "Cell collision scene is already built";
		for (index in indices) {
			if (index < 0 || index >= description.objects.length) throw "Collision adapter returned an unknown object";
			var object = description.objects[index];
			if (objects.exists(object.name)) throw 'Duplicate cell collision id "${object.name}"';
			objects.set(object.name, index);
			ownership.set(object.name, {owner: owner, tool: tool});
			memberBodies.set(object.member, object.body);
		}
	}

	public function margins(contact:Bool = false):CollisionMargins
		return contact ? description.contactMargins : description.margins;

	public function build(world:CollisionWorld):CollisionBuild {
		sealed = true;
		var articulation = bodies.declare();
		var build = description.build(world);
		return build;
	}

	/** The same geometry and named pair policy through RobotKit's process adapter. */
	public function clearance(makeWorld:Void->CollisionWorld):robotkit.collision.CollisionClearance {
		sealed = true;
		var articulation = bodies.declare();
		return new robotkit.collision.CollisionClearance(arm,
			description, reference, makeWorld, bodies);
	}

	public static function pose(value:Transform3):CollisionPose return new CollisionPose(value.translation.x,
		value.translation.y, value.translation.z, value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w);

}
