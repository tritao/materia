package cadbridge;

import cadbridge.AssemblySimulationBridge.AssemblyLinkHull;
import robotkit.collision.CollisionSceneBuilder;
import robotkit.collision.CollisionSceneBuilder.CollisionOwner;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.RobotModel;
import robotkit.spatial.Transform3;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadbridge.CadCollision.CadPartCollision;

/** CAD geometry and its observed world pose remain source facts until description building. */
typedef CellCadObservation = {
	var member:String;
	var frame:AssemblyFrame;
	var metresPerUnit:Float;
	var collision:CadPartCollision;
}

/** Live geometry observations exist only before building the shared description. */
typedef CellHullObservation = {
	var id:String;
	var owner:CollisionOwner;
	var vertices:Array<Float>;
	var pose:Transform3;
	var tool:Bool;
	var member:String;
}

/** Geometry and live poses collected once, before choosing a collision consumer. */
class CellCollisionSnapshot {
	public final controller:String;
	public final root:Transform3;
	public final hulls:Array<CellHullObservation> = [];
	public final parts:Array<CellCadObservation> = [];
	final ignored:Array<String>;
	final ids = new Map<String, Bool>();
	final primaryLinks = new Map<String, Transform3>();
	var observed:Null<kinematicskit.KinematicState> = null;
	final physical = new Map<String, {points:Array<Float>, fixed:Bool}>();

	public static function observation(id:String, owner:CollisionOwner, vertices:Array<Float>, ?pose:Transform3,
		tool:Bool = false, ?member:String):CellHullObservation return {
		id: id, owner: owner, vertices: vertices.copy(), pose: pose == null ? Transform3.identity() : pose, tool: tool,
		member: member == null ? id : member
	};

	public function new(controller:String, root:Transform3, ignored:Array<String>, ?observations:Array<CellHullObservation>) {
		this.controller = controller; this.root = root; this.ignored = ignored.copy();
		if (observations != null) for (hull in observations)
			add(observation(hull.id, hull.owner, hull.vertices, hull.pose, hull.tool, hull.member));
	}

	/** Other controllers remain independent geometry at their supplied live poses. */
	public function mechanism(owner:String, model:RobotModel, geometry:Array<AssemblyLinkHull>,
		poses:Array<Transform3>, ?toolMembers:Array<String>, ?state:kinematicskit.KinematicState):Void {
		if (poses.length != model.links.length) throw "Cell mechanism needs one current pose per link";
		var compiled = robotkit.kinematics.RobotKinematics.compile(model);
		if (owner == controller) for (i in 0...poses.length) primaryLinks.set(model.links[i].id, poses[i]);
		if (owner == controller && state != null) {
			if (state.model.dofCount() != compiled.dofCount()) throw "Cell mechanism needs a complete observed DOF state";
			for (dof in 0...compiled.dofCount()) if (state.model.dofId(dof) != compiled.dofId(dof))
				throw "Cell observations use a different DOF ordering";
			observed = state.copy();
		}
		var pieces = new Map<String, Array<AssemblyLinkHull>>();
		var members:Array<String> = [];
		for (hull in geometry) if (ignored.indexOf(hull.part) < 0) {
			if (hull.link < 0 || hull.link >= model.links.length) throw 'Cell hull "${hull.part}" lost its link';
			var list = pieces.get(hull.part);
			if (list == null) { list = []; pieces.set(hull.part, list); members.push(hull.part); }
			list.push(hull);
		}
		for (member in members) {
			var list = pieces.get(member);
			if (list == null || list.length == 0) throw "Observed CAD member lost its pieces";
			var link = list[0].link;
			var points:Array<Float> = [];
			for (piece in list) {
				if (piece.link != link) throw 'Physical CAD member "$member" spans different mechanical links';
				for (point in CollisionSceneBuilder.placed(piece.vertices, poses[link])) points.push(point);
			}
			var fixed = compiled.bodyChain[compiled.bodyIndex(model.links[link].id)].length == 0;
			if (ids.exists(member)) {
				var first = physical.get(member);
				if (first == null || !first.fixed || !fixed || !same(first.points, points))
					throw 'Cell occurrence "$member" has conflicting controller geometry or moving ownership';
				// All pieces of a shared fixed occurrence remain one environment object set.
				for (piece in 0...list.length) {
					var id = list.length == 1 ? member : '$member/piece$piece';
					var world = CollisionSceneBuilder.placed(list[piece].vertices, poses[link]);
					for (i in 0...hulls.length) if (hulls[i].id == id)
						hulls[i] = observation(id, Fixture, world, root.inverse(), false, member);
				}
				continue;
			}
			var tool = toolMembers != null && toolMembers.indexOf(member) >= 0;
			var pose = owner == controller ? Transform3.identity() : root.inverse().compose(poses[link]);
			for (piece in 0...list.length) {
				var id = list.length == 1 ? member : '$member/piece$piece';
				add(observation(id, Mechanism(owner, model.links[link].id), list[piece].vertices, pose, tool, member));
			}
			ids.set(member, true);
			physical.set(member, {points: points, fixed: fixed});
		}
	}

	/** A loose or physically carried object; the caller supplies observed ownership. */
	public function object(id:String, pieces:Array<Array<Float>>, worldPose:Transform3, ?carrier:String,
		kind:CollisionOwner = Loose):Void {
		if (ids.exists(id) || ignored.indexOf(id) >= 0) return;
		var owner:CollisionOwner = kind;
		switch kind {
			case Fixture | Loose:
			case _: throw "An independent object must declare fixture or loose ownership before attachment";
		}
		var local = root.inverse().compose(worldPose);
		if (carrier != null) {
			var frame = primaryLinks.get(carrier);
			if (frame == null) throw 'Object "$id" has unknown carrier "$carrier"';
			local = frame.inverse().compose(worldPose);
			owner = Held(controller, carrier);
		}
		for (i in 0...pieces.length)
			add(observation(pieces.length == 1 ? id : '$id#$i', owner, pieces[i], local, false, id));
		ids.set(id, true);
	}

	/** Fixed installed geometry, including deposited metal, at its current world pose. */
	public function fixture(id:String, vertices:Array<Float>, worldPose:Transform3):Void {
		if (ids.exists(id) || ignored.indexOf(id) >= 0) return;
		add(observation(id, Fixture, vertices, root.inverse().compose(worldPose)));
	}

	/** Installed CAD at its actual pose, through the canonical CAD collision adapter. */
	public function part(observation:CellCadObservation):Void {
		if (ignored.indexOf(observation.member) >= 0) return;
		if (ids.exists(observation.member)) throw 'Duplicate observed CAD member "${observation.member}"';
		if (!Math.isFinite(observation.metresPerUnit) || observation.metresPerUnit <= 0)
			throw "Observed CAD geometry needs its length scale";
		parts.push(observation);
		ids.set(observation.member, true);
	}

	function add(hull:CellHullObservation):Void {
		if (ids.exists(hull.id)) throw 'Duplicate cell snapshot id "${hull.id}"';
		ids.set(hull.id, true); hulls.push(hull);
	}

	static function same(first:Array<Float>, second:Array<Float>):Bool {
		if (first.length != second.length) return false;
		for (i in 0...first.length) if (Math.abs(first[i] - second[i]) > 1e-9) return false;
		return true;
	}

	public function includes(id:String):Bool return ids.exists(id);

	/** The same fixed cell geometry viewed from another robot parking pose. */
	public function at(root:Transform3):CellCollisionSnapshot {
		var result = new CellCollisionSnapshot(controller, root, ignored);
		for (part in parts) result.part(part);
		var state = observed;
		if (state != null) result.observed = state.copy();
		var relocation = root.compose(this.root.inverse());
		for (id in primaryLinks.keys()) {
			var pose = primaryLinks.get(id);
			if (pose != null) result.primaryLinks.set(id, relocation.compose(pose));
		}
		for (hull in hulls) {
			var independent = switch hull.owner {
				case Mechanism(owner, _) | Held(owner, _): owner != controller;
				case Fixture | Loose: true;
			};
			result.add(observation(hull.id, hull.owner, hull.vertices,
				independent ? root.inverse().compose(this.root).compose(hull.pose) : hull.pose, hull.tool, hull.member));
		}
		for (id in physical.keys()) {
			var observation = physical.get(id);
			if (observation == null) continue;
			var movesWithPrimary = false;
			for (hull in hulls) if (hull.member == id) switch hull.owner {
				case Mechanism(owner, _) | Held(owner, _): movesWithPrimary = owner == controller;
				case Fixture | Loose:
			}
			result.physical.set(id, {points: movesWithPrimary ? CollisionSceneBuilder.placed(observation.points, relocation)
				: observation.points.copy(), fixed: observation.fixed});
		}
		return result;
	}

	/** CAD assembly parts use their actual posed hulls and exported mechanism ownership. */
	public static function fromAssembly(controller:String, definition:materia.assembly.AssemblyDefinition,
		physical:AssemblySimulationBridge.AssemblyPhysicalData, state:cadkit.modeling.AssemblyState,
		mechanism:AssemblySimulationBridge.AssemblySimulationModel, ignored:Array<String>, ?toolMembers:Array<String>):CellCollisionSnapshot {
		var result = new CellCollisionSnapshot(controller, Transform3.identity(), ignored);
		var model = robotkit.kinematics.RobotKinematics.compile(mechanism.model);
		var snapshot = new kinematicskit.KinematicSnapshot(model);
		snapshot.evaluate(new kinematicskit.KinematicState(model));
		result.mechanism(controller, mechanism.model, mechanism.linkHulls,
			[for (link in mechanism.model.links) robotkit.kinematics.RobotKinematics.toTransform3(snapshot.bodyPose(model.bodyIndex(link.id)))],
			toolMembers);
		var excluded = ignored.copy();
		for (part in mechanism.partLinks.keys()) if (excluded.indexOf(part) < 0) excluded.push(part);
		var partData = new Map<String, AssemblySimulationBridge.AssemblyPhysicalPart>();
		for (part in physical.parts) partData.set(part.id, part);
		for (occurrence in materia.assembly.AssemblyDefinitionFlattener.flatten(definition).occurrences)
			if (excluded.indexOf(occurrence.id) < 0) {
				var source = partData.get(occurrence.definition);
				if (source == null || source.collisionHull == null) throw 'Cell part "${occurrence.id}" needs its physical CAD hull';
				result.part({member: occurrence.id, frame: state.worldPose(occurrence.id),
					metresPerUnit: physical.metresPerUnit, collision: Hull(source.collisionHull)});
			}
		return result;
	}

	/** Fixed world bounds for floor navigation, derived from the same captured hulls. */
	public function fixedBounds():Array<{id:String, minimum:Array<Float>, maximum:Array<Float>}> {
		var result:Array<{id:String, minimum:Array<Float>, maximum:Array<Float>}> = [];
		for (part in parts) {
			var description = new collisionkit.CollisionDescription();
			var indices = CadCollision.describePart(description, part.member, part.frame, part.metresPerUnit, part.collision, 0);
			for (index in indices) {
				var object = description.objects[index], body = description.bodies[object.body];
				var vertices = switch object.geometry { case Convex(points): points; case _: throw "Fixed CAD bounds need finite convex geometry"; };
				var points = CollisionSceneBuilder.placed(vertices, Transform3.fromArrays(
					[body.pose.x, body.pose.y, body.pose.z], [body.pose.qx, body.pose.qy, body.pose.qz, body.pose.qw]));
				var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
				var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
				for (i in 0...points.length) { var axis = i % 3; low[axis] = Math.min(low[axis], points[i]); high[axis] = Math.max(high[axis], points[i]); }
				result.push({id: part.member, minimum: low, maximum: high});
			}
		}
		for (hull in hulls) {
			var independent = switch hull.owner {
				case Mechanism(owner, _) | Held(owner, _): owner != controller;
				case Fixture | Loose: true;
			};
			if (!independent) continue;
			var points = CollisionSceneBuilder.placed(hull.vertices, root.compose(hull.pose));
			var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
			var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
			for (i in 0...points.length) {
				var axis = i % 3;
				low[axis] = Math.min(low[axis], points[i]); high[axis] = Math.max(high[axis], points[i]);
			}
			result.push({id: hull.id, minimum: low, maximum: high});
		}
		return result;
	}

	/** Build in the compiled model's cell frame, without changing the captured geometry. */
	public function describe(arm:KinematicGroup, reference:Array<Float>, margin:Float):CollisionSceneBuilder {
		var mechanismHulls:Array<robotkit.collision.RobotCollision.RobotLinkHull> = [];
		for (hull in hulls) switch hull.owner {
			case Mechanism(owner, link) if (owner == controller):
				var index = -1;
				for (i in 0...arm.robot.links.length) if (arm.robot.links[i].id == link) index = i;
				if (index < 0) throw 'Observed hull "${hull.id}" has unknown link "$link"';
				mechanismHulls.push(new robotkit.collision.RobotCollision.RobotLinkHull(index, hull.id,
					CollisionSceneBuilder.placed(hull.vertices, hull.pose), hull.tool ? collisionkit.CollisionMarginClass.Tool : collisionkit.CollisionMarginClass.Installed, hull.member));
			case _:
		}
		var scene = new CollisionSceneBuilder(arm, reference, controller, margin, observed, mechanismHulls);
		// Observed independent geometry is already in the controller root's
		// frame. The selected arm group's root can be a descendant of it.
		for (hull in hulls) switch hull.owner {
			case Mechanism(owner, _) if (owner == controller): scene.identify(hull.id, hull.owner, hull.tool);
			case _: scene.add(hull.id, hull.owner, hull.vertices, hull.pose, hull.tool, hull.member);
		}
		for (part in parts) {
			var scale = part.metresPerUnit, frame = part.frame;
			var at = root.inverse().compose(Transform3.fromArrays([frame.x * scale, frame.y * scale, frame.z * scale],
				[frame.qx, frame.qy, frame.qz, frame.qw]));
			var posed:AssemblyFrame = {x: at.translation.x / scale, y: at.translation.y / scale, z: at.translation.z / scale,
				qx: at.rotation.x, qy: at.rotation.y, qz: at.rotation.z, qw: at.rotation.w};
			var indices = CadCollision.describePart(scene.description, part.member, posed, scale, part.collision, 0);
			scene.register(indices, Fixture, false);
		}
		// Initial held objects get the same collision policy as a live transfer;
		// subsequent process policy names the actual permitted contacts.
		var heldBodies:Array<Int> = [];
		for (hull in hulls) switch hull.owner {
			case Held(_, _):
				var body = scene.description.objects[scene.object(hull.id)].body;
				if (heldBodies.indexOf(body) < 0) { scene.description.setHeldContacts(body, []); heldBodies.push(body); }
			case _:
		}
		return scene;
	}
}
