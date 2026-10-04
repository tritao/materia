import machinekit.gantry.Gantry;
import machinekit.assembly.Transmission;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyBodies;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

private typedef ClearanceBox = {minX:Float, minY:Float, minZ:Float, maxX:Float, maxY:Float, maxZ:Float};

/** OCCT checks across moving bodies; fixed-body contacts and declared drive meshes are interfaces. */
class GantryClearanceChecks {
	final localParts = new Map<String, Part>();
	final body = new Map<String, String>();
	final interfaces = new Map<String, Bool>();
	final sensors = new Map<String, Bool>();
	final rackMeshes:Array<{pinion:String, rack:String}> = [];
	final shaftSeatings:Array<{parent:String, child:String}> = [];

	public function new(gantry:Gantry, definition:AssemblyDefinition) {
		for (group in AssemblyBodies.of(definition)) for (member in group.occurrences) body.set(member, group.id);
		// A shaft is seated inside the motor or journal on its moving mate.
		for (joint in definition.joints) if (joint.type == materia.assembly.AssemblyDefinition.AssemblyJointType.Continuous) {
			allow(joint.parent, joint.child); shaftSeatings.push({parent: joint.parent, child: joint.child});
		}
		var description = gantry.describe().machine;
		if (definition.switches != null) for (contact in definition.switches) sensors.set(contact.part, true);
		var transmissions = description.transmissions;
		if (transmissions != null) for (relation in transmissions) switch relation.source {
			case LeadScrew(screw, nut): allow(screw, nut);
			case TimingBelt(belt, pulley) | BeltIdler(belt, pulley): allow(belt, pulley);
			case RackAndPinion(pinion, rack):
				if (rack != null) { allow(pinion, rack); rackMeshes.push({pinion: pinion, rack: rack}); }
			case _: throw "Unexpected drive in the translational gantry";
		}
		var paths = description.beltPaths;
		if (paths != null) for (path in paths) {
			var clamp = path.clamp;
			if (clamp != null) allow(path.belt, clamp.instanceId);
		}
		try {
			for (member in gantry.components()) {
				if (!body.exists(member.id)) throw 'Gantry body ownership is missing for ${member.id}';
				localParts.set(member.id, member.component.geometry(ComponentDetail.Preview));
			}
		} catch (error:Dynamic) { close(); throw error; }
	}

	function allow(a:String, b:String):Void {
		interfaces.set(a + "|" + b, true); interfaces.set(b + "|" + a, true);
	}

	public function close():Void {
		for (part in localParts) if (!part.shape.isClosed()) part.close();
	}

	static function bounds(part:Part):ClearanceBox {
		var box = part.shape.bounds(), lo = box.get_min(), hi = box.get_max();
		return {minX: lo.get_x(), minY: lo.get_y(), minZ: lo.get_z(), maxX: hi.get_x(), maxY: hi.get_y(), maxZ: hi.get_z()};
	}
	static function boxesOverlap(a:ClearanceBox, b:ClearanceBox):Bool
		return Math.min(a.maxX, b.maxX) - Math.max(a.minX, b.minX) > 1e-7 &&
			Math.min(a.maxY, b.maxY) - Math.max(a.minY, b.minY) > 1e-7 &&
			Math.min(a.maxZ, b.maxZ) - Math.max(a.minZ, b.minZ) > 1e-7;

	public function check(state:AssemblyState):Void {
		var placed = new Map<String, Part>();
		var boxes = new Map<String, ClearanceBox>();
		var ids:Array<String> = [];
		function dispose():Void { for (part in placed) if (!part.shape.isClosed()) part.close(); }
		function overlap(a:String, b:String):Float {
			var first = placed.get(a), second = placed.get(b);
			if (first == null || second == null) throw "Gantry clearance member is missing";
			var common = first.intersect(second);
			try { var volume = common.volume(); common.close(); return volume; }
			catch (error:Dynamic) { common.close(); throw error; }
		}
		try {
			for (id in localParts.keys()) {
				var part = localParts.get(id);
				if (part == null) throw "Missing gantry geometry";
				var pose = state.worldPose(id);
				var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
				var world = part.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z),
					new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z))));
				placed.set(id, world); boxes.set(id, bounds(world)); ids.push(id);
			}
			var clashes:Array<String> = [];
			for (i in 0...ids.length) for (j in i + 1...ids.length) {
				var a = ids[i], b = ids[j];
				// Separate sensor bodies cannot occupy the same space even when
				// both mounts belong to one fixed structural body.
				var sensorPair = sensors.exists(a) && sensors.exists(b);
				if ((!sensorPair && body.get(a) == body.get(b)) || interfaces.exists(a + "|" + b)) continue;
				var boxA = boxes.get(a), boxB = boxes.get(b);
				if (boxA == null || boxB == null) throw "Missing gantry bounds";
				if (!boxesOverlap(boxA, boxB)) continue;
				var volume = overlap(a, b);
				if (volume > 1e-3) clashes.push('$a / $b: $volume mm³');
			}
			if (clashes.length > 0) throw "Gantry interference at " +
				[state.joint("x"), state.joint("y"), state.joint("z")].join(", ") + ":\n" + clashes.join("\n");
			for (pair in shaftSeatings) {
				var volume = overlap(pair.parent, pair.child);
				if (volume > 1e-3) throw 'Shaft does not fit ${pair.child}: $volume mm³';
			}
			// These teeth must actually mesh, unlike the belt's nominal band envelope.
			for (pair in rackMeshes) {
				var volume = overlap(pair.pinion, pair.rack);
				if (volume > 0.05) throw 'Rack/pinion tooth interference ${pair.pinion}: $volume mm³';
			}
			dispose();
		} catch (error:Dynamic) { dispose(); throw error; }
	}
}
