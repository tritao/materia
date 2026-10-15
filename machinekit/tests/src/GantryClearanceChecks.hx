import machinekit.gantry.Gantry;
import machinekit.assembly.Transmission;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyBodies;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;

/** OCCT checks across moving bodies; fixed-body contacts and declared drive meshes are interfaces. */
class GantryClearanceChecks {
	final localParts = new Map<String, Part>();
	final localBoxes = new Map<String, PartBox>();
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
				var part = member.component.geometry(ComponentDetail.Preview);
				localParts.set(member.id, part);
				localBoxes.set(member.id, PosedParts.boxOf(part));
			}
		} catch (error:Dynamic) { close(); throw error; }
	}

	function allow(a:String, b:String):Void {
		interfaces.set(a + "|" + b, true); interfaces.set(b + "|" + a, true);
	}

	public function close():Void {
		for (part in localParts) if (!part.shape.isClosed()) part.close();
	}

	static function boxesOverlap(a:PartBox, b:PartBox):Bool
		return Math.min(a.maxX, b.maxX) - Math.max(a.minX, b.minX) > 1e-7 &&
			Math.min(a.maxY, b.maxY) - Math.max(a.minY, b.minY) > 1e-7 &&
			Math.min(a.maxZ, b.maxZ) - Math.max(a.minZ, b.minZ) > 1e-7;

	/** Members are moved to their poses only for pairs whose boxes, the local boxes carried to the poses, overlap. */
	public function check(state:AssemblyState):Void {
		var placed = new Map<String, Part>();
		var boxes = new Map<String, PartBox>();
		var ids:Array<String> = [];
		function dispose():Void { for (part in placed) if (!part.shape.isClosed()) part.close(); }
		function at(id:String):Part {
			var world = placed.get(id);
			if (world != null) return world;
			var part = localParts.get(id);
			if (part == null) throw "Gantry clearance member is missing";
			var moved = PosedParts.moved(part, state.worldPose(id));
			placed.set(id, moved);
			return moved;
		}
		function overlap(a:String, b:String):Float {
			var first = at(a), second = at(b);
			var common = first.intersect(second);
			try { var volume = common.volume(); common.close(); return volume; }
			catch (error:Dynamic) { common.close(); throw error; }
		}
		try {
			for (id in localParts.keys()) {
				var local = localBoxes.get(id);
				if (local == null) throw "Missing gantry geometry";
				boxes.set(id, PosedParts.carried(local, state.worldPose(id))); ids.push(id);
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
