package machinekit.welding;

import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.assembly.Diagnostics;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.welding.WeldSeam;
import machinekit.welding.WeldSeams;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A joint the weldment says is welded: two members, and how the weld is to be made. */
typedef WeldJoint = {
	var a:String;
	var b:String;
	/** Leg size in millimetres. */
	var legSize:Float;
	var passes:Int;
	var travelAngle:Float;
	var workAngle:Float;
}

/** What `Weldment.find` found: the seams, and an error for every declared joint that has none. */
class WeldmentSeams {
	public final seams:Array<WeldSeam>;
	public final diagnostics:Diagnostics;

	public function new(seams:Array<WeldSeam>, diagnostics:Diagnostics) {
		this.seams = seams;
		this.diagnostics = diagnostics;
	}

	/** The seams, or an exception listing every joint that could not be found. */
	public function require():Array<WeldSeam> {
		diagnostics.throwIfErrors();
		return seams;
	}

	/** The seam with that name, or null. */
	public function named(name:String):Null<WeldSeam> {
		for (seam in seams) if (seam.name() == name) return seam;
		return null;
	}

	/** The seams between two members, whichever order they were declared in. */
	public function between(a:String, b:String):Array<WeldSeam>
		return [for (seam in seams) if (seam.a.member == a && seam.b.member == b || seam.a.member == b && seam.b.member == a) seam];
}

/**
 * The welded joints of a workpiece. It names the members that make up the workpiece and the pairs of them
 * that are welded, with the leg size and passes for each. It does not say where: the seams come from the
 * members' geometry (`WeldSeams`), so they follow the members as they are edited and a pair that does not
 * touch is reported rather than skipped.
 *
 * The seams are expressed in the workpiece's own frame, the pose of its `reference` member. A robot's
 * program gets the seam frames there and places them with the workpiece's pose in the world, so a
 * correction to that pose (the part lay a little off, or touch sensing found it) moves every seam.
 */
class Weldment {
	/** The member whose pose is the workpiece's frame. */
	public final reference:String;
	/** Occurrence ids of the members. */
	public final members:Array<String>;
	public final joints:Array<WeldJoint> = [];

	public function new(reference:String, members:Array<String>) {
		if (members.indexOf(reference) < 0) throw 'Weldment reference "$reference" is not one of its members';
		this.reference = reference;
		this.members = members.copy();
	}

	/** Declares that members `a` and `b` are welded, with a leg of `legSize` millimetres in `passes` passes. */
	public function join(a:String, b:String, legSize:Float, passes:Int = 1, travelAngle:Float = WeldSeam.PUSH_ANGLE,
			workAngle:Float = 0):Weldment {
		for (id in [a, b]) if (members.indexOf(id) < 0) throw 'Weldment joint names "$id", which is not a member';
		if (a == b) throw "A weld joint joins two different members";
		joints.push({a: a, b: b, legSize: legSize, passes: passes, travelAngle: travelAngle, workAngle: workAngle});
		return this;
	}

	/** The same weldment for an assembly that includes the workpiece under `prefix`, such as `work/`. */
	public function prefixed(prefix:String):Weldment {
		var result = new Weldment(prefix + reference, [for (id in members) prefix + id]);
		for (joint in joints) result.join(prefix + joint.a, prefix + joint.b, joint.legSize, joint.passes, joint.travelAngle, joint.workAngle);
		return result;
	}

	/**
	 * Finds the seams of every declared joint in the members' parts (by occurrence id, in the world), expressed in the
	 * frame of `workpiece`, the reference member's pose, when given. The parts are borrowed. A joint whose members do
	 * not touch gives an error naming them and the gap between their bounds.
	 */
	public function find(parts:Map<String, Part>, ?workpiece:AssemblyFrame):WeldmentSeams {
		var seams:Array<WeldSeam> = [];
		var diagnostics = new Diagnostics();
		var inverse = workpiece == null ? null : AssemblyFrames.inverse(workpiece);
		for (joint in joints) {
			var a = parts.get(joint.a), b = parts.get(joint.b);
			if (a == null || b == null) {
				diagnostics.error("weld.missing-geometry", '${joint.a}|${joint.b}', 'Welded joint ${joint.a} to ${joint.b} has no geometry for ${a == null ? joint.a : joint.b}');
				continue;
			}
			var found = WeldSeams.find({id: joint.a, part: a}, {id: joint.b, part: b}, joint.legSize, joint.passes, joint.travelAngle,
				joint.workAngle);
			if (found.length == 0) {
				var apart = gap(a, b);
				diagnostics.error("weld.no-seam", '${joint.a}|${joint.b}', 'No seam between ${joint.a} and ${joint.b}: ' + (apart > 0
					? 'their bounds are ${Math.round(apart * 100) / 100} mm apart'
					: 'they do not meet in a corner a weld can fill'));
				continue;
			}
			for (seam in found) seams.push(inverse == null ? seam : seam.transformed(inverse));
		}
		return new WeldmentSeams(seams, diagnostics);
	}

	/**
	 * Finds the seams in an assembly that contains the members, from their geometry at `poses` (`assembly.solvedPoses()`,
	 * or the state a robot is simulated in). They come out in the workpiece's frame, so they stay put when the
	 * workpiece is moved as a whole.
	 */
	public function findIn(assembly:MachineAssembly, poses:Map<String, AssemblyFrame>):WeldmentSeams {
		var parts:Map<String, Part> = new Map();
		try {
			for (entry in assembly.components()) if (members.indexOf(entry.id) >= 0) {
				var pose = poses.get(entry.id);
				if (pose == null) throw 'No pose for weldment member "${entry.id}"';
				parts.set(entry.id, posed(entry.component.geometry(ComponentDetail.Preview), pose));
			}
			var reference = poses.get(this.reference);
			if (reference == null) throw 'No pose for weldment reference "${this.reference}"';
			var result = find(parts, reference);
			for (part in parts) part.close();
			return result;
		} catch (error:Dynamic) {
			for (part in parts) part.close();
			throw error;
		}
	}

	/** The distance between the two parts' bounding boxes, 0 when they overlap. */
	static function gap(a:Part, b:Part):Float {
		var first = a.shape.bounds(), second = b.shape.bounds();
		var lowA = first.get_min(), highA = first.get_max(), lowB = second.get_min(), highB = second.get_max();
		var dx = Math.max(0, Math.max(lowA.get_x() - highB.get_x(), lowB.get_x() - highA.get_x()));
		var dy = Math.max(0, Math.max(lowA.get_y() - highB.get_y(), lowB.get_y() - highA.get_y()));
		var dz = Math.max(0, Math.max(lowA.get_z() - highB.get_z(), lowB.get_z() - highA.get_z()));
		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	/** `local` moved to `pose`; takes ownership of `local`. */
	static function posed(local:Part, pose:AssemblyFrame):Part {
		var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
		var placed = local.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z), new Vector(x.x, x.y, x.z),
			new Vector(z.x, z.y, z.z))));
		local.close();
		return placed;
	}
}
