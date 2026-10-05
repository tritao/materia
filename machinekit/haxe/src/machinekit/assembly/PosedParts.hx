package machinekit.assembly;

import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** An axis-aligned box, in mm. */
typedef PartBox = {
	final minX:Float;
	final minY:Float;
	final minZ:Float;
	final maxX:Float;
	final maxY:Float;
	final maxZ:Float;
}

/**
 * Parts of an assembly placed at poses for clearance checks, which pose the same part again and again (against each of
 * several others, at each position of a sweep).
 *
 * A component's geometry is built once per `geometryKey` and kept; each pose is a new part made by moving it, so the
 * expensive solid construction is not repeated. The kept shapes are native and belong to this object: close it when the
 * check is done. Poses it returned are the caller's to close and stay valid after it is closed.
 */
class PosedParts {
	/** Parts whose boxes are further apart than this (mm) are not tested with a boolean. */
	public static inline final APART_MARGIN = 1.0;

	final locals:Map<String, Part> = [];

	public function new() {}

	/** `component`'s geometry moved to `pose`: a new owned part. */
	public function posed(component:MachineComponent, pose:AssemblyFrame, detail:ComponentDetail = Preview):Part {
		var key = component.geometryKey(detail), local = locals.get(key);
		if (local == null) {
			local = component.geometry(detail);
			locals.set(key, local);
		}
		var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
		return local.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z), new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z))));
	}

	/** The member `id` of `assembly` at its pose in `state`: a new owned part. `label` names the assembly in the error for an unknown id. */
	public function member(assembly:MachineAssembly, state:AssemblyState, label:String, id:String):Part {
		for (entry in assembly.components()) if (entry.id == id) return posed(entry.component, state.worldPose(id));
		throw '$label has no member "$id"';
	}

	/** The volume (mm³) members `a` and `b` of `assembly` share at the poses in `state`. */
	public function volume(assembly:MachineAssembly, state:AssemblyState, label:String, a:String, b:String):Float {
		var first = member(assembly, state, label, a), second = member(assembly, state, label, b);
		var shared = commonVolume(first, boxOf(first), second, boxOf(second));
		first.close();
		second.close();
		return shared;
	}

	/**
	 * Throws, worded by `describe(moving, other, volume)`, for the first member of `moving` that shares more than `limit` mm³
	 * with a member of `others`, trying pairs in order. Each member is posed once for all its pairs.
	 */
	public function checkClear(assembly:MachineAssembly, state:AssemblyState, label:String, moving:Array<String>, others:Array<String>,
			describe:(String, String, Float) -> String, limit:Float = 1e-3):Void {
		var first:Array<Part> = [], second:Array<Part> = [];
		try {
			for (id in moving) first.push(member(assembly, state, label, id));
			for (id in others) second.push(member(assembly, state, label, id));
			var firstBoxes = [for (part in first) boxOf(part)], secondBoxes = [for (part in second) boxOf(part)];
			for (a in 0...first.length) for (b in 0...second.length) {
				var shared = commonVolume(first[a], firstBoxes[a], second[b], secondBoxes[b]);
				if (shared > limit) throw describe(moving[a], others[b], shared);
			}
		} catch (error:Dynamic) {
			closeAll(first);
			closeAll(second);
			throw error;
		}
		closeAll(first);
		closeAll(second);
	}

	/** Runs `body` with a new `PosedParts` and closes it afterwards, including when `body` throws. */
	public static function scope(body:PosedParts -> Void):Void {
		var parts = new PosedParts();
		try body(parts) catch (error:Dynamic) {
			parts.close();
			throw error;
		}
		parts.close();
	}

	/** Closes the geometry this object keeps. */
	public function close():Void {
		for (part in locals) part.close();
		locals.clear();
	}

	public static function boxOf(part:Part):PartBox {
		var bounds = part.shape.bounds(), low = bounds.get_min(), high = bounds.get_max();
		return {minX: low.get_x(), minY: low.get_y(), minZ: low.get_z(), maxX: high.get_x(), maxY: high.get_y(), maxZ: high.get_z()};
	}

	/**
	 * Whether the boxes are separated by more than `margin` along some axis, so the parts inside them cannot touch.
	 *
	 * `boxOf` is CadKit's tightest box: it ignores the shapes' own tolerances and, where a shape has been meshed, follows the
	 * mesh, which on a curved surface lies slightly inside it. The margin is far larger than either, so only parts that are
	 * clearly separated skip the boolean, which is where nearly all of its cost was; parts within a millimetre of each other
	 * are always tested exactly.
	 */
	public static function apart(a:PartBox, b:PartBox, margin:Float = APART_MARGIN):Bool
		return a.minX > b.maxX + margin || b.minX > a.maxX + margin || a.minY > b.maxY + margin || b.minY > a.maxY + margin
			|| a.minZ > b.maxZ + margin || b.minZ > a.maxZ + margin;

	/**
	 * The volume the two parts share. Parts whose boxes are apart share none, which is settled without a solid boolean, the
	 * expensive part of a check across many pairs. The boxes are the parts' own (see `boxOf`), computed once by a caller that
	 * tests a part against several others.
	 */
	public static function commonVolume(first:Part, firstBox:PartBox, second:Part, secondBox:PartBox):Float {
		if (apart(firstBox, secondBox)) return 0;
		var common = first.intersect(second);
		var volume = common.volume();
		common.close();
		return volume;
	}

	public static function closeAll(parts:Array<Part>):Void
		for (part in parts) part.close();
}
