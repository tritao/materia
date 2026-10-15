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
 * A component's geometry is built once per `geometryKey` and kept, with its box; each pose is a new part made by moving it, so
 * the expensive solid construction is not repeated. Pair checks (`overlap`, `volume`, `checkClear`) first compare the posed
 * boxes (`box`, the kept box carried to the pose) and move geometry only for pairs that may touch: moving a solid copies it
 * and measuring its tight box afterwards is an optimisation over its surfaces, and most posed pairs are far apart. The kept
 * shapes are native and belong to this object: close it when the check is done. Poses it returned are the caller's to close
 * and stay valid after it is closed.
 */
class PosedParts {
	/** Parts whose boxes are further apart than this (mm) are not tested with a boolean. */
	public static inline final APART_MARGIN = 1.0;

	final locals:Map<String, Part> = [];
	final localBoxes:Map<String, PartBox> = [];

	public function new() {}

	/** `component`'s geometry in its own frame, built on first use and kept by this object (do not close it). */
	public function local(component:MachineComponent, detail:ComponentDetail = Preview):Part {
		var key = component.geometryKey(detail), local = locals.get(key);
		if (local == null) {
			local = component.geometry(detail);
			locals.set(key, local);
		}
		return local;
	}

	/** `component`'s geometry moved to `pose`: a new owned part. */
	public function posed(component:MachineComponent, pose:AssemblyFrame, detail:ComponentDetail = Preview):Part
		return moved(local(component, detail), pose);

	/** `part` moved to `pose`: a new owned part. */
	public static function moved(part:Part, pose:AssemblyFrame):Part {
		var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
		return part.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z), new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z))));
	}

	/**
	 * A box holding `component` at `pose`, without moving its geometry: the box of its kept geometry carried to the pose. It
	 * contains the box `boxOf` measures on the posed part and is larger when the pose rotates the part, so a pair whose boxes
	 * are `apart` here is apart in fact.
	 */
	public function box(component:MachineComponent, pose:AssemblyFrame, detail:ComponentDetail = Preview):PartBox {
		var key = component.geometryKey(detail), own = localBoxes.get(key);
		if (own == null) {
			own = boxOf(local(component, detail));
			localBoxes.set(key, own);
		}
		return carried(own, pose);
	}

	/**
	 * The volume (mm³) `a` at `poseA` shares with `b` at `poseB`. Pairs whose posed boxes are `apart` share none, which is
	 * settled without moving either part.
	 */
	public function overlap(a:MachineComponent, poseA:AssemblyFrame, b:MachineComponent, poseB:AssemblyFrame,
			detail:ComponentDetail = Preview):Float {
		if (apart(box(a, poseA, detail), box(b, poseB, detail))) return 0;
		var first = posed(a, poseA, detail), second = posed(b, poseB, detail);
		var shared = sharedVolume(first, second);
		first.close();
		second.close();
		return shared;
	}

	/** The member `id` of `assembly` at its pose in `state`: a new owned part. `label` names the assembly in the error for an unknown id. */
	public function member(assembly:MachineAssembly, state:AssemblyState, label:String, id:String):Part
		return posed(component(assembly, label, id), state.worldPose(id));

	/** The volume (mm³) members `a` and `b` of `assembly` share at the poses in `state`. */
	public function volume(assembly:MachineAssembly, state:AssemblyState, label:String, a:String, b:String):Float
		return overlap(component(assembly, label, a), state.worldPose(a), component(assembly, label, b), state.worldPose(b));

	/**
	 * Throws, worded by `describe(moving, other, volume)`, for the first member of `moving` that shares more than `limit` mm³
	 * with a member of `others`, trying pairs in order. Each member is posed at most once for all its pairs, and only when a
	 * pair's boxes are not apart.
	 */
	public function checkClear(assembly:MachineAssembly, state:AssemblyState, label:String, moving:Array<String>, others:Array<String>,
			describe:(String, String, Float) -> String, limit:Float = 1e-3):Void {
		var ids = moving.concat(others), components = [for (id in ids) component(assembly, label, id)];
		var poses = [for (id in ids) state.worldPose(id)];
		var boxes = [for (index in 0...ids.length) box(components[index], poses[index])];
		var placed:Array<Null<Part>> = [for (_ in ids) null];
		function at(index:Int):Part {
			var part = placed[index];
			if (part == null) {
				part = posed(components[index], poses[index]);
				placed[index] = part;
			}
			return part;
		}
		function dispose():Void for (part in placed) if (part != null) part.close();
		try {
			for (a in 0...moving.length) for (b in moving.length...ids.length) {
				if (apart(boxes[a], boxes[b])) continue;
				var shared = sharedVolume(at(a), at(b));
				if (shared > limit) throw describe(ids[a], ids[b], shared);
			}
		} catch (error:Dynamic) {
			dispose();
			throw error;
		}
		dispose();
	}

	static function component(assembly:MachineAssembly, label:String, id:String):MachineComponent {
		for (entry in assembly.components()) if (entry.id == id) return entry.component;
		throw '$label has no member "$id"';
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
		localBoxes.clear();
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
	public static function commonVolume(first:Part, firstBox:PartBox, second:Part, secondBox:PartBox):Float
		return apart(firstBox, secondBox) ? 0 : sharedVolume(first, second);

	static function sharedVolume(first:Part, second:Part):Float {
		var common = first.intersect(second);
		var volume = common.volume();
		common.close();
		return volume;
	}

	/** The axis-aligned box holding `box` moved by the rigid `pose`: the box around its eight moved corners. */
	public static function carried(box:PartBox, pose:AssemblyFrame):PartBox {
		var corner = {x: 0.0, y: 0.0, z: 0.0};
		var minX = Math.POSITIVE_INFINITY, minY = Math.POSITIVE_INFINITY, minZ = Math.POSITIVE_INFINITY;
		var maxX = Math.NEGATIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY, maxZ = Math.NEGATIVE_INFINITY;
		for (x in [box.minX, box.maxX]) for (y in [box.minY, box.maxY]) for (z in [box.minZ, box.maxZ]) {
			AssemblyFrames.transformPointInto(pose, x, y, z, corner);
			minX = Math.min(minX, corner.x); minY = Math.min(minY, corner.y); minZ = Math.min(minZ, corner.z);
			maxX = Math.max(maxX, corner.x); maxY = Math.max(maxY, corner.y); maxZ = Math.max(maxZ, corner.z);
		}
		return {minX: minX, minY: minY, minZ: minZ, maxX: maxX, maxY: maxY, maxZ: maxZ};
	}

	public static function closeAll(parts:Array<Part>):Void
		for (part in parts) part.close();
}
