package machinekit.pressbrake.planning;

import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Plain-number axis-aligned envelope in the press frame, in mm. */
class PressEnvelope {
	public final minX:Float;
	public final minY:Float;
	public final minZ:Float;
	public final maxX:Float;
	public final maxY:Float;
	public final maxZ:Float;
	public function new(minX:Float, minY:Float, minZ:Float, maxX:Float, maxY:Float, maxZ:Float) {
		for (value in [minX, minY, minZ, maxX, maxY, maxZ]) if (!Math.isFinite(value)) throw "Press envelope must be finite";
		if (!(maxX > minX && maxY > minY && maxZ > minZ)) throw "Press envelope needs positive extents";
		this.minX = minX; this.minY = minY; this.minZ = minZ;
		this.maxX = maxX; this.maxY = maxY; this.maxZ = maxZ;
	}
	/** Read a borrowed part; the caller retains ownership. */
	public static function of(part:Part, pose:AssemblyFrame):PressEnvelope {
		var bounds = part.shape.bounds(), low = bounds.get_min(), high = bounds.get_max();
		var minX = Math.POSITIVE_INFINITY, minY = Math.POSITIVE_INFINITY, minZ = Math.POSITIVE_INFINITY;
		var maxX = Math.NEGATIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY, maxZ = Math.NEGATIVE_INFINITY;
		for (x in [low.get_x(), high.get_x()]) for (y in [low.get_y(), high.get_y()]) for (z in [low.get_z(), high.get_z()]) {
			var point = AssemblyFrames.transformPoint(pose, x, y, z);
			minX = Math.min(minX, point.x); minY = Math.min(minY, point.y); minZ = Math.min(minZ, point.z);
			maxX = Math.max(maxX, point.x); maxY = Math.max(maxY, point.y); maxZ = Math.max(maxZ, point.z);
		}
		return new PressEnvelope(minX, minY, minZ, maxX, maxY, maxZ);
	}
}
