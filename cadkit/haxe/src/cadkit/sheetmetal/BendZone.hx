package cadkit.sheetmetal;

import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

/** Evaluated bend strip. Its neutral length is fixed; its intermediate radius is derived from angle. */
class BendZone {
	public final id:Int;
	public final frame:Location;
	public final angle:Float;
	public final allowance:Float;
	public final width:Float;
	public final thickness:Float;
	public final innerRadius:Null<Float>;
	public final neutralRadius:Null<Float>;
	final sideRadius:Float;

	public function new(id:Int, frame:Location, angle:Float, allowance:Float, width:Float, thickness:Float,
			neutralOffset:Float) {
		this.id = id;
		this.frame = frame;
		this.angle = angle;
		this.allowance = allowance;
		this.width = width;
		this.thickness = thickness;
		if (angle == 0) {
			neutralRadius = null;
			innerRadius = null;
			sideRadius = 0;
		} else {
			var q = allowance / Math.abs(angle);
			neutralRadius = q;
			innerRadius = q - neutralOffset;
			sideRadius = q + (angle > 0 ? -neutralOffset : thickness - neutralOffset);
		}
	}

	/** Strip point: fraction along the arc, distance along the bend edge, depth from side A. */
	public function point(fraction:Float, along:Float = 0, depth:Float = 0):Vector {
		if (!Math.isFinite(fraction) || fraction < 0 || fraction > 1 || !Math.isFinite(along)
			|| along < 0 || along > width || !Math.isFinite(depth) || depth < 0 || depth > thickness)
			throw new SheetDiagnosis("sheet.invalid-bend-point", "Strip coordinates lie outside the bend", ["bend:" + id]);
		if (angle == 0)
			return frame.plane.toWorld(new Vector(allowance * fraction, along, -depth));
		var sign = angle > 0 ? 1.0 : -1.0;
		var a = Math.abs(angle) * fraction;
		var r = sideRadius + sign * depth;
		// 1-cos(a) loses precision near flat; this equivalent expression has the correct limit.
		var oneMinusCos = 2 * Math.sin(a / 2) * Math.sin(a / 2);
		return frame.plane.toWorld(new Vector(r * Math.sin(a), along, sign * r * oneMinusCos - depth));
	}

	/** Child attachment plane at the strip's far edge. */
	public function endFrame():Location {
		var x = new Vector(Math.cos(angle), 0, Math.sin(angle));
		var z = new Vector(-Math.sin(angle), 0, Math.cos(angle));
		return new Location(new Plane(point(1), frame.direction(x), frame.direction(z)));
	}

	public function axis():Vector {
		if (angle == 0)
			throw new SheetDiagnosis("sheet.flat-bend-axis", "A flat strip has no finite cylinder axis", ["bend:" + id]);
		return frame.plane.toWorld(new Vector(0, 0, angle > 0 ? sideRadius : -sideRadius));
	}
}
