package cadkit.sheetmetal;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

/** Exact supporting-edge offsets account for both flange thicknesses in their folded planes. */
class CornerGeometry {
	public final first:Float;
	public final second:Float;
	function new(first:Float, second:Float) { this.first = first; this.second = second; }

	/** Gap is the perpendicular stand-off from the mate's slab; overlap extends past its far face. */
	public static function of(firstOutline:WallOutline, firstEdge:WallEdge, firstPlane:Plane,
		secondOutline:WallOutline, secondEdge:WallEdge, secondPlane:Plane, thickness:Float,
		join:CornerJoin, gap:Float, overlap:Float = 0):CornerGeometry {
		if (!Math.isFinite(thickness) || thickness <= 0)
			throw new SheetDiagnosis("sheet.invalid-thickness", "Corner closure requires positive finite stock thickness");
		if (!Math.isFinite(gap) || gap < 0 || !Math.isFinite(overlap) || overlap < 0
			|| (join == Gap && overlap != 0) || (join != Gap && overlap == 0))
			throw new SheetDiagnosis("sheet.invalid-corner-dimension", "Corner gap must be nonnegative; an overlap join requires positive overlap");
		var axis = firstPlane.normal.cross(secondPlane.normal);
		if (axis.length() < 1e-8)
			throw new SheetDiagnosis("sheet.corner-planes", "Corner flanges must have intersecting folded planes");
		axis = axis.normalized();
		return new CornerGeometry(offset(firstOutline, firstEdge, firstPlane, secondPlane, axis,
			thickness, gap, join == OverlapFirst, overlap),
			offset(secondOutline, secondEdge, secondPlane, firstPlane, axis,
				thickness, gap, join == OverlapSecond, overlap));
	}
	static function offset(outline:WallOutline, edge:WallEdge, plane:Plane, mate:Plane, axis:Vector,
		thickness:Float, gap:Float, extendsMate:Bool, overlap:Float):Float {
		var segment = outline.segment(edge.loop, edge.segment);
		if (edge.loop != 0 || !segment.isLine())
			throw new SheetDiagnosis("sheet.corner-boundary", "Corner closure requires a straight outer boundary", [outline.identity(edge.loop, edge.segment)]);
		var length = segment.length();
		var tangent = plane.xDirection.scale((segment.end.x - segment.start.x) / length)
			.add(plane.yDirection.scale((segment.end.y - segment.start.y) / length));
		if (tangent.cross(axis).length() > 1e-8)
			throw new SheetDiagnosis("sheet.corner-boundary", "Corner boundary must run along the folded planes' intersection", [outline.identity(edge.loop, edge.segment)]);
		var outward = tangent.cross(plane.normal);
		var projection = outward.dot(mate.normal);
		var sign = projection > 0 ? 1.0 : -1.0;
		var normal = mate.normal.scale(sign);
		var mateDepth = -thickness * sign;
		var ownDepth = -thickness * plane.normal.dot(normal);
		var target = extendsMate ? Math.max(0, mateDepth) + overlap - Math.min(0, ownDepth)
			: Math.min(0, mateDepth) - gap - Math.max(0, ownDepth);
		var midpoint = segment.point(0.5);
		var current = plane.toWorld(new Vector(midpoint.x, midpoint.y, 0)).subtract(mate.origin).dot(normal);
		return (target - current) / Math.abs(projection);
	}
}
