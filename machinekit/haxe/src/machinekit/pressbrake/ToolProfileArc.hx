package machinekit.pressbrake;

import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;

/** Circular section arc with a conservative tangent-facet cover. Angles are radians. */
class ToolProfileArc {
	public final center:ToolProfilePoint;
	public final radius:Float;
	public final startAngle:Float;
	public final sweep:Float;

	public function new(center:ToolProfilePoint, radius:Float, startAngle:Float, sweep:Float) {
		if (center == null || !Math.isFinite(radius) || !(radius > 0)
			|| !Math.isFinite(startAngle) || !Math.isFinite(sweep) || !(Math.abs(sweep) > 0 && Math.abs(sweep) < Math.PI))
			throw "Tool contact arc needs a centre, radius and sweep below pi";
		this.center = center;
		this.radius = radius;
		this.startAngle = startAngle;
		this.sweep = sweep;
	}

	public function point(fraction:Float):ToolProfilePoint {
		var angle = startAngle + fraction * sweep;
		return new ToolProfilePoint(center.y + radius * Math.cos(angle), center.z + radius * Math.sin(angle));
	}
	/** Exact section coordinates use (Y,Z) as their two planar coordinates. */
	public function segment():Segment return Segment.arc(new Point2(center.y, center.z), radius, startAngle, sweep);

	/** Intersections of neighbouring circle tangents, excluding the exact endpoints.
	 * The radial excess is at most `maximumError` mm; no inscribed chord enters the material.
	 */
	public function tangentVertices(maximumError:Float):Array<ToolProfilePoint> {
		if (!Math.isFinite(maximumError) || !(maximumError > 0)) throw "Tool contact cover needs a positive error";
		var maximumSweep = 2 * Math.acos(radius / (radius + maximumError));
		if (!(maximumSweep > 0)) throw "Tool contact cover error is below numerical resolution";
		var count = Math.ceil(Math.abs(sweep) / maximumSweep);
		if (count > 100000) throw "Tool contact cover needs too many facets";
		var step = sweep / count;
		var outerRadius = radius / Math.cos(step / 2);
		return [for (i in 0...count) {
			var angle = startAngle + (i + 0.5) * step;
			new ToolProfilePoint(center.y + outerRadius * Math.cos(angle), center.z + outerRadius * Math.sin(angle));
		}];
	}
}
