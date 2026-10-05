package machinekit.pressbrake.planning;

import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Bend;
import cadkit.sheetmetal.BendZone;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.WallOutline;

/** Exact bend material projected through a bed interval when its axis is parallel to the press X axis. */
class PressBendSection {
	/** Null means the axis needs the general prism query; an empty array means no material in this interval. */
	public static function of(record:Bend, zone:BendZone, placement:Location, first:Float, last:Float):Null<Array<WallOutline>> {
		var frame = placement.compose(zone.frame), axis = frame.direction(new Vector(0, 1, 0));
		if (Math.abs(axis.y) > 1e-12 || Math.abs(axis.z) > 1e-12) return null;
		var start = placement.plane.toWorld(zone.point(0, 0));
		var a = (first - start.x) / axis.x, b = (last - start.x) / axis.x;
		var low = Math.max(0, Math.min(a, b)), high = Math.min(zone.width, Math.max(a, b));
		if (high - low <= 1e-8) return [];
		var result:Array<WallOutline> = [], query = new PressSectionQuery();
		for (region in record.materialRegions(zone.allowance)) {
			var intervals = record.hasCuts() ? query.projection(region, low, high) : [0.0, zone.allowance];
			for (i in 0...Std.int(intervals.length / 2))
				result.push(cap(zone, placement, intervals[2 * i] / zone.allowance, intervals[2 * i + 1] / zone.allowance));
		}
		return result;
	}
	static function cap(zone:BendZone, placement:Location, first:Float, last:Float):WallOutline {
		var inner = zone.angle >= 0 ? 0.0 : zone.thickness, outer = zone.thickness - inner;
		var edges:Array<Segment> = [];
		var a = point(zone, placement, first, inner), b = point(zone, placement, last, inner);
		var c = point(zone, placement, last, outer), d = point(zone, placement, first, outer);
		if (zone.angle == 0) edges = [Segment.line(a, b), Segment.line(b, c), Segment.line(c, d), Segment.line(d, a)];
		else {
			var center3 = placement.plane.toWorld(zone.axis()), center = new Point2(center3.y, center3.z);
			var frame = placement.compose(zone.frame), x = frame.direction(new Vector(1, 0, 0)), z = frame.direction(new Vector(0, 0, 1));
			var sweep = zone.angle * (x.y * z.z - x.z * z.y) * (last - first);
			if (zone.curvedInnerRadius() > 0)
				edges.push(Segment.arc(center, zone.curvedInnerRadius(), Math.atan2(a.y - center.y, a.x - center.x), sweep));
			edges.push(Segment.line(b, c));
			edges.push(Segment.arc(center, zone.curvedInnerRadius() + zone.thickness, Math.atan2(c.y - center.y, c.x - center.x), -sweep));
			edges.push(Segment.line(d, a));
		}
		var area = 0.0;
		for (edge in edges) area += edge.area();
		if (area < 0) {
			var reversed:Array<Segment> = [];
			for (i in 0...edges.length) {
				var edge = edges[edges.length - 1 - i];
				reversed.push(edge.isLine() ? Segment.line(edge.end, edge.start)
					: Segment.arc(edge.arcCenter(), edge.radius, edge.startAngle + edge.sweep, -edge.sweep));
			}
			edges = reversed;
		}
		return new WallOutline(edges);
	}
	static function point(zone:BendZone, placement:Location, fraction:Float, depth:Float):Point2 {
		var p = placement.plane.toWorld(zone.point(fraction, 0, depth)); return new Point2(p.y, p.z);
	}
}
