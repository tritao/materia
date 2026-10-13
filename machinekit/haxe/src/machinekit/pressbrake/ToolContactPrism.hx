package machinekit.pressbrake;

import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;

/** Exact line/arc section extruded along a finite bed interval, for contact refinement. */
class ToolContactPrism {
	public final section:WallOutline;
	public final start:Float;
	public final end:Float;
	public function new(section:WallOutline, start:Float, end:Float) {
		if (section == null || !Math.isFinite(start) || !Math.isFinite(end) || end <= start)
			throw "Contact prism needs a section and a finite positive bed interval";
		this.section = section; this.start = start; this.end = end;
	}
	public function placed(x:Float, y:Float, z:Float):ToolContactPrism {
		var loops:Array<Array<Segment>> = [];
		for (loop in section.loops()) {
			var edges:Array<Segment> = [];
			for (edge in loop) edges.push(edge.isLine()
				? Segment.line(new Point2(edge.start.x + y, edge.start.y + z), new Point2(edge.end.x + y, edge.end.y + z))
				: Segment.arc(new Point2(edge.arcCenter().x + y, edge.arcCenter().y + z), edge.radius, edge.startAngle, edge.sweep));
			loops.push(edges);
		}
		return new ToolContactPrism(new WallOutline(loops[0], loops.slice(1)), start + x, end + x);
	}
}
