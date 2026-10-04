package cadkit.sheetmetal;

/** Exact boundary edits retain authored identities and let outline validation reject crossings or lost holes. */
class WallOutlineEditor {
	/** Move one straight supporting edge, positive outward, and intersect its two neighbours analytically. */
	public static function offset(outline:WallOutline, edge:WallEdge, distance:Float):WallOutline {
		if (!Math.isFinite(distance))
			throw new SheetDiagnosis("sheet.flange-position", "Edge offset must be finite");
		var selected = outline.segment(edge.loop, edge.segment);
		if (!selected.isLine())
			throw new SheetDiagnosis("sheet.flange-position", "Only a straight edge can be offset for a bend");
		if (distance == 0) return outline;
		var loops = outline.loops();
		var loop = loops[edge.loop];
		if (loop.length < 2)
			throw new SheetDiagnosis("sheet.flange-position", "Offset edge needs two neighbouring boundaries");
		var previous = (edge.segment + loop.length - 1) % loop.length;
		var next = (edge.segment + 1) % loop.length;
		var length = selected.length();
		var tx = (selected.end.x - selected.start.x) / length;
		var ty = (selected.end.y - selected.start.y) / length;
		var origin = new Point2(selected.start.x + distance * ty, selected.start.y - distance * tx);
		var start = crossing(loop[previous], origin, tx, ty, selected.start);
		var end = crossing(loop[next], origin, tx, ty, selected.end);
		if ((end.x - start.x) * tx + (end.y - start.y) * ty <= 1e-9)
			throw new SheetDiagnosis("sheet.flange-position", "Offset consumes or reverses the selected edge");
		loop[previous] = replaceEnd(loop[previous], start);
		loop[next] = replaceStart(loop[next], end);
		loop[edge.segment] = Segment.line(start, end);
		return new WallOutline(loops[0], loops.slice(1), outline.identities());
	}
	static function crossing(neighbour:Segment, origin:Point2, tx:Float, ty:Float, old:Point2):Point2 {
		if (neighbour.isLine()) {
			var dx = neighbour.end.x - neighbour.start.x;
			var dy = neighbour.end.y - neighbour.start.y;
			var determinant = tx * dy - ty * dx;
			if (Math.abs(determinant) <= 1e-12 * neighbour.length())
				throw new SheetDiagnosis("sheet.flange-position", "Offset edge is parallel to its neighbour");
			var px = neighbour.start.x - origin.x;
			var py = neighbour.start.y - origin.y;
			var along = (px * dy - py * dx) / determinant;
			return new Point2(origin.x + along * tx, origin.y + along * ty);
		}
		var center:Point2 = cast neighbour.center;
		var projection = (center.x - origin.x) * tx + (center.y - origin.y) * ty;
		var nearest = new Point2(origin.x + projection * tx, origin.y + projection * ty);
		var separation = nearest.distance(center);
		var square = neighbour.radius * neighbour.radius - separation * separation;
		if (square < -1e-10)
			throw new SheetDiagnosis("sheet.flange-position", "Offset line does not intersect its circular neighbour");
		var delta = Math.sqrt(Math.max(0, square));
		var a = new Point2(nearest.x + delta * tx, nearest.y + delta * ty);
		var b = new Point2(nearest.x - delta * tx, nearest.y - delta * ty);
		if (a.distance(b) < 1e-9) return a;
		var difference = a.distance(old) - b.distance(old);
		if (Math.abs(difference) < 1e-9)
			throw new SheetDiagnosis("sheet.flange-position", "Two circular intersections are equally close to the authored corner");
		return difference < 0 ? a : b;
	}
	static function replaceStart(segment:Segment, point:Point2):Segment {
		if (segment.isLine()) return Segment.line(point, segment.end);
		var center:Point2 = cast segment.center;
		var start = Math.atan2(point.y - center.y, point.x - center.x);
		return Segment.arc(center, segment.radius, start, directed(segment.startAngle + segment.sweep - start, segment.sweep));
	}
	static function replaceEnd(segment:Segment, point:Point2):Segment {
		if (segment.isLine()) return Segment.line(segment.start, point);
		var center:Point2 = cast segment.center;
		var end = Math.atan2(point.y - center.y, point.x - center.x);
		return Segment.arc(center, segment.radius, segment.startAngle, directed(end - segment.startAngle, segment.sweep));
	}
	static function directed(delta:Float, previous:Float):Float {
		var sign = previous > 0 ? 1.0 : -1.0;
		var magnitude = (sign * delta) % (2 * Math.PI);
		if (magnitude < 0) magnitude += 2 * Math.PI;
		return sign * magnitude;
	}
}
