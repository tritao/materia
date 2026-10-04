package cadkit.sheetmetal;

/** Exact slots into the parent material at either end of a flange attachment. */
class ReliefGeometry {
	public static function cut(outline:WallOutline, boundary:Segment, gap1:Float, gap2:Float,
		type:ReliefType, width:Float, depth:Float, owner:String):WallOutline {
		if (type == None) return outline;
		validate(boundary, gap1, gap2, type, width, depth);
		var result = outline;
		for (end in 0...2) {
			var distance = end == 0 ? gap1 : boundary.length() - gap2 - width;
			var origin = boundary.point(distance / boundary.length());
			var dx = (boundary.end.x - boundary.start.x) / boundary.length();
			var dy = (boundary.end.y - boundary.start.y) / boundary.length();
			var tool = slot(type, width, depth, origin, dx, dy);
			result = OutlineDifference.subtract(result, tool, owner + ":relief:" + end);
		}
		return result;
	}
	public static function validate(boundary:Segment, gap1:Float, gap2:Float, type:ReliefType, width:Float, depth:Float):Void {
		if (type == None) return;
		if (!Math.isFinite(width) || !Math.isFinite(depth) || width <= 0 || depth <= 0)
			throw new SheetDiagnosis("sheet.invalid-relief", "Relief width and depth must be finite and positive");
		if (type == Obround && depth < width / 2)
			throw new SheetDiagnosis("sheet.invalid-relief", "An obround relief must contain its semicircular cap");
		if (!boundary.isLine() || !Math.isFinite(gap1) || !Math.isFinite(gap2) || gap1 < 0 || gap2 < 0
			|| gap1 + gap2 + 2 * width >= boundary.length() - 1e-9)
			throw new SheetDiagnosis("sheet.invalid-relief", "Reliefs and end gaps must leave a straight bend attachment");
	}
	static function slot(type:ReliefType, width:Float, depth:Float, origin:Point2, dx:Float, dy:Float):WallOutline {
		var reach = Math.max(width, depth) + 1;
		var local:Array<Segment>;
		if (type == Rectangular) local = WallOutline.polygon([new Point2(0, -reach), new Point2(width, -reach),
			new Point2(width, depth), new Point2(0, depth)]).loops()[0];
		else {
			var radius = width / 2;
			local = [Segment.line(new Point2(0, -reach), new Point2(width, -reach)),
				Segment.line(new Point2(width, -reach), new Point2(width, depth - radius)),
				Segment.arc(new Point2(radius, depth - radius), radius, 0, Math.PI),
				Segment.line(new Point2(0, depth - radius), new Point2(0, -reach))];
		}
		var transformed:Array<Segment> = [];
		for (segment in local) {
			if (segment.isLine()) transformed.push(Segment.line(point(segment.start, origin, dx, dy), point(segment.end, origin, dx, dy)));
			else transformed.push(Segment.arc(point(segment.arcCenter(), origin, dx, dy), segment.radius,
				segment.startAngle + Math.atan2(dy, dx), segment.sweep));
		}
		return new WallOutline(transformed);
	}
	static function point(local:Point2, origin:Point2, dx:Float, dy:Float):Point2 {
		return new Point2(origin.x + dx * local.x - dy * local.y, origin.y + dy * local.x + dx * local.y);
	}
}
