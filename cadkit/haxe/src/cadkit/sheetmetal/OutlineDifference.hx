package cadkit.sheetmetal;

/** Analytic subtraction of exact planar regions; disconnected walls are diagnosed rather than joined. */
class OutlineDifference {
	static inline var EPS:Float = 1e-8;
	public static function subtract(source:WallOutline, tool:WallOutline, owner:String):WallOutline {
		var pieces:Array<OutlinePiece> = [];
		collect(source, tool, false, owner, pieces);
		collect(tool, source, true, owner, pieces);
		var loops:Array<Array<Segment>> = [];
		var ids:Array<Array<String>> = [];
		while (pieces.length > 0) {
			var first = pieces.shift();
			if (first == null) throw new SheetDiagnosis("sheet.cut-boundary", "Cut boundary has no initial segment");
			var loop = [first.segment]; var names = [first.identity];
			var end = first.segment.end;
			while (end.distance(first.segment.start) > EPS) {
				var candidate = -1;
				for (i in 0...pieces.length) if (pieces[i].segment.start.distance(end) <= EPS) {
					if (candidate >= 0) throw new SheetDiagnosis("sheet.cut-boundary", "Cut boundary branches at a tangent contact");
					candidate = i;
				}
				if (candidate < 0) throw new SheetDiagnosis("sheet.cut-boundary", "Cut boundary does not close");
				var next = pieces[candidate]; pieces.splice(candidate, 1);
				loop.push(next.segment); names.push(next.identity); end = next.segment.end;
			}
			loops.push(loop); ids.push(names);
		}
		var outer = -1;
		for (i in 0...loops.length) {
			var area = 0.0; for (segment in loops[i]) area += segment.area();
			if (area > 1e-12) {
				if (outer >= 0) throw new SheetDiagnosis("sheet.cut-disconnected", "Cut splits the wall into separate regions");
				outer = i;
			}
		}
		if (outer < 0) throw new SheetDiagnosis("sheet.cut-empty", "Cut removes the entire wall");
		var boundary = loops[outer]; var boundaryIds = ids[outer];
		loops.splice(outer, 1); ids.splice(outer, 1); ids.unshift(boundaryIds);
		return new WallOutline(boundary, loops, ids);
	}
	static function collect(outline:WallOutline, other:WallOutline, reverse:Bool, owner:String, result:Array<OutlinePiece>):Void {
		var boundaries = outline.loops();
		for (i in 0...boundaries.length) for (j in 0...boundaries[i].length) {
			var segment = boundaries[i][j];
			var parameters = [0.0, 1.0];
			for (loop in other.loops()) for (edge in loop)
				for (point in OutlineGeometry.intersections(segment, edge)) {
					var value = fraction(segment, point);
					if (value > 1e-10 && value < 1 - 1e-10) parameters.push(value);
				}
			parameters.sort(function(a:Float, b:Float):Int { return a < b ? -1 : a > b ? 1 : 0; });
			var unique:Array<Float> = [];
			for (value in parameters) if (unique.length == 0 || value - unique[unique.length - 1] > 1e-10) unique.push(value);
			var retained:Array<Segment> = [];
			for (k in 0...(unique.length - 1)) {
				var piece = slice(segment, unique[k], unique[k + 1]);
				var midpoint = piece.point(0.5);
				// Check both sides to handle coincident boundaries without keeping duplicate edges.
				var tangent = direction(piece);
				var distance = Math.min(1e-6, piece.length() * 1e-4);
				var left = new Point2(midpoint.x - tangent.y * distance, midpoint.y + tangent.x * distance);
				var right = new Point2(midpoint.x + tangent.y * distance, midpoint.y - tangent.x * distance);
				var leftMaterial = reverse ? contains(other, left) && !contains(outline, left)
					: contains(outline, left) && !contains(other, left);
				var rightMaterial = reverse ? contains(other, right) && !contains(outline, right)
					: contains(outline, right) && !contains(other, right);
				if (leftMaterial == rightMaterial) continue;
				append(retained, leftMaterial ? piece : reversed(piece));
			}
			var identity = outline.identity(i, j);
			if (reverse) identity = "cut:" + owner.length + ":" + owner + ":" + identity;
			for (k in 0...retained.length) {
				var name = retained.length == 1 ? identity : "fragment:" + identity.length + ":" + identity + ":cut:"
					+ endpointKey(segment, other, retained[k].start) + ":" + endpointKey(segment, other, retained[k].end);
				var duplicate = false;
				for (existing in result) if (same(existing.segment, retained[k])) duplicate = true;
				if (!duplicate) result.push(new OutlinePiece(retained[k], name));
			}
		}
	}
	static function endpointKey(segment:Segment, other:WallOutline, point:Point2):String {
		if (point.distance(segment.start) <= EPS) return "start";
		if (point.distance(segment.end) <= EPS) return "end";
		var keys:Array<String> = []; var loops = other.loops();
		for (i in 0...loops.length) for (j in 0...loops[i].length) {
			var edge = loops[i][j];
			var touches = false;
			for (intersection in OutlineGeometry.intersections(segment, edge)) if (intersection.distance(point) <= EPS) touches = true;
			if (touches) {
				var identity = other.identity(i, j);
				keys.push(identity.length + ":" + identity + ":" + branch(segment, edge, point));
			}
		}
		if (keys.length == 0) throw new SheetDiagnosis("sheet.cut-boundary", "Cut fragment endpoint has no authored intersection");
		keys.sort(function(a:String, b:String):Int { return Reflect.compare(a, b); });
		return keys.join("|");
	}
	/** Infinite-curve branches do not renumber when another intersection leaves the finite segment. */
	static function branch(a:Segment, b:Segment, point:Point2):String {
		if (a.isLine() && b.isLine()) return "line";
		var sign:Float;
		if (a.isLine() || b.isLine()) {
			var line = a.isLine() ? a : b; var circle = a.isLine() ? b : a; var center = circle.arcCenter();
			sign = (point.x - center.x) * (line.end.x - line.start.x) + (point.y - center.y) * (line.end.y - line.start.y);
		} else {
			var ca = a.arcCenter(); var cb = b.arcCenter();
			sign = (cb.x - ca.x) * (point.y - ca.y) - (cb.y - ca.y) * (point.x - ca.x);
		}
		return Math.abs(sign) <= EPS ? "tangent" : sign < 0 ? "negative" : "positive";
	}
	static function append(pieces:Array<Segment>, piece:Segment):Void {
		if (pieces.length == 0) { pieces.push(piece); return; }
		var previous = pieces[pieces.length - 1];
		if (previous.end.distance(piece.start) <= EPS) pieces[pieces.length - 1] = joined(previous, piece);
		else if (piece.end.distance(previous.start) <= EPS) pieces[pieces.length - 1] = joined(piece, previous);
		else pieces.push(piece);
	}
	static function joined(a:Segment, b:Segment):Segment {
		return a.isLine() ? Segment.line(a.start, b.end)
			: Segment.arc(a.arcCenter(), a.radius, a.startAngle, Math.max(-2 * Math.PI, Math.min(2 * Math.PI, a.sweep + b.sweep)));
	}
	static function same(a:Segment, b:Segment):Bool {
		return a.start.distance(b.start) <= EPS && a.end.distance(b.end) <= EPS && a.point(0.5).distance(b.point(0.5)) <= EPS;
	}
	static function contains(outline:WallOutline, point:Point2):Bool {
		var loops = outline.loops();
		if (!OutlineGeometry.contains(loops[0], point)) return false;
		for (i in 1...loops.length) if (OutlineGeometry.contains(loops[i], point)) return false;
		return true;
	}
	static function direction(segment:Segment):Point2 {
		if (segment.isLine()) return new Point2((segment.end.x - segment.start.x) / segment.length(), (segment.end.y - segment.start.y) / segment.length());
		var angle = segment.startAngle + segment.sweep / 2; var sign = segment.sweep > 0 ? 1 : -1;
		return new Point2(-Math.sin(angle) * sign, Math.cos(angle) * sign);
	}
	static function fraction(segment:Segment, point:Point2):Float {
		if (segment.isLine()) {
			var dx = segment.end.x - segment.start.x; var dy = segment.end.y - segment.start.y;
			return ((point.x - segment.start.x) * dx + (point.y - segment.start.y) * dy) / (dx * dx + dy * dy);
		}
		var center = segment.arcCenter();
		var angle = Math.atan2(point.y - center.y, point.x - center.x);
		var delta = (segment.sweep > 0 ? angle - segment.startAngle : segment.startAngle - angle) % (2 * Math.PI);
		if (delta < 0) delta += 2 * Math.PI;
		if (delta > 2 * Math.PI - 1e-10) delta = 0;
		return delta / Math.abs(segment.sweep);
	}
	static function slice(segment:Segment, start:Float, end:Float):Segment {
		return segment.isLine() ? Segment.line(segment.point(start), segment.point(end))
			: Segment.arc(segment.arcCenter(), segment.radius, segment.startAngle + segment.sweep * start, segment.sweep * (end - start));
	}
	static function reversed(segment:Segment):Segment {
		return segment.isLine() ? Segment.line(segment.end, segment.start)
			: Segment.arc(segment.arcCenter(), segment.radius, segment.startAngle + segment.sweep, -segment.sweep);
	}
}

private class OutlinePiece {
	public final segment:Segment;
	public final identity:String;
	public function new(segment:Segment, identity:String) { this.segment = segment; this.identity = identity; }
}
