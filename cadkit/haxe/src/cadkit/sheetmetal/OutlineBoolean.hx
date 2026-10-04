package cadkit.sheetmetal;

import cadkit.sheetmetal.OutlineBoolean.OutlineMode;

/** Analytic line/arc region booleans; callers choose whether disconnected components are valid. */
class OutlineBoolean {
	static inline var EPS:Float = 1e-8;
	public static function subtract(source:WallOutline, tool:WallOutline, owner:String, ?origin:BoundaryOrigin):WallOutline {
		var result = regions(source, tool, owner, origin);
		if (result.length == 0) throw new SheetDiagnosis("sheet.cut-empty", "Cut removes the entire wall");
		if (result.length != 1) throw new SheetDiagnosis("sheet.cut-disconnected", "Cut splits the wall into separate regions");
		return result[0];
	}
	/** Bend strips may have separate patches joined through their neighbouring walls. */
	public static function regions(source:WallOutline, tool:WallOutline, owner:String, ?origin:BoundaryOrigin):Array<WallOutline> {
		return combine(source, tool, owner, Difference, origin == null ? BoundaryOrigin.Cut(owner) : origin);
	}
	public static function intersection(source:WallOutline, tool:WallOutline, owner:String):Array<WallOutline> {
		return combine(source, tool, owner, Intersection, BoundaryOrigin.Cut(owner));
	}
	/** Preserve authored identities; contributing contours must use distinct identity namespaces. */
	public static function union(source:WallOutline, tool:WallOutline):Array<WallOutline> {
		return combine(source, tool, "", Union, BoundaryOrigin.Authored);
	}
	static function combine(source:WallOutline, tool:WallOutline, owner:String, mode:OutlineMode, origin:BoundaryOrigin):Array<WallOutline> {
		var pieces:Array<OutlinePiece> = [];
		collect(source, tool, false, owner, pieces, mode, origin);
		collect(tool, source, true, owner, pieces, mode, origin);
		var loops:Array<Array<Segment>> = [];
		var ids:Array<Array<String>> = []; var roles:Array<Array<BoundaryOrigin>> = [];
		while (pieces.length > 0) {
			var first = pieces.shift();
			if (first == null) throw new SheetDiagnosis("sheet.cut-boundary", "Cut boundary has no initial segment");
			var loop = [first.segment]; var names = [first.identity]; var origins = [first.origin];
			var end = first.segment.end;
			while (end.distance(first.segment.start) > EPS) {
				var candidate = -1;
				for (i in 0...pieces.length) if (pieces[i].segment.start.distance(end) <= EPS) {
					if (candidate >= 0) throw new SheetDiagnosis("sheet.cut-boundary", "Cut boundary branches at a tangent contact");
					candidate = i;
				}
				if (candidate < 0) throw new SheetDiagnosis("sheet.cut-boundary", "Cut boundary does not close");
				var next = pieces[candidate]; pieces.splice(candidate, 1);
				loop.push(next.segment); names.push(next.identity); origins.push(next.origin); end = next.segment.end;
			}
			loops.push(loop); ids.push(names); roles.push(origins);
		}
		var outers:Array<Int> = []; var holes:Array<Int> = [];
		for (i in 0...loops.length) {
			var area = 0.0; for (segment in loops[i]) area += segment.area();
			if (area > 1e-12) outers.push(i);
			else if (area < -1e-12) holes.push(i);
			else throw new SheetDiagnosis("sheet.cut-boundary", "Cut boundary encloses zero area");
		}
		var owners:Map<Int, Int> = [];
		for (hole in holes) {
			var selected = -1; var selectedArea = Math.POSITIVE_INFINITY;
			for (outer in outers) if (OutlineGeometry.contains(loops[outer], loops[hole][0].start)) {
				var area = 0.0; for (segment in loops[outer]) area += segment.area();
				if (area < selectedArea) { selected = outer; selectedArea = area; }
			}
			if (selected < 0) throw new SheetDiagnosis("sheet.cut-boundary", "Boolean hole has no containing outer region");
			owners.set(hole, selected);
		}
		var result:Array<WallOutline> = [];
		for (outer in outers) {
			var nested:Array<Array<Segment>> = []; var names = [ids[outer]]; var origins = [roles[outer]];
			for (hole in holes) if (owners.get(hole) == outer) { nested.push(loops[hole]); names.push(ids[hole]); origins.push(roles[hole]); }
			result.push(new WallOutline(loops[outer], nested, names, origins));
		}
		return result;
	}

	static function collect(outline:WallOutline, other:WallOutline, reverse:Bool, owner:String, result:Array<OutlinePiece>, mode:OutlineMode, origin:BoundaryOrigin):Void {
		var boundaries = outline.loops();
		for (i in 0...boundaries.length) for (j in 0...boundaries[i].length) {
			var segment = boundaries[i][j];
			var parameters = [0.0, 1.0];
			for (loop in other.loops()) for (edge in loop)
				for (point in OutlineGeometry.intersections(segment, edge)) {
					var value = segment.fractionAt(point);
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
				var leftMaterial = mode == Union ? outline.contains(left) || other.contains(left) : mode == Intersection ? outline.contains(left) && other.contains(left) : reverse ? other.contains(left) && !outline.contains(left)
					: outline.contains(left) && !other.contains(left);
				var rightMaterial = mode == Union ? outline.contains(right) || other.contains(right) : mode == Intersection ? outline.contains(right) && other.contains(right) : reverse ? other.contains(right) && !outline.contains(right)
					: outline.contains(right) && !other.contains(right);
				if (leftMaterial == rightMaterial) continue;
				append(retained, leftMaterial ? piece : reversed(piece));
			}
			var identity = outline.identity(i, j);
			if (reverse && mode != Union) identity = "cut:" + owner.length + ":" + owner + ":" + identity;
			for (k in 0...retained.length) {
				var name = retained.length == 1 ? identity : "fragment:" + identity.length + ":" + identity + ":cut:"
					+ endpointKey(segment, other, retained[k].start) + ":" + endpointKey(segment, other, retained[k].end);
				var duplicate = false;
				for (existing in result) if (same(existing.segment, retained[k])) duplicate = true;
				if (!duplicate) result.push(new OutlinePiece(retained[k], name, reverse && mode != Union ? origin : outline.origin(i, j)));
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
	static function direction(segment:Segment):Point2 {
		if (segment.isLine()) return new Point2((segment.end.x - segment.start.x) / segment.length(), (segment.end.y - segment.start.y) / segment.length());
		var angle = segment.startAngle + segment.sweep / 2; var sign = segment.sweep > 0 ? 1 : -1;
		return new Point2(-Math.sin(angle) * sign, Math.cos(angle) * sign);
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
	public final origin:BoundaryOrigin;
	public function new(segment:Segment, identity:String, origin:BoundaryOrigin) { this.segment = segment; this.identity = identity; this.origin = origin; }
}

/** Operation choice is transient geometry policy, never saved feature state. */
private enum OutlineMode { Difference; Intersection; Union; }
