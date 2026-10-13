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
		collect(source, [tool], false, owner, pieces, mode, origin);
		collect(tool, [source], true, owner, pieces, mode, origin);
		return assemble(pieces);
	}
	/** Evaluate final union membership together, avoiding temporary point-contact contours. */
	public static function unionMany(outlines:Array<WallOutline>):Array<WallOutline> {
		var pieces:Array<OutlinePiece> = [];
		for (i in 0...outlines.length) {
			var others:Array<WallOutline> = [];
			for (j in 0...outlines.length) if (i != j) others.push(outlines[j]);
			collect(outlines[i], others, false, "", pieces, Union, BoundaryOrigin.Authored);
		}
		return assemble(pieces);
	}
	static function assemble(pieces:Array<OutlinePiece>):Array<WallOutline> {
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

	static function collect(outline:WallOutline, others:Array<WallOutline>, reverse:Bool, owner:String, result:Array<OutlinePiece>, mode:OutlineMode, origin:BoundaryOrigin):Void {
		var boundaries = outline.loops();
		for (i in 0...boundaries.length) for (j in 0...boundaries[i].length) {
			var segment = boundaries[i][j];
			var parameters = [0.0, 1.0];
			for (other in others) for (loop in other.loops()) for (edge in loop)
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
				var leftMaterial = mode == Union ? outline.contains(left) || containsAny(others, left) : mode == Intersection ? outline.contains(left) && containsAny(others, left) : reverse ? containsAny(others, left) && !outline.contains(left)
					: outline.contains(left) && !containsAny(others, left);
				var rightMaterial = mode == Union ? outline.contains(right) || containsAny(others, right) : mode == Intersection ? outline.contains(right) && containsAny(others, right) : reverse ? containsAny(others, right) && !outline.contains(right)
					: outline.contains(right) && !containsAny(others, right);
				if (leftMaterial == rightMaterial) continue;
				append(retained, leftMaterial ? piece : reversed(piece));
			}
			var identity = outline.identity(i, j);
			if (reverse && mode != Union) identity = "cut:" + owner.length + ":" + owner + ":" + identity;
			for (retainedPiece in retained) for (piece in uncovered(retainedPiece, result)) {
				var name = retained.length == 1 && same(piece, retainedPiece) ? identity : "fragment:" + identity.length + ":" + identity + ":cut:"
					+ endpointKey(segment, others, piece.start) + ":" + endpointKey(segment, others, piece.end);
				result.push(new OutlinePiece(piece, name, reverse && mode != Union ? origin : outline.origin(i, j)));
			}
		}
	}
	/** Joined fragments can partly cover an already retained coincident boundary. Keep each interval once. */
	static function uncovered(source:Segment, existing:Array<OutlinePiece>):Array<Segment> {
		var remaining = [source];
		for (record in existing) {
			var boundary = record.segment;
			if (!coincident(source, boundary)) continue;
			var next:Array<Segment> = [];
			for (piece in remaining) {
				var parameters = [0.0, 1.0];
				for (point in OutlineGeometry.intersections(piece, boundary)) {
					var fraction = piece.fractionAt(point);
					if (fraction > 1e-10 && fraction < 1 - 1e-10) parameters.push(fraction);
				}
				parameters.sort(function(a:Float, b:Float):Int return a < b ? -1 : a > b ? 1 : 0);
				var unique:Array<Float> = [];
				for (fraction in parameters) if (unique.length == 0 || fraction - unique[unique.length - 1] > 1e-10) unique.push(fraction);
				for (i in 0...(unique.length - 1)) {
					var part = slice(piece, unique[i], unique[i + 1]), midpoint = part.point(0.5), covered = false;
					for (point in OutlineGeometry.intersections(part, boundary)) if (point.distance(midpoint) <= EPS) covered = true;
					if (!covered) append(next, part);
				}
			}
			remaining = next;
		}
		return remaining;
	}
	static function coincident(a:Segment, b:Segment):Bool {
		if (a.isLine() != b.isLine()) return false;
		if (!a.isLine()) return a.arcCenter().distance(b.arcCenter()) <= EPS && Math.abs(a.radius - b.radius) <= EPS;
		var dx = (a.end.x - a.start.x) / a.length(), dy = (a.end.y - a.start.y) / a.length();
		return Math.abs(dx * (b.start.y - a.start.y) - dy * (b.start.x - a.start.x)) <= EPS
			&& Math.abs(dx * (b.end.y - a.start.y) - dy * (b.end.x - a.start.x)) <= EPS;
	}
	static function containsAny(outlines:Array<WallOutline>, point:Point2):Bool {
		for (outline in outlines) if (outline.contains(point)) return true; return false;
	}
	static function endpointKey(segment:Segment, others:Array<WallOutline>, point:Point2):String {
		if (point.distance(segment.start) <= EPS) return "start";
		if (point.distance(segment.end) <= EPS) return "end";
		var keys:Array<String> = [];
		for (other in others) { var loops = other.loops();
		for (i in 0...loops.length) for (j in 0...loops[i].length) {
			var edge = loops[i][j];
			var touches = false;
			for (intersection in OutlineGeometry.intersections(segment, edge)) if (intersection.distance(point) <= EPS) touches = true;
			if (touches) {
				var identity = other.identity(i, j);
				keys.push(identity.length + ":" + identity + ":" + branch(segment, edge, point));
			}
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
