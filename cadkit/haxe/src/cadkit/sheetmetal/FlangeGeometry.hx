package cadkit.sheetmetal;

/** Exact parent attachment and child outline; end gaps and extensions do not alter bend width implicitly. */
class FlangeGeometry {
	public static function span(outline:WallOutline, selected:WallEdge, gap1:Float, gap2:Float):FlangeBoundary {
		nonnegative(gap1, "gap1"); nonnegative(gap2, "gap2");
		var edge = outline.segment(selected.loop, selected.segment);
		if (!edge.isLine())
			throw new SheetDiagnosis("sheet.bend-edge-mismatch", "A flange span requires a straight edge");
		var length = edge.length();
		if (gap1 + gap2 >= length - 1e-9)
			throw new SheetDiagnosis("sheet.invalid-flange-gap", "End gaps consume the entire bend edge");
		if (gap1 == 0 && gap2 == 0) return new FlangeBoundary(outline, selected);
		var loops = outline.loops();
		var identities = outline.identities();
		var id = outline.identity(selected.loop, selected.segment);
		var first = edge.point(gap1 / length);
		var last = edge.point(1 - gap2 / length);
		var replacement:Array<Segment> = [];
		var replacementIds:Array<String> = [];
		if (gap1 > 0) {
			replacement.push(Segment.line(edge.start, first));
			replacementIds.push("fragment:" + id.length + ":" + id + ":gap:1");
		}
		var attachment = selected.segment + replacement.length;
		replacement.push(Segment.line(first, last)); replacementIds.push(id);
		if (gap2 > 0) {
			replacement.push(Segment.line(last, edge.end));
			replacementIds.push("fragment:" + id.length + ":" + id + ":gap:2");
		}
		var boundary:Array<Segment> = [];
		var boundaryIds:Array<String> = [];
		for (i in 0...loops[selected.loop].length) {
			if (i == selected.segment) {
				for (segment in replacement) boundary.push(segment);
				for (identity in replacementIds) boundaryIds.push(identity);
			} else { boundary.push(loops[selected.loop][i]); boundaryIds.push(identities[selected.loop][i]); }
		}
		loops[selected.loop] = boundary; identities[selected.loop] = boundaryIds;
		return new FlangeBoundary(new WallOutline(loops[0], loops.slice(1), identities), new WallEdge(attachment, selected.loop));
	}
	public static function wall(leg:Float, width:Float, extend1:Float, extend2:Float):FlangeBoundary {
		nonnegative(extend1, "extend1"); nonnegative(extend2, "extend2");
		var points = [new Point2(0, -extend1), new Point2(leg, -extend1),
			new Point2(leg, width + extend2), new Point2(0, width + extend2)];
		var ids = ["flange:start", "flange:tip", "flange:end"];
		if (extend2 > 0) { points.push(new Point2(0, width)); ids.push("flange:extend:2"); }
		var attachment = ids.length;
		ids.push("flange:join");
		if (extend1 > 0) { points.push(new Point2(0, 0)); ids.push("flange:extend:1"); }
		var segments:Array<Segment> = [];
		for (i in 0...points.length) segments.push(Segment.line(points[i], points[(i + 1) % points.length]));
		return new FlangeBoundary(new WallOutline(segments, [], [ids]), new WallEdge(attachment));
	}
	static function nonnegative(value:Float, name:String):Void {
		if (!Math.isFinite(value) || value < 0)
			throw new SheetDiagnosis("sheet.invalid-flange-dimension", name + " must be finite and nonnegative");
	}
}

/** An exact wall outline and its directed connection segment. */
class FlangeBoundary {
	public final outline:WallOutline;
	public final edge:WallEdge;
	public function new(outline:WallOutline, edge:WallEdge) {
		this.outline = outline;
		this.edge = edge;
	}
}
