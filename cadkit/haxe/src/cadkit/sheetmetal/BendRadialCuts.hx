package cadkit.sheetmetal;

import cadkit.sheetmetal.BendRadialCuts.RadialCutEvent;

/** Exact hole-free, Y-monotone radial sectors derived from authored cuts at a collapsed hem axis. */
class BendRadialCuts {
	static function splitExtrema(outline:WallOutline):WallOutline {
		var segments:Array<Segment> = []; var identities:Array<String> = []; var loop = outline.loops()[0];
		for (i in 0...loop.length) {
			var segment = loop[i]; var identity = outline.identity(0, i); var fractions = [0.0, 1.0];
			if (!segment.isLine()) for (sign in [-1.0, 1.0]) {
				var center = segment.arcCenter(); var extreme = new Point2(center.x, center.y + sign * segment.radius);
				var fraction = segment.fractionAt(extreme); if (fraction > 1e-10 && fraction < 1 - 1e-10) fractions.push(fraction);
			}
			fractions.sort(function(a:Float, b:Float):Int { return a < b ? -1 : a > b ? 1 : 0; });
			for (j in 0...(fractions.length - 1)) {
				var first = fractions[j]; var last = fractions[j + 1];
				segments.push(segment.isLine() ? segment : Segment.arc(segment.arcCenter(), segment.radius,
					segment.startAngle + segment.sweep * first, segment.sweep * (last - first)));
				identities.push(fractions.length == 2 ? identity : "radial:" + identity.length + ":" + identity + ":"
					+ role(segment, first) + ":" + role(segment, last));
			}
		}
		return new WallOutline(segments, [], [identities]);
	}
	static function role(segment:Segment, fraction:Float):String {
		if (fraction == 0) return "start"; if (fraction == 1) return "end";
		return Math.sin(segment.startAngle + segment.sweep * fraction) < 0 ? "y:min" : "y:max";
	}

	static function events(region:WallOutline, axis:Int):Array<RadialCutEvent> {
		var result:Array<RadialCutEvent> = []; var loops = region.loops();
		for (i in 0...loops.length) for (j in 0...loops[i].length) {
			var segment = loops[i][j]; var identity = region.identity(i, j);
			result.push({value: axis == 0 ? segment.start.x : segment.start.y, identity: identity + ":start"});
			result.push({value: axis == 0 ? segment.end.x : segment.end.y, identity: identity + ":end"});
			if (!segment.isLine()) for (cardinal in 0...4) {
				var center = segment.arcCenter(); var angle = cardinal * Math.PI / 2;
				var point = new Point2(center.x + segment.radius * Math.cos(angle), center.y + segment.radius * Math.sin(angle));
				var fraction = segment.fractionAt(point);
				if (fraction >= 0 && fraction <= 1) {
					var role = cardinal == 0 ? "x:max" : cardinal == 1 ? "y:max" : cardinal == 2 ? "x:min" : "y:min";
					result.push({value: axis == 0 ? point.x : point.y, identity: identity + ":" + role});
				}
			}
		}
		result.sort(function(a:RadialCutEvent, b:RadialCutEvent):Int {
			return a.value < b.value ? -1 : a.value > b.value ? 1 : Reflect.compare(a.identity, b.identity);
		});
		var unique:Array<RadialCutEvent> = [];
		for (event in result) if (unique.length == 0 || event.value - unique[unique.length - 1].value > 1e-10) unique.push(event);
		return unique;
	}
	public static function regions(bend:Bend, allowance:Float):Array<WallOutline> {
		var result:Array<WallOutline> = []; var strip = bend.developedRegion(allowance);
		for (cut in bend.cutTools()) for (region in OutlineDifference.intersection(cut.region, strip, cut.owner)) {
			// Both coordinates partition at authored vertices and arc extrema. A tangency
			// then lands on a cell corner, avoiding a pinched intermediate planar region.
			var xs = events(region, 0); var ys = events(region, 1);
			for (i in 0...(xs.length - 1)) for (j in 0...(ys.length - 1)) {
				var left = xs[i]; var right = xs[i + 1]; var lower = ys[j]; var upper = ys[j + 1];
				var cell = new WallOutline(WallOutline.polygon([new Point2(left.value, lower.value), new Point2(right.value, lower.value),
					new Point2(right.value, upper.value), new Point2(left.value, upper.value)]).loops()[0], [],
					[["partition:y:" + lower.identity, "partition:x:" + right.identity, "partition:y:" + upper.identity, "partition:x:" + left.identity]]);
				for (part in OutlineDifference.intersection(region, cell, cut.owner)) {
					if (part.loops().length != 1) throw new SheetDiagnosis("sheet.radial-cut-partition", "Radial sector retained a hole after analytic partition", ["bend:" + bend.id]);
					result.push(splitExtrema(part));
				}
			}
		}
		return result;
	}
}

/** An authored vertex or arc extremum names each partition boundary independently of its coordinate. */
typedef RadialCutEvent = {var value:Float; var identity:String;}
