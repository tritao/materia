package cadkit.sheetmetal;

import cadkit.sheetmetal.BendSeam.SeamInterval;

/** Exact seam intervals from analytic boundary intersections, with material on one side only. */
class BendSeam {
	public static function intervals(outline:WallOutline, span:Segment, materialLeft:Bool):Array<SeamInterval> {
		var unique = parameters(outline, span);
		var dx = (span.end.x - span.start.x) / span.length(); var dy = (span.end.y - span.start.y) / span.length();
		var result:Array<SeamInterval> = [];
		for (i in 0...(unique.length - 1)) {
			var midpoint = span.point((unique[i] + unique[i + 1]) / 2);
			var epsilon = Math.min(1e-6, (unique[i + 1] - unique[i]) * span.length() * 1e-4);
			var sign = materialLeft ? 1 : -1;
			var inside = new Point2(midpoint.x - dy * epsilon * sign, midpoint.y + dx * epsilon * sign);
			var outside = new Point2(midpoint.x + dy * epsilon * sign, midpoint.y - dx * epsilon * sign);
			if (outline.contains(inside) && !outline.contains(outside)) result.push({first: unique[i], last: unique[i + 1]});
		}
		return union(result);
	}
	/** Material along a developed interior fold line, including holes and notches. */
	public static function materialIntervals(outline:WallOutline, span:Segment):Array<SeamInterval> {
		var unique = parameters(outline, span); var result:Array<SeamInterval> = [];
		for (i in 0...(unique.length - 1)) if (outline.contains(span.point((unique[i] + unique[i + 1]) / 2)))
			result.push({first: unique[i], last: unique[i + 1]});
		return union(result);
	}
	static function parameters(outline:WallOutline, span:Segment):Array<Float> {
		var parameters = [0.0, 1.0];
		for (loop in outline.loops()) for (edge in loop) for (point in OutlineGeometry.intersections(span, edge)) {
			var fraction = span.fractionAt(point);
			if (fraction > 1e-10 && fraction < 1 - 1e-10) parameters.push(fraction);
		}
		parameters.sort(compare); var unique:Array<Float> = [];
		for (value in parameters) if (unique.length == 0 || value - unique[unique.length - 1] > 1e-10) unique.push(value);
		return unique;
	}
	public static function union(intervals:Array<SeamInterval>):Array<SeamInterval> {
		var sorted = intervals.copy(); sorted.sort(function(a:SeamInterval, b:SeamInterval):Int { return compare(a.first, b.first); });
		var result:Array<SeamInterval> = [];
		for (interval in sorted) {
			if (result.length == 0 || interval.first > result[result.length - 1].last + 1e-10) result.push({first: interval.first, last: interval.last});
			else { var previous = result[result.length - 1]; result[result.length - 1] = {first: previous.first, last: Math.max(previous.last, interval.last)}; }
		}
		return result;
	}
	public static function matches(a:Array<SeamInterval>, b:Array<SeamInterval>):Bool {
		if (a.length != b.length) return false;
		for (i in 0...a.length) if (Math.abs(a[i].first - b[i].first) > 1e-8 || Math.abs(a[i].last - b[i].last) > 1e-8) return false;
		return true;
	}
	static function compare(a:Float, b:Float):Int { return a < b ? -1 : a > b ? 1 : 0; }
}

/** Fractions of a nominal seam, independent of its width in millimetres. */
typedef SeamInterval = {var first:Float; var last:Float;}
