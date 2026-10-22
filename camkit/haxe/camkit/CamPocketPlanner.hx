package camkit;

import toolpathkit.path.Point3;

typedef CamPocketPass = {y:Float, left:Float, right:Float};

/** Horizontal cutter-centre passes inside a face eroded by a disk. */
class CamPocketPlanner {
	public static function plan(contour:CamContour, radius:Float, stepOver:Float, ?islands:Array<CamContour>):Array<CamPocketPass> {
		if (islands == null)
			islands = [];
		var vertices = contour.vertices;
		var minY = Math.POSITIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY;
		for (point in vertices) {
			minY = Math.min(minY, point.y);
			maxY = Math.max(maxY, point.y);
		}
		var outerRadius = clearanceRadius(contour, radius);
		var firstY = minY + outerRadius, lastY = maxY - outerRadius;
		if (firstY > lastY + 1e-10)
			throw "Tool does not fit inside CAM pocket";
		var rows:Array<Float> = [];
		var y = firstY;
		while (y <= lastY + 1e-10) {
			if (rows.length >= 10000)
				throw "CAM pocket exceeds 10000 clearing rows";
			rows.push(Math.min(y, lastY));
			y += stepOver;
		}
		if (rows.length == 0 || lastY - rows[rows.length - 1] > 1e-9)
			rows.push(lastY);
		var passes:Array<CamPocketPass> = [];
		for (row in rows) {
			var intervals = interiorIntervals(contour, row);
			for (i in 0...vertices.length) {
				var forbidden = capsuleSlice(vertices[i], vertices[(i + 1) % vertices.length], row, outerRadius);
				if (forbidden != null)
					intervals = subtract(intervals, forbidden);
			}
			for (island in islands) {
				for (blocked in interiorIntervals(island, row))
					intervals = subtract(intervals, blocked);
				var edgePoints = island.vertices, islandRadius = clearanceRadius(island, radius);
				for (i in 0...edgePoints.length) {
					var forbidden = capsuleSlice(edgePoints[i], edgePoints[(i + 1) % edgePoints.length], row, islandRadius);
					if (forbidden != null)
						intervals = subtract(intervals, forbidden);
				}
			}
			for (interval in intervals)
				if (interval.right - interval.left > 1e-8) {
					if (passes.length >= 100000)
						throw "CAM pocket exceeds 100000 clearing passes";
					passes.push({
						y: row,
						left: interval.left,
						right: interval.right
					});
				}
		}
		if (passes.length == 0)
			throw "Tool does not fit inside CAM pocket";
		return passes;
	}

	/**
		Horizontal passes across a face, one on every line `y = k * spacing` of a lattice through the origin
		that crosses it: the face's own interior, with no cutter eroding the edges. Because the lattice does
		not depend on the face, passes of neighbouring layers line up. The islands are holes.
	**/
	public static function fill(contour:CamContour, spacing:Float, ?islands:Array<CamContour>):Array<CamPocketPass> {
		if (!(spacing > 0.0) || !Math.isFinite(spacing))
			throw "CAM fill needs a positive spacing";
		var minY = Math.POSITIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY;
		for (point in contour.vertices) {
			minY = Math.min(minY, point.y);
			maxY = Math.max(maxY, point.y);
		}
		var first = Math.ceil((minY + 1e-9) / spacing),
			last = Math.floor((maxY - 1e-9) / spacing);
		if (last - first > 100000)
			throw "CAM fill exceeds 100000 rows";
		var passes:Array<CamPocketPass> = [];
		var row = first;
		while (row <= last) {
			var y = row * spacing;
			var intervals = interiorIntervals(contour, y);
			if (islands != null)
				for (island in islands)
					for (blocked in interiorIntervals(island, y))
						intervals = subtract(intervals, blocked);
			for (interval in intervals)
				if (interval.right - interval.left > 1e-8)
					passes.push({y: y, left: interval.left, right: interval.right});
			row += 1;
		}
		return passes;
	}

	/** Exact segment/edge clearance proof for a local link above a cleared layer. */
	public static function canLink(contour:CamContour, islands:Array<CamContour>, radius:Float, a:Point3, b:Point3):Bool {
		var x = (a.x + b.x) / 2, y = (a.y + b.y) / 2;
		function inside(c:CamContour):Bool {
			return interiorIntervals(c, y).filter(v -> x >= v.left - 1e-10 && x <= v.right + 1e-10).length > 0;
		}
		if (!inside(contour))
			return false;
		for (island in islands)
			if (inside(island))
				return false;
		function pointDistance(p:Point3, u:Point3, v:Point3):Float {
			var dx = v.x - u.x, dy = v.y - u.y, square = dx * dx + dy * dy;
			var t = square == 0 ? 0 : Math.max(0, Math.min(1, ((p.x - u.x) * dx + (p.y - u.y) * dy) / square));
			return Math.sqrt(Math.pow(p.x - u.x - t * dx, 2) + Math.pow(p.y - u.y - t * dy, 2));
		}
		for (c in [contour].concat(islands)) {
			var guardedRadius = clearanceRadius(c, radius);
			for (i in 0...c.vertices.length) {
				var u = c.vertices[i],
					v = c.vertices[(i + 1) % c.vertices.length];
				var denominator = (b.x - a.x) * (v.y - u.y) - (b.y - a.y) * (v.x - u.x);
				if (Math.abs(denominator) > 1e-15) {
					var t = ((u.x - a.x) * (v.y - u.y) - (u.y - a.y) * (v.x - u.x)) / denominator;
					var q = ((u.x - a.x) * (b.y - a.y) - (u.y - a.y) * (b.x - a.x)) / denominator;
					if (t >= 0 && t <= 1 && q >= 0 && q <= 1)
						return false;
				}
				if (Math.min(Math.min(pointDistance(a, u, v), pointDistance(b, u, v)), Math.min(pointDistance(u, a, b), pointDistance(v, a, b))) < guardedRadius
					- 1e-9)
					return false;
			}
		}
		return true;
	}

	/** Chordal clearing must stay on the removable side of the exact arcs.
	 * Their declared maximum sagitta is a conservative boundary clearance;
	 * the final offset pass still follows the exact finished boundary.
	 */
	static function clearanceRadius(contour:CamContour, radius:Float):Float {
		if (contour.exactBoundary == null
			|| contour.exactBoundary.filter(segment -> !segment.isLine()).length == 0) return radius;
		if (!Math.isFinite(contour.chordTolerance) || contour.chordTolerance <= 0)
			throw "CAM curved clearing needs a positive chord tolerance";
		return radius + contour.chordTolerance;
	}

	static function interiorIntervals(contour:CamContour, row:Float):Array<{left:Float, right:Float}> {
		var vertices = contour.vertices, crossings:Array<Float> = [];
		for (i in 0...vertices.length) {
			var a = vertices[i], b = vertices[(i + 1) % vertices.length];
			if ((a.y <= row && row < b.y) || (b.y <= row && row < a.y))
				crossings.push(a.x + (row - a.y) * (b.x - a.x) / (b.y - a.y));
		}
		crossings.sort((a, b) -> a < b ? -1 : (a > b ? 1 : 0));
		if (crossings.length % 2 != 0)
			throw "CAM pocket contour has unmatched scanline crossings";
		return [
			for (i in 0...Std.int(crossings.length / 2))
				{left: crossings[2 * i], right: crossings[2 * i + 1]}
		];
	}

	static function subtract(intervals:Array<{left:Float, right:Float}>, forbidden:{left:Float, right:Float}):Array<{left:Float, right:Float}> {
		var remaining:Array<{left:Float, right:Float}> = [];
		for (interval in intervals) {
			if (forbidden.right <= interval.left || forbidden.left >= interval.right) {
				remaining.push(interval);
			} else {
				if (forbidden.left - interval.left > 1e-9)
					remaining.push({left: interval.left, right: forbidden.left});
				if (interval.right - forbidden.right > 1e-9)
					remaining.push({left: forbidden.right, right: interval.right});
			}
		}
		return remaining;
	}

	/** A segment's radius-r neighbourhood has one interval on a horizontal row. */
	static function capsuleSlice(a:Point3, b:Point3, y:Float, radius:Float):Null<{left:Float, right:Float}> {
		if (y < Math.min(a.y, b.y) - radius || y > Math.max(a.y, b.y) + radius)
			return null;
		var closestX:Float;
		if (Math.abs(b.y - a.y) < 1e-14)
			closestX = (a.x + b.x) * 0.5;
		else if (y >= Math.min(a.y, b.y) && y <= Math.max(a.y, b.y))
			closestX = a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);
		else
			closestX = Math.abs(y - a.y) < Math.abs(y - b.y) ? a.x : b.x;
		if (distance(closestX, y, a, b) >= radius - 1e-12)
			return null;
		var low = Math.min(a.x, b.x) - radius, high = closestX;
		for (_ in 0...40) {
			var middle = (low + high) * 0.5;
			if (distance(middle, y, a, b) < radius)
				high = middle;
			else
				low = middle;
		}
		var left = high;
		low = closestX;
		high = Math.max(a.x, b.x) + radius;
		for (_ in 0...40) {
			var middle = (low + high) * 0.5;
			if (distance(middle, y, a, b) < radius)
				low = middle;
			else
				high = middle;
		}
		return {left: left, right: low};
	}

	static function distance(x:Float, y:Float, a:Point3, b:Point3):Float {
		var dx = b.x - a.x, dy = b.y - a.y;
		var t = Math.max(0.0, Math.min(1.0, ((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy)));
		var px = x - a.x - t * dx, py = y - a.y - t * dy;
		return Math.sqrt(px * px + py * py);
	}
}
