package cadkit.sketch;

import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.units.LengthUnits;

private class ProfileSegment {
	public final entity:SketchEntity;
	public final start:Array<Float>;
	public final end:Array<Float>;
	public final reversed:Bool;

	public function new(entity:SketchEntity, start:Array<Float>, end:Array<Float>, reversed:Bool = false) {
		this.entity = entity;
		this.start = start;
		this.end = end;
		this.reversed = reversed;
	}
}

private class ProfileBoundary {
	public final segments:Null<Array<ProfileSegment>>;
	public final circle:Null<SketchEntity>;
	public final polyline:Array<Array<Float>>;
	public final area:Float;
	public final entityIds:Array<String>;
	public var parent:Int;
	public var depth:Int;

	public function new(segments:Null<Array<ProfileSegment>>, circle:Null<SketchEntity>, polyline:Array<Array<Float>>, area:Float,
		entityIds:Array<String>) {
		this.segments = segments;
		this.circle = circle;
		this.polyline = polyline;
		this.area = area;
		this.entityIds = entityIds;
		parent = -1;
		depth = 0;
	}
}

/** Curve-aware validation and exact native construction of solved profile boundaries. */
class SketchProfile {
	public static function build(authored:ConstrainedSketch, solved:SolvedSketch):Sketch {
		var factor = LengthUnits.factorToMillimetres(authored.units);
		if (factor == null)
			throw new ProfileError("invalid", [], "unsupported sketch length unit: " + authored.units);
		var canonical = toMillimetres(authored, solved, factor);
		var scale = modelScale(authored, canonical);
		var tolerance = Math.max(authored.settings.tolerance * factor * Math.max(1, scale) * 10,
			1e-8 * Math.max(1, scale));
		var segments:Array<ProfileSegment> = [];
		var circles:Array<SketchEntity> = [];
		for (entity in authored.entities()) {
			if (entity.construction)
				continue;
			if (entity.kind == "circle") {
				circles.push(entity);
			} else if (entity.kind == "line") {
				segments.push(new ProfileSegment(entity, canonical.point(entity.first), canonical.point(entity.second)));
			} else if (entity.kind == "arc") {
				var center = canonical.point(entity.first);
				var radius = canonical.radius(entity.id);
				segments.push(new ProfileSegment(entity, arcPoint(center, radius, entity.startAngle),
					arcPoint(center, radius, entity.endAngle)));
			}
		}
		if (segments.length == 0 && circles.length == 0)
			throw new ProfileError("empty", [], "sketch has no profile geometry");

		var boundaries:Array<ProfileBoundary> = [];
		for (loop in connectedLoops(segments, tolerance))
			boundaries.push(loopBoundary(loop, canonical, tolerance));
		for (circle in circles)
			boundaries.push(circleBoundary(circle, canonical, tolerance));
		validateIntersections(boundaries, tolerance);
		classifyNesting(boundaries, tolerance);
		return constructFaces(boundaries, authored.plane, canonical);
	}

	private static function toMillimetres(authored:ConstrainedSketch, solved:SolvedSketch, factor:Float):SolvedSketch {
		var coordinates:Map<String, Array<Float>> = new Map();
		for (point in authored.points()) {
			var value = solved.point(point.id);
			coordinates.set(point.id, [value[0] * factor, value[1] * factor]);
		}
		var radii:Map<String, Float> = new Map();
		for (entity in authored.entities())
			if (entity.kind == "circle" || entity.kind == "arc")
				radii.set(entity.id, solved.radius(entity.id) * factor);
		return new SolvedSketch(coordinates, radii, solved.diagnostic);
	}

	private static function loopBoundary(loop:Array<ProfileSegment>, solved:SolvedSketch, tolerance:Float):ProfileBoundary {
		var polyline:Array<Array<Float>> = [];
		var signedArea = 0.0;
		for (segment in loop) {
			if (segment.entity.kind == "line") {
				appendPoint(polyline, segment.start, tolerance);
				signedArea += cross(segment.start, segment.end) / 2;
				continue;
			}
			var entity = segment.entity;
			var center = solved.point(entity.first);
			var radius = solved.radius(entity.id);
			var delta = arcDelta(entity);
			var startAngle = entity.startAngle;
			if (segment.reversed) {
				startAngle = entity.endAngle;
				delta = -delta;
			}
			var steps = arcSteps(radius, Math.abs(delta), tolerance);
			for (index in 0...steps)
				appendPoint(polyline, arcPoint(center, radius, startAngle + delta * index / steps), tolerance);
			signedArea += (center[0] * (segment.end[1] - segment.start[1])
				- center[1] * (segment.end[0] - segment.start[0]) + radius * radius * delta) / 2;
		}
		appendPoint(polyline, loop[loop.length - 1].end, tolerance);
		var entityIds = ids(loop);
		rejectSelfIntersection(polyline, entityIds, tolerance);
		return new ProfileBoundary(loop, null, polyline, Math.abs(signedArea), entityIds);
	}

	private static function circleBoundary(entity:SketchEntity, solved:SolvedSketch, tolerance:Float):ProfileBoundary {
		var center = solved.point(entity.first);
		var radius = solved.radius(entity.id);
		var count = arcSteps(radius, 2 * Math.PI, tolerance);
		var polyline:Array<Array<Float>> = [];
		for (index in 0...count)
			polyline.push(arcPoint(center, radius, 2 * Math.PI * index / count));
		polyline.push(polyline[0]);
		return new ProfileBoundary(null, entity, polyline, Math.PI * radius * radius, [entity.id]);
	}

	private static function classifyNesting(boundaries:Array<ProfileBoundary>, tolerance:Float):Void {
		for (index in 0...boundaries.length) {
			var sample = interiorPoint(boundaries[index].polyline);
			var best = -1;
			var bestArea = 1e300;
			for (candidate in 0...boundaries.length) {
				if (candidate == index || boundaries[candidate].area <= boundaries[index].area + tolerance * tolerance)
					continue;
				if (boundaries[candidate].area < bestArea && pointIn(boundaries[candidate].polyline, sample, tolerance)) {
					best = candidate;
					bestArea = boundaries[candidate].area;
				}
			}
			boundaries[index].parent = best;
		}
		for (index in 0...boundaries.length) {
			var depth = 0;
			var parent = boundaries[index].parent;
			var guard = 0;
			while (parent >= 0) {
				depth++;
				parent = boundaries[parent].parent;
				guard++;
				if (guard > boundaries.length)
					throw new ProfileError("ambiguous", boundaries[index].entityIds, "cyclic profile nesting");
			}
			boundaries[index].depth = depth;
		}
	}

	private static function validateIntersections(boundaries:Array<ProfileBoundary>, tolerance:Float):Void {
		for (first in 0...boundaries.length) {
			for (second in (first + 1)...boundaries.length) {
				if (polylinesIntersect(boundaries[first].polyline, boundaries[second].polyline, tolerance))
					throw new ProfileError("overlapping", boundaries[first].entityIds.concat(boundaries[second].entityIds),
						"profile boundaries overlap or touch");
			}
		}
	}

	private static function constructFaces(boundaries:Array<ProfileBoundary>, plane:Plane, solved:SolvedSketch):Sketch {
		var curves:Array<Curve> = [];
		var result:Null<Sketch> = null;
		try {
			for (boundary in boundaries)
				curves.push(makeCurve(boundary, plane, solved));
			for (index in 0...boundaries.length) {
				if (boundaries[index].depth % 2 != 0)
					continue;
				var holes:Array<Curve> = [];
				for (candidate in 0...boundaries.length)
					if (boundaries[candidate].parent == index && boundaries[candidate].depth % 2 == 1)
						holes.push(curves[candidate]);
				var face = Sketch.face(curves[index], holes, plane);
				if (result == null) {
					result = face;
				} else {
					var combined = result.combine(face);
					result.close();
					face.close();
					result = combined;
				}
			}
			if (result == null)
				throw new ProfileError("ambiguous", allIds(boundaries), "profile has no outer boundary");
			for (curve in curves)
				curve.close();
			return result;
		} catch (error:Dynamic) {
			for (curve in curves)
				curve.close();
			if (result != null)
				result.close();
			if (Std.isOfType(error, ProfileError))
				throw error;
			throw new ProfileError("invalid", allIds(boundaries), "native profile construction failed: " + Std.string(error));
		}
	}

	private static function makeCurve(boundary:ProfileBoundary, plane:Plane, solved:SolvedSketch):Curve {
		if (boundary.circle != null) {
			var entity = boundary.circle;
			var center = solved.point(entity.first);
			var localPlane = new Plane(world(plane, center), plane.xDirection, plane.normal);
			return Curve.circle(solved.radius(entity.id), localPlane);
		}
		var parts:Array<Curve> = [];
		try {
			for (segment in boundary.segments) {
				if (segment.entity.kind == "line") {
					parts.push(Curve.line(world(plane, segment.start), world(plane, segment.end)));
					continue;
				}
				var entity = segment.entity;
				var center = solved.point(entity.first);
				var radius = solved.radius(entity.id);
				var delta = arcDelta(entity);
				var startAngle = entity.startAngle;
				if (segment.reversed) {
					startAngle = entity.endAngle;
					delta = -delta;
				}
				parts.push(Curve.arc(world(plane, segment.start), world(plane, arcPoint(center, radius, startAngle + delta / 2)),
					world(plane, segment.end)));
			}
			var wire = Curve.wire(parts);
			for (part in parts)
				part.close();
			return wire;
		} catch (error:Dynamic) {
			for (part in parts)
				part.close();
			throw error;
		}
	}

	private static function connectedLoops(segments:Array<ProfileSegment>, tolerance:Float):Array<Array<ProfileSegment>> {
		var loops:Array<Array<ProfileSegment>> = [];
		var used:Array<Bool> = [];
		for (_ in segments)
			used.push(false);
		for (index in 0...segments.length) {
			if (used[index])
				continue;
			var loop:Array<ProfileSegment> = [];
			var current = index;
			var start = segments[index].start;
			var end = segments[index].end;
			while (true) {
				used[current] = true;
				loop.push(segments[current]);
				if (near(end, start, tolerance))
					break;
				var found = -1;
				var reversed = false;
				var count = 0;
				for (candidate in 0...segments.length) {
					if (used[candidate])
						continue;
					if (near(segments[candidate].start, end, tolerance)) {
						found = candidate;
						reversed = false;
						count++;
					} else if (near(segments[candidate].end, end, tolerance)) {
						found = candidate;
						reversed = true;
						count++;
					}
				}
				if (count == 0)
					throw new ProfileError("open", ids(loop), "open boundary near entity " + segments[current].entity.id);
				if (count > 1)
					throw new ProfileError("branching", ids(loop), "branching boundary near entity " + segments[current].entity.id);
				current = found;
				if (reversed) {
					var old = segments[current];
					segments[current] = new ProfileSegment(old.entity, old.end, old.start, !old.reversed);
				}
				end = segments[current].end;
			}
			loops.push(loop);
		}
		return loops;
	}

	private static function rejectSelfIntersection(polyline:Array<Array<Float>>, entityIds:Array<String>, tolerance:Float):Void {
		var count = polyline.length - 1;
		for (first in 0...count) {
			for (second in (first + 1)...count) {
				if (second == first + 1 || (first == 0 && second == count - 1))
					continue;
				if (segmentsIntersect(polyline[first], polyline[first + 1], polyline[second], polyline[second + 1], tolerance))
					throw new ProfileError("self-intersecting", entityIds, "self-intersecting profile boundary");
			}
		}
	}

	private static function polylinesIntersect(first:Array<Array<Float>>, second:Array<Array<Float>>, tolerance:Float):Bool {
		for (a in 0...(first.length - 1))
			for (b in 0...(second.length - 1))
				if (segmentsIntersect(first[a], first[a + 1], second[b], second[b + 1], tolerance))
					return true;
		return false;
	}

	private static function segmentsIntersect(a:Array<Float>, b:Array<Float>, c:Array<Float>, d:Array<Float>, tolerance:Float):Bool {
		// Most pairs in a discretized profile are spatially separate. Reject those
		// before building vectors or doing the more expensive intersection math.
		// The normalized crossing tolerance can extend a segment by tolerance *
		// its length, while the parallel test uses a distance tolerance, so account
		// for both to keep this rejection conservative.
		var abX = b[0] - a[0];
		var abY = b[1] - a[1];
		var cdX = d[0] - c[0];
		var cdY = d[1] - c[1];
		var padding = Math.max(tolerance, tolerance * Math.max(Math.max(Math.abs(abX), Math.abs(abY)),
			Math.max(Math.abs(cdX), Math.abs(cdY))));
		if (Math.max(a[0], b[0]) + padding < Math.min(c[0], d[0]) - padding
			|| Math.max(c[0], d[0]) + padding < Math.min(a[0], b[0]) - padding
			|| Math.max(a[1], b[1]) + padding < Math.min(c[1], d[1]) - padding
			|| Math.max(c[1], d[1]) + padding < Math.min(a[1], b[1]) - padding)
			return false;
		var ab = [b[0] - a[0], b[1] - a[1]];
		var cd = [d[0] - c[0], d[1] - c[1]];
		var denominator = cross(ab, cd);
		if (Math.abs(denominator) <= tolerance)
			return pointSegmentDistance(a, c, d) <= tolerance || pointSegmentDistance(b, c, d) <= tolerance
				|| pointSegmentDistance(c, a, b) <= tolerance || pointSegmentDistance(d, a, b) <= tolerance;
		var ac = [c[0] - a[0], c[1] - a[1]];
		var u = cross(ac, cd) / denominator;
		var v = cross(ac, ab) / denominator;
		return u >= -tolerance && u <= 1 + tolerance && v >= -tolerance && v <= 1 + tolerance;
	}

	private static function pointSegmentDistance(point:Array<Float>, start:Array<Float>, end:Array<Float>):Float {
		var dx = end[0] - start[0];
		var dy = end[1] - start[1];
		var lengthSquared = dx * dx + dy * dy;
		if (lengthSquared == 0)
			return distance(point, start);
		var parameter = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / lengthSquared;
		parameter = Math.max(0, Math.min(1, parameter));
		return distance(point, [start[0] + parameter * dx, start[1] + parameter * dy]);
	}

	private static function pointIn(polyline:Array<Array<Float>>, point:Array<Float>, tolerance:Float):Bool {
		for (index in 0...(polyline.length - 1))
			if (pointSegmentDistance(point, polyline[index], polyline[index + 1]) <= tolerance)
				return false;
		var inside = false;
		var previous = polyline.length - 2;
		for (index in 0...(polyline.length - 1)) {
			var current = polyline[index];
			var prior = polyline[previous];
			if ((current[1] > point[1]) != (prior[1] > point[1])
				&& point[0] < (prior[0] - current[0]) * (point[1] - current[1]) / (prior[1] - current[1]) + current[0])
				inside = !inside;
			previous = index;
		}
		return inside;
	}

	private static function interiorPoint(polyline:Array<Array<Float>>):Array<Float> {
		var x = 0.0;
		var y = 0.0;
		var count = polyline.length - 1;
		for (index in 0...count) {
			x += polyline[index][0];
			y += polyline[index][1];
		}
		var center = [x / count, y / count];
		if (pointIn(polyline, center, 0))
			return center;
		for (index in 0...count) {
			var candidate = [(polyline[index][0] + center[0]) / 2, (polyline[index][1] + center[1]) / 2];
			if (pointIn(polyline, candidate, 0))
				return candidate;
		}
		return polyline[0];
	}

	private static function modelScale(authored:ConstrainedSketch, solved:SolvedSketch):Float {
		var scale = 1.0;
		for (point in authored.points()) {
			var value = solved.point(point.id);
			scale = Math.max(scale, Math.max(Math.abs(value[0]), Math.abs(value[1])));
		}
		for (entity in authored.entities())
			if (entity.kind == "circle" || entity.kind == "arc")
				scale = Math.max(scale, solved.radius(entity.id));
		return scale;
	}

	private static function arcDelta(entity:SketchEntity):Float {
		var delta = entity.endAngle - entity.startAngle;
		if (entity.clockwise) {
			while (delta >= 0)
				delta -= 2 * Math.PI;
		} else {
			while (delta <= 0)
				delta += 2 * Math.PI;
		}
		return delta;
	}

	private static function arcSteps(radius:Float, sweep:Float, tolerance:Float):Int {
		var target = Math.max(tolerance, radius * 1e-4);
		var step = Math.pow(8 * target / Math.max(radius, 1e-12), 0.5);
		return Std.int(Math.max(8, Math.min(512, Math.ceil(sweep / Math.max(step, 0.01)))));
	}

	private static function arcPoint(center:Array<Float>, radius:Float, angle:Float):Array<Float> {
		return [center[0] + radius * Math.cos(angle), center[1] + radius * Math.sin(angle)];
	}

	private static function appendPoint(values:Array<Array<Float>>, point:Array<Float>, tolerance:Float):Void {
		if (values.length == 0 || !near(values[values.length - 1], point, tolerance))
			values.push(point);
	}

	private static function world(plane:Plane, point:Array<Float>):Vector {
		return plane.toWorld(new Vector(point[0], point[1]));
	}

	private static function near(first:Array<Float>, second:Array<Float>, tolerance:Float):Bool {
		return distance(first, second) <= tolerance;
	}

	private static function distance(first:Array<Float>, second:Array<Float>):Float {
		var x = first[0] - second[0];
		var y = first[1] - second[1];
		return Math.pow(x * x + y * y, 0.5);
	}

	private static function cross(first:Array<Float>, second:Array<Float>):Float {
		return first[0] * second[1] - first[1] * second[0];
	}

	private static function ids(loop:Array<ProfileSegment>):Array<String> {
		var result:Array<String> = [];
		for (segment in loop)
			result.push(segment.entity.id);
		return result;
	}

	private static function allIds(boundaries:Array<ProfileBoundary>):Array<String> {
		var result:Array<String> = [];
		for (boundary in boundaries)
			for (id in boundary.entityIds)
				result.push(id);
		return result;
	}
}
