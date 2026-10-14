package materia.drawing;

import haxe.io.Bytes;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.ThreadRenderData;
import materia.project.ThreadRenderData.ThreadRenderDataValues;
import materia.project.ThreadSpecData.ThreadProfile;

/** Orthographic view inputs use the part's model units; direction points toward the viewer. */
typedef DrawingViewSpec = {
	var id:String;
	var direction:Array<Float>;
	var up:Array<Float>;
	@:optional var showHiddenLines:Bool;
	@:optional var section:DrawingSectionSpec;
}

/** A section keeps the selected half-space of the source part. */
typedef DrawingSectionSpec = {
	var origin:Array<Float>;
	var normal:Array<Float>;
	var keepPositive:Bool;
}

/** Projected drawing primitives are in millimetres, with +Y upward. */
typedef DrawingLine = {
	var x1:Float;
	var y1:Float;
	var x2:Float;
	var y2:Float;
	var style:String;
	@:optional var faceName:String;
}

typedef DrawingCallout = {
	var x:Float;
	var y:Float;
	var leaderX:Float;
	var leaderY:Float;
	var text:String;
	var faceName:String;
}

typedef DrawingBounds = {var minX:Float; var minY:Float; var maxX:Float; var maxY:Float;}

typedef ProjectedDrawingView = {
	var partId:String;
	var viewId:String;
	var lengthUnit:String;
	var lines:Array<DrawingLine>;
	var callouts:Array<DrawingCallout>;
	var bounds:DrawingBounds;
}

private typedef Point3 = {var x:Float; var y:Float; var z:Float;}
private typedef Point2 = {var x:Float; var y:Float; var depth:Float;}
private typedef Segment3 = {var a:Point3; var b:Point3;}

private class ProjectedTriangle {
	public final x0:Float;
	public final y0:Float;
	public final d0:Float;
	public final x1:Float;
	public final y1:Float;
	public final d1:Float;
	public final x2:Float;
	public final y2:Float;
	public final d2:Float;
	public final minX:Float;
	public final maxX:Float;
	public final minY:Float;
	public final maxY:Float;
	public final centreX:Float;
	public final centreY:Float;

	public function new(a:Point2, b:Point2, c:Point2) {
		x0 = a.x; y0 = a.y; d0 = a.depth;
		x1 = b.x; y1 = b.y; d1 = b.depth;
		x2 = c.x; y2 = c.y; d2 = c.depth;
		minX = Math.min(x0, Math.min(x1, x2)); maxX = Math.max(x0, Math.max(x1, x2));
		minY = Math.min(y0, Math.min(y1, y2)); maxY = Math.max(y0, Math.max(y1, y2));
		centreX = (x0 + x1 + x2) / 3; centreY = (y0 + y1 + y2) / 3;
	}

	public function depthAt(x:Float, y:Float):Null<Float> {
		var denominator = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
		if (Math.abs(denominator) < 1e-14) return null;
		var a = ((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) / denominator;
		var b = ((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) / denominator;
		var c = 1 - a - b;
		if (a < -1e-8 || b < -1e-8 || c < -1e-8) return null;
		return a * d0 + b * d1 + c * d2;
	}
}

private class DrawingOcclusionNode {
	public final minX:Float;
	public final maxX:Float;
	public final minY:Float;
	public final maxY:Float;
	public final triangles:Null<Array<ProjectedTriangle>>;
	public final left:Null<DrawingOcclusionNode>;
	public final right:Null<DrawingOcclusionNode>;

	public function new(values:Array<ProjectedTriangle>) {
		var loX = Math.POSITIVE_INFINITY, hiX = Math.NEGATIVE_INFINITY;
		var loY = Math.POSITIVE_INFINITY, hiY = Math.NEGATIVE_INFINITY;
		for (triangle in values) {
			loX = Math.min(loX, triangle.minX); hiX = Math.max(hiX, triangle.maxX);
			loY = Math.min(loY, triangle.minY); hiY = Math.max(hiY, triangle.maxY);
		}
		minX = loX; maxX = hiX; minY = loY; maxY = hiY;
		if (values.length <= 8) {
			triangles = values; left = null; right = null;
		} else {
			triangles = null;
			var splitX = hiX - loX >= hiY - loY;
			values.sort(function(a, b) {
				var first = splitX ? a.centreX : a.centreY, second = splitX ? b.centreX : b.centreY;
				return first < second ? -1 : first > second ? 1 : 0;
			});
			var middle = Std.int(values.length / 2);
			left = new DrawingOcclusionNode(values.slice(0, middle));
			right = new DrawingOcclusionNode(values.slice(middle));
		}
	}

	public function collectCandidates(minX:Float, maxX:Float, minY:Float, maxY:Float,
		output:Array<ProjectedTriangle>):Void {
		if (maxX < this.minX || minX > this.maxX || maxY < this.minY || minY > this.maxY) return;
		var leaf = triangles;
		if (leaf != null) {
			for (triangle in leaf) if (!(maxX < triangle.minX || minX > triangle.maxX ||
				maxY < triangle.minY || minY > triangle.maxY)) output.push(triangle);
			return;
		}
		var first = left, second = right;
		if (first != null) first.collectCandidates(minX, maxX, minY, maxY, output);
		if (second != null) second.collectCandidates(minX, maxX, minY, maxY, output);
	}
}

/** Builds vector orthographic views from the portable CAD mesh and its named thread packets. */
class DrawingProjection {
	static inline var MAX_EDGE_SEGMENTS:Int = 2000000;
	static inline var THREAD_CIRCLE_SEGMENTS:Int = 64;
	static inline var SECTION_HATCH_SPACING_MM:Float = 3.0;
	static inline var MAX_SECTION_HATCH_LINES:Int = 20000;

	public static function project(part:SceneArtifactPart, metresPerUnit:Float, view:DrawingViewSpec):ProjectedDrawingView {
		if (part == null || view == null || part.id == null || part.id.length == 0)
			throw "Drawing projection needs a named CAD part and view";
		if (!Math.isFinite(metresPerUnit) || metresPerUnit <= 0)
			throw 'Drawing part "${part.id}" needs a positive finite unit scale';
		if (view.id == null || view.id.length == 0) throw "Drawing view needs an ID";
		var direction = unit(view.direction, 'Drawing view "${view.id}" direction');
		var up = unit(view.up, 'Drawing view "${view.id}" up');
		if (Math.abs(dot(direction, up)) > 1e-6) throw 'Drawing view "${view.id}" axes must be perpendicular';
		var right = unitPoint(cross(up, direction), 'Drawing view "${view.id}" horizontal axis');
		var section:Null<{origin:Point3, normal:Point3, sign:Float}> = null;
		if (view.section != null) {
			var sectionSpec:DrawingSectionSpec = cast view.section;
			section = {origin:point(sectionSpec.origin, 'Drawing view "${view.id}" section origin'),
				normal:unit(sectionSpec.normal, 'Drawing view "${view.id}" section normal'),
				sign:sectionSpec.keepPositive ? 1.0 : -1.0};
		}
		if (part.vertices == null || part.indices == null || part.vertexCount <= 0 || part.indexCount <= 0 ||
			part.vertices.length != part.vertexCount * 24 || part.indices.length != part.indexCount * 4 || part.indexCount % 3 != 0)
			throw 'Drawing part "${part.id}" has invalid mesh streams';
		var edges = part.edgeSegments;
		if (edges == null || edges.length == 0 || edges.length % 48 != 0 || edges.length / 48 > MAX_EDGE_SEGMENTS)
			throw 'Drawing part "${part.id}" needs tessellated CAD edges for projection';
		var unitScale = metresPerUnit * 1000.0;
		var projectPoint = function(p:Point3):Point2 return {
			x:dot(p, right) * unitScale, y:dot(p, up) * unitScale, depth:dot(p, direction) * unitScale};
		var vertices:Array<Point3> = [];
		for (index in 0...part.vertexCount) vertices.push({x:part.vertices.getDouble(index * 24),
			y:part.vertices.getDouble(index * 24 + 8), z:part.vertices.getDouble(index * 24 + 16)});
		var occlusionTriangles:Array<ProjectedTriangle> = [], sectionLines:Array<Segment3> = [];
		var sectionKeys = new Map<String, Bool>();
		var sectionCoplanarTriangles:Array<Array<Point2>> = [];
		var sectionFaceEdges = new Map<String, Segment3>();
		var sectionFaceEdgeCounts = new Map<String, Int>();
		for (offset in 0...Std.int(part.indexCount / 3)) {
			var base = offset * 12;
			var ia = part.indices.getInt32(base), ib = part.indices.getInt32(base + 4), ic = part.indices.getInt32(base + 8);
			if (ia < 0 || ib < 0 || ic < 0 || ia >= vertices.length || ib >= vertices.length || ic >= vertices.length)
				throw 'Drawing part "${part.id}" has a triangle index outside its vertex stream';
			var a = vertices[ia], b = vertices[ib], c = vertices[ic];
			if (section != null) {
				var sectionValue:{origin:Point3, normal:Point3, sign:Float} = cast section;
				var onPlaneA = Math.abs(signedSection(a, sectionValue)) <= 1e-10;
				var onPlaneB = Math.abs(signedSection(b, sectionValue)) <= 1e-10;
				var onPlaneC = Math.abs(signedSection(c, sectionValue)) <= 1e-10;
				if (onPlaneA && onPlaneB && onPlaneC) {
					sectionCoplanarTriangles.push([projectPoint(a), projectPoint(b), projectPoint(c)]);
					for (edge in [{a:a,b:b},{a:b,b:c},{a:c,b:a}]) {
						var key = segmentKey(edge);
						sectionFaceEdges.set(key, edge);
						sectionFaceEdgeCounts.set(key, sectionFaceEdgeCounts.exists(key) ? sectionFaceEdgeCounts.get(key) + 1 : 1);
					}
				} else {
					var cut = triangleSection(a, b, c, sectionValue);
					if (cut != null) {
						var key = segmentKey(cut);
						if (!sectionKeys.exists(key)) {sectionKeys.set(key, true); sectionLines.push(cut);}
					}
				}
			}
			var polygon = section == null ? [a, b, c] : clipTriangle(a, b, c, cast section);
			if (polygon.length >= 3) {
				var first = projectPoint(polygon[0]);
				for (index in 1...polygon.length - 1) {
					var second = projectPoint(polygon[index]), third = projectPoint(polygon[index + 1]);
					var area = (second.x - first.x) * (third.y - first.y) - (second.y - first.y) * (third.x - first.x);
					if (Math.abs(area) > 1e-12) occlusionTriangles.push(new ProjectedTriangle(first, second, third));
				}
			}
		}
		for (key in sectionFaceEdgeCounts.keys()) if (sectionFaceEdgeCounts.get(key) % 2 == 1 && !sectionKeys.exists(key)) {
			sectionKeys.set(key, true);
			sectionLines.push(sectionFaceEdges.get(key));
		}
		var occlusion = occlusionTriangles.length == 0 ? null : new DrawingOcclusionNode(occlusionTriangles);
		var edgeLines:Array<DrawingLine> = [];
		var edgeCount = Std.int(edges.length / 48);
		var depthEpsilon = Math.max(1e-7, unitScale * 1e-8);
		for (index in 0...edgeCount) {
			var offset = index * 48;
			var a:Point3 = {x:edges.getDouble(offset), y:edges.getDouble(offset + 8), z:edges.getDouble(offset + 16)};
			var b:Point3 = {x:edges.getDouble(offset + 24), y:edges.getDouble(offset + 32), z:edges.getDouble(offset + 40)};
			var clipped = section == null ? {a:a, b:b} : clipEdge(a, b, cast section);
			if (clipped == null) continue;
			appendClassifiedEdge(edgeLines, clipped, projectPoint, occlusion, view.showHiddenLines == true, depthEpsilon, "visible", null);
		}
		var lines:Array<DrawingLine> = [];
		if (section != null) appendSectionHatches(lines, sectionLines, sectionCoplanarTriangles, projectPoint);
		for (cut in sectionLines) appendLine(lines, cut, projectPoint, "section", null, section);
		for (line in edgeLines) lines.push(line);
		var callouts:Array<DrawingCallout> = [];
		var threads = part.threads;
		if (threads != null) for (thread in threads) {
			ThreadRenderDataValues.validate(thread);
			appendThread(lines, callouts, thread, direction, projectPoint, section, occlusion,
				view.showHiddenLines == true, depthEpsilon, unitScale);
		}
		if (lines.length == 0) throw 'Drawing view "${view.id}" projected no CAD edges or thread symbols from part "${part.id}"';
		var minX = Math.POSITIVE_INFINITY, minY = Math.POSITIVE_INFINITY;
		var maxX = Math.NEGATIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY;
		for (line in lines) {
			minX = Math.min(minX, Math.min(line.x1, line.x2)); minY = Math.min(minY, Math.min(line.y1, line.y2));
			maxX = Math.max(maxX, Math.max(line.x1, line.x2)); maxY = Math.max(maxY, Math.max(line.y1, line.y2));
		}
		layoutCallouts(callouts, maxX, minY, maxY);
		for (callout in callouts) {
			minX = Math.min(minX, callout.x); minY = Math.min(minY, callout.y - 1.5);
			maxX = Math.max(maxX, callout.x + callout.text.length * 1.35); maxY = Math.max(maxY, callout.y + 1.5);
		}
		return {partId:part.id, viewId:view.id, lengthUnit:"mm", lines:lines, callouts:callouts,
			bounds:{minX:minX, minY:minY, maxX:maxX, maxY:maxY}};
	}

	/** Hatches the section's actual coplanar material mesh or its closed cut loops. */
	static function appendSectionHatches(output:Array<DrawingLine>, boundaries:Array<Segment3>,
		coplanarTriangles:Array<Array<Point2>>, project:Point3->Point2):Void {
		if (boundaries.length == 0 && coplanarTriangles.length == 0) return;
		var axis = Math.sqrt(0.5), normalX = -axis, normalY = axis;
		var projectedBoundaries:Array<{a:Point2, b:Point2}> = [];
		var minimum = Math.POSITIVE_INFINITY, maximum = Math.NEGATIVE_INFINITY;
		if (coplanarTriangles.length > 0) {
			for (triangle in coplanarTriangles) for (point in triangle) {
				var offset = point.x * normalX + point.y * normalY;
				minimum = Math.min(minimum, offset);
				maximum = Math.max(maximum, offset);
			}
		} else {
			for (boundary in boundaries) {
				var a = project(boundary.a), b = project(boundary.b);
				var da = a.x * normalX + a.y * normalY, db = b.x * normalX + b.y * normalY;
				projectedBoundaries.push({a:a, b:b});
				minimum = Math.min(minimum, Math.min(da, db));
				maximum = Math.max(maximum, Math.max(da, db));
			}
		}
		var first = Math.fceil(minimum / SECTION_HATCH_SPACING_MM) * SECTION_HATCH_SPACING_MM;
		var count = Std.int(Math.floor((maximum - first) / SECTION_HATCH_SPACING_MM)) + 1;
		if (count > MAX_SECTION_HATCH_LINES)
			throw 'Section view needs $count hatch lines; the limit is $MAX_SECTION_HATCH_LINES';
		for (lineIndex in 0...count) {
			var offset = first + lineIndex * SECTION_HATCH_SPACING_MM;
			var intervals:Array<{start:Float, end:Float}> = [];
			if (coplanarTriangles.length > 0) {
				for (triangle in coplanarTriangles)
					appendHatchIntervals(intervals, lineHits(triangle, offset, axis, normalX, normalY), offset);
			} else {
				var intersections:Array<Float> = [];
				for (boundary in projectedBoundaries)
					appendSegmentHit(intersections, boundary.a, boundary.b, offset, axis, normalX, normalY);
				appendHatchIntervals(intervals, parityHits(intersections), offset);
		}
			intervals.sort(function(a, b) return a.start < b.start ? -1 : a.start > b.start ? 1 : 0);
			var start:Null<Float> = null, end = 0.0;
			for (interval in intervals) {
				if (start == null) {start = interval.start; end = interval.end;}
				else if (interval.start <= end + 1e-7) end = Math.max(end, interval.end);
				else {
					appendHatch(output, cast start, end, offset, axis, normalX, normalY);
					start = interval.start; end = interval.end;
				}
			}
			if (start != null) appendHatch(output, cast start, end, offset, axis, normalX, normalY);
		}
	}

	static function lineHits(points:Array<Point2>, offset:Float, axis:Float, normalX:Float, normalY:Float):Array<Float> {
		var intersections:Array<Float> = [];
		for (index in 0...points.length)
			appendSegmentHit(intersections, points[index], points[(index + 1) % points.length], offset, axis, normalX, normalY);
		return parityHits(intersections);
	}

	static function appendSegmentHit(intersections:Array<Float>, a:Point2, b:Point2, offset:Float,
		axis:Float, normalX:Float, normalY:Float):Void {
		var aNormal = a.x * normalX + a.y * normalY, bNormal = b.x * normalX + b.y * normalY;
		// Half-open crossing avoids counting a shared vertex twice.
		if ((aNormal <= offset && offset < bNormal) || (bNormal <= offset && offset < aNormal)) {
			var fraction = (offset - aNormal) / (bNormal - aNormal);
			var x = a.x + (b.x - a.x) * fraction, y = a.y + (b.y - a.y) * fraction;
			intersections.push(x * axis + y * axis);
		}
	}

	static function parityHits(intersections:Array<Float>):Array<Float> {
	intersections.sort(function(a, b) return a < b ? -1 : a > b ? 1 : 0);
	var unique:Array<Float> = [];
	var groupStart = 0;
	while (groupStart < intersections.length) {
		var groupEnd = groupStart + 1;
		while (groupEnd < intersections.length && Math.abs(intersections[groupEnd] - intersections[groupStart]) <= 1e-7)
			groupEnd++;
		// Tangencies contribute two coincident hits and do not change parity.
		if ((groupEnd - groupStart) % 2 == 1) unique.push(intersections[groupStart]);
		groupStart = groupEnd;
	}
	return unique;
	}

	static function appendHatchIntervals(intervals:Array<{start:Float, end:Float}>, hits:Array<Float>, offset:Float):Void {
		if (hits.length % 2 != 0)
			throw 'Section boundary is open or ambiguous at hatch offset ${offset} mm (${hits.join(", ")})';
		for (index in 0...Std.int(hits.length / 2))
			if (hits[index * 2 + 1] - hits[index * 2] > 1e-7)
				intervals.push({start:hits[index * 2], end:hits[index * 2 + 1]});
	}

	static function appendHatch(output:Array<DrawingLine>, start:Float, end:Float, offset:Float,
		axis:Float, normalX:Float, normalY:Float):Void {
		output.push({x1:start * axis + offset * normalX, y1:start * axis + offset * normalY,
			x2:end * axis + offset * normalX, y2:end * axis + offset * normalY, style:"section-hatch"});
	}

	static function appendClassifiedEdge(output:Array<DrawingLine>, segment:Segment3, project:Point3->Point2,
		occlusion:Null<DrawingOcclusionNode>, showHidden:Bool, epsilon:Float, visibleStyle:String,
		faceName:Null<String>):Void {
		var a = project(segment.a), b = project(segment.b);
		var hidden:Array<{start:Float, end:Float}> = [];
		if (occlusion != null) {
			var candidates:Array<ProjectedTriangle> = [];
			occlusion.collectCandidates(Math.min(a.x, b.x), Math.max(a.x, b.x),
				Math.min(a.y, b.y), Math.max(a.y, b.y), candidates);
			for (triangle in candidates) {
				var overlap = segmentTriangleInterval(a, b, triangle);
				if (overlap == null) continue;
				var interval:{start:Float, end:Float} = cast overlap;
				var first = pointAlong(a, b, interval.start), last = pointAlong(a, b, interval.end);
				var firstTriangleDepth = triangle.depthAt(first.x, first.y), lastTriangleDepth = triangle.depthAt(last.x, last.y);
				if (firstTriangleDepth == null || lastTriangleDepth == null) continue;
				var firstDifference = cast(firstTriangleDepth, Float) - first.depth - epsilon;
				var lastDifference = cast(lastTriangleDepth, Float) - last.depth - epsilon;
				if (firstDifference > 0 && lastDifference > 0) hidden.push(interval);
				else if (firstDifference > 0) {
					var crossing = interval.start + (interval.end - interval.start) *
						firstDifference / (firstDifference - lastDifference);
					hidden.push({start:interval.start, end:crossing});
				} else if (lastDifference > 0) {
					var crossing = interval.start + (interval.end - interval.start) *
						firstDifference / (firstDifference - lastDifference);
					hidden.push({start:crossing, end:interval.end});
				}
			}
		}
		hidden.sort(function(a, b) return a.start < b.start ? -1 : a.start > b.start ? 1 : 0);
		var cursor = 0.0, activeStart:Null<Float> = null, activeEnd = 0.0;
		for (interval in hidden) {
			if (interval.end <= cursor + 1e-12) continue;
			if (activeStart == null) {activeStart = interval.start; activeEnd = interval.end;}
			else if (interval.start <= activeEnd + 1e-12) activeEnd = Math.max(activeEnd, interval.end);
			else {
				appendEdgeRange(output, a, b, cursor, cast activeStart, visibleStyle, faceName);
				if (showHidden) appendEdgeRange(output, a, b, cast activeStart, activeEnd, "hidden", faceName);
				cursor = activeEnd; activeStart = interval.start; activeEnd = interval.end;
			}
		}
		if (activeStart != null) {
			appendEdgeRange(output, a, b, cursor, cast activeStart, visibleStyle, faceName);
			if (showHidden) appendEdgeRange(output, a, b, cast activeStart, activeEnd, "hidden", faceName);
			cursor = activeEnd;
		}
		appendEdgeRange(output, a, b, cursor, 1.0, visibleStyle, faceName);
	}

	static function segmentTriangleInterval(a:Point2, b:Point2, triangle:ProjectedTriangle):Null<{start:Float, end:Float}> {
		var area = (triangle.x1 - triangle.x0) * (triangle.y2 - triangle.y0) -
			(triangle.y1 - triangle.y0) * (triangle.x2 - triangle.x0);
		if (Math.abs(area) < 1e-14) return null;
		var orientation = area > 0 ? 1.0 : -1.0, lower = 0.0, upper = 1.0;
		for (edge in [{x0:triangle.x0,y0:triangle.y0,x1:triangle.x1,y1:triangle.y1},
			{x0:triangle.x1,y0:triangle.y1,x1:triangle.x2,y1:triangle.y2},
			{x0:triangle.x2,y0:triangle.y2,x1:triangle.x0,y1:triangle.y0}]) {
			var dx = edge.x1 - edge.x0, dy = edge.y1 - edge.y0;
			var first = orientation * (dx * (a.y - edge.y0) - dy * (a.x - edge.x0));
			var last = orientation * (dx * (b.y - edge.y0) - dy * (b.x - edge.x0));
			if (first < -1e-10 && last < -1e-10) return null;
			if (first < -1e-10) lower = Math.max(lower, first / (first - last));
			else if (last < -1e-10) upper = Math.min(upper, first / (first - last));
			if (lower > upper) return null;
		}
		return upper - lower <= 1e-12 ? null : {start:lower, end:upper};
	}

	static function pointAlong(a:Point2, b:Point2, amount:Float):Point2 return {
		x:a.x + (b.x - a.x) * amount,
		y:a.y + (b.y - a.y) * amount,
		depth:a.depth + (b.depth - a.depth) * amount};

	static function appendEdgeRange(output:Array<DrawingLine>, a:Point2, b:Point2,
		start:Float, end:Float, style:String, faceName:Null<String>):Void {
		if (end - start <= 1e-12) return;
		appendProjectedLine(output, pointAlong(a, b, start), pointAlong(a, b, end), style, faceName);
	}

	static function appendLine(output:Array<DrawingLine>, segment:Segment3, project:Point3->Point2, style:String,
		faceName:Null<String>, section:Null<{origin:Point3, normal:Point3, sign:Float}>):Void {
		var clipped = section == null ? segment : clipEdge(segment.a, segment.b, cast section);
		if (clipped != null) appendProjectedLine(output, project(clipped.a), project(clipped.b), style, faceName);
	}

	static function appendThreadLine(output:Array<DrawingLine>, segment:Segment3, project:Point3->Point2,
		section:Null<{origin:Point3, normal:Point3, sign:Float}>, occlusion:Null<DrawingOcclusionNode>,
		showHidden:Bool, epsilon:Float, style:String, faceName:String):Void {
		var clipped = section == null ? segment : clipEdge(segment.a, segment.b, cast section);
		if (clipped != null) appendClassifiedEdge(output, clipped, project, occlusion, showHidden, epsilon, style, faceName);
	}

	static function appendProjectedLine(output:Array<DrawingLine>, a:Point2, b:Point2, style:String, faceName:Null<String>):Void {
		if (Math.abs(a.x - b.x) + Math.abs(a.y - b.y) < 1e-9) return;
		var line:DrawingLine = {x1:a.x, y1:a.y, x2:b.x, y2:b.y, style:style};
		if (faceName != null) line.faceName = faceName;
		output.push(line);
	}

	static function appendThread(lines:Array<DrawingLine>, callouts:Array<DrawingCallout>, thread:ThreadRenderData,
		direction:Point3, project:Point3->Point2,
		section:Null<{origin:Point3, normal:Point3, sign:Float}>, occlusion:Null<DrawingOcclusionNode>,
		showHidden:Bool, epsilon:Float, unitScale:Float):Void {
		// Ball-race packets are semantic intervals, not screw-thread profiles.
		if (thread.spec.profile == ThreadProfile.Ball) return;
		var start = point(thread.origin, 'Thread "${thread.faceName}" origin');
		var axis = unit(thread.direction, 'Thread "${thread.faceName}" axis');
		var end = add(start, scale(axis, thread.length));
		var radial = unit(thread.xDirection, 'Thread "${thread.faceName}" radial axis');
		var transverse = unitPoint(cross(axis, radial), 'Thread "${thread.faceName}" transverse axis');
		var major = thread.spec.majorDiameter / 2.0 / unitScale;
		var minorDiameter = minorDiameterMm(thread);
		var minor = minorDiameter / 2.0 / unitScale;
		var axisView = Math.abs(dot(axis, direction));
		var majorStyle = thread.spec.internal ? "thread-minor" : "thread-major";
		var minorStyle = thread.spec.internal ? "thread-major" : "thread-minor";
		var face = thread.faceName;
		if (axisView >= 0.92) {
			var visibleEnd = dot(axis, direction) >= 0 ? end : start;
			var projectedCentre = project(visibleEnd);
			var visibleRing = dot(axis, direction) >= 0 ? thread.ringAtEnd : thread.ringAtStart;
			if (visibleRing) {
				// ISO conventional end views use a full major-diameter circle outside
				// the part, but a 3/4 circle for internal threads; the minor diameter
				// uses the complementary convention.
				appendThreadCircle(lines, visibleEnd, radial, transverse, major, !thread.spec.internal, majorStyle, face, project, section,
					occlusion, showHidden, epsilon);
				appendThreadCircle(lines, visibleEnd, radial, transverse, minor, thread.spec.internal, minorStyle, face, project, section,
					occlusion, showHidden, epsilon);
			}
			var callout:DrawingCallout = {x:0, y:0, leaderX:projectedCentre.x, leaderY:projectedCentre.y,
				text:threadCallout(thread, unitScale), faceName:face};
			callouts.push(callout);
			return;
		}
		var side = unitPoint(cross(direction, axis), 'Thread "${thread.faceName}" projected radial axis');
		for (entry in [{radius:major, style:majorStyle}, {radius:minor, style:minorStyle}]) {
			for (sign in [-1.0, 1.0]) {
				var offset = scale(side, entry.radius * sign);
				appendThreadLine(lines, {a:add(start, offset), b:add(end, offset)}, project, section,
					occlusion, showHidden, epsilon, entry.style, face);
			}
		}
		if (thread.ringAtStart) appendThreadLine(lines, {a:add(start, scale(side, -major)), b:add(start, scale(side, major))},
			project, section, occlusion, showHidden, epsilon, majorStyle, face);
		if (thread.ringAtEnd) appendThreadLine(lines, {a:add(end, scale(side, -major)), b:add(end, scale(side, major))},
			project, section, occlusion, showHidden, epsilon, majorStyle, face);
		var midpoint = project(scale(add(start, end), 0.5));
		callouts.push({x:0, y:0, leaderX:midpoint.x, leaderY:midpoint.y,
			text:threadCallout(thread, unitScale), faceName:face});
	}

	static function layoutCallouts(callouts:Array<DrawingCallout>, right:Float, bottom:Float, top:Float):Void {
		if (callouts.length == 0) return;
		callouts.sort(function(a, b) return Reflect.compare(a.faceName, b.faceName));
		var centre = (bottom + top) / 2;
		var spacing = Math.max(4.0, (top - bottom) / (callouts.length + 1));
		for (index in 0...callouts.length) {
			var callout = callouts[index];
			callout.x = right + 7.0;
			callout.y = centre + (index - (callouts.length - 1) / 2) * spacing;
		}
	}

	static function appendThreadCircle(lines:Array<DrawingLine>, centre:Point3, axisX:Point3, axisY:Point3,
		radius:Float, full:Bool, style:String, faceName:String, project:Point3->Point2,
		section:Null<{origin:Point3, normal:Point3, sign:Float}>, occlusion:Null<DrawingOcclusionNode>,
		showHidden:Bool, epsilon:Float):Void {
		var start = full ? 0 : Math.PI / 4, finish = full ? 2 * Math.PI : start + 1.5 * Math.PI;
		var previous:Null<Point3> = null;
		var segments = full ? THREAD_CIRCLE_SEGMENTS : Std.int(THREAD_CIRCLE_SEGMENTS * 0.75);
		for (step in 0...segments + 1) {
			var angle = start + (finish - start) * step / segments;
			var current = add(centre, add(scale(axisX, radius * Math.cos(angle)), scale(axisY, radius * Math.sin(angle))));
			if (previous != null) appendThreadLine(lines, {a:cast previous, b:current}, project, section,
				occlusion, showHidden, epsilon, style, faceName);
			previous = current;
		}
	}

	static function minorDiameterMm(thread:ThreadRenderData):Float {
		var spec = thread.spec, reduction = switch spec.profile {
			case ThreadProfile.Metric: spec.internal ? 1.082532 * spec.pitch : 1.226869 * spec.pitch;
			case ThreadProfile.Trapezoidal: spec.pitch;
			case ThreadProfile.Acme: 0.5 * spec.pitch;
			case ThreadProfile.Ball: throw 'Ball-race face "${thread.faceName}" has no screw-thread minor diameter';
		};
		var value = spec.majorDiameter - reduction;
		if (!Math.isFinite(value) || value <= 0) throw 'Thread "${thread.faceName}" has no positive symbolic minor diameter';
		return value;
	}

	static function threadCallout(thread:ThreadRenderData, unitScale:Float):String {
		var spec = thread.spec, pitch = fmt(spec.pitch), lengthMm = thread.length * unitScale;
		var starts = spec.starts > 1 ? ' · ${spec.starts}-start' : "";
		var hand = switch spec.hand {case materia.project.ThreadSpecData.ThreadHand.LeftHand: " LH"; case _: "";};
		var extent = spec.internal ? " deep" : " long";
		return '${spec.designation} × ${pitch}${starts}${hand} · ${spec.toleranceClass} · ${fmt(lengthMm)} mm$extent';
	}

	static function triangleSection(a:Point3, b:Point3, c:Point3,
		section:{origin:Point3, normal:Point3, sign:Float}):Null<Segment3> {
		if (Math.abs(signedSection(a, section)) < 1e-10 && Math.abs(signedSection(b, section)) < 1e-10 &&
			Math.abs(signedSection(c, section)) < 1e-10) return null;
		var points:Array<Point3> = [];
		for (edge in [{a:a,b:b},{a:b,b:c},{a:c,b:a}]) {
			var da = signedSection(edge.a, section), db = signedSection(edge.b, section);
			if (Math.abs(da) < 1e-10) addUnique(points, edge.a, 1e-9);
			if (da * db < -1e-20) addUnique(points, lerp(edge.a, edge.b, da / (da - db)), 1e-9);
		}
		if (points.length < 2) return null;
		var first = points[0], second = points[1], farthest = distanceSquared(first, second);
		for (i in 0...points.length) for (j in i + 1...points.length) {
			var distance = distanceSquared(points[i], points[j]);
			if (distance > farthest) {first = points[i]; second = points[j]; farthest = distance;}
		}
		return farthest < 1e-18 ? null : {a:first,b:second};
	}

	static function clipTriangle(a:Point3, b:Point3, c:Point3,
		section:{origin:Point3, normal:Point3, sign:Float}):Array<Point3> {
		var input = [a, b, c], output:Array<Point3> = [];
		for (index in 0...input.length) {
			var current = input[index], next = input[(index + 1) % input.length];
			var currentDistance = signedSection(current, section), nextDistance = signedSection(next, section);
			var currentInside = currentDistance >= -1e-10, nextInside = nextDistance >= -1e-10;
			if (currentInside && nextInside) output.push(next);
			else if (currentInside && !nextInside) output.push(lerp(current, next, currentDistance / (currentDistance - nextDistance)));
			else if (!currentInside && nextInside) {
				output.push(lerp(current, next, currentDistance / (currentDistance - nextDistance)));
				output.push(next);
			}
		}
		return output;
	}

	static function clipEdge(a:Point3, b:Point3, section:{origin:Point3, normal:Point3, sign:Float}):Null<Segment3> {
		var da = signedSection(a, section), db = signedSection(b, section);
		var insideA = da >= -1e-10, insideB = db >= -1e-10;
		if (!insideA && !insideB) return null;
		if (insideA && insideB) return {a:a,b:b};
		var crossing = lerp(a, b, da / (da - db));
		return insideA ? {a:a,b:crossing} : {a:crossing,b:b};
	}

	static function signedSection(point:Point3, section:{origin:Point3, normal:Point3, sign:Float}):Float
		return dot(sub(point, section.origin), section.normal) * section.sign;

	static function addUnique(points:Array<Point3>, value:Point3, epsilon:Float):Void {
		for (point in points) if (distanceSquared(point, value) <= epsilon * epsilon) return;
		points.push(value);
	}

	static function segmentKey(segment:Segment3):String {
		var first = pointKey(segment.a), second = pointKey(segment.b);
		return Reflect.compare(first, second) <= 0 ? first + "|" + second : second + "|" + first;
	}

	static function pointKey(value:Point3):String return Std.string(Math.round(value.x * 1e9)) + "," +
		Std.string(Math.round(value.y * 1e9)) + "," + Std.string(Math.round(value.z * 1e9));

	static function point(values:Array<Float>, label:String):Point3 {
		if (values == null || values.length != 3) throw '$label must contain three coordinates';
		for (value in values) if (!Math.isFinite(value)) throw '$label must be finite';
		return {x:values[0],y:values[1],z:values[2]};
	}

	static function unit(values:Array<Float>, label:String):Point3 {
		return unitPoint(point(values, label), label);
	}

	static function unitPoint(value:Point3, label:String):Point3 {
		var length = Math.sqrt(dot(value, value));
		if (!Math.isFinite(length) || length <= 1e-12) throw '$label must be nonzero';
		if (Math.abs(length - 1) > 1e-6) throw '$label must be normalized';
		return scale(value, 1 / length);
	}

	static function add(a:Point3, b:Point3):Point3 return {x:a.x+b.x,y:a.y+b.y,z:a.z+b.z};
	static function sub(a:Point3, b:Point3):Point3 return {x:a.x-b.x,y:a.y-b.y,z:a.z-b.z};
	static function scale(a:Point3, factor:Float):Point3 return {x:a.x*factor,y:a.y*factor,z:a.z*factor};
	static function dot(a:Point3, b:Point3):Float return a.x*b.x+a.y*b.y+a.z*b.z;
	static function cross(a:Point3, b:Point3):Point3 return {x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x};
	static function lerp(a:Point3, b:Point3, t:Float):Point3 return add(a,scale(sub(b,a),t));
	static function distanceSquared(a:Point3, b:Point3):Float return dot(sub(a,b),sub(a,b));
	static function fmt(value:Float):String return Std.string(Math.round(value * 1000) / 1000);
}
