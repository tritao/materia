package cadkit.threads;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import materia.project.ThreadSpecData;
import materia.project.ThreadSpecData.ThreadHand;

private typedef ThreadVertex = {var point:Vector; var normal:Vector; var u:Float; var v:Float; var depth:Float; var sourceIndex:Int; var seam:Bool;}

/**
 * A cosmetic thread's surface: a threaded face's own triangles, clipped to the thread's span and unwrapped so
 * a `ThreadTexture` tiles along the helix (u turns once per start around the axis, v advances one per pitch).
 * It never asks the kernel for another tessellation. Planes where the thread stops inside the face become ring
 * segments, drawn as edges. Shared by the editor and other viewers of tessellated CAD (the docs' catalogue).
 */
class ThreadSurface {
	/** Flat xyz per vertex, in the input's units. */
	public final positions:Array<Float> = [];

	public final normals:Array<Float> = [];

	/** Flat uv per vertex. */
	public final uvs:Array<Float> = [];

	/** Three vertex indices per triangle. */
	public final triangles:Array<Int> = [];

	/** Ring segments where the thread ends inside its face: six values (two points) each. */
	public final rings:Array<Float> = [];

	/**
	 * The rest of the face, outside the thread's span, as plain triangles (flat xyz positions and normals, three
	 * vertices per triangle): with the thread surface it covers the face, for viewers that replace the face rather
	 * than draw over it.
	 */
	public final outsidePositions:Array<Float> = [];

	public final outsideNormals:Array<Float> = [];

	function new() {}

	public var vertexCount(get, never):Int;

	function get_vertexCount():Int
		return Std.int(positions.length / 3);

	/** A stable texture phase per face name, so neighbouring threads do not line up. */
	public static function phase(name:String):Float {
		var hash = -2128831035;
		for (index in 0...name.length)
			hash = (hash ^ name.charCodeAt(index)) * 16777619;
		return (hash & 0x7fffffff) / 2147483648.0;
	}

	/**
	 * Unwraps one face's triangles (`points` and `normals` flat xyz per vertex, `faceTriangles` three indices per
	 * triangle) for a thread whose axis frame is `frame` (origin on the axis at the thread's start, normal along
	 * the axis) running `length` along it, with `pitch` in the same units.
	 */
	public static function unwrap(faceName:String, spec:ThreadSpecData, frame:Plane, length:Float, pitch:Float, ringAtStart:Bool,
			ringAtEnd:Bool, points:Array<Float>, normals:Array<Float>, faceTriangles:Array<Int>):ThreadSurface {
		var result = new ThreadSurface();
		var vertexMap:Map<Int, Int> = [];
		var phaseOffset = phase(faceName);
		var starts = spec.starts;
		var hand = spec.hand == LeftHand ? -1 : 1;
		for (triangle in 0...Std.int(faceTriangles.length / 3)) {
			var polygon:Array<ThreadVertex> = [];
			for (corner in 0...3) {
				var index = faceTriangles[triangle * 3 + corner];
				var point = new Vector(points[index * 3], points[index * 3 + 1], points[index * 3 + 2]);
				var normal = new Vector(normals[index * 3], normals[index * 3 + 1], normals[index * 3 + 2]);
				var local = frame.toLocal(point);
				var depth = local.z;
				if (Math.abs(depth) < 1e-8)
					depth = 0;
				if (Math.abs(depth - length) < 1e-8)
					depth = length;
				var angle = Math.atan2(local.y, local.x) / (2 * Math.PI);
				if (angle < 0)
					angle += 1;
				polygon.push({point: point, normal: normal, u: angle, v: depth / pitch, depth: depth, sourceIndex: index, seam: false});
			}
			var minimum = Math.min(polygon[0].u, Math.min(polygon[1].u, polygon[2].u));
			var maximum = Math.max(polygon[0].u, Math.max(polygon[1].u, polygon[2].u));
			// Unwrap before multiplying by starts: a seam shift spans 'starts' texture periods.
			for (vertex in polygon) {
				if (maximum - minimum > 0.5 && vertex.u < 0.5) {
					vertex.u += 1;
					vertex.seam = true;
				}
				vertex.u = vertex.u * starts * hand + phaseOffset;
			}
			if (ringAtStart)
				ring(result, polygon, 0);
			if (ringAtEnd)
				ring(result, polygon, length);
			outside(result, clip(polygon, 0, false), 0);
			outside(result, clip(polygon, length, true), length);
			polygon = clip(clip(polygon, 0, true), length, false);
			if (polygon.length < 3)
				continue;
			var polygonIndices:Array<Int> = [];
			for (vertex in polygon) {
				var key = vertex.sourceIndex * 2 + (vertex.seam ? 1 : 0);
				var existing = vertex.sourceIndex < 0 ? null : vertexMap.get(key);
				var index:Int;
				if (existing == null) {
					index = result.vertexCount;
					result.positions.push(vertex.point.x);
					result.positions.push(vertex.point.y);
					result.positions.push(vertex.point.z);
					result.normals.push(vertex.normal.x);
					result.normals.push(vertex.normal.y);
					result.normals.push(vertex.normal.z);
					result.uvs.push(vertex.u);
					result.uvs.push(vertex.v);
					if (vertex.sourceIndex >= 0)
						vertexMap.set(key, index);
				} else
					index = existing;
				polygonIndices.push(index);
			}
			for (index in 1...polygon.length - 1) {
				result.triangles.push(polygonIndices[0]);
				result.triangles.push(polygonIndices[index]);
				result.triangles.push(polygonIndices[index + 1]);
			}
		}
		return result;
	}

	static function interpolate(a:ThreadVertex, b:ThreadVertex, depth:Float):ThreadVertex {
		var amount = (depth - a.depth) / (b.depth - a.depth);
		return {
			point: a.point.add(b.point.subtract(a.point).scale(amount)),
			normal: a.normal.add(b.normal.subtract(a.normal).scale(amount)).normalized(),
			u: a.u + (b.u - a.u) * amount,
			v: a.v + (b.v - a.v) * amount,
			depth: depth,
			sourceIndex: -1,
			seam: false
		};
	}

	static function clip(polygon:Array<ThreadVertex>, depth:Float, greater:Bool):Array<ThreadVertex> {
		var result:Array<ThreadVertex> = [];
		if (polygon.length == 0)
			return result;
		var previous = polygon[polygon.length - 1];
		for (current in polygon) {
			var before = greater ? previous.depth >= depth : previous.depth <= depth;
			var after = greater ? current.depth >= depth : current.depth <= depth;
			if (before != after)
				result.push(interpolate(previous, current, depth));
			if (after)
				result.push(current);
			previous = current;
		}
		return result;
	}

	/** A clipped piece outside the span, fanned into triangles; pieces touching the span only along a line drop out. */
	static function outside(result:ThreadSurface, polygon:Array<ThreadVertex>, boundary:Float):Void {
		var flat = true;
		for (vertex in polygon)
			if (Math.abs(vertex.depth - boundary) > 1e-9)
				flat = false;
		if (polygon.length < 3 || flat)
			return;
		for (index in 1...polygon.length - 1)
			for (vertex in [polygon[0], polygon[index], polygon[index + 1]]) {
				result.outsidePositions.push(vertex.point.x);
				result.outsidePositions.push(vertex.point.y);
				result.outsidePositions.push(vertex.point.z);
				result.outsideNormals.push(vertex.normal.x);
				result.outsideNormals.push(vertex.normal.y);
				result.outsideNormals.push(vertex.normal.z);
			}
	}

	/** Intersection with an interior depth plane: one ring segment per crossed triangle. */
	static function ring(result:ThreadSurface, polygon:Array<ThreadVertex>, depth:Float):Void {
		var crossings:Array<Vector> = [];
		var previous = polygon[polygon.length - 1];
		for (current in polygon) {
			if ((previous.depth < depth && current.depth >= depth) || (current.depth < depth && previous.depth >= depth))
				crossings.push(interpolate(previous, current, depth).point);
			previous = current;
		}
		if (crossings.length == 2 && crossings[0].subtract(crossings[1]).length() > 1e-10)
			for (point in crossings) {
				result.rings.push(point.x);
				result.rings.push(point.y);
				result.rings.push(point.z);
			}
	}
}
