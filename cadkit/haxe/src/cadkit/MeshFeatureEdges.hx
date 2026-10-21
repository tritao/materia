package cadkit;

import haxe.io.Bytes;

private typedef EdgeSides = {var normals:Array<Array<Float>>;}

/**
 * Which tessellated edge segments are feature edges worth drawing, without changing the topology: only a segment
 * shared by exactly two triangles whose analytic normals agree at both ends is smooth (a cylinder's seam, a face
 * split across a plane). Open, non-manifold, unmatched or degenerate segments stay visible, and a BRep edge is
 * drawn whole or not at all. The streams are a `Mesh`'s, or the same layout from a stored artifact.
 */
class MeshFeatureEdges {
	public static function ofMesh(mesh:Mesh):Array<Bool>
		return classify(mesh.vertices, mesh.normals, mesh.indices, mesh.vertexCount, mesh.indexCount, mesh.edgeSegments, mesh.edgeIds);

	/** One flag per segment of `edgeSegments` (pairs of double 3D points, 48 bytes each): true when it is a feature. */
	public static function classify(vertices:Bytes, normals:Bytes, indices:Bytes, vertexCount:Int, indexCount:Int,
			edgeSegments:Null<Bytes>, edgeIds:Null<Bytes>):Array<Bool> {
		var strokes = edgeSegments;
		if (strokes == null)
			return [];
		var count = Std.int(strokes.length / 48), keys:Array<String> = [], requested:Map<String, EdgeSides> = [];
		function point(x:Float, y:Float, z:Float):String {
			// Source CAD units are mm; keep distinct features above OCCT's nominal modelling tolerance.
			return Std.string(Math.ffloor(x / 1e-7 + 0.5)) + ":" + Std.string(Math.ffloor(y / 1e-7 + 0.5)) + ":"
				+ Std.string(Math.ffloor(z / 1e-7 + 0.5));
		}
		function pair(a:String, b:String):String
			return a < b ? a + "/" + b : b + "/" + a;
		for (i in 0...count) {
			var o = i * 48, key = pair(point(strokes.getDouble(o), strokes.getDouble(o + 8), strokes.getDouble(o + 16)),
				point(strokes.getDouble(o + 24), strokes.getDouble(o + 32), strokes.getDouble(o + 40)));
			keys.push(key);
			requested.set(key, {normals: []});
		}
		var points:Array<String> = [];
		for (i in 0...vertexCount)
			points.push(point(vertices.getDouble(i * 24), vertices.getDouble(i * 24 + 8), vertices.getDouble(i * 24 + 16)));
		for (i in 0...Std.int(indexCount / 3)) {
			var triangle = [indices.getInt32(i * 12), indices.getInt32(i * 12 + 4), indices.getInt32(i * 12 + 8)];
			for (j in 0...3) {
				var a = triangle[j], b = triangle[(j + 1) % 3];
				if (a < 0 || b < 0 || a >= vertexCount || b >= vertexCount)
					throw "CAD feature edge index out of bounds";
				var sides = requested.get(pair(points[a], points[b]));
				if (sides == null)
					continue;
				// Order endpoint normals by position so opposite triangle winding cannot swap them.
				if (points[a] > points[b]) {
					var swap = a;
					a = b;
					b = swap;
				}
				var pairNormals:Array<Float> = [];
				for (v in [a, b])
					for (axis in 0...3)
						pairNormals.push(normals.getDouble(v * 24 + axis * 8));
				sides.normals.push(pairNormals);
			}
		}
		var threshold = Math.cos(Math.PI / 180), result:Array<Bool> = [], features:Map<Int, Bool> = [];
		var identified = edgeIds != null && edgeIds.length == count * 4;
		for (i in 0...count) {
			var sides = requested.get(keys[i]), smooth = sides != null && sides.normals.length == 2;
			if (smooth && sides != null)
				for (endpoint in 0...2) {
					var a = sides.normals[0], b = sides.normals[1], o = endpoint * 3;
					var aa = a[o] * a[o] + a[o + 1] * a[o + 1] + a[o + 2] * a[o + 2];
					var bb = b[o] * b[o] + b[o + 1] * b[o + 1] + b[o + 2] * b[o + 2];
					var dot = a[o] * b[o] + a[o + 1] * b[o + 1] + a[o + 2] * b[o + 2];
					if (!Math.isFinite(dot + aa + bb) || aa < 1e-20 || bb < 1e-20 || dot / Math.sqrt(aa * bb) < threshold)
						smooth = false;
				}
			result.push(!smooth);
			if (!smooth && identified)
				features.set(edgeIds.getInt32(i * 4), true);
		}
		// Never draw only part of one BRep edge when an endpoint/triangulation is uncertain.
		if (identified)
			for (i in 0...count)
				if (features.exists(edgeIds.getInt32(i * 4)))
					result[i] = true;
		return result;
	}
}
