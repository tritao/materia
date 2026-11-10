package cadkit;

import haxe.io.Bytes;

private typedef EdgeSides = {
  var normals:Array<Array<Float>>;
}

/**
 * Which tessellated edge segments are feature edges worth drawing, without changing the topology: only a segment
 * shared by exactly two triangles whose analytic normals agree at both ends is smooth (a cylinder's seam, a face
 * split across a plane). Open, non-manifold, unmatched or degenerate segments stay visible, and a BRep edge is
 * drawn whole or not at all. The streams are a `Mesh`'s, or the same layout from a stored artifact.
 */
/** One planar-or-not B-rep face of a mesh, as seen by the step analysis. */
private class StepFace {
  public final triangles:Array<Int> = [];
  public final neighbours:Map<Int, Bool> = [];
  public final boundary:Array<StepBoundary> = [];
  public var planar = false;
  public var normal:Array<Float> = [0.0, 0.0, 0.0];
  public var offset = 0.0;

  public function new() {}
}

/** One mesh edge on a face's boundary and the face across it. */
private class StepBoundary {
  public final start:Array<Float>;
  public final end:Array<Float>;
  public final other:Int;

  public function new(start:Array<Float>, end:Array<Float>, other:Int) {
    this.start = start;
    this.end = end;
    this.other = other;
  }
}

class MeshFeatureEdges {
  /** Faces whose normals differ by less than this are parallel for a step (cos 1 degree). */
  static inline var PARALLEL_COS = 0.9998476951563913;

  /**
   * One step height per segment of `edgeSegments`, in mesh units: how far apart the two parallel faces are that this
   * edge's narrow neighbour joins, or 0 when the edge is a corner (its faces turn, or the topology is not a plain
   * step). A box edge or a chamfer edge is a corner; both edges of a 0.05 mm raised band are steps of 0.05. Only
   * planar faces form steps. `triangleFaces` gives each triangle's B-rep face (-1 when unknown). Segments of one
   * B-rep edge (`edgeIds`) share one value, a corner if any of them is.
   */
  public static function steps(
    vertices:Bytes,
    indices:Bytes,
    vertexCount:Int,
    indexCount:Int,
    triangleFaces:Array<Int>,
    edgeSegments:Null<Bytes>,
    edgeIds:Null<Bytes>
  ):Array<Float> {
    var strokes = edgeSegments;
    if (strokes == null) return [];
    var count = Std.int(strokes.length / 48), triangleCount = Std.int(indexCount / 3);
    if (triangleFaces.length != triangleCount) throw "Step analysis needs one face per triangle";
    var points:Array<String> = [for (i in 0...vertexCount) pointKey(
      vertices.getDouble(i * 24),
      vertices.getDouble(i * 24 + 8),
      vertices.getDouble(i * 24 + 16)
    )];
    function vertex(index:Int, axis:Int):Float return vertices.getDouble(index * 24 + axis * 8);
    // Mesh edges to their triangles, and B-rep faces to their triangles and neighbours.
    var edgeTriangles:Map<String, Array<Int>> = [], faces:Map<Int, StepFace> = [];
    for (t in 0...triangleCount) {
      var face = triangleFaces[t];
      if (face >= 0) {
        var entry = faces.get(face);
        if (entry == null) faces.set(face, entry = new StepFace());
        entry.triangles.push(t);
      }
      for (j in 0...3) {
        var a = indices.getInt32((t * 3 + j) * 4), b = indices.getInt32((t * 3 + (j + 1) % 3) * 4);
        if (a < 0 || b < 0 || a >= vertexCount || b >= vertexCount) throw "CAD step edge index out of bounds";
        var key = pairKey(points[a], points[b]), list = edgeTriangles.get(key);
        if (list == null) edgeTriangles.set(key, list = []);
        list.push(t);
      }
    }
    for (list in edgeTriangles) for (x in list) for (y in list) {
      var f = triangleFaces[x], g = triangleFaces[y];
      var entry = f >= 0 && g >= 0 && f != g ? faces.get(f) : null;
      if (entry != null) entry.neighbours.set(g, true);
    }
    // Each face's boundary: mesh edges it shares with another face, with their end points.
    var edgeEnds:Map<String, Array<Int>> = [];
    for (t in 0...triangleCount) for (j in 0...3) {
      var a = indices.getInt32((t * 3 + j) * 4), b = indices.getInt32((t * 3 + (j + 1) % 3) * 4);
      var key = pairKey(points[a], points[b]);
      if (!edgeEnds.exists(key)) edgeEnds.set(key, [a, b]);
    }
    for (key => list in edgeTriangles) {
      var ends = edgeEnds.get(key);
      if (ends == null) continue;
      for (x in list) for (y in list) {
        var f = triangleFaces[x], g = triangleFaces[y];
        var entry = f >= 0 && g >= 0 && f != g ? faces.get(f) : null;
        if (entry != null) entry.boundary.push(new StepBoundary(
          [vertex(ends[0], 0), vertex(ends[0], 1), vertex(ends[0], 2)],
          [vertex(ends[1], 0), vertex(ends[1], 1), vertex(ends[1], 2)],
          g
        ));
      }
    }
    // A face is planar when every vertex lies on the plane of its area-weighted normal.
    for (face in faces) {
      var n = [0.0, 0.0, 0.0], scale = 0.0;
      for (t in face.triangles) {
        var i0 = indices.getInt32(t * 12), i1 = indices.getInt32(t * 12 + 4), i2 = indices.getInt32(t * 12 + 8);
        var ux = vertex(i1, 0) - vertex(i0, 0), uy = vertex(i1, 1) - vertex(i0, 1), uz = vertex(i1, 2) - vertex(i0, 2);
        var vx = vertex(i2, 0) - vertex(i0, 0), vy = vertex(i2, 1) - vertex(i0, 1), vz = vertex(i2, 2) - vertex(i0, 2);
        n[0] += uy * vz - uz * vy;
        n[1] += uz * vx - ux * vz;
        n[2] += ux * vy - uy * vx;
        scale = Math.max(scale, Math.max(Math.abs(ux), Math.max(Math.abs(uy), Math.max(Math.abs(uz),
          Math.max(Math.abs(vx), Math.max(Math.abs(vy), Math.abs(vz)))))));
      }
      var length = Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
      if (!(length > 0.0)) continue;
      for (axis in 0...3) n[axis] /= length;
      var first = indices.getInt32(face.triangles[0] * 12);
      var offset = n[0] * vertex(first, 0) + n[1] * vertex(first, 1) + n[2] * vertex(first, 2);
      var planar = true, tolerance = Math.max(1e-7, scale * 1e-6);
      for (t in face.triangles) for (j in 0...3) {
        var v = indices.getInt32((t * 3 + j) * 4);
        if (Math.abs(n[0] * vertex(v, 0) + n[1] * vertex(v, 1) + n[2] * vertex(v, 2) - offset) > tolerance) planar = false;
      }
      face.planar = planar;
      face.normal = n;
      face.offset = offset;
    }
    function parallel(a:StepFace, b:StepFace):Bool
      return a.normal[0] * b.normal[0] + a.normal[1] * b.normal[1] + a.normal[2] * b.normal[2] > PARALLEL_COS;
    // Step height across `narrow` from `side`, for the edge p0-p1 between them. Walk from the edge straight across
    // `narrow`: the first boundary reached that runs along the edge must belong to a face parallel to `side` and facing
    // the same way, at the distance between their planes. That is a step (a raised band, a proud collar, a ledge out
    // to a band). Anything else is a corner: a box's top runs to a face turned the other way, and a thin ring's first
    // boundary across is its own inner face, whatever parallel face lies on the far side of the ring.
    function across(side:Int, narrow:Int, p0:Array<Float>, p1:Array<Float>, touching:Array<Int>):Float {
      var a = faces.get(side), n = faces.get(narrow);
      if (a == null || n == null || !a.planar || !n.planar) return 0.0;
      var d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
      var length = Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
      if (!(length > 0.0)) return 0.0;
      for (axis in 0...3) d[axis] /= length;
      // Across the edge within `narrow`'s plane; its sign is fixed by which side the face lies on.
      var u = [n.normal[1] * d[2] - n.normal[2] * d[1], n.normal[2] * d[0] - n.normal[0] * d[2], n.normal[0] * d[1] - n.normal[1] * d[0]];
      var nearest = Math.POSITIVE_INFINITY, nearestFace = -1, scale = length;
      // Into `narrow` is the side its triangle at this edge lies on, not where the rest of the face is: a ring's
      // far side lies opposite its near side.
      var inward = 0.0;
      for (t in touching) if (triangleFaces[t] == narrow) for (j in 0...3) {
        var v = indices.getInt32((t * 3 + j) * 4);
        var along = (vertex(v, 0) - p0[0]) * u[0] + (vertex(v, 1) - p0[1]) * u[1] + (vertex(v, 2) - p0[2]) * u[2];
        if (Math.abs(along) > Math.abs(inward)) inward = along;
      }
      if (inward == 0.0) return 0.0;
      if (inward < 0.0) for (axis in 0...3) u[axis] = -u[axis];
      for (edge in n.boundary) for (q in [edge.start, edge.end])
        scale = Math.max(scale, Math.abs((q[0] - p0[0]) * u[0] + (q[1] - p0[1]) * u[1] + (q[2] - p0[2]) * u[2]));
      var tolerance = Math.max(1e-7, scale * 1e-7);
      for (edge in n.boundary) {
        var e = [edge.end[0] - edge.start[0], edge.end[1] - edge.start[1], edge.end[2] - edge.start[2]];
        var el = Math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        if (!(el > 0.0) || Math.abs((e[0] * d[0] + e[1] * d[1] + e[2] * d[2]) / el) < PARALLEL_COS) continue;
        var distance = (edge.start[0] - p0[0]) * u[0] + (edge.start[1] - p0[1]) * u[1] + (edge.start[2] - p0[2]) * u[2];
        if (distance <= tolerance) continue;
        // It must lie across from the edge: their spans along the edge overlap.
        var s0 = (edge.start[0] - p0[0]) * d[0] + (edge.start[1] - p0[1]) * d[1] + (edge.start[2] - p0[2]) * d[2];
        var s1 = (edge.end[0] - p0[0]) * d[0] + (edge.end[1] - p0[1]) * d[1] + (edge.end[2] - p0[2]) * d[2];
        if (Math.max(s0, s1) <= tolerance || Math.min(s0, s1) >= length - tolerance) continue;
        if (distance < nearest - tolerance) {
          nearest = distance;
          nearestFace = edge.other;
        }
      }
      if (nearestFace < 0 || nearestFace == side) return 0.0;
      var g = faces.get(nearestFace);
      if (g == null || !g.planar || !parallel(a, g)) return 0.0;
      var h = Math.abs(g.offset - a.offset);
      return h > 0.0 ? h : 0.0;
    }
    var result:Array<Float> = [];
    for (i in 0...count) {
      var o = i * 48;
      var key = pairKey(
        pointKey(strokes.getDouble(o), strokes.getDouble(o + 8), strokes.getDouble(o + 16)),
        pointKey(strokes.getDouble(o + 24), strokes.getDouble(o + 32), strokes.getDouble(o + 40))
      );
      var beside:Array<Int> = [];
      var list = edgeTriangles.get(key);
      if (list != null) for (t in list) {
        var f = triangleFaces[t];
        if (f >= 0 && beside.indexOf(f) < 0) beside.push(f);
      }
      var step = 0.0;
      if (beside.length == 2) {
        var p0 = [strokes.getDouble(o), strokes.getDouble(o + 8), strokes.getDouble(o + 16)];
        var p1 = [strokes.getDouble(o + 24), strokes.getDouble(o + 32), strokes.getDouble(o + 40)];
        var first = across(beside[0], beside[1], p0, p1, list), second = across(beside[1], beside[0], p0, p1, list);
        step = first == 0.0 ? second : second == 0.0 ? first : Math.min(first, second);
      }
      result.push(step);
    }
    // One B-rep edge draws one way: a corner if any segment is, else its smallest step.
    if (edgeIds != null && edgeIds.length == count * 4) {
      var byEdge:Map<Int, Float> = [];
      for (i in 0...count) {
        var id = edgeIds.getInt32(i * 4), known = byEdge.get(id);
        byEdge.set(id, known == null ? result[i] : (known == 0.0 || result[i] == 0.0 ? 0.0 : Math.min(known, result[i])));
      }
      for (i in 0...count) {
        var shared = byEdge.get(edgeIds.getInt32(i * 4));
        if (shared != null) result[i] = shared;
      }
    }
    return result;
  }

  /** Each triangle's B-rep face from face ranges (first index and index count per face); -1 outside every range. */
  public static function triangleFaces(indexCount:Int, faceIndices:Array<Int>, firstIndices:Array<Int>,
      indexCounts:Array<Int>):Array<Int> {
    var result = [for (_ in 0...Std.int(indexCount / 3)) -1];
    for (r in 0...faceIndices.length) for (t in Std.int(firstIndices[r] / 3)...Std.int((firstIndices[r] + indexCounts[r]) / 3))
      if (t >= 0 && t < result.length) result[t] = faceIndices[r];
    return result;
  }

  // Source CAD units are mm; keep distinct features above OCCT's nominal modelling tolerance.
  static function pointKey(x:Float, y:Float, z:Float):String
    return Std.string(Math.ffloor(x / 1e-7 + 0.5)) + ":" + Std.string(Math.ffloor(y / 1e-7 + 0.5)) + ":"
      + Std.string(Math.ffloor(z / 1e-7 + 0.5));

  static function pairKey(a:String, b:String):String return a < b ? a + "/" + b : b + "/" + a;
  public static function ofMesh(mesh:Mesh):Array<Bool> return classify(
    mesh.vertices,
    mesh.normals,
    mesh.indices,
    mesh.vertexCount,
    mesh.indexCount,
    mesh.edgeSegments,
    mesh.edgeIds
  );

  /** One flag per segment of `edgeSegments` (pairs of double 3D points, 48 bytes each): true when it is a feature. */
  public static function classify(
    vertices:Bytes,
    normals:Bytes,
    indices:Bytes,
    vertexCount:Int,
    indexCount:Int,
    edgeSegments:Null<Bytes>,
    edgeIds:Null<Bytes>
  ):Array<Bool> {
    var strokes = edgeSegments;
    if (strokes == null) return [];
    var count = Std.int(strokes.length / 48), keys:Array<String> = [], requested:Map<String, EdgeSides> = [];
    function point(x:Float, y:Float, z:Float):String {
      // Source CAD units are mm; keep distinct features above OCCT's nominal modelling tolerance.
      return Std.string(Math.ffloor(x
        / 1e-7 + 0.5)) + ":" + Std.string(Math.ffloor(y / 1e-7 + 0.5)) + ":" + Std.string(Math.ffloor(z / 1e-7 + 0.5));
    }
    function pair(a:String, b:String):String return a < b ? a + "/" + b : b + "/" + a;
    for (i in 0...count) {
      var o = i * 48, key = pair(
        point(strokes.getDouble(o), strokes.getDouble(o + 8), strokes.getDouble(o + 16)),
        point(strokes.getDouble(o + 24), strokes.getDouble(o + 32), strokes.getDouble(o + 40))
      );
      keys.push(key);
      requested.set(key, {normals: []});
    }
    var points:Array<String> = [];
    for (i in 0...vertexCount) points.push(point(
      vertices.getDouble(i * 24),
      vertices.getDouble(i * 24 + 8),
      vertices.getDouble(i * 24 + 16)
    ));
    for (i in 0...Std.int(indexCount / 3)) {
      var triangle = [indices.getInt32(i * 12), indices.getInt32(i * 12 + 4), indices.getInt32(i * 12 + 8)];
      for (j in 0...3) {
        var a = triangle[j], b = triangle[(j + 1) % 3];
        if (a < 0 || b < 0 || a >= vertexCount || b >= vertexCount) throw "CAD feature edge index out of bounds";
        var sides = requested.get(pair(points[a], points[b]));
        if (sides == null) continue;
        // Order endpoint normals by position so opposite triangle winding cannot swap them.
        if (points[a] > points[b]) {
          var swap = a;
          a = b;
          b = swap;
        }
        var pairNormals:Array<Float> = [];
        for (v in [a, b]) for (axis in 0...3) pairNormals.push(normals.getDouble(v * 24 + axis * 8));
        sides.normals.push(pairNormals);
      }
    }
    var threshold = Math.cos(Math.PI / 180), result:Array<Bool> = [], features:Map<Int, Bool> = [];
    var identified = edgeIds != null && edgeIds.length == count * 4;
    for (i in 0...count) {
      var sides = requested.get(keys[i]), smooth = sides != null && sides.normals.length == 2;
      if (smooth && sides != null) for (endpoint in 0...2) {
        var a = sides.normals[0], b = sides.normals[1], o = endpoint * 3;
        var aa = a[o] * a[o] + a[o + 1] * a[o + 1] + a[o + 2] * a[o + 2];
        var bb = b[o] * b[o] + b[o + 1] * b[o + 1] + b[o + 2] * b[o + 2];
        var dot = a[o] * b[o] + a[o + 1] * b[o + 1] + a[o + 2] * b[o + 2];
        if (!Math.isFinite(dot + aa + bb)
          || aa < 1e-20 || bb < 1e-20 || dot / Math.sqrt(aa * bb) < threshold) smooth = false;
      }
      result.push(!smooth);
      if (!smooth && identified) features.set(edgeIds.getInt32(i * 4), true);
    }
    // Never draw only part of one BRep edge when an endpoint/triangulation is uncertain.
    if (identified) for (i in 0...count) if (features.exists(edgeIds.getInt32(i * 4))) result[i] = true;
    return result;
  }
}
