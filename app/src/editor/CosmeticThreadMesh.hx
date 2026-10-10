package app.editor;

import app.CadGeometryData.CadThreadGeometry;
import cadkit.modeling.Vector;
import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import materia.project.ThreadSpecData.ThreadHand;

private typedef ThreadVertex = {var point:Vector; var normal:Vector; var u:Float; var v:Float; var depth:Float; var sourceIndex:Int; var seam:Bool;}

/** Clip and unwrap a face's own triangles; never ask the kernel for another tessellation. */
class CosmeticThreadMesh {
  public static function phase(name:String):Float {
    var hash = -2128831035;
    for (index in 0...name.length) hash = (hash ^ name.charCodeAt(index)) * 16777619;
    return (hash & 0x7fffffff) / 2147483648.0;
  }

  public static function build(base:GeometryData, thread:CadThreadGeometry):GeometryData {
    var face = base.subelementGeometry(thread.source.faceIndex);
    if (face == null) throw 'Thread ${thread.source.faceName} has no base face triangles';
    var positions = face.stream(1), normals = face.stream(2);
    if (positions == null || normals == null || positions.format != 2 || normals.format != 2)
      throw 'Thread ${thread.source.faceName} needs Float32 position and normal streams';
    var result = new GeometryData();
    var points:Array<Vector> = [], normalValues:Array<Vector> = [], coordinates:Array<Float> = [];
    var vertexMap:Map<Int, Int> = [];
    var phaseOffset = phase(thread.source.faceName);
    var starts = thread.source.spec.starts;
    var hand = thread.source.spec.hand == LeftHand ? -1 : 1;
    for (triangle in 0...face.triangleCount()) {
      var polygon:Array<ThreadVertex> = [];
      for (corner in 0...3) {
        var index = face.indexAt(triangle * 3 + corner);
        var point = read(positions.bytes, index * (positions.stride == 0 ? 12 : positions.stride));
        var normal = read(normals.bytes, index * (normals.stride == 0 ? 12 : normals.stride));
        var local = thread.frame.toLocal(point);
        var depth = local.z;
        if (Math.abs(depth) < 1e-8) depth = 0;
        if (Math.abs(depth - thread.length) < 1e-8) depth = thread.length;
        var angle = Math.atan2(local.y, local.x) / (2 * Math.PI);
        if (angle < 0) angle += 1;
        polygon.push({point: point, normal: normal, u: angle, v: depth / thread.pitch, depth: depth, sourceIndex: index, seam: false});
      }
      var minimum = Math.min(polygon[0].u, Math.min(polygon[1].u, polygon[2].u));
      var maximum = Math.max(polygon[0].u, Math.max(polygon[1].u, polygon[2].u));
      // Unwrap before multiplying by starts: a seam shift spans 'starts' texture periods.
      for (vertex in polygon) {
        if (maximum - minimum > 0.5 && vertex.u < 0.5) { vertex.u += 1; vertex.seam = true; }
        vertex.u = vertex.u * starts * hand + phaseOffset;
      }
      if (thread.source.ringAtStart) ring(result, polygon, 0);
      if (thread.source.ringAtEnd) ring(result, polygon, thread.length);
      polygon = clip(clip(polygon, 0, true), thread.length, false);
      if (polygon.length < 3) continue;
      var polygonIndices:Array<Int> = [];
      for (vertex in polygon) {
        var key = vertex.sourceIndex * 2 + (vertex.seam ? 1 : 0);
        var existing = vertex.sourceIndex < 0 ? null : vertexMap.get(key);
        var index:Int;
        if (existing == null) {
          index = points.length;
          points.push(vertex.point); normalValues.push(vertex.normal); coordinates.push(vertex.u); coordinates.push(vertex.v);
          if (vertex.sourceIndex >= 0) vertexMap.set(key, index);
        } else index = existing;
        polygonIndices.push(index);
      }
      for (index in 1...polygon.length - 1) result.addTriangle(polygonIndices[0], polygonIndices[index], polygonIndices[index + 1]);
    }
    var positionBytes = Bytes.alloc(points.length * 12), normalBytes = Bytes.alloc(points.length * 12);
    var uvBytes = Bytes.alloc(points.length * 8);
    var low = [1e300, 1e300, 1e300], high = [-1e300, -1e300, -1e300];
    for (index in 0...points.length) {
      var point = points[index], normal = normalValues[index];
      var xyz = [point.x, point.y, point.z], nxyz = [normal.x, normal.y, normal.z];
      for (axis in 0...3) {
        positionBytes.setFloat(index * 12 + axis * 4, xyz[axis]);
        normalBytes.setFloat(index * 12 + axis * 4, nxyz[axis]);
        low[axis] = Math.min(low[axis], xyz[axis]); high[axis] = Math.max(high[axis], xyz[axis]);
      }
      uvBytes.setFloat(index * 8, coordinates[index * 2]); uvBytes.setFloat(index * 8 + 4, coordinates[index * 2 + 1]);
    }
    result.addStream(1, 2, positionBytes, points.length, 12);
    result.addStream(2, 2, normalBytes, points.length, 12);
    result.addStream(4, 1, uvBytes, points.length, 8);
    result.addSubelement(0, result.triangleCount(), thread.source.faceIndex);
    if (points.length > 0) result.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
    return result;
  }

  static function read(bytes:Bytes, offset:Int):Vector
    return new Vector(bytes.getFloat(offset), bytes.getFloat(offset + 4), bytes.getFloat(offset + 8));

  static function interpolate(a:ThreadVertex, b:ThreadVertex, depth:Float):ThreadVertex {
    var amount = (depth - a.depth) / (b.depth - a.depth);
    return {point: a.point.add(b.point.subtract(a.point).scale(amount)),
      normal: a.normal.add(b.normal.subtract(a.normal).scale(amount)).normalized(),
      u: a.u + (b.u - a.u) * amount, v: a.v + (b.v - a.v) * amount, depth: depth, sourceIndex: -1, seam: false};
  }

  static function clip(polygon:Array<ThreadVertex>, depth:Float, greater:Bool):Array<ThreadVertex> {
    var result:Array<ThreadVertex> = [];
    if (polygon.length == 0) return result;
    var previous = polygon[polygon.length - 1];
    for (current in polygon) {
      var before = greater ? previous.depth >= depth : previous.depth <= depth;
      var after = greater ? current.depth >= depth : current.depth <= depth;
      if (before != after) result.push(interpolate(previous, current, depth));
      if (after) result.push(current);
      previous = current;
    }
    return result;
  }

  /** Intersection with an interior depth plane draws the cosmetic ring through the existing stroke pass. */
  static function ring(result:GeometryData, polygon:Array<ThreadVertex>, depth:Float):Void {
    var points:Array<Vector> = [];
    var previous = polygon[polygon.length - 1];
    for (current in polygon) {
      if ((previous.depth < depth && current.depth >= depth) || (current.depth < depth && previous.depth >= depth))
        points.push(interpolate(previous, current, depth).point);
      previous = current;
    }
    if (points.length == 2 && points[0].subtract(points[1]).length() > 1e-10)
      result.addStrokeSegment(points[0].x, points[0].y, points[0].z, points[1].x, points[1].y, points[1].z);
  }
}
