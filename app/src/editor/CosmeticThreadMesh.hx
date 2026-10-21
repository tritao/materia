package app.editor;

import app.CadGeometryData.CadThreadGeometry;
import cadkit.threads.ThreadSurface;
import haxe.io.Bytes;
import nativekit.scene.GeometryData;

/** A threaded face's overlay geometry for the scene: CadKit's ThreadSurface over the base mesh's own triangles. */
class CosmeticThreadMesh {
  public static function phase(name:String):Float return ThreadSurface.phase(name);

  public static function build(base:GeometryData, thread:CadThreadGeometry):GeometryData {
    var face = base.subelementGeometry(thread.source.faceIndex);
    if (face == null) throw 'Thread ${thread.source.faceName} has no base face triangles';
    var positions = face.stream(1), normals = face.stream(2);
    if (positions == null || normals == null || positions.format != 2 || normals.format != 2)
      throw 'Thread ${thread.source.faceName} needs Float32 position and normal streams';
    var points:Array<Float> = [], normalValues:Array<Float> = [], triangles:Array<Int> = [];
    var positionStride = positions.stride == 0 ? 12 : positions.stride, normalStride = normals.stride == 0 ? 12 : normals.stride;
    var vertexCount = 0;
    for (index in 0...face.triangleCount() * 3) vertexCount = Std.int(Math.max(vertexCount, face.indexAt(index) + 1));
    for (vertex in 0...vertexCount) for (axis in 0...3) {
      points.push(positions.bytes.getFloat(vertex * positionStride + axis * 4));
      normalValues.push(normals.bytes.getFloat(vertex * normalStride + axis * 4));
    }
    for (index in 0...face.triangleCount() * 3) triangles.push(face.indexAt(index));
    var surface = ThreadSurface.unwrap(thread.source.faceName, thread.source.spec, thread.frame, thread.length, thread.pitch,
      thread.source.ringAtStart, thread.source.ringAtEnd, points, normalValues, triangles);
    var result = new GeometryData();
    for (triangle in 0...Std.int(surface.triangles.length / 3))
      result.addTriangle(surface.triangles[triangle * 3], surface.triangles[triangle * 3 + 1], surface.triangles[triangle * 3 + 2]);
    var rings = surface.rings;
    for (segment in 0...Std.int(rings.length / 6))
      result.addStrokeSegment(rings[segment * 6], rings[segment * 6 + 1], rings[segment * 6 + 2],
        rings[segment * 6 + 3], rings[segment * 6 + 4], rings[segment * 6 + 5]);
    var count = surface.vertexCount;
    var positionBytes = Bytes.alloc(count * 12), normalBytes = Bytes.alloc(count * 12), uvBytes = Bytes.alloc(count * 8);
    var low = [1e300, 1e300, 1e300], high = [-1e300, -1e300, -1e300];
    for (index in 0...count) {
      for (axis in 0...3) {
        var value = surface.positions[index * 3 + axis];
        positionBytes.setFloat(index * 12 + axis * 4, value);
        normalBytes.setFloat(index * 12 + axis * 4, surface.normals[index * 3 + axis]);
        low[axis] = Math.min(low[axis], value); high[axis] = Math.max(high[axis], value);
      }
      uvBytes.setFloat(index * 8, surface.uvs[index * 2]); uvBytes.setFloat(index * 8 + 4, surface.uvs[index * 2 + 1]);
    }
    result.addStream(1, 2, positionBytes, count, 12);
    result.addStream(2, 2, normalBytes, count, 12);
    result.addStream(4, 1, uvBytes, count, 8);
    result.addSubelement(0, result.triangleCount(), thread.source.faceIndex);
    if (count > 0) result.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
    return result;
  }
}
