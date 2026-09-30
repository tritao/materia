package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Converts a code-generated CAD tessellation artifact into SceneKit geometry. */
class CadPreviewGeometry {
  /** Build generated geometry directly from the validated binary artifact. */
  public static function fromArtifact(part:SceneArtifactPart, minimum:Array<Float>,
      maximum:Array<Float>, scale:Float, centered:Bool = true,
      includeTopology:Bool = false):GeometryData {
    if (!finite(scale) || scale <= 0 || part.vertexCount <= 0 || part.indexCount <= 0 ||
        part.indexCount % 3 != 0 || part.vertices.length != part.vertexCount * 24 ||
        part.normals.length != part.vertexCount * 24 || part.indices.length != part.indexCount * 4)
      throw "CAD mesh has inconsistent streams";
    var geometry = new GeometryData();
    var positions = Bytes.alloc(part.vertexCount * 12);
    var normals = Bytes.alloc(part.vertexCount * 12);
    var centerX = centered ? (minimum[0] + maximum[0]) * 0.5 : 0.0;
    var centerY = centered ? (minimum[1] + maximum[1]) * 0.5 : 0.0;
    var centerZ = centered ? (minimum[2] + maximum[2]) * 0.5 : 0.0;
    for (index in 0...part.vertexCount) {
      var source = index * 24, target = index * 12;
      var x = (part.vertices.getDouble(source) - centerX) * scale;
      var y = (part.vertices.getDouble(source + 8) - centerY) * scale;
      var z = (part.vertices.getDouble(source + 16) - centerZ) * scale;
      var nx = part.normals.getDouble(source), ny = part.normals.getDouble(source + 8),
        nz = part.normals.getDouble(source + 16);
      if (!finite(x) || !finite(y) || !finite(z) || !finite(nx) || !finite(ny) || !finite(nz))
        throw "CAD mesh contains a non-finite vertex or normal";
      if (includeTopology) geometry.addVertex(x, y, z);
      positions.setFloat(target, x); positions.setFloat(target + 4, y); positions.setFloat(target + 8, z);
      normals.setFloat(target, nx); normals.setFloat(target + 4, ny); normals.setFloat(target + 8, nz);
    }
    for (triangle in 0...Std.int(part.indexCount / 3)) {
      var offset = triangle * 12;
      var first = part.indices.getInt32(offset), second = part.indices.getInt32(offset + 4),
        third = part.indices.getInt32(offset + 8);
      if (first < 0 || second < 0 || third < 0 || first >= part.vertexCount ||
          second >= part.vertexCount || third >= part.vertexCount)
        throw "CAD mesh has an out-of-range triangle index";
      if (includeTopology) geometry.addTriangle(first, second, third);
    }
    geometry.addStream(1, 2, positions, part.vertexCount, 12);
    geometry.addStream(2, 2, normals, part.vertexCount, 12);
    if (!includeTopology) geometry.setIndexBuffer(part.indices, part.indexCount);
    for (range in part.faceRanges) {
      if (range.firstIndex < 0 || range.indexCount < 0 || range.firstIndex % 3 != 0 ||
          range.indexCount % 3 != 0 || range.firstIndex + range.indexCount > part.indexCount)
        throw "CAD mesh has an invalid face range";
      geometry.addSubelement(Std.int(range.firstIndex / 3), Std.int(range.indexCount / 3), range.faceIndex);
    }
    var edges = part.edgeSegments;
    var edgeIds = part.edgeIds;
    if (edges != null && edges.length % 48 != 0) throw "CAD mesh has invalid edges";
    if (edgeIds != null && edgeIds.length != 0 &&
        (edges == null || edgeIds.length != Std.int(edges.length / 48) * 4))
      throw "CAD mesh has invalid edge IDs";
    if (edges != null) for (index in 0...Std.int(edges.length / 48)) {
      var offset = index * 48;
      for (axis in 0...6) if (!finite(edges.getDouble(offset + axis * 8)))
        throw "CAD mesh has a non-finite edge endpoint";
      geometry.addStrokeSegment(
        (edges.getDouble(offset) - centerX) * scale,
        (edges.getDouble(offset + 8) - centerY) * scale,
        (edges.getDouble(offset + 16) - centerZ) * scale,
        (edges.getDouble(offset + 24) - centerX) * scale,
        (edges.getDouble(offset + 32) - centerY) * scale,
        (edges.getDouble(offset + 40) - centerZ) * scale,
        edgeIds == null || edgeIds.length == 0 ? -1 : edgeIds.getInt32(index * 4));
    }
    geometry.setBounds((minimum[0] - centerX) * scale,
      (minimum[1] - centerY) * scale, (minimum[2] - centerZ) * scale,
      (maximum[0] - centerX) * scale, (maximum[1] - centerY) * scale,
      (maximum[2] - centerZ) * scale);
    return geometry;
  }

  static inline function finite(value:Float):Bool return value == value && value - value == 0.0;
}
