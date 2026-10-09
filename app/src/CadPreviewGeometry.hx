package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import materia.project.SceneArtifact.SceneArtifactPart;

import app.PortableCadGeometry.PreparedCadGeometry;

/** Converts a code-generated CAD tessellation artifact into SceneKit geometry. */
class CadPreviewGeometry {
  /** Build generated geometry directly from the validated binary artifact. */
  public static function fromArtifact(part:SceneArtifactPart, minimum:Array<Float>,
      maximum:Array<Float>, scale:Float, centered:Bool = true,
      includeTopology:Bool = false):GeometryData {
    return fromPrepared(part, prepare(part, minimum, maximum, scale, centered), scale, centered, includeTopology);
  }

  /** The expensive per-vertex conversion is independent of native geometry ownership. */
  public static function prepare(part:SceneArtifactPart, minimum:Array<Float>, maximum:Array<Float>,
      scale:Float, centered:Bool = true):PreparedCadGeometry {
    return PortableCadGeometry.prepare(part, minimum, maximum, scale, centered);
  }

  /** Reconstruct fresh native geometry from portable streams and the validated source artifact. */
  public static function fromPrepared(part:SceneArtifactPart, prepared:PreparedCadGeometry,
      scale:Float, centered:Bool = true, includeTopology:Bool = false):GeometryData {
    var minimum = prepared.minimum, maximum = prepared.maximum;
    if (prepared.positions.length != part.vertexCount * 12 || prepared.normals.length != part.vertexCount * 12)
      throw "Prepared CAD streams have invalid sizes";
    var positions = prepared.positions, normals = prepared.normals;
    var centerX = centered ? (minimum[0] + maximum[0]) * 0.5 : 0.0;
    var centerY = centered ? (minimum[1] + maximum[1]) * 0.5 : 0.0;
    var centerZ = centered ? (minimum[2] + maximum[2]) * 0.5 : 0.0;
    var geometry = new GeometryData();
    if (includeTopology) for (index in 0...part.vertexCount) {
      var source = index * 24;
      geometry.addVertex((part.vertices.getDouble(source) - centerX) * scale,
        (part.vertices.getDouble(source + 8) - centerY) * scale,
        (part.vertices.getDouble(source + 16) - centerZ) * scale);
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
