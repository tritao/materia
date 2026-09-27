package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Converts a code-generated CAD tessellation snapshot into SceneKit geometry. */
class CadPreviewGeometry {
  public static inline var METRES_PER_MILLIMETRE:Float = 0.001;
  static inline var MAX_VERTICES:Int = 2000000;
  static inline var MAX_TRIANGLES:Int = 4000000;

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

  public static function geometry(snapshot:String):GeometryData {
    if (snapshot == null || snapshot.length == 0 || snapshot.length > 50000000)
      throw "CAD preview snapshot is empty or too large";
    var fields = snapshot.split("|");
    var legacy = fields.length == 9 && fields[0] == "materia.geometry-preview/2";
    var current = fields.length == 10 && fields[0] == "materia.geometry-preview/3";
    var withEdges = fields.length == 11 && fields[0] == "materia.geometry-preview/4";
    if (!legacy && !current && !withEdges)
      throw "Unsupported CAD preview snapshot format";
    var shift = legacy ? 0 : 1;
    var scale = legacy ? METRES_PER_MILLIMETRE : Std.parseFloat(fields[1]);
    if (!finite(scale) || scale <= 0.0) throw "CAD preview snapshot has invalid units";
    var vertexCount = parseInt(fields[1 + shift]), totalIndexCount = parseInt(fields[2 + shift]);
    if (vertexCount <= 0 || vertexCount > MAX_VERTICES || totalIndexCount <= 0 || totalIndexCount % 3 != 0 ||
        totalIndexCount / 3 > MAX_TRIANGLES)
      throw "CAD preview snapshot has inconsistent mesh streams";
    var vertices = MateriaBase64.decode(fields[6 + shift], MAX_VERTICES * 24);
    var normals = MateriaBase64.decode(fields[7 + shift], MAX_VERTICES * 24);
    var indices = MateriaBase64.decode(fields[8 + shift], MAX_TRIANGLES * 12);
    var edges = withEdges ? MateriaBase64.decode(fields[10], 50000000) : Bytes.alloc(0);
    if (vertices.length != vertexCount * 24 || normals.length != vertexCount * 24 ||
        indices.length != totalIndexCount * 4 || edges.length % 48 != 0)
      throw "CAD preview snapshot has inconsistent mesh streams";

    var triangleCount = Std.int(totalIndexCount / 3);
    var minimum = numberTriple(fields[3 + shift]), maximum = numberTriple(fields[4 + shift]);
    for (axis in 0...3) if (minimum[axis] > maximum[axis])
      throw "CAD preview snapshot has invalid bounds";
    var actualMinimum = [1e300, 1e300, 1e300], actualMaximum = [-1e300, -1e300, -1e300];
    for (index in 0...vertexCount) for (axis in 0...3) {
      var value = vertices.getDouble(index * 24 + axis * 8);
      if (!finite(value) || !finite(normals.getDouble(index * 24 + axis * 8)))
        throw "CAD preview snapshot contains a non-finite vertex or normal";
      actualMinimum[axis] = Math.min(actualMinimum[axis], value);
      actualMaximum[axis] = Math.max(actualMaximum[axis], value);
    }
    for (axis in 0...3) if (Math.abs(actualMinimum[axis] - minimum[axis]) > 1e-6 ||
        Math.abs(actualMaximum[axis] - maximum[axis]) > 1e-6)
      throw "CAD preview snapshot bounds do not match its vertices";
    for (triangle in 0...triangleCount) for (corner in 0...3) {
      var vertex = indices.getInt32(triangle * 12 + corner * 4);
      if (vertex < 0 || vertex >= vertexCount)
        throw "CAD preview snapshot has an out-of-range triangle index";
    }
    var faceRanges:Array<String> = fields[5 + shift].length == 0 ? [] : fields[5 + shift].split(";");
    if (faceRanges.length > 100000) throw "CAD preview snapshot has invalid face ranges";
    var ranges:Array<materia.project.SceneArtifact.SceneArtifactFaceRange> = [];
    for (range in faceRanges) {
      var values = range.split(",");
      if (values.length != 3) throw "CAD preview snapshot has invalid face ranges";
      var faceIndex = parseInt(values[0]), firstIndex = parseInt(values[1]), indexCount = parseInt(values[2]);
      if (faceIndex < 0 || firstIndex < 0 || indexCount < 0 || firstIndex % 3 != 0 || indexCount % 3 != 0 ||
          firstIndex + indexCount > totalIndexCount)
        throw "CAD preview snapshot has an invalid face range";
      ranges.push({faceIndex: faceIndex, firstIndex: firstIndex, indexCount: indexCount});
    }
    for (index in 0...Std.int(edges.length / 8))
      if (!finite(edges.getDouble(index * 8)))
        throw "CAD preview snapshot contains a non-finite edge endpoint";
    var part:SceneArtifactPart = {id: "snapshot", name: "snapshot", red: 0.7, green: 0.7, blue: 0.7,
      vertexCount: vertexCount, indexCount: totalIndexCount, vertices: vertices,
      normals: normals, indices: indices, edgeSegments: edges, edgeIds: Bytes.alloc(0),
      faceRanges: ranges};
    return fromArtifact(part, minimum, maximum, scale);
  }

  static function parseInt(value:String):Int {
    var result = Std.parseInt(value);
    if (result == null) throw "CAD preview snapshot contains an invalid integer";
    return result;
  }

  static function numberTriple(value:String):Array<Float> {
    var values = value.split(",");
    if (values.length != 3) throw "Invalid CAD preview bounds";
    var result:Array<Float> = [];
    for (item in values) {
      var number = Std.parseFloat(item);
      if (!finite(number)) throw "Invalid CAD preview bounds";
      result.push(number);
    }
    return result;
  }

  static inline function finite(value:Float):Bool return value == value && value - value == 0.0;
}
