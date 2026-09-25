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
      maximum:Array<Float>, scale:Float):GeometryData {
    var geometry = new GeometryData();
    var positions = Bytes.alloc(part.vertexCount * 12);
    var normals = Bytes.alloc(part.vertexCount * 12);
    var centerX = (minimum[0] + maximum[0]) * 0.5;
    var centerY = (minimum[1] + maximum[1]) * 0.5;
    var centerZ = (minimum[2] + maximum[2]) * 0.5;
    for (index in 0...part.vertexCount) {
      var source = index * 24, target = index * 12;
      positions.setFloat(target, (part.vertices.getDouble(source) - centerX) * scale);
      positions.setFloat(target + 4, (part.vertices.getDouble(source + 8) - centerY) * scale);
      positions.setFloat(target + 8, (part.vertices.getDouble(source + 16) - centerZ) * scale);
      normals.setFloat(target, part.normals.getDouble(source));
      normals.setFloat(target + 4, part.normals.getDouble(source + 8));
      normals.setFloat(target + 8, part.normals.getDouble(source + 16));
    }
    geometry.addStream(1, 2, positions, part.vertexCount, 12);
    geometry.addStream(2, 2, normals, part.vertexCount, 12);
    geometry.setIndexBuffer(part.indices, part.indexCount);
    for (range in part.faceRanges)
      geometry.addSubelement(Std.int(range.firstIndex / 3), Std.int(range.indexCount / 3), range.faceIndex);
    var edges = part.edgeSegments;
    var edgeIds = part.edgeIds;
    if (edges != null) for (index in 0...Std.int(edges.length / 48)) {
      var offset = index * 48;
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
    var centerX = (minimum[0] + maximum[0]) / 2.0;
    var centerY = (minimum[1] + maximum[1]) / 2.0;
    var centerZ = (minimum[2] + maximum[2]) / 2.0;
    var geometry = new GeometryData();
    var positionStream = Bytes.alloc(vertexCount * 12);
    var normalStream = Bytes.alloc(vertexCount * 12);
    var actualMinimum = [1e300, 1e300, 1e300], actualMaximum = [-1e300, -1e300, -1e300];
    for (index in 0...vertexCount) {
      var offset = index * 24;
      var rawX = vertices.getDouble(offset), rawY = vertices.getDouble(offset + 8), rawZ = vertices.getDouble(offset + 16);
      if (!finite(rawX) || !finite(rawY) || !finite(rawZ))
        throw "CAD preview snapshot contains a non-finite vertex";
      actualMinimum[0] = Math.min(actualMinimum[0], rawX);
      actualMinimum[1] = Math.min(actualMinimum[1], rawY);
      actualMinimum[2] = Math.min(actualMinimum[2], rawZ);
      actualMaximum[0] = Math.max(actualMaximum[0], rawX);
      actualMaximum[1] = Math.max(actualMaximum[1], rawY);
      actualMaximum[2] = Math.max(actualMaximum[2], rawZ);
      var x = (rawX - centerX) * scale;
      var y = (rawY - centerY) * scale;
      var z = (rawZ - centerZ) * scale;
      var nx = normals.getDouble(offset), ny = normals.getDouble(offset + 8), nz = normals.getDouble(offset + 16);
      if (!finite(x) || !finite(y) || !finite(z) || !finite(nx) || !finite(ny) || !finite(nz))
        throw "CAD preview snapshot contains a non-finite vertex or normal";
      positionStream.setFloat(index * 12, x);
      positionStream.setFloat(index * 12 + 4, y);
      positionStream.setFloat(index * 12 + 8, z);
      normalStream.setFloat(index * 12, nx);
      normalStream.setFloat(index * 12 + 4, ny);
      normalStream.setFloat(index * 12 + 8, nz);
    }
    for (axis in 0...3) if (Math.abs(actualMinimum[axis] - minimum[axis]) > 1e-6 ||
        Math.abs(actualMaximum[axis] - maximum[axis]) > 1e-6)
      throw "CAD preview snapshot bounds do not match its vertices";
    for (triangle in 0...triangleCount) {
      var offset = triangle * 12;
      var first = indices.getInt32(offset), second = indices.getInt32(offset + 4), third = indices.getInt32(offset + 8);
      if (first < 0 || second < 0 || third < 0 || first >= vertexCount || second >= vertexCount || third >= vertexCount)
        throw "CAD preview snapshot has an out-of-range triangle index";
    }
    geometry.addStream(1, 2, positionStream, vertexCount, 12);
    geometry.addStream(2, 2, normalStream, vertexCount, 12);
    geometry.setIndexBuffer(indices, totalIndexCount);
    var faceRanges:Array<String> = fields[5 + shift].length == 0 ? [] : fields[5 + shift].split(";");
    if (faceRanges.length > 100000) throw "CAD preview snapshot has invalid face ranges";
    for (range in faceRanges) {
      var values = range.split(",");
      if (values.length != 3) throw "CAD preview snapshot has invalid face ranges";
      var faceIndex = parseInt(values[0]), firstIndex = parseInt(values[1]), indexCount = parseInt(values[2]);
      if (faceIndex < 0 || firstIndex < 0 || indexCount < 0 || firstIndex % 3 != 0 || indexCount % 3 != 0 ||
          firstIndex + indexCount > totalIndexCount)
        throw "CAD preview snapshot has an invalid face range";
      geometry.addSubelement(Std.int(firstIndex / 3), Std.int(indexCount / 3), faceIndex);
    }
    for (index in 0...Std.int(edges.length / 48)) {
      var offset = index * 48;
      var x0 = edges.getDouble(offset), y0 = edges.getDouble(offset + 8), z0 = edges.getDouble(offset + 16);
      var x1 = edges.getDouble(offset + 24), y1 = edges.getDouble(offset + 32), z1 = edges.getDouble(offset + 40);
      if (!finite(x0) || !finite(y0) || !finite(z0) || !finite(x1) || !finite(y1) || !finite(z1))
        throw "CAD preview snapshot contains a non-finite edge endpoint";
      geometry.addStrokeSegment((x0 - centerX) * scale, (y0 - centerY) * scale, (z0 - centerZ) * scale,
        (x1 - centerX) * scale, (y1 - centerY) * scale, (z1 - centerZ) * scale);
    }
    geometry.setBounds((minimum[0] - centerX) * scale,
      (minimum[1] - centerY) * scale,
      (minimum[2] - centerZ) * scale,
      (maximum[0] - centerX) * scale,
      (maximum[1] - centerY) * scale,
      (maximum[2] - centerZ) * scale);
    return geometry;
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
