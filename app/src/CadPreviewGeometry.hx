package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;

/** Converts a code-generated CAD tessellation snapshot into SceneKit geometry. */
class CadPreviewGeometry {
  public static inline var METRES_PER_MILLIMETRE:Float = 0.001;
  static inline var MAX_VERTICES:Int = 2000000;
  static inline var MAX_TRIANGLES:Int = 4000000;

  public static function geometry(snapshot:String):GeometryData {
    if (snapshot == null || snapshot.length == 0 || snapshot.length > 50000000)
      throw "CAD preview snapshot is empty or too large";
    var fields = snapshot.split("|");
    if (fields.length != 9 || fields[0] != "materia.geometry-preview/2")
      throw "Unsupported CAD preview snapshot format";
    var vertexCount = parseInt(fields[1]), totalIndexCount = parseInt(fields[2]);
    if (vertexCount <= 0 || vertexCount > MAX_VERTICES || totalIndexCount <= 0 || totalIndexCount % 3 != 0 ||
        totalIndexCount / 3 > MAX_TRIANGLES)
      throw "CAD preview snapshot has inconsistent mesh streams";
    var vertices = MateriaBase64.decode(fields[6], MAX_VERTICES * 24);
    var normals = MateriaBase64.decode(fields[7], MAX_VERTICES * 24);
    var indices = MateriaBase64.decode(fields[8], MAX_TRIANGLES * 12);
    if (vertices.length != vertexCount * 24 || normals.length != vertexCount * 24 ||
        indices.length != totalIndexCount * 4)
      throw "CAD preview snapshot has inconsistent mesh streams";

    var triangleCount = Std.int(totalIndexCount / 3);
    var minimum = numberTriple(fields[3]), maximum = numberTriple(fields[4]);
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
      var x = (rawX - centerX) * METRES_PER_MILLIMETRE;
      var y = (rawY - centerY) * METRES_PER_MILLIMETRE;
      var z = (rawZ - centerZ) * METRES_PER_MILLIMETRE;
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
    var faceRanges:Array<String> = fields[5].length == 0 ? [] : fields[5].split(";");
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
    geometry.setBounds((minimum[0] - centerX) * METRES_PER_MILLIMETRE,
      (minimum[1] - centerY) * METRES_PER_MILLIMETRE,
      (minimum[2] - centerZ) * METRES_PER_MILLIMETRE,
      (maximum[0] - centerX) * METRES_PER_MILLIMETRE,
      (maximum[1] - centerY) * METRES_PER_MILLIMETRE,
      (maximum[2] - centerZ) * METRES_PER_MILLIMETRE);
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
