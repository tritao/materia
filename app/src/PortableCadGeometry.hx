package app;

import haxe.io.Bytes;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Portable vertex streams; bounds remain in the original CAD frame. */
typedef PreparedCadGeometry = {
  var minimum:Array<Float>;
  var maximum:Array<Float>;
  var positions:Bytes;
  var normals:Bytes;
};

class PortableCadGeometry {
  public static function prepare(part:SceneArtifactPart, minimum:Array<Float>, maximum:Array<Float>,
      scale:Float, centered:Bool = true):PreparedCadGeometry {
    if (!finite(scale) || scale <= 0 || part.vertexCount <= 0 || part.indexCount <= 0 ||
        part.indexCount % 3 != 0 || part.vertices.length != part.vertexCount * 24 ||
        part.normals.length != part.vertexCount * 24 || part.indices.length != part.indexCount * 4)
      throw "CAD mesh has inconsistent streams";
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
      positions.setFloat(target, x); positions.setFloat(target + 4, y); positions.setFloat(target + 8, z);
      normals.setFloat(target, nx); normals.setFloat(target + 4, ny); normals.setFloat(target + 8, nz);
    }
    return {minimum: minimum.copy(), maximum: maximum.copy(), positions: positions, normals: normals};
  }

  static inline function finite(value:Float):Bool return Math.isFinite(value);
}
