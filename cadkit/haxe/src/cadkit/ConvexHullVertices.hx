package cadkit;

import haxe.io.Bytes;

typedef HullPlane = {x:Float, y:Float, z:Float, support:Float};

/** Conservative convex outer hull of a tessellated part, in CAD units. */
class ConvexHullVertices {
  // The 26 support planes yield at most 2 * 26 - 4 = 48 vertices.
  public static inline final MAX_VERTICES:Int = 48;

  public static function fromMesh(vertices:Bytes, count:Int):Array<Float> {
    if (vertices == null || count < 4 || vertices.length < count * 24)
      throw "Convex hull needs at least four 3D mesh vertices";
    var planes:Array<HullPlane> = [];
    var scale = 1.0;
    for (ix in 0...3) for (iy in 0...3) for (iz in 0...3) {
      var nx = ix - 1, ny = iy - 1, nz = iz - 1;
      if (nx == 0 && ny == 0 && nz == 0) continue;
      var support = Math.NEGATIVE_INFINITY;
      for (vertex in 0...count) {
        var at = vertex * 24;
        var x = vertices.getDouble(at), y = vertices.getDouble(at + 8),
          z = vertices.getDouble(at + 16);
        if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z))
          throw "Convex hull mesh has a non-finite vertex";
        scale = Math.max(scale, Math.max(Math.abs(x), Math.max(Math.abs(y), Math.abs(z))));
        support = Math.max(support, nx * x + ny * y + nz * z);
      }
      planes.push({x: nx, y: ny, z: nz, support: support});
    }
    var result:Array<Float> = [];
    var tolerance = scale * 1e-8;
    for (i in 0...planes.length) for (j in i + 1...planes.length)
      for (k in j + 1...planes.length) {
        var a = planes[i], b = planes[j], c = planes[k];
        var bcX = b.y * c.z - b.z * c.y;
        var bcY = b.z * c.x - b.x * c.z;
        var bcZ = b.x * c.y - b.y * c.x;
        var caX = c.y * a.z - c.z * a.y;
        var caY = c.z * a.x - c.x * a.z;
        var caZ = c.x * a.y - c.y * a.x;
        var abX = a.y * b.z - a.z * b.y;
        var abY = a.z * b.x - a.x * b.z;
        var abZ = a.x * b.y - a.y * b.x;
        var determinant = a.x * bcX + a.y * bcY + a.z * bcZ;
        if (Math.abs(determinant) < 1e-12) continue;
        var x = (a.support * bcX + b.support * caX + c.support * abX) / determinant;
        var y = (a.support * bcY + b.support * caY + c.support * abY) / determinant;
        var z = (a.support * bcZ + b.support * caZ + c.support * abZ) / determinant;
        var inside = true;
        for (plane in planes)
          if (plane.x * x + plane.y * y + plane.z * z > plane.support + tolerance) {
            inside = false;
            break;
          }
        if (!inside) continue;
        var duplicate = false;
        for (existing in 0...Std.int(result.length / 3)) {
          var at = existing * 3;
          if (Math.abs(result[at] - x) <= tolerance &&
              Math.abs(result[at + 1] - y) <= tolerance &&
              Math.abs(result[at + 2] - z) <= tolerance) {
            duplicate = true;
            break;
          }
        }
        if (!duplicate) { result.push(x); result.push(y); result.push(z); }
      }
    if (result.length < 12 || result.length > MAX_VERTICES * 3)
      throw "Convex hull is degenerate or exceeds its vertex cap";
    return result;
  }
}
