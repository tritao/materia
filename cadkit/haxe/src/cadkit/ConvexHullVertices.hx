package cadkit;

import haxe.io.Bytes;

typedef HullPlane = {x:Float, y:Float, z:Float, support:Float};
typedef CollisionHullResult = {vertices:Array<Float>, warning:Null<String>};

/** Conservative convex outer hull of a tessellated part, in CAD units. */
class ConvexHullVertices {
  // The 26 support planes yield at most 2 * 26 - 4 = 48 vertices.
  public static inline final MAX_VERTICES:Int = 48;

  /** Keeps a thin or degenerate part loadable with a minimum-thickness box. */
  public static function safeFromMesh(vertices:Bytes, count:Int,
      minimumThickness:Float):CollisionHullResult {
    if (vertices == null || count < 1 || vertices.length < count * 24 ||
        !Math.isFinite(minimumThickness) || minimumThickness <= 0)
      throw "Collision mesh data or minimum thickness is invalid";
    var minimum = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
    var maximum = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
    for (index in 0...count) for (axis in 0...3) {
      var value = vertices.getDouble(index * 24 + axis * 8);
      if (!Math.isFinite(value)) throw "Convex hull mesh has a non-finite vertex";
      minimum[axis] = Math.min(minimum[axis], value);
      maximum[axis] = Math.max(maximum[axis], value);
    }
    var diagonal = Math.sqrt(Math.pow(maximum[0] - minimum[0], 2) +
      Math.pow(maximum[1] - minimum[1], 2) + Math.pow(maximum[2] - minimum[2], 2));
    var origin = [for (axis in 0...3) vertices.getDouble(axis * 8)];
    var first = -1, longest = 0.0;
    for (index in 1...count) {
      var distance = 0.0;
      for (axis in 0...3) distance += Math.pow(vertices.getDouble(index * 24 + axis * 8) - origin[axis], 2);
      if (distance > longest) { longest = distance; first = index; }
    }
    var flat = first < 0 || longest <= diagonal * diagonal * 1e-16;
    if (!flat) {
      var direction = [for (axis in 0...3) vertices.getDouble(first * 24 + axis * 8) - origin[axis]];
      var normal = [0.0, 0.0, 0.0], widest = 0.0;
      for (index in 1...count) {
        var delta = [for (axis in 0...3) vertices.getDouble(index * 24 + axis * 8) - origin[axis]];
        var cross = [direction[1] * delta[2] - direction[2] * delta[1],
          direction[2] * delta[0] - direction[0] * delta[2],
          direction[0] * delta[1] - direction[1] * delta[0]];
        var area = cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2];
        if (area > widest) { widest = area; normal = cross; }
      }
      flat = widest <= diagonal * diagonal * diagonal * diagonal * 1e-16;
      if (!flat) {
        var maximumHeight = 0.0;
        for (index in 1...count) {
          var projection = 0.0;
          for (axis in 0...3) projection += normal[axis] *
            (vertices.getDouble(index * 24 + axis * 8) - origin[axis]);
          maximumHeight = Math.max(maximumHeight, Math.abs(projection) / Math.sqrt(widest));
        }
        flat = maximumHeight <= Math.max(minimumThickness * 1e-6, diagonal * 1e-8);
      }
    }
    if (!flat) {
      try {
        return {vertices: fromMesh(vertices, count), warning: null};
      } catch (_:Dynamic) {}
    }
    for (axis in 0...3) if (maximum[axis] - minimum[axis] < minimumThickness) {
      var center = (minimum[axis] + maximum[axis]) * 0.5;
      minimum[axis] = center - minimumThickness * 0.5;
      maximum[axis] = center + minimumThickness * 0.5;
    }
    var box:Array<Float> = [];
    for (index in 0...8) for (axis in 0...3)
      box.push((index & (1 << axis)) == 0 ? minimum[axis] : maximum[axis]);
    return {vertices: box, warning: flat ?
      "Flat collision mesh uses a thickened box" :
      "Collision hull could not be sampled; using a bounding box"};
  }

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
