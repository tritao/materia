package cadkit;

import haxe.io.Bytes;

typedef CollisionHullResult = {vertices:Array<Float>, warning:Null<String>, errorRatio:Float};

/** Bounded support-point hull of a tessellated part, in CAD units. */
class ConvexHullVertices {
  public static inline final MAX_VERTICES:Int = 64;

  /** An enclosing 26-DOP built from exact mesh support in fixed directions. */
  public static function enclosingFromMesh(vertices:Bytes, count:Int,
      minimumThickness:Float):CollisionHullResult {
    if (vertices == null || count < 1 || vertices.length < count * 24 ||
        !Math.isFinite(minimumThickness) || minimumThickness <= 0)
      throw "Collision mesh data or minimum thickness is invalid";
    var directions:Array<Array<Float>> = [];
    var supports:Array<Float> = [];
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
    var flat = false;
    for (axis in 0...3) if (maximum[axis] - minimum[axis] < minimumThickness) flat = true;
    for (xi in 0...3) for (yi in 0...3) for (zi in 0...3) {
      var x = xi - 1, y = yi - 1, z = zi - 1;
      if (x == 0 && y == 0 && z == 0) continue;
      var length = Math.sqrt(x * x + y * y + z * z);
      var direction = [x / length, y / length, z / length];
      var support = Math.NEGATIVE_INFINITY;
      for (index in 0...count) {
        var at = index * 24;
        support = Math.max(support, direction[0] * vertices.getDouble(at) +
          direction[1] * vertices.getDouble(at + 8) + direction[2] * vertices.getDouble(at + 16));
      }
      directions.push(direction);
      // A small outward offset also gives oblique planar meshes full volume.
      supports.push(support + minimumThickness * 0.5);
    }
    var result:Array<Float> = [];
    var scale = Math.max(1.0, Math.max(diagonal, minimumThickness));
    for (i in 0...directions.length) for (j in i + 1...directions.length)
      for (k in j + 1...directions.length) {
        var a = directions[i], b = directions[j], c = directions[k];
        var bc = cross(b, c), ca = cross(c, a), ab = cross(a, b);
        var determinant = dot(a, bc);
        if (Math.abs(determinant) < 1e-10) continue;
        var point = [for (axis in 0...3)
          (supports[i] * bc[axis] + supports[j] * ca[axis] + supports[k] * ab[axis]) / determinant];
        var inside = true;
        for (plane in 0...directions.length)
          if (dot(directions[plane], point) > supports[plane] + scale * 1e-8) {
            inside = false;
            break;
          }
        if (!inside) continue;
        var duplicate = false;
        for (index in 0...Std.int(result.length / 3)) {
          var at = index * 3;
          if (Math.abs(result[at] - point[0]) <= scale * 1e-8 &&
              Math.abs(result[at + 1] - point[1]) <= scale * 1e-8 &&
              Math.abs(result[at + 2] - point[2]) <= scale * 1e-8) {
            duplicate = true;
            break;
          }
        }
        if (!duplicate) for (axis in 0...3) result.push(point[axis]);
      }
    if (result.length < 12 || result.length > MAX_VERTICES * 3)
      throw 'Enclosing k-DOP has ${Std.int(result.length / 3)} vertices';
    return {vertices: result, warning: flat ? "Thin collision mesh was thickened" : null,
      errorRatio: supportErrorRatio(vertices, count, result, diagonal)};
  }

  static function dot(a:Array<Float>, b:Array<Float>):Float
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

  static function cross(a:Array<Float>, b:Array<Float>):Array<Float>
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]];

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
        var hull = fromMesh(vertices, count);
        return {vertices: hull, warning: null,
          errorRatio: supportErrorRatio(vertices, count, hull, diagonal)};
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
    return {vertices: box, errorRatio: supportErrorRatio(vertices, count, box, diagonal), warning: flat ?
      "Flat collision mesh uses a thickened box" :
      "Collision hull could not be sampled; using a bounding box"};
  }

  /** Maximum sampled support excess of the enclosing hull, relative to mesh diagonal. */
  static function supportErrorRatio(mesh:Bytes, count:Int, hull:Array<Float>, diagonal:Float):Float {
    if (diagonal <= 0) return 0.0;
    var maximum = 0.0;
    for (sample in 0...128) {
      var z = 1.0 - 2.0 * (sample + 0.5) / 128.0;
      var radius = Math.sqrt(Math.max(0.0, 1.0 - z * z));
      var angle = sample * Math.PI * (3.0 - Math.sqrt(5.0));
      var x = radius * Math.cos(angle), y = radius * Math.sin(angle);
      var meshSupport = Math.NEGATIVE_INFINITY, hullSupport = Math.NEGATIVE_INFINITY;
      for (index in 0...count) {
        var at = index * 24;
        meshSupport = Math.max(meshSupport, x * mesh.getDouble(at) +
          y * mesh.getDouble(at + 8) + z * mesh.getDouble(at + 16));
      }
      for (index in 0...Std.int(hull.length / 3)) {
        var at = index * 3;
        hullSupport = Math.max(hullSupport, x * hull[at] + y * hull[at + 1] + z * hull[at + 2]);
      }
      maximum = Math.max(maximum, meshSupport - hullSupport);
    }
    return maximum / diagonal;
  }

  public static function fromMesh(vertices:Bytes, count:Int):Array<Float> {
    if (vertices == null || count < 4 || vertices.length < count * 24)
      throw "Convex hull needs at least four 3D mesh vertices";
    var result:Array<Float> = [];
    var selected = new Map<Int, Bool>();
    for (sample in 0...MAX_VERTICES) {
      var x:Float, y:Float, z:Float;
      if (sample < 6) {
        x = sample == 0 ? 1.0 : sample == 1 ? -1.0 : 0.0;
        y = sample == 2 ? 1.0 : sample == 3 ? -1.0 : 0.0;
        z = sample == 4 ? 1.0 : sample == 5 ? -1.0 : 0.0;
      } else {
        var step = sample - 6;
        z = 1.0 - 2.0 * (step + 0.5) / (MAX_VERTICES - 6);
        var radius = Math.sqrt(Math.max(0.0, 1.0 - z * z));
        var angle = step * Math.PI * (3.0 - Math.sqrt(5.0));
        x = radius * Math.cos(angle);
        y = radius * Math.sin(angle);
      }
      var best = -1, score = Math.NEGATIVE_INFINITY;
      for (index in 0...count) {
        var at = index * 24;
        var projected = x * vertices.getDouble(at) +
          y * vertices.getDouble(at + 8) + z * vertices.getDouble(at + 16);
        if (projected > score) { score = projected; best = index; }
      }
      if (!selected.exists(best)) {
        selected.set(best, true);
        var at = best * 24;
        for (axis in 0...3) result.push(vertices.getDouble(at + axis * 8));
      }
    }
    if (result.length < 12) throw "Convex hull vertices are degenerate";
    return result;
  }
}
