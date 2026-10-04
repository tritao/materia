package robotkit.tool;

/** The nearest point of a simplex to the origin, and which of the simplex's points it is made of. */
private typedef Nearest = {
  var x:Float;
  var y:Float;
  var z:Float;
  var keep:Array<Int>;
  /** The origin is inside the (tetrahedral) simplex. */
  var inside:Bool;
}

/**
 * The distance between two convex hulls, from their corners: the Gilbert-Johnson-Keerthi algorithm. It looks for the point of
 * the Minkowski difference of the two hulls nearest the origin, by repeatedly taking the difference's extreme point toward
 * the origin and finding the nearest point of the small simplex they make. A hull is its corners (any points whose hull it is),
 * a few dozen for a part's collision hull, so a query costs a few hundred operations however big the parts are.
 *
 * The distance is exact for hulls that are apart. Hulls that touch or overlap give zero (how deep they overlap is not worked
 * out). When a separating plane shows the hulls are farther apart than `enough`, the search stops with that plane's gap, a lower
 * bound of the distance: the caller asking whether bodies keep a margin need not know more.
 */
class ConvexDistance {
  public static inline var ITERATIONS:Int = 48;

  /** `a` and `b` are corners as x, y, z triples in one frame. */
  public static function between(a:Array<Float>, b:Array<Float>, enough:Float):Float {
    var countA = Std.int(a.length / 3), countB = Std.int(b.length / 3);
    if (countA == 0 || countB == 0) throw "Convex distance needs points on both hulls";
    var vx = a[0] - b[0], vy = a[1] - b[1], vz = a[2] - b[2];
    var simplex:Array<Array<Float>> = [];
    var separated = 0.0;
    for (iteration in 0...ITERATIONS) {
      var vv = vx * vx + vy * vy + vz * vz;
      if (vv < 1e-18) return 0.0;
      // The point of the difference farthest from the origin's side: a's extreme point away from v, minus b's toward it.
      var bestA = 0, highA = Math.NEGATIVE_INFINITY;
      for (i in 0...countA) {
        var d = -(a[3 * i] * vx + a[3 * i + 1] * vy + a[3 * i + 2] * vz);
        if (d > highA) {
          highA = d;
          bestA = i;
        }
      }
      var bestB = 0, highB = Math.NEGATIVE_INFINITY;
      for (i in 0...countB) {
        var d = b[3 * i] * vx + b[3 * i + 1] * vy + b[3 * i + 2] * vz;
        if (d > highB) {
          highB = d;
          bestB = i;
        }
      }
      var wx = a[3 * bestA] - b[3 * bestB], wy = a[3 * bestA + 1] - b[3 * bestB + 1], wz = a[3 * bestA + 2] - b[3 * bestB + 2];
      var vw = vx * wx + vy * wy + vz * wz;
      // v.w is the least v.p over the difference: when it is positive, the plane through w square to v separates the hulls.
      if (vw > 0.0) {
        separated = Math.max(separated, vw / Math.sqrt(vv));
        if (separated > enough) return separated;
      }
      if (vv - vw <= 1e-10 * vv) return Math.sqrt(vv);
      for (point in simplex) if (point[0] == wx && point[1] == wy && point[2] == wz) return Math.sqrt(vv);
      simplex.push([wx, wy, wz]);
      var near = nearest(simplex);
      if (near.inside) return 0.0;
      vx = near.x;
      vy = near.y;
      vz = near.z;
      simplex = [for (index in near.keep) simplex[index]];
    }
    // At the iteration bound, retain a proven lower bound; an unfinished simplex gives an upper bound that could miss a collision.
    return separated;
  }

  /** The nearest point to the origin of the simplex of one to four points. */
  static function nearest(p:Array<Array<Float>>):Nearest {
    return switch p.length {
      case 1: {x: p[0][0], y: p[0][1], z: p[0][2], keep: [0], inside: false};
      case 2: segment(p[0], p[1], 0, 1);
      case 3: triangle(p[0], p[1], p[2], 0, 1, 2);
      case _: tetrahedron(p);
    };
  }

  static function segment(a:Array<Float>, b:Array<Float>, ia:Int, ib:Int):Nearest {
    var abx = b[0] - a[0], aby = b[1] - a[1], abz = b[2] - a[2];
    var length2 = abx * abx + aby * aby + abz * abz;
    var t = length2 > 0.0 ? -(a[0] * abx + a[1] * aby + a[2] * abz) / length2 : 0.0;
    if (t <= 0.0) return {x: a[0], y: a[1], z: a[2], keep: [ia], inside: false};
    if (t >= 1.0) return {x: b[0], y: b[1], z: b[2], keep: [ib], inside: false};
    return {x: a[0] + abx * t, y: a[1] + aby * t, z: a[2] + abz * t, keep: [ia, ib], inside: false};
  }

  /** Ericson's closest point of a triangle to a point, here the origin. */
  static function triangle(a:Array<Float>, b:Array<Float>, c:Array<Float>, ia:Int, ib:Int, ic:Int):Nearest {
    var abx = b[0] - a[0], aby = b[1] - a[1], abz = b[2] - a[2];
    var acx = c[0] - a[0], acy = c[1] - a[1], acz = c[2] - a[2];
    var d1 = -(abx * a[0] + aby * a[1] + abz * a[2]), d2 = -(acx * a[0] + acy * a[1] + acz * a[2]);
    if (d1 <= 0.0 && d2 <= 0.0) return {x: a[0], y: a[1], z: a[2], keep: [ia], inside: false};
    var d3 = -(abx * b[0] + aby * b[1] + abz * b[2]), d4 = -(acx * b[0] + acy * b[1] + acz * b[2]);
    if (d3 >= 0.0 && d4 <= d3) return {x: b[0], y: b[1], z: b[2], keep: [ib], inside: false};
    var vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
      var v = d1 / (d1 - d3);
      return {x: a[0] + abx * v, y: a[1] + aby * v, z: a[2] + abz * v, keep: [ia, ib], inside: false};
    }
    var d5 = -(abx * c[0] + aby * c[1] + abz * c[2]), d6 = -(acx * c[0] + acy * c[1] + acz * c[2]);
    if (d6 >= 0.0 && d5 <= d6) return {x: c[0], y: c[1], z: c[2], keep: [ic], inside: false};
    var vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
      var w = d2 / (d2 - d6);
      return {x: a[0] + acx * w, y: a[1] + acy * w, z: a[2] + acz * w, keep: [ia, ic], inside: false};
    }
    var va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
      var w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      return {x: b[0] + (c[0] - b[0]) * w, y: b[1] + (c[1] - b[1]) * w, z: b[2] + (c[2] - b[2]) * w, keep: [ib, ic], inside: false};
    }
    var denominator = va + vb + vc;
    if (!(Math.abs(denominator) > 1e-30)) return {x: a[0], y: a[1], z: a[2], keep: [ia], inside: false};
    var v = vb / denominator, w = vc / denominator;
    return {x: a[0] + abx * v + acx * w, y: a[1] + aby * v + acy * w, z: a[2] + abz * v + acz * w, keep: [ia, ib, ic], inside: false};
  }

  /** The nearest point of a tetrahedron to the origin: zero (inside) when no face has the origin outside it, else the nearest of those faces (of all four when it is flat). */
  static function tetrahedron(p:Array<Array<Float>>):Nearest {
    var faces = [[0, 1, 2, 3], [0, 3, 1, 2], [0, 2, 3, 1], [1, 3, 2, 0]];
    var best:Null<Nearest> = null;
    var bestDistance = Math.POSITIVE_INFINITY;
    for (face in faces) {
      var a = p[face[0]], b = p[face[1]], c = p[face[2]], d = p[face[3]];
      var abx = b[0] - a[0], aby = b[1] - a[1], abz = b[2] - a[2];
      var acx = c[0] - a[0], acy = c[1] - a[1], acz = c[2] - a[2];
      var nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
      // Which side of the face's plane the fourth point and the origin are on.
      var sideD = nx * (d[0] - a[0]) + ny * (d[1] - a[1]) + nz * (d[2] - a[2]);
      var sideO = nx * (-a[0]) + ny * (-a[1]) + nz * (-a[2]);
      // A flat tetrahedron (four coplanar points, as symmetric hulls give) has no inside: every face counts.
      var dx = d[0] - a[0], dy = d[1] - a[1], dz = d[2] - a[2];
      var flat = sideD * sideD <= 1e-18 * (nx * nx + ny * ny + nz * nz) * (dx * dx + dy * dy + dz * dz);
      if (flat || sideD * sideO < 0.0) {
        var near = triangle(a, b, c, face[0], face[1], face[2]);
        var distance = near.x * near.x + near.y * near.y + near.z * near.z;
        if (distance < bestDistance) {
          bestDistance = distance;
          best = near;
        }
      }
    }
    if (best == null) return {x: 0.0, y: 0.0, z: 0.0, keep: [0, 1, 2, 3], inside: true};
    return best;
  }
}
