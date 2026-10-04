package robotkit.tool;

/**
 * A convex solid made from the vertices of its hull, for geometric questions the physics engine's
 * contacts do not answer: how far a point is from it, and where a ray enters it. It keeps the
 * hull as outward planes, so both questions cost one pass over them.
 *
 * Planes come from every triple of vertices that has all the others on one side, so a hull of up to
 * `EXACT_LIMIT` vertices is exact; a larger one is replaced by its bounding box, which only errs
 * on the safe side of "the work is at least this big".
 */
class ConvexSolid {
  /** Hulls with more vertices than this are boxed (the triple search is cubic). */
  public static inline var EXACT_LIMIT:Int = 80;
  /** The most sweeps over the planes the nearest-point search makes. */
  public static inline var DYKSTRA_SWEEPS:Int = 200;

  final normals:Array<Float> = [];
  final offsets:Array<Float> = [];
  /** The hull's corners as x, y, z triples (the box's, for a boxed hull). */
  final corners:Array<Float> = [];

  /** `vertices` is x, y, z per vertex, in the solid's own frame. */
  public function new(vertices:Array<Float>) {
    if (vertices == null || vertices.length % 3 != 0 || vertices.length < 12)
      throw "A convex solid needs at least four vertices as x, y, z triples";
    for (value in vertices) if (!Math.isFinite(value)) throw "A convex solid needs finite vertices";
    var count = Std.int(vertices.length / 3);
    if (count > EXACT_LIMIT) addBox(vertices, count);
    else addHull(vertices, count);
    collectCorners(vertices, count);
    if (offsets.length < 4) throw "A convex solid needs vertices that are not flat";
  }

  /** The bounding box of `min` to `max`, as a solid. */
  public static function box(minX:Float, minY:Float, minZ:Float, maxX:Float, maxY:Float, maxZ:Float):ConvexSolid {
    var vertices:Array<Float> = [];
    for (x in [minX, maxX]) for (y in [minY, maxY]) for (z in [minZ, maxZ]) {
      vertices.push(x);
      vertices.push(y);
      vertices.push(z);
    }
    return new ConvexSolid(vertices);
  }

  /** The vertices that are corners of the solid: those on three planes that do not share a line. */
  function collectCorners(vertices:Array<Float>, count:Int):Void {
    var scale = 0.0;
    for (value in vertices) scale = Math.max(scale, Math.abs(value));
    var slack = 1e-7 * Math.max(scale, 1e-6);
    for (index in 0...count) {
      var x = vertices[3 * index], y = vertices[3 * index + 1], z = vertices[3 * index + 2];
      var on:Array<Int> = [];
      for (plane in 0...offsets.length)
        if (Math.abs(normals[3 * plane] * x + normals[3 * plane + 1] * y + normals[3 * plane + 2] * z - offsets[plane]) <= slack)
          on.push(plane);
      // A vertex on fewer than three planes is inside a face or an edge of a boxed hull.
      if (on.length < 3) continue;
      var duplicate = false;
      var seen = Std.int(corners.length / 3);
      for (other in 0...seen)
        if (Math.abs(corners[3 * other] - x) <= slack && Math.abs(corners[3 * other + 1] - y) <= slack &&
            Math.abs(corners[3 * other + 2] - z) <= slack) {
          duplicate = true;
          break;
        }
      if (duplicate) continue;
      corners.push(x);
      corners.push(y);
      corners.push(z);
    }
    if (corners.length < 12) throw "A convex solid needs at least four corners";
  }

  /** The solid's corners, as x, y, z triples in its own frame. */
  public function cornerPoints():Array<Float> return corners.copy();

  function addPlane(nx:Float, ny:Float, nz:Float, offset:Float):Void {
    for (index in 0...offsets.length) {
      if (Math.abs(normals[3 * index] - nx) < 1e-9 && Math.abs(normals[3 * index + 1] - ny) < 1e-9 &&
          Math.abs(normals[3 * index + 2] - nz) < 1e-9 && Math.abs(offsets[index] - offset) < 1e-9) return;
    }
    normals.push(nx);
    normals.push(ny);
    normals.push(nz);
    offsets.push(offset);
  }

  function addBox(vertices:Array<Float>, count:Int):Void {
    var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
    var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
    for (index in 0...count) for (axis in 0...3) {
      low[axis] = Math.min(low[axis], vertices[3 * index + axis]);
      high[axis] = Math.max(high[axis], vertices[3 * index + axis]);
    }
    for (axis in 0...3) {
      var normal = [0.0, 0.0, 0.0];
      normal[axis] = 1.0;
      addPlane(normal[0], normal[1], normal[2], high[axis]);
      addPlane(-normal[0], -normal[1], -normal[2], -low[axis]);
    }
  }

  function addHull(vertices:Array<Float>, count:Int):Void {
    var scale = 0.0;
    for (value in vertices) scale = Math.max(scale, Math.abs(value));
    var slack = 1e-9 * Math.max(scale, 1e-6);
    for (i in 0...count) for (j in i + 1...count) for (k in j + 1...count) {
      var ax = vertices[3 * j] - vertices[3 * i], ay = vertices[3 * j + 1] - vertices[3 * i + 1], az = vertices[3 * j + 2] - vertices[3 * i + 2];
      var bx = vertices[3 * k] - vertices[3 * i], by = vertices[3 * k + 1] - vertices[3 * i + 1], bz = vertices[3 * k + 2] - vertices[3 * i + 2];
      var nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
      var length = Math.sqrt(nx * nx + ny * ny + nz * nz);
      if (length < 1e-12 * Math.max(scale * scale, 1e-12)) continue;
      nx /= length;
      ny /= length;
      nz /= length;
      var offset = nx * vertices[3 * i] + ny * vertices[3 * i + 1] + nz * vertices[3 * i + 2];
      var above = false, below = false;
      for (m in 0...count) {
        var side = nx * vertices[3 * m] + ny * vertices[3 * m + 1] + nz * vertices[3 * m + 2] - offset;
        if (side > slack) above = true;
        else if (side < -slack) below = true;
        if (above && below) break;
      }
      if (above && below) continue;
      if (above) addPlane(-nx, -ny, -nz, -offset);
      else addPlane(nx, ny, nz, offset);
    }
  }

  /**
   * How far the point is outside the solid: negative inside (how deep, to the nearest face), zero on the surface. Outside
   * it is the true distance to the solid. In front of a face that is the face plane's distance, found at once when the
   * point's projection on that plane lies in the solid; beside an edge or at a corner (where the largest plane distance
   * falls short, to 0.71 or 0.58 of the truth at its worst) it is found by projecting on the planes in turn with
   * Dykstra's corrections, which converges to the nearest point of the intersection of the half-spaces.
   */
  public function distance(x:Float, y:Float, z:Float):Float {
    var result = Math.NEGATIVE_INFINITY, farthest = -1;
    for (index in 0...offsets.length) {
      var d = normals[3 * index] * x + normals[3 * index + 1] * y + normals[3 * index + 2] * z - offsets[index];
      if (d > result) {
        result = d;
        farthest = index;
      }
    }
    if (result <= 0.0) return result;
    // The foot of the perpendicular on the farthest plane: if the solid is there, nothing is nearer.
    var fx = x - normals[3 * farthest] * result, fy = y - normals[3 * farthest + 1] * result, fz = z - normals[3 * farthest + 2] * result;
    var slack = 1e-9 * Math.max(1.0, result);
    var inside = true;
    for (index in 0...offsets.length)
      if (normals[3 * index] * fx + normals[3 * index + 1] * fy + normals[3 * index + 2] * fz - offsets[index] > slack) {
        inside = false;
        break;
      }
    if (inside) return result;
    // Dykstra's alternating projections.
    var count = offsets.length;
    var qx = x, qy = y, qz = z;
    var ix = [for (_ in 0...count) 0.0], iy = [for (_ in 0...count) 0.0], iz = [for (_ in 0...count) 0.0];
    for (sweep in 0...DYKSTRA_SWEEPS) {
      var moved = 0.0;
      for (index in 0...count) {
        var yx = qx + ix[index], yy = qy + iy[index], yz = qz + iz[index];
        var d = normals[3 * index] * yx + normals[3 * index + 1] * yy + normals[3 * index + 2] * yz - offsets[index];
        var px = yx, py = yy, pz = yz;
        if (d > 0.0) {
          px -= normals[3 * index] * d;
          py -= normals[3 * index + 1] * d;
          pz -= normals[3 * index + 2] * d;
        }
        ix[index] = yx - px;
        iy[index] = yy - py;
        iz[index] = yz - pz;
        moved = Math.max(moved, Math.abs(px - qx) + Math.abs(py - qy) + Math.abs(pz - qz));
        qx = px;
        qy = py;
        qz = pz;
      }
      if (moved < 1e-12 * Math.max(1.0, result)) break;
    }
    var dx = x - qx, dy = y - qy, dz = z - qz;
    return Math.max(result, Math.sqrt(dx * dx + dy * dy + dz * dz));
  }

  /**
   * Exact distance when it can be below `threshold`; otherwise a lower bound at least that large.
   * A separating face already proves a clearance test, without a nearest-point projection.
   */
  public function distanceBelow(x:Float, y:Float, z:Float, threshold:Float):Float {
    for (index in 0...offsets.length) {
      var d = normals[3 * index] * x + normals[3 * index + 1] * y + normals[3 * index + 2] * z - offsets[index];
      if (d >= threshold) return d;
    }
    return distance(x, y, z);
  }

  /**
   * Where the ray from `o` along the unit vector `d` first meets the solid: the distance along it, zero when it starts
   * inside, or positive infinity when it misses or the solid is farther than `range`.
   */
  public function ray(ox:Float, oy:Float, oz:Float, dx:Float, dy:Float, dz:Float, range:Float):Float {
    var enter = Math.NEGATIVE_INFINITY, leave = Math.POSITIVE_INFINITY;
    for (index in 0...offsets.length) {
      var nx = normals[3 * index], ny = normals[3 * index + 1], nz = normals[3 * index + 2];
      var toward = nx * dx + ny * dy + nz * dz;
      var room = offsets[index] - (nx * ox + ny * oy + nz * oz);
      if (Math.abs(toward) < 1e-12) {
        if (room < 0) return Math.POSITIVE_INFINITY;
        continue;
      }
      var t = room / toward;
      if (toward < 0) enter = Math.max(enter, t);
      else leave = Math.min(leave, t);
    }
    if (enter > leave || leave < 0) return Math.POSITIVE_INFINITY;
    var hit = Math.max(enter, 0.0);
    return hit <= range ? hit : Math.POSITIVE_INFINITY;
  }
}
