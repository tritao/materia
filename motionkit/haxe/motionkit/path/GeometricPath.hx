package motionkit.path;

import motionkit.program.MotionPath;

/** Immutable ordered geometric path made from reusable primitives. */
class GeometricPath implements MotionPath {
  public final primitives:Array<PathPrimitive>;
  public final totalLength:Float;
  final cumulativeLengths:Array<Float>;
  /** Primitives by the grid cells their pieces touch, built on the first distance query. */
  var grid:Null<Map<Int, Array<Int>>> = null;
  static inline final CELL:Float = 0.001;

  public function new(primitives:Array<PathPrimitive>) {
    if (primitives == null || primitives.length == 0)
      throw "Geometric path needs at least one primitive";
    this.primitives = primitives.copy();
    cumulativeLengths = [];
    var total = 0.0;
    for (primitive in this.primitives) {
      if (primitive == null) throw "Geometric path cannot contain null primitives";
      var length = primitive.length();
      if (!Math.isFinite(length) || length < 0.0)
        throw "Geometric path primitive length must be finite and non-negative";
      total += length;
      cumulativeLengths.push(total);
    }
    totalLength = total;
  }

  public static function lines(points:Array<PathPoint>):GeometricPath {
    if (points == null || points.length < 2)
      throw "A line path needs at least two points";
    var segments:Array<PathPrimitive> = [];
    for (i in 0...(points.length - 1))
      segments.push(new LineSegment(points[i], points[i + 1]));
    return new GeometricPath(segments);
  }

  public function length():Float return totalLength;

  public function pointAt(distance:Float):PathPoint {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > totalLength)
      throw "Geometric-path distance is outside its length";
    var previous = 0.0;
    for (i in 0...primitives.length) {
      var end = cumulativeLengths[i];
      if (distance <= end || i == primitives.length - 1)
        return primitives[i].pointAt(distance - previous);
      previous = end;
    }
    return primitives[primitives.length - 1].pointAt(primitives[primitives.length - 1].length());
  }

  /**
    How far `point` is from the nearest point of the path. A grid of the cells along the primitives bounds the
    search to the cells near the point, so a query costs about the same however long the path is.
  **/
  public function distanceTo(point:PathPoint):Float {
    if (grid == null) buildGrid();
    var cells = grid;
    var ix = Math.floor(point.x / CELL), iy = Math.floor(point.y / CELL), iz = Math.floor(point.z / CELL);
    // How far the point is from the faces of its own cell: after `ring` rings the visited block reaches that
    // much plus `ring` cells beyond the point in every direction.
    var fx = point.x / CELL - ix, fy = point.y / CELL - iy, fz = point.z / CELL - iz;
    var inner = Math.min(Math.min(Math.min(fx, 1.0 - fx), Math.min(fy, 1.0 - fy)), Math.min(fz, 1.0 - fz)) * CELL;
    var best = Math.POSITIVE_INFINITY;
    // A primitive reaching several cells is measured in each: the nearest distance is unchanged by it, and keeping a
    // list of the primitives measured made a query quadratic in the primitives around the point (a gear's teeth).
    // The path itself stays unchanged by a query, so threads may share it.
    for (ring in 0...9) {
      for (dx in -ring...(ring + 1)) for (dy in -ring...(ring + 1)) for (dz in -ring...(ring + 1)) {
        if (Math.max(Math.abs(dx), Math.max(Math.abs(dy), Math.abs(dz))) != ring) continue;
        var found = cells.get(cellKey(ix + dx, iy + dy, iz + dz));
        if (found == null) continue;
        for (index in found) best = Math.min(best, distanceToPrimitive(primitives[index], point));
      }
      // Everything the visited block reaches has been seen; nothing unvisited is nearer than that.
      if (best <= ring * CELL + inner) return best;
    }
    // Far from the path (or in a cell the grid hashed away): look at every primitive.
    best = Math.POSITIVE_INFINITY;
    for (primitive in primitives) best = Math.min(best, distanceToPrimitive(primitive, point));
    return best;
  }

  static function cellKey(x:Int, y:Int, z:Int):Int
    return x * 73856093 + y * 19349663 + z * 83492791;

  function buildGrid():Void {
    var cells = new Map<Int, Array<Int>>();
    for (index in 0...primitives.length) {
      var primitive = primitives[index], length = primitive.length();
      // Cut the primitive into pieces of at most half a cell and file each piece by its own box, so a primitive
      // touches only the cells along it: the work and the entries grow with its length, not with the volume of its
      // bounding box (a long three-axis rapid would otherwise fill every cell of the box it spans).
      var pieces = Std.int(Math.max(1, Math.ceil(length / (CELL * 0.5))));
      var step = length / pieces;
      // A piece of a line is its chord; a piece of an arc bulges past its chord's box by less than the step.
      var pad = Std.isOfType(primitive, LineSegment) ? 1e-9 : Math.max(step, 1e-9);
      var previous = primitive.pointAt(0.0);
      for (piece in 1...(pieces + 1)) {
        var at = primitive.pointAt(piece == pieces ? length : length * piece / pieces);
        var x0 = Std.int(Math.floor((Math.min(previous.x, at.x) - pad) / CELL)), x1 = Std.int(Math.floor((Math.max(previous.x, at.x) + pad) / CELL));
        var y0 = Std.int(Math.floor((Math.min(previous.y, at.y) - pad) / CELL)), y1 = Std.int(Math.floor((Math.max(previous.y, at.y) + pad) / CELL));
        var z0 = Std.int(Math.floor((Math.min(previous.z, at.z) - pad) / CELL)), z1 = Std.int(Math.floor((Math.max(previous.z, at.z) + pad) / CELL));
        for (x in x0...(x1 + 1)) for (y in y0...(y1 + 1)) for (z in z0...(z1 + 1)) {
          var key = cellKey(x, y, z);
          var list = cells.get(key);
          if (list == null) { list = []; cells.set(key, list); }
          // Consecutive pieces share cells; a primitive is filed once per cell.
          if (list.length == 0 || list[list.length - 1] != index) list.push(index);
        }
        previous = at;
      }
    }
    grid = cells;
  }

  static function distanceToPrimitive(primitive:PathPrimitive, point:PathPoint):Float {
    if (Std.isOfType(primitive, LineSegment)) {
      var line:LineSegment = cast primitive;
      return segmentDistance(point, line.start, line.end);
    }
    if (Std.isOfType(primitive, ArcSegment)) {
      var arc:ArcSegment = cast primitive;
      var angle = Math.atan2(point.y - arc.center.y, point.x - arc.center.x);
      var delta = angle - arc.startAngle;
      if (arc.sweepAngle >= 0.0) {
        while (delta < 0.0) delta += 2.0 * Math.PI;
      } else {
        while (delta > 0.0) delta -= 2.0 * Math.PI;
      }
      // Beyond the swept angle the nearest point is whichever end is nearer.
      var inside = arc.sweepAngle >= 0.0 ? delta <= arc.sweepAngle : delta >= arc.sweepAngle;
      if (!inside) return Math.min(point.distanceTo(arc.start), point.distanceTo(arc.end));
      var at = arc.pointAt(arc.length() * (arc.sweepAngle == 0.0 ? 0.0 : delta / arc.sweepAngle));
      return Math.sqrt((point.x - at.x) * (point.x - at.x) + (point.y - at.y) * (point.y - at.y) + (point.z - at.z) * (point.z - at.z));
    }
    if (Std.isOfType(primitive, CircularSegment)) {
      var circular:CircularSegment = cast primitive;
      return circular.distanceTo(point);
    }
    throw "Unsupported authored path primitive";
  }

  static function segmentDistance(point:PathPoint, start:PathPoint, end:PathPoint):Float {
    var dx = end.x - start.x, dy = end.y - start.y, dz = end.z - start.z;
    var lengthSquared = dx * dx + dy * dy + dz * dz;
    var fraction = lengthSquared <= 0.0 ? 0.0 : Math.max(0.0, Math.min(1.0,
      ((point.x - start.x) * dx + (point.y - start.y) * dy + (point.z - start.z) * dz) / lengthSquared));
    var x = point.x - start.x - fraction * dx, y = point.y - start.y - fraction * dy, z = point.z - start.z - fraction * dz;
    return Math.sqrt(x * x + y * y + z * z);
  }
}
