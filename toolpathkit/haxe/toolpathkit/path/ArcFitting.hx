package toolpathkit.path;

/**
  Replaces runs of short cutting lines that lie on one circle with arcs, so a
  circle that CAM polygonized reaches the machine as G2/G3 again. A run
  becomes an arc only if the arc stays within `tolerance` of the lines both
  ways: every vertex lies on the circle and no chord strays from it by more,
  so a genuine polygon is never rounded.
**/
class ArcFitting {
  /** Fewest lines worth replacing with one arc. */
  static inline final MIN_LINES = 3;

  public static function fit(ops:Array<ToolpathOp>, tolerance:Float):Array<ToolpathOp> {
    if (ops == null || !Math.isFinite(tolerance) || tolerance < 0.0)
      throw "arc fitting needs operations and a finite tolerance";
    if (tolerance == 0.0) return ops.copy();
    var result:Array<ToolpathOp> = [];
    var run:Array<Point3> = [];
    var runFeed = 0.0, runBlend = 0.0, runSpan:Null<Provenance> = null;
    function flush():Void {
      if (run.length >= 2) emitRun(run, runFeed, runBlend, runSpan, tolerance, result);
      run = [];
      runSpan = null;
    }
    for (op in ops) switch op {
      case Move(Cut, Line(start, end), feed, blend, span)
          if (Math.abs(start.z - end.z) <= 1e-12):
        var continues = run.length > 0 && runSpan == span && feed == runFeed && blend == runBlend &&
          run[run.length - 1].distanceTo(start) <= 1e-12 && Math.abs(start.z - run[0].z) <= 1e-12;
        if (!continues) {
          flush();
          run = [start];
          runFeed = feed; runBlend = blend; runSpan = span;
        }
        run.push(end);
      case _:
        flush();
        result.push(op);
    }
    flush();
    return result;
  }

  /** The lines through `points`, with every long enough stretch on a circle as one arc. */
  static function emitRun(points:Array<Point3>, feed:Float, blend:Float, span:Provenance,
      tolerance:Float, result:Array<ToolpathOp>):Void {
    var i = 0, last = points.length - 1;
    while (i < last) {
      var best:Null<PathGeometry> = null, bestEnd = -1;
      var j = i + MIN_LINES;
      while (j <= last) {
        var arc = circleThrough(points, i, j, tolerance);
        if (arc == null) break;
        best = arc;
        bestEnd = j;
        j++;
      }
      if (best != null) {
        result.push(ToolpathOp.Move(Cut, best, feed, blend, span));
        i = bestEnd;
      } else {
        result.push(ToolpathOp.Move(Cut, Line(points[i], points[i + 1]), feed, blend, span));
        i++;
      }
    }
  }

  /** The arc through points `from` to `to`, if it stays within `tolerance` of their lines. */
  static function circleThrough(points:Array<Point3>, from:Int, to:Int, tolerance:Float):Null<PathGeometry> {
    // Open arcs must interpolate both ends: adjoining moves and G-code's modal start
    // refer to those exact points. Closed runs have coincident ends, so use thirds.
    var a = points[from], end = points[to];
    var closed = a.distanceTo(end) <= 1e-12;
    var b = points[from + Std.int((to - from) / (closed ? 3 : 2))];
    var c = closed ? points[from + Std.int(2 * (to - from) / 3)] : end;
    var d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if (Math.abs(d) < 1e-18) return null;
    var aa = a.x * a.x + a.y * a.y, bb = b.x * b.x + b.y * b.y, cc = c.x * c.x + c.y * c.y;
    var cx = (aa * (b.y - c.y) + bb * (c.y - a.y) + cc * (a.y - b.y)) / d;
    var cy = (aa * (c.x - b.x) + bb * (a.x - c.x) + cc * (b.x - a.x)) / d;
    var radius = Math.sqrt((a.x - cx) * (a.x - cx) + (a.y - cy) * (a.y - cy));
    var sweep = 0.0, direction = 0.0;
    for (k in from...(to + 1)) {
      var p = points[k];
      if (Math.abs(Math.sqrt((p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy)) - radius) > tolerance)
        return null;
      if (k == from) continue;
      var q = points[k - 1];
      var chord = q.distanceTo(p);
      if (chord >= 2.0 * radius) return null;
      // How far the arc bulges from this chord.
      if (radius - Math.sqrt(radius * radius - chord * chord / 4.0) > tolerance) return null;
      var turn = Math.atan2((q.x - cx) * (p.y - cy) - (q.y - cy) * (p.x - cx),
        (q.x - cx) * (p.x - cx) + (q.y - cy) * (p.y - cy));
      var sign = turn > 0.0 ? 1.0 : -1.0;
      if (direction != 0.0 && sign != direction) return null;
      direction = sign;
      sweep += turn;
    }
    if (Math.abs(sweep) > 2.0 * Math.PI + 1e-9) return null;
    return Arc(new Point3(cx, cy, a.z), radius, Math.atan2(a.y - cy, a.x - cx), sweep);
  }
}
