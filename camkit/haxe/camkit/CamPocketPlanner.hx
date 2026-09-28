package camkit;

import toolpathkit.path.Point3;

typedef CamPocketPass = {y:Float, left:Float, right:Float};

/** Horizontal cutter-centre passes inside a face eroded by a disk. */
class CamPocketPlanner {
  public static function plan(contour:CamContour, radius:Float,
      stepOver:Float, ?islands:Array<CamContour>):Array<CamPocketPass> {
    if (islands == null) islands = [];
    var vertices = contour.vertices;
    var minY = Math.POSITIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY;
    for (point in vertices) {
      minY = Math.min(minY, point.y);
      maxY = Math.max(maxY, point.y);
    }
    var firstY = minY + radius, lastY = maxY - radius;
    if (firstY > lastY + 1e-10) throw "Tool does not fit inside CAM pocket";
    var rows:Array<Float> = [];
    var y = firstY;
    while (y <= lastY + 1e-10) {
      if (rows.length >= 10000) throw "CAM pocket exceeds 10000 clearing rows";
      rows.push(Math.min(y, lastY));
      y += stepOver;
    }
    if (rows.length == 0 || lastY - rows[rows.length - 1] > 1e-9)
      rows.push(lastY);
    var passes:Array<CamPocketPass> = [];
    for (row in rows) {
      var intervals = interiorIntervals(contour, row);
      for (i in 0...vertices.length) {
        var forbidden = capsuleSlice(vertices[i],
          vertices[(i + 1) % vertices.length], row, radius);
        if (forbidden != null) intervals = subtract(intervals, forbidden);
      }
      for (island in islands) {
        for (blocked in interiorIntervals(island, row))
          intervals = subtract(intervals, blocked);
        var edgePoints = island.vertices;
        for (i in 0...edgePoints.length) {
          var forbidden = capsuleSlice(edgePoints[i],
            edgePoints[(i + 1) % edgePoints.length], row, radius);
          if (forbidden != null) intervals = subtract(intervals, forbidden);
        }
      }
      for (interval in intervals)
        if (interval.right - interval.left > 1e-8) {
          if (passes.length >= 100000)
            throw "CAM pocket exceeds 100000 clearing passes";
          passes.push({y: row, left: interval.left,
            right: interval.right});
        }
    }
    if (passes.length == 0) throw "Tool does not fit inside CAM pocket";
    return passes;
  }

  static function interiorIntervals(contour:CamContour, row:Float)
      :Array<{left:Float, right:Float}> {
    var vertices = contour.vertices, crossings:Array<Float> = [];
    for (i in 0...vertices.length) {
      var a = vertices[i], b = vertices[(i + 1) % vertices.length];
      if ((a.y <= row && row < b.y) || (b.y <= row && row < a.y))
        crossings.push(a.x + (row - a.y) * (b.x - a.x) / (b.y - a.y));
    }
    crossings.sort((a, b) -> a < b ? -1 : (a > b ? 1 : 0));
    if (crossings.length % 2 != 0)
      throw "CAM pocket contour has unmatched scanline crossings";
    return [for (i in 0...Std.int(crossings.length / 2))
      {left: crossings[2 * i], right: crossings[2 * i + 1]}];
  }

  static function subtract(intervals:Array<{left:Float, right:Float}>,
      forbidden:{left:Float, right:Float})
      :Array<{left:Float, right:Float}> {
    var remaining:Array<{left:Float, right:Float}> = [];
    for (interval in intervals) {
      if (forbidden.right <= interval.left ||
          forbidden.left >= interval.right) {
        remaining.push(interval);
      } else {
        if (forbidden.left - interval.left > 1e-9)
          remaining.push({left: interval.left, right: forbidden.left});
        if (interval.right - forbidden.right > 1e-9)
          remaining.push({left: forbidden.right, right: interval.right});
      }
    }
    return remaining;
  }

  /** A segment's radius-r neighbourhood has one interval on a horizontal row. */
  static function capsuleSlice(a:Point3, b:Point3, y:Float,
      radius:Float):Null<{left:Float, right:Float}> {
    if (y < Math.min(a.y, b.y) - radius ||
        y > Math.max(a.y, b.y) + radius) return null;
    var closestX:Float;
    if (Math.abs(b.y - a.y) < 1e-14)
      closestX = (a.x + b.x) * 0.5;
    else if (y >= Math.min(a.y, b.y) && y <= Math.max(a.y, b.y))
      closestX = a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);
    else closestX = Math.abs(y - a.y) < Math.abs(y - b.y) ? a.x : b.x;
    if (distance(closestX, y, a, b) >= radius - 1e-12) return null;
    var low = Math.min(a.x, b.x) - radius, high = closestX;
    for (_ in 0...40) {
      var middle = (low + high) * 0.5;
      if (distance(middle, y, a, b) < radius) high = middle;
      else low = middle;
    }
    var left = high;
    low = closestX; high = Math.max(a.x, b.x) + radius;
    for (_ in 0...40) {
      var middle = (low + high) * 0.5;
      if (distance(middle, y, a, b) < radius) low = middle;
      else high = middle;
    }
    return {left: left, right: low};
  }

  static function distance(x:Float, y:Float,
      a:Point3, b:Point3):Float {
    var dx = b.x - a.x, dy = b.y - a.y;
    var t = Math.max(0.0, Math.min(1.0,
      ((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy)));
    var px = x - a.x - t * dx, py = y - a.y - t * dy;
    return Math.sqrt(px * px + py * py);
  }
}
