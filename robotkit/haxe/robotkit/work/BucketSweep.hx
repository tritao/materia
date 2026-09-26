package robotkit.work;

/**
 * Approximate material removal for a `HeightMap`: a straight cutting-edge
 * sweep from `from` to `to`, `halfWidth` wide, lowers every grid vertex
 * within that capsule footprint (perpendicular distance to the swept
 * segment `<= halfWidth`) that is currently higher than `edgeHeight` down to
 * `edgeHeight`. An optional design map clamps the cut at each vertex so the
 * sweep cannot remove material below design, and an optional footprint limits
 * the cut to the work region. The reported volume is the integrated height-map
 * difference before and after the sweep, rather than a per-vertex estimate.
 */
class BucketSweep {
  public static function apply(map:HeightMap, from:Point2, to:Point2, halfWidth:Float, edgeHeight:Float,
      ?design:HeightMap, ?footprint:Polygon2, ?footprintTolerance:Float = 0.0):BucketSweepResult {
    if (map == null) throw "Bucket sweep requires a height map";
    if (from == null || to == null) throw "Bucket sweep requires from/to endpoints";
    if (!Math.isFinite(halfWidth) || halfWidth <= 0.0) throw "Bucket sweep half-width must be positive and finite";
    if (!Math.isFinite(edgeHeight)) throw "Bucket sweep edge height must be finite";
    if (!Math.isFinite(footprintTolerance) || footprintTolerance < 0.0)
      throw "Bucket sweep footprint tolerance must be finite and non-negative";
    if (design != null) HeightMap.ensureSameGrid(map, design);

    var before = map.copy();
    var lowered = 0;
    for (row in 0...map.rows) for (col in 0...map.columns) {
      var x = map.worldX(col), y = map.worldY(row);
      if (pointSegmentDistance(x, y, from.x, from.y, to.x, to.y) > halfWidth) continue;
      if (footprint != null && !footprint.containsOrWithin(new Point2(x, y), footprintTolerance)) continue;
      var current = map.elevationAt(col, row);
      var target = edgeHeight;
      if (design != null) {
        var designElevation = design.elevationAt(col, row);
        if (target < designElevation) target = designElevation;
      }
      if (current <= target) continue;
      map.setElevation(col, row, target);
      lowered++;
    }
    var integrated = HeightMap.volumeBetween(before, map);
    return new BucketSweepResult(integrated.cut, lowered);
  }

  /** Perpendicular distance from `(px, py)` to the closest point of segment `(ax,ay)-(bx,by)`. */
  static function pointSegmentDistance(px:Float, py:Float, ax:Float, ay:Float, bx:Float, by:Float):Float {
    var dx = bx - ax, dy = by - ay;
    var lengthSq = dx * dx + dy * dy;
    var t = lengthSq <= 1e-12 ? 0.0 : ((px - ax) * dx + (py - ay) * dy) / lengthSq;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    var cx = ax + t * dx, cy = ay + t * dy;
    var ex = px - cx, ey = py - cy;
    return Math.sqrt(ex * ex + ey * ey);
  }
}
