package robotkit.work;

/**
 * Approximate material removal for a `HeightMap`: a straight cutting-edge
 * sweep from `from` to `to`, `halfWidth` wide, lowers every grid vertex
 * within that capsule footprint (perpendicular distance to the swept
 * segment `<= halfWidth`) that is currently higher than `edgeHeight` down to
 * `edgeHeight`, and reports the removed volume as
 * `sum(oldElevation - edgeHeight) * cellSize^2` over the vertices actually
 * lowered (a box/Voronoi area approximation per vertex — no soil mechanics,
 * no bucket fill-factor or spillage model).
 */
class BucketSweep {
  public static function apply(map:HeightMap, from:Point2, to:Point2, halfWidth:Float, edgeHeight:Float):BucketSweepResult {
    if (map == null) throw "Bucket sweep requires a height map";
    if (from == null || to == null) throw "Bucket sweep requires from/to endpoints";
    if (!Math.isFinite(halfWidth) || halfWidth <= 0.0) throw "Bucket sweep half-width must be positive and finite";
    if (!Math.isFinite(edgeHeight)) throw "Bucket sweep edge height must be finite";

    var area = map.cellSize * map.cellSize;
    var removed = 0.0;
    var lowered = 0;
    for (row in 0...map.rows) for (col in 0...map.columns) {
      var x = map.worldX(col), y = map.worldY(row);
      if (pointSegmentDistance(x, y, from.x, from.y, to.x, to.y) > halfWidth) continue;
      var current = map.elevationAt(col, row);
      if (current <= edgeHeight) continue;
      map.setElevation(col, row, edgeHeight);
      removed += (current - edgeHeight) * area;
      lowered++;
    }
    return new BucketSweepResult(removed, lowered);
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
