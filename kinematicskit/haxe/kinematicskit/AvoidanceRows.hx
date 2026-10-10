package kinematicskit;

/**
 * Velocity-damper rows for collision avoidance (COLLISION.md CL5, Lane D's
 * D4). The distance of a pair changes at n·(J_b − J_a)·q̇, where n is the
 * normal from `a` toward `b` and J the point Jacobians at the closest
 * points. Over one step Δ of period `dt`, each pair within its influence
 * distance gives the row
 *
 *   n·(J_b − J_a)·Δ ≥ −ξ·(d − ds)/(di − ds)·dt
 *
 * with `ξ` a speed: far from the safety margin the pair may close at up to
 * ξ, at the margin not at all, and inside it the row demands separation. A
 * side outside the solved model has a zero Jacobian. Columns are the
 * problem's layout DOFs; a moving root's columns get no row entries.
 */
class AvoidanceRows {
  /** Row-major, one row per pair, `layout.width` columns. */
  public final matrix:Array<Float>;
  public final lower:Array<Float>;
  public final upper:Array<Float>;
  /** The pairs the rows come from, in row order. */
  public final pairs:Array<AvoidancePair>;

  function new(matrix:Array<Float>, lower:Array<Float>, upper:Array<Float>, pairs:Array<AvoidancePair>) {
    this.matrix = matrix;
    this.lower = lower;
    this.upper = upper;
    this.pairs = pairs;
  }

  public function count():Int return pairs.length;

  /** Rows for the pairs within their influence distance, at the evaluated `snapshot`. */
  public static function build(layout:JacobianLayout, snapshot:KinematicSnapshot, pairs:Array<AvoidancePair>, xi:Float,
      dt:Float):AvoidanceRows {
    if (!(xi > 0) || !Math.isFinite(xi) || !(dt > 0) || !Math.isFinite(dt)) throw "Avoidance rows need a speed xi > 0 and dt > 0";
    var w = layout.width;
    var near = [for (pair in pairs) if (pair.distance < pair.influence) pair];
    var matrix = [for (_ in 0...near.length * w) 0.0];
    var lower:Array<Float> = [], upper:Array<Float> = [];
    var jacobian = [for (_ in 0...6 * w) 0.0];
    for (row in 0...near.length) {
      var pair = near[row];
      var n = pair.normal;
      // n·J_b·Δ − n·J_a·Δ
      for (side in 0...2) {
        var body = side == 0 ? pair.bodyA : pair.bodyB;
        if (body < 0) continue;
        var p = side == 0 ? pair.pointA : pair.pointB;
        snapshot.pointJacobianColumns(body, p.x, p.y, p.z, layout, jacobian);
        var sign = side == 0 ? -1.0 : 1.0;
        for (column in 0...w)
          matrix[row * w + column] += sign * (n.x * jacobian[column] + n.y * jacobian[w + column] + n.z * jacobian[2 * w + column]);
      }
      lower.push(-xi * (pair.distance - pair.safety) / (pair.influence - pair.safety) * dt);
      upper.push(Math.POSITIVE_INFINITY);
    }
    return new AvoidanceRows(matrix, lower, upper, near);
  }
}
