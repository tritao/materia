package motionkit.planner;

/** Joint path q(s) and its first two path derivatives at monotonic samples. */
class JointPathSamples {
  public final s:Array<Float>;
  public final q:Array<Array<Float>>;
  public final qPrime:Array<Array<Float>>;
  /** d²q/ds² leaving each sample. */
  public final qDoublePrime:Array<Array<Float>>;
  /**
    d²q/ds² arriving at each sample: the same as `qDoublePrime` unless the
    path's curvature jumps there, as where a line meets an arc.
  **/
  public final qDoublePrimeBefore:Array<Array<Float>>;
  public final jointCount:Int;

  public function new(s:Array<Float>, q:Array<Array<Float>>,
      qPrime:Array<Array<Float>>, qDoublePrime:Array<Array<Float>>,
      ?qDoublePrimeBefore:Array<Array<Float>>) {
    if (qDoublePrimeBefore == null) qDoublePrimeBefore = qDoublePrime;
    if (s == null || q == null || qPrime == null || qDoublePrime == null ||
        s.length < 2 || q.length != s.length || qPrime.length != s.length ||
        qDoublePrime.length != s.length || qDoublePrimeBefore.length != s.length)
      throw "Joint path needs at least two matching samples";
    if (q[0] == null || q[0].length == 0)
      throw "Joint path needs at least one joint";
    jointCount = q[0].length;
    var previous = 0.0;
    for (sample in 0...s.length) {
      if (!Math.isFinite(s[sample]) || (sample > 0 && s[sample] <= previous))
        throw "Joint path positions must be finite and strictly increasing";
      previous = s[sample];
      for (values in [q[sample], qPrime[sample], qDoublePrime[sample],
          qDoublePrimeBefore[sample]]) {
        if (values == null || values.length != jointCount)
          throw "Joint path sample joint counts must match";
        for (value in values)
          if (!Math.isFinite(value)) throw "Joint path values must be finite";
      }
    }
    this.s = s.copy();
    this.q = copyRows(q);
    this.qPrime = copyRows(qPrime);
    this.qDoublePrime = copyRows(qDoublePrime);
    this.qDoublePrimeBefore = copyRows(qDoublePrimeBefore);
  }

  public function start():Float return s[0];

  public function end():Float return s[s.length - 1];

  /** Linear q(s) interpolation used by the reference degree-1 lowering. */
  public function positionAt(distance:Float):Array<Float> {
    if (!Math.isFinite(distance) || distance < start() || distance > end())
      throw "Joint-path position is outside the sampled range";
    var span = s.length - 2;
    for (index in 0...(s.length - 1)) {
      if (distance <= s[index + 1]) { span = index; break; }
    }
    var alpha = (distance - s[span]) / (s[span + 1] - s[span]);
    return [for (joint in 0...jointCount)
      q[span][joint] + (q[span + 1][joint] - q[span][joint]) * alpha];
  }

  static function copyRows(source:Array<Array<Float>>):Array<Array<Float>>
    return [for (row in source) row.copy()];
}
