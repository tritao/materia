package robotkit.perception;

/**
 * Deterministic linear congruential generator, the same recurrence
 * `KinematicsTests`' seeded IK fixture uses, promoted to a library type so
 * `PlaneFit`'s RANSAC and `SimulatedSurfaceScanner` can both use one seeded,
 * dependency-free source of randomness (no wall-clock, per the plan's
 * determinism rule).
 */
class SeededRandom {
  var state:Int;

  public function new(seed:Int) {
    this.state = seed;
  }

  /** Returns a value in [0, 1). */
  public function next():Float {
    state = (state * 1103515245 + 12345) & 0x7fffffff;
    return state / 2147483647.0;
  }

  /** Returns an integer in [0, bound). */
  public function nextInt(bound:Int):Int {
    if (bound <= 0) throw "SeededRandom.nextInt requires a positive bound";
    var value = Std.int(next() * bound);
    if (value >= bound) value = bound - 1;
    return value;
  }

  /**
   * Approximately standard-normal (mean 0, stddev 1) via the sum of twelve
   * uniforms (Irwin-Hall/CLT approximation) rather than Box-Muller, since
   * `haxeon/stdlib/Math.hx` has no `Math.log`.
   */
  public function nextGaussian():Float {
    var sum = 0.0;
    for (_ in 0...12) sum += next();
    return sum - 6.0;
  }
}
