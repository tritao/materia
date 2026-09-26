package robotkit.manipulation;

import robotkit.spatial.Transform3;
import robotkit.process.Toolpath;

/**
 * Outcome of `ReachabilityChecker.check`: the fraction of a `Toolpath`'s
 * points a `Manipulator` could reach from one base placement, the index of
 * the first unreachable point (`-1` if every point converged), and the
 * final joint solution attempted (converged or not), so a caller can seed a
 * following check with it.
 */
class ReachabilityResult {
  public final reachableFraction:Float;
  public final firstFailureIndex:Int;
  public final finalQ:Array<Float>;

  public function new(reachableFraction:Float, firstFailureIndex:Int, finalQ:Array<Float>) {
    this.reachableFraction = reachableFraction;
    this.firstFailureIndex = firstFailureIndex;
    this.finalQ = finalQ.copy();
  }

  public function fullyReachable():Bool return firstFailureIndex < 0;
}

/**
 * For one candidate base placement (folded into `base_T_work`, the
 * manipulator chain's base frame to the toolpath's own frame — the same
 * convention `ToolpathExecutor` uses for `base_T_work`), solves IK for
 * every point of a `Toolpath` segment, seeded by continuation from the
 * previous point, and reports the reachable fraction rather than aborting
 * at the first failure (unlike `ToolpathExecutor`, which is used once a
 * placement has already been chosen).
 */
class ReachabilityChecker {
  public static function check(manipulator:Manipulator, toolpath:Toolpath, base_T_work:Transform3,
      seed:Array<Float>, ?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3,
      ?maxIterations:Int = 100, ?damping:Float = 0.02):ReachabilityResult {
    if (manipulator == null || toolpath == null || base_T_work == null)
      throw "Reachability check requires a manipulator, toolpath, and base_T_work transform";
    if (seed == null) throw "Reachability check requires a seed joint configuration";

    // Seeding continues from the last *converged* solution, not a failed
    // attempt's wandering final iterate: a point beyond reach can leave the
    // solver far from any good configuration, which would otherwise poison
    // the seed for every following (possibly reachable) point.
    var seedQ = seed.copy();
    var lastQ = seed.copy();
    var reachableCount = 0;
    var firstFailure = -1;
    for (index in 0...toolpath.points.length) {
      var point = toolpath.points[index];
      var target = base_T_work.compose(point.work_T_tcp);
      var ik = manipulator.solveIkForTcp(target, seedQ, positionTolerance, orientationTolerance,
        maxIterations, damping);
      lastQ = ik.q;
      if (ik.converged) {
        reachableCount++;
        seedQ = ik.q;
      } else if (firstFailure < 0) firstFailure = index;
    }
    var total = toolpath.points.length;
    var fraction = total == 0 ? 1.0 : reachableCount / total;
    return new ReachabilityResult(fraction, firstFailure, lastQ);
  }
}
