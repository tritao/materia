package processkit.manipulation;

import robotkit.manipulation.*;

import robotkit.manipulation.IkOptions.IkMethod;
import robotkit.spatial.Transform3;
import processkit.path.Toolpath;

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
 * manipulator chain's base frame to the toolpath's own frame), solves IK for
 * every point of a `Toolpath` segment, seeded by continuation from the
 * previous point, and reports the reachable fraction rather than aborting
 * at the first failure.
 */
class ReachabilityChecker {
  /**
   * Solves the pose a tool starts a path from, with the arm at `seed`. That pose is far from the seed (the arm unfolds from its
   * rest posture to the work), which is what the reaching method is for: tracking steps from a nearby seed, and from here it
   * can stall at a joint limit or wind a joint a full turn to the same pose, depending on millimetres of base error. Planning
   * (`check`) and running a plan (`SurfacePlanRunner`) both solve it here, so the one that calls a pose reachable is the one
   * that reaches it.
   */
  public static function solveApproach(manipulator:KinematicGroup, target:Transform3, seed:Array<Float>, ?positionTolerance:Float = 1e-4,
      ?orientationTolerance:Float = 1e-3, ?maxIterations:Int = 100, ?damping:Float = 0.02):IKResult {
    var options = new IkOptions(positionTolerance, orientationTolerance, maxIterations, damping);
    options.method = IkMethod.Reaching;
    return manipulator.solve(target, seed, options);
  }

  public static function check(manipulator:Manipulator, toolpath:Toolpath, base_T_work:Transform3,
      seed:Array<Float>, ?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3,
      ?maxIterations:Int = 100, ?damping:Float = 0.02,
      ?toolClearance:ToolClearanceChecker, ?toolJointStep:Null<Float>,
      ?maxToolStep:Float = 0.005, ?checkApproach:Bool = false):ReachabilityResult {
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
      // The first point is the approach from the seed posture; the following ones continue from the previous solution.
      var ik = index == 0 ? solveApproach(manipulator, target, seedQ, positionTolerance, orientationTolerance, maxIterations, damping) : manipulator.solve(target,
        seedQ, new IkOptions(positionTolerance, orientationTolerance, maxIterations, damping));
      lastQ = ik.q;
      var clear = ik.converged && (toolClearance == null || (reachableCount == 0 && !checkApproach
        ? toolClearance.isClear(manipulator.forwardKinematics(ik.q))
        : toolClearance.clearJointSegment(manipulator, seedQ, ik.q,
            toolJointStep, maxToolStep)));
      if (clear) {
        reachableCount++;
        seedQ = ik.q;
      } else if (firstFailure < 0) firstFailure = index;
    }
    var total = toolpath.points.length;
    var fraction = total == 0 ? 1.0 : reachableCount / total;
    return new ReachabilityResult(fraction, firstFailure, lastQ);
  }
}
