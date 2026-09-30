package robotkit.manipulation;

import kinematicskit.DampedLeastSquares;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicState;
import kinematicskit.KinematicStatus;
import robotkit.spatial.Transform3;

/**
 * Damped-least-squares (Levenberg-Marquardt style) numerical IK. Joint
 * limits are clamped every iteration; non-convergence is reported in the
 * result rather than thrown. Runs `kinematicskit.DampedLeastSquares` on the
 * chain's compiled model with the group's limits.
 */
class InverseKinematics {
  public static function solve(chain:KinematicChain, group:JointGroup, target:Transform3,
      seed:Array<Float>, ?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3,
      ?maxIterations:Int = 100, ?damping:Float = 0.02):IKResult {
    if (chain == null || group == null || target == null)
      throw "Inverse kinematics requires a chain, joint group, and target";
    if (chain.dofCount() != group.count())
      throw "Inverse kinematics requires the chain and joint group to share the same degrees of freedom";
    var n = chain.dofCount();
    var start = seed == null ? [for (_ in 0...n) 0.0] : seed;
    if (start.length != n) throw 'Joint group requires $n values, got ${start.length}';
    var problem = new KinematicProblem(chain.model);
    for (i in 0...n) {
      // The group's convention: lower >= upper means unlimited.
      var limits = group.limitsOf(i);
      if (limits.lower < limits.upper) problem.setLimits(i, limits.lower, limits.upper);
      else problem.setLimits(i, Math.NEGATIVE_INFINITY, Math.POSITIVE_INFINITY);
    }
    problem.add(chain.tipTask(target, positionTolerance, orientationTolerance));
    var solution = DampedLeastSquares.solve(problem, new KinematicState(chain.model, start), maxIterations, damping);
    var tip = solution.tasks[0];
    return new IKResult(solution.status == KinematicStatus.Converged, solution.state.q, tip.positionError,
      tip.orientationError, solution.iterations);
  }
}
