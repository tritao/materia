package kinematicskit;

/**
 * Fixed-damping least squares: each iteration takes the full step
 * `(JᵀJ + λ²I) Δq = Jᵀe` over the active DOFs, then clamps them into their
 * limits. This is RobotKit's original IK iteration, kept exactly: it stops
 * as soon as the hard tasks are met (reporting the iteration index), and
 * after `maxIterations` steps it reports `IterationLimit` without checking
 * the final step.
 */
class DampedLeastSquares {
  public static function solve(problem:KinematicProblem, seed:KinematicState, ?maxIterations:Int = 100,
      ?damping:Float = 0.02, ?rankTolerance:Float = 1e-8):KinematicSolution {
    if (problem == null || seed == null || seed.model != problem.model)
      throw "Damped least squares requires a problem and a seed state of its model";
    if (maxIterations < 0 || !Math.isFinite(damping) || damping < 0.0)
      throw "Damped least squares settings must be finite and non-negative";
    var model = problem.model;
    var n = model.dofCount();
    var rows = problem.rowCount();
    var active = problem.activeDofs();
    var residual = SolverSupport.workspace(rows);
    var jacobian = SolverSupport.workspace(rows * n);
    var snapshot = new KinematicSnapshot(model);
    var state = seed.copy();
    problem.clamp(state.q);
    for (iteration in 0...maxIterations) {
      problem.evaluate(state, snapshot, residual, jacobian);
      if (problem.satisfied())
        return SolverSupport.finish(problem, state, snapshot, residual, jacobian, KinematicStatus.Converged,
          iteration, rankTolerance);
      var delta = LinearAlgebra.dampedStep(jacobian, rows, n, active, residual, damping);
      if (delta == null)
        return SolverSupport.finish(problem, state, snapshot, residual, jacobian, KinematicStatus.NumericalFailure,
          iteration, rankTolerance);
      for (i in 0...active.length) state.q[active[i]] += delta[i];
      problem.clamp(state.q);
    }
    return SolverSupport.finish(problem, state, snapshot, residual, jacobian, KinematicStatus.IterationLimit,
      maxIterations, rankTolerance);
  }
}
