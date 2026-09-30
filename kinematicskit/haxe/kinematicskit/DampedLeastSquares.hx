package kinematicskit;

/**
 * Fixed-damping least squares: each iteration takes the full step
 * `(JᵀJ + λ²I) Δq = Jᵀe` over the active DOFs, then clamps them into their
 * limits. This is RobotKit's original IK iteration, kept exactly: it stops
 * as soon as the hard tasks are met (reporting the iteration index), and
 * after `maxIterations` steps it reports `IterationLimit` without checking
 * the final step. With a reused `workspace` it allocates nothing per
 * iteration.
 */
class DampedLeastSquares {
  public static function solve(problem:KinematicProblem, seed:KinematicState, ?maxIterations:Int = 100,
      ?damping:Float = 0.02, ?rankTolerance:Float = 1e-8, ?workspace:SolverWorkspace):KinematicSolution {
    if (problem == null || seed == null || seed.model != problem.model)
      throw "Damped least squares requires a problem and a seed state of its model";
    if (maxIterations < 0 || !Math.isFinite(damping) || damping < 0.0)
      throw "Damped least squares settings must be finite and non-negative";
    var work = workspace == null ? new SolverWorkspace() : workspace;
    work.prepare(problem);
    var rows = problem.rowCount();
    var layout = problem.layout();
    var width = layout.width;
    var snapshot = work.snapshot;
    var state = seed.copy();
    problem.clamp(state.q);
    for (iteration in 0...maxIterations) {
      problem.evaluate(state, snapshot, work.residual, work.jacobian);
      if (problem.satisfied())
        return SolverSupport.finish(problem, state, work, KinematicStatus.Converged, iteration, rankTolerance, false);
      LinearAlgebra.normalEquationsDense(work.jacobian, rows, width, work.residual, work.normal, work.rhs);
      for (i in 0...width) work.normal[i * width + i] += damping * damping;
      if (!LinearAlgebra.solveInPlace(work.normal, work.rhs, width, work.delta, 1e-15))
        return SolverSupport.finish(problem, state, work, KinematicStatus.NumericalFailure, iteration, rankTolerance,
          false);
      for (i in 0...width) state.q[layout.dofs[i]] += work.delta[i];
      problem.clamp(state.q);
    }
    return SolverSupport.finish(problem, state, work, KinematicStatus.IterationLimit, maxIterations, rankTolerance,
      false);
  }
}
