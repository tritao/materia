package kinematicskit;

/**
 * Fixed-damping least squares: each iteration takes the full step
 * `(JᵀJ + λ²I) Δq = Jᵀe` over the active DOFs, then clamps them into their
 * limits. This is RobotKit's original IK iteration: it stops as soon as the
 * hard tasks are met (reporting the iteration index). After `maxIterations`
 * steps it checks the final state too, reporting `Converged` if the last
 * step landed inside the tolerances (the original iteration reported
 * `IterationLimit` there). With a reused `workspace` it allocates nothing
 * per iteration.
 *
 * `maxStep` (unlimited by default, which is the original iteration) caps
 * each step: if any DOF would move more than `maxStep` (radians, or
 * `maxStep · translationScale` for a prismatic DOF), the whole step is
 * scaled down, keeping its direction. Targets far outside the reachable set
 * otherwise produce linearised steps of several radians.
 */
class DampedLeastSquares {
  public static function solve(problem:KinematicProblem, seed:KinematicState, ?maxIterations:Int = 100,
      ?damping:Float = 0.02, ?rankTolerance:Float = 1e-8, ?workspace:SolverWorkspace,
      ?maxStep:Float = Math.POSITIVE_INFINITY, ?translationScale:Float = 1.0):KinematicSolution {
    if (problem == null || seed == null || seed.model != problem.model)
      throw "Damped least squares requires a problem and a seed state of its model";
    if (maxIterations < 0 || !Math.isFinite(damping) || damping < 0.0 || !(maxStep > 0.0) || !(translationScale > 0.0))
      throw "Damped least squares settings must be finite and non-negative, with a positive step cap";
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
      if (maxStep != Math.POSITIVE_INFINITY) {
        var ratio = 0.0;
        for (i in 0...width) ratio = Math.max(ratio, Math.abs(work.delta[i]) / (maxStep * columnScale(problem, i, translationScale)));
        if (ratio > 1.0) for (i in 0...width) work.delta[i] /= ratio;
      }
      SolverSupport.applyStep(problem, state, work.delta, 1.0);
      problem.clamp(state.q);
    }
    // The last step may have landed inside the tolerances.
    problem.evaluate(state, snapshot, work.residual, work.jacobian);
    var status = problem.satisfied() ? KinematicStatus.Converged : KinematicStatus.IterationLimit;
    return SolverSupport.finish(problem, state, work, status, maxIterations, rankTolerance, false);
  }

  /** 1 for an angular column, `translationScale` for a translational one (prismatic DOF, root v). */
  public static function columnScale(problem:KinematicProblem, column:Int, translationScale:Float):Float {
    var layout = problem.layout();
    if (column < layout.dofs.length) return problem.model.dofIsAngular(layout.dofs[column]) ? 1.0 : translationScale;
    var block = layout.rootColumns.length - 1;
    while (block > 0 && layout.rootColumns[block] > column) block--;
    var offset = column - layout.rootColumns[block];
    var linear = layout.rootModes[block] == RootMotion.Planar ? offset < 2 : offset < 3;
    return linear ? translationScale : 1.0;
  }
}
