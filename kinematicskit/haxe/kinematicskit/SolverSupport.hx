package kinematicskit;

/** Shared bookkeeping for the solver policies. */
class SolverSupport {
  /**
   * Re-evaluates the problem at `state` and packages the diagnostics
   * (allocates; once per solve). `scales` multiplies each Jacobian column
   * for the rank test, as the solver saw it.
   */
  public static function finish(problem:KinematicProblem, state:KinematicState, workspace:SolverWorkspace,
      status:KinematicStatus, iterations:Int, rankTolerance:Float, useScales:Bool):KinematicSolution {
    var snapshot = workspace.snapshot;
    problem.evaluate(state, snapshot, workspace.residual, workspace.jacobian);
    var rows = problem.rowCount();
    var width = problem.layout().width;
    var hard = problem.hardRows();
    var matrix:Array<Float> = [];
    for (row in hard) for (c in 0...width)
      matrix.push(workspace.jacobian[row * width + c] * (useScales ? workspace.scales[c] : 1.0));
    var rank = LinearAlgebra.rank(matrix, hard.length, width, rankTolerance);
    return new KinematicSolution(status, state, iterations, LinearAlgebra.norm(workspace.residual, rows),
      [for (task in problem.tasks) new TaskResult(task)], rank, width - rank, problem.limitHits(state.q));
  }
}
