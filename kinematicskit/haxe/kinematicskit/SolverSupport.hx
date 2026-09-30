package kinematicskit;

/** Shared bookkeeping for the solver policies. */
class SolverSupport {
  /** Re-evaluates the problem at `state` and packages the diagnostics. */
  public static function finish(problem:KinematicProblem, state:KinematicState, snapshot:KinematicSnapshot,
      residual:Array<Float>, jacobian:Array<Float>, status:KinematicStatus, iterations:Int,
      rankTolerance:Float, ?scales:Array<Float>):KinematicSolution {
    problem.evaluate(state, snapshot, residual, jacobian);
    var rows = problem.rowCount();
    var rank = hardRank(problem, jacobian, rankTolerance, scales);
    var active = problem.activeDofs();
    return new KinematicSolution(status, state, iterations, LinearAlgebra.norm(residual, rows),
      [for (task in problem.tasks) new TaskResult(task)], rank, active.length - rank, problem.limitHits(state.q));
  }

  /** Rank of the hard-task rows over the active columns, each column multiplied by `scales` if given. */
  public static function hardRank(problem:KinematicProblem, jacobian:Array<Float>, tolerance:Float,
      ?scales:Array<Float>):Int {
    var n = problem.model.dofCount();
    var rows = problem.hardRows();
    var active = problem.activeDofs();
    var matrix:Array<Float> = [];
    for (row in rows) for (j in 0...active.length)
      matrix.push(jacobian[row * n + active[j]] * (scales == null ? 1.0 : scales[j]));
    return LinearAlgebra.rank(matrix, rows.length, active.length, tolerance);
  }

  public static function workspace(size:Int):Array<Float> return [for (_ in 0...size) 0.0];
}
