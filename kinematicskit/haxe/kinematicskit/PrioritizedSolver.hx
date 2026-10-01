package kinematicskit;

/**
 * Hard tasks exactly, soft tasks as well as they allow (KINEMATICS.md
 * KK-D16). Each iteration takes the step Δ that
 *
 *   minimizes  ½‖J_soft·Δ − e_soft‖² + ½λ²‖Δ‖²
 *   subject to J_hard·Δ = e_hard   and   lower − q ≤ Δ ≤ upper − q
 *
 * over the active columns: an equality-constrained least-squares step,
 * solved through its KKT system (regularized by ε on the constraint block,
 * so rank-deficient hard rows, e.g. at a singularity, still solve). Limits
 * form an active set: DOFs whose step would leave their range are pinned at
 * it and the rest re-solved. Unlike `DampedLeastSquares`, soft tasks do not
 * pull the hard ones off: a posture preference shapes the configuration
 * while the tool target is met exactly. Pure Haxe, so it runs wherever the
 * kit does.
 *
 * Stops `Converged` once the hard tasks are met and the step has settled
 * (below `stepTolerance`), or met at the iteration limit; otherwise
 * `IterationLimit`. `maxStep` caps each step as in `DampedLeastSquares`.
 */
class PrioritizedSolver {
  public static function solve(problem:KinematicProblem, seed:KinematicState, ?maxIterations:Int = 100,
      ?damping:Float = 0.02, ?rankTolerance:Float = 1e-8, ?workspace:SolverWorkspace,
      ?maxStep:Float = Math.POSITIVE_INFINITY, ?translationScale:Float = 1.0,
      ?stepTolerance:Float = 1e-10):KinematicSolution {
    if (problem == null || seed == null || seed.model != problem.model)
      throw "Prioritized solver requires a problem and a seed state of its model";
    if (maxIterations < 0 || !Math.isFinite(damping) || !(damping > 0.0) || !(maxStep > 0.0) ||
        !(translationScale > 0.0) || !(stepTolerance >= 0.0))
      throw "Prioritized solver settings must be finite and positive";
    var work = workspace == null ? new SolverWorkspace() : workspace;
    work.prepare(problem);
    var rows = problem.rowCount();
    var layout = problem.layout();
    var width = layout.width;
    var dofCount = layout.dofs.length;
    var snapshot = work.snapshot;
    var state = seed.copy();
    problem.clamp(state.q);
    var hard = problem.hardRows();
    var isHard = [for (_ in 0...rows) false];
    for (row in hard) isHard[row] = true;
    var hasSoft = hard.length < rows;
    var m = hard.length;
    // Normal block of the objective and its right-hand side, built once per iteration.
    var normal = [for (_ in 0...width * width) 0.0], gradient = [for (_ in 0...width) 0.0];
    var delta = [for (_ in 0...width) 0.0];
    var pinned = [for (_ in 0...width) false];
    var lastStep = Math.POSITIVE_INFINITY;
    for (iteration in 0...maxIterations) {
      problem.evaluate(state, snapshot, work.residual, work.jacobian);
      var met = problem.satisfied();
      if (met && (!hasSoft || lastStep <= stepTolerance))
        return SolverSupport.finish(problem, state, work, KinematicStatus.Converged, iteration, rankTolerance, false);
      var jacobian = work.jacobian, residual = work.residual;
      for (i in 0...width * width) normal[i] = 0.0;
      for (i in 0...width) gradient[i] = 0.0;
      for (row in 0...rows) if (!isHard[row]) {
        var e = residual[row];
        for (a in 0...width) {
          var ja = jacobian[row * width + a];
          if (ja == 0.0) continue;
          gradient[a] += ja * e;
          for (b in 0...width) normal[a * width + b] += ja * jacobian[row * width + b];
        }
      }
      for (i in 0...width) normal[i * width + i] += damping * damping;
      for (i in 0...width) {
        pinned[i] = false;
        delta[i] = 0.0;
      }
      if (!stepWithin(problem, state, layout, normal, gradient, jacobian, residual, hard, width, dofCount, m, delta,
          pinned))
        return SolverSupport.finish(problem, state, work, KinematicStatus.NumericalFailure, iteration, rankTolerance,
          false);
      if (maxStep != Math.POSITIVE_INFINITY) {
        var ratio = 0.0;
        for (i in 0...width)
          ratio = Math.max(ratio, Math.abs(delta[i]) / (maxStep * DampedLeastSquares.columnScale(problem, i, translationScale)));
        if (ratio > 1.0) for (i in 0...width) delta[i] /= ratio;
      }
      lastStep = 0.0;
      for (i in 0...width) lastStep = Math.max(lastStep, Math.abs(delta[i]));
      SolverSupport.applyStep(problem, state, delta, 1.0);
      problem.clamp(state.q);
    }
    problem.evaluate(state, snapshot, work.residual, work.jacobian);
    var status = problem.satisfied() ? KinematicStatus.Converged : KinematicStatus.IterationLimit;
    return SolverSupport.finish(problem, state, work, status, maxIterations, rankTolerance, false);
  }

  /**
   * The constrained step into `delta`, pinning DOFs whose step leaves their
   * range and re-solving the rest, at most once per column. False when the
   * KKT system cannot be solved.
   */
  static function stepWithin(problem:KinematicProblem, state:KinematicState, layout:JacobianLayout,
      normal:Array<Float>, gradient:Array<Float>, jacobian:Array<Float>, residual:Array<Float>, hard:Array<Int>,
      width:Int, dofCount:Int, m:Int, delta:Array<Float>, pinned:Array<Bool>):Bool {
    // A small regularization of the constraint block: rank-deficient hard rows still solve, as a
    // least-squares fit of the constraint at weight 1/ε.
    var epsilon = 1e-10;
    for (_ in 0...(width + 1)) {
      var free = [for (i in 0...width) if (!pinned[i]) i];
      var f = free.length, n = f + m;
      var kkt = [for (_ in 0...n * n) 0.0], rhs = [for (_ in 0...n) 0.0];
      for (a in 0...f) {
        var ca = free[a];
        var g = gradient[ca];
        for (pinnedColumn in 0...width) if (pinned[pinnedColumn])
          g -= normal[ca * width + pinnedColumn] * delta[pinnedColumn];
        rhs[a] = g;
        for (b in 0...f) kkt[a * n + b] = normal[ca * width + free[b]];
      }
      for (k in 0...m) {
        var row = hard[k];
        var d = residual[row];
        for (pinnedColumn in 0...width) if (pinned[pinnedColumn])
          d -= jacobian[row * width + pinnedColumn] * delta[pinnedColumn];
        rhs[f + k] = d;
        for (a in 0...f) {
          var value = jacobian[row * width + free[a]];
          kkt[(f + k) * n + a] = value;
          kkt[a * n + f + k] = value;
        }
        kkt[(f + k) * n + f + k] = -epsilon;
      }
      var x = [for (_ in 0...n) 0.0];
      if (!LinearAlgebra.solveInPlace(kkt, rhs, n, x, 1e-18)) return false;
      for (a in 0...f) delta[free[a]] = x[a];
      // Pin any DOF the step takes out of its range, then re-solve the rest.
      var changed = false;
      for (a in 0...f) {
        var column = free[a];
        if (column >= dofCount) continue;
        var dof = layout.dofs[column];
        var low = problem.lower[dof] - state.q[dof], high = problem.upper[dof] - state.q[dof];
        if (delta[column] < low - 1e-15 || delta[column] > high + 1e-15) {
          delta[column] = delta[column] < low ? low : high;
          pinned[column] = true;
          changed = true;
        }
      }
      if (!changed) return true;
    }
    return true;
  }
}
