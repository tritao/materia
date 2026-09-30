package kinematicskit;

/**
 * Levenberg-Marquardt with trial steps: solve
 * `(JᵀJ + μ·diag(max(JᵀJ, 1e-12))) Δ = Jᵀe` over the active DOFs, limit the
 * largest component to 1, clamp into the limits, and keep the step only if
 * the residual norm drops (then μ ← 0.3μ; otherwise roll back and μ ← 10μ).
 * Prismatic DOFs are solved in units of `translationScale` so a step of 1
 * means "one characteristic length". This is CadKit's closure-solver
 * algorithm, with analytic instead of finite-difference Jacobians.
 *
 * Stops `LimitBlocked` when an active DOF ends on a limit with the descent
 * direction pushing it further out,
 * `Conflicting` when the gradient vanishes or no step of any size lowers
 * the residual (the damping reached its 1e16 ceiling) without meeting the
 * tolerances, and `IterationLimit` otherwise. (CadKit reported an exhausted
 * damping as nonconvergent.)
 */
class LevenbergMarquardt {
  public static function solve(problem:KinematicProblem, seed:KinematicState, ?maxIterations:Int = 100,
      ?initialDamping:Float = 1e-3, ?rankTolerance:Float = 1e-8, ?translationScale:Float = 1.0):KinematicSolution {
    if (problem == null || seed == null || seed.model != problem.model)
      throw "Levenberg-Marquardt requires a problem and a seed state of its model";
    if (maxIterations <= 0 || !(initialDamping > 0.0) || !Math.isFinite(initialDamping) ||
        !(rankTolerance > 0.0) || !(translationScale > 0.0) || !Math.isFinite(translationScale))
      throw "Levenberg-Marquardt settings must be finite and positive";
    var model = problem.model;
    var n = model.dofCount();
    var rows = problem.rowCount();
    var active = problem.activeDofs();
    var count = active.length;
    var scales = [for (dof in active) model.dofIsAngular(dof) ? 1.0 : translationScale];
    var residual = SolverSupport.workspace(rows);
    var jacobian = SolverSupport.workspace(rows * n);
    var scaled = SolverSupport.workspace(rows * count);
    var normal = SolverSupport.workspace(count * count);
    var rhs = SolverSupport.workspace(count);
    var columns = [for (j in 0...count) j];
    var snapshot = new KinematicSnapshot(model);

    var state = seed.copy();
    problem.evaluate(state, snapshot, residual, jacobian);
    var converged = problem.satisfied();
    var currentNorm = LinearAlgebra.norm(residual, rows);
    var accepted = [for (dof in active) state.q[dof]];
    var damping = initialDamping;
    var iterations = 0;
    var limitStalled = false;
    var exhausted = false;
    while (!converged && iterations < maxIterations) {
      limitStalled = false;
      iterations++;
      // The residual and Jacobian always describe the last accepted state here.
      for (k in 0...rows) for (j in 0...count) scaled[k * count + j] = jacobian[k * n + active[j]] * scales[j];
      LinearAlgebra.normalEquations(scaled, rows, count, columns, residual, normal, rhs);
      for (j in 0...count) normal[j * count + j] += damping * Math.max(normal[j * count + j], 1e-12);
      var delta = LinearAlgebra.solve(normal, rhs, count, 1e-30);
      if (delta == null) {
        damping = Math.min(1e16, damping * 10);
        if (damping >= 1e16) { exhausted = true; break; }
        continue;
      }

      var maxDelta = 0.0;
      for (value in delta) maxDelta = Math.max(maxDelta, Math.abs(value));
      var stepFactor = maxDelta > 1 ? 1 / maxDelta : 1.0;
      var moved = false;
      var outwardAtLimit = false;
      for (j in 0...count) {
        var dof = active[j];
        var current = state.q[dof];
        var requested = current + delta[j] * stepFactor * scales[j];
        var value = Math.min(Math.max(requested, problem.lower[dof]), problem.upper[dof]);
        if (Math.abs(value - current) > 1e-14) moved = true;
        else if (Math.abs(requested - current) > 1e-14 &&
            ((current <= problem.lower[dof] && requested < current) || (current >= problem.upper[dof] && requested > current)))
          outwardAtLimit = true;
        state.q[dof] = value;
      }
      if (!moved) {
        limitStalled = outwardAtLimit;
        damping = Math.min(1e16, damping * 10);
        if (damping >= 1e16) { exhausted = true; break; }
        // Nothing moved, so the residual and Jacobian still describe `state`.
        continue;
      }

      problem.evaluate(state, snapshot, residual, jacobian);
      var trialNorm = LinearAlgebra.norm(residual, rows);
      var trialConverged = problem.satisfied();
      if (trialConverged || trialNorm < currentNorm) {
        converged = trialConverged;
        currentNorm = trialNorm;
        limitStalled = false;
        damping = Math.max(1e-12, damping * 0.3);
        for (j in 0...count) accepted[j] = state.q[active[j]];
      } else {
        for (j in 0...count) state.q[active[j]] = accepted[j];
        problem.evaluate(state, snapshot, residual, jacobian);
        damping = Math.min(1e16, damping * 10);
        if (damping >= 1e16) { exhausted = true; break; }
      }
    }

    if (problem.satisfied())
      return SolverSupport.finish(problem, state, snapshot, residual, jacobian, KinematicStatus.Converged,
        iterations, rankTolerance, scales);
    // Stationary test on the scaled gradient Jᵀe at the final state.
    for (k in 0...rows) for (j in 0...count) scaled[k * count + j] = jacobian[k * n + active[j]] * scales[j];
    LinearAlgebra.normalEquations(scaled, rows, count, columns, residual, normal, rhs);
    var gradientNorm = LinearAlgebra.norm(rhs, count);
    var jacobianNorm = LinearAlgebra.norm(scaled, rows * count);
    var threshold = 1e-10 * (1 + jacobianNorm * LinearAlgebra.norm(residual, rows));
    var stationary = gradientNorm <= threshold;
    // A DOF on a bound whose descent direction (Jᵀe) points outward is held by its limit.
    var blocked = false;
    for (j in 0...count) {
      var dof = active[j];
      if ((state.q[dof] >= problem.upper[dof] && rhs[j] > threshold) ||
          (state.q[dof] <= problem.lower[dof] && rhs[j] < -threshold)) blocked = true;
    }
    var status = limitStalled || blocked ? KinematicStatus.LimitBlocked
      : stationary || exhausted ? KinematicStatus.Conflicting : KinematicStatus.IterationLimit;
    return SolverSupport.finish(problem, state, snapshot, residual, jacobian, status, iterations, rankTolerance, scales);
  }
}
