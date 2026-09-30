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
      ?initialDamping:Float = 1e-3, ?rankTolerance:Float = 1e-8, ?translationScale:Float = 1.0,
      ?workspace:SolverWorkspace):KinematicSolution {
    if (problem == null || seed == null || seed.model != problem.model)
      throw "Levenberg-Marquardt requires a problem and a seed state of its model";
    if (maxIterations <= 0 || !(initialDamping > 0.0) || !Math.isFinite(initialDamping) ||
        !(rankTolerance > 0.0) || !(translationScale > 0.0) || !Math.isFinite(translationScale))
      throw "Levenberg-Marquardt settings must be finite and positive";
    var model = problem.model;
    var work = workspace == null ? new SolverWorkspace() : workspace;
    work.prepare(problem);
    var rows = problem.rowCount();
    var active = problem.layout().dofs;
    var count = active.length;
    var scales = work.scales, residual = work.residual, jacobian = work.jacobian, scaled = work.scaled;
    var normal = work.normal, rhs = work.rhs, delta = work.delta, accepted = work.accepted;
    for (j in 0...count) scales[j] = model.dofIsAngular(active[j]) ? 1.0 : translationScale;
    var snapshot = work.snapshot;

    var state = seed.copy();
    problem.evaluate(state, snapshot, residual, jacobian);
    var converged = problem.satisfied();
    var currentNorm = LinearAlgebra.norm(residual, rows);
    for (j in 0...count) accepted[j] = state.q[active[j]];
    var damping = initialDamping;
    var iterations = 0;
    var limitStalled = false;
    var exhausted = false;
    while (!converged && iterations < maxIterations) {
      limitStalled = false;
      iterations++;
      // The residual and Jacobian always describe the last accepted state here.
      for (k in 0...rows) for (j in 0...count) scaled[k * count + j] = jacobian[k * count + j] * scales[j];
      LinearAlgebra.normalEquationsDense(scaled, rows, count, residual, normal, rhs);
      for (j in 0...count) normal[j * count + j] += damping * Math.max(normal[j * count + j], 1e-12);
      if (!LinearAlgebra.solveInPlace(normal, rhs, count, delta, 1e-30)) {
        damping = Math.min(1e16, damping * 10);
        if (damping >= 1e16) { exhausted = true; break; }
        continue;
      }

      var maxDelta = 0.0;
      for (j in 0...count) maxDelta = Math.max(maxDelta, Math.abs(delta[j]));
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
      return SolverSupport.finish(problem, state, work, KinematicStatus.Converged, iterations, rankTolerance, true);
    // Stationary test on the scaled gradient Jᵀe at the final state.
    for (k in 0...rows) for (j in 0...count) scaled[k * count + j] = jacobian[k * count + j] * scales[j];
    LinearAlgebra.normalEquationsDense(scaled, rows, count, residual, normal, rhs);
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
    return SolverSupport.finish(problem, state, work, status, iterations, rankTolerance, true);
  }
}
