package kinematicskit.native;

import kinematicskit.KinematicProblem;
import kinematicskit.KinematicState;
import kinematicskit.SolverWorkspace;

/** One differential step: the velocities of the problem's active DOFs, and why the QP stopped. */
class DifferentialStep {
  /** DOF velocity per active DOF (the problem's layout order), in DOF units per second. */
  public final velocity:Array<Float>;
  public final status:Int;
  public final iterations:Int;

  public function new(velocity:Array<Float>, status:Int, iterations:Int) {
    this.velocity = velocity;
    this.status = status;
    this.iterations = iterations;
  }
}

/**
 * Differential IK in the shape of mink: each step evaluates the problem's
 * tasks at `state` (residuals `e`, Jacobians `J`, already weighted), then
 * solves for the joint displacement Δ over one period `dt`:
 *
 *   minimize ½‖J·Δ − gain·e‖² + ½λ²‖Δ‖²
 *   subject to  lower − q ≤ Δ ≤ upper − q        (configuration limits)
 *               −v·dt ≤ Δ ≤ v·dt                 (velocity limits, if given)
 *
 * and returns Δ / dt. `gain` in (0, 1] closes that fraction of the task error
 * per step (1 = all of it). Hard limits hold exactly, so integrating the
 * returned velocities never leaves the configuration range.
 */
class DifferentialIk {
  public static function step(problem:KinematicProblem, state:KinematicState, dt:Float, qp:NativeQpStep,
      ?velocityLimits:Array<Float>, ?gain:Float = 1.0, ?damping:Float = 1e-3,
      ?workspace:SolverWorkspace):DifferentialStep {
    if (problem == null || state == null || state.model != problem.model || qp == null)
      throw "Differential IK requires a problem, a state of its model and a QP step";
    if (!(dt > 0.0) || !Math.isFinite(dt) || !(gain > 0.0) || gain > 1.0 || !(damping >= 0.0))
      throw "Differential IK needs dt > 0, gain in (0, 1] and damping >= 0";
    var layout = problem.layout();
    var width = layout.width;
    if (qp.width != width) throw 'Differential IK QP has ${qp.width} variables, the problem ${width}';
    if (velocityLimits != null && velocityLimits.length != width)
      throw 'Differential IK needs one velocity limit per active DOF ($width)';
    var work = workspace == null ? new SolverWorkspace() : workspace;
    work.prepare(problem);
    var rows = problem.rowCount();
    problem.evaluate(state, work.snapshot, work.residual, work.jacobian);
    var jacobian = work.jacobian.slice(0, rows * width);
    var residual = [for (row in 0...rows) gain * work.residual[row]];
    var lower:Array<Float> = [], upper:Array<Float> = [];
    for (column in 0...width) {
      var dof = layout.dofs[column];
      var low = problem.lower[dof] - state.q[dof], high = problem.upper[dof] - state.q[dof];
      if (velocityLimits != null) {
        var reach = velocityLimits[column] * dt;
        if (!(reach >= 0.0)) throw "Differential IK velocity limits must be non-negative";
        low = Math.max(low, -reach);
        high = Math.min(high, reach);
      }
      // A DOF already outside its range may only move back towards it.
      if (low > 0.0) low = 0.0;
      if (high < 0.0) high = 0.0;
      lower.push(low);
      upper.push(high);
    }
    var solved = qp.solve(jacobian, residual, lower, upper, damping);
    return new DifferentialStep([for (value in solved.step) value / dt], solved.status, solved.iterations);
  }
}
