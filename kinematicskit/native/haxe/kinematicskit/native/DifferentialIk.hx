package kinematicskit.native;

import kinematicskit.KinematicProblem;
import kinematicskit.KinematicState;
import kinematicskit.LinearAlgebra;
import kinematicskit.SolverWorkspace;
import kinematicskit.StepLimits;

/**
 * One differential step: a velocity per layout column (the active DOFs, then
 * any moving root's twist, world frame), and why the QP stopped. Integrate
 * roots with `SolverSupport.applyStep(problem, state, velocity, dt)`.
 */
class DifferentialStep {
  /** Velocity per layout column (active DOFs, then moving roots' v and ω), per second. */
  public final velocity:Array<Float>;
  /** Why the QP stopped (`KinematicsKitNativeConstants.KK_QP_*`), or -1 when it threw. */
  public final status:Int;
  public final iterations:Int;
  /** True when the QP did not solve and the clamped damped step was used instead. */
  public final fallback:Bool;
  /** Columns whose step sits on a bound. */
  public final limited:Array<Int>;

  public function new(velocity:Array<Float>, status:Int, iterations:Int, ?fallback:Bool = false, ?limited:Array<Int>) {
    this.velocity = velocity;
    this.status = status;
    this.iterations = iterations;
    this.fallback = fallback;
    this.limited = limited == null ? [] : limited;
  }
}

/**
 * Differential IK in the shape of mink: each step evaluates the problem's
 * tasks at `state` (residuals `e`, Jacobians `J`, already weighted), then
 * solves for the joint displacement Δ over one period `dt`:
 *
 *   minimize ½‖J·Δ − gain·e‖² + ½λ²‖Δ‖²   subject to `limits` (`StepLimits`)
 *
 * and returns Δ / dt. `gain` in (0, 1] closes that fraction of the task error
 * per step (1 = all of it; a `FrameVelocityTask` asks for a twist, so 1).
 * Without `limits`, only the configuration limits bound the step. The limits
 * hold exactly, so integrating the returned velocities never leaves the
 * range. Should the QP fail, the damped least-squares step clamped into the
 * same bounds is returned, flagged `fallback`.
 */
class DifferentialIk {
  public static function step(problem:KinematicProblem, state:KinematicState, dt:Float, qp:NativeQpStep,
      ?limits:StepLimits, ?gain:Float = 1.0, ?damping:Float = 1e-3, ?workspace:SolverWorkspace,
      ?maxIterations:Int = 1000):DifferentialStep {
    if (problem == null || state == null || state.model != problem.model || qp == null)
      throw "Differential IK requires a problem, a state of its model and a QP step";
    if (!(dt > 0.0) || !Math.isFinite(dt) || !(gain > 0.0) || gain > 1.0 || !(damping >= 0.0))
      throw "Differential IK needs dt > 0, gain in (0, 1] and damping >= 0";
    var layout = problem.layout();
    var width = layout.width;
    if (qp.width != width) throw 'Differential IK QP has ${qp.width} variables, the problem ${width}';
    var work = workspace == null ? new SolverWorkspace() : workspace;
    work.prepare(problem);
    var rows = problem.rowCount();
    problem.evaluate(state, work.snapshot, work.residual, work.jacobian);
    var jacobian = work.jacobian.slice(0, rows * width);
    var residual = [for (row in 0...rows) gain * work.residual[row]];
    var lower = [for (_ in 0...width) 0.0], upper = [for (_ in 0...width) 0.0];
    (limits == null ? new StepLimits() : limits).bounds(problem, state, dt, lower, upper);
    var delta:Null<Array<Float>> = null;
    var status = -1, iterations = 0;
    try {
      var solved = qp.solve(jacobian, residual, lower, upper, damping, 1e-9, maxIterations);
      status = solved.status;
      iterations = solved.iterations;
      if (solved.solved()) delta = solved.step;
    } catch (_:Dynamic) {}
    var fallback = delta == null;
    var result:Array<Float> = delta == null ? [] : delta;
    if (fallback) {
      var damped = LinearAlgebra.dampedStep(jacobian, rows, width, [for (column in 0...width) column], residual,
        Math.max(damping, 1e-6));
      result = damped == null ? [for (_ in 0...width) 0.0] : damped;
      for (column in 0...width) result[column] = Math.min(Math.max(result[column], lower[column]), upper[column]);
    }
    var limited = [for (column in 0...width)
      if (result[column] <= lower[column] + 1e-12 || result[column] >= upper[column] - 1e-12) column];
    return new DifferentialStep([for (value in result) value / dt], status, iterations, fallback, limited);
  }
}
