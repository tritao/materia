package kinematicskit.native;

import KinematicsKitNative;

/** Outcome of one `NativeQpStep.solve`. */
class QpStepResult {
  /** The step, one entry per variable (the solver's last iterate when it did not solve). */
  public final step:Array<Float>;
  /** `KinematicsKitNativeConstants.KK_QP_*`. */
  public final status:Int;
  public final iterations:Int;

  public function new(step:Array<Float>, status:Int, iterations:Int) {
    this.step = step;
    this.status = status;
    this.iterations = iterations;
  }

  public function solved():Bool return status == KinematicsKitNativeConstants.KK_QP_SOLVED;
}

/**
 * The native bounded damped least-squares step (ProxQP, dense):
 * minimize ½‖J·Δ − e‖² + ½λ²‖Δ‖² subject to lower ≤ Δ ≤ upper. Keep one per
 * problem width and reuse it; every solve warm-starts from the previous one.
 */
class NativeQpStep {
  public final width:Int;
  final owner:Ownedkk_qp_handle;
  var disposed = false;

  public function new(width:Int) {
    if (width <= 0) throw "Native QP step needs at least one variable";
    this.width = width;
    var created = KinematicsKitNative.kk_qp_create(width);
    if (created.status != KinematicsKitNativeConstants.KK_OK)
      throw 'Native QP step creation failed with error ${created.status}';
    owner = created.out_qp;
  }

  /** `jacobian` is `residual.length` x `width`, row-major; infinite bounds mean unbounded. */
  public function solve(jacobian:Array<Float>, residual:Array<Float>, lower:Array<Float>, upper:Array<Float>,
      damping:Float, ?tolerance:Float = 1e-9, ?maxIterations:Int = 1000):QpStepResult {
    if (disposed) throw "Native QP step has been disposed";
    var result = KinematicsKitNative.kk_qp_solve(owner.borrow(), jacobian, residual, lower, upper, damping, tolerance,
      maxIterations, width);
    if (result.status != KinematicsKitNativeConstants.KK_OK)
      throw 'Native QP step failed with error ${result.status}';
    return new QpStepResult(result.out_step, result.out_status, result.out_iterations);
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }
}
