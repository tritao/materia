package motionkit.robot;

import kinematicskit.LinearAlgebra;
import kinematicskit.native.NativeQpStep;
import motionkit.kinematics.Twist6;
import robotkit.manipulation.Manipulator;

/** One servo tick's answer. */
class ServoStep {
  /** Joint velocities, one per arm DOF, in `q` order. */
  public final velocity:Array<Float>;
  /** True when the QP did not solve and the clamped Haxe damped step was used instead. */
  public final fallback:Bool;
  /** Why the QP stopped (`KinematicsKitNativeConstants.KK_QP_*`), or -1 when it threw. */
  public final qpStatus:Int;
  public final iterations:Int;
  /** Arm DOF indices whose step sits on a position or velocity bound. */
  public final limited:Array<Int>;

  public function new(velocity:Array<Float>, fallback:Bool, qpStatus:Int, iterations:Int, limited:Array<Int>) {
    this.velocity = velocity;
    this.fallback = fallback;
    this.qpStatus = qpStatus;
    this.iterations = iterations;
    this.limited = limited;
  }
}

/**
 * Bounded differential IK for live servoing (jogging, teleoperation): each
 * tick turns a requested tool twist into joint velocities for the next
 * period `dt`. It solves, natively on ProxQP,
 *
 *   minimize ½‖J·Δ − twist·dt‖² + ½λ²‖Δ‖²
 *   subject to  lower − q ≤ Δ ≤ upper − q   and   −v·dt ≤ Δ ≤ v·dt
 *
 * with `J` the tool-centre-point Jacobian in the base frame (as
 * `ManipulatorKinematics.solveDifferential`), the group's position limits
 * (`lower >= upper` means unlimited) and velocity limits (`velocity <= 0`
 * means unlimited, unless `velocityLimits` overrides them), and returns
 * Δ / dt. The limits hold exactly, so integrating the answer never leaves
 * the joint range. If the QP does not solve, the Haxe damped step is used,
 * clamped into the same bounds, and the answer says so.
 *
 * Keep one per arm and reuse it: the QP warm-starts from the previous tick.
 */
class ManipulatorServo {
  public final manipulator:Manipulator;
  public final damping:Float;
  final qp:NativeQpStep;
  final columns:Array<Int>;

  public function new(manipulator:Manipulator, ?damping:Float = 1e-3) {
    if (manipulator == null) throw "Servo requires a manipulator";
    if (!Math.isFinite(damping) || damping < 0.0) throw "Servo damping must be finite and non-negative";
    this.manipulator = manipulator;
    this.damping = damping;
    qp = new NativeQpStep(manipulator.dofCount());
    columns = [for (i in 0...manipulator.dofCount()) i];
  }

  public function step(q:Array<Float>, twist:Twist6, dt:Float, ?velocityLimits:Array<Float>,
      ?maxIterations:Int = 1000):ServoStep {
    var n = manipulator.dofCount();
    if (q == null || q.length != n) throw 'Servo requires $n joint values';
    if (twist == null) throw "Servo requires a tool twist";
    if (!(dt > 0.0) || !Math.isFinite(dt)) throw "Servo period must be positive and finite";
    if (velocityLimits != null && velocityLimits.length != n) throw 'Servo needs $n velocity limits';
    var jacobian = manipulator.tcpJacobian(q);
    var requested = twist.toArray();
    var displacement = [for (value in requested) value * dt];
    var lower:Array<Float> = [], upper:Array<Float> = [];
    for (joint in 0...n) {
      var limits = manipulator.group.limitsOf(joint);
      var low = Math.NEGATIVE_INFINITY, high = Math.POSITIVE_INFINITY;
      if (limits.lower < limits.upper) {
        low = limits.lower - q[joint];
        high = limits.upper - q[joint];
      }
      var speed = velocityLimits != null ? velocityLimits[joint] : (limits.velocity > 0.0 ? limits.velocity : Math.POSITIVE_INFINITY);
      if (!(speed >= 0.0)) throw "Servo velocity limits must be non-negative";
      low = Math.max(low, -speed * dt);
      high = Math.min(high, speed * dt);
      // A joint already outside its range may only move back towards it.
      if (low > 0.0) low = 0.0;
      if (high < 0.0) high = 0.0;
      lower.push(low);
      upper.push(high);
    }

    var delta:Array<Float> = null;
    var status = -1, iterations = 0;
    try {
      var solved = qp.solve(jacobian, displacement, lower, upper, damping, 1e-9, maxIterations);
      status = solved.status;
      iterations = solved.iterations;
      if (solved.solved()) delta = solved.step;
    } catch (_:Dynamic) {}
    var fallback = delta == null;
    if (fallback) {
      delta = LinearAlgebra.dampedStep(jacobian, 6, n, columns, displacement, Math.max(damping, 1e-6));
      if (delta == null) delta = [for (_ in 0...n) 0.0];
      for (joint in 0...n) delta[joint] = Math.min(Math.max(delta[joint], lower[joint]), upper[joint]);
    }
    var limited = [for (joint in 0...n) if (delta[joint] <= lower[joint] + 1e-12 || delta[joint] >= upper[joint] - 1e-12) joint];
    return new ServoStep([for (value in delta) value / dt], fallback, status, iterations, limited);
  }

  public function dispose():Void qp.dispose();
}
