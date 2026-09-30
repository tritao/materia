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
 *   subject to  k·(lower − q) ≤ Δ ≤ k·(upper − q)   and   −v·dt ≤ Δ ≤ v·dt
 *
 * with `J` the tool-centre-point Jacobian in the base frame (as
 * `ManipulatorKinematics.solveDifferential`), the group's position limits
 * (`lower >= upper` means unlimited) and velocity limits (`velocity <= 0`
 * means unlimited, unless `velocityLimits` overrides them), and returns
 * Δ / dt. `limitGain` k in (0, 1] (1 by default) lets a joint cover at most
 * that fraction of its remaining distance to a stop per tick, so it slows
 * into the stop instead of arriving in one tick. The limits hold exactly, so integrating the answer never leaves
 * the joint range. If the QP does not solve, the Haxe damped step is used,
 * clamped into the same bounds, and the answer says so.
 *
 * With `previousVelocity` (the velocities commanded last tick) the step is
 * also acceleration-limited: each joint's velocity changes by at most a·dt
 * (the group's `maxAcceleration`, or `accelerationLimits`; 0 = unknown =
 * unlimited), and it never approaches a stop faster than it can brake: one
 * tick of travel plus a full brake must fit in the distance left,
 * |v|·dt + v²/(2a) ≤ distance (which also keeps the braking itself within a). A zero twist then brakes the arm to rest within its
 * acceleration limits. Should the bounds conflict (a joint already moving too
 * fast near a stop), the position and braking bounds win.
 *
 * `ramped` says the velocity will ramp linearly from `previousVelocity` to
 * the answer over the tick (a streamed plan chunk) rather than switch at
 * once, so a joint travels (before + v)·dt/2. The position and braking
 * bounds then count that travel, (before + v)·dt/2 + v²/(2a) ≤ distance,
 * and a ramp that turns back within the tick keeps its turning point in range.
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
      ?maxIterations:Int = 1000, ?limitGain:Float = 1.0, ?previousVelocity:Array<Float>,
      ?accelerationLimits:Array<Float>, ?ramped:Bool = false):ServoStep {
    if (ramped && previousVelocity == null) throw "A ramped servo step needs the previous velocities";
    var n = manipulator.dofCount();
    if (q == null || q.length != n) throw 'Servo requires $n joint values';
    if (twist == null) throw "Servo requires a tool twist";
    if (!(dt > 0.0) || !Math.isFinite(dt)) throw "Servo period must be positive and finite";
    if (velocityLimits != null && velocityLimits.length != n) throw 'Servo needs $n velocity limits';
    if (!(limitGain > 0.0) || limitGain > 1.0) throw "Servo limit gain must be in (0, 1]";
    if (previousVelocity != null && previousVelocity.length != n) throw 'Servo needs $n previous velocities';
    if (accelerationLimits != null && accelerationLimits.length != n) throw 'Servo needs $n acceleration limits';
    var jacobian = manipulator.tcpJacobian(q);
    var requested = twist.toArray();
    var displacement = [for (value in requested) value * dt];
    var lower:Array<Float> = [], upper:Array<Float> = [];
    for (joint in 0...n) {
      var limits = manipulator.group.limitsOf(joint);
      var limited = limits.lower < limits.upper;
      // Displacement bounds for this tick.
      var low = Math.NEGATIVE_INFINITY, high = Math.POSITIVE_INFINITY;
      if (limited) {
        low = limitGain * (limits.lower - q[joint]);
        high = limitGain * (limits.upper - q[joint]);
      }
      var speed = velocityLimits != null ? velocityLimits[joint] : (limits.velocity > 0.0 ? limits.velocity : Math.POSITIVE_INFINITY);
      if (!(speed >= 0.0)) throw "Servo velocity limits must be non-negative";
      var vLow = -speed, vHigh = speed;
      var accel = accelerationLimits != null ? accelerationLimits[joint] : limits.maxAcceleration;
      if (!(accel >= 0.0)) throw "Servo acceleration limits must be non-negative";
      if (previousVelocity != null && accel > 0.0) {
        var before = previousVelocity[joint];
        // Braking: one tick of travel plus a full brake must fit in the distance to the stop.
        var safeLow = vLow, safeHigh = vHigh;
        if (limited && ramped) {
          safeHigh = Math.min(vHigh, rampedBrakingSpeed(accel, dt, before, limitGain * (limits.upper - q[joint])));
          safeLow = Math.max(vLow, -rampedBrakingSpeed(accel, dt, -before, limitGain * (q[joint] - limits.lower)));
        } else if (limited) {
          safeHigh = Math.min(vHigh, brakingSpeed(accel, dt, limitGain * (limits.upper - q[joint])));
          safeLow = Math.max(vLow, -brakingSpeed(accel, dt, limitGain * (q[joint] - limits.lower)));
        }
        var accelLow = before - accel * dt, accelHigh = before + accel * dt;
        vLow = Math.max(safeLow, accelLow);
        vHigh = Math.min(safeHigh, accelHigh);
        // Entered too fast near a stop: position and braking win over acceleration.
        if (vLow > vHigh) {
          if (accelLow > safeHigh) { vLow = safeHigh; vHigh = safeHigh; }
          else { vLow = safeLow; vHigh = safeLow; }
        }
      }
      // A joint already outside its range may stay or move back, not further out.
      if (low > 0.0) low = 0.0;
      if (high < 0.0) high = 0.0;
      // Ramped, the tick's travel is (before + v)·dt/2: bound v·dt so that travel stays in range.
      if (ramped && limited) {
        var drift = previousVelocity[joint] * dt;
        low = 2.0 * low - drift;
        high = 2.0 * high - drift;
      }
      var stepLow = Math.max(low, vLow * dt), stepHigh = Math.min(high, vHigh * dt);
      // Should velocity and position bounds exclude each other, the position side wins.
      if (stepLow > stepHigh) {
        if (vLow * dt > high) { stepLow = high; stepHigh = high; }
        else { stepLow = low; stepHigh = low; }
      }
      lower.push(stepLow);
      upper.push(stepHigh);
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

  /**
   * The fastest speed v such that ramping from `before` to v over dt, then
   * braking at a, stays within `distance`: (before + v)·dt/2 + v²/(2a) ≤ distance.
   */
  static function rampedBrakingSpeed(accel:Float, dt:Float, before:Float, distance:Float):Float {
    var left = distance - 0.5 * before * dt;
    if (left > 0.0) return accel * (Math.sqrt(0.25 * dt * dt + 2.0 * left / accel) - 0.5 * dt);
    // The joint must turn back within this tick: keep the ramp's turning point,
    // before²·dt / (2·(before − v)), inside the distance.
    if (distance > 0.0 && before > 0.0) return before - before * before * dt / (2.0 * distance);
    return 2.0 * left / dt;
  }

  /** The fastest speed from which v·dt + v²/(2a) ≤ distance: a·(√(dt² + 2·distance/a) − dt). */
  static function brakingSpeed(accel:Float, dt:Float, distance:Float):Float {
    if (!(distance > 0.0)) return 0.0;
    return accel * (Math.sqrt(dt * dt + 2.0 * distance / accel) - dt);
  }
}
