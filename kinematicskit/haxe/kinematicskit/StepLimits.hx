package kinematicskit;

/**
 * Bounds on one differential step Δ over a period `dt` (one per layout
 * column: the active DOFs, then any moving roots), for servoing and other
 * per-tick solves (KINEMATICS.md KK-D17).
 *
 * - Configuration: a DOF covers at most `limitGain` (in (0, 1]) of its
 *   remaining distance to a stop per step; one already outside its range may
 *   only move back.
 * - Velocity: |Δ| ≤ v·dt with `velocity` per column (unlimited without).
 * - Acceleration, with `previousVelocity`: the velocity changes by at most
 *   a·dt per step (`acceleration` per DOF; 0 = unlimited), and a DOF never
 *   approaches a stop faster than it can brake: one step of travel plus a
 *   full brake fits in the distance left, |v|·dt + v²/(2a) ≤ distance. Where
 *   the two conflict (a DOF already too fast near a stop), the position and
 *   braking bounds win.
 * - `ramped`: the velocity ramps linearly from the previous one to the new
 *   one over the step (a streamed plan chunk) instead of switching at once,
 *   so a DOF travels (before + v)·dt/2, and a ramp that turns back within
 *   the step keeps its turning point inside the range.
 *
 * Should velocity and position bounds exclude each other, the position side
 * wins.
 */
class StepLimits {
  public var velocity:Null<Array<Float>> = null;
  public var acceleration:Null<Array<Float>> = null;
  public var previousVelocity:Null<Array<Float>> = null;
  public var limitGain:Float = 1.0;
  public var ramped:Bool = false;

  public function new() {}

  /** Velocity limits per column (and a configuration-limit gain): mink's differential-IK limits. */
  public static function ofVelocity(?velocity:Array<Float>, ?limitGain:Float = 1.0):StepLimits {
    var result = new StepLimits();
    result.velocity = velocity == null ? null : velocity.copy();
    result.limitGain = limitGain;
    return result;
  }

  /** Writes the step bounds for `state` into `lower` and `upper` (one per layout column). */
  public function bounds(problem:KinematicProblem, state:KinematicState, dt:Float, lower:Array<Float>,
      upper:Array<Float>):Void {
    var layout = problem.layout();
    var width = layout.width;
    var dofCount = layout.dofs.length;
    if (!(dt > 0.0) || !Math.isFinite(dt)) throw "Step limits need a positive period";
    if (!(limitGain > 0.0) || limitGain > 1.0) throw "Step limit gain must be in (0, 1]";
    for (values in [velocity, previousVelocity])
      if (values != null && values.length != width) throw 'Step limits need one value per column ($width)';
    if (acceleration != null && acceleration.length != width)
      throw 'Step limits need one acceleration per column ($width)';
    if (ramped && previousVelocity == null) throw "A ramped step needs the previous velocities";
    for (column in 0...width) {
      // Distances to the stops (infinite without one, and for moving roots); each side bounds itself.
      var below = Math.POSITIVE_INFINITY, above = Math.POSITIVE_INFINITY;
      if (column < dofCount) {
        var dof = layout.dofs[column];
        var q = state.q[dof];
        if (problem.lower[dof] > Math.NEGATIVE_INFINITY) below = limitGain * (q - problem.lower[dof]);
        if (problem.upper[dof] < Math.POSITIVE_INFINITY) above = limitGain * (problem.upper[dof] - q);
      }
      var low = -below, high = above;
      var speed = velocity != null ? velocity[column] : Math.POSITIVE_INFINITY;
      if (!(speed >= 0.0)) throw "Step velocity limits must be non-negative";
      var vLow = -speed, vHigh = speed;
      var accel = acceleration != null ? acceleration[column] : 0.0;
      if (!(accel >= 0.0)) throw "Step acceleration limits must be non-negative";
      var previous:Array<Float> = previousVelocity;
      if (previous != null && accel > 0.0) {
        var before = previous[column];
        // Braking: one step of travel (or ramp) plus a full brake fits in the distance to the stop.
        var safeLow = vLow, safeHigh = vHigh;
        if (ramped) {
          safeHigh = Math.min(vHigh, rampedBrakingSpeed(accel, dt, before, above));
          safeLow = Math.max(vLow, -rampedBrakingSpeed(accel, dt, -before, below));
        } else {
          safeHigh = Math.min(vHigh, brakingSpeed(accel, dt, above));
          safeLow = Math.max(vLow, -brakingSpeed(accel, dt, below));
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
      // A DOF already outside its range may stay or move back, not further out.
      if (low > 0.0) low = 0.0;
      if (high < 0.0) high = 0.0;
      // Ramped, the step's travel is (before + v)·dt/2: bound v·dt so that travel stays in range.
      if (ramped) {
        var drift = previous[column] * dt;
        low = 2.0 * low - drift;
        high = 2.0 * high - drift;
      }
      var stepLow = Math.max(low, vLow * dt), stepHigh = Math.min(high, vHigh * dt);
      // Should velocity and position bounds exclude each other, the position side wins.
      if (stepLow > stepHigh) {
        if (vLow * dt > high) { stepLow = high; stepHigh = high; }
        else { stepLow = low; stepHigh = low; }
      }
      lower[column] = stepLow;
      upper[column] = stepHigh;
    }
  }

  /**
   * The fastest speed v such that ramping from `before` to v over dt, then
   * braking at a, stays within `distance`: (before + v)·dt/2 + v²/(2a) ≤ distance.
   */
  public static function rampedBrakingSpeed(accel:Float, dt:Float, before:Float, distance:Float):Float {
    if (distance == Math.POSITIVE_INFINITY) return Math.POSITIVE_INFINITY;
    var left = distance - 0.5 * before * dt;
    if (left > 0.0) return accel * (Math.sqrt(0.25 * dt * dt + 2.0 * left / accel) - 0.5 * dt);
    // The DOF must turn back within this step: keep the ramp's turning point,
    // before²·dt / (2·(before − v)), inside the distance.
    if (distance > 0.0 && before > 0.0) return before - before * before * dt / (2.0 * distance);
    return 2.0 * left / dt;
  }

  /** The fastest speed from which v·dt + v²/(2a) ≤ distance: a·(√(dt² + 2·distance/a) − dt). */
  public static function brakingSpeed(accel:Float, dt:Float, distance:Float):Float {
    if (!(distance > 0.0)) return 0.0;
    if (distance == Math.POSITIVE_INFINITY) return Math.POSITIVE_INFINITY;
    return accel * (Math.sqrt(dt * dt + 2.0 * distance / accel) - dt);
  }
}
