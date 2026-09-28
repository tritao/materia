package robotkit.policy;

/**
 * The direction of gravity in the base frame, estimated from a gyroscope and
 * an accelerometer alone, as a robot's own controller has to: it has no
 * simulator to ask.
 *
 * The estimate is the unit vector d that points down. The gyroscope rotates
 * it, d' = -w x d, which drifts; the accelerometer measures specific force f,
 * which at rest is -g, so -f/|f| points down and pulls the estimate back with a
 * complementary-filter gain. While the robot accelerates, |f| differs from
 * gravity and the pull is faded out, so steps and swings do not tilt the
 * estimate. This is the projected-gravity vector that legged-locomotion
 * policies take, (0, 0, -1) when upright.
 */
class GravityEstimator {
  final gravity:Float;
  final gain:Float;
  final trust:Float;
  var down:Array<Float> = [0.0, 0.0, -1.0];
  var started:Bool = false;

  /**
   * `gain` is the accelerometer's pull in 1/s (the filter's time constant is
   * its inverse); `trust` is the fraction of gravity by which |f| may differ
   * before the pull is faded out entirely.
   */
  public function new(?gain:Float = 1.0, ?trust:Float = 0.3, ?gravity:Float = 9.81) {
    this.gain = gain;
    this.trust = trust;
    this.gravity = gravity;
  }

  public function reset():Void {
    started = false;
    down = [0.0, 0.0, -1.0];
  }

  /**
   * Feeds one sample, in the base frame: angular velocity (rad/s), specific
   * force (m/s^2) and the time since the previous sample. Returns the unit
   * down vector.
   */
  public function update(angularVelocity:Array<Float>, specificForce:Array<Float>, seconds:Float):Array<Float> {
    var norm = Math.sqrt(specificForce[0] * specificForce[0] + specificForce[1] * specificForce[1]
      + specificForce[2] * specificForce[2]);
    var measured = norm > 1e-6 ? [-specificForce[0] / norm, -specificForce[1] / norm, -specificForce[2] / norm] : null;
    if (!started) {
      if (measured != null) {
        down = measured;
        started = true;
      }
      return down.copy();
    }
    var w = angularVelocity;
    // d' = -w x d
    var rotated = [
      down[0] - seconds * (w[1] * down[2] - w[2] * down[1]),
      down[1] - seconds * (w[2] * down[0] - w[0] * down[2]),
      down[2] - seconds * (w[0] * down[1] - w[1] * down[0])
    ];
    if (measured != null) {
      var weight = Math.max(0.0, 1.0 - Math.abs(norm - gravity) / (trust * gravity));
      var pull = Math.min(1.0, gain * seconds * weight);
      for (i in 0...3) rotated[i] += pull * (measured[i] - rotated[i]);
    }
    var length = Math.sqrt(rotated[0] * rotated[0] + rotated[1] * rotated[1] + rotated[2] * rotated[2]);
    down = [rotated[0] / length, rotated[1] / length, rotated[2] / length];
    return down.copy();
  }
}
