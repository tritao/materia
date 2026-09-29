package robotkit.policy;

/**
 * The direction of gravity in the base frame, estimated from a gyroscope and
 * an accelerometer alone, as a robot's own controller has to: it has no
 * simulator to ask.
 *
 * The estimate is the unit vector d that points down. The gyroscope rotates
 * it, d' = -w x d, which drifts; the accelerometer measures specific force f,
 * which at rest is -g, so -f/|f| points down and pulls the estimate back with a
 * complementary-filter gain. Only while the robot is still does it pull: when
 * |f| differs from gravity, or the body turns, the pull fades out. Walking
 * accelerates the body in step with its bobbing, and an accelerometer averaged
 * over a gait reads a gravity tilted by 0.05 to 0.1 rad, which the policy
 * would then lean against. Feed it at the IMU's rate, not the control rate.
 * This is the projected-gravity vector that legged-locomotion policies take,
 * (0, 0, -1) when upright.
 */
class GravityEstimator {
  final gravity:Float;
  final gain:Float;
  final trust:Float;
  final stillRate:Float;
  var down:Array<Float> = [0.0, 0.0, -1.0];
  var started:Bool = false;

  /**
   * `gain` is the accelerometer's pull in 1/s when the robot is still (the
   * filter's time constant is its inverse); `trust` is the fraction of gravity
   * by which |f| may differ, and `stillRate` the angular rate (rad/s) at which
   * the body counts as moving, before the pull fades out entirely.
   */
  public function new(?gain:Float = 2.0, ?trust:Float = 0.1, ?stillRate:Float = 0.1, ?gravity:Float = 9.81) {
    this.gain = gain;
    this.trust = trust;
    this.stillRate = stillRate;
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
      // Start from the accelerometer only if it reads about 1 g and the body is still: a first sample taken
      // while the robot settles or is being handled says nothing about which way is down,
      // and the filter then starts upright and lets the accelerometer pull it in.
      if (measured != null) {
        var rate = Math.sqrt(angularVelocity[0] * angularVelocity[0] + angularVelocity[1] * angularVelocity[1]
          + angularVelocity[2] * angularVelocity[2]);
        if (Math.abs(norm - gravity) <= trust * gravity && rate <= stillRate) down = measured;
        started = true;
      }
      return down.copy();
    }
    // The body turns by w * dt, so the world's down turns by the opposite rotation in the body
    // frame. Rotate exactly, by Rodrigues' formula, using the rate at the end of the step, which is
    // what a semi-implicit Euler simulator integrates; stepping d' = -w x d instead drifts.
    var w = angularVelocity;
    var speed = Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    var rotated = down.copy();
    if (speed * seconds > 1e-12) {
      var angle = -speed * seconds;
      var axis = [w[0] / speed, w[1] / speed, w[2] / speed];
      var c = Math.cos(angle), sn = Math.sin(angle);
      var cross = [axis[1] * down[2] - axis[2] * down[1], axis[2] * down[0] - axis[0] * down[2], axis[0] * down[1] - axis[1] * down[0]];
      var along = axis[0] * down[0] + axis[1] * down[1] + axis[2] * down[2];
      for (i in 0...3) rotated[i] = down[i] * c + cross[i] * sn + axis[i] * along * (1.0 - c);
    }
    if (measured != null) {
      // Trust the accelerometer only while the robot is still: walking accelerates the body in step
      // with its bobbing, and the accelerometer then reads a gravity that leans with the gait.
      var weight = Math.max(0.0, 1.0 - Math.abs(norm - gravity) / (trust * gravity))
        * Math.max(0.0, 1.0 - speed / stillRate);
      var pull = Math.min(1.0, gain * seconds * weight);
      for (i in 0...3) rotated[i] += pull * (measured[i] - rotated[i]);
    }
    var length = Math.sqrt(rotated[0] * rotated[0] + rotated[1] * rotated[1] + rotated[2] * rotated[2]);
    down = [rotated[0] / length, rotated[1] / length, rotated[2] / length];
    return down.copy();
  }
}
