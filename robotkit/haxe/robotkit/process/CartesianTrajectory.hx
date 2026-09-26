package robotkit.process;

import robotkit.spatial.Transform3;

/**
 * Time-parameterized TCP samples built from a `Toolpath`: each consecutive
 * pair of points gets its own independent trapezoidal (or triangular, if too
 * short to reach cruise speed) velocity profile toward the *arriving*
 * point's feed rate, capped by `maxAcceleration`. Samples are bounded by the
 * requested time interval, linear distance step, and rotation angle step.
 * Position is linearly interpolated along the straight line between the two
 * points; rotation is slerped using the same normalized profile fraction, so
 * translation and rotation always reach their segment endpoint together.
 */
class CartesianTrajectory {
  public final toolpath:Toolpath;
  public final maxAcceleration:Float;
  public final samples:Array<CartesianTrajectorySample>;

  function new(toolpath:Toolpath, maxAcceleration:Float, samples:Array<CartesianTrajectorySample>) {
    this.toolpath = toolpath;
    this.maxAcceleration = maxAcceleration;
    this.samples = samples;
  }

  public function duration():Float return samples.length == 0 ? 0.0 : samples[samples.length - 1].time;

  public static function build(toolpath:Toolpath, maxAcceleration:Float, sampleInterval:Float,
      ?maxLinearStep:Float = 0.05, ?maxAngularStep:Float = Math.PI / 36.0):CartesianTrajectory {
    if (toolpath == null) throw "Cartesian trajectory requires a toolpath";
    if (!Math.isFinite(maxAcceleration) || maxAcceleration <= 0.0)
      throw "Cartesian trajectory max acceleration must be positive and finite";
    if (!Math.isFinite(sampleInterval) || sampleInterval <= 0.0)
      throw "Cartesian trajectory sample interval must be positive and finite";
    if (!Math.isFinite(maxLinearStep) || maxLinearStep <= 0.0)
      throw "Cartesian trajectory max linear step must be positive and finite";
    if (!Math.isFinite(maxAngularStep) || maxAngularStep <= 0.0)
      throw "Cartesian trajectory max angular step must be positive and finite";
    var points = toolpath.points;
    var samples:Array<CartesianTrajectorySample> = [];
    if (points.length == 0) return new CartesianTrajectory(toolpath, maxAcceleration, samples);
    samples.push(new CartesianTrajectorySample(points[0].work_T_tcp, 0.0, points[0].processOn, 0));

    var elapsedTotal = 0.0;
    for (i in 0...(points.length - 1)) {
      var from = points[i];
      var to = points[i + 1];
      var delta = to.work_T_tcp.translation.sub(from.work_T_tcp.translation);
      var distance = delta.norm();
      var rotationAngle = from.work_T_tcp.rotation.angularDistance(to.work_T_tcp.rotation);
      // A pure reorientation still needs a timed profile. Feed rate is the
      // only per-point rate available at this layer, so angular distance is
      // profiled in the same units when there is no linear travel.
      var profiledDistance = distance <= 1e-12 ? rotationAngle : distance;
      var profile = new TrapezoidalProfile(profiledDistance, to.feedRate, maxAcceleration);
      var timeSteps = profile.duration <= 0.0 ? 1 : Math.ceil(profile.duration / sampleInterval);
      var linearSteps = Math.ceil(distance / maxLinearStep);
      var angularSteps = Math.ceil(rotationAngle / maxAngularStep);
      var steps = Math.max(1, Math.max(timeSteps, Math.max(linearSteps, angularSteps)));
      if (steps < 1) steps = 1;
      var step = 1;
      while (step <= steps) {
        var elapsed = step == steps ? profile.duration : profile.duration * step / steps;
        var fraction = profiledDistance <= 1e-12 ? 1.0 : profile.distanceAt(elapsed) / profiledDistance;
        var translation = from.work_T_tcp.translation.add(delta.scale(fraction));
        var rotation = from.work_T_tcp.rotation.slerp(to.work_T_tcp.rotation, fraction);
        var sampleTime = elapsedTotal + elapsed;
        samples.push(new CartesianTrajectorySample(new Transform3(translation, rotation), sampleTime, from.processOn, i));
        step++;
      }
      elapsedTotal += profile.duration;
    }
    return new CartesianTrajectory(toolpath, maxAcceleration, samples);
  }
}

/**
 * Symmetric trapezoidal (or triangular, when the target velocity can't be
 * reached in the available distance) velocity profile from rest to rest.
 */
private class TrapezoidalProfile {
  public final distance:Float;
  public final duration:Float;
  final accel:Float;
  final accelTime:Float;
  final cruiseTime:Float;
  final peakVelocity:Float;

  public function new(distance:Float, targetVelocity:Float, maxAcceleration:Float) {
    if (!Math.isFinite(distance) || distance < 0.0) throw "Trapezoidal profile distance must be finite and non-negative";
    if (!Math.isFinite(targetVelocity) || targetVelocity <= 0.0)
      throw "Trapezoidal profile target velocity must be positive and finite";
    this.distance = distance;
    this.accel = maxAcceleration;
    if (distance <= 1e-12) {
      accelTime = 0.0;
      cruiseTime = 0.0;
      peakVelocity = 0.0;
      duration = 0.0;
      return;
    }
    var accelTimeToTarget = targetVelocity / maxAcceleration;
    var accelDistanceToTarget = 0.5 * maxAcceleration * accelTimeToTarget * accelTimeToTarget;
    if (2.0 * accelDistanceToTarget <= distance) {
      accelTime = accelTimeToTarget;
      peakVelocity = targetVelocity;
      cruiseTime = (distance - 2.0 * accelDistanceToTarget) / targetVelocity;
    } else {
      peakVelocity = Math.sqrt(maxAcceleration * distance);
      accelTime = peakVelocity / maxAcceleration;
      cruiseTime = 0.0;
    }
    duration = 2.0 * accelTime + cruiseTime;
  }

  /** Distance traveled `elapsed` seconds into this segment, clamped to [0, distance]. */
  public function distanceAt(elapsed:Float):Float {
    if (elapsed <= 0.0) return 0.0;
    if (elapsed >= duration) return distance;
    var accelDistance = 0.5 * accel * accelTime * accelTime;
    if (elapsed <= accelTime) return 0.5 * accel * elapsed * elapsed;
    var decelStart = accelTime + cruiseTime;
    if (elapsed <= decelStart) return accelDistance + peakVelocity * (elapsed - accelTime);
    var tDecel = elapsed - decelStart;
    var cruiseDistance = peakVelocity * cruiseTime;
    return accelDistance + cruiseDistance + peakVelocity * tDecel - 0.5 * accel * tDecel * tDecel;
  }
}
