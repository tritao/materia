package motionkit.trajectory;

/**
 * Re-times part of a trajectory so it slows to rest, or speeds up from rest,
 * along its own path while every joint stays within its acceleration limit.
 *
 * The trajectory's clock runs at a rate between 0 and 1; a joint then moves
 * at rate * v and accelerates at rate' * v + rate^2 * a, where v and a belong
 * to the source trajectory. Samples are piecewise linear, so the source's own
 * speed changes arrive as steps at its samples rather than smoothly: the rate
 * change is budgeted conservatively, never counting on the source's motion to
 * help and allowing a whole step to land within one period. This matches the
 * RobotKit runtime's path-following stop.
 */
class TimeScaling {
  static inline var SUBSTEPS:Int = 16;
  static inline var MAX_PERIODS:Int = 1000000;

  /**
   * Slows the source from `startRate` at `sourceStartSeconds` to rest.
   * `jointAccelerationLimits` gives each joint's limit; zero leaves a joint
   * unconstrained. The result starts at the source's pose at that time.
   */
  public static function stop(source:JointTrajectory, sourceStartSeconds:Float, startRate:Float,
      jointAccelerationLimits:Array<Float>, periodSeconds:Float):TimeScaledTrajectory {
    return retime(source, sourceStartSeconds, startRate, 0.0, jointAccelerationLimits,
      periodSeconds);
  }

  /**
   * Speeds the source up from rest at `sourceStartSeconds` until it runs at
   * its own timing, then follows the rest of the source unchanged.
   */
  public static function start(source:JointTrajectory, sourceStartSeconds:Float,
      jointAccelerationLimits:Array<Float>, periodSeconds:Float):TimeScaledTrajectory {
    return retime(source, sourceStartSeconds, 0.0, 1.0, jointAccelerationLimits,
      periodSeconds);
  }

  static function retime(source:JointTrajectory, sourceStartSeconds:Float, startRate:Float,
      targetRate:Float, limits:Array<Float>, periodSeconds:Float):TimeScaledTrajectory {
    if (source == null || limits == null) throw "Time scaling needs a trajectory and limits";
    if (!Math.isFinite(periodSeconds) || periodSeconds <= 0.0)
      throw "Time scaling period must be finite and positive";
    if (limits.length != source.jointCount)
      throw "Time scaling needs one acceleration limit per joint";
    var sourceTime = Math.max(0.0, Math.min(source.durationSeconds, sourceStartSeconds));
    var rate = Math.max(0.0, Math.min(1.0, startRate));
    var speedingUp = targetRate > rate;
    var jointCount = source.jointCount;

    var samples:Array<JointTrajectorySample> = [];
    var sourceTimes:Array<Float> = [];
    var rates:Array<Float> = [];
    var velocities = [for (_ in 0...jointCount) 0.0];
    var accelerations = [for (_ in 0...jointCount) 0.0];
    var recentAccelerations = [for (_ in 0...jointCount) 0.0];
    function record(time:Float):Void {
      chordVelocity(source, sourceTime, velocities);
      samples.push(new JointTrajectorySample(time, source.sample(sourceTime).positions,
        [for (joint in 0...jointCount) rate * velocities[joint]]));
      sourceTimes.push(sourceTime);
      rates.push(rate);
    }

    var time = 0.0;
    record(time);
    var stepSeconds = periodSeconds / SUBSTEPS;
    var periods = 0;
    while (rate != targetRate && sourceTime < source.durationSeconds) {
      if (++periods > MAX_PERIODS) throw "Time scaling did not converge";
      for (_ in 0...SUBSTEPS) {
        if (sourceTime >= source.durationSeconds) break;
        if (rate == targetRate) {
          // Reached the target partway through this period: the source keeps
          // running at that rate for the rest of it, so the recorded sample
          // at the period's end stays on the source's own timing.
          sourceTime = Math.min(source.durationSeconds, sourceTime + rate * stepSeconds);
          continue;
        }
        chordAcceleration(source, Math.max(0.0, sourceTime - periodSeconds), periodSeconds,
          velocities, recentAccelerations);
        chordVelocity(source, sourceTime, velocities);
        chordAcceleration(source, sourceTime, periodSeconds, velocities, accelerations);
        var maxChange = Math.POSITIVE_INFINITY;
        for (joint in 0...jointCount) {
          var limit = limits[joint];
          var speed = Math.abs(velocities[joint]);
          if (limit <= 0.0 || speed <= 1e-12) continue;
          var direction = velocities[joint] > 0.0 ? 1.0 : -1.0;
          // Speeding up must leave room for the source's own speed-up, and
          // slowing down for its own braking, both taken at rate * |a|. A
          // step one period behind can still fall within the current period,
          // so the budget covers the source's changes on both sides.
          var sign = speedingUp ? direction : -direction;
          var alongChange = Math.max(sign * accelerations[joint], sign * recentAccelerations[joint]);
          maxChange = Math.min(maxChange,
            (limit - rate * Math.max(0.0, alongChange)) / speed);
        }
        var nextRate:Float;
        if (!Math.isFinite(maxChange)) {
          // Nothing constrained is moving here, so the rate can jump.
          nextRate = targetRate;
        } else {
          var change = Math.max(0.0, maxChange) * stepSeconds;
          nextRate = speedingUp ? Math.min(targetRate, rate + change)
            : Math.max(targetRate, rate - change);
        }
        sourceTime = Math.min(source.durationSeconds,
          sourceTime + 0.5 * (rate + nextRate) * stepSeconds);
        rate = nextRate;
      }
      time += periodSeconds;
      record(time);
    }

    var sourceEnd = sourceTime;
    if (speedingUp && sourceTime < source.durationSeconds) {
      // Up to speed: the rest of the source follows on its own timing.
      for (sample in source.samples) {
        if (sample.timeSeconds <= sourceTime + 1e-12) continue;
        samples.push(new JointTrajectorySample(time + sample.timeSeconds - sourceTime,
          sample.positions, sample.velocities, sample.accelerations));
        sourceTimes.push(sample.timeSeconds);
        rates.push(1.0);
      }
      sourceEnd = source.durationSeconds;
    }
    return new TimeScaledTrajectory(source, new JointTrajectory(samples), sourceTimes, rates,
      sourceEnd);
  }

  /** Velocity of the source segment being entered at `time`; zero at the end. */
  static function chordVelocity(source:JointTrajectory, time:Float, out:Array<Float>):Float {
    var samples = source.samples;
    for (index in 0...(samples.length - 1)) {
      var before = samples[index];
      var after = samples[index + 1];
      if (after.timeSeconds <= time || after.timeSeconds <= before.timeSeconds) continue;
      var span = after.timeSeconds - before.timeSeconds;
      for (joint in 0...out.length)
        out[joint] = (after.positions[joint] - before.positions[joint]) / span;
      return 0.5 * (before.timeSeconds + after.timeSeconds);
    }
    for (joint in 0...out.length) out[joint] = 0.0;
    return time;
  }

  /**
   * Source acceleration at `time`, from this segment's velocity and the one
   * a window ahead over the time between their midpoints. Leaves `out`
   * unchanged when no later segment exists, keeping the previous estimate.
   */
  static function chordAcceleration(source:JointTrajectory, time:Float, windowSeconds:Float,
      velocities:Array<Float>, out:Array<Float>):Void {
    if (time >= source.durationSeconds) return;
    var midpoint = chordVelocity(source, time, velocities);
    var ahead = [for (_ in velocities) 0.0];
    var aheadTime = Math.min(time + windowSeconds, source.durationSeconds - 1e-12);
    var aheadMidpoint = chordVelocity(source, aheadTime, ahead);
    var spacing = aheadMidpoint - midpoint;
    if (spacing <= 1e-12) return;
    for (joint in 0...out.length)
      out[joint] = (ahead[joint] - velocities[joint]) / spacing;
  }
}
