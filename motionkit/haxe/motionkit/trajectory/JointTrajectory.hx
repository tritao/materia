package motionkit.trajectory;

/** Deterministic, immutable sequence of timed joint samples. */
class JointTrajectory {
  public final samples:Array<JointTrajectorySample>;
  public final jointCount:Int;
  public final durationSeconds:Float;

  public function new(samples:Array<JointTrajectorySample>) {
    if (samples == null || samples.length == 0)
      throw "Joint trajectory needs at least one sample";
    this.samples = samples.copy();
    jointCount = samples[0].positions.length;
    var previousTime = -1.0;
    for (sample in this.samples) {
      if (sample == null) throw "Joint trajectory cannot contain null samples";
      if (sample.positions.length != jointCount)
        throw "Joint trajectory samples have inconsistent joint counts";
      if (sample.timeSeconds < previousTime)
        throw "Joint trajectory sample times must be monotonic";
      previousTime = sample.timeSeconds;
    }
    durationSeconds = this.samples[this.samples.length - 1].timeSeconds;
  }

  /** Samples with linear interpolation between the deterministic source samples. */
  public function sample(timeSeconds:Float):JointTrajectorySample {
    if (!Math.isFinite(timeSeconds)) throw "Trajectory sample time must be finite";
    if (timeSeconds <= samples[0].timeSeconds) return copySample(samples[0]);
    if (timeSeconds >= durationSeconds) return copySample(samples[samples.length - 1]);

    var low = 0;
    var high = samples.length - 1;
    while (low + 1 < high) {
      var middle = Std.int((low + high) / 2);
      if (samples[middle].timeSeconds <= timeSeconds) low = middle;
      else high = middle;
    }
    var before = samples[low];
    var after = samples[high];
    var span = after.timeSeconds - before.timeSeconds;
    var alpha = span <= 0.0 ? 0.0 : (timeSeconds - before.timeSeconds) / span;
    return interpolate(before, after, alpha, timeSeconds);
  }

  static function interpolate(before:JointTrajectorySample, after:JointTrajectorySample,
      alpha:Float, timeSeconds:Float):JointTrajectorySample {
    var positions:Array<Float> = [];
    var velocities:Array<Float> = [];
    var accelerations:Array<Float> = [];
    for (i in 0...before.positions.length) {
      positions.push(lerp(before.positions[i], after.positions[i], alpha));
      velocities.push(lerp(before.velocities[i], after.velocities[i], alpha));
      accelerations.push(lerp(before.accelerations[i], after.accelerations[i], alpha));
    }
    return new JointTrajectorySample(timeSeconds, positions, velocities, accelerations);
  }

  static function lerp(a:Float, b:Float, alpha:Float):Float return a + (b - a) * alpha;

  static function copySample(sample:JointTrajectorySample):JointTrajectorySample
    return new JointTrajectorySample(sample.timeSeconds, sample.positions,
      sample.velocities, sample.accelerations);
}
