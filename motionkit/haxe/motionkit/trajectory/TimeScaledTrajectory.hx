package motionkit.trajectory;

/**
 * A re-timed trajectory together with the planned (source) trajectory it
 * follows. Each executed sample records the source time it reached and the
 * clock rate there, so progress can still be reported against the plan.
 */
class TimeScaledTrajectory {
  public final source:JointTrajectory;
  public final trajectory:JointTrajectory;
  /** Source time reached at the end of `trajectory`. */
  public final sourceEndSeconds:Float;
  final sourceTimes:Array<Float>;
  final rates:Array<Float>;

  public function new(source:JointTrajectory, trajectory:JointTrajectory,
      sourceTimes:Array<Float>, rates:Array<Float>, sourceEndSeconds:Float) {
    if (source == null || trajectory == null) throw "Time-scaled trajectory needs both trajectories";
    if (sourceTimes.length != trajectory.samples.length || rates.length != trajectory.samples.length)
      throw "Time-scaled trajectory needs one source time and rate per sample";
    this.source = source;
    this.trajectory = trajectory;
    this.sourceTimes = sourceTimes.copy();
    this.rates = rates.copy();
    this.sourceEndSeconds = sourceEndSeconds;
  }

  /** Source time corresponding to `timeSeconds` along the re-timed trajectory. */
  public function sourceTimeAt(timeSeconds:Float):Float
    return interpolate(sourceTimes, timeSeconds);

  /** Source clock rate at `timeSeconds`: 0 at rest, 1 on the planned timing. */
  public function rateAt(timeSeconds:Float):Float
    return interpolate(rates, timeSeconds);

  function interpolate(values:Array<Float>, timeSeconds:Float):Float {
    var samples = trajectory.samples;
    if (timeSeconds <= samples[0].timeSeconds) return values[0];
    var last = samples.length - 1;
    if (timeSeconds >= samples[last].timeSeconds) return values[last];
    var low = 0;
    var high = last;
    while (low + 1 < high) {
      var middle = Std.int((low + high) / 2);
      if (samples[middle].timeSeconds <= timeSeconds) low = middle;
      else high = middle;
    }
    var span = samples[high].timeSeconds - samples[low].timeSeconds;
    var alpha = span <= 0.0 ? 0.0 : (timeSeconds - samples[low].timeSeconds) / span;
    return values[low] + (values[high] - values[low]) * alpha;
  }
}
