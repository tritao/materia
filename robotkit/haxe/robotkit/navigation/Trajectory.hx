package robotkit.navigation;

import robotkit.mobile.Pose2;
import robotkit.mobile.Twist2;

/** Immutable time-parameterized planar path. */
class Trajectory {
  final values:Array<TrajectorySample>;
  public final frameId:String;
  public final durationSeconds:Float;

  public function new(samples:Array<TrajectorySample>, ?frameId:String = "map") {
    if (samples == null || samples.length == 0)
      throw "A trajectory requires at least one sample";
    if (frameId == null || frameId.length == 0)
      throw "A trajectory requires a frame ID";
    this.frameId = frameId;
    values = [];
    var previousTime = -1.0;
    for (sample in samples) {
      if (sample == null || sample.timeFromStartSeconds <= previousTime)
        throw "Trajectory sample times must be strictly increasing";
      values.push(new TrajectorySample(sample.timeFromStartSeconds, sample.pose, sample.twist));
      previousTime = sample.timeFromStartSeconds;
    }
    durationSeconds = previousTime;
  }

  public function count():Int return values.length;

  public function samples():Array<TrajectorySample>
    return [for (sample in values)
      new TrajectorySample(sample.timeFromStartSeconds, sample.pose, sample.twist)];

  /** Linearly interpolates pose and body velocity, clamping outside the time span. */
  public function sampleAt(timeFromStartSeconds:Float):TrajectorySample {
    if (!Math.isFinite(timeFromStartSeconds)) throw "Trajectory time must be finite";
    if (timeFromStartSeconds <= values[0].timeFromStartSeconds)
      return copy(values[0]);
    var last = values[values.length - 1];
    if (timeFromStartSeconds >= last.timeFromStartSeconds) return copy(last);
    for (index in 0...values.length - 1) {
      var from = values[index];
      var to = values[index + 1];
      if (timeFromStartSeconds > to.timeFromStartSeconds) continue;
      var alpha = (timeFromStartSeconds - from.timeFromStartSeconds) /
        (to.timeFromStartSeconds - from.timeFromStartSeconds);
      var yawDelta = Pose2.wrapAngle(to.pose.yaw - from.pose.yaw);
      return new TrajectorySample(timeFromStartSeconds,
        new Pose2(from.pose.x + (to.pose.x - from.pose.x) * alpha,
          from.pose.y + (to.pose.y - from.pose.y) * alpha,
          from.pose.yaw + yawDelta * alpha),
        new Twist2(from.twist.linear + (to.twist.linear - from.twist.linear) * alpha,
          from.twist.angular + (to.twist.angular - from.twist.angular) * alpha));
    }
    return copy(last);
  }

  static function copy(sample:TrajectorySample):TrajectorySample
    return new TrajectorySample(sample.timeFromStartSeconds, sample.pose, sample.twist);
}
