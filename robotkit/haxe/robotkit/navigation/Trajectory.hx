package robotkit.navigation;

import robotkit.mobile.Pose2;
import robotkit.mobile.MobileBase;
import robotkit.mobile.PlanarMath;
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

  /**
   * Time-parameterizes a geometric path using the base's drive and motion limits.
   * Path pose yaw defines body heading between waypoints; reverse segments are
   * selected when that heading faces away from the segment tangent.
   */
  public static function fromPath(path:Path, base:MobileBase,
      ?maxSpeed:Float = 1.0e300, ?maxLateralAcceleration:Float = 1.0,
      ?sampleSpacing:Float = 0.1, ?allowReverse:Bool = true):Trajectory {
    if (path == null || base == null || !Math.isFinite(maxSpeed) || maxSpeed <= 0.0 ||
        !Math.isFinite(maxLateralAcceleration) || maxLateralAcceleration <= 0.0 ||
        !Math.isFinite(sampleSpacing) || sampleSpacing <= 0.0)
      throw "Trajectory parameterization requires a path, base, and positive finite limits";

    var poses:Array<Pose2> = [];
    var distances:Array<Float> = [];
    var tangents:Array<Float> = [];
    var pathPoses = path.poses();
    var distance = 0.0;
    for (segment in 0...pathPoses.length - 1) {
      var from = pathPoses[segment];
      var to = pathPoses[segment + 1];
      var dx = to.x - from.x;
      var dy = to.y - from.y;
      var length = Math.pow(dx * dx + dy * dy, 0.5);
      if (length <= 1e-9) {
        if (Math.abs(Pose2.wrapAngle(to.yaw - from.yaw)) > 1e-6)
          throw "Path contains an in-place rotation; provide explicit trajectory samples";
        continue;
      }
      var tangent = PlanarMath.atan2(dy, dx);
      if (poses.length == 0) {
        poses.push(from);
        distances.push(distance);
        tangents.push(tangent);
      } else {
        // A corner belongs to its outgoing segment for direction selection.
        // This lets a stop at a forward/reverse cusp occur at the waypoint.
        tangents[tangents.length - 1] = tangent;
      }
      var subdivisions = Std.int(Math.ceil(length / sampleSpacing));
      // Endpoints can both be zero-speed (route endpoints or a gear-change
      // cusp). An interior sample is needed to accelerate and decelerate over
      // short segments instead of producing an untraversable zero-speed edge.
      if (subdivisions < 2) subdivisions = 2;
      for (part in 1...subdivisions + 1) {
        var alongSegment = length * part / subdivisions;
        var absoluteDistance = distance + alongSegment;
        var pose = path.poseAt(absoluteDistance);
        if (absoluteDistance - distances[distances.length - 1] <= 1e-9) continue;
        poses.push(pose);
        distances.push(absoluteDistance);
        tangents.push(tangent);
      }
      distance += length;
    }
    if (poses.length < 2 || distances[distances.length - 1] <= 1e-9)
      throw "Trajectory parameterization requires a non-degenerate path";

    // Preserve exact waypoint corners in the samples and accumulated arc length.
    // Segment resampling above ends each segment at its endpoint.
    var unwrappedYaw:Array<Float> = [poses[0].yaw];
    for (index in 1...poses.length) {
      unwrappedYaw.push(unwrappedYaw[index - 1] +
        Pose2.wrapAngle(poses[index].yaw - poses[index - 1].yaw));
    }
    var curvature:Array<Float> = [];
    var direction:Array<Float> = [];
    var speedCaps:Array<Float> = [];
    var maxCurvature = base.driveModel.maxCurvature();
    var configuredMaxSpeed = Math.min(maxSpeed, base.motionLimits.maxLinearSpeed);
    for (index in 0...poses.length) {
      var left = index == 0 ? 0 : index - 1;
      var right = index == poses.length - 1 ? poses.length - 1 : index + 1;
      var span = distances[right] - distances[left];
      var yawSlope = span <= 1e-9 ? 0.0 :
        (unwrappedYaw[right] - unwrappedYaw[left]) / span;
      curvature.push(yawSlope);
      var alignment = Math.cos(Pose2.wrapAngle(poses[index].yaw - tangents[index]));
      var sign = alignment < 0.0 ? -1.0 : 1.0;
      if (sign < 0.0 && !allowReverse)
        throw "Trajectory path requires reverse motion, but reverse is disabled";
      direction.push(sign);
      if (Math.abs(yawSlope) > maxCurvature + 1e-8)
        throw "Path curvature exceeds the drive model's steering limit";
      var cap = configuredMaxSpeed;
      if (Math.abs(yawSlope) > 1e-9) {
        cap = Math.min(cap, base.motionLimits.maxAngularSpeed / Math.abs(yawSlope));
        cap = Math.min(cap, Math.pow(maxLateralAcceleration / Math.abs(yawSlope), 0.5));
      }
      speedCaps.push(cap);
    }
    speedCaps[0] = 0.0;
    speedCaps[speedCaps.length - 1] = 0.0;
    for (index in 1...direction.length) {
      if (direction[index] != direction[index - 1]) {
        speedCaps[index] = 0.0;
      }
    }

    var speeds = profileSpeeds(speedCaps, distances,
      base.motionLimits.maxLinearAcceleration);
    var angularAcceleration = base.motionLimits.maxAngularAcceleration;
    for (_ in 0...32) {
      var adjusted = false;
      for (index in 0...poses.length - 1) {
        var ds = distances[index + 1] - distances[index];
        var sum = speeds[index] + speeds[index + 1];
        if (sum <= 1e-9)
          throw "Trajectory cannot move between adjacent zero-speed constraints";
        var interval = 2.0 * ds / sum;
        var omegaFrom = curvature[index] * speeds[index];
        var omegaTo = curvature[index + 1] * speeds[index + 1];
        var acceleration = Math.abs(omegaTo - omegaFrom) / interval;
        if (acceleration > angularAcceleration * 1.0001) {
          var scale = Math.pow(angularAcceleration / acceleration, 0.5) * 0.98;
          speedCaps[index] = Math.min(speedCaps[index], speeds[index] * scale);
          speedCaps[index + 1] = Math.min(speedCaps[index + 1], speeds[index + 1] * scale);
          adjusted = true;
        }
      }
      if (!adjusted) break;
      speeds = profileSpeeds(speedCaps, distances,
        base.motionLimits.maxLinearAcceleration);
    }
    for (index in 0...poses.length - 1) {
      var ds = distances[index + 1] - distances[index];
      var sum = speeds[index] + speeds[index + 1];
      if (sum <= 1e-9) throw "Trajectory has an untraversable zero-speed interval";
      var interval = 2.0 * ds / sum;
      var angularAccelerationUsed = Math.abs(
        curvature[index + 1] * speeds[index + 1] - curvature[index] * speeds[index]) / interval;
      if (angularAccelerationUsed > angularAcceleration * 1.01)
        throw "Trajectory curvature cannot satisfy the base's angular acceleration limit";
    }

    var samples:Array<TrajectorySample> = [];
    var time = 0.0;
    for (index in 0...poses.length) {
      samples.push(new TrajectorySample(time, poses[index],
        new Twist2(direction[index] * speeds[index], curvature[index] * speeds[index])));
      if (index < poses.length - 1) {
        var sum = speeds[index] + speeds[index + 1];
        time += 2.0 * (distances[index + 1] - distances[index]) / sum;
      }
    }
    return new Trajectory(samples, path.frameId);
  }

  static function profileSpeeds(caps:Array<Float>, distances:Array<Float>,
      maxAcceleration:Float):Array<Float> {
    var speeds = caps.copy();
    for (index in 1...speeds.length) {
      var ds = distances[index] - distances[index - 1];
      speeds[index] = Math.min(speeds[index],
        Math.pow(speeds[index - 1] * speeds[index - 1] + 2.0 * maxAcceleration * ds, 0.5));
    }
    var index = speeds.length - 2;
    while (index >= 0) {
      var ds = distances[index + 1] - distances[index];
      speeds[index] = Math.min(speeds[index],
        Math.pow(speeds[index + 1] * speeds[index + 1] + 2.0 * maxAcceleration * ds, 0.5));
      index--;
    }
    return speeds;
  }

  public function count():Int return values.length;

  public function start():Pose2 {
    var pose = values[0].pose;
    return new Pose2(pose.x, pose.y, pose.yaw);
  }

  public function goal():Pose2 {
    var pose = values[values.length - 1].pose;
    return new Pose2(pose.x, pose.y, pose.yaw);
  }

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
          from.twist.angular + (to.twist.angular - from.twist.angular) * alpha,
          from.twist.lateral + (to.twist.lateral - from.twist.lateral) * alpha));
    }
    return copy(last);
  }

  static function copy(sample:TrajectorySample):TrajectorySample
    return new TrajectorySample(sample.timeFromStartSeconds, sample.pose, sample.twist);
}
