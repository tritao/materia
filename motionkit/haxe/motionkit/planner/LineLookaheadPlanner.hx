package motionkit.planner;

import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.trajectory.JointTrajectory;
import motionkit.trajectory.JointTrajectorySample;
import motionkit.trajectory.MotionLimits;

/**
 * Deterministic line-path planner with velocity lookahead at junctions.
 *
 * The generated samples contain every segment and phase boundary, so linear
 * interpolation between samples stays on the authored polyline. Blend mode
 * carries a non-zero junction speed when the requested tolerance permits it;
 * exact-stop mode inserts a full stop at every corner.
 */
class LineLookaheadPlanner {
  public final samplePeriodSeconds:Float;
  static inline var EPSILON:Float = 1e-9;
  static inline var MIN_SEGMENT_LENGTH:Float = 1e-12;

  public function new(samplePeriodSeconds:Float = 0.01) {
    if (!Math.isFinite(samplePeriodSeconds) || samplePeriodSeconds <= 0.0)
      throw "Line-lookahead sample period must be finite and positive";
    this.samplePeriodSeconds = samplePeriodSeconds;
  }

  /** Plans a connected polyline as a three-coordinate Cartesian trajectory. */
  public function planPath(path:GeometricPath, limits:MotionLimits,
      ?options:PathPlanningOptions):JointTrajectory {
    if (path == null || limits == null) throw "Line-path planning needs a path and limits";
    var chosenOptions = options == null ? new PathPlanningOptions() : options;
    if (limits.maxVelocity <= 0.0 || limits.maxAcceleration <= 0.0)
      throw "Line-path planning needs positive velocity and acceleration limits";

    var segments:Array<LineSegment> = [];
    var firstLine:Null<LineSegment> = null;
    var previousEnd:Null<PathPoint> = null;
    for (primitive in path.primitives) {
      // Haxeon keeps PathPrimitive implementations structural at runtime, so
      // use the line-specific public fields rather than a nominal type test.
      if (!Reflect.hasField(primitive, "start") || !Reflect.hasField(primitive, "end"))
        throw "Line-lookahead planning only supports line segments";
      var line:LineSegment = cast primitive;
      if (firstLine == null) firstLine = line;
      if (previousEnd != null && previousEnd.distanceTo(line.start) > 1e-8)
        throw "Line path primitives must form a connected path";
      previousEnd = line.end;
      if (line.length() > MIN_SEGMENT_LENGTH) segments.push(line);
    }
    if (firstLine == null) throw "Line-lookahead planning needs at least one line";
    if (segments.length == 0)
      return new JointTrajectory([new JointTrajectorySample(0.0,
        [firstLine.start.x, firstLine.start.y, firstLine.start.z])]);

    var lengths:Array<Float> = [];
    var directions:Array<Array<Float>> = [];
    for (segment in segments) {
      var length = segment.length();
      var dx = segment.end.x - segment.start.x;
      var dy = segment.end.y - segment.start.y;
      var dz = segment.end.z - segment.start.z;
      lengths.push(length);
      directions.push([dx / length, dy / length, dz / length]);
    }

    var boundarySpeeds:Array<Float> = [for (_ in 0...(segments.length + 1)) 0.0];
    for (i in 1...segments.length) {
      boundarySpeeds[i] = chosenOptions.exactStop || chosenOptions.blendTolerance <= 0.0
        ? 0.0
        : cornerSpeed(directions[i - 1], directions[i], chosenOptions.blendTolerance,
          limits.maxVelocity, limits.maxAcceleration);
    }

    // Forward and backward passes project the requested corner speeds through
    // the acceleration limit across the whole path.
    for (i in 0...segments.length) {
      var reachable = Math.sqrt(boundarySpeeds[i] * boundarySpeeds[i] +
        2.0 * limits.maxAcceleration * lengths[i]);
      boundarySpeeds[i + 1] = Math.min(boundarySpeeds[i + 1],
        Math.min(limits.maxVelocity, reachable));
    }
    for (offset in 0...segments.length) {
      var i = segments.length - 1 - offset;
      var reachable = Math.sqrt(boundarySpeeds[i + 1] * boundarySpeeds[i + 1] +
        2.0 * limits.maxAcceleration * lengths[i]);
      boundarySpeeds[i] = Math.min(boundarySpeeds[i],
        Math.min(limits.maxVelocity, reachable));
    }

    var profiles:Array<LineProfile> = [];
    for (i in 0...segments.length) {
      profiles.push(new LineProfile(segments[i], directions[i], lengths[i],
        boundarySpeeds[i], boundarySpeeds[i + 1], limits.maxVelocity,
        limits.maxAcceleration));
    }

    var segmentStartTimes:Array<Float> = [];
    var totalDuration = 0.0;
    for (profile in profiles) {
      segmentStartTimes.push(totalDuration);
      profile.startTime = totalDuration;
      totalDuration += profile.duration;
    }

    var times:Array<Float> = [0.0, totalDuration];
    var regularCount = Std.int(Math.ceil(totalDuration / samplePeriodSeconds));
    for (i in 0...(regularCount + 1))
      times.push(Math.min(totalDuration, i * samplePeriodSeconds));
    for (profile in profiles) {
      times.push(profile.startTime);
      times.push(profile.startTime + profile.accelerationTime);
      times.push(profile.startTime + profile.accelerationTime + profile.cruiseTime);
      times.push(profile.startTime + profile.duration);
    }
    times.sort(function(a:Float, b:Float):Int return a < b ? -1 : a > b ? 1 : 0);

    var uniqueTimes:Array<Float> = [];
    for (time in times) {
      if (uniqueTimes.length == 0 || Math.abs(time - uniqueTimes[uniqueTimes.length - 1]) > EPSILON)
        uniqueTimes.push(time);
    }

    var samples:Array<JointTrajectorySample> = [];
    for (time in uniqueTimes)
      samples.push(sampleAt(profiles, segmentStartTimes, totalDuration, time));
    return new JointTrajectory(samples);
  }

  static function cornerSpeed(incoming:Array<Float>, outgoing:Array<Float>, tolerance:Float,
      maxVelocity:Float, maxAcceleration:Float):Float {
    var dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1] + incoming[2] * outgoing[2];
    dot = Math.max(-1.0, Math.min(1.0, dot));
    var sineHalfAngle = Math.sqrt(Math.max(0.0, (1.0 - dot) * 0.5));
    if (sineHalfAngle <= EPSILON) return maxVelocity;
    if (sineHalfAngle >= 1.0 - EPSILON) return 0.0;
    var radius = tolerance * sineHalfAngle / (1.0 - sineHalfAngle);
    return Math.min(maxVelocity, Math.sqrt(maxAcceleration * radius));
  }

  static function sampleAt(profiles:Array<LineProfile>, segmentStartTimes:Array<Float>,
      totalDuration:Float, time:Float):JointTrajectorySample {
    if (time >= totalDuration - EPSILON) {
      var last = profiles[profiles.length - 1];
      return new JointTrajectorySample(totalDuration,
        [last.segment.end.x, last.segment.end.y, last.segment.end.z]);
    }

    var index = 0;
    while (index + 1 < profiles.length &&
        time >= segmentStartTimes[index] + profiles[index].duration - EPSILON)
      index++;
    var profile = profiles[index];
    var state = profile.stateAt(Math.max(0.0, time - profile.startTime));
    var point = profile.segment.pointAt(Math.min(profile.length, state.distance));
    var velocities = [profile.direction[0] * state.velocity,
      profile.direction[1] * state.velocity, profile.direction[2] * state.velocity];
    var accelerations = [profile.direction[0] * state.acceleration,
      profile.direction[1] * state.acceleration, profile.direction[2] * state.acceleration];
    return new JointTrajectorySample(time, [point.x, point.y, point.z], velocities,
      accelerations);
  }
}

private class LineProfile {
  public final segment:LineSegment;
  public final direction:Array<Float>;
  public final length:Float;
  public final startSpeed:Float;
  public final endSpeed:Float;
  public final peakSpeed:Float;
  public final acceleration:Float;
  public final accelerationTime:Float;
  public final cruiseTime:Float;
  public final decelerationTime:Float;
  public final duration:Float;
  public var startTime:Float = 0.0;

  public function new(segment:LineSegment, direction:Array<Float>, length:Float,
      startSpeed:Float, endSpeed:Float, maxVelocity:Float, acceleration:Float) {
    this.segment = segment;
    this.direction = direction;
    this.length = length;
    this.startSpeed = startSpeed;
    this.endSpeed = endSpeed;
    this.acceleration = acceleration;
    peakSpeed = Math.min(maxVelocity,
      Math.sqrt(Math.max(0.0, (2.0 * acceleration * length +
        startSpeed * startSpeed + endSpeed * endSpeed) * 0.5)));
    accelerationTime = Math.max(0.0, (peakSpeed - startSpeed) / acceleration);
    decelerationTime = Math.max(0.0, (peakSpeed - endSpeed) / acceleration);
    var accelerationDistance = 0.5 * (startSpeed + peakSpeed) * accelerationTime;
    var decelerationDistance = 0.5 * (endSpeed + peakSpeed) * decelerationTime;
    var cruiseDistance = Math.max(0.0, length - accelerationDistance - decelerationDistance);
    cruiseTime = peakSpeed <= 0.0 ? 0.0 : cruiseDistance / peakSpeed;
    duration = accelerationTime + cruiseTime + decelerationTime;
  }

  public function stateAt(time:Float):LineProfileState {
    var t = Math.max(0.0, Math.min(duration, time));
    if (t <= accelerationTime + LineLookaheadPlanner.EPSILON) {
      var distance = startSpeed * t + 0.5 * acceleration * t * t;
      return new LineProfileState(distance, startSpeed + acceleration * t, acceleration);
    }
    var accelerationDistance = 0.5 * (startSpeed + peakSpeed) * accelerationTime;
    if (t <= accelerationTime + cruiseTime + LineLookaheadPlanner.EPSILON) {
      var cruiseElapsed = t - accelerationTime;
      return new LineProfileState(accelerationDistance + peakSpeed * cruiseElapsed,
        peakSpeed, 0.0);
    }
    var decelerationElapsed = t - accelerationTime - cruiseTime;
    var cruiseDistance = peakSpeed * cruiseTime;
    var distance = accelerationDistance + cruiseDistance + peakSpeed * decelerationElapsed -
      0.5 * acceleration * decelerationElapsed * decelerationElapsed;
    return new LineProfileState(distance, peakSpeed - acceleration * decelerationElapsed,
      -acceleration);
  }
}

private class LineProfileState {
  public final distance:Float;
  public final velocity:Float;
  public final acceleration:Float;

  public function new(distance:Float, velocity:Float, acceleration:Float) {
    this.distance = distance;
    this.velocity = velocity;
    this.acceleration = acceleration;
  }
}
