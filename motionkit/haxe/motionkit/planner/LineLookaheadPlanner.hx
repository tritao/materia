package motionkit.planner;

import motionkit.path.GeometricPath;
import motionkit.path.PathPoint;
import motionkit.path.PathPrimitive;
import motionkit.trajectory.JointTrajectory;
import motionkit.trajectory.JointTrajectorySample;
import motionkit.trajectory.MotionLimits;

/**
 * Deterministic line/arc path planner with velocity lookahead at junctions.
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
    if (path == null || limits == null) throw "Path planning needs a path and limits";
    var chosenOptions = options == null ? new PathPlanningOptions() : options;
    if (limits.maxVelocity <= 0.0 || limits.maxAcceleration <= 0.0)
      throw "Line-path planning needs positive velocity and acceleration limits";

    var segments:Array<PathPrimitive> = [];
    var firstPrimitive:Null<PathPrimitive> = null;
    var previousEnd:Null<PathPoint> = null;
    for (primitive in path.primitives) {
      if (firstPrimitive == null) firstPrimitive = primitive;
      var length = primitive.length();
      var start = primitive.pointAt(0.0);
      var end = primitive.pointAt(length);
      if (previousEnd != null && previousEnd.distanceTo(start) > 1e-8)
        throw "Path primitives must form a connected path";
      previousEnd = end;
      if (length > MIN_SEGMENT_LENGTH) segments.push(primitive);
    }
    if (firstPrimitive == null) throw "Path planning needs at least one primitive";
    if (segments.length == 0)
      return new JointTrajectory([new JointTrajectorySample(0.0,
        [firstPrimitive.pointAt(0.0).x, firstPrimitive.pointAt(0.0).y,
          firstPrimitive.pointAt(0.0).z])]);

    var lengths:Array<Float> = [];
    var startDirections:Array<Array<Float>> = [];
    var endDirections:Array<Array<Float>> = [];
    var segmentVelocityLimits:Array<Float> = [];
    for (segment in segments) {
      var length = segment.length();
      lengths.push(length);
      startDirections.push(normalize(segment.tangentAt(0.0)));
      endDirections.push(normalize(segment.tangentAt(length)));
      var segmentMaxVelocity = limits.maxVelocity;
      for (sampleIndex in 0...65) {
        var curvature = Math.abs(segment.curvatureAt(length * sampleIndex / 64.0));
        if (curvature > EPSILON)
          segmentMaxVelocity = Math.min(segmentMaxVelocity,
            Math.sqrt(limits.maxAcceleration / curvature));
      }
      segmentVelocityLimits.push(segmentMaxVelocity);
    }

    var boundarySpeeds:Array<Float> = [for (_ in 0...(segments.length + 1)) 0.0];
    for (i in 1...segments.length) {
      boundarySpeeds[i] = chosenOptions.exactStop || chosenOptions.blendTolerance <= 0.0
        ? 0.0
        : cornerSpeed(endDirections[i - 1], startDirections[i], chosenOptions.blendTolerance,
          limits.maxVelocity, limits.maxAcceleration);
      boundarySpeeds[i] = Math.min(boundarySpeeds[i],
        Math.min(segmentVelocityLimits[i - 1], segmentVelocityLimits[i]));
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

    var profiles:Array<PathProfile> = [];
    for (i in 0...segments.length) {
      profiles.push(new PathProfile(segments[i], startDirections[i], lengths[i],
        boundarySpeeds[i], boundarySpeeds[i + 1], segmentVelocityLimits[i],
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

  static function normalize(vector:Array<Float>):Array<Float> {
    var length = Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] +
      vector[2] * vector[2]);
    if (!Math.isFinite(length) || length <= MIN_SEGMENT_LENGTH)
      throw "Path primitive tangent must be finite and non-zero";
    return [vector[0] / length, vector[1] / length, vector[2] / length];
  }

  static function cornerSpeed(incoming:Array<Float>, outgoing:Array<Float>, tolerance:Float,
      maxVelocity:Float, maxAcceleration:Float):Float {
    var dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1] + incoming[2] * outgoing[2];
    dot = Math.max(-1.0, Math.min(1.0, dot));
    // The junction-deviation construction uses the interior angle between
    // the reversed incoming tangent and the outgoing tangent. Using the
    // travel-direction dot product directly inverts gentle bends and near
    // reversals: shallow corners nearly stop while sharp ones run through.
    dot = -dot;
    var sineHalfAngle = Math.sqrt(Math.max(0.0, (1.0 - dot) * 0.5));
    if (sineHalfAngle <= EPSILON) return maxVelocity;
    if (sineHalfAngle >= 1.0 - EPSILON) return 0.0;
    var radius = tolerance * sineHalfAngle / (1.0 - sineHalfAngle);
    return Math.min(maxVelocity, Math.sqrt(maxAcceleration * radius));
  }

  static function sampleAt(profiles:Array<PathProfile>, segmentStartTimes:Array<Float>,
      totalDuration:Float, time:Float):JointTrajectorySample {
    if (time >= totalDuration - EPSILON) {
      var last = profiles[profiles.length - 1];
      return new JointTrajectorySample(totalDuration,
        [last.primitive.pointAt(last.length).x, last.primitive.pointAt(last.length).y,
          last.primitive.pointAt(last.length).z]);
    }

    var index = 0;
    while (index + 1 < profiles.length &&
        time >= segmentStartTimes[index] + profiles[index].duration - EPSILON)
      index++;
    var profile = profiles[index];
    var state = profile.stateAt(Math.max(0.0, time - profile.startTime));
    var distance = Math.min(profile.length, state.distance);
    var point = profile.primitive.pointAt(distance);
    var direction = normalize(profile.primitive.tangentAt(distance));
    var curvature = profile.primitive.curvatureAt(distance);
    var velocities = [direction[0] * state.velocity,
      direction[1] * state.velocity, direction[2] * state.velocity];
    var normalAcceleration = state.velocity * state.velocity * curvature;
    var accelerations = [direction[0] * state.acceleration,
      direction[1] * state.acceleration, direction[2] * state.acceleration];
    accelerations[0] += -direction[1] * normalAcceleration;
    accelerations[1] += direction[0] * normalAcceleration;
    return new JointTrajectorySample(time, [point.x, point.y, point.z], velocities,
      accelerations);
  }
}

private class PathProfile {
  public final primitive:PathPrimitive;
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

  public function new(primitive:PathPrimitive, direction:Array<Float>, length:Float,
      startSpeed:Float, endSpeed:Float, maxVelocity:Float, acceleration:Float) {
    this.primitive = primitive;
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

  public function stateAt(time:Float):PathProfileState {
    var t = Math.max(0.0, Math.min(duration, time));
    if (t <= accelerationTime + LineLookaheadPlanner.EPSILON) {
      var distance = startSpeed * t + 0.5 * acceleration * t * t;
      return new PathProfileState(distance, startSpeed + acceleration * t, acceleration);
    }
    var accelerationDistance = 0.5 * (startSpeed + peakSpeed) * accelerationTime;
    if (t <= accelerationTime + cruiseTime + LineLookaheadPlanner.EPSILON) {
      var cruiseElapsed = t - accelerationTime;
      return new PathProfileState(accelerationDistance + peakSpeed * cruiseElapsed,
        peakSpeed, 0.0);
    }
    var decelerationElapsed = t - accelerationTime - cruiseTime;
    var cruiseDistance = peakSpeed * cruiseTime;
    var distance = accelerationDistance + cruiseDistance + peakSpeed * decelerationElapsed -
      0.5 * acceleration * decelerationElapsed * decelerationElapsed;
    return new PathProfileState(distance, peakSpeed - acceleration * decelerationElapsed,
      -acceleration);
  }
}

private class PathProfileState {
  public final distance:Float;
  public final velocity:Float;
  public final acceleration:Float;

  public function new(distance:Float, velocity:Float, acceleration:Float) {
    this.distance = distance;
    this.velocity = velocity;
    this.acceleration = acceleration;
  }
}
