package motionkit.planner;

import haxe.Int64;
import motionkit.trajectory.Trajectory;

/** Deterministic reference timer using conservative trapezoids in path distance. */
class SimplePathTiming implements PathTimingBackend {
  public final samplePeriodSeconds:Float;
  static inline var EPSILON:Float = 1e-12;
  static inline var LARGE_LIMIT:Float = 1e100;

  public function new(?samplePeriodSeconds:Float = 0.01) {
    if (!Math.isFinite(samplePeriodSeconds) || samplePeriodSeconds <= 0.0)
      throw "Simple path-timing sample period must be finite and positive";
    this.samplePeriodSeconds = samplePeriodSeconds;
  }

  public function time(path:JointPathSamples, limits:PathTimingLimits):TimedPath {
    if (path == null || limits == null) throw "Path timing requires a path and limits";
    if (limits.maxVelocity.length != path.jointCount)
      throw "Path timing joint-limit count does not match the path";
    var spanCount = path.s.length - 1;
    if (limits.speedCaps.length != 0 && limits.speedCaps.length != spanCount)
      throw "Path timing needs one speed cap per path span";

    var bindings:Array<BindingConstraint> = [];
    var spans:Array<SimpleTimingSpan> = [];
    for (index in 0...spanCount)
      spans.push(buildSpan(path, limits, index, bindings));

    if (limits.startPathSpeed > spans[0].speedLimit + EPSILON ||
        limits.endPathSpeed > spans[spanCount - 1].speedLimit + EPSILON)
      throw "Path timing endpoint speed exceeds its span limit";
    var speeds:Array<Float> = [for (_ in 0...(spanCount + 1)) LARGE_LIMIT];
    speeds[0] = limits.startPathSpeed;
    speeds[spanCount] = limits.endPathSpeed;
    for (boundary in 1...spanCount)
      speeds[boundary] = Math.min(spans[boundary - 1].speedLimit,
        spans[boundary].speedLimit);

    for (index in 0...spanCount) {
      var reachable = spans[index].reachableSpeed(speeds[index]);
      if (index + 1 == spanCount) {
        if (speeds[index + 1] > reachable + 1e-9)
          throw "Requested end path speed is unreachable under joint acceleration limits";
      } else {
        speeds[index + 1] = Math.min(speeds[index + 1], reachable);
      }
    }
    var offset = 0;
    while (offset < spanCount) {
      var index = spanCount - 1 - offset;
      var reachable = spans[index].reachableSpeed(speeds[index + 1]);
      if (index == 0) {
        if (speeds[0] > reachable + 1e-9)
          throw "Requested start path speed cannot reach the end speed under joint acceleration limits";
      } else {
        speeds[index] = Math.min(speeds[index], reachable);
      }
      offset++;
    }

    var profiles:Array<SimpleTimingProfile> = [];
    var totalDuration = 0.0;
    for (index in 0...spanCount) {
      var profile = new SimpleTimingProfile(spans[index], speeds[index], speeds[index + 1]);
      profile.startTime = totalDuration;
      totalDuration += profile.duration;
      profiles.push(profile);
    }
    if (!Math.isFinite(totalDuration) || totalDuration <= 0.0)
      throw "Path timing produced an invalid duration";

    var times:Array<Float> = [0.0, totalDuration];
    var regularCount = Std.int(Math.ceil(totalDuration / samplePeriodSeconds));
    for (sample in 0...(regularCount + 1))
      times.push(Math.min(totalDuration, sample * samplePeriodSeconds));
    for (profile in profiles) {
      times.push(profile.startTime);
      times.push(profile.startTime + profile.accelerationTime);
      times.push(profile.startTime + profile.accelerationTime + profile.cruiseTime);
      times.push(profile.startTime + profile.duration);
    }
    times.sort(function(left:Float, right:Float):Int
      return left < right ? -1 : (left > right ? 1 : 0));
    var uniqueTimes:Array<Float> = [];
    for (value in times)
      if (uniqueTimes.length == 0 ||
          value - uniqueTimes[uniqueTimes.length - 1] > 1e-8)
        uniqueTimes.push(value);
    if (uniqueTimes[uniqueTimes.length - 1] != totalDuration)
      uniqueTimes.push(totalDuration);

    var positions:Array<Array<Float>> = [];
    for (value in uniqueTimes)
      positions.push(path.positionAt(distanceAtTime(profiles, value, totalDuration)));
    var trajectory = Trajectory.fromPositionSamples(uniqueTimes, positions);
    var duration = trajectory.durationSeconds();
    return new TimedPath(trajectory, function(distance:Float):Float {
      // The tolerances of the native time law: a path's length and its samples' last distance
      // are computed apart, and may differ in their last bits.
      var scale = Math.max(1.0, Math.abs(distance));
      if (!Math.isFinite(distance) || distance < path.start() - 1e-12 * scale ||
          distance > path.end() + 1e-8 * scale)
        throw "Path distance is outside the timed path";
      if (distance <= path.start()) return 0.0;
      if (distance >= path.end()) return duration;
      for (profile in profiles)
        if (distance <= profile.span.endS)
          return roundedSeconds(profile.startTime +
            profile.timeAtDistance(distance - profile.span.startS));
      return duration;
    }, bindings);
  }

  static function buildSpan(path:JointPathSamples, limits:PathTimingLimits,
      index:Int, bindings:Array<BindingConstraint>):SimpleTimingSpan {
    var speedLimit = limits.speedCaps.length == 0 || limits.speedCaps[index] == 0.0
      ? LARGE_LIMIT : limits.speedCaps[index];
    var bindingJoint = -1;
    var bindingKind = BindingConstraintKind.SpeedCap;
    var qPrime:Array<Float> = [];
    var qDoublePrime:Array<Float> = [];
    for (joint in 0...path.jointCount) {
      var first = Math.max(Math.abs(path.qPrime[index][joint]),
        Math.abs(path.qPrime[index + 1][joint]));
      var second = Math.max(Math.abs(path.qDoublePrime[index][joint]),
        Math.abs(path.qDoublePrimeBefore[index + 1][joint]));
      qPrime.push(first);
      qDoublePrime.push(second);
      if (first > EPSILON) {
        var velocityBound = limits.maxVelocity[joint] / first;
        if (velocityBound < speedLimit) {
          speedLimit = velocityBound;
          bindingJoint = joint;
          bindingKind = BindingConstraintKind.JointVelocity;
        }
      }
      if (second > EPSILON) {
        var accelerationBound = Math.sqrt(limits.maxAcceleration[joint] / second);
        if (accelerationBound < speedLimit) {
          speedLimit = accelerationBound;
          bindingJoint = joint;
          bindingKind = BindingConstraintKind.JointAcceleration;
        }
      }
    }
    if (!Math.isFinite(speedLimit) || speedLimit >= LARGE_LIMIT)
      throw 'Path timing span $index needs motion derivatives or an authored speed cap';
    bindings.push(new BindingConstraint(index, bindingJoint, bindingKind, speedLimit));
    return new SimpleTimingSpan(index, path.s[index], path.s[index + 1], speedLimit,
      qPrime, qDoublePrime, limits.maxAcceleration);
  }

  static function distanceAtTime(profiles:Array<SimpleTimingProfile>, time:Float,
      totalDuration:Float):Float {
    if (time >= totalDuration) return profiles[profiles.length - 1].span.endS;
    for (profile in profiles)
      if (time <= profile.startTime + profile.duration)
        return profile.span.startS + profile.distanceAt(time - profile.startTime);
    return profiles[profiles.length - 1].span.endS;
  }

  static function roundedSeconds(seconds:Float):Float
    return Int64.toFloat(Trajectory.nanoseconds(seconds)) * 1e-9;
}

private class SimpleTimingSpan {
  public final index:Int;
  public final startS:Float;
  public final endS:Float;
  public final length:Float;
  public final speedLimit:Float;
  public final qPrime:Array<Float>;
  public final qDoublePrime:Array<Float>;
  public final maxAcceleration:Array<Float>;

  public function new(index:Int, startS:Float, endS:Float, speedLimit:Float,
      qPrime:Array<Float>, qDoublePrime:Array<Float>, maxAcceleration:Array<Float>) {
    this.index = index;
    this.startS = startS;
    this.endS = endS;
    this.length = endS - startS;
    this.speedLimit = speedLimit;
    this.qPrime = qPrime;
    this.qDoublePrime = qDoublePrime;
    this.maxAcceleration = maxAcceleration;
  }

  /**
    The fastest speed this span reaches from `knownSpeed` at either end,
    accelerating within the budget left at the speed it reaches, the budget
    the span's profile then uses. Per joint, v² = v0² + 2 L a(v) with
    a(v) = (A - q'' v²) / (q' + 2 q'' L) solves in closed form.
  **/
  public function reachableSpeed(knownSpeed:Float):Float {
    if (accelerationFor(knownSpeed) >= SimplePathTiming.LARGE_LIMIT)
      return speedLimit;
    var known = knownSpeed * knownSpeed, reachable = speedLimit * speedLimit;
    for (joint in 0...qPrime.length) {
      var denominator = qPrime[joint] + 2.0 * qDoublePrime[joint] * length;
      if (denominator <= SimplePathTiming.EPSILON) continue;
      var squared = (known * denominator + 2.0 * length * maxAcceleration[joint]) /
        (denominator + 2.0 * length * qDoublePrime[joint]);
      reachable = Math.min(reachable, Math.max(known, squared));
    }
    return Math.sqrt(reachable);
  }

  public function accelerationFor(referenceSpeed:Float):Float {
    var result = SimplePathTiming.LARGE_LIMIT;
    for (joint in 0...qPrime.length) {
      var numerator = maxAcceleration[joint] -
        qDoublePrime[joint] * referenceSpeed * referenceSpeed;
      var denominator = qPrime[joint] + 2.0 * qDoublePrime[joint] * length;
      if (denominator > SimplePathTiming.EPSILON)
        result = Math.min(result, Math.max(0.0, numerator) / denominator);
    }
    return result;
  }
}

private class SimpleTimingProfile {
  public final span:SimpleTimingSpan;
  public final startSpeed:Float;
  public final endSpeed:Float;
  public final peakSpeed:Float;
  public final acceleration:Float;
  public final accelerationTime:Float;
  public final cruiseTime:Float;
  public final decelerationTime:Float;
  public final accelerationDistance:Float;
  public final cruiseDistance:Float;
  public final duration:Float;
  public var startTime:Float = 0.0;

  public function new(span:SimpleTimingSpan, startSpeed:Float, endSpeed:Float) {
    this.span = span;
    this.startSpeed = startSpeed;
    this.endSpeed = endSpeed;
    var reference = Math.max(startSpeed, endSpeed);
    var chosenAcceleration = span.accelerationFor(reference);
    if (chosenAcceleration >= SimplePathTiming.LARGE_LIMIT)
      chosenAcceleration = Math.max(span.speedLimit, 1.0);
    if (chosenAcceleration <= SimplePathTiming.EPSILON) {
      if (Math.abs(startSpeed - endSpeed) > 1e-9 || startSpeed <= 0.0)
        throw 'Path timing span ${span.index} has no acceleration budget';
      acceleration = 0.0;
      peakSpeed = startSpeed;
      accelerationTime = 0.0;
      decelerationTime = 0.0;
      accelerationDistance = 0.0;
      cruiseDistance = span.length;
      cruiseTime = span.length / startSpeed;
      duration = cruiseTime;
      return;
    }
    acceleration = chosenAcceleration;
    var accelToLimit = (span.speedLimit * span.speedLimit - startSpeed * startSpeed) /
      (2.0 * acceleration);
    var decelFromLimit = (span.speedLimit * span.speedLimit - endSpeed * endSpeed) /
      (2.0 * acceleration);
    var chosenPeak = span.speedLimit;
    if (accelToLimit + decelFromLimit > span.length)
      // Reachability leaves the faster end speed attainable; this keeps rounding from losing it.
      chosenPeak = Math.max(Math.max(startSpeed, endSpeed), Math.sqrt(Math.max(0.0,
        (2.0 * acceleration * span.length + startSpeed * startSpeed +
          endSpeed * endSpeed) * 0.5)));
    peakSpeed = chosenPeak;
    accelerationTime = Math.max(0.0, (peakSpeed - startSpeed) / acceleration);
    decelerationTime = Math.max(0.0, (peakSpeed - endSpeed) / acceleration);
    accelerationDistance = (startSpeed + peakSpeed) * accelerationTime * 0.5;
    var decelerationDistance = (endSpeed + peakSpeed) * decelerationTime * 0.5;
    cruiseDistance = Math.max(0.0, span.length - accelerationDistance - decelerationDistance);
    cruiseTime = cruiseDistance <= SimplePathTiming.EPSILON ? 0.0 : cruiseDistance / peakSpeed;
    duration = accelerationTime + cruiseTime + decelerationTime;
  }

  public function distanceAt(time:Float):Float {
    if (time <= 0.0) return 0.0;
    if (time < accelerationTime)
      return startSpeed * time + 0.5 * acceleration * time * time;
    if (time < accelerationTime + cruiseTime)
      return accelerationDistance + peakSpeed * (time - accelerationTime);
    if (time >= duration) return span.length;
    var decelerationTimeNow = time - accelerationTime - cruiseTime;
    return accelerationDistance + cruiseDistance + peakSpeed * decelerationTimeNow -
      0.5 * acceleration * decelerationTimeNow * decelerationTimeNow;
  }

  public function timeAtDistance(distance:Float):Float {
    if (distance <= 0.0) return 0.0;
    if (distance >= span.length) return duration;
    if (acceleration <= SimplePathTiming.EPSILON) return distance / startSpeed;
    if (distance < accelerationDistance) {
      return (-startSpeed + Math.sqrt(startSpeed * startSpeed +
        2.0 * acceleration * distance)) / acceleration;
    }
    if (distance < accelerationDistance + cruiseDistance)
      return accelerationTime + (distance - accelerationDistance) / peakSpeed;
    var intoDeceleration = distance - accelerationDistance - cruiseDistance;
    return accelerationTime + cruiseTime +
      (peakSpeed - Math.sqrt(Math.max(0.0, peakSpeed * peakSpeed -
        2.0 * acceleration * intoDeceleration))) / acceleration;
  }
}
