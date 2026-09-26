package motionkit.planner;

import motionkit.trajectory.JointTrajectory;
import motionkit.trajectory.JointTrajectorySample;
import motionkit.trajectory.MotionLimits;

/**
 * Deterministic synchronized planner for the bootstrap motion boundary.
 *
 * Each joint uses a symmetric triangular or trapezoidal velocity profile and
 * all joints are stretched to the slowest required duration. This obeys
 * velocity and acceleration limits and leaves jerk in the API for the later
 * S-curve planner. Samples are generated on a fixed period so simulation and
 * a deployment adapter consume identical values.
 */
class TrapezoidalPlanner implements TrajectoryPlanner {
  public final samplePeriodSeconds:Float;

  public function new(?samplePeriodSeconds:Float = 0.01) {
    if (!Math.isFinite(samplePeriodSeconds) || samplePeriodSeconds <= 0.0)
      throw "Trajectory sample period must be finite and positive";
    this.samplePeriodSeconds = samplePeriodSeconds;
  }

  public function plan(startPositions:Array<Float>, goalPositions:Array<Float>,
      limits:MotionLimits):JointTrajectory {
    if (startPositions == null || goalPositions == null ||
        startPositions.length == 0 || startPositions.length != goalPositions.length)
      throw "Trajectory start and goal vectors must have the same non-zero length";
    if (limits == null) throw "Trajectory limits are required";

    var duration = 0.0;
    for (i in 0...startPositions.length) {
      requireFinite(startPositions[i], "start position");
      requireFinite(goalPositions[i], "goal position");
      var distance = Math.abs(goalPositions[i] - startPositions[i]);
      duration = Math.max(duration, minimumDuration(distance, limits));
    }

    if (duration <= 0.0)
      return new JointTrajectory([new JointTrajectorySample(0.0, startPositions)]);

    var count:Int = Std.int(Math.max(1.0, Math.ceil(duration / samplePeriodSeconds)));
    var samples:Array<JointTrajectorySample> = [];
    for (index in 0...(count + 1)) {
      var time = index == count ? duration : index * samplePeriodSeconds;
      var positions:Array<Float> = [];
      var velocities:Array<Float> = [];
      var accelerations:Array<Float> = [];
      for (i in 0...startPositions.length) {
        var profile = profileAt(startPositions[i], goalPositions[i], time, duration,
          limits.maxVelocity);
        positions.push(profile.position);
        velocities.push(profile.velocity);
        accelerations.push(profile.acceleration);
      }
      samples.push(new JointTrajectorySample(time, positions, velocities, accelerations));
    }
    return new JointTrajectory(samples);
  }

  static function minimumDuration(distance:Float, limits:MotionLimits):Float {
    if (distance <= 0.0) return 0.0;
    if (limits.maxVelocity <= 0.0 || limits.maxAcceleration <= 0.0)
      throw "Moving trajectory needs positive velocity and acceleration limits";
    var accelerationTime = limits.maxVelocity / limits.maxAcceleration;
    var accelerationDistance = limits.maxAcceleration * accelerationTime * accelerationTime;
    if (distance <= accelerationDistance)
      return 2.0 * Math.sqrt(distance / limits.maxAcceleration);
    return 2.0 * accelerationTime +
      (distance - accelerationDistance) / limits.maxVelocity;
  }

  static function profileAt(start:Float, goal:Float, time:Float,
      duration:Float, maxVelocity:Float):{position:Float, velocity:Float, acceleration:Float} {
    var delta = goal - start;
    var distance = Math.abs(delta);
    if (distance <= 0.0) return {position: start, velocity: 0.0, acceleration: 0.0};
    var sign = delta < 0.0 ? -1.0 : 1.0;
    // A profile's duration is always feasible because it is at least the
    // minimum duration computed from the same velocity and acceleration caps.
    var half = duration / 2.0;
    var accelerationTime = Math.min(half,
      duration - distance / Math.max(1e-12, maxVelocity));
    var velocity = distance / Math.max(1e-12, duration - accelerationTime);
    var acceleration = velocity / Math.max(1e-12, accelerationTime);
    var cruiseStart = accelerationTime;
    var cruiseEnd = duration - accelerationTime;
    var distanceAlong:Float;
    var velocityAlong:Float;
    var accelerationAlong:Float;
    if (time <= cruiseStart) {
      distanceAlong = 0.5 * acceleration * time * time;
      velocityAlong = acceleration * time;
      accelerationAlong = acceleration;
    } else if (time <= cruiseEnd) {
      var cruiseTime = time - cruiseStart;
      distanceAlong = 0.5 * acceleration * accelerationTime * accelerationTime +
        velocity * cruiseTime;
      velocityAlong = velocity;
      accelerationAlong = 0.0;
    } else {
      var afterCruise = Math.min(time - cruiseEnd, accelerationTime);
      var cruiseDistance = 0.5 * acceleration * accelerationTime * accelerationTime +
        velocity * (cruiseEnd - cruiseStart);
      distanceAlong = cruiseDistance + velocity * afterCruise -
        0.5 * acceleration * afterCruise * afterCruise;
      velocityAlong = velocity - acceleration * afterCruise;
      accelerationAlong = -acceleration;
    }
    return {
      position: start + sign * distanceAlong,
      velocity: sign * velocityAlong,
      acceleration: sign * accelerationAlong
    };
  }

  static function requireFinite(value:Float, label:String):Void {
    if (!Math.isFinite(value)) throw 'Trajectory $label must be finite';
  }
}
