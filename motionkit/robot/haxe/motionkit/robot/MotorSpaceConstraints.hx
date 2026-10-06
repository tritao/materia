package motionkit.robot;

import haxe.Int64;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingBackend;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.TimedPath;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ValidationLimits;
import robotkit.model.RobotModel;

/** Linear velocity limits in motor coordinates, including shared CoreXY drives. */
class MotorSpaceConstraints {
  public final rows:Array<Array<Float>>;
  public final rates:Array<Float>;

  public function new(rows:Array<Array<Float>>, rates:Array<Float>) {
    if (rows == null || rates == null || rows.length != rates.length)
      throw "Motor constraints need matching rows and rates";
    this.rows = [for (row in rows) row.copy()];
    this.rates = rates.copy();
    for (i in 0...rows.length) {
      if (rows[i].length == 0 || rows[i].length != rows[0].length)
        throw "Motor constraint rows must have matching joint counts";
      for (value in rows[i]) if (!Math.isFinite(value)) throw "Motor constraint coefficients must be finite";
      if (!Math.isFinite(rates[i]) || rates[i] <= 0.0) throw "Motor constraint rates must be finite and positive";
    }
  }

  /** A planned subset of independent axes; shared followers are derived from their coupling terms. */
  public static function of(model:RobotModel, jointIds:Array<String>, ?rateScale:Float = 1.0):MotorSpaceConstraints {
    if (!Math.isFinite(rateScale) || rateScale <= 0.0 || rateScale > 1.0)
      throw "Motor rate scale must be in (0, 1]";
    function weights(id:String, depth:Int):Array<Float> {
      if (depth > model.joints.length) throw "Cyclic motor coupling";
      var result = [for (_ in jointIds) 0.0];
      var coupled = false;
      for (term in model.couplings) if (term.follower == id) coupled = true;
      var index = jointIds.indexOf(id);
      if (!coupled && index >= 0) { result[index] = 1.0; return result; }
      for (term in model.couplings) if (term.follower == id) {
        var source = weights(term.leader, depth + 1);
        for (i in 0...result.length) result[i] += term.ratio * source[i];
      }
      return result;
    }
    var rows:Array<Array<Float>> = [], rates:Array<Float> = [];
    function add(row:Array<Float>, rate:Null<Float>):Void {
      if (rate == null || rate <= 0.0) return;
      var nonzero = 0;
      for (value in row) if (Math.abs(value) > 1e-12) nonzero++;
      if (nonzero < 2) return;
      rows.push(row); rates.push(rate * rateScale);
    }
    for (joint in model.joints) {
      var mechanical = joint.mechanicalLimits == null ? joint.limits : joint.mechanicalLimits;
      add(weights(joint.id, 0), mechanical.velocity);
    }
    for (actuator in model.actuators) switch actuator.transmission {
      case SimpleTransmission(joint, ratio, _):
        add([for (value in weights(joint, 0)) value * ratio], actuator.planningRate());
    }
    return new MotorSpaceConstraints(rows, rates);
  }

  function dot(row:Array<Float>, values:Array<Float>):Float {
    if (row.length != values.length) throw "Motor constraints and planned joint counts differ";
    var value = 0.0;
    for (i in 0...row.length) value += row[i] * values[i];
    return value;
  }

  /** A straight joint move with one jerk-limited progress profile, so all motor sums remain bounded. */
  public function move(start:Array<Float>, goal:Array<Float>, velocity:Array<Float>,
      acceleration:Array<Float>, jerk:Array<Float>):Trajectory {
    var delta = [for (i in 0...start.length) goal[i] - start[i]];
    var moving = false;
    for (value in delta) if (Math.abs(value) > 1e-12) moving = true;
    if (!moving) return Trajectory.fromPositionSamples([0.0, 0.01], [start, start]);
    var speed = 1e12, accel = 1e12, jolt = 1e12;
    for (i in 0...delta.length) if (Math.abs(delta[i]) > 1e-12) {
      speed = Math.min(speed, velocity[i] / Math.abs(delta[i]));
      accel = Math.min(accel, acceleration[i] / Math.abs(delta[i]));
      jolt = Math.min(jolt, jerk[i] / Math.abs(delta[i]));
    }
    for (i in 0...rows.length) {
      var scale = Math.abs(dot(rows[i], delta));
      if (scale > 1e-12) speed = Math.min(speed, rates[i] / scale);
    }
    var progress = Trajectory.generateStateToState([0.0], [0.0], [0.0], [1.0], [speed], [accel], [jolt]);
    try {
      var segments = [for (segment in progress.segments()) {
        timeFromStartNs: segment.timeFromStartNs, durationNs: segment.durationNs,
        coefficients: [for (i in 0...start.length) [for (p in 0...segment.coefficients[0].length)
          (p == 0 ? start[i] : 0.0) + delta[i] * segment.coefficients[0][p]]]
      }];
      var result = Trajectory.fromSegments(segments);
      progress.dispose();
      return result;
    } catch (error:Dynamic) { progress.dispose(); throw error; }
  }

  /** Time the axes and linear motor coordinates together; return only the authored joints. */
  public function time(backend:PathTimingBackend, path:JointPathSamples, limits:PathTimingLimits):TimedPath {
    if (rows.length == 0) return backend.time(path, limits);
    function augment(samples:Array<Array<Float>>):Array<Array<Float>>
      return [for (sample in samples) sample.concat([for (row in rows) dot(row, sample)])];
    var timed = backend.time(new JointPathSamples(path.s, augment(path.q), augment(path.qPrime),
      augment(path.qDoublePrime), augment(path.qDoublePrimeBefore)),
      new PathTimingLimits(limits.maxVelocity.concat(rates),
        limits.maxAcceleration.concat([for (_ in rows) 1e12]), limits.speedCaps,
        limits.startPathSpeed, limits.endPathSpeed,limits.requireFeasiblePath));
    try {
      var trajectory = Trajectory.fromSegments([for (segment in timed.trajectory.segments()) {
        timeFromStartNs: segment.timeFromStartNs, durationNs: segment.durationNs,
        coefficients: segment.coefficients.slice(0, path.jointCount)
      }]);
      timed.trajectory.dispose();
      return new TimedPath(trajectory, distance -> timed.distanceToTime(distance), timed.bindingConstraints,
        () -> timed.releaseDistanceMap(), timed.hasDirectInverse() ? seconds -> timed.timesToDistances(seconds) : null);
    } catch (error:Dynamic) { timed.trajectory.dispose(); timed.releaseDistanceMap(); throw error; }
  }

  /** Check polynomial extrema after lowering, including between path timing samples. */
  public function validate(trajectory:Trajectory):Void {
    if (rows.length == 0) return;
    var motors = Trajectory.fromSegments([for (segment in trajectory.segments()) {
      timeFromStartNs: segment.timeFromStartNs, durationNs: segment.durationNs,
      coefficients: [for (row in rows) [for (power in 0...segment.coefficients[0].length)
        dot(row, [for (joint in segment.coefficients) joint[power]])]]
    }]);
    try {
      var limits = new ValidationLimits(rows.length, Int64.ofInt(1), Int64.ofInt(0));
      for (i in 0...rows.length) limits.velocity(i, rates[i]);
      if (motors.validate(limits).hasFailure()) throw "Motor-space velocity limit exceeded";
      motors.dispose();
    } catch (error:Dynamic) { motors.dispose(); throw error; }
  }
}
