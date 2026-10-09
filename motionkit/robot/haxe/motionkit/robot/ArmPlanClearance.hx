package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.Trajectory;
import robotkit.manipulation.ArmClearance;

private enum ClearanceFailureKind {
  Collision;
  Unresolved;
  Invalid;
}

private typedef ClearanceFailure = {
  var kind:ClearanceFailureKind;
  var message:String;
}

/** Certifies polynomial motion against a cell snapshot. Interval bounds include
    every hull point's travel, so narrow obstacles cannot hide between samples.
    Unresolved intervals fail admission when the subdivision budget is exhausted. */
class ArmPlanClearance implements PlanValidator {
  final clearance:ArmClearance;

  /** Largest adaptive tree admitted while expanding a joint-space move. */
  static inline var MAX_JOINT_SEGMENT_CHECKS:Int = 4096;
  static inline var MAX_JOINT_SEGMENT_DEPTH:Int = 16;

  public function new(clearance:ArmClearance) {
    if (clearance == null) throw "Plan clearance needs its arm and cell bodies";
    this.clearance = clearance;
  }

  public function fork():PlanValidator return this;

  /**
   * Split a rest-to-rest joint move until the actual synchronized trajectory
   * for each segment passes the same polynomial clearance check used at plan
   * admission. This handles axes that take different phase profiles while
   * avoiding needless waypoints when the original trajectory is already clear.
   * Each returned endpoint is an exact-stop target; the final entry is `to`.
   */
  public function partitionJointSegment(from:Array<Float>, to:Array<Float>, maximumVelocity:Array<Float>,
      maximumAcceleration:Array<Float>, maximumJerk:Array<Float>):Array<Array<Float>> {
    var count = clearance.arm.dofCount();
    if (from == null || to == null || from.length != count || to.length != count)
      throw "Joint clearance partition needs matching group coordinates";
    for (values in [maximumVelocity, maximumAcceleration, maximumJerk])
      if (values == null || values.length != count) throw "Joint clearance partition needs one rate per joint";
    for (joint in 0...count) if (!Math.isFinite(from[joint]) || !Math.isFinite(to[joint]))
      throw "Joint clearance partition coordinates must be finite";
    for (joint in 0...count) for (values in [maximumVelocity, maximumAcceleration, maximumJerk])
      if (!Math.isFinite(values[joint]) || values[joint] <= 0.0)
        throw "Joint clearance partition rates must be finite and positive";
    var hit = clearance.violation(from);
    if (hit != null) throw 'Joint motion starts inside clearance: ${hit.a} to ${hit.b} is ${hit.distance} m, needs ${hit.required} m';
    hit = clearance.violation(to);
    if (hit != null) throw 'Joint motion ends inside clearance: ${hit.a} to ${hit.b} is ${hit.distance} m, needs ${hit.required} m';

    var result:Array<Array<Float>> = [];
    var checked = 0;
    function split(start:Array<Float>, end:Array<Float>, depth:Int):Void {
      if (++checked > MAX_JOINT_SEGMENT_CHECKS)
        throw "Joint clearance route exceeded its adaptive subdivision budget";
      var trajectory = Trajectory.generateStateToState(start, [for (_ in 0...count) 0.0],
        [for (_ in 0...count) 0.0], end, maximumVelocity, maximumAcceleration, maximumJerk);
      var failure:Null<ClearanceFailure>;
      try {
        failure = trajectoryFailure(trajectory, count);
      } catch (error:Dynamic) {
        trajectory.dispose();
        throw error;
      }
      trajectory.dispose();
      if (failure == null) {
        result.push(end.copy());
        return;
      }
      if (failure.kind == ClearanceFailureKind.Invalid) throw failure.message;
      // The joint-space chord is not the path the drive executes. A chord can
      // cross a hull even when the synchronized rest-to-rest trajectory clears
      // it, or when two shorter rest-to-rest moves around their midpoint clear
      // it. Subdivide the timed move and certify those actual trajectories.
      if (depth >= MAX_JOINT_SEGMENT_DEPTH)
        throw 'Joint clearance route could not be certified after adaptive subdivision: ${failure.message}';
      var middle = [for (joint in 0...count) start[joint] + (end[joint] - start[joint]) * 0.5];
      var middleHit = clearance.violation(middle);
      if (middleHit != null)
        throw 'Joint motion route midpoint violates clearance: ${middleHit.a} to ${middleHit.b} is ${middleHit.distance} m, needs ${middleHit.required} m';
      split(start, middle, depth + 1);
      split(middle, end, depth + 1);
    }
    split(from, to, 0);
    return result;
  }

  public function check(plan:ExecutionPlan):Void {
    var joints = clearance.arm.dofCount();
    if (plan.jointCount < joints) throw "Clearance plan has too few joints";
    var failure = checkSegments(plan.segments(), joints);
    if (failure != null) throw failure.message;
  }

  /** Checks an offline rest-to-rest trajectory with the plan admission rules. */
  public function checkTrajectory(trajectory:Trajectory):Void {
    var joints = clearance.arm.dofCount();
    if (trajectory.jointCount() < joints) throw "Clearance trajectory has too few joints";
    var failure = trajectoryFailure(trajectory, joints);
    if (failure != null) throw failure.message;
  }

  function trajectoryFailure(trajectory:Trajectory, joints:Int):Null<ClearanceFailure>
    return checkSegments(trajectory.segments(), joints);

  function checkSegments(segments:Array<{timeFromStartNs:Int64, durationNs:Int64,
      coefficients:Array<Array<Float>>}>, joints:Int):Null<ClearanceFailure> {
    for (segment in segments) {
      var duration = Int64.toFloat(segment.durationNs) * 1e-9;
      var speedBounds:Array<Float> = [];
      for (joint in 0...joints) {
        var coefficients = segment.coefficients[joint];
        var speedBound = 0.0, timePower = 1.0;
        for (power in 1...coefficients.length) {
          speedBound += power * Math.abs(coefficients[power]) * timePower;
          timePower *= duration;
        }
        if (!Math.isFinite(speedBound)) return {kind: ClearanceFailureKind.Invalid,
          message: "Clearance polynomial speed bound must be finite"};
        speedBounds.push(speedBound);
      }
      function positions(time:Float):Array<Float> {
        var result:Array<Float> = [];
        for (joint in 0...joints) {
          var coefficients = segment.coefficients[joint], position = 0.0;
          var power = coefficients.length;
          while (power > 0) {power--; position = position * time + coefficients[power];}
          result.push(position);
        }
        return result;
      }
      // Endpoints belong to the admitted motion even if an interval has zero duration.
      for (time in [0.0, duration]) {
        var hit = clearance.violation(positions(time));
        if (hit != null) return {kind: ClearanceFailureKind.Collision,
          message: 'Joint motion clearance: ${hit.a} to ${hit.b} is ${hit.distance} m, needs ${hit.required} m'};
      }
      var pending = [{start: 0.0, end: duration}];
      var checked = 0;
      while (pending.length > 0) {
        if (++checked > 100000) return {kind: ClearanceFailureKind.Unresolved,
          message: "Clearance certification budget exceeded; split the motion"};
        var interval = pending.pop();
        if (interval == null) return {kind: ClearanceFailureKind.Invalid,
          message: "Clearance interval stack is empty"};
        var middle = interval.start + (interval.end - interval.start) / 2;
        var q = positions(middle);
        var hit = clearance.violation(q);
        if (hit != null) return {kind: ClearanceFailureKind.Collision,
          message: 'Joint motion clearance: ${hit.a} to ${hit.b} is ${hit.distance} m, needs ${hit.required} m'};
        var radius = (interval.end - interval.start) / 2;
        if (clearance.intervalClear(q, [for (speed in speedBounds) speed * radius])) continue;
        if (middle == interval.start || middle == interval.end)
          return {kind: ClearanceFailureKind.Unresolved,
            message: "Joint motion clearance cannot certify the required margin"};
        pending.push({start: middle, end: interval.end});
        pending.push({start: interval.start, end: middle});
      }
    }
    return null;
  }
}
