package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;
import robotkit.manipulation.ArmClearance;

/** Certifies polynomial motion against a cell snapshot. Interval bounds include
    every hull point's travel, so narrow obstacles cannot hide between samples.
    Unresolved intervals fail admission when the subdivision budget is exhausted. */
class ArmPlanClearance implements PlanValidator {
  final clearance:ArmClearance;

  public function new(clearance:ArmClearance) {
    if (clearance == null) throw "Plan clearance needs its arm and cell bodies";
    this.clearance = clearance;
  }

  public function fork():PlanValidator return this;

  public function check(plan:ExecutionPlan):Void {
    var joints = clearance.arm.dofCount();
    if (plan.jointCount < joints) throw "Clearance plan has too few joints";
    for (segment in plan.segments()) {
      var duration = Int64.toFloat(segment.durationNs) * 1e-9;
      var speedBounds:Array<Float> = [];
      for (joint in 0...joints) {
        var coefficients = segment.coefficients[joint];
        var speedBound = 0.0, timePower = 1.0;
        for (power in 1...coefficients.length) {
          speedBound += power * Math.abs(coefficients[power]) * timePower;
          timePower *= duration;
        }
        if (!Math.isFinite(speedBound)) throw "Clearance polynomial speed bound must be finite";
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
        if (hit != null) throw 'Joint motion clearance: ${hit.a} to ${hit.b} is ${hit.distance} m, needs ${hit.required} m';
      }
      var pending = [{start: 0.0, end: duration}];
      var checked = 0;
      while (pending.length > 0) {
        if (++checked > 100000) throw "Clearance certification budget exceeded; split the motion";
        var interval = pending.pop();
        if (interval == null) throw "Clearance interval stack is empty";
        var middle = interval.start + (interval.end - interval.start) / 2;
        var q = positions(middle);
        var hit = clearance.violation(q);
        if (hit != null) throw 'Joint motion clearance: ${hit.a} to ${hit.b} is ${hit.distance} m, needs ${hit.required} m';
        var radius = (interval.end - interval.start) / 2;
        if (clearance.intervalClear(q, [for (speed in speedBounds) speed * radius])) continue;
        if (middle == interval.start || middle == interval.end)
          throw "Joint motion clearance cannot certify the required margin";
        pending.push({start: middle, end: interval.end});
        pending.push({start: interval.start, end: middle});
      }
    }
  }
}
