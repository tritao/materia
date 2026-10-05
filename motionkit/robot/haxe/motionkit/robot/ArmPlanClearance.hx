package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;
import robotkit.manipulation.ArmClearance;

/** Checks the actual polynomial motion, including interior poses, against a cell snapshot.
    Sampling is bounded by each joint's maximum polynomial speed, rather than endpoint distance.
    Like ArmClearance.sweep this is a sampled check, not a continuous collision certificate. */
class ArmPlanClearance implements PlanValidator {
  public static inline var ANGULAR_STEP:Float = 0.02;
  public static inline var LINEAR_STEP:Float = 0.001;
  final clearance:ArmClearance;
  final jointSteps:Array<Float>;

  public function new(clearance:ArmClearance, jointSteps:Array<Float>) {
    if (clearance == null || jointSteps == null || jointSteps.length != clearance.arm.dofCount())
      throw "Plan clearance needs its arm and one sample step per joint";
    for (step in jointSteps) if (!(step > 0) || !Math.isFinite(step))
      throw "Plan clearance needs finite positive joint sample steps";
    this.clearance = clearance;
    this.jointSteps = jointSteps.copy();
  }

  public function fork():PlanValidator return this;

  public function check(plan:ExecutionPlan):Void {
    if (plan.jointCount < jointSteps.length) throw "Clearance plan has too few joints";
    for (segment in plan.segments()) {
      var duration = Int64.toFloat(segment.durationNs) * 1e-9;
      var samples = 1;
      for (joint in 0...jointSteps.length) {
        var coefficients = segment.coefficients[joint];
        var speedBound = 0.0, timePower = 1.0;
        for (power in 1...coefficients.length) {
          speedBound += power * Math.abs(coefficients[power]) * timePower;
          timePower *= duration;
        }
        var needed = Math.ceil(duration * speedBound / jointSteps[joint]);
        if (!Math.isFinite(needed) || needed > 100000)
          throw "Clearance sampling budget exceeded; split the motion";
        samples = Std.int(Math.max(samples, needed));
      }
      for (sample in 0...samples + 1) {
        var time = duration * sample / samples;
        var positions:Array<Float> = [];
        for (joint in 0...jointSteps.length) {
          var coefficients = segment.coefficients[joint], position = 0.0;
          var power = coefficients.length;
          while (power > 0) {power--; position = position * time + coefficients[power];}
          positions.push(position);
        }
        var hit = clearance.violation(positions);
        if (hit != null)
          throw 'Joint motion clearance: ${hit.a} to ${hit.b} is ${hit.distance} m, needs ${hit.required} m';
      }
    }
  }
}
