import haxe.Int64;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.execution.TrajectorySegment;

class PlanControlRegression {
  public static function main():Void {
    var segments = [for (i in 0...4) new TrajectorySegment(Int64.ofInt(i * 1000000), Int64.ofInt(1000000),
      [for (_ in 0...6) [0.0]])];
    var plan = new ExecutionPlanSubmission(Int64.ofInt(1), Int64.ofInt(1), Int64.ofInt(1), 0,
      [for (_ in 0...6) 0.0], [for (_ in 0...6) 0.0], [for (_ in 0...6) 0.0], segments);
    Sys.println('Latest-main regression: ${plan.startPosition.length} joints, ${plan.segments.length} segments, ' +
      '${plan.controlAcceleration.length} control accelerations');
    if (plan.controlAcceleration.length != plan.startPosition.length)
      throw "Control-acceleration defaults are sized by segments instead of joints";
  }
}
