import haxe.Int64;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.execution.TrajectorySegment;

class PlanControlRegression {
  public static function main():Void {
    var segments = [for (i in 0...4) new TrajectorySegment(Int64.ofInt(i * 1000000), Int64.ofInt(1000000),
      [for (_ in 0...6) [0.0]])];
    var plan = new ExecutionPlanSubmission(Int64.ofInt(1), Int64.ofInt(1), Int64.ofInt(1), 0,
      [for (_ in 0...6) 0.0], [for (_ in 0...6) 0.0], [for (_ in 0...6) 0.0], segments);
    Sys.println('Plan control: ${plan.startPosition.length} joints, ${plan.segments.length} segments, ' +
      '${plan.controlAcceleration.length} control accelerations');
    if (plan.controlAcceleration.length != plan.startPosition.length)
      throw "Control-acceleration defaults are sized by segments instead of joints";
    var explicit = make(segments, [for (_ in 0...6) 2.0]);
    if (explicit.controlAcceleration.get(5) != 2.0) throw "Explicit joint acceleration lost";
    var rejected = false;
    try make(segments, [for (_ in 0...4) 2.0]) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "Segment-sized control acceleration accepted";
    Sys.println("Default and explicit joint arrays pass; segment-sized array rejected");
  }
  static function make(segments:Array<TrajectorySegment>, control:Array<Float>):ExecutionPlanSubmission {
    return new ExecutionPlanSubmission(Int64.ofInt(1), Int64.ofInt(1), Int64.ofInt(1), 0,
      [for (_ in 0...6) 0.0], [for (_ in 0...6) 0.0], [for (_ in 0...6) 0.0], segments,
      null, null, null, null, null, true, null, false, null, control);
  }
}
