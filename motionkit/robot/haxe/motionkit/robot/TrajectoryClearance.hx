package motionkit.robot;

import motionkit.trajectory.Trajectory;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.ArmClearance.ClearanceViolation;

/** Sweep the generated joint curve, retaining ArmClearance's sampled sweep contract. */
class TrajectoryClearance {
  public static function violation(world:ArmClearance,trajectory:Trajectory,contact:Bool=false,
      stepSeconds:Float=0.01):Null<ClearanceViolation> {
    if (world == null || trajectory == null || !Math.isFinite(stepSeconds) || stepSeconds <= 0.0)
      throw "Trajectory clearance requires a world, motion and positive sample step";
    var duration = trajectory.durationSeconds();
    var steps = Std.int(Math.ceil(duration / stepSeconds));
    if (steps < 1) steps = 1;
    var previous = trajectory.evaluate(0.0).positions;
    var failure = world.violation(previous,contact);
    if (failure != null) return failure;
    for (i in 1...(steps+1)) {
      var q = trajectory.evaluate(duration*i/steps).positions;
      failure = world.sweep(previous,q,contact);
      if (failure != null) return failure;
      previous = q;
    }
    return null;
  }
}
