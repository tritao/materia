package motionkit.robot;

import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ValidationReport;

/** Output of a planning request; diagnostics belong to that request only. */
typedef PlanningResult = {
  var trajectory:Trajectory;
  var report:Null<ValidationReport>;
  var diagnostics:Array<String>;
}
