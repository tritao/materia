package motionkit.robot;

import motionkit.trajectory.ExecutionPlan;

/** An admission check on the actual timed plan, before any segment is sent. */
interface PlanValidator {
  public function check(plan:ExecutionPlan):Void;
  /** A worker copy, or the same object when the check is immutable. */
  public function fork():PlanValidator;
}
