package robotkit.world;

import haxe.Int64;

/** Transport-independent commands accepted by a logical robot. */
enum RobotCommand {
  /**
   * One atomic set of position, velocity, and/or effort targets.
   * `expiryNs` (null/zero: none) is a deadline on the robot's source clock
   * (`RobotSnapshot.sourceTimestampNs`): after it, the batch's velocity
   * targets lapse and those joints brake to zero within their acceleration
   * limits. In-process robots enforce it in the runtime; remote ones reject
   * it until host and robot clocks are mapped.
   */
  JointTargets(targets:Array<JointTarget>, expiryNs:Null<Int64>);
  /** Submit or replace a revision-bound execution plan. */
  ExecutionPlan(plan:robotkit.world.ExecutionPlanSubmission);
  /** Pause the native path clock while retaining its queue. */
  Hold;
  /** Resume a held native path. */
  Resume;
  /** Controlled straight-ramp stop that discards the path. */
  Abort;
}
