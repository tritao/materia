package robotkit.world;

import haxe.Int64;

/** Transport-independent commands accepted by a logical robot. */
enum RobotCommand {
  /**
   * One atomic set of position, velocity, and/or effort targets.
   * expiryNs must remain null/zero until runtime deadline enforcement is available.
   */
  JointTargets(targets:Array<JointTarget>, expiryNs:Null<Int64>);
  /** Append bounded polynomial segments to a runtime-owned queue. */
  TrajectoryChunk(chunk:robotkit.world.TrajectoryChunk);
  /** Submit or replace a revision-bound execution plan. */
  ExecutionPlan(plan:robotkit.world.ExecutionPlanSubmission);
  /** Pause the native path clock while retaining its queue. */
  Hold;
  /** Resume a held native path. */
  Resume;
  /** Controlled straight-ramp stop that discards the path. */
  Abort;
}
