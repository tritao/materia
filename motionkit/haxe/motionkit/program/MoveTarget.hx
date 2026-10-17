package motionkit.program;

import motionkit.kinematics.Pose3;

/** Joint-space or framed Cartesian endpoint for a MoveJ operation. */
enum MoveTarget {
  /** Absolute position of one independent joint; all others retain their planned coordinates. */
  JointPosition(joint:Int, position:Float);
  JointTarget(joints:Array<Float>);
  /**
    One joint moved by `delta` from wherever the program has it, the joints that follow it (a motor behind a gear
    pair) moving with it. `MotionOptions` limits are the driven joint's; the followers' are in proportion.
  **/
  JointOffset(joint:Int, delta:Float);
  PoseTarget(pose:Pose3, frameId:String, configurationHint:Null<Array<Float>>);
}
