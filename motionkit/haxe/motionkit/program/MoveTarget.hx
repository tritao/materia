package motionkit.program;

import motionkit.kinematics.Pose3;

/** Joint-space or framed Cartesian endpoint for a MoveJ operation. */
enum MoveTarget {
  JointTarget(joints:Array<Float>);
  PoseTarget(pose:Pose3, frameId:String, configurationHint:Null<Array<Float>>);
}
