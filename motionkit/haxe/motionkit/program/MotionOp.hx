package motionkit.program;

import motionkit.MotionOptions;
import motionkit.event.EventValue;
import motionkit.event.PathEvent;
import motionkit.kinematics.Pose3;
import motionkit.path.PosePath;
import motionkit.path.OrientationPolicy;

/** Controller-independent requested motion and program barriers. */
enum MotionOp {
  MoveJ(target:MoveTarget, limits:MotionOptions, blend:Blend);
  MoveL(pose:Pose3, frameId:String, feed:Float, blend:Blend, ?freedom:OrientationPolicy);
  MoveC(via:Pose3, end:Pose3, frameId:String, feed:Float, blend:Blend, ?freedom:OrientationPolicy);
  FollowPath(path:PosePath, frameId:String, timing:Float, events:Array<PathEvent>);
  Dwell(seconds:Float);
  SetOutput(channel:String, value:EventValue);
  /** A null timeout waits indefinitely. */
  WaitInput(channel:String, predicate:InputPredicate, timeoutSeconds:Null<Float>);
}
