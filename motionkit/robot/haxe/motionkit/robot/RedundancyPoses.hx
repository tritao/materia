package motionkit.robot;

import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import robotkit.manipulation.IkOptions;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Conversions shared by the redundancy parameterizations. */
class RedundancyPoses {
  public static function transform(value:Pose3):Transform3
    return new Transform3(new Vec3(value.x, value.y, value.z), new Quat(value.qx, value.qy, value.qz, value.qw));

  public static function options(tolerance:IkTolerance):IkOptions
    return new IkOptions(tolerance.position, tolerance.orientation, tolerance.maxIterations, tolerance.damping);
}
