package motionkit.trajectory;

import MotionKitNative;

/** The shared path derivative source for STOP and hold-lead calculations. */
class PathDerivativeEstimate {
  public final velocities:Array<Float>;
  public final accelerations:Array<Float>;
  public final recentAccelerations:Array<Float>;
  public final hasForwardAcceleration:Bool;
  public final hasRecentAcceleration:Bool;

  public function new(native:mk_path_derivative_estimate) {
    velocities = [];
    accelerations = [];
    recentAccelerations = [];
    for (joint in 0...native.get_joint_count()) {
      velocities.push(native.get_velocity(joint));
      accelerations.push(native.get_acceleration(joint));
      recentAccelerations.push(native.get_recent_acceleration(joint));
    }
    hasForwardAcceleration = native.get_has_forward_acceleration() != 0;
    hasRecentAcceleration = native.get_has_recent_acceleration() != 0;
  }
}
