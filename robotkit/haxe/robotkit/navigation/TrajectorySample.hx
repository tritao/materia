package robotkit.navigation;

import robotkit.mobile.Pose2;
import robotkit.mobile.Twist2;

/** One immutable pose and body velocity sample along a trajectory. */
class TrajectorySample {
  public final timeFromStartSeconds:Float;
  public final pose:Pose2;
  public final twist:Twist2;

  public function new(timeFromStartSeconds:Float, pose:Pose2, twist:Twist2) {
    if (!Math.isFinite(timeFromStartSeconds) || timeFromStartSeconds < 0.0 ||
        pose == null || twist == null)
      throw "Trajectory samples require a finite time, pose, and twist";
    this.timeFromStartSeconds = timeFromStartSeconds;
    this.pose = new Pose2(pose.x, pose.y, pose.yaw);
    this.twist = new Twist2(twist.linear, twist.angular, twist.lateral);
  }
}
