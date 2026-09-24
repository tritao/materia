package robotkit.perception;

import robotkit.mobile.Pose2;

/** Detected dock pose and a configured approach pose in the same frame. */
class DockingTarget {
  public final detection:Detection;
  public final approachPose:Pose2;

  public function new(detection:Detection, approachPose:Pose2) {
    if (detection == null || approachPose == null)
      throw "Docking target requires a detection and approach pose";
    this.detection = detection;
    this.approachPose = new Pose2(approachPose.x, approachPose.y, approachPose.yaw);
  }
}
