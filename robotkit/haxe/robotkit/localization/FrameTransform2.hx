package robotkit.localization;

import robotkit.mobile.Pose2;

/** Static planar transform: child frame pose expressed in the parent frame. */
class FrameTransform2 {
  public final parentFrame:String;
  public final childFrame:String;
  public final childPoseInParent:Pose2;

  public function new(parentFrame:String, childFrame:String, childPoseInParent:Pose2) {
    if (parentFrame == null || parentFrame.length == 0 || childFrame == null ||
        childFrame.length == 0 || parentFrame == childFrame || childPoseInParent == null)
      throw "Frame transform requires distinct frame IDs and a pose";
    this.parentFrame = parentFrame;
    this.childFrame = childFrame;
    this.childPoseInParent = new Pose2(childPoseInParent.x, childPoseInParent.y,
      childPoseInParent.yaw);
  }
}
