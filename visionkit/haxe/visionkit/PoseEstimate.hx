package visionkit;

class PoseEstimate {
  public final camera_T_object:Pose3;
  public final reprojectionErrors:Array<Float>;
  public function new(pose:Pose3, errors:Array<Float>) {
    camera_T_object = pose;
    reprojectionErrors = errors;
  }
}
