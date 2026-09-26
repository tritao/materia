package robotkit.perception;

import robotkit.mobile.Pose2;
import robotkit.mobile.Pose3;

/** Typed detector result for one fiducial pose in its camera frame. */
class FiducialMarkerObservation {
  public final markerId:Int;
  public final pose:Pose2;
  /** Full camera-frame pose when supplied by a 3D detector. */
  public final pose3:Null<Pose3>;
  public final confidence:Float;

  public function new(markerId:Int, pose:Pose2, confidence:Float,
      ?pose3:Pose3) {
    if (markerId < 0 || pose == null || !Math.isFinite(confidence) ||
        confidence < 0.0 || confidence > 1.0)
      throw "Fiducial observation requires a non-negative marker ID, pose, and confidence from zero to one";
    this.markerId = markerId;
    this.pose = new Pose2(pose.x, pose.y, pose.yaw);
    this.pose3 = pose3 == null ? null : new Pose3(pose3.x, pose3.y, pose3.z,
      pose3.qx, pose3.qy, pose3.qz, pose3.qw);
    this.confidence = confidence;
  }

  /** Creates an observation whose translation and orientation use camera 3D axes. */
  public static function fromPose3(markerId:Int, pose:Pose3,
      confidence:Float):FiducialMarkerObservation {
    if (pose == null)
      throw "3D fiducial observation requires a pose";
    return new FiducialMarkerObservation(markerId, pose.planarPose(), confidence,
      pose);
  }
}
