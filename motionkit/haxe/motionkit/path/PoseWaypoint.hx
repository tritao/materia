package motionkit.path;

import motionkit.kinematics.Pose3;

/** Authored pose and maximum Cartesian and angular error, in metres and radians. */
class PoseWaypoint {
  public final pose:Pose3;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;

  public function new(pose:Pose3, positionTolerance:Float, orientationTolerance:Float) {
    if (pose == null) throw "Pose waypoint requires a pose";
    if (!Math.isFinite(positionTolerance) || positionTolerance < 0.0 ||
        !Math.isFinite(orientationTolerance) || orientationTolerance < 0.0)
      throw "Pose tolerances must be finite and non-negative";
    this.pose = pose;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
  }
}
