package robotkit.navigation;

import robotkit.mobile.Pose2;

/** Framed target pose and arrival tolerances for path following. */
class NavigationGoal {
  public final pose:Pose2;
  public final frameId:String;
  public final positionTolerance:Float;
  public final headingTolerance:Float;

  public function new(pose:Pose2, ?frameId:String = "map",
      ?positionTolerance:Float = 0.1, ?headingTolerance:Float = 0.1) {
    if (pose == null || frameId == null || frameId.length == 0 ||
        !Math.isFinite(positionTolerance) || positionTolerance <= 0.0 ||
        !Math.isFinite(headingTolerance) || headingTolerance <= 0.0)
      throw "Navigation goal requires a pose, frame, and positive finite tolerances";
    this.pose = new Pose2(pose.x, pose.y, pose.yaw);
    this.frameId = frameId;
    this.positionTolerance = positionTolerance;
    this.headingTolerance = headingTolerance;
  }
}
