package robotkit.manipulation;

import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** An oriented obstacle box. Its pose is in the planner's map frame, or in
 * the manipulator base frame when passed directly to ToolClearanceChecker. */
class ToolBoxObstacle {
  public final pose:Transform3;
  public final halfExtents:Vec3;

  public function new(pose:Transform3, halfExtents:Vec3) {
    if (pose == null || halfExtents == null || halfExtents.x <= 0 ||
        halfExtents.y <= 0 || halfExtents.z <= 0)
      throw "Tool obstacle requires a pose and positive half-extents";
    this.pose = pose;
    this.halfExtents = halfExtents;
  }
}
