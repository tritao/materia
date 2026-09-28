package toolpathkit.motion;

import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PathPrimitive;
import motionkit.path.PathPoint;
import motionkit.path.PosePrimitive;
import motionkit.path.PoseWaypoint;

/** Keeps exact line, arc, helix, or fillet geometry for ProgramCompiler. */
class ToolpathPosePrimitive implements PosePrimitive {
  public final geometry:PathPrimitive;
  public final feed:Float;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;

  public function new(geometry:PathPrimitive, feed:Float,
      positionTolerance:Float, orientationTolerance:Float) {
    if (geometry == null || geometry.length() <= 0.0 ||
        !Math.isFinite(feed) || feed <= 0.0)
      throw "CNC path primitive needs geometry and positive feed";
    this.geometry = geometry;
    this.feed = feed;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
  }

  public function length():Float return geometry.length();
  public function startWaypoint():PoseWaypoint return waypoint(geometry.pointAt(0.0));
  public function endWaypoint():PoseWaypoint return waypoint(geometry.pointAt(geometry.length()));
  public function speedLimit():Float return feed;
  public function orientationPolicy():OrientationPolicy return OrientationPolicy.Fixed;
  public function waypointAt(distance:Float):PoseWaypoint
    return waypoint(geometry.pointAt(distance));

  function waypoint(point:PathPoint):PoseWaypoint
    return new PoseWaypoint(new Pose3(point.x, point.y, point.z),
      positionTolerance, orientationTolerance);
}
