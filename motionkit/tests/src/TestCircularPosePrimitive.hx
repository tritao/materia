import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PathPrimitive;
import motionkit.path.PathPoint;
import motionkit.path.PosePrimitive;
import motionkit.path.PoseWaypoint;

/** Exact geometric primitive for MotionKit's circular compiler test. */
class TestCircularPosePrimitive implements PosePrimitive {
  final geometry:PathPrimitive;
  final feed:Float;

  public function new(geometry:PathPrimitive, feed:Float) {
    this.geometry = geometry;
    this.feed = feed;
  }

  public function length():Float return geometry.length();
  public function startWaypoint():PoseWaypoint return waypoint(geometry.pointAt(0.0));
  public function endWaypoint():PoseWaypoint return waypoint(geometry.pointAt(geometry.length()));
  public function speedLimit():Float return feed;
  public function orientationPolicy():OrientationPolicy return Fixed;
  public function waypointAt(distance:Float):PoseWaypoint
    return waypoint(geometry.pointAt(distance));

  function waypoint(point:PathPoint):PoseWaypoint
    return new PoseWaypoint(new Pose3(point.x, point.y, point.z), 0.0005, 0.02);
}
