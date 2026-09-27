package motionkit.path;

/** Straight TCP move. Rotation-to-length weight is metres per radian. */
class PoseLine implements PosePrimitive {
  public final start:PoseWaypoint;
  public final end:PoseWaypoint;
  public final policy:OrientationPolicy;
  public final rotationWeight:Float;
  public final feed:Float;
  final pathLength:Float;

  public function new(start:PoseWaypoint, end:PoseWaypoint, policy:OrientationPolicy,
      rotationWeight:Float, feed:Float) {
    if (start == null || end == null || policy == null) throw "Pose line needs endpoints and orientation policy";
    if (!Math.isFinite(rotationWeight) || rotationWeight <= 0.0 || !Math.isFinite(feed) || feed <= 0.0)
      throw "Pose line requires positive finite rotation weight and feed";
    this.start = start; this.end = end; this.policy = policy;
    this.rotationWeight = rotationWeight; this.feed = feed;
    var distance = PoseMath.distance(start.pose, end.pose);
    pathLength = distance > 0.0 ? distance : rotationWeight * PoseMath.angle(start.pose, end.pose);
    if (pathLength <= 0.0) throw "Pose line needs translation or rotation";
  }

  public function length():Float return pathLength;
  public function startWaypoint():PoseWaypoint return start;
  public function endWaypoint():PoseWaypoint return end;
  public function speedLimit():Float return feed;

  public function waypointAt(distance:Float):PoseWaypoint {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > pathLength)
      throw "Pose-line distance outside segment";
    var t = distance / pathLength;
    return PoseMath.interpolate(start, end, t, policy,
      start.pose.x + t * (end.pose.x - start.pose.x),
      start.pose.y + t * (end.pose.y - start.pose.y),
      start.pose.z + t * (end.pose.z - start.pose.z));
  }
}
