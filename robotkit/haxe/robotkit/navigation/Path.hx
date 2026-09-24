package robotkit.navigation;

import robotkit.mobile.Pose2;

/** Immutable polyline of planar poses expressed in one named frame. */
class Path {
  final waypoints:Array<Pose2>;
  final distances:Array<Float>;
  public final frameId:String;
  public final length:Float;

  public function new(waypoints:Array<Pose2>, ?frameId:String = "map") {
    if (waypoints == null || waypoints.length < 2)
      throw "A path requires at least two poses";
    if (frameId == null || frameId.length == 0)
      throw "A path requires a frame ID";
    this.frameId = frameId;
    this.waypoints = [];
    distances = [0.0];
    var total = 0.0;
    for (index in 0...waypoints.length) {
      var point = waypoints[index];
      if (point == null) throw "Path poses cannot be null";
      this.waypoints.push(new Pose2(point.x, point.y, point.yaw));
      if (index > 0) {
        var previous = waypoints[index - 1];
        var dx = point.x - previous.x;
        var dy = point.y - previous.y;
        total += Math.pow(dx * dx + dy * dy, 0.5);
        distances.push(total);
      }
    }
    if (!Math.isFinite(total) || total <= 1e-9)
      throw "Path length must be finite and positive";
    length = total;
  }

  public function count():Int return waypoints.length;

  public function start():Pose2 {
    var point = waypoints[0];
    return new Pose2(point.x, point.y, point.yaw);
  }

  public function goal():Pose2 {
    var point = waypoints[waypoints.length - 1];
    return new Pose2(point.x, point.y, point.yaw);
  }

  public function poses():Array<Pose2>
    return [for (point in waypoints) new Pose2(point.x, point.y, point.yaw)];

  /** Returns the path pose at an arc-length distance from its start. */
  public function poseAt(distance:Float):Pose2 {
    if (!Math.isFinite(distance)) throw "Path distance must be finite";
    if (distance <= 0.0) return start();
    if (distance >= length) return goal();
    for (index in 0...waypoints.length - 1) {
      var startDistance = distances[index];
      var endDistance = distances[index + 1];
      var segmentLength = endDistance - startDistance;
      if (segmentLength <= 1e-9 || distance > endDistance) continue;
      var alpha = (distance - startDistance) / segmentLength;
      var from = waypoints[index];
      var to = waypoints[index + 1];
      var yawDelta = Pose2.wrapAngle(to.yaw - from.yaw);
      return new Pose2(from.x + (to.x - from.x) * alpha,
        from.y + (to.y - from.y) * alpha, from.yaw + yawDelta * alpha);
    }
    return goal();
  }

  /** Projects a pose onto the not-yet-traversed path and returns monotonic arc length. */
  public function nearestDistance(pose:Pose2, minimumDistance:Float):Float {
    if (pose == null || !Math.isFinite(minimumDistance))
      throw "Path projection requires a pose and finite progress";
    var minimum = minimumDistance < 0.0 ? 0.0 : minimumDistance;
    if (minimum > length) minimum = length;
    var bestDistance = 1.0e300;
    var bestProgress = minimum;
    for (index in 0...waypoints.length - 1) {
      var startDistance = distances[index];
      var endDistance = distances[index + 1];
      var segmentLength = endDistance - startDistance;
      if (segmentLength <= 1e-9 || endDistance < minimum) continue;
      var from = waypoints[index];
      var to = waypoints[index + 1];
      var dx = to.x - from.x;
      var dy = to.y - from.y;
      var alpha = ((pose.x - from.x) * dx + (pose.y - from.y) * dy) /
        (segmentLength * segmentLength);
      if (alpha < 0.0) alpha = 0.0;
      if (alpha > 1.0) alpha = 1.0;
      var candidate = startDistance + alpha * segmentLength;
      if (candidate < minimum) continue;
      var px = from.x + alpha * dx;
      var py = from.y + alpha * dy;
      var errorX = pose.x - px;
      var errorY = pose.y - py;
      var squaredError = errorX * errorX + errorY * errorY;
      if (squaredError < bestDistance) {
        bestDistance = squaredError;
        bestProgress = candidate;
      }
    }
    return bestProgress;
  }
}
