package robotkit.perception;

import robotkit.mobile.Pose2;

/**
 * Obstacle observation in the detection's frame: a capsule, the points within `radiusMeters` of a segment
 * that runs `halfLengthMeters` either side of the detection's pose along its heading. A zero half length is
 * a disk, so a point-like obstacle and a long flat one are the same kind of value.
 */
class Obstacle {
  public final detection:Detection;
  public final radiusMeters:Float;
  public final halfLengthMeters:Float;

  public function new(detection:Detection, radiusMeters:Float, ?halfLengthMeters:Float = 0.0) {
    if (detection == null || !Math.isFinite(radiusMeters) || radiusMeters <= 0.0 ||
        !Math.isFinite(halfLengthMeters) || halfLengthMeters < 0.0)
      throw "Obstacle requires a detection, a positive finite radius, and a nonnegative half length";
    this.detection = detection;
    this.radiusMeters = radiusMeters;
    this.halfLengthMeters = halfLengthMeters;
  }

  /** The segment's two ends, in the detection's frame. */
  public function ends():Array<Pose2> {
    var pose = detection.pose;
    var dx = Math.cos(pose.yaw) * halfLengthMeters, dy = Math.sin(pose.yaw) * halfLengthMeters;
    return [new Pose2(pose.x - dx, pose.y - dy, pose.yaw), new Pose2(pose.x + dx, pose.y + dy, pose.yaw)];
  }

  /**
   * Centres of disks of this obstacle's radius that, spaced no further than `spacingMeters`, cover the
   * segment end to end; the obstacle reduced to the points a disk test can handle.
   */
  public function disks(spacingMeters:Float):Array<Pose2> {
    if (halfLengthMeters <= 0.0) return [detection.pose];
    var count = Std.int(Math.ceil(2.0 * halfLengthMeters / Math.max(1e-6, spacingMeters))) + 1;
    var pose = detection.pose;
    return [for (index in 0...count) {
      var along = (count == 1 ? 0.0 : (index / (count - 1) * 2.0 - 1.0)) * halfLengthMeters;
      new Pose2(pose.x + Math.cos(pose.yaw) * along, pose.y + Math.sin(pose.yaw) * along, pose.yaw);
    }];
  }

  /** Shortest distance between this obstacle's segment and `other`'s (zero when they cross). */
  public function gapTo(other:Obstacle):Float {
    var a = ends(), b = other.ends();
    if (crosses(a[0], a[1], b[0], b[1])) return 0.0;
    return Math.min(Math.min(pointToSegment(a[0], b[0], b[1]), pointToSegment(a[1], b[0], b[1])),
      Math.min(pointToSegment(b[0], a[0], a[1]), pointToSegment(b[1], a[0], a[1])));
  }

  /** Distance from a point to the segment from `a` to `b`. */
  public static function pointToSegment(point:Pose2, a:Pose2, b:Pose2):Float {
    var dx = b.x - a.x, dy = b.y - a.y;
    var lengthSquared = dx * dx + dy * dy;
    var t = lengthSquared <= 1e-18 ? 0.0 : Math.max(0.0, Math.min(1.0, ((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSquared));
    var ex = a.x + t * dx - point.x, ey = a.y + t * dy - point.y;
    return Math.sqrt(ex * ex + ey * ey);
  }

  static function crosses(a:Pose2, b:Pose2, c:Pose2, d:Pose2):Bool {
    function side(p:Pose2, q:Pose2, r:Pose2):Float return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    var d1 = side(a, b, c), d2 = side(a, b, d), d3 = side(c, d, a), d4 = side(c, d, b);
    return d1 * d2 < 0.0 && d3 * d4 < 0.0;
  }
}
