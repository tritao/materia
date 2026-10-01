package motionkit.path;

/** Circular TCP arc through three non-collinear points in three dimensions. */
class PoseArc implements PosePrimitive {
  public final start:PoseWaypoint;
  public final via:PoseWaypoint;
  public final end:PoseWaypoint;
  public final policy:OrientationPolicy;
  public final feed:Float;
  final center:Array<Float>;
  final u:Array<Float>;
  final v:Array<Float>;
  final radius:Float;
  final sweep:Float;

  public function new(start:PoseWaypoint, via:PoseWaypoint, end:PoseWaypoint,
      policy:OrientationPolicy, feed:Float) {
    if (start == null || via == null || end == null || policy == null ||
        !Math.isFinite(feed) || feed <= 0.0) throw "Invalid pose arc";
    this.start = start; this.via = via; this.end = end;
    this.policy = policy; this.feed = feed;
    var a = [via.pose.x-start.pose.x, via.pose.y-start.pose.y, via.pose.z-start.pose.z];
    var b = [end.pose.x-start.pose.x, end.pose.y-start.pose.y, end.pose.z-start.pose.z];
    var aa = dot(a,a), ab = dot(a,b), bb = dot(b,b);
    var denominator = aa*bb-ab*ab;
    if (denominator <= 1e-20) throw "Pose arc points must be non-collinear";
    var alpha = bb*(aa-ab)/(2.0*denominator);
    var beta = aa*(bb-ab)/(2.0*denominator);
    center = [start.pose.x+alpha*a[0]+beta*b[0],
      start.pose.y+alpha*a[1]+beta*b[1], start.pose.z+alpha*a[2]+beta*b[2]];
    u = [start.pose.x-center[0], start.pose.y-center[1], start.pose.z-center[2]];
    radius = Math.sqrt(dot(u,u));
    for (i in 0...3) u[i] /= radius;
    var normal = cross(a,b);
    var normalLength = Math.sqrt(dot(normal,normal));
    for (i in 0...3) normal[i] /= normalLength;
    v = cross(normal,u);
    var viaAngle = Math.atan2(dot([via.pose.x-center[0],via.pose.y-center[1],via.pose.z-center[2]],v),
      dot([via.pose.x-center[0],via.pose.y-center[1],via.pose.z-center[2]],u));
    var endAngle = Math.atan2(dot([end.pose.x-center[0],end.pose.y-center[1],end.pose.z-center[2]],v),
      dot([end.pose.x-center[0],end.pose.y-center[1],end.pose.z-center[2]],u));
    if (viaAngle < 0.0) viaAngle += 2.0*Math.PI;
    if (endAngle < 0.0) endAngle += 2.0*Math.PI;
    sweep = viaAngle <= endAngle ? endAngle : endAngle+2.0*Math.PI;
  }

  static function dot(a:Array<Float>, b:Array<Float>):Float
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  static function cross(a:Array<Float>, b:Array<Float>):Array<Float>
    return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  public function length():Float return radius*sweep;
  public function startWaypoint():PoseWaypoint return start;
  public function endWaypoint():PoseWaypoint return end;
  public function speedLimit():Float return feed;
  public function orientationPolicy():OrientationPolicy return policy;
  public function derivativesAt(distance:Float):PoseDerivatives
    return PoseDerivatives.numeric(this, distance);

  public function waypointAt(distance:Float):PoseWaypoint {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > length())
      throw "Pose-arc distance outside segment";
    var t = distance/length(), angle = t*sweep;
    return PoseMath.interpolate(start,end,t,policy,
      center[0]+radius*(Math.cos(angle)*u[0]+Math.sin(angle)*v[0]),
      center[1]+radius*(Math.cos(angle)*u[1]+Math.sin(angle)*v[1]),
      center[2]+radius*(Math.cos(angle)*u[2]+Math.sin(angle)*v[2]));
  }
}
