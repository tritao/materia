package motionkit.path;

/** An exact interval of an authored primitive, preserving its parameterization. */
class PoseSlice implements PosePrimitive implements PosePositionBound {
  public function positionErrorBound(control:Array<Array<Float>>,start:Float,end:Float):Float {
    if(!Std.isOfType(source,PosePositionBound))return Math.POSITIVE_INFINITY;
    var low=Math.min(to,Math.max(from,from+start)),high=Math.min(to,Math.max(from,from+end));
    return high<=low ? Math.POSITIVE_INFINITY : (cast source:PosePositionBound).positionErrorBound(control,low,high);
  }
  public final source:PosePrimitive;
  public final from:Float;
  public final to:Float;

  public function new(source:PosePrimitive, from:Float, to:Float) {
    if (source == null || !Math.isFinite(from) || !Math.isFinite(to) || from < 0.0 || to > source.length() || to <= from)
      throw "Invalid process primitive slice";
    this.source = source;
    this.from = from;
    this.to = to;
  }

  public function length():Float return to - from;
  public function waypointAt(distance:Float):PoseWaypoint {
    if (!Math.isFinite(distance) || distance < -1e-12 ||
        distance > length() + 1e-12)
      throw "Process primitive distance outside slice";
    return source.waypointAt(Math.min(source.length(),
      Math.max(from, from + distance)));
  }
  public function startWaypoint():PoseWaypoint return source.waypointAt(from);
  public function endWaypoint():PoseWaypoint return source.waypointAt(to);
  public function speedLimit():Float return source.speedLimit();
  public function orientationPolicy():OrientationPolicy return source.orientationPolicy();
  public function derivativesAt(distance:Float):PoseDerivatives {
    if(!Math.isFinite(distance) || distance < -1e-12 || distance > length()+1e-12)
      throw "Pose slice derivative distance outside segment";
    return source.derivativesAt(Math.min(to, Math.max(from, from + distance)));
  }
}
