package processkit;

import motionkit.path.OrientationPolicy;
import motionkit.path.PosePath;
import motionkit.path.PosePrimitive;
import motionkit.path.PoseWaypoint;
import motionkit.path.PoseDerivatives;

/** Preserves authored primitive geometry when restarting inside a path. */
class ProcessPathSlice {
  public static function from(path:PosePath, startDistance:Float):PosePath {
    if (path == null || !Math.isFinite(startDistance) || startDistance < 0.0 ||
        startDistance >= path.length())
      throw "Process path restart distance lies outside the path";
    var pieces:Array<PosePrimitive> = [];
    var offset = 0.0;
    for (primitive in path.primitives) {
      var end = offset + primitive.length();
      if (end > startDistance + 1e-12) {
        var localStart = Math.max(0.0, startDistance - offset);
        pieces.push(new SlicedPosePrimitive(primitive, localStart,
          primitive.length()));
      }
      offset = end;
    }
    return new PosePath(path.frameId, pieces);
  }
}

private class SlicedPosePrimitive implements PosePrimitive {
  final source:PosePrimitive;
  final from:Float;
  final to:Float;

  public function new(source:PosePrimitive, from:Float, to:Float) {
    if (source == null || from < 0.0 || to > source.length() || to <= from)
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
  public function derivativesAt(distance:Float):PoseDerivatives
    return source.derivativesAt(Math.min(source.length(), Math.max(from, from + distance)));
}
