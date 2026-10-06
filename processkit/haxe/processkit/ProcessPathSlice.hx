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
        pieces.push(new motionkit.path.PoseSlice(primitive, localStart,
          primitive.length()));
      }
      offset = end;
    }
    return new PosePath(path.frameId, pieces);
  }
}
