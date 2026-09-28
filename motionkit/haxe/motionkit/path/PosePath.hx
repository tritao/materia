package motionkit.path;

import motionkit.event.PathEvent;
import motionkit.kinematics.Pose3;
import motionkit.program.MotionPath;

/** Ordered path in one explicit frame, measured in metres of path progress. */
class PosePath implements MotionPath {
  public final frameId:String;
  public final primitives:Array<PosePrimitive>;
  public final events:Array<PathEvent>;
  /** Original geometry for paths that replace corners with tolerance blends. */
  public var authoredGeometry(default, null):Null<GeometricPath> = null;
  public var blendTolerance(default, null):Float = 0.0;
  final totalLength:Float;

  public function new(frameId:String, primitives:Array<PosePrimitive>, ?events:Array<PathEvent>) {
    if (frameId == null || StringTools.trim(frameId).length == 0) throw "Pose path needs a frame";
    if (primitives == null || primitives.length == 0) throw "Pose path needs primitives";
    this.frameId = frameId;
    this.primitives = primitives.copy();
    var sum = 0.0;
    for (i in 0...this.primitives.length) {
      var primitive = this.primitives[i];
      if (primitive == null) throw "Pose path contains a null primitive";
      if (i > 0 && PoseMath.distance(this.primitives[i-1].endWaypoint().pose,
          primitive.startWaypoint().pose) > 1e-8)
        throw "Pose path has disconnected primitives";
      sum += primitive.length();
    }
    totalLength = sum;
    this.events = events == null ? [] : events.copy();
    var previous = 0.0;
    for (event in this.events) {
      if (event == null || event.distance < previous || event.distance > sum)
        throw "Pose path events must be sorted and within the path";
      previous = event.distance;
    }
  }

  public function length():Float return totalLength;

  public function withAuthoredGeometry(geometry:GeometricPath, tolerance:Float):PosePath {
    if (geometry == null || !Math.isFinite(tolerance) || tolerance <= 0.0)
      throw "Authored geometry needs a positive blend tolerance";
    authoredGeometry = geometry;
    blendTolerance = tolerance;
    return this;
  }
  public function poseAt(distance:Float):Pose3 return waypointAt(distance).pose;

  public function orientationPolicyAt(distance:Float):OrientationPolicy {
    if (!Math.isFinite(distance) || distance < -1e-12 ||
        distance > totalLength + 1e-12)
      throw "Pose-path distance outside path";
    distance = Math.min(totalLength, Math.max(0.0, distance));
    var start = 0.0;
    for (primitive in primitives) {
      start += primitive.length();
      if (distance <= start) return primitive.orientationPolicy();
    }
    return primitives[primitives.length-1].orientationPolicy();
  }

  public function waypointAt(distance:Float):PoseWaypoint {
    if (!Math.isFinite(distance) || distance < -1e-12 ||
        distance > totalLength + 1e-12)
      throw "Pose-path distance outside path";
    distance = Math.min(totalLength, Math.max(0.0, distance));
    var start = 0.0;
    for (i in 0...primitives.length) {
      var end = start + primitives[i].length();
      if (distance <= end || i == primitives.length - 1)
        return primitives[i].waypointAt(Math.min(primitives[i].length(),
          Math.max(0.0, distance - start)));
      start = end;
    }
    return primitives[primitives.length-1].endWaypoint();
  }
}
