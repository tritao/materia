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
  /** Where each primitive ends along the path, summed in order. */
  final ends:Array<Float> = [];

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
      ends.push(sum);
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

  /** The first primitive that ends at or after `distance` (the last one past the end), by bisection. */
  public function indexAt(distance:Float):Int {
    var low = 0, high = primitives.length - 1;
    while (low < high) {
      var middle = (low + high) >> 1;
      if (distance <= ends[middle]) high = middle; else low = middle + 1;
    }
    return low;
  }

  /** The first primitive that ends after `distance` (the last one past the end). */
  public function indexAfter(distance:Float):Int {
    var low = 0, high = primitives.length - 1;
    while (low < high) {
      var middle = (low + high) >> 1;
      if (distance < ends[middle]) high = middle; else low = middle + 1;
    }
    return low;
  }

  /** Where primitive `index` starts along the path. */
  public function startOf(index:Int):Float return index == 0 ? 0.0 : ends[index - 1];

  public function orientationPolicyAt(distance:Float):OrientationPolicy {
    if (!Math.isFinite(distance) || distance < -1e-12 ||
        distance > totalLength + 1e-12)
      throw "Pose-path distance outside path";
    distance = Math.min(totalLength, Math.max(0.0, distance));
    return primitives[indexAt(distance)].orientationPolicy();
  }

  public function waypointAt(distance:Float):PoseWaypoint {
    if (!Math.isFinite(distance) || distance < -1e-12 ||
        distance > totalLength + 1e-12)
      throw "Pose-path distance outside path";
    distance = Math.min(totalLength, Math.max(0.0, distance));
    var index = indexAt(distance), primitive = primitives[index];
    return primitive.waypointAt(Math.min(primitive.length(), Math.max(0.0, distance - startOf(index))));
  }
}
