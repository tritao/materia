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
  /** Proven deviation of the rounded geometry itself; null when the author supplies no bound. */
  public var geometryDeviationBound(default, null):Null<Float> = null;
  /** A joint driven by distance along the path (an extruder), if any. */
  public var drive(default, null):Null<DistanceDrive> = null;
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

  public function withAuthoredGeometry(geometry:GeometricPath, tolerance:Float, ?deviationBound:Float):PosePath {
    if (geometry == null || !Math.isFinite(tolerance) || tolerance <= 0.0)
      throw "Authored geometry needs a positive blend tolerance";
    authoredGeometry = geometry;
    blendTolerance = tolerance;
    if(deviationBound!=null && (!Math.isFinite(deviationBound) || deviationBound<0 || deviationBound>tolerance))
      throw "Authored geometry deviation must fit its tolerance";
    geometryDeviationBound=deviationBound;
    return this;
  }
  public function withDrive(value:DistanceDrive):PosePath {
    if (value == null || Math.abs(value.pathLength - totalLength) > 1e-9 * Math.max(1.0, totalLength))
      throw "A path drive must span its path";
    drive = value;
    return this;
  }

  /** The primitives from `first` up to (not including) `last` as a path of their own, with the geometry and drive that belong to them. */
  public function sub(first:Int, last:Int):PosePath {
    if (first < 0 || last > primitives.length || first >= last) throw "Path slice is out of range";
    var result = new PosePath(frameId, primitives.slice(first, last));
    if (authoredGeometry != null) result.withAuthoredGeometry(authoredGeometry, blendTolerance,geometryDeviationBound);
    if (drive != null) {
      var from = startOf(first), to = ends[last - 1];
      var driven = drive.slice(from, to);
      // The slice spans the pieces' lengths as summed from zero; keep the path's own.
      if (driven != null) result.drive = Math.abs(driven.pathLength - result.length()) <= 1e-9 ? driven :
        new DistanceDrive(driven.joint, driven.intervals, result.length(), driven.ramp);
    }
    return result;
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
