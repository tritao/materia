package motionkit.planner;

import motionkit.trajectory.Trajectory;

/** Lowered trajectory plus its exact path-distance timing contract. */
class TimedPath {
  public final trajectory:Trajectory;
  public final bindingConstraints:Array<BindingConstraint>;
  final distanceToTimeMap:Float -> Float;
  final releaseMap:Null<Void -> Void>;
  final timesToDistancesMap:Null<Array<Float> -> Array<Float>>;
  var mapReleased:Bool = false;

  /**
   * `timesToDistancesMap`, when the timing backend can invert its time law directly, maps many times to
   * path distances in one step; callers otherwise search with `distanceToTime`.
   */
  public function new(trajectory:Trajectory, distanceToTimeMap:Float -> Float,
      bindingConstraints:Array<BindingConstraint>, ?releaseMap:Void -> Void,
      ?timesToDistancesMap:Array<Float> -> Array<Float>) {
    this.timesToDistancesMap = timesToDistancesMap;
    if (trajectory == null || distanceToTimeMap == null || bindingConstraints == null)
      throw "Timed path needs a trajectory, distance map, and constraint report";
    this.trajectory = trajectory;
    this.distanceToTimeMap = distanceToTimeMap;
    this.bindingConstraints = bindingConstraints.copy();
    this.releaseMap = releaseMap;
  }

  public function distanceToTime(distance:Float):Float {
    if (mapReleased) throw "Timed path distance map has been released";
    return distanceToTimeMap(distance);
  }

  public function hasDirectInverse():Bool
    return timesToDistancesMap != null;

  /** Path distance reached at each time; only valid when `hasDirectInverse()`. */
  public function timesToDistances(seconds:Array<Float>):Array<Float> {
    if (mapReleased) throw "Timed path distance map has been released";
    if (timesToDistancesMap == null) throw "Timed path has no direct time-to-distance map";
    return timesToDistancesMap(seconds);
  }

  /** Releases the native time law after all path events have been lowered. */
  public function releaseDistanceMap():Void {
    if (mapReleased) return;
    mapReleased = true;
    if (releaseMap != null) releaseMap();
  }
}
