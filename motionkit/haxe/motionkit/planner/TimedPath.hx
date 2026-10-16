package motionkit.planner;

import motionkit.trajectory.Trajectory;

/** Lowered trajectory plus its exact path-distance timing contract. */
class TimedPath {
  public final trajectory:Trajectory;
  /** Which limit bound each stage of the time law; read lazily, and only while the time law is still held. */
  public var bindingConstraints(get, never):Array<BindingConstraint>;
  var bindings:Null<Array<BindingConstraint>>;
  final bindingSupplier:Null<Void -> Array<BindingConstraint>>;
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
      ?timesToDistancesMap:Array<Float> -> Array<Float>, ?lazyBindings:Void -> Array<BindingConstraint>) {
    this.timesToDistancesMap = timesToDistancesMap;
    if (trajectory == null || distanceToTimeMap == null || bindingConstraints == null)
      throw "Timed path needs a trajectory, distance map, and constraint report";
    this.trajectory = trajectory;
    this.distanceToTimeMap = distanceToTimeMap;
    bindings = lazyBindings == null ? bindingConstraints.copy() : null;
    bindingSupplier = lazyBindings;
    this.releaseMap = releaseMap;
  }

  /** Consume this result, transferring its distance-law ownership to a selected-joint copy.
   * Failure leaves the original result live. The selected copy preserves lazy binding diagnostics.
   */
  public function takeSelectedJoints(indices:Array<Int>):TimedPath {
    if (mapReleased) throw "Timed path distance map has been released";
    var selected = trajectory.selectJoints(indices);
    try {
      var result = new TimedPath(selected,distanceToTimeMap,bindings == null ? [] : bindings,
        releaseMap,timesToDistancesMap,bindings == null ? bindingSupplier : null);
      mapReleased = true;
      trajectory.dispose();
      return result;
    } catch (error:Dynamic) { selected.dispose(); throw error; }
  }

  function get_bindingConstraints():Array<BindingConstraint> {
    if (bindings == null) {
      if (mapReleased) throw "Timed path bindings must be read before its distance map is released";
      var supplier = bindingSupplier;
      if (supplier == null) throw "Timed path has no binding constraints";
      bindings = supplier();
    }
    return bindings;
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
