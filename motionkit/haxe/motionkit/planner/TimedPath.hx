package motionkit.planner;

import motionkit.trajectory.Trajectory;

/** Lowered trajectory plus its exact path-distance timing contract. */
class TimedPath {
  public final trajectory:Trajectory;
  public final bindingConstraints:Array<BindingConstraint>;
  final distanceToTimeMap:Float -> Float;
  final releaseMap:Null<Void -> Void>;
  var mapReleased:Bool = false;

  public function new(trajectory:Trajectory, distanceToTimeMap:Float -> Float,
      bindingConstraints:Array<BindingConstraint>, ?releaseMap:Void -> Void) {
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

  /** Releases the native time law after all path events have been lowered. */
  public function releaseDistanceMap():Void {
    if (mapReleased) return;
    mapReleased = true;
    if (releaseMap != null) releaseMap();
  }
}
