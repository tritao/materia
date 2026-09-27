package motionkit.planner;

import motionkit.trajectory.Trajectory;

/** Lowered trajectory plus its exact path-distance timing contract. */
class TimedPath {
  public final trajectory:Trajectory;
  public final bindingConstraints:Array<BindingConstraint>;
  final distanceToTimeMap:Float -> Float;

  public function new(trajectory:Trajectory, distanceToTimeMap:Float -> Float,
      bindingConstraints:Array<BindingConstraint>) {
    if (trajectory == null || distanceToTimeMap == null || bindingConstraints == null)
      throw "Timed path needs a trajectory, distance map, and constraint report";
    this.trajectory = trajectory;
    this.distanceToTimeMap = distanceToTimeMap;
    this.bindingConstraints = bindingConstraints.copy();
  }

  public function distanceToTime(distance:Float):Float return distanceToTimeMap(distance);
}
