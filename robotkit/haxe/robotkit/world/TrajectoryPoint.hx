package robotkit.world;

import haxe.Int64;

/** One timestamped full-joint position sample accepted by a RobotRuntime. */
class TrajectoryPoint {
  public static inline final MAX_JOINTS:Int = 64;
  public final timeFromStartNs:Int64;
  public final positions:Array<Float>;

  public function new(timeFromStartNs:Int64, positions:Array<Float>) {
    if (timeFromStartNs == null || Int64.compare(timeFromStartNs, Int64.ofInt(0)) < 0)
      throw "Trajectory point time must be finite and non-negative";
    if (positions == null || positions.length == 0 || positions.length > MAX_JOINTS)
      throw 'Trajectory point needs one to $MAX_JOINTS joint positions';
    for (position in positions)
      if (!Math.isFinite(position)) throw "Trajectory point positions must be finite";
    this.timeFromStartNs = timeFromStartNs;
    this.positions = positions.copy();
  }

  public function copy():TrajectoryPoint
    return new TrajectoryPoint(timeFromStartNs, positions);
}
