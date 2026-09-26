package robotkit.world;

/** Transport-independent capabilities of a logical robot. */
class RobotCapabilities {
  public final id:RobotId;
  public final jointCount:Int;
  public final supportsPosition:Bool;
  public final supportsVelocity:Bool;
  public final supportsEffort:Bool;
  public final supportsPrediction:Bool;
  public final supportsTrajectoryQueue:Bool;

  public function new(
    id:RobotId,
    jointCount:Int,
    supportsPosition:Bool,
    supportsVelocity:Bool,
    supportsEffort:Bool,
    supportsPrediction:Bool,
    ?supportsTrajectoryQueue:Bool = false
  ) {
    this.id = id;
    this.jointCount = jointCount;
    this.supportsPosition = supportsPosition;
    this.supportsVelocity = supportsVelocity;
    this.supportsEffort = supportsEffort;
    this.supportsPrediction = supportsPrediction;
    this.supportsTrajectoryQueue = supportsTrajectoryQueue;
  }
}
