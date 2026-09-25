package robotkit.model;

/** Authored fork-axis roles and application-level payload/lift envelope. */
class RobotForkConfiguration {
  public final liftJointId:JointId;
  public final tiltJointId:Null<JointId>;
  public final spreadJointId:Null<JointId>;
  public final maxMassKg:Float;
  public final maxLoadMomentKgMeters:Float;
  public final maxLiftHeightMeters:Float;

  public function new(
    liftJointId:JointId,
    maxMassKg:Float,
    maxLoadMomentKgMeters:Float,
    maxLiftHeightMeters:Float,
    ? tiltJointId:JointId,
    ? spreadJointId:JointId
  ) {
    this.liftJointId = liftJointId;
    this.tiltJointId = tiltJointId;
    this.spreadJointId = spreadJointId;
    this.maxMassKg = maxMassKg;
    this.maxLoadMomentKgMeters = maxLoadMomentKgMeters;
    this.maxLiftHeightMeters = maxLiftHeightMeters;
  }
}
