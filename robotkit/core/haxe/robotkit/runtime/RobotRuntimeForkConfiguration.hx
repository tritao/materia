package robotkit.runtime;

/** Immutable resolved configuration used to construct a user-facing Forks view. */
class RobotRuntimeForkConfiguration {
  public final lift:RobotRuntimeForkAxisConfiguration;
  public final tilt:Null<RobotRuntimeForkAxisConfiguration>;
  public final spread:Null<RobotRuntimeForkAxisConfiguration>;
  public final maxMassKg:Float;
  public final maxLoadMomentKgMeters:Float;
  public final maxLiftHeightMeters:Float;

  public function new(
    lift:RobotRuntimeForkAxisConfiguration,
    tilt:Null<RobotRuntimeForkAxisConfiguration>,
    spread:Null<RobotRuntimeForkAxisConfiguration>,
    maxMassKg:Float,
    maxLoadMomentKgMeters:Float,
    maxLiftHeightMeters:Float
  ) {
    this.lift = lift;
    this.tilt = tilt;
    this.spread = spread;
    this.maxMassKg = maxMassKg;
    this.maxLoadMomentKgMeters = maxLoadMomentKgMeters;
    this.maxLiftHeightMeters = maxLiftHeightMeters;
  }
}
