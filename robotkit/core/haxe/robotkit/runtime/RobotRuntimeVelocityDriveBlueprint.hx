package robotkit.runtime;

/** Compiled analog process binding for one continuously rotating joint. */
class RobotRuntimeVelocityDriveBlueprint {
  public final joint:Int;
  public final speedChannel:String;
  public final directionChannel:String;
  public final radiansPerSpeedUnit:Float;
  public final maxEffort:Float;
  public final maxRate:Float;

  public function new(joint:Int, speedChannel:String, directionChannel:String,
      radiansPerSpeedUnit:Float, maxEffort:Float, maxRate:Float) {
    this.joint = joint;
    this.speedChannel = speedChannel;
    this.directionChannel = directionChannel;
    this.radiansPerSpeedUnit = radiansPerSpeedUnit;
    this.maxEffort = maxEffort;
    this.maxRate = maxRate;
  }
}
