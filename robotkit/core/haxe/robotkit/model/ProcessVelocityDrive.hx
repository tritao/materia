package robotkit.model;

/** Analog process channels selecting a signed velocity for one continuous joint. */
class ProcessVelocityDrive {
  public final speedChannel:String;
  public final directionChannel:String;
  /** Radians per second for one unit of the speed channel. */
  public final radiansPerSpeedUnit:Float;

  public function new(speedChannel:String, directionChannel:String, radiansPerSpeedUnit:Float) {
    if (speedChannel == null || speedChannel.length == 0 || directionChannel == null ||
        directionChannel.length == 0 || speedChannel == directionChannel ||
        !(radiansPerSpeedUnit > 0.0) || !Math.isFinite(radiansPerSpeedUnit))
      throw "Process velocity drive needs two analog channels and a positive finite speed scale";
    this.speedChannel = speedChannel;
    this.directionChannel = directionChannel;
    this.radiansPerSpeedUnit = radiansPerSpeedUnit;
  }
}
