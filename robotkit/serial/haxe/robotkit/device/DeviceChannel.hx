package robotkit.device;

/** One hardware channel, naming the model actuator it drives. */
class DeviceChannel {
  public final index:Int;
  public final actuator:String;
  /** 1, or -1 when the driver is wired in reverse. */
  public final direction:Int;
  public final directionSetupTicks:Int;
  /** Largest tolerated disagreement after pulse counts are normalized to SI leader coordinates. */
  public final skewBound:Float;

  public function new(index:Int, actuator:String, direction:Int = 1,
      directionSetupTicks:Int = 0, skewBound:Float = 0.0) {
    this.index = index;
    this.actuator = actuator;
    this.direction = direction;
    this.directionSetupTicks = directionSetupTicks;
    this.skewBound = skewBound;
  }
}
