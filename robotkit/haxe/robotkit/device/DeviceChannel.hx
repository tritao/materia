package robotkit.device;

/**
 * One ordered hardware channel. A wired channel names the model actuator it drives and how the
 * driver is set up; a joint-only channel (`actuator` null) only maps a channel to a joint.
 */
class DeviceChannel {
  public final index:Int;
  /** The model joint this channel moves, or empty when the layout names only the actuator. */
  public final jointId:String;
  public final actuator:Null<String>;
  /** 1, or -1 when the driver is wired so a positive actuator coordinate steps the other way. */
  public final direction:Int;
  /** Driver microsteps per full step. */
  public final microsteps:Int;
  /** Step ticks the driver needs between a direction change and the next step. */
  public final directionSetupTicks:Int;
  /** Largest disagreement, in actuator units, tolerated between dual-driven channels. */
  public final skewBound:Float;

  public function new(index:Int, jointId:String, ?actuator:String, direction:Int = 1,
      microsteps:Int = 1, directionSetupTicks:Int = 0, skewBound:Float = 0.0) {
    this.index = index;
    this.jointId = jointId;
    this.actuator = actuator;
    this.direction = direction;
    this.microsteps = microsteps;
    this.directionSetupTicks = directionSetupTicks;
    this.skewBound = skewBound;
  }
}
