package robotkit.device;

/** Deployment wiring for a physical switch; thresholds remain machine-model data. */
class DeviceInput {
  public final index:Int;
  public final switchId:String;
  public final actuator:String;
  public final activeHigh:Bool;
  public final pin:String;

  public function new(index:Int, switchId:String, actuator:String, activeHigh:Bool, pin:String = "") {
    this.index = index;
    this.switchId = switchId;
    this.actuator = actuator;
    this.activeHigh = activeHigh;
    this.pin = pin;
  }
}
