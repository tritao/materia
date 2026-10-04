package robotkit.runtime;

import haxe.Int64;

/** A physical virtual switch bound to an actuator's actual step counter. */
class VirtualInputOptions {
  public final switchId:String;
  public final actuator:Int;
  public final thresholdSteps:Int64;
  public final activeAbove:Bool;
  public final activeHigh:Bool;
  public function new(switchId:String, actuator:Int, thresholdSteps:Int64, activeAbove:Bool, activeHigh:Bool) {
    if (switchId == null || switchId.length == 0 || switchId.length > 63 || actuator < 0 || actuator >= 64)
      throw "Virtual switch requires an ID and actuator channel";
    this.switchId = switchId; this.actuator = actuator; this.thresholdSteps = thresholdSteps;
    this.activeAbove = activeAbove; this.activeHigh = activeHigh;
  }
}
