package robotkit.tool;

import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;

/**
 * The channels a heater is worked by: its target, in degrees Celsius, and optionally the part-cooling fan that blows on
 * it (a duty from 0 to 1). Both are zero when the outputs are made safe, so a fault or an emergency stop switches them
 * off. The target keeps its value through a commanded stop or an aborted motion: pausing a print does not cool the
 * hotend, and a held temperature is the controller's to hold. The fan keeps its duty too.
 */
class HeaterChannels implements ToolChannels {
  public static inline var TARGET_KEEPS_ON_STOP:Bool = true;
  public static inline var FAN_KEEPS_ON_STOP:Bool = true;

  public final target:String;
  public final fan:Null<String>;

  public function new(target:String, ?fan:String) {
    if (fan != null && fan == target) throw "A heater's target and fan channels must be distinct";
    this.target = target;
    this.fan = fan;
  }

  public function declarations():Array<ProcessChannelDeclaration> {
    var result = [new ProcessChannelDeclaration(target, ProcessEventValue.Analog(0.0), TARGET_KEEPS_ON_STOP)];
    if (fan != null) result.push(new ProcessChannelDeclaration(fan, ProcessEventValue.Analog(0.0), FAN_KEEPS_ON_STOP));
    return result;
  }
}
