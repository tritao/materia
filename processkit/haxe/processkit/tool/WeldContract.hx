package processkit.tool;

import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;

/** Device-neutral welding units, feedback layout and safe output declarations. */
class WeldContract {
  public static inline var SENSOR_KIND:String = "tool_weld";
  public static inline var ARC:Int = 0;
  public static inline var CURRENT:Int = 1;
  public static inline var VOLTAGE:Int = 2;
  public static inline var TOUCH:Int = 3;
  public static inline var FAULT:Int = 4;
  public static inline var POWER:Int = 5;
  public static inline var SENSOR_COUNT:Int = 6;

  /** Wire speed is m/min, voltage is V; an optional job is a nonnegative integer encoded as an analog value. */
  public static function declarations(arc:String, wireSpeed:String, voltage:String, job:Null<String> = null):Array<ProcessChannelDeclaration> {
    var names = [arc, wireSpeed, voltage];
    if (job != null) names.push(job);
    for (i in 0...names.length) for (j in 0...i)
      if (names[i] == names[j]) throw "Welder channels must be distinct";
    var result = [
      new ProcessChannelDeclaration(arc, ProcessEventValue.Digital(false), WeldChannelPolicy.ARC_KEEPS_ON_STOP),
      new ProcessChannelDeclaration(wireSpeed, ProcessEventValue.Analog(0.0), WeldChannelPolicy.WIRE_SPEED_KEEPS_ON_STOP),
      new ProcessChannelDeclaration(voltage, ProcessEventValue.Analog(0.0), WeldChannelPolicy.VOLTAGE_KEEPS_ON_STOP)
    ];
    if (job != null) result.push(new ProcessChannelDeclaration(job, ProcessEventValue.Analog(0.0), true));
    return result;
  }
}
