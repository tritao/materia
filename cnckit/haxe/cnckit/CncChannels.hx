package cnckit;

/** Machine I/O channels used by the LinuxCNC dialect. */
class CncChannels {
  public static inline var SpindleDirection = "spindle.direction";
  public static inline var SpindleSpeed = "spindle.speed";
  public static inline var CoolantMist = "coolant.mist";
  public static inline var CoolantFlood = "coolant.flood";
  public static inline var OperatorResume = "cnc.operator.resume";
  public static inline var ToolChangePrefix = "cnc.tool_change.";

  public static function toolChange(number:Int):String
    return ToolChangePrefix + number;
}
