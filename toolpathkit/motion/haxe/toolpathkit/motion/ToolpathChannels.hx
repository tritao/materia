package toolpathkit.motion;

/** Controller channels scoped to one machine. A standalone machine uses an empty scope. */
class ToolpathChannels {
  public final spindleDirection:String;
  public final spindleSpeed:String;
  public final spindleAtSpeed:String;
  public final coolantMist:String;
  public final coolantFlood:String;
  public final operatorResume:String;
  public final toolChangePrefix:String;

  public function new(scope:String) {
    if (scope == null || StringTools.endsWith(scope, "/")) throw "CNC channel scope must omit its trailing slash";
    var prefix = scope.length == 0 ? "" : scope + "/";
    spindleDirection = prefix + "spindle.direction";
    spindleSpeed = prefix + "spindle.speed";
    spindleAtSpeed = prefix + "spindle.at_speed";
    coolantMist = prefix + "coolant.mist";
    coolantFlood = prefix + "coolant.flood";
    operatorResume = prefix + "cnc.operator.resume";
    toolChangePrefix = prefix + "cnc.tool_change.";
  }

  public function toolChange(number:Int):String return toolChangePrefix + number;
}
