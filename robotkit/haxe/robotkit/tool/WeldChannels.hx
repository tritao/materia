package robotkit.tool;

import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;

/** The channels a welder is worked by, declared on the robot with their safe values and stop policy (`WeldChannelPolicy`). */
class WeldChannels {
  public static function declarations(arc:String, wireSpeed:String, voltage:String):Array<ProcessChannelDeclaration>
    return [
      new ProcessChannelDeclaration(arc, ProcessEventValue.Digital(false), WeldChannelPolicy.ARC_KEEPS_ON_STOP),
      new ProcessChannelDeclaration(wireSpeed, ProcessEventValue.Analog(0.0), WeldChannelPolicy.WIRE_SPEED_KEEPS_ON_STOP),
      new ProcessChannelDeclaration(voltage, ProcessEventValue.Analog(0.0), WeldChannelPolicy.VOLTAGE_KEEPS_ON_STOP)
    ];
}
