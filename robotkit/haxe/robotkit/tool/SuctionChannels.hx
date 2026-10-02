package robotkit.tool;

import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;

/**
 * The channel a suction tool is worked by, declared on the robot with its safe value and stop policy. It keeps its output
 * through a commanded stop or an aborted motion, so a robot that stops (or whose base arrives somewhere) does not drop
 * what it holds; a fault or an emergency stop still takes it to off. The opposite of a welder's (`WeldChannels`).
 */
class SuctionChannels {
  public static inline var KEEPS_ON_STOP:Bool = true;

  public static function declaration(channel:String):ProcessChannelDeclaration
    return new ProcessChannelDeclaration(channel, ProcessEventValue.Digital(false), KEEPS_ON_STOP);
}
