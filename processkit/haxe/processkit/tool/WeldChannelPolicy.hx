package processkit.tool;

import robotkit.tool.*;

/**
 * Whether each of a welder's channels keeps its output through a commanded stop or an aborted motion. The arc is off
 * and the wire stopped at their safe values, and neither keeps its output: a robot that stops must not leave an arc
 * burning or the wire feeding. The voltage setpoint harmlessly keeps its value, so a restarted process need not write
 * it again. Faults and emergency stops take every channel to its safe value regardless.
 */
class WeldChannelPolicy {
  public static inline var ARC_KEEPS_ON_STOP:Bool = false;
  public static inline var WIRE_SPEED_KEEPS_ON_STOP:Bool = false;
  public static inline var VOLTAGE_KEEPS_ON_STOP:Bool = true;
}
