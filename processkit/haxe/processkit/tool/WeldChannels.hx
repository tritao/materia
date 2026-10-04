package processkit.tool;

import robotkit.tool.*;

import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;

/**
 * The channels a welding torch is worked by, with their safe values and stop policy (`WeldChannelPolicy`): whatever stops
 * the robot, the arc goes off and the wire stops. Every torch tool a robot is set up with has them, so the runtime
 * enforces it for every way of running the robot.
 */
class WeldChannels implements ToolChannels {
  public final arc:String;
  public final wireSpeed:String;
  public final voltage:String;

  public function new(arc:String, wireSpeed:String, voltage:String) {
    this.arc = arc;
    this.wireSpeed = wireSpeed;
    this.voltage = voltage;
  }

  public function declarations():Array<ProcessChannelDeclaration>
    return [
      new ProcessChannelDeclaration(arc, ProcessEventValue.Digital(false), WeldChannelPolicy.ARC_KEEPS_ON_STOP),
      new ProcessChannelDeclaration(wireSpeed, ProcessEventValue.Analog(0.0), WeldChannelPolicy.WIRE_SPEED_KEEPS_ON_STOP),
      new ProcessChannelDeclaration(voltage, ProcessEventValue.Analog(0.0), WeldChannelPolicy.VOLTAGE_KEEPS_ON_STOP)
    ];
}
