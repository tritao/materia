package processkit.tool;

import robotkit.tool.*;

import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;

/**
 * The channels a welding torch is worked by, with their safe values and stop policy (`WeldChannelPolicy`): whatever stops
 * the robot, the arc goes off and the wire stops. Every torch tool a robot is set up with has them, so the runtime
 * enforces it for every way of running the robot.
 */
class WeldChannels implements ToolChannels {
  public final arc:String;
  public final wireSpeed:String;
  public final voltage:String;
  public final job:Null<String>;

  public function new(arc:String, wireSpeed:String, voltage:String, job:Null<String> = null) {
    this.arc = arc;
    this.wireSpeed = wireSpeed;
    this.voltage = voltage;
    this.job = job;
  }

  public function declarations():Array<ProcessChannelDeclaration>
    return WeldContract.declarations(arc, wireSpeed, voltage, job);
}
