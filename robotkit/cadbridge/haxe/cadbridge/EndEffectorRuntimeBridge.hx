package cadbridge;

import cadkit.modeling.AssemblyState;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import robotkit.tool.SimulatedGripper;
import robotkit.tool.SimulatedVacuum;
import robotkit.tool.ToolRuntime;

/** Build the mounted runtime and its output channels for one coupled EOAT. */
class EndEffectorRuntimeBridge {
  public static function toRuntime(set:EndEffectorSet, configurationId:String,
      frameName:String, controls:Array<EndEffectorControlBinding>, ?state:AssemblyState):ToolRuntime {
    if (set == null || controls == null) throw "End effector set and control bindings are required";
    var configuration = set.configuration(configurationId);
    var gripper:Null<SimulatedGripper> = null;
    var vacuum:Null<SimulatedVacuum> = null;
    for (control in controls) {
      if (control == null) throw "End effector control binding is required";
      switch control {
        case Gripper(_, instanceId, openPort, closePort):
          if (gripper != null) throw "End effector has duplicate gripper bindings";
          if (openPort == closePort) throw "Gripper open and close need distinct ports";
          requireInlet(configuration, instanceId, openPort, [PortKind.Pneumatic, PortKind.Signal]);
          requireInlet(configuration, instanceId, closePort, [PortKind.Pneumatic, PortKind.Signal]);
          gripper = new SimulatedGripper();
        case Vacuum(_, instanceId, inletPort):
          if (vacuum != null) throw "End effector has duplicate vacuum bindings";
          requireInlet(configuration, instanceId, inletPort,
            [PortKind.Pneumatic, PortKind.Vacuum, PortKind.Signal]);
          vacuum = new SimulatedVacuum();
      }
    }
    var tool = EndEffectorBridge.toTool(configuration, frameName, state,
      configurationId + "/" + frameName);
    var runtime = new ToolRuntime(tool, gripper, vacuum);
    for (control in controls) switch control {
      case Gripper(channel, _, _, _): runtime.bindGripper(channel);
      case Vacuum(channel, _, _): runtime.bindVacuum(channel);
    }
    return runtime;
  }

  static function requireInlet(configuration:EndEffector, instanceId:String,
      portName:String, allowed:Array<PortKind>):Void {
    for (member in configuration.components()) if (member.id == instanceId) {
      var port = member.component.port(portName);
      if (port.role != PortRole.Consumer)
        throw 'Control port "$instanceId/$portName" must be a consumer';
      for (kind in allowed) if (port.kind == kind) {
        configuration.upstream(instanceId, portName);
        return;
      }
      throw 'Control port "$instanceId/$portName" has the wrong service kind';
    }
    throw 'Unknown control member "$instanceId"';
  }
}
