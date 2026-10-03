package cadbridge;

import cadkit.modeling.AssemblyState;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.ComponentCapability;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import machinekit.robotics.EndEffectorFrames;
import materia.assembly.AssemblyFrames;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
import robotkit.tool.SimulatedGripper;
import robotkit.tool.SimulatedVacuum;
import robotkit.tool.SimulatedChangerLock;
import robotkit.tool.SimulatedToolSensorAdapter;
import robotkit.tool.ToolRuntimeSelection;
import robotkit.tool.ToolRuntime;

typedef EndEffectorRuntimeBindings = {
  var controls:Array<EndEffectorControlBinding>;
  var vacuumSensorId:Null<String>;
}

typedef EndEffectorRuntimeBundle = {
  var runtime:ToolRuntime;
  var bindings:EndEffectorRuntimeBindings;
}

/** Build the mounted runtime and its output channels for one coupled EOAT. */
class EndEffectorRuntimeBridge {
  public static function toRuntime(set:EndEffectorSet, configurationId:String,
      frameName:String, controls:Array<EndEffectorControlBinding>, ?state:AssemblyState):ToolRuntime {
    if (set == null || controls == null) throw "End effector set and control bindings are required";
    var configuration = set.configuration(configurationId);
    var gripper:Null<SimulatedGripper> = null;
    var vacuum:Null<SimulatedVacuum> = null;
    var changerLock:Null<SimulatedChangerLock> = null;
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
        case Lock(_, instanceId, inletPort):
          if (changerLock != null) throw "End effector has duplicate changer-lock bindings";
          requireInlet(configuration, instanceId, inletPort, [PortKind.Pneumatic]);
          changerLock = new SimulatedChangerLock();
        case Arc(_, instanceId, controlPort):
          // The arc has no tool runtime: the robot's welder works it over its channels. The inlet still has to be fed.
          requireInlet(configuration, instanceId, controlPort, [PortKind.Signal]);
      }
    }
    var tool = EndEffectorBridge.toTool(configuration, frameName, state,
      configurationId + "/" + frameName);
    var runtime = new ToolRuntime(tool, gripper, vacuum, null, null, null, changerLock);
    for (control in controls) switch control {
      case Gripper(channel, _, _, _): runtime.bindGripper(channel);
      case Vacuum(channel, _, _): runtime.bindVacuum(channel);
      case Lock(channel, _, _): runtime.bindChangerLock(channel);
      case Arc(_, _, _):
    }
    return runtime;
  }

  /** Build control channels and pressure feedback from component port intents. */
  public static function deriveBindings(set:EndEffectorSet,
      configurationId:String):EndEffectorRuntimeBindings {
    if (set == null) throw "End effector set is required";
    var derived = machinekit.robotics.EndEffectorControls.derive(set.configuration(configurationId), configurationId);
    var controls:Array<EndEffectorControlBinding> = [for (control in derived.controls) switch control {
      case Gripper(channel, member, openPort, closePort): EndEffectorControlBinding.Gripper(channel, member, openPort, closePort);
      case Vacuum(channel, member, inletPort): EndEffectorControlBinding.Vacuum(channel, member, inletPort);
      case Lock(channel, member, inletPort): EndEffectorControlBinding.Lock(channel, member, inletPort);
      case Arc(channel, member, controlPort): EndEffectorControlBinding.Arc(channel, member, controlPort);
    }];
    return {controls: controls, vacuumSensorId: derived.vacuumSensor};
  }

  /** Build a runtime using only declarations on the coupled components. */
  public static function toRuntimeFromDesign(set:EndEffectorSet, configurationId:String,
      frameName:String, ?state:AssemblyState):EndEffectorRuntimeBundle {
    var bindings = deriveBindings(set, configurationId);
    return {runtime: toRuntime(set, configurationId, frameName, bindings.controls, state),
      bindings: bindings};
  }

  /** Add the design's pressure sensor to a robot blueprint before simulation
   * creation. Its mount is expressed relative to the flange link. */
  public static function addVacuumSensorToBlueprint(set:EndEffectorSet,
      configurationId:String, blueprint:RobotRuntimeBlueprint,
      flangeLinkIndex:Int, ?flangeLinkId:String,
      ?state:AssemblyState):RobotRuntimeSensorBlueprint {
    if (blueprint == null || flangeLinkIndex < 0 || flangeLinkIndex >= blueprint.linkCount)
      throw "Vacuum sensor needs a valid robot flange link";
    var bindings = deriveBindings(set, configurationId);
    var sensorId = bindings.vacuumSensorId;
    if (sensorId == null) throw "End effector has no vacuum pressure sensor";
    if (blueprint.sensorById(sensorId) != null)
      throw 'Robot blueprint already has sensor "$sensorId"';
    var configuration = set.configuration(configurationId);
    var solved = configuration.solve(state);
    for (member in configuration.components())
      for (intent in member.component.capabilities()) switch intent {
        case VacuumPressureSensor(_, signalPort):
          var port = member.component.port(signalPort);
          var connector = port.connector == null ? "mount" : port.connector;
          var pose = solved.poses.get(member.id);
          if (pose == null) throw 'Missing pressure sensor pose for "${member.id}"';
          var mountWorld = AssemblyFrames.compose(pose,
            configuration.memberConnectorFrame(member.id, connector));
          var mountRelative = AssemblyFrames.compose(
            AssemblyFrames.inverse(solved.mountWorld), mountWorld);
          var frame = EndEffectorFrames.toRobotFrame(new machinekit.robotics.ConnectorFrame(mountRelative));
          var mappedLinkId = blueprint.identity == null ? null :
            blueprint.identity.linkId(flangeLinkIndex);
          if (flangeLinkId != null && mappedLinkId != null && flangeLinkId != mappedLinkId)
            throw "Pressure sensor flange link ID does not match the robot blueprint";
          var linkId = flangeLinkId != null ? flangeLinkId : mappedLinkId != null ?
            mappedLinkId : flangeLinkIndex == 0 ? "base_link" : null;
          if (linkId == null) throw "Robot blueprint has no flange link identity";
          var sensor = new RobotRuntimeSensorBlueprint(sensorId, "tool_vacuum_kpa",
            sensorId, linkId, flangeLinkIndex,
            [frame.position.x, frame.position.y, frame.position.z],
            [frame.quaternion.x, frame.quaternion.y, frame.quaternion.z,
              frame.quaternion.w]);
          blueprint.sensors.push(sensor);
          return sensor;
        case _:
      }
    throw "End effector pressure sensor intent did not resolve";
  }

  /** Register declared pressure feedback on a selected runtime. */
  public static function bindSensors(selection:ToolRuntimeSelection,
      bundle:EndEffectorRuntimeBundle):SimulatedToolSensorAdapter {
    if (selection == null || bundle == null) throw "Tool selection and runtime are required";
    var adapter = new SimulatedToolSensorAdapter(selection);
    if (bundle.bindings.vacuumSensorId != null)
      adapter.bindVacuumPressure(bundle.runtime, bundle.bindings.vacuumSensorId);
    return adapter;
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
