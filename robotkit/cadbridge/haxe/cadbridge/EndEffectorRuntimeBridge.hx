package cadbridge;

import cadkit.modeling.AssemblyState;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.RuntimePortIntent;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
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
      }
    }
    var tool = EndEffectorBridge.toTool(configuration, frameName, state,
      configurationId + "/" + frameName);
    var runtime = new ToolRuntime(tool, gripper, vacuum, null, null, null, changerLock);
    for (control in controls) switch control {
      case Gripper(channel, _, _, _): runtime.bindGripper(channel);
      case Vacuum(channel, _, _): runtime.bindVacuum(channel);
      case Lock(channel, _, _): runtime.bindChangerLock(channel);
    }
    return runtime;
  }

  /** Build control channels and pressure feedback from component port intents. */
  public static function deriveBindings(set:EndEffectorSet,
      configurationId:String):EndEffectorRuntimeBindings {
    if (set == null) throw "End effector set is required";
    var configuration = set.configuration(configurationId);
    var controls:Array<EndEffectorControlBinding> = [];
    var sensorId:Null<String> = null;
    var hasGripper = false, hasVacuum = false, hasLock = false;
    var hasExplicitVacuumValve = false;
    for (member in configuration.components()) for (intent in member.component.runtimePortIntents())
      switch intent {
        case VacuumValve(_): hasExplicitVacuumValve = true;
        case _:
      }
    for (member in configuration.components()) for (intent in member.component.runtimePortIntents()) {
      var prefix = '$configurationId/${member.id}';
      switch intent {
        case Gripper(openPort, closePort):
          if (hasGripper || openPort == closePort) throw "Ambiguous gripper runtime ports";
          requireInlet(configuration, member.id, openPort, [PortKind.Pneumatic, PortKind.Signal]);
          requireInlet(configuration, member.id, closePort, [PortKind.Pneumatic, PortKind.Signal]);
          controls.push(EndEffectorControlBinding.Gripper('$prefix.close', member.id,
            openPort, closePort));
          hasGripper = true;
        case VacuumActuator(inletPort):
          if (hasExplicitVacuumValve) continue;
          if (hasVacuum) throw "Ambiguous vacuum runtime ports";
          requireInlet(configuration, member.id, inletPort, [PortKind.Pneumatic, PortKind.Signal]);
          controls.push(EndEffectorControlBinding.Vacuum('$prefix.enable', member.id, inletPort));
          hasVacuum = true;
        case VacuumValve(controlPort):
          if (hasVacuum) throw "Ambiguous vacuum runtime ports";
          requireInlet(configuration, member.id, controlPort, [PortKind.Signal]);
          controls.push(EndEffectorControlBinding.Vacuum('$prefix.enable', member.id, controlPort));
          hasVacuum = true;
        case ChangerLock(inletPort):
          if (hasLock) throw "Ambiguous changer-lock runtime ports";
          requireInlet(configuration, member.id, inletPort, [PortKind.Pneumatic]);
          controls.push(EndEffectorControlBinding.Lock('$prefix.lock', member.id, inletPort));
          hasLock = true;
        case VacuumPressureSensor(vacuumPort, signalPort):
          if (sensorId != null) throw "Ambiguous vacuum pressure sensors";
          requireInlet(configuration, member.id, vacuumPort, [PortKind.Vacuum]);
          var signal = member.component.port(signalPort);
          if (signal.role != PortRole.Supply || signal.kind != PortKind.Signal)
            throw 'Pressure sensor "$prefix" needs a signal supply';
          sensorId = '$prefix.$signalPort';
      }
    }
    if (sensorId != null && !hasVacuum)
      throw "Vacuum pressure sensor has no bound vacuum actuator";
    return {controls: controls, vacuumSensorId: sensorId};
  }

  /** Build a runtime using only declarations on the coupled components. */
  public static function toRuntimeFromDesign(set:EndEffectorSet, configurationId:String,
      frameName:String, ?state:AssemblyState):EndEffectorRuntimeBundle {
    var bindings = deriveBindings(set, configurationId);
    return {runtime: toRuntime(set, configurationId, frameName, bindings.controls, state),
      bindings: bindings};
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
