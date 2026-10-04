package robotkit.tool;

import haxe.Int64;
import robotkit.core.RobotSnapshot;
import robotkit.core.SensorFrame;

private typedef ToolFeedbackBinding = {
  var runtime:ToolRuntime;
  var kind:String;
  var lastSequence:Null<Int64>;
}

/** Route simulated contact and vacuum sensor frames to the selected tool. */
class SimulatedToolSensorAdapter {
  final selection:ToolRuntimeSelection;
  final bindings:Map<String, ToolFeedbackBinding> = new Map();

  public function new(selection:ToolRuntimeSelection) {
    if (selection == null) throw "Tool sensor adapter requires a selection";
    this.selection = selection;
  }

  public function bindGripperContact(runtime:ToolRuntime, sensorId:String):Void {
    if (runtime == null || !Std.isOfType(runtime.gripper, SimulatedGripper))
      throw "Contact sensor requires a simulated gripper";
    bind(runtime, sensorId, "tool_contact");
  }

  public function bindVacuumPressure(runtime:ToolRuntime, sensorId:String):Void {
    if (runtime == null || !Std.isOfType(runtime.vacuum, SimulatedVacuum))
      throw "Vacuum sensor requires a simulated vacuum capability";
    bind(runtime, sensorId, "tool_vacuum_kpa");
  }

  function bind(runtime:ToolRuntime, sensorId:String, kind:String):Void {
    if (sensorId == null || sensorId.length == 0) throw "Tool sensor ID is required";
    if (bindings.exists(sensorId)) throw 'Duplicate tool sensor "$sensorId"';
    bindings.set(sensorId, {runtime: runtime, kind: kind, lastSequence: null});
  }

  /** Returns true only when a fresh frame changed the active tool's observation. */
  public function apply(frame:SensorFrame):Bool {
    if (frame == null) throw "Tool sensor frame is required";
    var binding = bindings.get(frame.sensorId);
    if (binding == null || selection.active() != binding.runtime) return false;
    if (frame.kind != binding.kind || frame.image != null || frame.values.length != 1)
      throw 'Tool sensor "${frame.sensorId}" has the wrong payload';
    if (frame.sourceClockId != selection.selectedClockId())
      throw 'Tool sensor "${frame.sensorId}" uses a different source clock from tool selection';
    if (Int64.compare(frame.sourceTimestampNs, selection.selectedAtNs()) < 0 ||
        (binding.lastSequence != null && Int64.compare(frame.sequence, binding.lastSequence) <= 0))
      return false;
    var value = frame.values.get(0);
    switch binding.kind {
      case "tool_contact":
        if (value != 0.0 && value != 1.0)
          throw 'Tool contact "${frame.sensorId}" requires zero or one';
        var gripper:SimulatedGripper = cast binding.runtime.gripper;
        gripper.observeContact(value == 1.0, frame.sourceTimestampNs);
      case "tool_vacuum_kpa":
        var vacuum:SimulatedVacuum = cast binding.runtime.vacuum;
        vacuum.observeVacuumKpa(value, frame.sourceTimestampNs);
      case _: throw "Unsupported tool sensor binding";
    }
    binding.lastSequence = frame.sequence;
    return true;
  }

  public function applySnapshot(snapshot:RobotSnapshot):Int {
    if (snapshot == null) throw "Tool sensor snapshot is required";
    var applied = 0;
    for (frame in snapshot.sensors.toArray()) if (apply(frame)) applied++;
    return applied;
  }
}
