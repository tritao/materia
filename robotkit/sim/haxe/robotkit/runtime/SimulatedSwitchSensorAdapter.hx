package robotkit.runtime;

import haxe.Int64;
import robotkit.model.SwitchReading;

/** Publish mechanical switch observations after each simulation tick.
 * The reader supplies actual joint coordinates, including any simulated slip. */
class SimulatedSwitchSensorAdapter implements SimulationStepObserver {
  final runtime:RobotRuntime;
  final readPositions:Void->Array<Float>;
  final clockId:String;
  final readings:Array<SwitchReading> = [];
  final joints:Array<Int> = [];
  final jointCount:Int;
  var sequence:Int64 = Int64.ofInt(0);

  public function new(blueprint:RobotRuntimeBlueprint, runtime:RobotRuntime, readPositions:Void->Array<Float>, clockId:String) {
    if (blueprint == null || blueprint.identity == null || runtime == null || readPositions == null || clockId == null || clockId.length == 0)
      throw "Switch sensor adapter needs a model, runtime, actual-position reader and clock";
    this.runtime = runtime; this.readPositions = readPositions; this.clockId = clockId;
    jointCount = blueprint.jointCount;
    var indices = bindingIndices(blueprint);
    for (index in indices) joints.push(index);
    for (contact in blueprint.switches) readings.push(new SwitchReading(contact));
  }

  /** Validate before creating a native robot, so bad mounts cannot leave a partial addition. */
  public static function bindingIndices(blueprint:RobotRuntimeBlueprint):Array<Int> {
    if (blueprint == null || blueprint.identity == null) throw "Switches require a semantic runtime identity";
    var indices:Array<Int> = [], ids = new Map<String, Bool>();
    for (contact in blueprint.switches) {
      if (contact == null || ids.exists(contact.id)) throw "Switch layout has a null or duplicate switch";
      ids.set(contact.id, true);
      var index = -1;
      for (i in 0...blueprint.jointCount) if (blueprint.identity.jointId(i) == contact.joint) index = i;
      if (index < 0) throw 'Switch ${contact.id} has no monitored joint';
      var mounted = false;
      for (sensor in blueprint.sensors) if (sensor.id == contact.id && sensor.kind == "joint_switch" &&
          sensor.frameId == contact.frameId) mounted = true;
      if (!mounted) throw 'Switch ${contact.id} has no matching external sensor mount';
      indices.push(index);
    }
    return indices;
  }

  /** Reset mechanical history and deterministic repeatability after a robot power cycle.
   * Publication sequence stays monotonic for existing sensor stream consumers. */
  public function reset():Void {
    for (i in 0...readings.length) readings[i] = new SwitchReading(readings[i].source);
  }

  public function reading(id:String):SwitchReading {
    for (value in readings) if (value.source.id == id) return value;
    throw 'Unknown simulated switch "$id"';
  }

  public function afterSimulationStep(sourceTimestampNs:Int64):Void {
    var positions = readPositions();
    if (positions == null || positions.length != jointCount)
      throw "Switch actual-position reader returned the wrong joint count";
    sequence = Int64.add(sequence, Int64.ofInt(1));
    for (i in 0...readings.length) {
      var value = readings[i];
      var active = value.sample(positions[joints[i]]);
      var edge = value.closingEdgePosition;
      runtime.publishSensorFrame(value.source.id,
        [active ? 1.0 : 0.0, edge == null ? 0.0 : 1.0, edge == null ? 0.0 : edge, value.closingEdges], sequence,
        sourceTimestampNs, clockId, null);
    }
  }
}
