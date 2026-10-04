package robotkit.runtime;

import haxe.Int64;
import robotkit.model.RobotModel;
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

  public function new(model:RobotModel, runtime:RobotRuntime, readPositions:Void->Array<Float>, clockId:String) {
    if (model == null || runtime == null || readPositions == null || clockId == null || clockId.length == 0)
      throw "Switch sensor adapter needs a model, runtime, actual-position reader and clock";
    this.runtime = runtime; this.readPositions = readPositions; this.clockId = clockId;
    jointCount = model.joints.length;
    for (contact in model.switches) {
      var index = -1;
      for (i in 0...model.joints.length) if (model.joints[i].id == contact.joint) index = i;
      if (index < 0) throw 'Switch ${contact.id} has no monitored joint';
      var mounted = false;
      for (sensor in model.sensors) if (sensor.id == contact.id && sensor.kind == "joint_switch" &&
          sensor.frame != null && sensor.frame.id == contact.frameId) mounted = true;
      if (!mounted) throw 'Switch ${contact.id} has no matching external sensor mount';
      joints.push(index);
      readings.push(new SwitchReading(contact));
    }
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
      runtime.publishSensorFrame(value.source.id, [active ? 1.0 : 0.0], sequence,
        sourceTimestampNs, clockId, null);
    }
  }
}
