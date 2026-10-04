package processkit.simulation;

import haxe.Int64;
import robotkit.runtime.Simulation;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.VirtualDeviceOptions;
import processkit.tool.WeldSensor;
import processkit.tool.WeldSensor.WeldReading;
import processkit.WelderProcessDevice.WelderChannels;

/** ProcessKit owns the board welding profile and maps its numeric samples to the authored sensor. */
class VirtualWelderSupply implements SimulationWelderSupply {
  final simulation:Simulation;
  final runtime:RobotRuntime;
  final robotIndex:Int;
  final sensorId:String;
  final slot:Int;
  var sequence:Int64 = Int64.ofInt(0);
  var publicationSequence:Int64 = Int64.ofInt(0);
  var epoch:Int = 0;
  var latest:WeldReading = {arc:false, currentA:0.0, voltageV:0.0, touch:false, fault:0, powerW:0.0};

  public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, sensorId:String, slot:Int = 0) {
    this.simulation = simulation; this.runtime = runtime; this.robotIndex = robotIndex; this.sensorId = sensorId; this.slot = slot;
  }

  public static function configure(options:VirtualDeviceOptions, blueprint:RobotRuntimeBlueprint,
      channels:WelderChannels, efficiency:Float, slot:Int = 0):Void {
    var indices:Array<Float> = [];
    for (name in [channels.arc, channels.wireSpeed, channels.voltage]) {
      var found = -1;
      for (i in 0...blueprint.channels.length) if (blueprint.channels[i].id == name) found = i;
      if (found < 0) throw 'Virtual welder channel "$name" is undeclared';
      indices.push(found);
    }
    options.peripheralKind = 1;
    options.externalSensorSlots = [slot];
    options.peripheralParameters = [slot, indices[0], indices[1], indices[2], 0.02, 0.2, efficiency];
  }

  public function observe(dt:Float, timestamp:Int64, grounded:Bool):WeldReading {
    simulation.setVirtualDeviceInput(robotIndex, 0, grounded ? 1.0 : 0.0);
    var value = simulation.virtualDeviceSensor(robotIndex, slot);
    var sample = value.get_sample();
    if (Int64.compare(sample.get_sequence(), sequence) > 0) {
      var values = [for (i in 0...sample.get_value_count()) value.get_values(i)];
      if (!WeldSensor.valid(values)) throw "Malformed virtual welding sensor";
      publicationSequence = Int64.add(publicationSequence, Int64.ofInt(1));
      runtime.publishSensorFrame(sensorId, values, publicationSequence, sample.get_source_timestamp_ns(), "robotkit.device." + epoch);
      sequence = sample.get_sequence(); latest = WeldSensor.reading(values);
    }
    return latest;
  }
  public function safe():Void simulation.stopVirtualDevice(robotIndex);
  public function reset():Void {
    // The simulation recreates the board; preserve publication ordering across that device epoch.
    sequence = Int64.ofInt(0);
    epoch++;
    latest = {arc:false, currentA:0.0, voltageV:0.0, touch:false, fault:0, powerW:0.0};
  }
}
