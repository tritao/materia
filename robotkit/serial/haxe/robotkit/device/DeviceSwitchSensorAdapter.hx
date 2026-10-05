package robotkit.device;

import haxe.Int64;
import robotkit.device.DeviceBinding.BoundInput;
import robotkit.runtime.DeviceInputReading;
import robotkit.runtime.NativeRuntimeEndpoint;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;

/** Publish physical device switch captures on the runtime's external sensor stream. */
class DeviceSwitchSensorAdapter {
  final runtime:RobotRuntime;
  final inputs:Array<BoundInput>;
  final leaders:Array<Int> = [];
  final read:String->Null<DeviceInputReading>;
  final timestamps:Array<Null<Int64>> = [];
  final counts:Array<Int64> = [];
  var sequence:Int64 = Int64.ofInt(0);

  public function new(blueprint:RobotRuntimeBlueprint, runtime:RobotRuntime, binding:DeviceBinding,
      ?read:String->Null<DeviceInputReading>) {
    if (blueprint == null || runtime == null || binding == null) throw "Device switch adapter requires blueprint, runtime and binding";
    this.runtime = runtime;
    inputs = binding.inputs.copy();
    if (read != null) this.read = read;
    else {
      if (!Std.isOfType(runtime.endpoint, NativeRuntimeEndpoint)) throw "Device switches require a native endpoint";
      var endpoint:NativeRuntimeEndpoint = cast runtime.endpoint;
      this.read = id -> endpoint.deviceInput(id);
    }
    for (input in inputs) {
      var mounted = false;
      for (sensor in blueprint.sensors) if (sensor.id == input.wiring.switchId && sensor.kind == "trip_switch") mounted = true;
      if (!mounted) throw "Device switch has no external sensor mount";
      leaders.push(binding.channels[input.actuatorChannel].jointIndex);
      timestamps.push(null);
      counts.push(Int64.ofInt(0));
    }
  }

  /** Called after copying joint state; never advances the device clock. */
  public function poll(snapshot:robotkit.runtime.RobotSnapshot):Void {
    for (index in 0...inputs.length) {
      var input = inputs[index];
      var observed = read(input.wiring.switchId);
      if (observed == null) continue;
      if (Int64.compare(observed.sourceTimestampNs, snapshot.sourceTimestampNs) > 0) continue;
      var previous = timestamps[index];
      if (previous != null && Int64.compare(observed.sourceTimestampNs, previous) <= 0) continue;
      if (Int64.compare(observed.closingCount, counts[index]) < 0 ||
          Int64.compare(observed.closingCount, Int64.ofInt(2147483647)) > 0 ||
          Int64.compare(observed.closingCount, Int64.ofInt(0)) < 0 ||
          Int64.compare(observed.capturedTimestampNs, observed.sourceTimestampNs) > 0)
        throw "Device switch capture counter or timestamp is invalid";
      var count = Int64.toInt(observed.closingCount);
      var edge = count == 0 ? 0.0 : observed.capturedPosition - runtime.referenceOffset(leaders[index]);
      sequence = Int64.add(sequence, Int64.ofInt(1));
      runtime.publishSensorFrame(input.wiring.switchId,
        [observed.active ? 1.0 : 0.0, count == 0 ? 0.0 : 1.0, edge, count], sequence,
        observed.sourceTimestampNs, runtime.sourceClockId, null);
      timestamps[index] = observed.sourceTimestampNs;
      counts[index] = observed.closingCount;
    }
  }
}
