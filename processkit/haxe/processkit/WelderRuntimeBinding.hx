package processkit;

import haxe.Int64;
import robotkit.runtime.RobotRuntime;
import robotkit.core.SensorFrame;
import processkit.tool.WeldSensor;

/** Process-owned bridge from executed channels and supply feedback to the authored tool sensor. */
class WelderRuntimeBinding {
  public final runtime:RobotRuntime;
  public final binding:WelderChannelBinding;
  public final sensorId:String;
  public final sourceClockId:String;
  var sequence:Int64 = Int64.ofInt(0);
  var previousTimestamp:Null<Int64> = null;

  public function new(runtime:RobotRuntime, binding:WelderChannelBinding, sensorId:String, sourceClockId:String) {
    if (runtime == null || binding == null || sensorId == null || StringTools.trim(sensorId).length == 0 ||
        sourceClockId == null || StringTools.trim(sourceClockId).length == 0)
      throw "Welder runtime binding needs a runtime, supply, authored sensor and source clock";
    this.runtime = runtime; this.binding = binding; this.sensorId = sensorId; this.sourceClockId = sourceClockId;
  }

  /** Poll time belongs to the transport owner; source time identifies the published observation. */
  public function update(now:Float, sourceTimestampNs:Int64):SensorFrame {
    try {
      if (!Math.isFinite(now) || now < 0.0 || Int64.compare(sourceTimestampNs, Int64.ofInt(0)) < 0 ||
          (previousTimestamp != null && Int64.compare(sourceTimestampNs, previousTimestamp) < 0))
        throw "Welder observation clocks must be finite, nonnegative and monotonic";
      var reading = binding.update(now, runtime.channelValue);
      var next = Int64.add(sequence, Int64.ofInt(1));
      var frame = runtime.publishSensorFrameAndGet(sensorId, WeldSensor.values(reading), next,
        sourceTimestampNs, sourceClockId);
      sequence = next; previousTimestamp = sourceTimestampNs;
      return frame;
    } catch (error:Dynamic) {
      binding.safe(); throw error;
    }
  }

  /** Call on owner stop/abort and keep polling until shutdown is acknowledged or the device lease expires. */
  public function safe():Void binding.safe();
}
