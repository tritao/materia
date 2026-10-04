package processkit;

import robotkit.execution.ProcessEventValue;
import processkit.WelderProcessDevice.WelderChannels;
import processkit.tool.WeldSensor;
import processkit.tool.WeldSensor.WeldReading;

/** Binds executed robot process channels to a supply without changing the authored weld program. */
class WelderChannelBinding {
  public final channels:WelderChannels;
  final outputs:WelderOutputs;
  final feedback:WelderFeedback;
  final pollDevice:Float->Void;
  var inhibited = false;
  var previousArc:Null<Bool> = null;
  var previousWire:Null<Float> = null;
  var previousVoltage:Null<Float> = null;

  public function new(channels:WelderChannels, outputs:WelderOutputs, feedback:WelderFeedback, pollDevice:Float->Void) {
    if (channels == null || outputs == null || feedback == null || pollDevice == null)
      throw "Welder binding needs channels, outputs, feedback and a device poll";
    processkit.tool.WeldContract.declarations(channels.arc, channels.wireSpeed, channels.voltage);
    this.channels = channels; this.outputs = outputs; this.feedback = feedback; this.pollDevice = pollDevice;
  }

  /** Called on the process owner's clock, including while engagement or shutdown is waiting. */
  public function update(now:Float, read:String->ProcessEventValue):WeldReading {
    try return updateChannels(now, read) catch (error:Dynamic) {
      safe(); throw error;
    }
  }

  function updateChannels(now:Float, read:String->ProcessEventValue):WeldReading {
    var arc = switch read(channels.arc) {
      case Digital(value): value;
      case _: throw "Welder arc channel must be digital";
    };
    var wire = analog(read(channels.wireSpeed));
    if (inhibited) {
      if (!arc) inhibited = false;
      else { arc = false; wire = 0.0; }
    }
    var voltage = analog(read(channels.voltage));
    if (previousVoltage == null || previousVoltage != voltage) {
      outputs.setVoltage(voltage); previousVoltage = voltage;
    }
    if (previousWire == null || previousWire != wire) {
      outputs.setWireSpeed(wire); previousWire = wire;
    }
    // Only channel transitions write the arc: repeated off samples must not cancel queued feedback reads.
    if (previousArc == null || previousArc != arc) { outputs.setArc(arc); previousArc = arc; }
    pollDevice(now);
    var reading = feedback.reading();
    if (!WeldSensor.valid(WeldSensor.values(reading))) throw "Welder binding received malformed feedback";
    if (reading.fault != 0) safe();
    return reading;
  }

  public function safe():Void {
    inhibited = true;
    outputs.setWireSpeed(0.0); outputs.setArc(false);
    previousWire = null; previousArc = null; previousVoltage = null;
  }

  static function analog(value:ProcessEventValue):Float return switch value {
    case Analog(number):
      if (!Math.isFinite(number) || number < 0.0) throw "Welder setpoint must be finite and nonnegative";
      number;
    case _: throw "Welder setpoint channel must be analog";
  };
}
