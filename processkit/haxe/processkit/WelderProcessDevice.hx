package processkit;

import robotkit.tool.WeldFault;
import robotkit.tool.WeldSensor;
import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventValue;

/** The names of the channels a welder is worked by, as the end effector derives them. */
typedef WelderChannels = {arc:String, wireSpeed:String, voltage:String};

/** The setpoint `prepare` writes: the voltage, in volts. The wire speed comes with the process records. */
typedef WelderSetpoints = {voltage:Float};

/**
 * The `ProcessDevice` for a welder. It speaks only the welder's channels and its `tool_weld` reading, so the
 * simulated welder (RobotKit's `SimulatedWelder`) and a real one (a retrofit I/O board, or a supply on Modbus)
 * take its place without the process knowing.
 * - `prepare` sets the voltage setpoint and holds the wire still and the arc off;
 * - `ready` means the device was prepared and the supply reports no fault;
 * - `fault` is the supply's fault, in words;
 * - `safe` switches the arc off, then stops the wire, and leaves the device to be prepared again;
 * - `apply` carries out the fired process records: the arc channel takes a digital value or an analogue rate
 *   (above zero is on, as a process run emits it), the other two take analogue values.
 */
class WelderProcessDevice implements ProcessDevice {
  public final channels:WelderChannels;
  public final setpoints:WelderSetpoints;
  final outputs:WelderOutputs;
  final feedback:WelderFeedback;
  var prepared = false;

  public function new(outputs:WelderOutputs, feedback:WelderFeedback, channels:WelderChannels, setpoints:WelderSetpoints) {
    if (outputs == null || feedback == null || channels == null || setpoints == null)
      throw "Welder process device needs outputs, feedback, channels and setpoints";
    for (name in [channels.arc, channels.wireSpeed, channels.voltage])
      if (name == null || StringTools.trim(name).length == 0) throw "Welder process device needs three channel names";
    if (channels.arc == channels.wireSpeed || channels.arc == channels.voltage || channels.wireSpeed == channels.voltage)
      throw "Welder channels must be distinct";
    if (!Math.isFinite(setpoints.voltage) || setpoints.voltage <= 0.0) throw "Welder setpoints need a positive voltage";
    this.outputs = outputs;
    this.feedback = feedback;
    this.channels = channels;
    this.setpoints = setpoints;
  }

  public function prepare():Void {
    outputs.setArc(false);
    outputs.setWireSpeed(0.0);
    outputs.setVoltage(setpoints.voltage);
    prepared = true;
  }

  public function ready():Bool return prepared && feedback.reading().fault == WeldFault.None;

  public function fault():Null<String> return WeldSensor.faultMessage(feedback.reading().fault);

  public function safe():Void {
    prepared = false;
    outputs.setArc(false);
    outputs.setWireSpeed(0.0);
  }

  public function apply(records:Array<FiredProcessEvent>):Void {
    for (record in records) {
      var value = record.value;
      if (record.channel == channels.arc) outputs.setArc(switch value {
        case Digital(on): on;
        case Analog(rate): rate > 0.0;
        case Process(_, _): throw "The arc channel needs a digital or analog event";
      });
      else if (record.channel == channels.wireSpeed) outputs.setWireSpeed(analog(value, record.channel));
      else if (record.channel == channels.voltage) outputs.setVoltage(analog(value, record.channel));
      else throw 'The welder has no channel ${record.channel}';
    }
  }

  static function analog(value:ProcessEventValue, channel:String):Float
    return switch value {
      case Analog(number): number;
      case Digital(_), Process(_, _): throw 'Welder channel $channel needs an analog event';
    };
}
