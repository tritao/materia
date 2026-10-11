package processkit.screwdriving;

import robotkit.execution.FiredProcessEvent;
import robotkit.execution.ProcessEventValue;
/** Channel names derived from the fitted screwdriver. */
typedef ScrewdriverChannelNames = {start:String, direction:String, program:String, vacuum:String};

/** The same process boundary for the simulated spindle and a hardware controller. */
class ScrewdriverProcessDevice implements processkit.ProcessDevice {
  final outputs:ScrewdriverOutputs;
  final feedback:ScrewdriverFeedback;
  final channels:ScrewdriverChannelNames;
  final slot:Int;
  final status:Null<ScrewdriverDeviceStatus>;
  var prepared = false;
  public function new(outputs:ScrewdriverOutputs, feedback:ScrewdriverFeedback, channels:ScrewdriverChannelNames, slot:Int,
      status:Null<ScrewdriverDeviceStatus> = null) {
    if (outputs == null || feedback == null || channels == null || slot <= 0) throw "Screwdriver device needs outputs, feedback, channels and a program slot";
    var names = [channels.start, channels.direction, channels.program, channels.vacuum];
    for (i in 0...names.length) {
      if (names[i] == null || StringTools.trim(names[i]).length == 0) throw "Screwdriver device needs channel names";
      for (j in 0...i) if (names[i] == names[j]) throw "Screwdriver device needs distinct channels";
    }
    this.outputs = outputs; this.feedback = feedback; this.channels = channels; this.slot = slot; this.status = status;
  }
  public function prepare():Void {
    outputs.start(false); outputs.reverse(false); outputs.selectProgram(slot); prepared = true;
  }
  public function ready():Bool return prepared && fault() == null && (status == null || status.ready());
  public function fault():Null<String> {
    var detail = status == null ? null : status.faultDetail();
    return detail == null ? ScrewdriverSensor.faultMessage(feedback.reading().result) : detail;
  }
  /** A controlled process stop retains the held screw; e-stop policy is declared by the tool channels. */
  public function safe():Void { outputs.start(false); outputs.reverse(false); prepared = false; }
  /** Decode an analog selection without silently truncating it to a signed Haxe Int. */
  public static function programSlot(value:Float, allowIdle:Bool = false):Int {
    if (!Math.isFinite(value) || value < (allowIdle ? 0 : 1) || value > 2147483647 || value != Math.floor(value))
      throw "Screwdriver selection needs a representable integer program slot";
    return Std.int(value);
  }
  public function apply(records:Array<FiredProcessEvent>):Void {
    for (record in records) {
      if (record.channel == channels.program) switch record.value {
        case Analog(value):
          outputs.selectProgram(programSlot(value));
        case _: throw "Screwdriver program needs an analog slot";
      } else {
        var on = switch record.value {case Digital(value): value; case _: throw "Screwdriver outputs need digital events";};
        if (record.channel == channels.start) outputs.start(on);
        else if (record.channel == channels.direction) outputs.reverse(on);
        else if (record.channel == channels.vacuum) outputs.vacuum(on);
        else throw 'Screwdriver has no channel "${record.channel}"';
      }
    }
  }
}
