package processkit.screwdriving;

import processkit.screwdriving.ScrewdriverProcessDevice.ScrewdriverChannelNames;
import robotkit.execution.ProcessEventValue;
/** Writes the declared outputs without consuming the controller's event stream. */
class ChannelScrewdriverOutputs implements ScrewdriverOutputs {
  final runtime:robotkit.runtime.RobotRuntime;
  final channels:ScrewdriverChannelNames;
  public function new(runtime:robotkit.runtime.RobotRuntime, channels:ScrewdriverChannelNames) {
    this.runtime = runtime; this.channels = channels;
  }
  public function selectProgram(slot:Int):Void {
    if (slot <= 0) throw "Screwdriver needs a positive program slot";
    runtime.writeOutput(channels.program, Analog(slot));
  }
  public function start(on:Bool):Void runtime.writeOutput(channels.start, Digital(on));
  public function reverse(on:Bool):Void runtime.writeOutput(channels.direction, Digital(on));
  public function vacuum(on:Bool):Void runtime.writeOutput(channels.vacuum, Digital(on));
}
