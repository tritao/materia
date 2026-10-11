package robotkit.tool;

import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;

/** A driver stops spinning on any stop; vacuum keeps its screw through a commanded stop. */
class ScrewdriverChannels implements ToolChannels {
  public final start:String;
  public final direction:String;
  public final program:String;
  public final vacuum:String;

  public function new(start:String, direction:String, program:String, vacuum:String) {
    this.start = start; this.direction = direction; this.program = program; this.vacuum = vacuum;
  }

  public function declarations():Array<ProcessChannelDeclaration> return [
    new ProcessChannelDeclaration(start, ProcessEventValue.Digital(false), false),
    new ProcessChannelDeclaration(direction, ProcessEventValue.Digital(false), false),
    new ProcessChannelDeclaration(program, ProcessEventValue.Analog(0), false),
    new ProcessChannelDeclaration(vacuum, ProcessEventValue.Digital(false), SuctionChannels.KEEPS_ON_STOP)
  ];
}
