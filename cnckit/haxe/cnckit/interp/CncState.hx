package cnckit.interp;

import cnckit.CncMachine;

/** Modal state. A block is committed only after its interpretation succeeds. */
class CncState {
  public final machine:CncMachine;
  public var metric:Bool = true;
  public var absolute:Bool = true;
  public var wcs:Int = 54;
  public var toolLength:Float = 0.0;
  public var feedCommand:Float = Math.NaN;
  public var spindleSpeed:Float = 0.0;
  public var spindleDirection:Int = 0;
  public var selectedTool:Int = -1;
  public var motionMode:Int = -1;
  public var blendTolerance:Float = 0.0;
  public var position:Array<Float>;
  public var ended:Bool = false;

  public function new(machine:CncMachine) {
    this.machine = machine;
    position = machine.initialPosition.copy();
  }

  public function copy():CncState {
    var result = new CncState(machine);
    result.metric = metric; result.absolute = absolute; result.wcs = wcs;
    result.toolLength = toolLength; result.feedCommand = feedCommand;
    result.spindleSpeed = spindleSpeed; result.spindleDirection = spindleDirection;
    result.selectedTool = selectedTool; result.motionMode = motionMode;
    result.blendTolerance = blendTolerance; result.position = position.copy();
    result.ended = ended;
    return result;
  }
}
