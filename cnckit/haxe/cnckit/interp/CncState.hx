package cnckit.interp;

import cnckit.CncMachine;
import cnckit.parse.CncSpan;

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
  public var coolantMist:Bool = false;
  public var coolantFlood:Bool = false;
  public var selectedTool:Int = -1;
  public var activeTool:Int = -1;
  public var cutterSide:Int = 0;
  public var motionMode:Int = -1;
  public var plane:Int = 17;
  public var cycleCode:Int = 0;
  public var cycleDepth:Float = Math.NaN;
  public var cycleR:Float = Math.NaN;
  public var cycleQ:Float = Math.NaN;
  public var cycleP:Float = Math.NaN;
  public var cycleInitialZ:Float = Math.NaN;
  public var cycleSpan:Null<CncSpan> = null;
  public var retractToInitial:Bool = true;
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
    result.coolantMist = coolantMist; result.coolantFlood = coolantFlood;
    result.selectedTool = selectedTool; result.motionMode = motionMode;
    result.activeTool = activeTool; result.cutterSide = cutterSide;
    result.plane = plane;
    result.cycleCode = cycleCode; result.cycleDepth = cycleDepth;
    result.cycleR = cycleR; result.cycleQ = cycleQ; result.cycleP = cycleP;
    result.cycleInitialZ = cycleInitialZ;
    result.cycleSpan = cycleSpan;
    result.retractToInitial = retractToInitial;
    result.blendTolerance = blendTolerance; result.position = position.copy();
    result.ended = ended;
    return result;
  }
}
