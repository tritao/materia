package motionkit.robot;

import motionkit.trajectory.ExecutionPlan;

/**
  Exact-stop plans followed by an optional host-side barrier. A block being
  planned is open: plans are added as they are made, and it is complete once
  its barrier (or the program's end) is reached.
**/
class ProgramBlock {
  public final plans:Array<ExecutionPlan>;
  public final opIndices:Array<Int>;
  public final pathLengths:Array<Float>;
  public final pathDistances:Array<Array<Float>>;
  public final pathTimes:Array<Array<Float>>;
  public var barrier(default, null):Null<ProgramBarrier>;
  public var complete(default, null):Bool;

  public function new(plans:Array<ExecutionPlan>, opIndices:Array<Int>,
      barrier:Null<ProgramBarrier>, ?pathLengths:Array<Float>,
      ?pathDistances:Array<Array<Float>>, ?pathTimes:Array<Array<Float>>,
      ?complete:Bool = true) {
    this.plans = plans.copy();
    this.opIndices = opIndices.copy();
    this.pathLengths = pathLengths == null ? [for (_ in plans) 0.0] : pathLengths.copy();
    this.pathDistances = pathDistances == null ? [for (_ in plans) []] :
      [for (values in pathDistances) values.copy()];
    this.pathTimes = pathTimes == null ? [for (_ in plans) []] :
      [for (values in pathTimes) values.copy()];
    this.barrier = barrier;
    this.complete = complete;
  }

  /** An open block, to be filled by `add` and finished by `close`. */
  public static function open():ProgramBlock return new ProgramBlock([], [], null, [], [], [], false);

  public function add(plan:ExecutionPlan, opIndex:Int, length:Float, distances:Array<Float>,
      times:Array<Float>):Void {
    if (complete) throw "Program block is already complete";
    plans.push(plan);
    opIndices.push(opIndex);
    pathLengths.push(length);
    pathDistances.push(distances);
    pathTimes.push(times);
  }

  public function close(barrier:Null<ProgramBarrier>):Void {
    if (complete) throw "Program block is already complete";
    this.barrier = barrier;
    complete = true;
  }
}
