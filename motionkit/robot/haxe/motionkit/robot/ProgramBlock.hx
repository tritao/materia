package motionkit.robot;

import motionkit.trajectory.ExecutionPlan;

/** Exact-stop plans followed by an optional host-side barrier. */
class ProgramBlock {
  public final plans:Array<ExecutionPlan>;
  public final opIndices:Array<Int>;
  public final pathLengths:Array<Float>;
  public final pathDistances:Array<Array<Float>>;
  public final pathTimes:Array<Array<Float>>;
  public final barrier:Null<ProgramBarrier>;

  public function new(plans:Array<ExecutionPlan>, opIndices:Array<Int>,
      barrier:Null<ProgramBarrier>, ?pathLengths:Array<Float>,
      ?pathDistances:Array<Array<Float>>, ?pathTimes:Array<Array<Float>>) {
    this.plans = plans.copy();
    this.opIndices = opIndices.copy();
    this.pathLengths = pathLengths == null ? [for (_ in plans) 0.0] : pathLengths.copy();
    this.pathDistances = pathDistances == null ? [for (_ in plans) []] :
      [for (values in pathDistances) values.copy()];
    this.pathTimes = pathTimes == null ? [for (_ in plans) []] :
      [for (values in pathTimes) values.copy()];
    this.barrier = barrier;
  }
}
