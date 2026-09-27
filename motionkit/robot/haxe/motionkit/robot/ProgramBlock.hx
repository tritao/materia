package motionkit.robot;

import motionkit.trajectory.ExecutionPlan;

/** Exact-stop plans followed by an optional host-side barrier. */
class ProgramBlock {
  public final plans:Array<ExecutionPlan>;
  public final opIndices:Array<Int>;
  public final barrier:Null<ProgramBarrier>;

  public function new(plans:Array<ExecutionPlan>, opIndices:Array<Int>,
      barrier:Null<ProgramBarrier>) {
    this.plans = plans.copy();
    this.opIndices = opIndices.copy();
    this.barrier = barrier;
  }
}
