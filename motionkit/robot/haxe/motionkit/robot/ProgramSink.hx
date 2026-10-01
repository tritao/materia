package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;

/** Receives what a ProgramCompilation plans, in program order. */
interface ProgramSink {
  /** A finished plan of op `opIndex`; the sink owns it from here. */
  function plan(plan:ExecutionPlan, opIndex:Int, length:Float, distances:Array<Float>,
    times:Array<Float>):Void;

  /** The current block ends at `barrier`; later plans start the next block. */
  function barrier(barrier:ProgramBarrier):Void;

  /** A nonfatal planning note. */
  function note(text:String):Void;

  /** The whole program is planned, its last block ends, and `nextPlanId` is the next unused plan id. */
  function finish(nextPlanId:Int64):Void;
}
