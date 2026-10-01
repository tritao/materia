package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;

/**
  A program's plans gathered into blocks as they are planned. The last block
  stays open until its barrier or the program's end. Owns its plans.
**/
class ProgramBlocks implements ProgramSink {
  public final blocks:Array<ProgramBlock> = [ProgramBlock.open()];
  public final notes:Array<String> = [];
  /** Whether the whole program is planned. */
  public var done(default, null):Bool = false;
  /** The next unused plan id, once `done`. */
  public var nextPlanId(default, null):Null<Int64> = null;

  public function new() {}

  public function plan(plan:ExecutionPlan, opIndex:Int, length:Float, distances:Array<Float>,
      times:Array<Float>):Void
    blocks[blocks.length - 1].add(plan, opIndex, length, distances, times);

  public function barrier(barrier:ProgramBarrier):Void {
    blocks[blocks.length - 1].close(barrier);
    blocks.push(ProgramBlock.open());
  }

  public function note(text:String):Void notes.push(text);

  public function finish(nextPlanId:Int64):Void {
    blocks[blocks.length - 1].close(null);
    this.nextPlanId = nextPlanId;
    done = true;
  }

  /** Whether there is a plan or barrier to execute yet. */
  public function hasWork():Bool
    return done || blocks.length > 1 || blocks[0].plans.length > 0;

  /** The blocks of a finished program, without the empty block its end may close. */
  public function finishedBlocks():Array<ProgramBlock> {
    var result = blocks.copy();
    var last = result[result.length - 1];
    if (last.plans.length == 0 && last.barrier == null) result.pop();
    return result;
  }

  public function dispose():Void
    for (block in blocks) for (plan in block.plans) plan.dispose();
}
