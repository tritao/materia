package motionkit.robot;

/** Validated plan blocks and nonfatal planning notes. Owns its plans. */
class CompiledProgram {
  public final blocks:Array<ProgramBlock>;
  public final notes:Array<String>;
  var ownsPlans:Bool=true;

  public function new(blocks:Array<ProgramBlock>, notes:Array<String>) {
    this.blocks = blocks.copy();
    this.notes = notes.copy();
  }

  /** Transfer plan ownership once to the execution queue. */
  public function takeBlocks():Array<ProgramBlock> {
    if(!ownsPlans)throw "Compiled program plans were already transferred or disposed";
    ownsPlans=false;return blocks.copy();
  }

  public function dispose():Void {
    if(!ownsPlans)return;
    ownsPlans=false;
    for (block in blocks)
      for (plan in block.plans) plan.dispose();
  }
}
