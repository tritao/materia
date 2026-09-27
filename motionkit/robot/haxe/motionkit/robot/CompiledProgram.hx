package motionkit.robot;

/** Validated plan blocks and nonfatal planning notes. Owns its plans. */
class CompiledProgram {
  public final blocks:Array<ProgramBlock>;
  public final notes:Array<String>;

  public function new(blocks:Array<ProgramBlock>, notes:Array<String>) {
    this.blocks = blocks.copy();
    this.notes = notes.copy();
  }

  public function dispose():Void {
    for (block in blocks)
      for (plan in block.plans) plan.dispose();
  }
}
