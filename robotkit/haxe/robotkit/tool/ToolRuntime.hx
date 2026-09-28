package robotkit.tool;

/**
 * One mounted tool and its independently controlled capabilities. A physical
 * end effector can provide both a gripper and vacuum pickup at the same TCP.
 */
class ToolRuntime {
  public final tool:Tool;
  public final gripper:Null<Gripper>;
  public final vacuum:Null<Vacuum>;
  public final surface:Null<SurfaceTool>;
  public final sprayer:Null<Sprayer>;
  public final sander:Null<Sander>;

  public function new(tool:Tool, ?gripper:Gripper, ?vacuum:Vacuum,
      ?surface:SurfaceTool, ?sprayer:Sprayer, ?sander:Sander) {
    if (tool == null) throw "Tool runtime requires a mounted tool";
    this.tool = tool;
    this.gripper = gripper;
    this.vacuum = vacuum;
    this.surface = surface;
    this.sprayer = sprayer;
    this.sander = sander;
  }
}
