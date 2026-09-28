package robotkit.tool;

import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;

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
  public final adapter:ChannelToolAdapter = new ChannelToolAdapter();
  final declaredChannels:Array<ProcessChannelDeclaration> = [];

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

  /** Bind a digital close/open channel; false is the safe open state. */
  public function bindGripper(channel:String):Void {
    if (gripper == null) throw "Mounted tool has no gripper";
    var declaration = new ProcessChannelDeclaration(channel, ProcessEventValue.Digital(false));
    adapter.bindGripper(channel, gripper);
    declaredChannels.push(declaration);
  }

  /** Bind a digital vacuum channel; false is the safe released state. */
  public function bindVacuum(channel:String):Void {
    if (vacuum == null) throw "Mounted tool has no vacuum";
    var declaration = new ProcessChannelDeclaration(channel, ProcessEventValue.Digital(false));
    adapter.bindVacuum(channel, vacuum);
    declaredChannels.push(declaration);
  }

  public function channelDeclarations():Array<ProcessChannelDeclaration>
    return declaredChannels.copy();
}
