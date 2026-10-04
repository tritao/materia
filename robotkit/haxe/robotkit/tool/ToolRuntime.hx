package robotkit.tool;

import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;

/**
 * One mounted tool and its independently controlled capabilities. A physical
 * end effector can provide both a gripper and vacuum pickup at the same TCP.
 */
class ToolRuntime {
  public final tool:Tool;
  public final gripper:Null<Gripper>;
  public final vacuum:Null<Vacuum>;
  public final changerLock:Null<ChangerLock>;
  public final surface:Null<SurfaceTool>;
  public final sprayer:Null<Sprayer>;
  public final sander:Null<Sander>;
  public final adapter:ChannelToolAdapter = new ChannelToolAdapter();
  final declaredChannels:Array<ProcessChannelDeclaration> = [];

  public function new(tool:Tool, ?gripper:Gripper, ?vacuum:Vacuum,
      ?surface:SurfaceTool, ?sprayer:Sprayer, ?sander:Sander,
      ?changerLock:ChangerLock) {
    if (tool == null) throw "Tool runtime requires a mounted tool";
    this.tool = tool;
    this.gripper = gripper;
    this.vacuum = vacuum;
    this.changerLock = changerLock;
    this.surface = surface;
    this.sprayer = sprayer;
    this.sander = sander;
  }

  /**
   * Bind a digital close/open channel; false is the safe open state, which faults and emergency stops
   * take it to, while a commanded stop keeps a closed gripper closed on its part.
   */
  public function bindGripper(channel:String):Void {
    if (gripper == null) throw "Mounted tool has no gripper";
    var declaration = new ProcessChannelDeclaration(channel, ProcessEventValue.Digital(false), true);
    adapter.bindGripper(channel, gripper);
    declaredChannels.push(declaration);
  }

  /**
   * Bind a digital vacuum channel; false is the safe released state, which faults and emergency stops
   * take it to, while a commanded stop keeps holding what it holds.
   */
  public function bindVacuum(channel:String):Void {
    if (vacuum == null) throw "Mounted tool has no vacuum";
    var declaration = new ProcessChannelDeclaration(channel, ProcessEventValue.Digital(false), true);
    adapter.bindVacuum(channel, vacuum);
    declaredChannels.push(declaration);
  }

  /** Bind a lock command; true is the safe state during a configuration switch. */
  public function bindChangerLock(channel:String):Void {
    if (changerLock == null) throw "Mounted tool has no changer lock";
    var declaration = new ProcessChannelDeclaration(channel, ProcessEventValue.Digital(true));
    adapter.bindChangerLock(channel, changerLock);
    declaredChannels.push(declaration);
  }

  public function channelDeclarations():Array<ProcessChannelDeclaration>
    return declaredChannels.copy();
}
