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
  /** Physical changer identity expected for this mounted configuration. */
  public final changerToolId:Null<String>;
  public final surface:Null<SurfaceTool>;
  public final sprayer:Null<Sprayer>;
  public final sander:Null<Sander>;
  @:allow(robotkit.tool.ToolRuntimeSelection)
  final adapter:ChannelToolAdapter = new ChannelToolAdapter();
  final declaredChannels:Array<ProcessChannelDeclaration> = [];
  final channelValues:Map<String, ProcessEventValue> = new Map();

  public function new(tool:Tool, ?gripper:Gripper, ?vacuum:Vacuum,
      ?surface:SurfaceTool, ?sprayer:Sprayer, ?sander:Sander,
      ?changerLock:ChangerLock, ?changerToolId:String) {
    if (tool == null) throw "Tool runtime requires a mounted tool";
    if (changerLock != null && (changerToolId == null || changerToolId.length == 0))
      throw "Changer-backed tool runtime requires its physical tool ID";
    this.tool = tool;
    this.gripper = gripper;
    this.vacuum = vacuum;
    this.changerLock = changerLock;
    this.changerToolId = changerToolId;
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

  @:allow(robotkit.tool.ToolRuntimeSelection)
  function apply(event:robotkit.execution.FiredProcessEvent):Void {
    adapter.apply(event);
    channelValues.set(event.channel, event.value);
  }

  @:allow(robotkit.tool.ToolRuntimeSelection)
  function applySafeValues(timestampNs:haxe.Int64):Void {
    for (channel in declaredChannels) apply(new robotkit.execution.FiredProcessEvent(
      haxe.Int64.ofInt(0), channel.id, channel.safeValue, timestampNs, timestampNs, 0));
  }

  @:allow(robotkit.tool.ToolRuntimeSelection)
  function unsafeChannels():Array<String> {
    var result:Array<String> = [];
    for (channel in declaredChannels) {
      var value = channelValues.get(channel.id);
      if (value == null || !sameValue(value, channel.safeValue)) result.push(channel.id);
    }
    return result;
  }

  static function sameValue(a:ProcessEventValue, b:ProcessEventValue):Bool return switch [a, b] {
    case [Digital(left), Digital(right)]: left == right;
    case [Analog(left), Analog(right)]: left == right;
    case [Process(leftCommand, leftArgument), Process(rightCommand, rightArgument)]:
      leftCommand == rightCommand && leftArgument == rightArgument;
    case _: false;
  }
}
