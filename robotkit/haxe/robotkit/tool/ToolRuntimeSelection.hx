package robotkit.tool;

import haxe.Int64;
import robotkit.world.FiredProcessEvent;

/** Routes process output to the selected mounted tool configuration. */
class ToolRuntimeSelection {
  var current:Null<ToolRuntime>;
  var selectedAt:Int64 = Int64.ofInt(0);

  public function new() {}

  public function active():Null<ToolRuntime> return current;
  public function selectedAtNs():Int64 return selectedAt;

  /** Release the old tool's bound controls before changing configuration. */
  public function select(next:Null<ToolRuntime>, timestampNs:Int64):Void {
    if (current == next) return;
    var previous = current;
    if (previous != null) for (channel in previous.channelDeclarations())
      previous.adapter.apply(new FiredProcessEvent(Int64.ofInt(0), channel.id,
        channel.safeValue, timestampNs, timestampNs, 0));
    current = next;
    selectedAt = timestampNs;
  }

  public function apply(event:FiredProcessEvent):Void {
    if (current == null) throw "No mounted tool configuration is selected";
    current.adapter.apply(event);
  }
}
