package robotkit.tool;

import haxe.Int64;
import robotkit.execution.FiredProcessEvent;

/** Routes process output to the selected mounted tool configuration. */
class ToolRuntimeSelection {
  var current:Null<ToolRuntime>;
  var processOwner:Null<String>;
  var processToolId:Null<String>;
  var processRuntimeToolId:Null<String>;
  var selectedAt:Int64 = Int64.ofInt(0);
  var clockId:String = "robotkit.monotonic";

  public function new() {}

  public function active():Null<ToolRuntime> return current;
  /** Process identity that currently owns the mounted tool's live outputs, if any. */
  public function activeProcessOwner():Null<String> return processOwner;
  /** Tool identity captured when the active process acquired ownership. */
  public function activeProcessToolId():Null<String> return processToolId;
  public function selectedAtNs():Int64 return selectedAt;
  public function selectedClockId():String return clockId;

  /** Reserve a safe selected tool for one process; its outputs remain leased until all are safe again. */
  public function beginProcess(owner:String):Void {
    if (owner == null || owner.length == 0) throw "Tool process needs a non-empty owner";
    var selected:Null<ToolRuntime> = current;
    if (selected == null) throw 'Tool process "$owner" has no selected mounted tool';
    var mounted:ToolRuntime = cast selected;
    if (processOwner != null)
      throw 'Tool process "$owner" cannot take the tool from live process "$processOwner"';
    var unsafe = mounted.unsafeChannels();
    if (unsafe.length != 0)
      throw 'Tool process "$owner" cannot start while tool outputs are unsafe: ${unsafe.join(", ")}';
    processOwner = owner;
    processToolId = mounted.changerToolId == null ? mounted.tool.id : mounted.changerToolId;
    processRuntimeToolId = mounted.tool.id;
  }

  /** Release the exact process lease only after its runner stopped and declared outputs are safe. */
  public function finishProcess(owner:String):Void {
    if (processOwner == null) throw 'Tool process "$owner" does not own a mounted tool';
    if (owner != processOwner) throw 'Tool process "$owner" cannot release owner "$processOwner"';
    var selected:Null<ToolRuntime> = current;
    if (selected == null || selected.tool.id != processRuntimeToolId)
      throw 'Tool process "$owner" lost its mounted tool "$processToolId"';
    var mounted:ToolRuntime = cast selected;
    var unsafe = mounted.unsafeChannels();
    if (unsafe.length != 0)
      throw 'Tool process "$owner" cannot release its lease while outputs are unsafe: ${unsafe.join(", ")}';
    processOwner = null;
    processToolId = null;
    processRuntimeToolId = null;
  }

  /**
   * Select a software runtime after confirming the requested mounted tool.
   * Passing null disables process routing and safes the current controls; it
   * does not claim that the physical tool detached from the changer.
   */
  public function select(next:Null<ToolRuntime>, timestampNs:Int64,
      ?sourceClockId:String = "robotkit.monotonic"):Void {
    if (sourceClockId == null || sourceClockId.length == 0)
      throw "Tool selection needs a source clock";
    if (next != null) {
      var nextLock = next.changerLock;
      var currentLock = current == null ? null : current.changerLock;
      if (nextLock != null) {
        if (current != null && currentLock != nextLock)
          throw "Tool configurations must share one changer lock before selection can change";
        var observed = nextLock.observation();
        if (observed == null) throw "Changer returned no lock/tool observation";
        if (observed.sourceClockId == null || observed.sourceClockId.length == 0)
          throw "Changer feedback has no source clock identity";
        if (observed.sourceClockId != sourceClockId)
          throw 'Changer feedback clock "${observed.sourceClockId}" does not match selection clock "$sourceClockId"';
        if (Int64.compare(observed.sourceTimestampNs, timestampNs) > 0)
          throw "Changer feedback is newer than the tool selection event";
        if (!observed.locked)
          throw 'Cannot select tool "${next.tool.id}" before changer state reports engagement';
        if (observed.toolId == null || observed.toolId != next.changerToolId)
          throw 'Changer reports tool "${observed.toolId}"; selection requested "${next.changerToolId}"';
      } else if (currentLock != null) {
        throw "Tool configuration change would lose the active changer lock feedback";
      }
    }
    if (current == next) return;
    if (processOwner != null)
      throw 'Cannot change mounted tool while process "$processOwner" owns tool "$processToolId"';
    var previous = current;
    if (previous != null) previous.applySafeValues(timestampNs);
    if (next != null) next.applySafeValues(timestampNs);
    current = next;
    selectedAt = timestampNs;
    clockId = sourceClockId;
  }

  public function apply(event:FiredProcessEvent):Void {
    if (processOwner != null)
      throw 'Tool output needs owner "$processOwner" while that process is active';
    var selected:Null<ToolRuntime> = current;
    if (selected == null) throw "No mounted tool configuration is selected";
    var mounted:ToolRuntime = cast selected;
    mounted.apply(event);
  }

  /** Apply a process event only while its named process owns the currently selected tool. */
  public function applyFor(owner:String, event:FiredProcessEvent):Void {
    if (processOwner == null) throw 'Tool output owner "$owner" has no active process lease';
    if (owner != processOwner) throw 'Tool output owner "$owner" does not own "$processOwner"';
    var selected:Null<ToolRuntime> = current;
    if (selected == null || selected.tool.id != processRuntimeToolId)
      throw 'Tool process "$processOwner" lost its mounted tool "$processToolId"';
    var mounted:ToolRuntime = cast selected;
    mounted.apply(event);
  }
}
