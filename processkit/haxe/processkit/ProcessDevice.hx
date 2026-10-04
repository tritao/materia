package processkit;

import robotkit.execution.FiredProcessEvent;

/** Device readiness, fault and safe-output boundary for a process run. */
interface ProcessDevice {
  function prepare():Void;
  function ready():Bool;
  function fault():Null<String>;
  function safe():Void;
  function apply(records:Array<FiredProcessEvent>):Void;
}
