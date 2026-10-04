package robotkit.tool;

import haxe.Int64;

/**
 * A tool that works against a standoff distance from a work surface and can
 * be turned on/off for a process. Commands carry the caller's timestamp
 * (simulation or issuing clock) so implementations never read a wall clock.
 */
interface SurfaceTool {
  function enable(timestampNs:Int64):Void;
  function disable(timestampNs:Int64):Void;
  function isEnabled():Bool;
  function setStandoff(standoffMeters:Float, timestampNs:Int64):Void;
  function standoff():Float;
}
