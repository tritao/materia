package robotkit.tool;

import haxe.Int64;

/** Spray finishing tool controls: material flow rate and atomizing pressure. */
interface Sprayer {
  function setFlow(litersPerMinute:Float, timestampNs:Int64):Void;
  function flow():Float;
  function setPressure(bar:Float, timestampNs:Int64):Void;
  function pressure():Float;
}
