package robotkit.tool;

import haxe.Int64;

/** Vacuum pickup controls and observed holding state. */
interface Vacuum {
  function enable(timestampNs:Int64):Void;
  function disable(timestampNs:Int64):Void;
  function isEnabled():Bool;
  function isHolding():Bool;
}
