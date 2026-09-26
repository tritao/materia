package robotkit.tool;

import haxe.Int64;

/** Rotary/orbital sander controls: pad speed and commanded contact force. */
interface Sander {
  function setSpeed(rpm:Float, timestampNs:Int64):Void;
  function speed():Float;
  function setContactForce(newtons:Float, timestampNs:Int64):Void;
  function contactForce():Float;
}
