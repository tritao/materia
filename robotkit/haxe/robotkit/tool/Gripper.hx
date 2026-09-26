package robotkit.tool;

import haxe.Int64;

/** Two-state gripper controls and observed grasp outcome. */
interface Gripper {
  function open(timestampNs:Int64):Void;
  function close(timestampNs:Int64):Void;
  function isOpen():Bool;
  function isGrasped():Bool;
}
