package processkit.screwdriving;

import robotkit.spatial.Transform3;

/** Position motion of the arm alone; feed and spindle belong to the screwdriver. */
interface ScrewdrivingMotion {
  function move(target:Transform3, contact:Bool):Void;
  function update(durationSeconds:Float):Void;
  function completed():Bool;
  function failure():Null<String>;
  function abort():Void;
}
