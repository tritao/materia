package processkit.screwdriving;

import robotkit.spatial.Transform3;

/** Position motion of the arm alone; feed and spindle belong to the screwdriver. */
interface ScrewdrivingMotion {
  function move(target:Transform3, contact:Bool):Void;
  function update(durationSeconds:Float):Void;
  function completed():Bool;
  function failure():Null<String>;
  /** Rebase the next planned move on measured state after an external servo motion. */
  function synchronize():Void;
  /** Request a controlled stop without clearing any motion or runtime fault. */
  function stop():Void;
  /** True only after motion update confirms physical rest, with no fault. */
  function stopped():Bool;
  function abort():Void;
}
