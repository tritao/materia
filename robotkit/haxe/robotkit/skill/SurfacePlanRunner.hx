package robotkit.skill;

import robotkit.manipulation.WorkPatch;
import robotkit.spatial.Transform3;
import robotkit.world.FiredProcessEvent;

/** Plan execution boundary used by finishing skills without a MotionKit dependency. */
interface SurfacePlanRunner {
  function runPatch(patch:WorkPatch, base_T_work:Transform3, seed:Array<Float>):Void;
  function update(dtSeconds:Float):Void;
  function hold():Void;
  function resume():Void;
  function abort():Void;
  function running():Bool;
  function completed():Bool;
  function failure():Null<String>;
  function firedEvents():Array<FiredProcessEvent>;
  function jointIndices():Array<Int>;
}
