package robotkit.skill;

import robotkit.process.Toolpath;

/** Plan execution boundary for toolpath skills without a MotionKit dependency. */
interface ToolpathPlanRunner {
  function run(toolpath:Toolpath, seed:Array<Float>):Void;
  function update(dtSeconds:Float):Void;
  function abort():Void;
  function running():Bool;
  function completed():Bool;
  function failure():Null<String>;
  function cuttingMoveActive():Bool;
}
