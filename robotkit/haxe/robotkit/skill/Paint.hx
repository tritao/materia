package robotkit.skill;

import haxe.Int64;
import robotkit.manipulation.BaseObstacle;
import robotkit.manipulation.Manipulator;
import robotkit.navigation.Navigator;
import robotkit.perception.PerceptionSnapshot;
import robotkit.spatial.Transform3;
import robotkit.tool.Sprayer;
import robotkit.work.CoverageMap;
import robotkit.work.WorkSurface;
import robotkit.world.Robot;
import robotkit.world.RobotSnapshot;

/**
 * `FinishSurface` bound to a `Sprayer`: `setProcessOn` commands the
 * configured flow/pressure while spraying and zeroes both when off, per the
 * plan's "Paint ... as a thin FinishSpec variant".
 */
class Paint implements Skill {
  public final finish:FinishSurface;

  public function new(navigator:Navigator, manipulator:Manipulator, robot:Robot,
      map_T_surface:Transform3, surface:WorkSurface, spec:FinishSpec,
      observePerception:RobotSnapshot -> PerceptionSnapshot, sprayer:Sprayer,
      litersPerMinute:Float, pressureBar:Float, seed:Array<Float>, ?obstacles:Array<BaseObstacle>) {
    if (sprayer == null) throw "Paint requires a sprayer";
    finish = new FinishSurface(navigator, manipulator, robot, map_T_surface, surface, spec,
      observePerception, function(on:Bool, tick:Int64) {
        sprayer.setFlow(on ? litersPerMinute : 0.0, tick);
        sprayer.setPressure(on ? pressureBar : 0.0, tick);
      }, seed, obstacles);
  }

  public function coverage():Null<CoverageMap> return finish.coverage;

  public function start():Void finish.start();
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus
    return finish.update(snapshot, durationSeconds);
  public function cancel():Void finish.cancel();
  public function status():SkillStatus return finish.status();
  public function result():Null<SkillResult> return finish.result();
}
