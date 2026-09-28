package robotkit.skill;

import robotkit.manipulation.BaseObstacle;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ToolPlanningContext;
import robotkit.navigation.Navigator;
import robotkit.perception.PerceptionSnapshot;
import robotkit.spatial.Transform3;
import robotkit.tool.Sprayer;
import robotkit.tool.ChannelToolAdapter;
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
      litersPerMinute:Float, pressureBar:Float, seed:Array<Float>,
      runner:SurfacePlanRunner, ?obstacles:Array<BaseObstacle>, ?toolPlanning:ToolPlanningContext) {
    if (sprayer == null) throw "Paint requires a sprayer";
    var adapter = new ChannelToolAdapter();
    adapter.bind("surface.process", function(event) {
      var on = switch event.value {
        case Digital(value): value;
        case Analog(value): value > 0.0;
        case Process(_, _): throw "Paint process channel needs a digital or analog value";
      };
      sprayer.setFlow(on ? litersPerMinute : 0.0, event.scheduledTimeNs);
      sprayer.setPressure(on ? pressureBar : 0.0, event.scheduledTimeNs);
    });
    finish = new FinishSurface(navigator, manipulator, robot, map_T_surface, surface, spec,
      observePerception, runner, adapter, "surface.process", seed, obstacles, toolPlanning);
  }

  public function coverage():Null<CoverageMap> return finish.coverage;

  public function start():Void finish.start();
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus
    return finish.update(snapshot, durationSeconds);
  public function cancel():Void finish.cancel();
  public function hold():Void finish.hold();
  public function resume():Void finish.resume();
  public function status():SkillStatus return finish.status();
  public function result():Null<SkillResult> return finish.result();
}
