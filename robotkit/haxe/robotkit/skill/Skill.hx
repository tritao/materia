package robotkit.skill;

import robotkit.world.RobotSnapshot;

/** Stateful task primitive advanced by the application/control loop. */
interface Skill {
  function start():Void;
  function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus;
  function cancel():Void;
  function status():SkillStatus;
  function result():Null<SkillResult>;
}
