package motionkit.robot;

import motionkit.program.MotionProgram;
import robotkit.skill.Skill;
import robotkit.skill.SkillLifecycle;
import robotkit.skill.SkillResult;
import robotkit.skill.SkillStatus;
import robotkit.core.RobotSnapshot;

/** Runs a planned arm program through the same lifecycle as the other robot skills. */
class MotionProgramSkill implements Skill {
  final motion:ManipulatorMotion;
  final program:MotionProgram;
  final lifecycle = new SkillLifecycle();

  public function new(motion:ManipulatorMotion, program:MotionProgram) {
    if (motion == null || program == null) throw "MotionProgramSkill needs motion and a program";
    this.motion = motion;
    this.program = program;
  }

  public function start():Void {
    lifecycle.begin();
    try { motion.run(program); settle(); }
    catch (error:Dynamic) fail(Std.string(error));
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (snapshot == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0) {
      fail("MotionProgramSkill needs a snapshot and positive finite duration");
      return lifecycle.status();
    }
    try { motion.update(durationSeconds); settle(); }
    catch (error:Dynamic) fail(Std.string(error));
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    motion.abort();
    lifecycle.cancel();
  }
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  function settle():Void {
    if (motion.failure != null) lifecycle.fail(motion.failure);
    else if (motion.completed) lifecycle.succeed("motion program completed");
  }
  function fail(message:String):Void {
    try motion.abort() catch (_:Dynamic) {}
    lifecycle.fail(message);
  }
}
