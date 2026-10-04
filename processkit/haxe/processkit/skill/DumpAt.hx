package processkit.skill;

import robotkit.skill.*;

import robotkit.manipulation.Manipulator;
import processkit.path.Toolpath;
import processkit.path.ToolpathPoint;
import robotkit.spatial.Transform3;
import robotkit.world.Robot;
import robotkit.world.RobotSnapshot;

/**
 * Moves the bucket to a target TCP `pose` (chain-base frame) and holds it
 * there -- a bounded joint move to a TCP IK target through a validated plan, the
 * same machinery `DigTrench`/`GradeRegion` use for their own dump legs,
 * exposed standalone for a caller that already holds a loaded bucket (e.g.
 * after a manual or externally-commanded dig) and just needs to relocate and
 * release. Unlike `DigTrench`/`GradeRegion`, `pose` is not required to lie
 * on the excavator chain's reachable orientation manifold (see
 * `DigCyclePlanner.poseAt`) -- a caller building one with that helper gets
 * an exact solve; an arbitrary pose gets the manipulator's ordinary
 * best-effort IK convergence.
 */
class DumpAt implements Skill {
  public final manipulator:Manipulator;
  public final robot:Robot;
  public final frameId:String;
  public final pose:Transform3;
  public final feedRate:Float;
  public final maxAcceleration:Float;
  public final maxJointStep:Float;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;
  public final ikMaxIterations:Int;
  public final ikDamping:Float;
  public final planRunner:ToolpathPlanRunner;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  final seed:Array<Float>;

  public function new(manipulator:Manipulator, robot:Robot, frameId:String, pose:Transform3, seed:Array<Float>,
      ?feedRate:Float = 0.4, ?maxAcceleration:Float = 0.6,
      ?maxJointStep:Float = 3.0, ?positionTolerance:Float = 2e-3, ?orientationTolerance:Float = 5e-3,
      ?ikMaxIterations:Int = 300, ?ikDamping:Float = 0.03,
      ?planRunner:ToolpathPlanRunner) {
    if (manipulator == null || robot == null || frameId == null || frameId.length == 0 || pose == null || seed == null)
      throw "DumpAt requires a manipulator, robot, frame id, pose, and seed";
    this.manipulator = manipulator;
    this.robot = robot;
    this.frameId = frameId;
    this.pose = pose;
    this.feedRate = feedRate;
    this.maxAcceleration = maxAcceleration;
    this.maxJointStep = maxJointStep;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.ikMaxIterations = ikMaxIterations;
    this.ikDamping = ikDamping;
    if (planRunner == null) throw "DumpAt needs a toolpath plan runner";
    this.planRunner = planRunner;
    this.seed = seed.copy();
  }

  public function start():Void {
    lifecycle.begin();
    try {
      var currentPose = manipulator.tcpPose(seed);
      var toolpath = new Toolpath(frameId, [
        new ToolpathPoint(currentPose, feedRate, false),
        new ToolpathPoint(pose, feedRate, false)
      ]);
      planRunner.run(toolpath, seed);
      if (!planRunner.running()) lifecycle.fail('DumpAt plan failed: ${planRunner.failure()}');
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (snapshot == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0.0) {
      lifecycle.fail("DumpAt update requires a robot snapshot and positive finite duration");
      return lifecycle.status();
    }
    try planRunner.update(durationSeconds) catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
      return lifecycle.status();
    }
    if (planRunner.completed()) {
      lifecycle.succeed("dump complete");
      return lifecycle.status();
    }
    if (!planRunner.running()) lifecycle.fail('DumpAt plan failed: ${planRunner.failure()}');
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    try planRunner.abort() catch (_:Dynamic) {}
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

}
