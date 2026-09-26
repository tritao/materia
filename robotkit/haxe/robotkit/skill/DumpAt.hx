package robotkit.skill;

import robotkit.manipulation.Manipulator;
import robotkit.process.CartesianTrajectory;
import robotkit.process.Toolpath;
import robotkit.process.ToolpathPoint;
import robotkit.process.ToolpathExecutionFailure;
import robotkit.process.ToolpathExecutionResult;
import robotkit.process.ToolpathExecutionStep;
import robotkit.process.ToolpathExecutor;
import robotkit.spatial.Transform3;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;

/**
 * Moves the bucket to a target TCP `pose` (chain-base frame) and holds it
 * there -- a single bounded Cartesian move through `ToolpathExecutor`, the
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
  public final sampleInterval:Float;
  public final maxJointStep:Float;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;
  public final ikMaxIterations:Int;
  public final ikDamping:Float;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var currentSteps:Array<ToolpathExecutionStep> = [];
  var stepIndex:Int = 0;
  var lastQ:Array<Float>;

  public function new(manipulator:Manipulator, robot:Robot, frameId:String, pose:Transform3, seed:Array<Float>,
      ?feedRate:Float = 0.4, ?maxAcceleration:Float = 0.6, ?sampleInterval:Float = 0.05,
      ?maxJointStep:Float = 3.0, ?positionTolerance:Float = 2e-3, ?orientationTolerance:Float = 5e-3,
      ?ikMaxIterations:Int = 300, ?ikDamping:Float = 0.03) {
    if (manipulator == null || robot == null || frameId == null || frameId.length == 0 || pose == null || seed == null)
      throw "DumpAt requires a manipulator, robot, frame id, pose, and seed";
    this.manipulator = manipulator;
    this.robot = robot;
    this.frameId = frameId;
    this.pose = pose;
    this.feedRate = feedRate;
    this.maxAcceleration = maxAcceleration;
    this.sampleInterval = sampleInterval;
    this.maxJointStep = maxJointStep;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.ikMaxIterations = ikMaxIterations;
    this.ikDamping = ikDamping;
    this.lastQ = seed.copy();
  }

  public function start():Void {
    lifecycle.begin();
    try {
      var currentPose = manipulator.tcpPose(lastQ);
      var toolpath = new Toolpath(frameId, [
        new ToolpathPoint(currentPose, feedRate, false),
        new ToolpathPoint(pose, feedRate, false)
      ]);
      var trajectory = CartesianTrajectory.build(toolpath, maxAcceleration, sampleInterval);
      var execution:ToolpathExecutionResult = ToolpathExecutor.execute(manipulator, trajectory, Transform3.identity(),
        lastQ, maxJointStep, positionTolerance, orientationTolerance, ikMaxIterations, ikDamping);
      if (!execution.success) {
        lifecycle.fail('DumpAt toolpath execution failed: ${describeFailure(execution.failure)}');
        return;
      }
      currentSteps = execution.steps;
      stepIndex = 0;
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
    if (stepIndex >= currentSteps.length) {
      lifecycle.succeed("dump complete");
      return lifecycle.status();
    }
    var step = currentSteps[stepIndex];
    robot.submit(RobotCommand.JointTargets(step.targets, null));
    lastQ = step.q;
    stepIndex++;
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    try robot.stop(robotkit.world.StopMode.Normal) catch (_:Dynamic) {}
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  static function describeFailure(failure:Null<ToolpathExecutionFailure>):String {
    return switch failure {
      case Unreachable(index, ik): 'unreachable at sample $index (positionError=${ik.positionError}, orientationError=${ik.orientationError}, iterations=${ik.iterations})';
      case Discontinuity(index, joint, delta): 'discontinuity at sample $index joint $joint (delta=$delta)';
      case null: "unknown failure";
    };
  }
}
