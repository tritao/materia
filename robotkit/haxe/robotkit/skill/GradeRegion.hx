package robotkit.skill;

import robotkit.manipulation.Manipulator;
import robotkit.process.CartesianTrajectory;
import robotkit.process.ToolpathExecutionFailure;
import robotkit.process.ToolpathExecutionResult;
import robotkit.process.ToolpathExecutionStep;
import robotkit.process.ToolpathExecutor;
import robotkit.spatial.Transform3;
import robotkit.work.BucketSweep;
import robotkit.work.DigCyclePlan;
import robotkit.work.DigCyclePlanner;
import robotkit.work.EarthworkRegion;
import robotkit.work.Point2;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;

/**
 * Tunable per-cycle parameters for `GradeRegion`; a pure-data anonymous
 * typedef (mirrors `DigTrenchSpec`, minus the trench-specific fields).
 */
typedef GradeRegionSpec = {
  var clearanceZ:Float;
  var dumpX:Float;
  var dumpY:Float;
  var dumpZ:Float;
  var digPitch:Float;
  var curlPitch:Float;
  var dumpPitch:Float;
  var feedRate:Float;
  var maxAcceleration:Float;
  var sampleInterval:Float;
  var maxCutPerPass:Float;
  var maxCycles:Int;
  var maxJointStep:Float;
  var positionTolerance:Float;
  var orientationTolerance:Float;
  var ikMaxIterations:Int;
  var ikDamping:Float;
  var bucketHalfWidth:Float;
}

private enum GradeRegionStage {
  PreparingCycle;
  ExecutingCycle;
}

/**
 * Grades an `EarthworkRegion` toward its design `HeightMap`, one bounded
 * cycle at a time: each cycle asks `region.worstVertex()` for the
 * largest-magnitude, non-excluded, out-of-tolerance vertex, cuts it down by
 * at most `spec.maxCutPerPass` toward its design elevation with a single
 * `DigCyclePlanner` cycle (a degenerate zero-length "cut" segment centered on
 * that vertex, since a `GradeRegion` cycle targets one spot rather than a
 * line), executes it, and applies the resulting `BucketSweep` to
 * `region.existing` directly. Stops (succeeds) once `worstVertex()` returns
 * null, or fails after `spec.maxCycles` -- a bounded search over the
 * region's existing grid, not open-ended, per the plan. Only cut (existing
 * above design) is handled: fill/embankment is out of scope (no soil
 * mechanics, per the plan's explicit boundary), so a region whose worst
 * vertex needs fill fails explicitly rather than looping forever.
 */
class GradeRegion implements Skill {
  public final manipulator:Manipulator;
  public final robot:Robot;
  public final region:EarthworkRegion;
  public final frameId:String;
  public final spec:GradeRegionSpec;

  public var cyclesCompleted(default, null):Int = 0;
  public var totalRemovedVolume(default, null):Float = 0.0;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  final initialSeed:Array<Float>;
  var stage:GradeRegionStage = PreparingCycle;
  var lastQ:Array<Float>;
  var currentSteps:Array<ToolpathExecutionStep> = [];
  var stepIndex:Int = 0;
  var currentSweep:Null<DigCyclePlan> = null;

  public function new(manipulator:Manipulator, robot:Robot, region:EarthworkRegion, frameId:String,
      spec:GradeRegionSpec, seed:Array<Float>) {
    if (manipulator == null || robot == null || region == null || frameId == null || frameId.length == 0 ||
        spec == null || seed == null)
      throw "GradeRegion requires a manipulator, robot, region, frame id, spec, and seed";
    this.manipulator = manipulator;
    this.robot = robot;
    this.region = region;
    this.frameId = frameId;
    this.spec = spec;
    this.initialSeed = seed.copy();
    this.lastQ = seed.copy();
  }

  public function start():Void {
    lifecycle.begin();
    cyclesCompleted = 0;
    totalRemovedVolume = 0.0;
    stage = PreparingCycle;
    beginNextCycle();
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (snapshot == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0.0) {
      lifecycle.fail("GradeRegion update requires a robot snapshot and positive finite duration");
      return lifecycle.status();
    }
    switch stage {
      case PreparingCycle:
        lifecycle.fail("GradeRegion reached update() before a cycle was staged");
      case ExecutingCycle:
        advanceExecution();
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    try robot.stop(robotkit.world.StopMode.Normal) catch (_:Dynamic) {}
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  function beginNextCycle():Void {
    var worst = region.worstVertex();
    if (worst == null) {
      lifecycle.succeed('region at grade after $cyclesCompleted cycle(s)');
      return;
    }
    if (cyclesCompleted >= spec.maxCycles) {
      lifecycle.fail('GradeRegion did not reach grade within ${spec.maxCycles} cycles');
      return;
    }
    if (worst.delta <= 0.0) {
      lifecycle.fail('GradeRegion vertex (${worst.col}, ${worst.row}) needs fill, which is out of scope');
      return;
    }
    try {
      var x = region.existing.worldX(worst.col), y = region.existing.worldY(worst.row);
      var currentZ = region.existing.elevationAt(worst.col, worst.row);
      var targetZ = region.design.elevationAt(worst.col, worst.row);
      var remaining = currentZ - targetZ;
      var cut = remaining < spec.maxCutPerPass ? remaining : spec.maxCutPerPass;
      var point = new Point2(x, y);
      var plan = DigCyclePlanner.planCycle(frameId, point, point, currentZ, cut,
        spec.clearanceZ, new Point2(spec.dumpX, spec.dumpY), spec.dumpZ, spec.bucketHalfWidth,
        spec.digPitch, spec.curlPitch, spec.dumpPitch, spec.feedRate);
      var trajectory = CartesianTrajectory.build(plan.toolpath, spec.maxAcceleration, spec.sampleInterval);
      // Seed from the constructor's own initial seed each cycle, not the previous
      // cycle's dump configuration -- see DigTrench's identical choice/rationale.
      var execution:ToolpathExecutionResult = ToolpathExecutor.execute(manipulator, trajectory, Transform3.identity(),
        initialSeed, spec.maxJointStep, spec.positionTolerance, spec.orientationTolerance, spec.ikMaxIterations, spec.ikDamping);
      if (!execution.success) {
        lifecycle.fail('grade cycle $cyclesCompleted toolpath execution failed: ${describeFailure(execution.failure)}');
        return;
      }
      currentSteps = execution.steps;
      stepIndex = 0;
      currentSweep = plan;
      stage = ExecutingCycle;
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  function advanceExecution():Void {
    if (stepIndex >= currentSteps.length) {
      var sweep = currentSweep;
      if (sweep != null)
        totalRemovedVolume += BucketSweep.apply(region.existing, sweep.sweepFrom, sweep.sweepTo, sweep.sweepHalfWidth, sweep.sweepEdgeHeight).removedVolume;
      cyclesCompleted++;
      stage = PreparingCycle;
      beginNextCycle();
      return;
    }
    var step = currentSteps[stepIndex];
    robot.submit(RobotCommand.JointTargets(step.targets, null));
    lastQ = step.q;
    stepIndex++;
  }

  static function describeFailure(failure:Null<ToolpathExecutionFailure>):String {
    return switch failure {
      case Unreachable(index, ik): 'unreachable at sample $index (positionError=${ik.positionError}, orientationError=${ik.orientationError}, iterations=${ik.iterations})';
      case Discontinuity(index, joint, delta): 'discontinuity at sample $index joint $joint (delta=$delta)';
      case null: "unknown failure";
    };
  }
}
