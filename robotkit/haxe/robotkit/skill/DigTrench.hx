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
import robotkit.work.HeightMap;
import robotkit.work.Point2;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;

/**
 * Tunable per-cycle parameters for `DigTrench`/`GradeRegion`; a pure-data
 * anonymous typedef, per haxeon's structural-typing rules (mirrors `FinishSpec`).
 */
typedef DigTrenchSpec = {
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
  var progressSamples:Int;
}

private enum DigTrenchStage {
  PreparingCycle;
  ExecutingCycle;
}

/**
 * Digs a straight trench (`lineFrom` -> `lineTo`, `width`, `depth` below the
 * ground elevation sampled from `heightMap` at `start()`, `gradeTolerance`)
 * with `DigCyclePlanner`, one bounded pass at a time: each cycle samples the
 * *current* terrain from `heightMap`, cuts down by at most
 * `spec.maxCutPerPass` toward the trench's design elevation, executes the
 * cycle's `Toolpath` through `ToolpathExecutor`, and applies the resulting
 * `BucketSweep` to `heightMap` directly -- so progress is always read back
 * from the height map (`remainingDepthError`), the way a real excavator's
 * only feedback is the ground it has actually moved, never the commanded
 * cycle alone. Stops (succeeds) once every one of `spec.progressSamples + 1`
 * evenly-spaced points along the line is within `gradeTolerance` of the
 * design elevation, or fails after `spec.maxCycles` -- a bounded operation,
 * not an open-ended search, per the plan. The trench's `width` becomes the
 * swept bucket half-width for the whole pass (a single lane), rather than
 * multiple side-by-side bucket passes -- the smallest correct reading of
 * "width" for this milestone's bounded scope; see ARCHITECTURE.md.
 */
class DigTrench implements Skill {
  public final manipulator:Manipulator;
  public final robot:Robot;
  public final heightMap:HeightMap;
  public final frameId:String;
  public final lineFrom:Point2;
  public final lineTo:Point2;
  public final width:Float;
  public final depth:Float;
  public final gradeTolerance:Float;
  public final spec:DigTrenchSpec;

  /** Dig cycles completed so far. */
  public var cyclesCompleted(default, null):Int = 0;
  /** Cumulative removed volume reported by `BucketSweep`, cubic meters. */
  public var totalRemovedVolume(default, null):Float = 0.0;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  final initialSeed:Array<Float>;
  var stage:DigTrenchStage = PreparingCycle;
  var originalGroundZ:Float = 0.0;
  var lastQ:Array<Float>;
  var currentSteps:Array<ToolpathExecutionStep> = [];
  var stepIndex:Int = 0;
  var currentSweep:Null<DigCyclePlan> = null;

  public function new(manipulator:Manipulator, robot:Robot, heightMap:HeightMap, frameId:String,
      lineFrom:Point2, lineTo:Point2, width:Float, depth:Float, gradeTolerance:Float,
      spec:DigTrenchSpec, seed:Array<Float>) {
    if (manipulator == null || robot == null || heightMap == null || frameId == null || frameId.length == 0 ||
        lineFrom == null || lineTo == null || spec == null || seed == null)
      throw "DigTrench requires a manipulator, robot, height map, frame id, line, spec, and seed";
    if (!Math.isFinite(width) || width <= 0.0) throw "DigTrench width must be positive and finite";
    if (!Math.isFinite(depth) || depth <= 0.0) throw "DigTrench depth must be positive and finite";
    if (!Math.isFinite(gradeTolerance) || gradeTolerance < 0.0) throw "DigTrench grade tolerance must be finite and non-negative";
    this.manipulator = manipulator;
    this.robot = robot;
    this.heightMap = heightMap;
    this.frameId = frameId;
    this.lineFrom = lineFrom;
    this.lineTo = lineTo;
    this.width = width;
    this.depth = depth;
    this.gradeTolerance = gradeTolerance;
    this.spec = spec;
    this.initialSeed = seed.copy();
    this.lastQ = seed.copy();
  }

  public function start():Void {
    lifecycle.begin();
    try {
      originalGroundZ = heightMap.bilinearSample(lineFrom.x, lineFrom.y);
      cyclesCompleted = 0;
      totalRemovedVolume = 0.0;
      stage = PreparingCycle;
      beginNextCycle();
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (snapshot == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0.0) {
      lifecycle.fail("DigTrench update requires a robot snapshot and positive finite duration");
      return lifecycle.status();
    }
    switch stage {
      case PreparingCycle:
        lifecycle.fail("DigTrench reached update() before a cycle was staged");
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

  /** Worst-case remaining cut (current elevation - design elevation) sampled along the line. */
  public function remainingDepthError():Float {
    var worst = 0.0;
    var targetZ = originalGroundZ - depth;
    var samples = spec.progressSamples < 1 ? 1 : spec.progressSamples;
    for (i in 0...(samples + 1)) {
      var t = i / samples;
      var x = lineFrom.x + (lineTo.x - lineFrom.x) * t;
      var y = lineFrom.y + (lineTo.y - lineFrom.y) * t;
      var current = heightMap.bilinearSample(x, y);
      var remaining = current - targetZ;
      if (remaining > worst) worst = remaining;
    }
    return worst;
  }

  function isAtGrade():Bool return remainingDepthError() <= gradeTolerance;

  function beginNextCycle():Void {
    if (isAtGrade()) {
      lifecycle.succeed('trench at grade after $cyclesCompleted cycle(s)');
      return;
    }
    if (cyclesCompleted >= spec.maxCycles) {
      lifecycle.fail('DigTrench did not reach grade within ${spec.maxCycles} cycles (remaining=${remainingDepthError()})');
      return;
    }
    try {
      var currentZ = heightMap.bilinearSample(lineFrom.x, lineFrom.y);
      var targetZ = originalGroundZ - depth;
      var remaining = currentZ - targetZ;
      var cut = remaining < spec.maxCutPerPass ? remaining : spec.maxCutPerPass;
      var plan = DigCyclePlanner.planCycle(frameId, lineFrom, lineTo, currentZ, cut,
        spec.clearanceZ, new Point2(spec.dumpX, spec.dumpY), spec.dumpZ, width * 0.5,
        spec.digPitch, spec.curlPitch, spec.dumpPitch, spec.feedRate);
      var trajectory = CartesianTrajectory.build(plan.toolpath, spec.maxAcceleration, spec.sampleInterval);
      // Seed every cycle's IK from the constructor's own initial seed, not the
      // previous cycle's final (dump) configuration: a dig cycle's entry pose is
      // usually a large joint-space jump away from where the previous cycle left
      // off (swung out to dump), and warm-starting from that far, ever-drifting
      // configuration is the same cold-start/local-optimum sensitivity M2/M8's
      // logs already document, not a new bug -- so each cycle instead
      // "re-approaches" from the same known-good configuration, matching how a
      // real operator would reposition the boom/stick/bucket before a new pass.
      var execution:ToolpathExecutionResult = ToolpathExecutor.execute(manipulator, trajectory, Transform3.identity(),
        initialSeed, spec.maxJointStep, spec.positionTolerance, spec.orientationTolerance, spec.ikMaxIterations, spec.ikDamping);
      if (!execution.success) {
        lifecycle.fail('dig cycle $cyclesCompleted toolpath execution failed: ${describeFailure(execution.failure)}');
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
        totalRemovedVolume += BucketSweep.apply(heightMap, sweep.sweepFrom, sweep.sweepTo, sweep.sweepHalfWidth, sweep.sweepEdgeHeight).removedVolume;
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
