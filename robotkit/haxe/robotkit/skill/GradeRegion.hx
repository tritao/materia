package robotkit.skill;

import robotkit.manipulation.Manipulator;
import robotkit.spatial.Transform3;
import robotkit.work.BucketSweep;
import robotkit.work.DigCyclePlan;
import robotkit.work.DigCyclePlanner;
import robotkit.work.EarthworkRegion;
import robotkit.work.Point2;
import robotkit.world.Robot;
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
  public final planRunner:ToolpathPlanRunner;

  public var cyclesCompleted(default, null):Int = 0;
  public var totalRemovedVolume(default, null):Float = 0.0;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  final initialSeed:Array<Float>;
  var stage:GradeRegionStage = PreparingCycle;
  var currentSweep:Null<DigCyclePlan> = null;
  var firstObservedCut:Null<Transform3> = null;
  var lastObservedCut:Null<Transform3> = null;
  var minObservedCutZ:Float = Math.POSITIVE_INFINITY;
  final jointIndices:Array<Int>;

  public function new(manipulator:Manipulator, robot:Robot, region:EarthworkRegion, frameId:String,
      spec:GradeRegionSpec, seed:Array<Float>, planRunner:ToolpathPlanRunner) {
    if (manipulator == null || robot == null || region == null || frameId == null || frameId.length == 0 ||
        spec == null || seed == null)
      throw "GradeRegion requires a manipulator, robot, region, frame id, spec, and seed";
    this.manipulator = manipulator;
    this.robot = robot;
    this.region = region;
    this.frameId = frameId;
    this.spec = spec;
    if (planRunner == null) throw "GradeRegion needs a toolpath plan runner";
    this.planRunner = planRunner;
    this.initialSeed = seed.copy();
    this.jointIndices = [for (target in manipulator.toJointTargets(seed)) target.joint];
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
        advanceExecution(snapshot, durationSeconds);
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    try planRunner.abort() catch (_:Dynamic) {}
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
      planRunner.run(plan.toolpath, initialSeed);
      if (!planRunner.running()) {
        lifecycle.fail('grade cycle $cyclesCompleted plan failed: ${planRunner.failure()}');
        return;
      }
      currentSweep = plan;
      firstObservedCut = null;
      lastObservedCut = null;
      minObservedCutZ = Math.POSITIVE_INFINITY;
      stage = ExecutingCycle;
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  function advanceExecution(snapshot:RobotSnapshot, durationSeconds:Float):Void {
    try planRunner.update(durationSeconds) catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
      return;
    }
    if (planRunner.cuttingMoveActive()) {
      var q = [for (joint in jointIndices) snapshot.positions.get(joint)];
      var observed = manipulator.tcpPose(q);
      if (firstObservedCut == null) firstObservedCut = observed;
      lastObservedCut = observed;
      minObservedCutZ = Math.min(minObservedCutZ, observed.translation.z);
    }
    if (!planRunner.running()) {
      if (!planRunner.completed()) {
        lifecycle.fail('grade cycle $cyclesCompleted plan failed: ${planRunner.failure()}');
        return;
      }
      var sweep = currentSweep;
      var from = firstObservedCut, to = lastObservedCut;
      if (sweep == null || from == null || to == null) {
        lifecycle.fail("GradeRegion completed without an observed cut");
        return;
      }
      totalRemovedVolume += BucketSweep.apply(region.existing,
        new Point2(from.translation.x, from.translation.y),
        new Point2(to.translation.x, to.translation.y), sweep.sweepHalfWidth,
        minObservedCutZ).removedVolume;
      cyclesCompleted++;
      stage = PreparingCycle;
      beginNextCycle();
    }
  }
}
