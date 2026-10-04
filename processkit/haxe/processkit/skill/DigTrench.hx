package processkit.skill;

import robotkit.skill.*;

import robotkit.manipulation.Manipulator;
import robotkit.spatial.Transform3;
import processkit.work.BucketSweep;
import processkit.work.DigCyclePlan;
import processkit.work.DigCyclePlanner;
import processkit.work.HeightMap;
import processkit.work.Polygon2;
import processkit.work.Point2;
import robotkit.world.Robot;
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
 * cycle's `Toolpath` through validated plans, and applies the resulting
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
  public final planRunner:ToolpathPlanRunner;
  /** Design surface used to clamp every executed cut. */
  public var designMap(default, null):Null<HeightMap>;
  /** Planar trench footprint used to reject sweep vertices outside the work region. */
  public var footprint(default, null):Null<Polygon2>;
  public final footprintTolerance:Float;

  /** Dig cycles completed so far. */
  public var cyclesCompleted(default, null):Int = 0;
  /** Cumulative removed volume reported by `BucketSweep`, cubic meters. */
  public var totalRemovedVolume(default, null):Float = 0.0;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  final initialSeed:Array<Float>;
  var stage:DigTrenchStage = PreparingCycle;
  var originalGroundZ:Float = 0.0;
  var currentSweep:Null<DigCyclePlan> = null;
  var firstObservedCut:Null<Transform3> = null;
  var lastObservedCut:Null<Transform3> = null;
  var minObservedCutZ:Float = Math.POSITIVE_INFINITY;
  final jointIndices:Array<Int>;

  public function new(manipulator:Manipulator, robot:Robot, heightMap:HeightMap, frameId:String,
      lineFrom:Point2, lineTo:Point2, width:Float, depth:Float, gradeTolerance:Float,
      spec:DigTrenchSpec, seed:Array<Float>, planRunner:ToolpathPlanRunner,
      ?designMap:HeightMap, ?footprint:Polygon2,
      ?footprintTolerance:Float = -1.0) {
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
    if (planRunner == null) throw "DigTrench needs a toolpath plan runner";
    this.planRunner = planRunner;
    this.initialSeed = seed.copy();
    this.jointIndices = [for (target in manipulator.toJointTargets(seed)) target.joint];
    if (designMap != null) HeightMap.ensureSameGrid(heightMap, designMap);
    if (!Math.isFinite(footprintTolerance) || footprintTolerance < -1.0)
      throw "DigTrench footprint tolerance must be finite and non-negative";
    this.designMap = designMap;
    this.footprint = footprint;
    this.footprintTolerance = footprintTolerance < 0.0 ? heightMap.cellSize * 0.5 : footprintTolerance;
  }

  public function start():Void {
    lifecycle.begin();
    try {
      originalGroundZ = heightMap.bilinearSample(lineFrom.x, lineFrom.y);
      if (footprint == null) footprint = makeFootprint(lineFrom, lineTo, width * 0.5);
      if (designMap == null) designMap = makeDesignMap();
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

  /** Worst-case remaining cut (current elevation - design elevation) sampled along the line. */
  public function remainingDepthError():Float {
    var worst = 0.0;
    var samples = spec.progressSamples < 1 ? 1 : spec.progressSamples;
    for (i in 0...(samples + 1)) {
      var t = i / samples;
      var x = lineFrom.x + (lineTo.x - lineFrom.x) * t;
      var y = lineFrom.y + (lineTo.y - lineFrom.y) * t;
      var current = heightMap.bilinearSample(x, y);
      var target = designMap == null ? originalGroundZ - depth : designMap.bilinearSample(x, y);
      var remaining = current - target;
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
      var targetZ = designMap == null ? originalGroundZ - depth : designMap.bilinearSample(lineFrom.x, lineFrom.y);
      var remaining = currentZ - targetZ;
      var cut = remaining < spec.maxCutPerPass ? remaining : spec.maxCutPerPass;
      var plan = DigCyclePlanner.planCycle(frameId, lineFrom, lineTo, currentZ, cut,
        spec.clearanceZ, new Point2(spec.dumpX, spec.dumpY), spec.dumpZ, width * 0.5,
        spec.digPitch, spec.curlPitch, spec.dumpPitch, spec.feedRate);
      planRunner.run(plan.toolpath, initialSeed);
      if (!planRunner.running()) {
        lifecycle.fail('dig cycle $cyclesCompleted plan failed: ${planRunner.failure()}');
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
        lifecycle.fail('dig cycle $cyclesCompleted plan failed: ${planRunner.failure()}');
        return;
      }
      var sweep = currentSweep;
      var from = firstObservedCut, to = lastObservedCut;
      if (sweep == null || from == null || to == null) {
        lifecycle.fail("DigTrench completed without an observed cut");
        return;
      }
      totalRemovedVolume += BucketSweep.apply(heightMap,
        new Point2(from.translation.x, from.translation.y),
        new Point2(to.translation.x, to.translation.y), sweep.sweepHalfWidth + 0.001,
        minObservedCutZ, designMap, footprint, footprintTolerance).removedVolume;
      cyclesCompleted++;
      stage = PreparingCycle;
      beginNextCycle();
    }
  }

  function makeDesignMap():HeightMap {
    var result = heightMap.copy();
    var region = footprint;
    if (region == null) throw "DigTrench has no footprint for its generated design map";
    for (row in 0...result.rows) for (col in 0...result.columns) {
      var point = new Point2(result.worldX(col), result.worldY(row));
      if (region.containsOrWithin(point, 1e-9))
        result.setElevation(col, row, heightMap.elevationAt(col, row) - depth);
    }
    return result;
  }

  static function makeFootprint(from:Point2, to:Point2, halfWidth:Float):Polygon2 {
    var dx = to.x - from.x, dy = to.y - from.y;
    var length = Math.sqrt(dx * dx + dy * dy);
    if (!Math.isFinite(length) || length <= 1e-9)
      throw "DigTrench line must have non-zero length";
    var normalX = -dy / length, normalY = dx / length;
    return new Polygon2([
      new Point2(from.x - normalX * halfWidth, from.y - normalY * halfWidth),
      new Point2(to.x - normalX * halfWidth, to.y - normalY * halfWidth),
      new Point2(to.x + normalX * halfWidth, to.y + normalY * halfWidth),
      new Point2(from.x + normalX * halfWidth, from.y + normalY * halfWidth)
    ]);
  }
}
