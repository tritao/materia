package robotkit.skill;

import haxe.Int64;
import robotkit.localization.LocalizationState;
import robotkit.manipulation.BaseObstacle;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.WorkPatch;
import robotkit.manipulation.WorkPatchPlanner;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigator;
import robotkit.perception.PerceptionSnapshot;
import robotkit.process.CartesianTrajectory;
import robotkit.process.ToolpathExecutionResult;
import robotkit.process.ToolpathExecutionFailure;
import robotkit.process.ToolpathExecutor;
import robotkit.spatial.Transform3;
import robotkit.work.CoverageMap;
import robotkit.work.Point2;
import robotkit.work.WorkSurface;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;

/**
 * Process parameters for one `FinishSurface` run: raster geometry
 * (`toolWidth`/`overlap`/`standoff`/`feedRate`/`leadInOut`, the same
 * `RasterToolpathGenerator` inputs `WorkPatchPlanner` forwards per patch),
 * `maxPatchWidth`/standoff-search band for `WorkPatchPlanner`, and the
 * Cartesian/IK tolerances `CartesianTrajectory`/`ToolpathExecutor` use. A
 * pure-data anonymous typedef, per haxeon's structural-typing rules.
 */
typedef FinishSpec = {
  var toolWidth:Float;
  var overlap:Float;
  var standoff:Float;
  var feedRate:Float;
  var leadInOut:Float;
  var maxPatchWidth:Float;
  var standoffDistanceMin:Float;
  var standoffDistanceMax:Float;
  var maxAcceleration:Float;
  var coverageCellSize:Float;
  var footprintRadius:Float;
  /**
   * Largest allowed joint-space step `ToolpathExecutor` accepts between
   * consecutive samples, *including* the initial move from the seed
   * configuration to a patch's first point — a real repositioning move
   * before a continuous raster starts, not a discontinuity in the raster
   * itself. Callers that seed from a configuration far from the work (a
   * cold start) need this larger than the small per-sample steps a slow
   * feed rate alone would produce.
   */
  var maxJointStep:Float;
}

private enum FinishStage {
  Planning;
  NavigatingToPatch;
  ExecutingPatch;
}

/**
 * Drives M8's `WorkPatchPlanner` through M4's `ToolpathExecutor`: split the
 * surface into patches, navigate the base to each patch's chosen pose
 * (`Navigator`/`GoTo`, unchanged per the plan's base-motion boundary), plan
 * and execute that patch's raster, and toggle the process on/off around
 * each step's `processOn` flag via a caller-supplied `setProcessOn`
 * callback — `Paint`/`Sand` bind this to `Sprayer`/`Sander` commands, so
 * `FinishSurface` itself never depends on which capability interface the
 * mounted tool implements. Coverage is tracked from the *planned* TCP pose
 * (`Manipulator.tcpPose` at each executed step's joint solution): unlike the
 * M9 scenario test, a skill has no `Simulation` to cross-check against and
 * must work identically over a `RemoteRobot`, `SimulatedRobot`, or
 * `ReplayRobot`.
 */
class FinishSurface implements Skill {
  public final navigator:Navigator;
  public final manipulator:Manipulator;
  public final robot:Robot;
  public final map_T_surface:Transform3;
  public final surface:WorkSurface;
  public final spec:FinishSpec;
  public final observePerception:RobotSnapshot -> PerceptionSnapshot;
  public final setProcessOn:(Bool, Int64) -> Void;
  public final seed:Array<Float>;
  public final obstacles:Array<BaseObstacle>;

  /** Cumulative coverage across every executed patch; available once planning succeeds. */
  public var coverage(default, null):Null<CoverageMap> = null;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var patches:Array<WorkPatch> = [];
  var patchIndex:Int = 0;
  var stage:FinishStage = Planning;
  var currentGoTo:Null<GoTo> = null;
  var currentSteps:Array<robotkit.process.ToolpathExecutionStep> = [];
  var stepIndex:Int = 0;
  var toolOn:Bool = false;
  var lastQ:Array<Float>;

  public function new(navigator:Navigator, manipulator:Manipulator, robot:Robot,
      map_T_surface:Transform3, surface:WorkSurface, spec:FinishSpec,
      observePerception:RobotSnapshot -> PerceptionSnapshot,
      setProcessOn:(Bool, Int64) -> Void, seed:Array<Float>, ?obstacles:Array<BaseObstacle>) {
    if (navigator == null || manipulator == null || robot == null || map_T_surface == null ||
        surface == null || spec == null || observePerception == null || setProcessOn == null || seed == null)
      throw "FinishSurface requires navigation, a manipulator, a robot, a surface, a spec, and a seed";
    this.navigator = navigator;
    this.manipulator = manipulator;
    this.robot = robot;
    this.map_T_surface = map_T_surface;
    this.surface = surface;
    this.spec = spec;
    this.observePerception = observePerception;
    this.setProcessOn = setProcessOn;
    this.seed = seed.copy();
    this.obstacles = obstacles == null ? [] : obstacles;
    lastQ = seed.copy();
  }

  public function start():Void {
    lifecycle.begin();
    try {
      var result = WorkPatchPlanner.plan(surface, map_T_surface, manipulator,
        spec.maxPatchWidth, spec.toolWidth, spec.overlap, spec.standoff, spec.feedRate, spec.leadInOut,
        spec.standoffDistanceMin, spec.standoffDistanceMax, seed, 3, 3, 0.4, obstacles);
      if (result.patches.length == 0) {
        lifecycle.fail("work patch planner produced no patches");
        return;
      }
      if (!result.fullyPlanned) {
        lifecycle.fail("work patch planner could not fully reach every patch");
        return;
      }
      patches = result.patches;
      coverage = new CoverageMap(surface, spec.coverageCellSize);
      patchIndex = 0;
      beginPatch(patchIndex);
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (snapshot == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0.0) {
      lifecycle.fail("FinishSurface update requires a robot snapshot and positive finite duration");
      return lifecycle.status();
    }
    switch stage {
      case Planning:
        lifecycle.fail("FinishSurface reached update() before a patch was staged");
      case NavigatingToPatch:
        var goTo:Null<GoTo> = currentGoTo;
        if (goTo == null) { lifecycle.fail("FinishSurface has no active navigation task"); return lifecycle.status(); }
        var navStatus = goTo.update(snapshot, durationSeconds);
        switch navStatus {
          case SkillStatus.Running:
          case SkillStatus.Succeeded: beginExecution();
          case SkillStatus.Cancelled: lifecycle.cancel();
          case SkillStatus.Failed(message): lifecycle.fail(message);
          case SkillStatus.Idle: lifecycle.fail("navigation returned to idle");
        }
      case ExecutingPatch:
        advanceExecution();
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    if (currentGoTo != null) currentGoTo.cancel();
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

  function beginPatch(index:Int):Void {
    var patch = patches[index];
    var goTo = new GoTo(navigator,
      new NavigationGoal(patch.basePose, surface.frameId, 0.02, 0.02), observePerception);
    currentGoTo = goTo;
    stage = NavigatingToPatch;
    goTo.start();
    switch goTo.status() {
      case SkillStatus.Succeeded: beginExecution();
      case SkillStatus.Failed(message): lifecycle.fail(message);
      case SkillStatus.Cancelled: lifecycle.cancel();
      case _:
    }
  }

  function beginExecution():Void {
    try {
      var patch = patches[patchIndex];
      var estimate = navigator.navigation.localization.state();
      if (estimate == null) throw "FinishSurface has no localization state to execute a patch from";
      var state:LocalizationState = cast estimate;
      var baseWorld = Transform3.fromPose2(state.pose, 0.0);
      var base_T_work = baseWorld.inverse().compose(map_T_surface);
      var trajectory = CartesianTrajectory.build(patch.toolpath, spec.maxAcceleration, 0.05);
      // 2mm/5mrad matches the M9 scenario test's own proven-converging IK
      // tolerances (and WorkSurface's own default 2mm `tolerance` field): a
      // tighter 1mm tolerance left this call within a hairline's width of a
      // damped-least-squares non-convergence at a near-limit raster pose
      // (observed as a rebuild-sensitive flake), matching the documented
      // cold-start/near-limit IK sensitivity in ARCHITECTURE.md.
      var execution:ToolpathExecutionResult = ToolpathExecutor.execute(manipulator, trajectory,
        base_T_work, lastQ, spec.maxJointStep, 2e-3, 5e-3, 300, 0.03);
      if (!execution.success) {
        lifecycle.fail('toolpath execution failed for patch $patchIndex: ${describeFailure(execution.failure)}');
        return;
      }
      currentSteps = execution.steps;
      stepIndex = 0;
      toolOn = false;
      stage = ExecutingPatch;
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  function advanceExecution():Void {
    if (stepIndex >= currentSteps.length) {
      if (toolOn) { setProcessOn(false, Int64.ofInt(0)); toolOn = false; }
      patchIndex++;
      if (patchIndex >= patches.length) {
        lifecycle.succeed('finished ${patches.length} patch(es)');
        return;
      }
      beginPatch(patchIndex);
      return;
    }
    var step = currentSteps[stepIndex];
    if (step.processOn != toolOn) {
      toolOn = step.processOn;
      setProcessOn(toolOn, Int64.ofInt(stepIndex));
    }
    robot.submit(RobotCommand.JointTargets(step.targets, null));
    lastQ = step.q;
    if (step.processOn) {
      var patch = patches[patchIndex];
      var estimate = navigator.navigation.localization.state();
      if (estimate != null) {
        var state:LocalizationState = cast estimate;
        var baseWorld = Transform3.fromPose2(state.pose, 0.0);
        var tcpWorld = baseWorld.compose(manipulator.tcpPose(step.q));
        var local = map_T_surface.inverse().transformPoint(tcpWorld.translation);
        if (coverage != null) coverage.markFootprint(new Point2(local.x, local.y), spec.footprintRadius);
      }
    }
    stepIndex++;
  }
}
