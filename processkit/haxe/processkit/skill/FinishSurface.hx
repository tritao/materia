package processkit.skill;

import robotkit.skill.*;

import robotkit.localization.LocalizationState;
import robotkit.manipulation.BaseObstacle;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ToolPlanningContext;
import processkit.manipulation.WorkPatch;
import processkit.manipulation.WorkPatchPlanner;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Navigator;
import robotkit.perception.PerceptionSnapshot;
import robotkit.spatial.Transform3;
import robotkit.tool.ChannelToolAdapter;
import processkit.work.CoverageMap;
import processkit.work.Point2;
import processkit.work.WorkSurface;
import robotkit.core.Robot;
import robotkit.core.RobotSnapshot;

/**
 * Process parameters for one `FinishSurface` run: raster geometry
 * (`toolWidth`/`overlap`/`standoff`/`feedRate`/`leadInOut`, the same
 * `RasterToolpathGenerator` inputs `WorkPatchPlanner` forwards per patch),
 * `maxPatchWidth`/standoff-search band for `WorkPatchPlanner`, and the
 * Cartesian/IK tolerances used while compiling each patch plan. A
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
   * Largest allowed joint-space step accepted between
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
  AwaitingBaseStop;
  ExecutingPatch;
}

/**
 * Splits the surface into patches, navigates the base, and runs each patch
 * through a motion plan. Process events drive the tool adapter. Coverage is
 * measured from observed joint positions and the current base localization.
 */
class FinishSurface implements Skill {
  public final navigator:Navigator;
  public final manipulator:Manipulator;
  public final robot:Robot;
  public final map_T_surface:Transform3;
  public final surface:WorkSurface;
  public final spec:FinishSpec;
  public final observePerception:RobotSnapshot -> PerceptionSnapshot;
  public final runner:SurfacePlanRunner;
  public final toolAdapter:ChannelToolAdapter;
  public final processChannel:String;
  public final seed:Array<Float>;
  public final obstacles:Array<BaseObstacle>;
  public final toolPlanning:Null<ToolPlanningContext>;

  /** Cumulative coverage across every executed patch; available once planning succeeds. */
  public var coverage(default, null):Null<CoverageMap> = null;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var patches:Array<WorkPatch> = [];
  var patchIndex:Int = 0;
  var stage:FinishStage = Planning;
  var currentGoTo:Null<GoTo> = null;
  var eventIndex:Int = 0;
  var toolOn:Bool = false;

  public function new(navigator:Navigator, manipulator:Manipulator, robot:Robot,
      map_T_surface:Transform3, surface:WorkSurface, spec:FinishSpec,
      observePerception:RobotSnapshot -> PerceptionSnapshot,
      runner:SurfacePlanRunner, toolAdapter:ChannelToolAdapter, processChannel:String,
      seed:Array<Float>, ?obstacles:Array<BaseObstacle>, ?toolPlanning:ToolPlanningContext) {
    if (navigator == null || manipulator == null || robot == null || map_T_surface == null ||
        surface == null || spec == null || observePerception == null || runner == null ||
        toolAdapter == null || processChannel == null || seed == null)
      throw "FinishSurface requires navigation, a manipulator, a robot, a surface, a spec, and a seed";
    this.navigator = navigator;
    this.manipulator = manipulator;
    this.robot = robot;
    this.map_T_surface = map_T_surface;
    this.surface = surface;
    this.spec = spec;
    this.observePerception = observePerception;
    this.runner = runner;
    this.toolAdapter = toolAdapter;
    this.processChannel = processChannel;
    this.seed = seed.copy();
    this.obstacles = obstacles == null ? [] : obstacles;
    if (toolPlanning != null &&
        (toolPlanning.tool.flangeTTcp.translation.sub(manipulator.flangeTTcp.translation).norm() > 1e-6 ||
         toolPlanning.tool.flangeTTcp.rotation.angularDistance(manipulator.flangeTTcp.rotation) > 1e-6))
      throw "FinishSurface mounted tool TCP does not match the manipulator TCP";
    this.toolPlanning = toolPlanning;
  }

  public function start():Void {
    lifecycle.begin();
    try {
      var result = WorkPatchPlanner.plan(surface, map_T_surface, manipulator,
        spec.maxPatchWidth, spec.toolWidth, spec.overlap, spec.standoff, spec.feedRate, spec.leadInOut,
        spec.standoffDistanceMin, spec.standoffDistanceMax, seed, 3, 3, 0.4, obstacles,
        1e-4, 1e-3,
        toolPlanning == null ? null : toolPlanning.tool.collision,
        toolPlanning == null ? null : toolPlanning.obstacles,
        toolPlanning == null ? 0.0 : toolPlanning.clearance,
        toolPlanning == null ? null : toolPlanning.maxJointStep,
        toolPlanning == null ? 0.005 : toolPlanning.maxToolStep,
        toolPlanning == null ? null : toolPlanning.preparedShape);
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
          case SkillStatus.Succeeded: awaitBaseStop();
          case SkillStatus.Cancelled: lifecycle.cancel();
          case SkillStatus.Failed(message): lifecycle.fail(message);
          case SkillStatus.Idle: lifecycle.fail("navigation returned to idle");
        }
      case ExecutingPatch:
        try advanceExecution(snapshot, durationSeconds)
        catch (error:Dynamic) lifecycle.fail(Std.string(error));
      case AwaitingBaseStop:
        if (snapshot.safety == RobotKitRuntimeConstants.RK_SAFETY_READY)
          beginExecution();
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    if (currentGoTo != null) currentGoTo.cancel();
    try runner.abort() catch (_:Dynamic) {}
    lifecycle.cancel();
  }

  public function hold():Void if (stage == ExecutingPatch) runner.hold();
  public function resume():Void if (stage == ExecutingPatch) runner.resume();

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  function beginPatch(index:Int):Void {
    var patch = patches[index];
    var goTo = new GoTo(navigator,
      new NavigationGoal(patch.basePose, surface.frameId, 0.02, 0.02), observePerception);
    currentGoTo = goTo;
    stage = NavigatingToPatch;
    goTo.start();
    switch goTo.status() {
      case SkillStatus.Succeeded: awaitBaseStop();
      case SkillStatus.Failed(message): lifecycle.fail(message);
      case SkillStatus.Cancelled: lifecycle.cancel();
      case _:
    }
  }

  function awaitBaseStop():Void {
    stage = AwaitingBaseStop;
  }

  function beginExecution():Void {
    try {
      var patch = patches[patchIndex];
      var estimate = navigator.navigation.localization.state();
      if (estimate == null) throw "FinishSurface has no localization state to execute a patch from";
      var state:LocalizationState = cast estimate;
      var baseWorld = Transform3.fromPose2(state.pose, 0.0);
      var base_T_work = baseWorld.inverse().compose(map_T_surface);
      runner.runPatch(patch, base_T_work, seed);
      if (runner.failure() != null) throw runner.failure();
      eventIndex = 0;
      toolOn = false;
      stage = ExecutingPatch;
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  function advanceExecution(snapshot:RobotSnapshot, dtSeconds:Float):Void {
    runner.update(dtSeconds);
    var fired = runner.firedEvents();
    while (eventIndex < fired.length) {
      var event = fired[eventIndex++];
      toolAdapter.apply(event);
      if (event.channel == processChannel) toolOn = switch event.value {
        case Digital(enabled): enabled;
        case Analog(value): value > 0.0;
        case Process(_, _): false;
      };
    }
    if (runner.failure() != null) {
      lifecycle.fail(runner.failure());
      return;
    }
    if (toolOn) {
      var indices = runner.jointIndices();
      var q = [for (index in indices) snapshot.positions.get(index)];
      var estimate = navigator.navigation.localization.state();
      if (estimate != null) {
        var state:LocalizationState = cast estimate;
        var baseWorld = Transform3.fromPose2(state.pose, 0.0);
        var tcpWorld = baseWorld.compose(manipulator.tcpPose(q));
        var local = map_T_surface.inverse().transformPoint(tcpWorld.translation);
        if (coverage != null)
          coverage.markFootprint(new Point2(local.x, local.y), spec.footprintRadius);
      }
    }
    if (runner.completed()) {
      patchIndex++;
      if (patchIndex >= patches.length) {
        lifecycle.succeed('finished ${patches.length} patch(es)');
        return;
      }
      beginPatch(patchIndex);
      return;
    }
  }
}
