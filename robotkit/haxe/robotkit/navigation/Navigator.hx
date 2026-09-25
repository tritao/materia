package robotkit.navigation;

import robotkit.localization.LocalizationQuality;
import robotkit.localization.LocalizationState;
import robotkit.mobile.Pose2;
import robotkit.perception.PerceptionSnapshot;

/**
 * Goal-level layer over a Planner and Navigation path follower. It refreshes
 * dynamic obstacles, replans when the remaining path becomes blocked, and
 * retries blocked goals after the map has had time to change.
 */
class Navigator {
  public final navigation:Navigation;
  public final planner:Planner;
  public final costmap:Costmap2;
  public final replanRetryIntervalSeconds:Float;
  public var status(default, null):NavigatorStatus = Idle;
  public var goal(default, null):Null<NavigationGoal> = null;
  public var activePath(default, null):Null<Path> = null;
  /** Number of replanning attempts after the initial plan. */
  public var replanCount(default, null):Int = 0;

  final motionGuard:Null<MotionGuard>;
  var blockedElapsedSeconds:Float = 0.0;

  public function new(navigation:Navigation, planner:Planner, costmap:Costmap2,
      ?replanRetryIntervalSeconds:Float = 0.5,
      ?motionGuard:MotionGuard) {
    if (navigation == null || planner == null || costmap == null ||
        !Math.isFinite(replanRetryIntervalSeconds) ||
        replanRetryIntervalSeconds <= 0.0)
      throw "Navigator requires navigation, planner, costmap, and a positive retry interval";
    if (motionGuard != null && motionGuard.navigation != navigation)
      throw "Navigator MotionGuard must filter its Navigation instance";
    this.navigation = navigation;
    this.planner = planner;
    this.costmap = costmap;
    this.replanRetryIntervalSeconds = replanRetryIntervalSeconds;
    this.motionGuard = motionGuard;
  }

  /** Plans from current localization and starts following the requested goal. */
  public function navigateTo(goal:NavigationGoal):NavigatorStatus {
    if (goal == null) throw "Navigator.navigateTo requires a goal";
    if (goal.frameId != costmap.grid.frameId)
      throw 'Navigator goal frame ${goal.frameId} does not match map frame ${costmap.grid.frameId}';
    stopFollower();
    this.goal = goal;
    activePath = null;
    replanCount = 0;
    blockedElapsedSeconds = 0.0;
    planRoute();
    return status;
  }

  /**
   * Updates dynamic obstacles and advances goal execution. Perception obstacles
   * must already be expressed in the costmap frame.
   */
  public function update(perception:PerceptionSnapshot,
      durationSeconds:Float):NavigatorStatus {
    if (perception == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0.0)
      throw "Navigator update requires a perception snapshot and positive finite duration";
    if (switch status {
      case Idle, Succeeded, Cancelled, Failed(_): true;
      case _: false;
    }) return status;

    try {
      costmap.setDynamicObstacles(perception.obstacles());
    } catch (error:Dynamic) {
      block(Std.string(error));
      return status;
    }
    if (motionGuard != null) motionGuard.updatePerception(perception);

    switch status {
      case Navigating:
        var path:Null<Path> = activePath;
        if (path == null || !remainingRouteIsClear(cast path)) {
          replanCount++;
          blockedElapsedSeconds = 0.0;
          if (!planRoute()) return status;
        }
      case Blocked(_):
        blockedElapsedSeconds += durationSeconds;
        if (blockedElapsedSeconds < replanRetryIntervalSeconds) return status;
        blockedElapsedSeconds = 0.0;
        replanCount++;
        if (!planRoute()) return status;
      case _:
        return status;
    }
    return advanceFollower(durationSeconds);
  }

  /** Stops execution and discards the active goal. */
  public function cancel():Void {
    stopFollower();
    goal = null;
    activePath = null;
    blockedElapsedSeconds = 0.0;
    status = Cancelled;
  }

  function planRoute():Bool {
    var activeGoal:Null<NavigationGoal> = goal;
    if (activeGoal == null) {
      status = Idle;
      return false;
    }
    try {
      var estimate:Null<LocalizationState> = navigation.localization.state();
      if (estimate == null || estimate.quality == LocalizationQuality.Invalid)
        throw "Navigator has no valid localization state";
      var state:LocalizationState = cast estimate;
      if (state.referenceFrame != activeGoal.frameId)
        throw 'Navigator localization frame ${state.referenceFrame} does not match goal frame ${activeGoal.frameId}';
      var planned = planner.plan(state.pose, activeGoal.pose);
      if (planned == null || planned.frameId != activeGoal.frameId)
        throw "Planner returned a path in the wrong frame";
      navigation.follow(planned, activeGoal);
      activePath = planned;
      status = Navigating;
      return true;
    } catch (error:Dynamic) {
      block(Std.string(error));
      return false;
    }
  }

  function advanceFollower(durationSeconds:Float):NavigatorStatus {
    switch navigation.update(durationSeconds) {
      case Succeeded:
        status = Succeeded;
      case Cancelled:
        status = Cancelled;
      case Failed(reason):
        status = Failed(reason);
      case Following:
        status = Navigating;
      case Idle:
        status = Failed("Navigation stopped without completing the active goal");
    }
    return status;
  }

  function remainingRouteIsClear(path:Path):Bool {
    var estimate:Null<LocalizationState> = navigation.localization.state();
    if (estimate == null || estimate.quality == LocalizationQuality.Invalid ||
        estimate.referenceFrame != costmap.grid.frameId)
      return false;
    var state:LocalizationState = cast estimate;
    if (!poseIsTraversable(state.pose)) return false;
    var startDistance = Math.max(0.0,
      Math.min(path.length, navigation.progressDistance));
    var remaining = path.length - startDistance;
    var sampleSpacing = costmap.grid.resolutionMeters * 0.5;
    var sampleCount = Std.int(Math.ceil(remaining / sampleSpacing));
    for (index in 0...sampleCount + 1) {
      var distance = Math.min(path.length, startDistance + index * sampleSpacing);
      if (!poseIsTraversable(path.poseAt(distance))) return false;
    }
    return true;
  }

  function poseIsTraversable(pose:Pose2):Bool {
    var cell = costmap.grid.worldToCell(pose);
    return cell != null && costmap.isTraversable(cell.x, cell.y);
  }

  function block(reason:String):Void {
    stopFollower();
    activePath = null;
    blockedElapsedSeconds = 0.0;
    status = Blocked(reason == null || reason.length == 0
      ? "Navigator could not plan a route"
      : reason);
  }

  function stopFollower():Void {
    if (switch navigation.status { case Following: true; case _: false; })
      navigation.cancel();
  }
}
