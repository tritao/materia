package robotkit.navigation;

import robotkit.localization.Localization;
import robotkit.localization.LocalizationQuality;
import robotkit.localization.LocalizationState;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.mobile.Twist2;
import robotkit.world.RobotSnapshot;
import robotkit.world.StopMode;

/** Application-frequency pure-pursuit path follower over localization and MobileBase. */
class Navigation {
  public final base:MobileBase;
  public final localization:Localization;
  public final lookaheadDistance:Float;
  public final cruiseSpeed:Float;
  public final maxAngularSpeed:Float;
  public var status(default, null):NavigationStatus = Idle;

  var currentPath:Null<Path> = null;
  var currentGoal:Null<NavigationGoal> = null;
  var progressDistance:Float = 0.0;

  public function new(base:MobileBase, localization:Localization,
      ?lookaheadDistance:Float = 0.5, ?cruiseSpeed:Float = 0.5,
      ?maxAngularSpeed:Float = 1.0) {
    if (base == null || localization == null ||
        !Math.isFinite(lookaheadDistance) || lookaheadDistance <= 0.0 ||
        !Math.isFinite(cruiseSpeed) || cruiseSpeed <= 0.0 ||
        !Math.isFinite(maxAngularSpeed) || maxAngularSpeed <= 0.0)
      throw "Navigation requires mobile and localization services with positive limits";
    this.base = base;
    this.localization = localization;
    this.lookaheadDistance = lookaheadDistance;
    this.cruiseSpeed = cruiseSpeed;
    this.maxAngularSpeed = maxAngularSpeed;
  }

  /** Starts following a framed polyline; the final waypoint supplies the default goal. */
  public function follow(path:Path, ?goal:NavigationGoal):Void {
    if (path == null) throw "Navigation.follow requires a path";
    var target = goal == null
      ? new NavigationGoal(path.goal(), path.frameId)
      : goal;
    if (target.frameId != path.frameId)
      throw "Navigation goal and path frames must match";
    currentPath = path;
    currentGoal = target;
    progressDistance = 0.0;
    status = Following;
  }

  /** Consumes one robot observation, updates localization, and advances the controller. */
  public function updateObservation(snapshot:RobotSnapshot, durationSeconds:Float):NavigationStatus {
    if (!isFollowing()) return status;
    try {
      localization.update(snapshot);
    } catch (error:Dynamic) {
      return fail(Std.string(error));
    }
    return update(durationSeconds);
  }

  /** Runs one control iteration using the localization service's latest state. */
  public function update(durationSeconds:Float):NavigationStatus {
    if (!isFollowing()) return status;
    if (!Math.isFinite(durationSeconds) || durationSeconds <= 0.0)
      return fail("Navigation update duration must be finite and positive");
    var estimate = localization.state();
    if (estimate == null) return fail("Navigation has no localization state");
    var state:LocalizationState = cast estimate;
    if (switch state.quality { case LocalizationQuality.Invalid: true; case _: false; })
      return fail("Localization quality is invalid");
    var path:Path = cast currentPath;
    var goal:NavigationGoal = cast currentGoal;
    if (state.referenceFrame != path.frameId || state.referenceFrame != goal.frameId)
      return fail('Navigation frame mismatch: state=${state.referenceFrame}, path=${path.frameId}');

    var dx = goal.pose.x - state.pose.x;
    var dy = goal.pose.y - state.pose.y;
    var goalDistance = Math.pow(dx * dx + dy * dy, 0.5);
    var headingError = Pose2.wrapAngle(goal.pose.yaw - state.pose.yaw);
    if (goalDistance <= goal.positionTolerance) {
      if (Math.abs(headingError) <= goal.headingTolerance) {
        base.stop(StopMode.Normal);
        status = Succeeded;
        return status;
      }
      if (!base.driveModel.supportsInPlaceRotation())
        return fail("Drive model cannot align final heading in place; include a final approach in the path");
      var angular = headingError * 2.0;
      if (angular > maxAngularSpeed) angular = maxAngularSpeed;
      if (angular < -maxAngularSpeed) angular = -maxAngularSpeed;
      return command(new Twist2(0.0, angular), durationSeconds);
    }

    progressDistance = path.nearestDistance(state.pose, progressDistance);
    var target = path.poseAt(Math.min(path.length, progressDistance + lookaheadDistance));
    var localTarget = target.relativeTo(state.pose);
    var distanceSquared = localTarget.x * localTarget.x + localTarget.y * localTarget.y;
    if (distanceSquared < 1e-9) return fail("Path lookahead collapsed at the robot pose");
    var curvature = 2.0 * localTarget.y / distanceSquared;
    var speed = Math.min(cruiseSpeed, Math.max(0.05, goalDistance * 1.5));
    if (Math.abs(curvature) > 1e-9)
      speed = Math.min(speed, maxAngularSpeed / Math.abs(curvature));
    var angularRate = speed * curvature;
    if (angularRate > maxAngularSpeed) angularRate = maxAngularSpeed;
    if (angularRate < -maxAngularSpeed) angularRate = -maxAngularSpeed;
    return command(new Twist2(speed, angularRate), durationSeconds);
  }

  public function cancel():Void {
    if (isFollowing()) {
      base.stop(StopMode.Normal);
      status = Cancelled;
    }
  }

  public function reset():Void {
    if (isFollowing()) base.stop(StopMode.Normal);
    currentPath = null;
    currentGoal = null;
    progressDistance = 0.0;
    status = Idle;
  }

  function command(twist:Twist2, durationSeconds:Float):NavigationStatus {
    try {
      base.command(twist, durationSeconds);
      return status;
    } catch (error:Dynamic) {
      return fail(Std.string(error));
    }
  }

  function fail(message:String):NavigationStatus {
    try base.stop(StopMode.Normal) catch (_:Dynamic) {}
    status = Failed(message == null || message.length == 0 ? "Navigation failed" : message);
    return status;
  }

  function isFollowing():Bool return switch status {
    case Following: true;
    case _: false;
  };
}
