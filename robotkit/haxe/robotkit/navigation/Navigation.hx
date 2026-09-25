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
  /** Additional lookahead seconds; distance grows with the recent commanded speed. */
  public final lookaheadTime:Float;
  public final cruiseSpeed:Float;
  public final maxAngularSpeed:Float;
  public final maxLateralAcceleration:Float;
  public final allowReverse:Bool;
  public var status(default, null):NavigationStatus = Idle;
  public var progressDistance(default, null):Float = 0.0;
  /** Signed cross-track error, positive to the left of the path direction. */
  public var crossTrackError(default, null):Float = 0.0;

  var currentPath:Null<Path> = null;
  var currentTrajectory:Null<Trajectory> = null;
  var currentGoal:Null<NavigationGoal> = null;
  var commandedLinearSpeed:Float = 0.0;
  var trajectoryElapsedSeconds:Float = 0.0;

  public function new(base:MobileBase, localization:Localization,
      ?lookaheadDistance:Float = 0.5, ?cruiseSpeed:Float = 0.5,
      ?maxAngularSpeed:Float = 1.0, ?allowReverse:Bool = true,
      ?lookaheadTime:Float = 0.0, ?maxLateralAcceleration:Float = 1.0) {
    if (base == null || localization == null ||
        !Math.isFinite(lookaheadDistance) || lookaheadDistance <= 0.0 ||
        !Math.isFinite(cruiseSpeed) || cruiseSpeed <= 0.0 ||
        !Math.isFinite(maxAngularSpeed) || maxAngularSpeed <= 0.0 ||
        !Math.isFinite(lookaheadTime) || lookaheadTime < 0.0 ||
        !Math.isFinite(maxLateralAcceleration) || maxLateralAcceleration <= 0.0)
      throw "Navigation requires mobile and localization services with positive limits";
    this.base = base;
    this.localization = localization;
    this.lookaheadDistance = lookaheadDistance;
    this.lookaheadTime = lookaheadTime;
    this.cruiseSpeed = cruiseSpeed;
    this.maxAngularSpeed = maxAngularSpeed;
    this.maxLateralAcceleration = maxLateralAcceleration;
    this.allowReverse = allowReverse;
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
    currentTrajectory = null;
    currentGoal = target;
    progressDistance = 0.0;
    crossTrackError = 0.0;
    commandedLinearSpeed = 0.0;
    status = Following;
  }

  /** Starts tracking a time-parameterized trajectory with pose feedback. */
  public function followTrajectory(trajectory:Trajectory,
      ?goal:NavigationGoal):Void {
    if (trajectory == null) throw "Navigation.followTrajectory requires a trajectory";
    var target = goal == null
      ? new NavigationGoal(trajectory.goal(), trajectory.frameId)
      : goal;
    if (target.frameId != trajectory.frameId)
      throw "Navigation goal and trajectory frames must match";
    currentPath = null;
    currentTrajectory = trajectory;
    currentGoal = target;
    trajectoryElapsedSeconds = 0.0;
    progressDistance = 0.0;
    crossTrackError = 0.0;
    commandedLinearSpeed = 0.0;
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
    var goal:NavigationGoal = cast currentGoal;
    var frameId:String;
    if (currentTrajectory != null) frameId = currentTrajectory.frameId;
    else {
      var activePath:Path = cast currentPath;
      frameId = activePath.frameId;
    }
    if (state.referenceFrame != frameId || state.referenceFrame != goal.frameId)
      return fail('Navigation frame mismatch: state=${state.referenceFrame}, target=$frameId');

    var dx = goal.pose.x - state.pose.x;
    var dy = goal.pose.y - state.pose.y;
    var goalDistance = Math.pow(dx * dx + dy * dy, 0.5);
    var headingError = Pose2.wrapAngle(goal.pose.yaw - state.pose.yaw);
    if (goalDistance <= goal.positionTolerance &&
        Math.abs(headingError) <= goal.headingTolerance) {
      base.stop(StopMode.Normal);
      status = Succeeded;
      return status;
    }
    if (currentTrajectory != null) {
      var trajectory:Trajectory = cast currentTrajectory;
      if (goalDistance <= goal.positionTolerance &&
          trajectoryElapsedSeconds >= trajectory.durationSeconds) {
        if (!base.driveModel.supportsInPlaceRotation())
          return fail("Drive model cannot align final heading in place; include a final approach in the path");
        var angular = headingError * 2.0;
        if (angular > maxAngularSpeed) angular = maxAngularSpeed;
        if (angular < -maxAngularSpeed) angular = -maxAngularSpeed;
        return command(new Twist2(0.0, angular), durationSeconds);
      }
      return updateTrajectory(state, cast currentTrajectory, durationSeconds);
    }

    if (goalDistance <= goal.positionTolerance) {
      if (!base.driveModel.supportsInPlaceRotation())
        return fail("Drive model cannot align final heading in place; include a final approach in the path");
      var angular = headingError * 2.0;
      if (angular > maxAngularSpeed) angular = maxAngularSpeed;
      if (angular < -maxAngularSpeed) angular = -maxAngularSpeed;
      return command(new Twist2(0.0, angular), durationSeconds);
    }

    var path:Path = cast currentPath;
    var projection = path.project(state.pose, progressDistance);
    progressDistance = projection.distanceAlongPath;
    crossTrackError = projection.crossTrackError;
    var lookahead = Math.max(lookaheadDistance,
      Math.abs(commandedLinearSpeed) * lookaheadTime);
    var target = path.poseAt(Math.min(path.length, progressDistance + lookahead));
    var localTarget = target.relativeTo(state.pose);
    var distanceSquared = localTarget.x * localTarget.x + localTarget.y * localTarget.y;
    if (distanceSquared < 1e-9) return fail("Path lookahead collapsed at the robot pose");
    var curvature = 2.0 * localTarget.y / distanceSquared;
    var pathHeading = projection.pose.yaw;
    var pathDirection = Math.cos(Pose2.wrapAngle(projection.tangentYaw - pathHeading));
    var direction = allowReverse && pathDirection < 0.0 ? -1.0 : 1.0;
    var remainingDistance = Math.max(path.length - progressDistance,
      Math.max(0.0, goalDistance - goal.positionTolerance));
    var speed = Math.min(cruiseSpeed,
      Math.pow(2.0 * base.motionLimits.maxLinearAcceleration * remainingDistance, 0.5));
    if (Math.abs(curvature) > 1e-9) {
      speed = Math.min(speed, maxAngularSpeed / Math.abs(curvature));
      speed = Math.min(speed,
        Math.pow(maxLateralAcceleration / Math.abs(curvature), 0.5));
    }
    var linearSpeed = direction * speed;
    var angularRate = linearSpeed * curvature;
    if (angularRate > maxAngularSpeed) angularRate = maxAngularSpeed;
    if (angularRate < -maxAngularSpeed) angularRate = -maxAngularSpeed;
    return command(new Twist2(linearSpeed, angularRate), durationSeconds);
  }

  public function cancel():Void {
    if (isFollowing()) {
      base.stop(StopMode.Normal);
      commandedLinearSpeed = 0.0;
      trajectoryElapsedSeconds = 0.0;
      status = Cancelled;
    }
  }

  public function reset():Void {
    if (isFollowing()) base.stop(StopMode.Normal);
    currentPath = null;
    currentTrajectory = null;
    currentGoal = null;
    progressDistance = 0.0;
    crossTrackError = 0.0;
    commandedLinearSpeed = 0.0;
    trajectoryElapsedSeconds = 0.0;
    status = Idle;
  }

  function updateTrajectory(state:LocalizationState, trajectory:Trajectory,
      durationSeconds:Float):NavigationStatus {
    trajectoryElapsedSeconds = Math.min(trajectory.durationSeconds,
      trajectoryElapsedSeconds + durationSeconds);
    var target = trajectory.sampleAt(trajectoryElapsedSeconds);
    var localTarget = target.pose.relativeTo(state.pose);
    var distanceSquared = localTarget.x * localTarget.x + localTarget.y * localTarget.y;
    var linear = target.twist.linear + 1.5 * localTarget.x;
    if (linear > Math.min(cruiseSpeed, base.motionLimits.maxLinearSpeed))
      linear = Math.min(cruiseSpeed, base.motionLimits.maxLinearSpeed);
    if (linear < -Math.min(cruiseSpeed, base.motionLimits.maxLinearSpeed))
      linear = -Math.min(cruiseSpeed, base.motionLimits.maxLinearSpeed);
    var angular = target.twist.angular +
      2.0 * linear * localTarget.y / Math.max(distanceSquared, 1e-6) +
      2.0 * Pose2.wrapAngle(target.pose.yaw - state.pose.yaw);
    if (angular > maxAngularSpeed) angular = maxAngularSpeed;
    if (angular < -maxAngularSpeed) angular = -maxAngularSpeed;
    return command(new Twist2(linear, angular), durationSeconds);
  }

  function command(twist:Twist2, durationSeconds:Float):NavigationStatus {
    try {
      var accepted = base.command(twist, durationSeconds);
      commandedLinearSpeed = accepted.linear;
      return status;
    } catch (error:Dynamic) {
      return fail(Std.string(error));
    }
  }

  function fail(message:String):NavigationStatus {
    try base.stop(StopMode.Normal) catch (_:Dynamic) {}
    commandedLinearSpeed = 0.0;
    status = Failed(message == null || message.length == 0 ? "Navigation failed" : message);
    return status;
  }

  function isFollowing():Bool return switch status {
    case Following: true;
    case _: false;
  };
}
