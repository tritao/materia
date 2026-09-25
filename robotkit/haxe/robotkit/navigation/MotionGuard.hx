package robotkit.navigation;

import robotkit.localization.LocalizationQuality;
import robotkit.localization.LocalizationState;
import robotkit.mobile.Footprint;
import robotkit.mobile.FootprintPoint;
import robotkit.mobile.Pose2;
import robotkit.mobile.Twist2;
import robotkit.perception.Obstacle;
import robotkit.perception.PerceptionSnapshot;
import robotkit.safety.StoppingEnvelope;

/**
 * Software collision-avoidance filter for a path follower. Perception obstacles
 * must be in the localization body or reference frame; other frames stop motion
 * until a frame-aware perception stage transforms them.
 */
class MotionGuard {
  public final navigation:Navigation;
  public final footprint:Footprint;
  public final reactionTimeSeconds:Float;
  public final decelerationMetersPerSecondSquared:Float;
  public final marginMeters:Float;
  public final slowdownDistanceMeters:Float;
  public var state(default, null):MotionGuardState = Blocked("No perception observation");

  final points:Array<FootprintPoint>;
  var observations:Null<PerceptionSnapshot> = null;
  var attached:Bool = true;

  public function new(navigation:Navigation, ?footprint:Footprint,
      ?reactionTimeSeconds:Float = 0.2,
      ?decelerationMetersPerSecondSquared:Float,
      ?marginMeters:Float = 0.1,
      ?slowdownDistanceMeters:Float = 0.5) {
    if (navigation == null) throw "MotionGuard requires Navigation";
    var selectedFootprint = footprint == null ? navigation.base.footprint : footprint;
    if (selectedFootprint == null) throw "MotionGuard requires a robot footprint";
    var chosenDeceleration = decelerationMetersPerSecondSquared == null
      ? navigation.base.motionLimits.maxLinearAcceleration
      : cast decelerationMetersPerSecondSquared;
    if (!Math.isFinite(reactionTimeSeconds) || reactionTimeSeconds < 0.0 ||
        !Math.isFinite(chosenDeceleration) || chosenDeceleration <= 0.0 ||
        !Math.isFinite(marginMeters) || marginMeters < 0.0 ||
        !Math.isFinite(slowdownDistanceMeters) || slowdownDistanceMeters < 0.0)
      throw "MotionGuard requires finite nonnegative margins and positive braking limits";
    this.navigation = navigation;
    this.footprint = cast selectedFootprint;
    this.reactionTimeSeconds = reactionTimeSeconds;
    this.decelerationMetersPerSecondSquared = chosenDeceleration;
    this.marginMeters = marginMeters;
    this.slowdownDistanceMeters = slowdownDistanceMeters;
    points = this.footprint.vertices();
    if (navigation.hasCommandFilter())
      throw "Navigation already has a command filter";
    navigation.setCommandFilter(filterCommand);
  }

  /** Updates the obstacle set and advances one navigation control iteration. */
  public function update(perception:PerceptionSnapshot,
      durationSeconds:Float):MotionGuardState {
    observations = perception;
    evaluate(navigation.base.currentCommand());
    navigation.update(durationSeconds);
    return state;
  }

  /** Updates localization and obstacle data from the same robot observation. */
  public function updateObservation(snapshot:robotkit.world.RobotSnapshot,
      perception:PerceptionSnapshot, durationSeconds:Float):MotionGuardState {
    observations = perception;
    evaluate(navigation.base.currentCommand());
    navigation.updateObservation(snapshot, durationSeconds);
    return state;
  }

  /** Removes this guard's command filter. */
  public function detach():Void {
    if (!attached) return;
    attached = false;
    navigation.setCommandFilter(null);
    observations = null;
    state = Clear;
  }

  function filterCommand(desired:Twist2):Twist2 return evaluate(desired);

  function evaluate(desired:Twist2):Twist2 {
    if (!attached) return desired;
    if (observations == null) return block("Perception observation is unavailable");
    var estimate:Null<LocalizationState> = navigation.localization.state();
    if (estimate == null || estimate.quality == LocalizationQuality.Invalid)
      return block("Localization is unavailable or invalid");
    var localization:LocalizationState = cast estimate;

    var current = navigation.base.currentCommand();
    var direction = desired.linear < -1e-9 ||
      (Math.abs(desired.linear) <= 1e-9 && current.linear < -1e-9) ? -1.0 : 1.0;
    var speed = Math.max(Math.abs(desired.linear), Math.abs(current.linear));
    var envelope = new StoppingEnvelope(speed, reactionTimeSeconds,
      decelerationMetersPerSecondSquared);
    var frontExtent = 0.0;
    var rearExtent = 0.0;
    var lateralExtent = 0.0;
    for (point in points) {
      var longitudinal = point.x * direction;
      frontExtent = Math.max(frontExtent, longitudinal);
      rearExtent = Math.max(rearExtent, -longitudinal);
      lateralExtent = Math.max(lateralExtent, Math.abs(point.y));
    }

    var nearest:Null<Obstacle> = null;
    var nearestClearance = 1.0e300;
    for (obstacle in observations.obstacles()) {
      var localPose:Pose2;
      if (obstacle.detection.frameId == localization.bodyFrame) {
        localPose = obstacle.detection.pose;
      } else if (obstacle.detection.frameId == localization.referenceFrame) {
        localPose = obstacle.detection.pose.relativeTo(localization.pose);
      } else {
        return block('Obstacle ${obstacle.detection.id} has unresolved frame ${obstacle.detection.frameId}');
      }

      var along = localPose.x * direction;
      if (Math.abs(localPose.y) > lateralExtent + obstacle.radiusMeters + marginMeters)
        continue;
      if (along + obstacle.radiusMeters < -rearExtent - marginMeters)
        continue;
      var clearance = along - frontExtent - obstacle.radiusMeters;
      if (clearance < nearestClearance) {
        nearest = obstacle;
        nearestClearance = clearance;
      }
    }

    if (nearest == null) {
      state = Clear;
      return desired;
    }
    if (nearestClearance <= marginMeters)
      return block('Obstacle ${nearest.detection.id} is inside the stopping margin');

    var slowdownRange = envelope.distanceMeters + slowdownDistanceMeters;
    if (nearestClearance >= marginMeters + slowdownRange) {
      state = Clear;
      return desired;
    }
    var scale = (nearestClearance - marginMeters) / slowdownRange;
    scale = Math.max(0.0, Math.min(1.0, scale));
    state = Approaching(nearest.detection.id, nearestClearance, scale);
    return new Twist2(desired.linear * scale, desired.angular * scale);
  }

  function block(reason:String):Twist2 {
    state = Blocked(reason);
    return Twist2.zero();
  }
}
