package motionkit.robot;

import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.path.GeometricPath;
import motionkit.path.PathPoint;
import motionkit.planner.PathPlanningOptions;
import motionkit.trajectory.Trajectory;

/** Copies caller-owned command inputs before a stop can defer execution. */
class MotionRequestCapture {
  public static function axes(targets:Array<AxisTarget>,
      options:Null<MotionOptions>):MotionRequest
    return MotionRequest.Axes(copyTargets(targets), copyOptions(options));

  public static function linear(target:PathPoint, feed:Feed,
      options:Null<MotionOptions>):MotionRequest
    return MotionRequest.Linear(copyPoint(target), copyFeed(feed),
      copyOptions(options));

  public static function path(path:GeometricPath,
      pathOptions:Null<PathPlanningOptions>,
      motionOptions:Null<MotionOptions>):MotionRequest
    return MotionRequest.Path(copyPath(path), copyPathOptions(pathOptions),
      copyOptions(motionOptions));

  public static function jog(axisId:String, velocity:Float, durationSeconds:Float,
      acceleration:Float):MotionRequest
    return MotionRequest.Jog(axisId, velocity, durationSeconds, acceleration);

  public static function queuedAxes(targets:Array<AxisTarget>,
      options:Null<MotionOptions>):MotionRequest
    return MotionRequest.Queued(QueuedMotionRequest.Axes(copyTargets(targets),
      copyOptions(options)));

  public static function queuedLinear(target:PathPoint, feed:Feed,
      options:Null<MotionOptions>):MotionRequest
    return MotionRequest.Queued(QueuedMotionRequest.Linear(copyPoint(target),
      copyFeed(feed), copyOptions(options)));

  public static function queuedPath(path:GeometricPath,
      pathOptions:Null<PathPlanningOptions>,
      motionOptions:Null<MotionOptions>):MotionRequest
    return MotionRequest.Queued(QueuedMotionRequest.Path(copyPath(path),
      copyPathOptions(pathOptions), copyOptions(motionOptions)));

  public static function queuedTrajectory(value:Trajectory):MotionRequest
    return MotionRequest.Queued(QueuedMotionRequest.Trajectory(value.segments()));

  static function copyTargets(targets:Array<AxisTarget>):Array<AxisTarget>
    return targets == null ? null : [for (target in targets)
      target == null ? null : new AxisTarget(target.axis, target.position)];

  static function copyPoint(point:PathPoint):PathPoint
    return point == null ? null : new PathPoint(point.x, point.y, point.z);

  static function copyFeed(feed:Feed):Feed
    return feed == null ? null : new Feed(feed.value);

  static function copyOptions(options:MotionOptions):MotionOptions
    return options == null ? null : new MotionOptions(options.maxVelocity,
      options.maxAcceleration, options.maxJerk);

  static function copyPathOptions(options:PathPlanningOptions):PathPlanningOptions
    return options == null ? null : new PathPlanningOptions(options.exactStop,
      options.blendTolerance, options.maxBlendTurnAngleRadians);

  static function copyPath(path:GeometricPath):GeometricPath
    return path == null ? null : new GeometricPath(path.primitives);
}
