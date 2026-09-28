package motionkit.robot;

import haxe.Int64;
import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.path.GeometricPath;
import motionkit.path.PathPoint;
import motionkit.planner.PathPlanningOptions;

/** Queued command payloads held behind a replacement stop. */
enum QueuedMotionRequest {
  Axes(targets:Array<AxisTarget>, options:Null<MotionOptions>);
  Linear(target:PathPoint, feed:Feed, options:Null<MotionOptions>);
  Path(path:GeometricPath, pathOptions:Null<PathPlanningOptions>,
    motionOptions:Null<MotionOptions>);
  Trajectory(segments:Array<{timeFromStartNs:Int64, durationNs:Int64,
    coefficients:Array<Array<Float>>}>);
}
