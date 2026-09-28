package motionkit.robot;

import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.path.GeometricPath;
import motionkit.path.PathPoint;
import motionkit.planner.PathPlanningOptions;

/** Inputs to execute after a controlled stop, captured at command time. */
enum MotionRequest {
  Axes(targets:Array<AxisTarget>, options:Null<MotionOptions>);
  Linear(target:PathPoint, feed:Feed, options:Null<MotionOptions>);
  Path(path:GeometricPath, pathOptions:Null<PathPlanningOptions>,
    motionOptions:Null<MotionOptions>);
  Queued(command:QueuedMotionRequest);
  Jog(axisId:String, velocity:Float, durationSeconds:Float, acceleration:Float);
}
