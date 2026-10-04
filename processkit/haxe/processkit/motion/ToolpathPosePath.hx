package processkit.motion;

import motionkit.robot.*;

import motionkit.event.EventValue;
import motionkit.event.PathEvent;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PoseMath;
import motionkit.path.PosePath;
import motionkit.path.PosePrimitive;
import motionkit.path.PoseWaypoint;
import processkit.path.Toolpath;
import robotkit.spatial.Transform3;

/** Converts RobotKit authoring points into a framed MotionKit process path. */
class ToolpathPosePath {
  public static function convert(toolpath:Toolpath, channel:String,
      ?rotationWeight:Float = 0.1):PosePath {
    if (toolpath == null || toolpath.points.length < 2)
      throw "Toolpath conversion needs at least two points";
    var primitives:Array<PosePrimitive> = [];
    var events:Array<PathEvent> = [];
    var distance = 0.0;
    var wasOn = false;
    for (i in 0...(toolpath.points.length - 1)) {
      var current = toolpath.points[i];
      var next = toolpath.points[i+1];
      if (current.processOn != wasOn) {
        events.push(new PathEvent(distance, channel, EventValue.Digital(current.processOn)));
        wasOn = current.processOn;
      }
      var start = waypoint(current.work_T_tcp, current.positionTolerance,
        current.orientationTolerance);
      var end = waypoint(next.work_T_tcp, next.positionTolerance,
        next.orientationTolerance);
      if (PoseMath.distance(start.pose, end.pose) <= 1e-12 &&
          PoseMath.angle(start.pose, end.pose) <= 1e-12) continue;
      var segment = new PoseLine(start, end, OrientationPolicy.Interpolated,
        rotationWeight, next.feedRate);
      primitives.push(segment);
      distance += segment.length();
    }
    if (primitives.length == 0) throw "Toolpath conversion needs a nonzero path";
    if (wasOn) events.push(new PathEvent(distance, channel, EventValue.Digital(false)));
    return new PosePath(toolpath.frameId, primitives, events);
  }

  static function waypoint(transform:Transform3, positionTolerance:Float,
      orientationTolerance:Float):PoseWaypoint {
    var p = transform.translation, q = transform.rotation;
    return new PoseWaypoint(new Pose3(p.x, p.y, p.z, q.x, q.y, q.z, q.w),
      positionTolerance, orientationTolerance);
  }
}
