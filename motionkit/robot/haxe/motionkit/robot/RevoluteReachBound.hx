package motionkit.robot;

import robotkit.manipulation.KinematicGroup;
import robotkit.model.Joint;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** A necessary reach condition, independent of joint limits and IK seeds.
 * Fixed spans between revolute pivots retain their length at every posture.
 * Unsupported chains deliberately have no bound.
 */
class RevoluteReachBound {
  final centre:Vec3;
  final radius:Float;
  final tcpToPivot:Vec3;

  function new(centre:Vec3, radius:Float, tcpToPivot:Vec3) {
    this.centre = centre;
    this.radius = radius;
    this.tcpToPivot = tcpToPivot;
  }

  public static function of(group:KinematicGroup):Null<RevoluteReachBound> {
    if (group.workFrame != null) return null;
    var path:Array<Joint> = [];
    var link = group.flangeLink;
    while (link != group.rootLink) {
      var parent:Null<Joint> = null;
      for (joint in group.robot.joints) if (joint.child.id == link) {
        if (parent != null) return null;
        parent = joint;
      }
      if (parent == null || path.indexOf(parent) >= 0) return null;
      switch parent.type {
        case Fixed | Revolute | Continuous:
        default: return null;
      }
      path.unshift(parent);
      link = parent.parent.id;
    }
    var span = Transform3.identity();
    var centre:Null<Vec3> = null;
    var radius = 0.0;
    for (joint in path) {
      var parentFrame = Transform3.fromArrays(joint.parentFramePosition, joint.parentFrameRotation);
      var childFrame = Transform3.fromArrays(joint.childFramePosition, joint.childFrameRotation);
      var pivot = span.compose(parentFrame);
      if (joint.type == robotkit.model.JointType.Fixed) {
        span = pivot.compose(childFrame.inverse());
      } else {
        if (centre == null) centre = pivot.translation;
        else radius += pivot.translation.norm();
        // Subsequent spans are expressed at this pivot, not at the root.
        span = childFrame.inverse();
      }
    }
    if (centre == null) return null;
    for (frame in group.robot.frames) if (frame.id == group.flangeFrame) {
      var pivotToTcp = span.compose(Transform3.fromArrays(frame.position, frame.rotation))
        .compose(group.flangeTTcp);
      return new RevoluteReachBound(centre, radius, pivotToTcp.inverse().translation);
    }
    return null;
  }

  /** Reject only outside the sphere enlarged by the task's allowed errors. */
  public function excludes(target:Transform3, positionTolerance:Float,
      orientationTolerance:Float, fullOrientation:Bool, toolAxisFixed:Bool = false):Bool {
    var offset = tcpToPivot.norm();
    var point = fullOrientation ? target.transformPoint(tcpToPivot) : target.translation;
    var allowance = positionTolerance + (fullOrientation ? orientationTolerance * offset : offset);
    if (!fullOrientation && toolAxisFixed) {
      // Free spin changes only the perpendicular offset. The axial component
      // stays on the requested tool axis, up to its angular tolerance.
      point = target.transformPoint(new Vec3(0, 0, tcpToPivot.z));
      allowance = positionTolerance + Math.sqrt(tcpToPivot.x*tcpToPivot.x + tcpToPivot.y*tcpToPivot.y)
        + orientationTolerance*Math.abs(tcpToPivot.z);
    }
    // Outward numerical allowance for rigid-transform composition and norms.
    var rounding = 1e-12 * (1.0 + radius + offset + centre.norm() + point.norm());
    return point.sub(centre).norm() > radius + allowance + rounding;
  }
}
