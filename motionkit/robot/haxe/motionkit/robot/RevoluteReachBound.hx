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
  final roundingScale:Int;

  function new(centre:Vec3, radius:Float, tcpToPivot:Vec3, roundingScale:Int) {
    this.centre = centre;
    this.radius = radius;
    this.tcpToPivot = tcpToPivot;
    this.roundingScale = roundingScale;
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
    var zero = Transform3.identity();
    var pivots:Array<Vec3> = [], axes:Array<Vec3> = [];
    for (joint in path) {
      var parentFrame = Transform3.fromArrays(joint.parentFramePosition, joint.parentFrameRotation);
      var childFrame = Transform3.fromArrays(joint.childFramePosition, joint.childFrameRotation);
      var pivot = zero.compose(parentFrame);
      if (joint.type != robotkit.model.JointType.Fixed) {
        pivots.push(pivot.translation);
        axes.push(pivot.rotation.rotate(Vec3.fromArray(joint.axis).normalized()));
      }
      zero = pivot.compose(childFrame.inverse());
    }
    if (pivots.length == 0) return null;
    var centre = pivots[0], radius = 0.0;
    if (pivots.length > 1) {
      var first = pivots[1].sub(centre);
      var height = axes[0].scale(axes[0].dot(first));
      centre = centre.add(height);
      first = first.sub(height);
      var b = axes[1];
      var axial = b.dot(first), planar = first.sub(b.scale(axial)).norm();
      var remaining = 0.0;
      for (i in 2...pivots.length) remaining += pivots[i].sub(pivots[i-1]).norm();
      radius = first.norm() + remaining;
      var rotationError = 0.0, positionError = 0.0;
      for (end in 2...pivots.length) {
        var carrier = end - 1;
        if (carrier > 1) {
          // Rodrigues' formula bounds ||R(u,t)-R(+/-b,t)|| by
          // 5*min(||u-b||,||u+b||), for every angle. Thus arbitrary
          // axes remain safe; parallel prefixes need no pattern threshold.
          rotationError += 5 * Math.min(axes[carrier].sub(b).norm(), axes[carrier].add(b).norm());
        }
        var span = pivots[end].sub(pivots[end-1]);
        var length = span.norm(), along = b.dot(span);
        positionError += rotationError * length;
        axial += along;
        planar += span.sub(b.scale(along)).norm();
        remaining = Math.max(0.0, remaining - length);
        radius = Math.min(radius, Math.sqrt(planar*planar + axial*axial) + positionError + remaining);
      }
    }
    for (frame in group.robot.frames) if (frame.id == group.flangeFrame) {
      var rootToTcp = zero.compose(Transform3.fromArrays(frame.position, frame.rotation))
        .compose(group.flangeTTcp);
      return new RevoluteReachBound(centre, radius,
        rootToTcp.inverse().transformPoint(pivots[pivots.length-1]), path.length + 1);
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
    var rounding = 1e-12 * roundingScale * (1.0 + radius + offset + centre.norm() + point.norm());
    return point.sub(centre).norm() > radius + allowance + rounding;
  }
}
