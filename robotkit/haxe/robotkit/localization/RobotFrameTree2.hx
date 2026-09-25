package robotkit.localization;

import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.mobile.Pose2;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeIdentity;
import robotkit.world.RobotSnapshot;

/** Builds a current planar frame tree from authored robot kinematics and joint state. */
class RobotFrameTree2 {
  /**
   * Resolves model links and frames from one robot snapshot. The model and
   * blueprint must describe the same revision. Nonplanar link/frame poses are
   * omitted because FrameTree2 cannot represent roll or pitch.
   */
  public static function fromSnapshot(model:RobotModel,
      blueprint:RobotRuntimeBlueprint, snapshot:RobotSnapshot,
      bodyLinkId:String):FrameTree2 {
    if (model == null || blueprint == null || snapshot == null ||
        bodyLinkId == null || bodyLinkId.length == 0)
      throw "Robot frame construction requires a model, blueprint, snapshot, and body link ID";
    if (blueprint.identity == null || blueprint.linkCount != model.links.length ||
        blueprint.jointCount != model.joints.length ||
        blueprint.frameCount != model.frames.length ||
        blueprint.joints.length != model.joints.length ||
        snapshot.positions.length != blueprint.jointCount)
      throw "Robot frame inputs do not describe the same compiled joint state";
    var identity:RobotRuntimeIdentity = cast blueprint.identity;

    var linkIds:Array<String> = [];
    var roots:Array<Link> = [];
    var children:Array<String> = [];
    for (link in model.links) {
      if (link == null || link.id == null || link.id.length == 0 ||
          linkIds.indexOf(link.id) >= 0)
        throw "Robot frame model has a null link or duplicate link ID";
      linkIds.push(link.id);
      var linkIndex = identity.linkIndex(link.id);
      if (linkIndex < 0 || identity.linkId(linkIndex) != link.id)
        throw 'Compiled robot identity is missing model link "${link.id}"';
    }
    if (linkIds.indexOf(bodyLinkId) < 0)
      throw 'Robot frame model has no body link "$bodyLinkId"';

    for (joint in model.joints) {
      if (joint == null || joint.id == null || joint.id.length == 0 ||
          joint.parent == null || joint.child == null ||
          linkIds.indexOf(joint.parent.id) < 0 || linkIds.indexOf(joint.child.id) < 0 ||
          model.links.indexOf(joint.parent) < 0 || model.links.indexOf(joint.child) < 0)
        throw "Robot frame model has a joint with a missing link";
      if (children.indexOf(joint.child.id) >= 0)
        throw 'Robot frame link "${joint.child.id}" has more than one parent joint';
      children.push(joint.child.id);
      var jointIndex = identity.jointIndex(joint.id);
      if (jointIndex < 0 || identity.jointId(jointIndex) != joint.id ||
          jointIndex >= blueprint.joints.length ||
          blueprint.joints[jointIndex].joint != jointIndex)
        throw 'Compiled robot identity is missing model joint "${joint.id}"';
    }
    for (link in model.links) if (children.indexOf(link.id) < 0) roots.push(link);
    if (roots.length != 1)
      throw 'Robot frame model must have one root link, found ${roots.length}';

    var rootFromLink = new Map<String, SpatialPose3>();
    rootFromLink.set(roots[0].id, SpatialPose3.identity());
    var jointResolved = [for (_ in model.joints) false];
    var unresolved = model.joints.length;
    while (unresolved > 0) {
      var madeProgress = false;
      for (index in 0...model.joints.length) {
        if (jointResolved[index]) continue;
        var joint:Joint = model.joints[index];
        if (!rootFromLink.exists(joint.parent.id)) continue;
        var jointIndex = identity.jointIndex(joint.id);
        var position = snapshot.positions.get(jointIndex);
        if (!Math.isFinite(position))
          throw 'Snapshot has a non-finite position for joint "${joint.id}"';
        var rootFromParent:SpatialPose3 = cast rootFromLink.get(joint.parent.id);
        var parentFromJoint = new SpatialPose3(joint.parentFramePosition,
          joint.parentFrameRotation);
        var childFromJoint = new SpatialPose3(joint.childFramePosition,
          joint.childFrameRotation);
        var motion = jointMotion(joint, position);
        var rootFromChild = rootFromParent.compose(parentFromJoint)
          .compose(motion).compose(childFromJoint.inverse());
        rootFromLink.set(joint.child.id, rootFromChild);
        jointResolved[index] = true;
        unresolved--;
        madeProgress = true;
      }
      if (!madeProgress)
        throw "Robot frame model has a disconnected or cyclic joint graph";
    }

    var bodyFromRoot:SpatialPose3 = cast rootFromLink.get(bodyLinkId);
    bodyFromRoot = bodyFromRoot.inverse();
    var result = new FrameTree2();
    var bodyFromLink = new Map<String, SpatialPose3>();
    for (link in model.links) {
      var rootFromCurrent:SpatialPose3 = cast rootFromLink.get(link.id);
      var transform = bodyFromRoot.compose(rootFromCurrent);
      bodyFromLink.set(link.id, transform);
      if (link.id == bodyLinkId) continue;
      var planar = transform.toPlanar();
      if (planar != null)
        result.add(new FrameTransform2(bodyLinkId, link.id, cast planar));
    }

    for (frame in model.frames) {
      if (frame == null || frame.id == null || frame.id.length == 0 ||
          frame.link == null || model.links.indexOf(frame.link) < 0 ||
          !bodyFromLink.exists(frame.link.id))
        throw "Robot frame model has a frame attached to a missing link";
      var frameIndex = identity.frameIndex(frame.id);
      if (frameIndex < 0 || identity.frameId(frameIndex) != frame.id ||
          identity.frameLinkId(frameIndex) != frame.link.id)
        throw 'Compiled robot identity is missing model frame "${frame.id}"';
      var linkFromFrame = new SpatialPose3(frame.position, frame.rotation);
      var bodyFromFrame:SpatialPose3 = cast bodyFromLink.get(frame.link.id);
      var transform = bodyFromFrame.compose(linkFromFrame).toPlanar();
      if (transform != null)
        result.add(new FrameTransform2(bodyLinkId, frame.id, transform));
    }
    return result;
  }

  static function jointMotion(joint:Joint, value:Float):SpatialPose3 {
    var axis = joint.axis;
    if (axis == null || axis.length != 3 || !Math.isFinite(axis[0]) ||
        !Math.isFinite(axis[1]) || !Math.isFinite(axis[2]))
      throw 'Joint "${joint.id}" has an invalid motion axis';
    var norm = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (Math.abs(norm - 1.0) > 1e-6)
      throw 'Joint "${joint.id}" motion axis must be unit length';
    return switch joint.type {
      case JointType.Fixed:
        SpatialPose3.identity();
      case JointType.Revolute, JointType.Continuous:
        var half = value * 0.5;
        new SpatialPose3([0.0, 0.0, 0.0], [axis[0] * Math.sin(half),
          axis[1] * Math.sin(half), axis[2] * Math.sin(half), Math.cos(half)]);
      case JointType.Prismatic:
        new SpatialPose3([axis[0] * value, axis[1] * value, axis[2] * value],
          [0.0, 0.0, 0.0, 1.0]);
      case _:
        throw 'Joint "${joint.id}" has an unsupported frame-motion type';
    };
  }
}

private class SpatialPose3 {
  public final position:Array<Float>;
  public final rotation:Array<Float>;

  public function new(position:Array<Float>, rotation:Array<Float>) {
    if (position == null || position.length != 3 || rotation == null ||
        rotation.length != 4)
      throw "Spatial transform requires a three-vector and xyzw quaternion";
    for (value in position) if (!Math.isFinite(value))
      throw "Spatial transform position must be finite";
    for (value in rotation) if (!Math.isFinite(value))
      throw "Spatial transform quaternion must be finite";
    var norm = Math.sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1] +
      rotation[2] * rotation[2] + rotation[3] * rotation[3]);
    if (!Math.isFinite(norm) || norm <= 1e-12)
      throw "Spatial transform quaternion must be non-zero";
    if (Math.abs(norm - 1.0) > 1e-6)
      throw "Spatial transform quaternion must be unit length";
    this.position = position.copy();
    this.rotation = [rotation[0] / norm, rotation[1] / norm,
      rotation[2] / norm, rotation[3] / norm];
  }

  public static function identity():SpatialPose3
    return new SpatialPose3([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

  public function compose(local:SpatialPose3):SpatialPose3 {
    var offset = rotate(rotation, local.position);
    return new SpatialPose3([
      position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]
    ], multiply(rotation, local.rotation));
  }

  public function inverse():SpatialPose3 {
    var inverseRotation = [-rotation[0], -rotation[1], -rotation[2], rotation[3]];
    var inversePosition = rotate(inverseRotation,
      [-position[0], -position[1], -position[2]]);
    return new SpatialPose3(inversePosition, inverseRotation);
  }

  public function toPlanar():Null<Pose2> {
    var qx = rotation[0];
    var qy = rotation[1];
    var qz = rotation[2];
    var qw = rotation[3];
    var verticalAlignment = 1.0 - 2.0 * (qx * qx + qy * qy);
    if (Math.abs(verticalAlignment - 1.0) > 1e-6) return null;
    var yaw = Math.atan2(2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));
    return new Pose2(position[0], position[1], yaw);
  }

  static function rotate(q:Array<Float>, vector:Array<Float>):Array<Float> {
    var x = q[0], y = q[1], z = q[2], w = q[3];
    var tx = 2.0 * (y * vector[2] - z * vector[1]);
    var ty = 2.0 * (z * vector[0] - x * vector[2]);
    var tz = 2.0 * (x * vector[1] - y * vector[0]);
    return [vector[0] + w * tx + y * tz - z * ty,
      vector[1] + w * ty + z * tx - x * tz,
      vector[2] + w * tz + x * ty - y * tx];
  }

  static function multiply(a:Array<Float>, b:Array<Float>):Array<Float> {
    return [
      a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
      a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
      a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
      a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]
    ];
  }
}
