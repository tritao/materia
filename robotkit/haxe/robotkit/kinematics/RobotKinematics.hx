package robotkit.kinematics;

import kinematicskit.JointKind;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.Transform;
import kinematicskit.Vector3;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.RobotModel;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/**
 * Compiles a `RobotModel` into a `kinematicskit.KinematicModel`. Compiled
 * IDs are the robot's own: body IDs are `LinkId`s, joint and DOF IDs are
 * `JointId`s, frame IDs are `FrameId`s. Joints follow ARCHITECTURE.md's
 * joint-frame convention: `child = parent · parentFrame · motion · childFrame⁻¹`.
 */
class RobotKinematics {
  /** The whole model: every link, joint, joint coupling and frame. */
  public static function compile(robot:RobotModel):KinematicModel {
    if (robot == null) throw "Robot kinematics requires a robot model";
    var builder = new KinematicModelBuilder();
    var bodies = new Map<String, Int>();
    for (link in robot.links) if (link != null) bodies.set(link.id, builder.addBody(link.id));
    for (joint in robot.joints) if (joint != null) addJoint(builder, joint, bodies);
    for (coupling in robot.couplings) builder.couple(coupling.follower, coupling.leader, coupling.ratio, coupling.offset);
    addFrames(builder, robot, bodies);
    return builder.build();
  }

  public static function toTransform(value:Transform3):Transform {
    var t = value.translation, r = value.rotation;
    return new Transform(t.x, t.y, t.z, r.x, r.y, r.z, r.w);
  }

  public static function toTransform3(value:Transform):Transform3
    return new Transform3(new Vec3(value.x, value.y, value.z), new Quat(value.qx, value.qy, value.qz, value.qw));

  static function addJoint(builder:KinematicModelBuilder, joint:Joint, bodies:Map<String, Int>):Void {
    var parent = joint.parent == null ? null : bodies.get(joint.parent.id);
    var child = joint.child == null ? null : bodies.get(joint.child.id);
    if (parent == null || child == null) throw 'Joint "${joint.id}" connects links that are not part of the model';
    var parentTJoint = transform(joint.parentFramePosition, joint.parentFrameRotation);
    var jointTChild = transform(joint.childFramePosition, joint.childFrameRotation).inverse();
    switch joint.type {
      case JointType.Fixed:
        builder.addJoint(joint.id, JointKind.Fixed, parent, child, parentTJoint, jointTChild);
      case JointType.Revolute, JointType.Continuous, JointType.Prismatic:
        var axis = unitAxis(joint);
        var kind = joint.type == JointType.Prismatic ? JointKind.Prismatic : JointKind.Revolute;
        var continuous = joint.type == JointType.Continuous;
        // `lower >= upper` is the model's convention for an unbounded joint (see JointGroup).
        var limited = !continuous && joint.limits != null && joint.limits.lower < joint.limits.upper;
        builder.addJoint(joint.id, kind, parent, child, parentTJoint, jointTChild, axis,
          limited ? joint.limits.lower : null, limited ? joint.limits.upper : null, 0.0, continuous);
      case other:
        throw 'Joint "${joint.id}" has unsupported kinematic chain type "$other"';
    }
  }

  static function addFrames(builder:KinematicModelBuilder, robot:RobotModel, bodies:Map<String, Int>):Void {
    for (frame in robot.frames) if (frame != null && frame.link != null) {
      var body = bodies.get(frame.link.id);
      if (body != null) builder.addFrame(frame.id, body, transform(frame.position, frame.rotation));
    }
  }

  /** Same arithmetic as `Transform3.fromArrays`, whose `Quat` normalizes its input. */
  static function transform(position:Array<Float>, rotation:Array<Float>):Transform
    return toTransform(Transform3.fromArrays(position, rotation));

  static function unitAxis(joint:Joint):Vector3 {
    if (joint.axis == null || joint.axis.length != 3)
      throw 'Joint "${joint.id}" has an invalid motion axis';
    var x = joint.axis[0], y = joint.axis[1], z = joint.axis[2];
    var norm = Math.sqrt(x * x + y * y + z * z);
    if (!Math.isFinite(norm) || Math.abs(norm - 1.0) > 1e-6)
      throw 'Joint "${joint.id}" motion axis must be unit length';
    return new Vector3(x, y, z);
  }
}
