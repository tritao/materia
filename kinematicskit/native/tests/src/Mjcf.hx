import kinematicskit.JointKind;
import kinematicskit.KinematicModel;
import kinematicskit.Transform;

/**
 * MJCF for a kinematicskit model, for the mink oracle only: nested bodies
 * following the tree (each at `parent_T_joint · joint_T_child`, its joint
 * placed and oriented in the body frame), every frame as a site, unit
 * inertias. Couplings and closures are not representable and are rejected.
 */
class Mjcf {
  public static function write(model:KinematicModel):String {
    for (joint in 0...model.jointCount()) if (model.jointSource[joint] >= 0) throw "MJCF export does not support couplings";
    if (model.closureCount() > 0) throw "MJCF export does not support closures";
    var out = new StringBuf();
    out.add('<mujoco model="kinematicskit-oracle"><compiler angle="radian"/><worldbody>');
    for (body in model.bodyOrder) if (model.bodyParentJoint[body] < 0) writeBody(model, body, model.bodyRootPoses[body], -1, out);
    out.add('</worldbody></mujoco>');
    return out.toString();
  }

  static function writeBody(model:KinematicModel, body:Int, pose:Transform, joint:Int, out:StringBuf):Void {
    out.add('<body name="${model.bodyIds[body]}" pos="${pose.x} ${pose.y} ${pose.z}" quat="${pose.qw} ${pose.qx} ${pose.qy} ${pose.qz}">');
    out.add('<inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>');
    if (joint >= 0 && model.jointKind[joint] != JointKind.Fixed) {
      // The joint frame seen from the child body: (joint_T_child)⁻¹.
      var childTJoint = flat(model.jointJointTChild, joint).inverse();
      var a = childTJoint.transformVector(model.jointAxis[joint * 3], model.jointAxis[joint * 3 + 1], model.jointAxis[joint * 3 + 2]);
      var type = model.jointKind[joint] == JointKind.Revolute ? "hinge" : "slide";
      var dof = model.jointDof[joint];
      var lower = model.dofLower[dof], upper = model.dofUpper[dof];
      var range = Math.isFinite(lower) && Math.isFinite(upper) ? ' limited="true" range="$lower $upper"' : ' limited="false"';
      out.add('<joint name="${model.jointIds[joint]}" type="$type" pos="${childTJoint.x} ${childTJoint.y} ${childTJoint.z}" axis="${a.x} ${a.y} ${a.z}"$range/>');
    }
    for (frame in 0...model.frameCount()) if (model.frameBody[frame] == body) {
      var f = model.frameTransform(frame);
      out.add('<site name="${model.frameIds[frame]}" pos="${f.x} ${f.y} ${f.z}" quat="${f.qw} ${f.qx} ${f.qy} ${f.qz}"/>');
    }
    for (child in 0...model.jointCount()) if (model.jointParent[child] == body) {
      var place = flat(model.jointParentTJoint, child).compose(flat(model.jointJointTChild, child));
      writeBody(model, model.jointChild[child], place, child, out);
    }
    out.add('</body>');
  }

  static function flat(values:Array<Float>, index:Int):Transform
    return new Transform(values[index * 7], values[index * 7 + 1], values[index * 7 + 2], values[index * 7 + 3],
      values[index * 7 + 4], values[index * 7 + 5], values[index * 7 + 6]);
}
