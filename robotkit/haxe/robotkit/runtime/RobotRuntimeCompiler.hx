package robotkit.runtime;

import robotkit.model.Robot;
import robotkit.model.JointType;
import RobotKitRuntime;

/** Compiles the editable semantic robot model into an execution blueprint. */
class RobotRuntimeCompiler {
  /** Validates a model and captures its immutable source/revision boundary. */
  public static function compile(robot:Robot, ?revision:Int = 1):CompiledRobot {
    var errors = robot.validate();
    if (errors.length > 0)
      throw 'Cannot compile robot "${robot.name}": ${errors.join("; ")}';
    return new CompiledRobot(robot, revision);
  }

  /** Converts the semantic Haxeon model into one bulk native description. */
  public static function blueprint(compiled:CompiledRobot):RobotRuntimeBlueprint {
    var robot = compiled.source;
    var result = new RobotRuntimeBlueprint(compiled.revision, compiled.jointCount,
      robot.links.length);
    for (index in 0...robot.joints.length) {
      var joint = robot.joints[index];
      var parent = robot.links.indexOf(joint.parent);
      var child = robot.links.indexOf(joint.child);
      if (parent < 0 || child < 0)
        throw 'Joint ${joint.name} references a link outside robot ${robot.name}';
      var nativeType = switch (joint.type) {
        case JointType.Fixed: RobotKitRuntimeConstants.RK_RUNTIME_JOINT_FIXED;
        case JointType.Revolute, JointType.Continuous:
          RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE;
        case JointType.Prismatic: RobotKitRuntimeConstants.RK_RUNTIME_JOINT_PRISMATIC;
        case JointType.Floating:
          throw 'Joint ${joint.name} has unsupported floating type';
        default:
          throw 'Joint ${joint.name} has unknown type ${joint.type}';
      };
      var maxEffort = joint.drive == null
        ? joint.limits.effort
        : joint.drive.maxEffort;
      result.addJoint(new RobotRuntimeJointBlueprint(index, nativeType, parent, child,
        joint.limits.lower, joint.limits.upper, maxEffort));
    }
    return result;
  }
}
