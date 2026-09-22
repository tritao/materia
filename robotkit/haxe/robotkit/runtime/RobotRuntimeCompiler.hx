package robotkit.runtime;

import robotkit.model.RobotModel;
import robotkit.model.JointType;
import RobotKitRuntime;

/** Compiles the editable semantic robot model into an execution blueprint. */
class RobotRuntimeCompiler {
  /**
   * Validates a model and produces its immutable native execution blueprint.
   *
   * This is the domain-typing boundary: it checks topology and backend
   * support before assigning deterministic runtime indices.
   */
  public static function compile(robot:RobotModel, ?revision:Int = 1):RobotRuntimeBlueprint {
    var diagnostics = validate(robot, revision);
    if (diagnostics.length > 0)
      throw new RobotCompileException(robot == null ? "<null>" : robot.name, diagnostics);
    var result = new RobotRuntimeBlueprint(revision, robot.joints.length,
      robot.links.length);
    for (index in 0...robot.joints.length) {
      var joint:robotkit.model.Joint = robot.joints[index];
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
      var maxEffort = joint.limits.effort;
      var maxRate = joint.limits.velocity;
      var drive = joint.drive;
      if (drive != null) {
        maxEffort = drive.maxEffort;
        if (drive.maxRate > 0.0 && (maxRate == 0.0 || drive.maxRate < maxRate))
          maxRate = drive.maxRate;
      }
      result.addJoint(new RobotRuntimeJointBlueprint(index, nativeType, parent, child,
        joint.limits.lower, joint.limits.upper, maxEffort, maxRate));
    }
    return result;
  }

  /** Returns all semantic diagnostics without attempting native lowering. */
  public static function validate(robot:RobotModel, ?revision:Int = 1):Array<RobotCompileDiagnostic> {
    var diagnostics:Array<RobotCompileDiagnostic> = [];
    if (robot == null) {
      diagnostics.push(new RobotCompileDiagnostic("RK_MODEL_NULL", "robot",
        "robot model is null"));
      return diagnostics;
    }
    if (revision < 0)
      diagnostics.push(new RobotCompileDiagnostic("RK_REVISION", "revision",
        "runtime revision must be non-negative"));
    if (robot.name == null || robot.name.length == 0)
      diagnostics.push(new RobotCompileDiagnostic("RK_MODEL_NAME", "robot.name",
        "robot name is empty"));
    if (robot.links.length == 0)
      diagnostics.push(new RobotCompileDiagnostic("RK_TOPOLOGY_EMPTY", "links",
        "robot must contain at least one link"));
    if (robot.joints.length > RobotKitRuntimeConstants.RK_MAX_JOINTS)
      diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_LIMIT", "joints",
        'robot contains ${robot.joints.length} joints, but the runtime supports at most ${RobotKitRuntimeConstants.RK_MAX_JOINTS}'));

    var linkNames = new Map<String, Bool>();
    for (index in 0...robot.links.length) {
      var link = robot.links[index];
      var path = 'links[$index]';
      if (link == null) {
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_NULL", path,
          "link is null"));
        continue;
      }
      if (link.name == null || link.name.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_NAME", '$path.name',
          "link name is empty"));
      else if (linkNames.exists(link.name))
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_DUPLICATE", '$path.name',
          'duplicate link name "${link.name}"'));
      else
        linkNames.set(link.name, true);
    }

    var jointNames = new Map<String, Bool>();
    var adjacency:Array<Array<Int>> = [];
    var indegree:Array<Int> = [];
    for (_ in 0...robot.links.length) {
      adjacency.push([]);
      indegree.push(0);
    }

    for (index in 0...robot.joints.length) {
      var joint = robot.joints[index];
      var path = 'joints[$index]';
      if (joint == null) {
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_NULL", path,
          "joint is null"));
        continue;
      }
      if (joint.name == null || joint.name.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_NAME", '$path.name',
          "joint name is empty"));
      else if (jointNames.exists(joint.name))
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_DUPLICATE", '$path.name',
          'duplicate joint name "${joint.name}"'));
      else
        jointNames.set(joint.name, true);

      var parent = joint.parent == null ? -1 : robot.links.indexOf(joint.parent);
      var child = joint.child == null ? -1 : robot.links.indexOf(joint.child);
      if (joint.parent == null)
        diagnostics.push(new RobotCompileDiagnostic("RK_PARENT_NULL", '$path.parent',
          "joint parent is null"));
      else if (parent < 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_PARENT_FOREIGN", '$path.parent',
          "joint parent does not belong to this robot"));
      if (joint.child == null)
        diagnostics.push(new RobotCompileDiagnostic("RK_CHILD_NULL", '$path.child',
          "joint child is null"));
      else if (child < 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_CHILD_FOREIGN", '$path.child',
          "joint child does not belong to this robot"));
      if (parent >= 0 && child >= 0) {
        if (parent == child)
          diagnostics.push(new RobotCompileDiagnostic("RK_SELF_JOINT", path,
            "joint parent and child must be different links"));
        else {
          adjacency[parent].push(child);
          indegree[child]++;
        }
      }

      if (joint.limits == null)
        diagnostics.push(new RobotCompileDiagnostic("RK_LIMITS_NULL", '$path.limits',
          "joint limits are missing"));
      else {
        var limitError = joint.limits.validate();
        if (limitError != null)
          diagnostics.push(new RobotCompileDiagnostic("RK_LIMITS", '$path.limits', limitError));
      }
      var drive = joint.drive;
      if (drive != null) {
        if (drive.name == null || drive.name.length == 0)
          diagnostics.push(new RobotCompileDiagnostic("RK_ACTUATOR_NAME", '$path.drive.name',
            "actuator name is empty"));
        if (drive.maxEffort < 0.0)
          diagnostics.push(new RobotCompileDiagnostic("RK_ACTUATOR_EFFORT", '$path.drive.maxEffort',
            "actuator effort must be non-negative"));
        if (drive.maxRate < 0.0)
          diagnostics.push(new RobotCompileDiagnostic("RK_ACTUATOR_RATE", '$path.drive.maxRate',
            "actuator rate must be non-negative"));
      }
      if (joint.type == JointType.Floating)
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_UNSUPPORTED", '$path.type',
          "floating joints are not supported by the current runtime backend"));
      else if (joint.type != JointType.Fixed && joint.type != JointType.Revolute
          && joint.type != JointType.Continuous && joint.type != JointType.Prismatic)
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_TYPE", '$path.type',
          'unknown joint type "${joint.type}"'));
    }

    var sensorNames = new Map<String, Bool>();
    for (index in 0...robot.sensors.length) {
      var sensor = robot.sensors[index];
      var path = 'sensors[$index]';
      if (sensor == null) {
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_NULL", path,
          "sensor is null"));
        continue;
      }
      if (sensor.name == null || sensor.name.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_NAME", '$path.name',
          "sensor name is empty"));
      else if (sensorNames.exists(sensor.name))
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_DUPLICATE", '$path.name',
          'duplicate sensor name "${sensor.name}"'));
      else
        sensorNames.set(sensor.name, true);
      if (sensor.kind == null || sensor.kind.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_KIND", '$path.kind',
          "sensor kind is empty"));
      if (sensor.updateRate < 0.0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_RATE", '$path.updateRate',
          "sensor update rate must be non-negative"));
    }

    var roots:Array<Int> = [];
    for (index in 0...indegree.length)
      if (indegree[index] == 0) roots.push(index);
    if (robot.links.length > 0 && roots.length != 1)
      diagnostics.push(new RobotCompileDiagnostic("RK_TOPOLOGY_ROOT", "links",
        'robot must have exactly one root link, found ${roots.length}'));
    for (index in 0...indegree.length)
      if (indegree[index] > 1)
        diagnostics.push(new RobotCompileDiagnostic("RK_TOPOLOGY_PARENT", 'links[$index]',
          "link has more than one parent joint"));

    // Kahn's algorithm detects cycles without recursion, keeping validation
    // safe for imported models with very deep link chains.
    var remainingIndegree = indegree.copy();
    var queue:Array<Int> = [];
    for (index in 0...remainingIndegree.length)
      if (remainingIndegree[index] == 0) queue.push(index);
    var processed = 0;
    while (queue.length > 0) {
      var parent = queue.shift();
      processed++;
      for (child in adjacency[parent]) {
        remainingIndegree[child]--;
        if (remainingIndegree[child] == 0) queue.push(child);
      }
    }
    if (processed < robot.links.length)
      diagnostics.push(new RobotCompileDiagnostic("RK_TOPOLOGY_CYCLE", "joints",
        "joint graph contains a cycle"));
    if (roots.length == 1) {
      var reachable:Array<Bool> = [];
      for (_ in 0...robot.links.length) reachable.push(false);
      var pending:Array<Int> = [roots[0]];
      while (pending.length > 0) {
        var node = pending.pop();
        if (reachable[node]) continue;
        reachable[node] = true;
        for (child in adjacency[node]) pending.push(child);
      }
      for (index in 0...reachable.length)
        if (!reachable[index])
          diagnostics.push(new RobotCompileDiagnostic("RK_TOPOLOGY_DISCONNECTED", 'links[$index]',
            "link is not reachable from the robot root"));
    }
    return diagnostics;
  }
}
