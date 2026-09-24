package robotkit.runtime;

import robotkit.model.RobotModel;
import robotkit.model.JointType;
import robotkit.model.CollisionApproximation;
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
      robot.links.length, robot.frames.length, new RobotRuntimeIdentity(
        [for (link in robot.links) link.id],
        [for (joint in robot.joints) joint.id],
        robot.sensors.length == 0 ? ["joint_encoders", "imu", "lidar"] : [for (sensor in robot.sensors) sensor.id],
        [for (frame in robot.frames) frame.id],
        [for (frame in robot.frames) frame.link.id],
        [for (link in robot.links) link.visualGeometry],
        [for (link in robot.links) link.collisionGeometry], robot.collisionApproximation));
    result.collisionApproximation = switch (robot.collisionApproximation) {
      case CollisionApproximation.None: RobotKitRuntimeConstants.RK_COLLISION_APPROXIMATION_NONE;
      case CollisionApproximation.BoundsBox: RobotKitRuntimeConstants.RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
      default: throw 'Unknown collision approximation ${robot.collisionApproximation}';
    };
    for (index in 0...robot.links.length) {
      var link = robot.links[index];
      result.links[index] = new RobotRuntimeLinkBlueprint(link.mass, link.centerOfMass, link.inertiaTensor);
    }
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
        joint.limits.lower, joint.limits.upper, maxEffort, maxRate,
        joint.parentFramePosition, joint.parentFrameRotation,
        joint.childFramePosition, joint.childFrameRotation, joint.axis));
    }
    var children = [for (joint in robot.joints) joint.child];
    var root = robot.links[0];
    for (link in robot.links) if (children.indexOf(link) < 0) { root = link; break; }
    if (robot.sensors.length == 0) {
      for (sensor in RobotRuntimeSensorBlueprint.defaults(robot.links.indexOf(root), root.id))
        result.sensors.push(sensor);
    } else for (sensor in robot.sensors) {
      var frame = sensor.frame;
      var link = frame == null ? root : frame.link;
      result.sensors.push(new RobotRuntimeSensorBlueprint(sensor.id, sensor.kind,
        frame == null ? link.id : frame.id, link.id, robot.links.indexOf(link),
        frame == null ? [0.0, 0.0, 0.0] : frame.position,
        frame == null ? [0.0, 0.0, 0.0, 1.0] : frame.rotation,
        sensor.updateRate, sensor.rayCount, sensor.maxRange, sensor.noiseStddev, sensor.noiseSeed));
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
    if (robot.links.length > RobotKitRuntimeConstants.RK_MAX_LINKS)
      diagnostics.push(new RobotCompileDiagnostic("RK_LINK_LIMIT", "links",
        'robot contains ${robot.links.length} links, but the runtime supports at most ${RobotKitRuntimeConstants.RK_MAX_LINKS}'));
    if (robot.collisionApproximation != CollisionApproximation.None && robot.collisionApproximation != CollisionApproximation.BoundsBox)
      diagnostics.push(new RobotCompileDiagnostic("RK_COLLISION_APPROXIMATION", "collisionApproximation", "unsupported collision approximation"));

    var linkIds = new Map<String, Bool>();
    var linkNames = new Map<String, Bool>();
    for (index in 0...robot.links.length) {
      var link = robot.links[index];
      var path = 'links[$index]';
      if (link == null) {
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_NULL", path,
          "link is null"));
        continue;
      }
      if (link.id == null || link.id.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_ID", '$path.id',
          "link ID is empty"));
      else if (linkIds.exists(link.id))
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_ID_DUPLICATE", '$path.id',
          'duplicate link ID "${link.id}"'));
      else
        linkIds.set(link.id, true);
      if (link.name == null || link.name.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_NAME", '$path.name',
          "link name is empty"));
      else if (linkNames.exists(link.name))
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_DUPLICATE", '$path.name',
          'duplicate link name "${link.name}"'));
      else
        linkNames.set(link.name, true);
      if (!Math.isFinite(link.mass) || link.mass <= 0.0)
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_MASS", '$path.mass', "link mass must be finite and positive"));
      if (!validVector(link.centerOfMass, 3))
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_COM", '$path.centerOfMass', "center of mass requires three finite values"));
      if (!validInertia(link.inertiaTensor))
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_INERTIA", '$path.inertiaTensor', "inertia tensor must be finite, symmetric, and positive definite"));
      if (!validGeometryReference(link.visualGeometry) || !validGeometryReference(link.collisionGeometry))
        diagnostics.push(new RobotCompileDiagnostic("RK_LINK_GEOMETRY", path, "geometry reference must be null or a non-empty string"));
    }

    var jointIds = new Map<String, Bool>();
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
      if (joint.id == null || joint.id.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_ID", '$path.id',
          "joint ID is empty"));
      else if (jointIds.exists(joint.id))
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_ID_DUPLICATE", '$path.id',
          'duplicate joint ID "${joint.id}"'));
      else
        jointIds.set(joint.id, true);
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
      if (!validVector(joint.parentFramePosition, 3) || !validRotation(joint.parentFrameRotation)
          || !validVector(joint.childFramePosition, 3) || !validRotation(joint.childFrameRotation))
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_FRAME", path, "joint frames require finite translations and unit xyzw quaternions"));
      if (!validUnitVector(joint.axis))
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_AXIS", '$path.axis', "joint axis must be a finite unit vector"));
      // Cache the mutable nullable field before checking it. This makes the
      // narrowing explicit and keeps all actuator checks on one value.
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

    var frameIds = new Map<String, Bool>();
    for (index in 0...robot.frames.length) {
      var frame = robot.frames[index];
      var path = 'frames[$index]';
      if (frame == null) {
        diagnostics.push(new RobotCompileDiagnostic("RK_FRAME_NULL", path, "frame is null"));
        continue;
      }
      if (frame.id == null || frame.id.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_FRAME_ID", '$path.id', "frame ID is empty"));
      else if (frameIds.exists(frame.id))
        diagnostics.push(new RobotCompileDiagnostic("RK_FRAME_ID_DUPLICATE", '$path.id', "duplicate frame ID"));
      else frameIds.set(frame.id, true);
      if (frame.name == null || frame.name.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_FRAME_NAME", '$path.name', "frame name is empty"));
      if (frame.link == null || robot.links.indexOf(frame.link) < 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_FRAME_LINK", '$path.link', "frame link does not belong to this robot"));
      if (!validVector(frame.position, 3) || !validRotation(frame.rotation))
        diagnostics.push(new RobotCompileDiagnostic("RK_FRAME_POSE", path, "mount requires finite translation and unit xyzw quaternion"));
    }

    var sensorIds = new Map<String, Bool>();
    var sensorNames = new Map<String, Bool>();
    if (robot.sensors.length > RobotKitRuntimeConstants.RK_MAX_SENSORS)
      diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_LIMIT", "sensors", "too many sensors"));
    for (index in 0...robot.sensors.length) {
      var sensor = robot.sensors[index];
      var path = 'sensors[$index]';
      if (sensor == null) {
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_NULL", path,
          "sensor is null"));
        continue;
      }
      if (sensor.id == null || sensor.id.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_ID", '$path.id',
          "sensor ID is empty"));
      else if (sensorIds.exists(sensor.id))
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_ID_DUPLICATE", '$path.id',
          'duplicate sensor ID "${sensor.id}"'));
      else
        sensorIds.set(sensor.id, true);
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
      if (sensor.kind != "imu" && sensor.kind != "lidar" && sensor.kind != "joint_encoder")
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_UNSUPPORTED", '$path.kind', "unsupported sensor kind"));
      if (sensor.frame != null && robot.frames.indexOf(sensor.frame) < 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_FRAME", '$path.frame', "sensor frame does not belong to model"));
      if (!Math.isFinite(sensor.noiseStddev) || sensor.noiseStddev < 0.0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_NOISE", path, "invalid noise standard deviation"));
      if (sensor.kind == "lidar" && (sensor.rayCount < 1 || sensor.rayCount > RobotKitRuntimeConstants.RK_MAX_SENSOR_VALUES
          || !Math.isFinite(sensor.maxRange) || sensor.maxRange <= 0.0))
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_SCAN", path, "invalid LiDAR resolution or range"));
      if (!Math.isFinite(sensor.updateRate) || sensor.updateRate < 0.0)
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

  static function validVector(value:Array<Float>, count:Int):Bool {
    if (value == null || value.length != count) return false;
    for (v in value) if (!Math.isFinite(v)) return false;
    return true;
  }

  static function validRotation(value:Array<Float>):Bool {
    if (!validVector(value, 4)) return false;
    var norm = 0.0;
    for (v in value) norm += v*v;
    return Math.abs(norm - 1.0) < 0.000001;
  }

  static function validUnitVector(value:Array<Float>):Bool {
    if (!validVector(value, 3)) return false;
    var norm = 0.0;
    for (v in value) norm += v*v;
    return Math.abs(norm - 1.0) < 0.000001;
  }

  static function validInertia(value:Array<Float>):Bool {
    if (!validVector(value, 9)) return false;
    var scale = 1.0;
    for (v in value) if (Math.abs(v) > scale) scale = Math.abs(v);
    var epsilon = scale * 1e-10;
    if (Math.abs(value[1]-value[3]) > epsilon || Math.abs(value[2]-value[6]) > epsilon || Math.abs(value[5]-value[7]) > epsilon) return false;
    var minor2 = value[0]*value[4]-value[1]*value[3];
    var det = value[0]*(value[4]*value[8]-value[5]*value[7])-value[1]*(value[3]*value[8]-value[5]*value[6])+value[2]*(value[3]*value[7]-value[4]*value[6]);
    return value[0] > epsilon && minor2 > epsilon*epsilon && det > epsilon*epsilon*epsilon;
  }

  static function validGeometryReference(value:Null<String>):Bool
    return value == null || value.length > 0;
}
