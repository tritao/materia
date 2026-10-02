package robotkit.runtime;

import robotkit.model.RobotModel;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;
import robotkit.model.CollisionApproximation;
import robotkit.model.RobotDriveConfiguration;
import robotkit.model.RobotForkConfiguration;
import robotkit.model.RobotMobileConfiguration;
import robotkit.model.Transmission;
import RobotKitRuntime;

/** Compiles the editable semantic robot model into an execution blueprint. */
class RobotRuntimeCompiler {
  /**
   * Validates a model and produces its immutable native execution blueprint.
   *
   * This is the domain-typing boundary: it checks topology and backend
   * support before assigning deterministic runtime indices.
   */
  public static function compile(robot:RobotModel, ?revision:Int = 1,
      ?calibrationRevision:Int = 0):RobotRuntimeBlueprint {
    var diagnostics = validate(robot, revision, calibrationRevision);
    if (diagnostics.length > 0)
      throw new RobotCompileException(robot == null ? "<null>" : robot.name, diagnostics);
    // A robot whose authored sensors are all external still gets the native
    // runtime's default sensor slots, ahead of its authored ones.
    var authoredNativeSensors = false;
    for (sensor in robot.sensors)
      if (!RobotRuntimeSensorBlueprint.isExternalKind(sensor.kind)) authoredNativeSensors = true;
    var sensorIdentityIds = [for (sensor in robot.sensors) sensor.id];
    if (!authoredNativeSensors)
      sensorIdentityIds = ["joint_encoders", "imu", "lidar"].concat(sensorIdentityIds);
    var result = new RobotRuntimeBlueprint(revision, robot.joints.length,
      robot.links.length, robot.frames.length, new RobotRuntimeIdentity(
        [for (link in robot.links) link.id],
        [for (joint in robot.joints) joint.id],
        sensorIdentityIds,
        [for (frame in robot.frames) frame.id],
        [for (frame in robot.frames) frame.link.id],
        [for (link in robot.links) link.visualGeometry],
        [for (link in robot.links) link.collisionGeometry], robot.collisionApproximation),
      compileConfiguration(robot), calibrationRevision);
    result.floatingBase = robot.floatingBase;
    result.collisionApproximation = switch (robot.collisionApproximation) {
      case CollisionApproximation.None: RobotKitRuntimeConstants.RK_COLLISION_APPROXIMATION_NONE;
      case CollisionApproximation.BoundsBox: RobotKitRuntimeConstants.RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
      default: throw 'Unknown collision approximation ${robot.collisionApproximation}';
    };
    for (index in 0...robot.links.length) {
      var link = robot.links[index];
      result.links[index] = new RobotRuntimeLinkBlueprint(link.mass, link.centerOfMass, link.inertiaTensor);
      for (shape in link.collisionShapes)
        result.linkCollisionShapes.push(new RobotRuntimeLinkShape(index, shape));
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
          throw 'Joint ${joint.name} has unsupported floating type; set RobotModel.floatingBase';
        default:
          throw 'Joint ${joint.name} has unknown type ${joint.type}';
      };
      var maxEffort = joint.limits.effort;
      var maxRate = joint.limits.velocity;
      var actuatorEffort = 0.0;
      var actuatorRate = 0.0;
      for (actuator in robot.actuators) switch actuator.transmission {
        case SimpleTransmission(jointId, ratio, _) if (jointId == joint.id):
          var magnitude = Math.abs(ratio);
          // Ideal lossless transmission: joint rate = actuator rate / |ratio|,
          // and joint effort = actuator effort * |ratio|.
          if (actuator.maxRate > 0.0)
            actuatorRate = tighterLimit(actuatorRate, actuator.maxRate / magnitude);
          if (actuator.maxEffort > 0.0)
            actuatorEffort += actuator.maxEffort * magnitude;
        case _:
      }
      // The joints coupled to this one and their motors limit it too, such as an axis by the
      // motors turning its lead screws (see RobotModel.coupledLimits).
      var coupled = robot.coupledLimits(joint.id);
      maxRate = tighterLimit(tighterLimit(maxRate, actuatorRate), coupled.velocity);
      maxEffort = tighterLimit(maxEffort, actuatorEffort);
      var compiled = new RobotRuntimeJointBlueprint(index, nativeType, parent, child,
        joint.limits.lower, joint.limits.upper, maxEffort, maxRate,
        joint.parentFramePosition, joint.parentFrameRotation,
        joint.childFramePosition, joint.childFrameRotation, joint.axis,
        coupled.maxAcceleration);
      compiled.overtravel = joint.limits.overtravel;
      compiled.armature = joint.armature;
      compiled.damping = joint.damping;
      compiled.frictionLoss = joint.frictionLoss;
      compiled.limitTimeConstant = joint.limitTimeConstant;
      compiled.limitDampingRatio = joint.limitDampingRatio;
      compiled.limitImpedance = joint.limitImpedance.copy();
      result.addJoint(compiled);
    }
    // Contact pairs refer to shapes by their place in linkCollisionShapes.
    var firstShape = new Map<String, Int>();
    var shapeCount = 0;
    for (link in robot.links) {
      firstShape.set(link.id, shapeCount);
      shapeCount += link.collisionShapes.length;
    }
    for (pair in robot.contactPairs) {
      // Validation guarantees both links exist.
      var firstA:Int = cast firstShape.get(pair.linkA), firstB:Int = cast firstShape.get(pair.linkB);
      result.contactPairs.push(new RobotRuntimeContactPair(firstA + pair.shapeA,
        firstB + pair.shapeB, pair.surface));
    }
    for (coupling in robot.couplings) {
      var leader = -1, follower = -1;
      for (index in 0...robot.joints.length) {
        if (robot.joints[index].id == coupling.leader) leader = index;
        if (robot.joints[index].id == coupling.follower) follower = index;
      }
      result.couplings.push(new RobotRuntimeJointCouplingBlueprint(
        leader, follower, coupling.ratio, coupling.offset));
    }
    var children = [for (joint in robot.joints) joint.child];
    var root = robot.links[0];
    for (link in robot.links) if (children.indexOf(link) < 0) { root = link; break; }
    if (!authoredNativeSensors)
      for (sensor in RobotRuntimeSensorBlueprint.defaults(robot.links.indexOf(root), root.id))
        result.sensors.push(sensor);
    for (sensor in robot.sensors) {
      var frame = sensor.frame;
      var link = frame == null ? root : frame.link;
      result.sensors.push(new RobotRuntimeSensorBlueprint(sensor.id, sensor.kind,
        frame == null ? link.id : frame.id, link.id, robot.links.indexOf(link),
        frame == null ? [0.0, 0.0, 0.0] : frame.position,
        frame == null ? [0.0, 0.0, 0.0, 1.0] : frame.rotation,
        sensor.updateRate, sensor.rayCount, sensor.maxRange, sensor.noiseStddev, sensor.noiseSeed,
        sensor.startAngleRadians, sensor.fieldOfViewRadians));
    }
    return result;
  }

  static function tighterLimit(first:Float, second:Float):Float {
    if (first == 0.0) return second;
    if (second == 0.0) return first;
    return Math.min(first, second);
  }

  /** Returns all semantic diagnostics without attempting native lowering. */
  public static function validate(robot:RobotModel, ?revision:Int = 1,
      ?calibrationRevision:Int = 0):Array<RobotCompileDiagnostic> {
    var diagnostics:Array<RobotCompileDiagnostic> = [];
    if (robot == null) {
      diagnostics.push(new RobotCompileDiagnostic("RK_MODEL_NULL", "robot",
        "robot model is null"));
      return diagnostics;
    }
    if (revision < 0)
      diagnostics.push(new RobotCompileDiagnostic("RK_REVISION", "revision",
        "runtime revision must be non-negative"));
    if (calibrationRevision < 0)
      diagnostics.push(new RobotCompileDiagnostic("RK_CALIBRATION_REVISION", "calibrationRevision",
        "runtime calibration revision must be non-negative"));
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
      if (link.collisionShapes == null)
        diagnostics.push(new RobotCompileDiagnostic("RK_COLLISION_SHAPE", '$path.collisionShapes',
          "collision shape list is missing"));
      else for (shapeIndex in 0...link.collisionShapes.length) {
        var shape = link.collisionShapes[shapeIndex];
        var error = shape == null ? "collision shape is null" : shape.validate();
        if (error != null)
          diagnostics.push(new RobotCompileDiagnostic("RK_COLLISION_SHAPE",
            '$path.collisionShapes[$shapeIndex]', error));
      }
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
      if (!(joint.armature >= 0.0) || !(joint.damping >= 0.0) || !(joint.frictionLoss >= 0.0) ||
          !Math.isFinite(joint.armature) || !Math.isFinite(joint.damping) || !Math.isFinite(joint.frictionLoss) ||
          !(joint.limitTimeConstant >= 0.0) || !(joint.limitDampingRatio >= 0.0) ||
          !Math.isFinite(joint.limitTimeConstant) || !Math.isFinite(joint.limitDampingRatio) ||
          !validVector(joint.limitImpedance, 5))
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_DYNAMICS", path,
          "joint armature, damping and friction loss must be finite and non-negative"));
      if (joint.type == JointType.Floating)
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_UNSUPPORTED", '$path.type',
          "floating joints are not supported; set RobotModel.floatingBase to free the root link"));
      else if (joint.type != JointType.Fixed && joint.type != JointType.Revolute
          && joint.type != JointType.Continuous && joint.type != JointType.Prismatic)
        diagnostics.push(new RobotCompileDiagnostic("RK_JOINT_TYPE", '$path.type',
          'unknown joint type "${joint.type}"'));
    }

    for (index in 0...robot.contactPairs.length) {
      var pair = robot.contactPairs[index];
      var path = 'contactPairs[$index]';
      var a:Null<robotkit.model.Link> = null, b:Null<robotkit.model.Link> = null;
      for (link in robot.links) if (link != null) {
        if (pair != null && link.id == pair.linkA) a = link;
        if (pair != null && link.id == pair.linkB) b = link;
      }
      if (pair == null || a == null || b == null || a == b || pair.shapeA < 0 || pair.shapeB < 0 ||
          pair.shapeA >= a.collisionShapes.length || pair.shapeB >= b.collisionShapes.length)
        diagnostics.push(new RobotCompileDiagnostic("RK_CONTACT_PAIR", path,
          "contact pair must name one shape on each of two different links"));
      else {
        var error = pair.surface == null ? "contact pair surface is missing" : pair.surface.validate();
        if (error != null) diagnostics.push(new RobotCompileDiagnostic("RK_CONTACT_PAIR", path, error));
      }
    }

    var actuatorIds = new Map<String, Bool>();
    if (robot.couplings.length > RobotKitRuntimeConstants.RK_MAX_JOINT_COUPLINGS)
      diagnostics.push(new RobotCompileDiagnostic("RK_COUPLING_LIMIT", "couplings", "too many joint couplings"));
    var couplingIds = new Map<String, Bool>();
    var followers = new Map<String, Bool>();
    for (index in 0...robot.couplings.length) {
      var coupling = robot.couplings[index];
      var path = 'couplings[$index]';
      if (coupling == null) {
        diagnostics.push(new RobotCompileDiagnostic("RK_COUPLING_NULL", path, "joint coupling is null"));
        continue;
      }
      if (couplingIds.exists(coupling.id))
        diagnostics.push(new RobotCompileDiagnostic("RK_COUPLING_ID", path, "duplicate joint coupling ID"));
      couplingIds.set(coupling.id, true);
      if (followers.exists(coupling.follower))
        diagnostics.push(new RobotCompileDiagnostic("RK_COUPLING_FOLLOWER", path, "joint has multiple leaders"));
      followers.set(coupling.follower, true);
      if (!jointIds.exists(coupling.leader) || !jointIds.exists(coupling.follower))
        diagnostics.push(new RobotCompileDiagnostic("RK_COUPLING_JOINT", path, "coupling references an unknown joint"));
      for (joint in robot.joints) if (joint != null &&
          (joint.id == coupling.leader || joint.id == coupling.follower) &&
          joint.type == JointType.Fixed)
        diagnostics.push(new RobotCompileDiagnostic("RK_COUPLING_FIXED", path,
          "joint coupling requires movable joints"));
      if (!Math.isFinite(coupling.ratio) || coupling.ratio == 0.0 || !Math.isFinite(coupling.offset))
        diagnostics.push(new RobotCompileDiagnostic("RK_COUPLING_VALUE", path, "invalid coupling ratio or offset"));
    }
    for (index in 0...robot.actuators.length) {
      var actuator = robot.actuators[index];
      var path = 'actuators[$index]';
      if (actuator == null) {
        diagnostics.push(new RobotCompileDiagnostic("RK_ACTUATOR_NULL", path,
          "actuator is null"));
        continue;
      }
      if (actuator.id == null || actuator.id.length == 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_ACTUATOR_ID", '$path.id',
          "actuator ID is empty"));
      else if (actuatorIds.exists(actuator.id))
        diagnostics.push(new RobotCompileDiagnostic("RK_ACTUATOR_ID_DUPLICATE", '$path.id',
          'duplicate actuator ID "${actuator.id}"'));
      else actuatorIds.set(actuator.id, true);
      if (!Math.isFinite(actuator.maxEffort) || actuator.maxEffort < 0.0 ||
          !Math.isFinite(actuator.maxRate) || actuator.maxRate < 0.0)
        diagnostics.push(new RobotCompileDiagnostic("RK_ACTUATOR_LIMIT", path,
          "actuator limits must be finite and non-negative"));
      if (actuator.transmission == null)
        diagnostics.push(new RobotCompileDiagnostic("RK_TRANSMISSION_NULL", '$path.transmission',
          "actuator transmission is missing"));
      else switch actuator.transmission {
        case SimpleTransmission(jointId, ratio, offset):
          if (!jointIds.exists(jointId))
            diagnostics.push(new RobotCompileDiagnostic("RK_TRANSMISSION_JOINT",
              '$path.transmission.jointId', 'unknown joint ID "$jointId"'));
          if (!Math.isFinite(ratio) || ratio == 0.0 || !Math.isFinite(offset))
            diagnostics.push(new RobotCompileDiagnostic("RK_TRANSMISSION_VALUE",
              '$path.transmission', "transmission ratio must be finite and nonzero, and offset finite"));
      }
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

    // The sensors' values share one pool in every state, so what they report together must fit it.
    var pooledValues = 0;
    for (sensor in robot.sensors) if (sensor != null)
      pooledValues += sensor.kind == "lidar" ? sensor.rayCount : sensor.kind == "imu" ? 6
        : sensor.kind == "joint_encoder" ? robot.joints.length : 0;
    if (pooledValues > RobotKitRuntimeConstants.RK_SENSOR_VALUE_POOL)
      diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_VALUES", "sensors", "sensors report more values together than a state holds"));
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
      if (!RobotRuntimeSensorBlueprint.isNativeKind(sensor.kind) &&
          !RobotRuntimeSensorBlueprint.isExternalKind(sensor.kind))
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_UNSUPPORTED", '$path.kind', "unsupported sensor kind"));
      if (sensor.frame != null && robot.frames.indexOf(sensor.frame) < 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_FRAME", '$path.frame', "sensor frame does not belong to model"));
      if (!Math.isFinite(sensor.noiseStddev) || sensor.noiseStddev < 0.0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_NOISE", path, "invalid noise standard deviation"));
      if (sensor.kind == "lidar" && (sensor.rayCount < 1 || sensor.rayCount > RobotKitRuntimeConstants.RK_MAX_SENSOR_VALUES
          || !Math.isFinite(sensor.maxRange) || sensor.maxRange <= 0.0
          || !Math.isFinite(sensor.startAngleRadians) || !Math.isFinite(sensor.fieldOfViewRadians)
          || sensor.fieldOfViewRadians <= 0.0 || sensor.fieldOfViewRadians > Math.PI * 2.0 + 1e-6))
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_SCAN", path,
          "invalid LiDAR resolution, range, or angular coverage"));
      if (!Math.isFinite(sensor.updateRate) || sensor.updateRate < 0.0)
        diagnostics.push(new RobotCompileDiagnostic("RK_SENSOR_RATE", '$path.updateRate',
          "sensor update rate must be non-negative"));
    }

    validateUserConfiguration(robot, diagnostics);

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

  static function validateUserConfiguration(robot:RobotModel,
      diagnostics:Array<RobotCompileDiagnostic>):Void {
    var roleOwners = new Map<String, String>();
    function resolveRole(id:Null<String>, path:String,
        allowed:Array<JointType>):Null<Int> {
      if (id == null || id.length == 0) {
        diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_JOINT", path,
          "mechanism role requires a joint ID"));
        return null;
      }
      var index = -1;
      for (i in 0...robot.joints.length) {
        var candidate = robot.joints[i];
        if (candidate != null && candidate.id == id) { index = i; break; }
      }
      if (index < 0) {
        diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_JOINT", path,
          'joint ID "$id" is not part of this robot'));
        return null;
      }
      var joint = robot.joints[index];
      if (allowed.indexOf(joint.type) < 0)
        diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_TYPE", path,
          'joint "${joint.name}" has type ${joint.type}, which cannot fill this role'));
      var previous = roleOwners.get(id);
      if (previous != null)
        diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_DUPLICATE", path,
          'joint "$id" is already assigned to $previous'));
      else
        roleOwners.set(id, path);
      return index;
    }
    function positive(value:Float, path:String):Void {
      if (!Math.isFinite(value) || value <= 0.0)
        diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_VALUE", path,
          "value must be finite and positive"));
    }

    var mobile = robot.mobileBase;
    if (mobile != null && robot.floatingBase)
      diagnostics.push(new RobotCompileDiagnostic("RK_FLOATING_MOBILE", "mobileBase",
        "a floating base moves under physics and cannot also be a wheeled mobile base"));
    if (mobile != null) {
      positive(mobile.maxLinearSpeed, "mobileBase.maxLinearSpeed");
      positive(mobile.maxAngularSpeed, "mobileBase.maxAngularSpeed");
      positive(mobile.maxLinearAcceleration, "mobileBase.maxLinearAcceleration");
      positive(mobile.maxAngularAcceleration, "mobileBase.maxAngularAcceleration");
      if ((mobile.footprintLength == null) != (mobile.footprintWidth == null))
        diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_FOOTPRINT", "mobileBase.footprint",
          "footprint length and width must both be set or both be omitted"));
      if (mobile.footprintLength != null) positive(mobile.footprintLength, "mobileBase.footprint.length");
      if (mobile.footprintWidth != null) positive(mobile.footprintWidth, "mobileBase.footprint.width");
      switch mobile.drive {
        case Differential(leftId, rightId, radius, trackWidth):
          for (role in [{id: leftId, path: "mobileBase.drive.leftWheelJointId"},
              {id: rightId, path: "mobileBase.drive.rightWheelJointId"}]) {
            var index = resolveRole(role.id, role.path, [JointType.Revolute, JointType.Continuous]);
            if (index != null && wheelDirection(robot, index) == 0)
              diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_WHEEL_AXIS", role.path,
                'wheel joint "${robot.joints[index].name}" must turn about the base\'s lateral (Y) axis'));
          }
          positive(radius, "mobileBase.drive.wheelRadius");
          positive(trackWidth, "mobileBase.drive.trackWidth");
        case Ackermann(steeringId, driveId, wheelBase, radius, maxSteeringAngle):
          resolveRole(steeringId, "mobileBase.drive.steeringJointId", [JointType.Revolute]);
          resolveRole(driveId, "mobileBase.drive.driveWheelJointId", [JointType.Revolute, JointType.Continuous]);
          positive(wheelBase, "mobileBase.drive.wheelBase");
          positive(radius, "mobileBase.drive.wheelRadius");
          if (!Math.isFinite(maxSteeringAngle) || maxSteeringAngle <= 0.0 || maxSteeringAngle >= Math.PI * 0.5)
            diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_VALUE", "mobileBase.drive.maxSteeringAngle",
              "steering angle must be finite and between zero and pi/2"));
        case Holonomic(wheelIds, radius, baseRadius):
          if (wheelIds == null || wheelIds.length != 3)
            diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_JOINT", "mobileBase.drive.wheelJointIds",
              "holonomic drive requires exactly three wheel joint IDs"));
          else for (index in 0...wheelIds.length)
            resolveRole(wheelIds[index], 'mobileBase.drive.wheelJointIds[$index]',
              [JointType.Revolute, JointType.Continuous]);
          positive(radius, "mobileBase.drive.wheelRadius");
          positive(baseRadius, "mobileBase.drive.baseRadius");
        case null:
          diagnostics.push(new RobotCompileDiagnostic("RK_ROLE_DRIVE", "mobileBase.drive",
            "mobile base drive configuration is missing"));
      }
    }

    var forks:Null<RobotForkConfiguration> = robot.forkMechanism;
    if (forks != null) {
      var linear = [JointType.Prismatic];
      resolveRole(forks.liftJointId, "forkMechanism.liftJointId", linear);
      if (forks.tiltJointId != null)
        resolveRole(forks.tiltJointId, "forkMechanism.tiltJointId", [JointType.Revolute]);
      if (forks.spreadJointId != null)
        resolveRole(forks.spreadJointId, "forkMechanism.spreadJointId", linear);
      positive(forks.maxMassKg, "forkMechanism.maxMassKg");
      positive(forks.maxLoadMomentKgMeters, "forkMechanism.maxLoadMomentKgMeters");
      positive(forks.maxLiftHeightMeters, "forkMechanism.maxLiftHeightMeters");
    }
  }

  static function compileConfiguration(robot:RobotModel):RobotRuntimeConfiguration {
    function jointIndex(id:String):Int {
      for (index in 0...robot.joints.length)
        if (robot.joints[index].id == id) return index;
      throw 'Validated robot configuration references missing joint "$id"';
    }
    var mobileConfig:Null<RobotRuntimeMobileConfiguration> = null;
    var mobile = robot.mobileBase;
    if (mobile != null) {
      var drive:RobotRuntimeDriveConfiguration = switch mobile.drive {
        case Differential(leftId, rightId, radius, trackWidth):
          var leftIndex = jointIndex(leftId), rightIndex = jointIndex(rightId);
          RobotRuntimeDriveConfiguration.Differential(leftIndex, robot.joints[leftIndex].name,
            rightIndex, robot.joints[rightIndex].name, radius, trackWidth,
            wheelDirection(robot, leftIndex), wheelDirection(robot, rightIndex));
        case Ackermann(steeringId, driveId, wheelBase, radius, maxAngle):
          var steeringIndex = jointIndex(steeringId), driveIndex = jointIndex(driveId);
          RobotRuntimeDriveConfiguration.Ackermann(steeringIndex, robot.joints[steeringIndex].name,
            driveIndex, robot.joints[driveIndex].name, wheelBase, radius, maxAngle);
        case Holonomic(wheelIds, radius, baseRadius):
          var indices = [for (id in wheelIds) jointIndex(id)];
          var names = [for (index in indices) robot.joints[index].name];
          RobotRuntimeDriveConfiguration.Holonomic(indices, names, radius, baseRadius);
        case null: throw "Validated robot configuration has no drive";
      };
      mobileConfig = new RobotRuntimeMobileConfiguration(drive,
        mobile.maxLinearSpeed, mobile.maxAngularSpeed,
        mobile.maxLinearAcceleration, mobile.maxAngularAcceleration,
        mobile.footprintLength, mobile.footprintWidth);
    }
    var forkConfig:Null<RobotRuntimeForkConfiguration> = null;
    var forks = robot.forkMechanism;
    if (forks != null) {
      function axis(id:String):RobotRuntimeForkAxisConfiguration {
        var index = jointIndex(id), joint = robot.joints[index];
        return new RobotRuntimeForkAxisConfiguration(index, joint.name,
          joint.limits.lower, joint.limits.upper);
      }
      forkConfig = new RobotRuntimeForkConfiguration(axis(forks.liftJointId),
        forks.tiltJointId == null ? null : axis(forks.tiltJointId),
        forks.spreadJointId == null ? null : axis(forks.spreadJointId),
        forks.maxMassKg, forks.maxLoadMomentKgMeters, forks.maxLiftHeightMeters);
    }
    return new RobotRuntimeConfiguration(mobileConfig, forkConfig);
  }

  /**
   * Which way positive motion of wheel joint `index` drives the base, with every
   * joint above it at zero: +1 when the wheel spins about the root link's +Y, so
   * it rolls along +X, -1 about -Y, and 0 when its axis is not lateral.
   */
  public static function wheelDirection(robot:RobotModel, index:Int):Int {
    var joint = robot.joints[index];
    if (joint == null || joint.axis == null || joint.axis.length != 3) return 0;
    // Rotation from the joint frame to the root: child = parent · parentFrame · motion · childFrame⁻¹.
    var rotation = Quat.fromArray(joint.parentFrameRotation);
    var link = joint.parent;
    for (_ in 0...robot.joints.length) {
      var above:Null<Joint> = null;
      for (candidate in robot.joints) if (candidate != null && candidate.child == link) above = candidate;
      if (above == null) break;
      rotation = Quat.fromArray(above.parentFrameRotation)
        .multiply(Quat.fromArray(above.childFrameRotation).conjugate()).multiply(rotation);
      link = above.parent;
    }
    var axis = rotation.rotate(new Vec3(joint.axis[0], joint.axis[1], joint.axis[2]));
    var length = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (!(length > 0)) return 0;
    var lateral = axis.y / length;
    return lateral > 0.999 ? 1 : lateral < -0.999 ? -1 : 0;
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
