package robotkit.runtime;

import RobotKitRuntime;
import robotkit.tool.ToolChannels;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventCodec;
import robotkit.execution.ProcessEventValue;

/**
 * Immutable-at-execution compiled robot description consumed by RobotRuntime.
 *
 * Keeping this boundary separate from the editable `robotkit.model.RobotModel`
 * model makes validation and native handle creation deterministic.
 */
class RobotRuntimeBlueprint {
  /** Semantic mappings are absent only for manually constructed native blueprints. */
  public final identity:Null<RobotRuntimeIdentity>;
  public final revision:Int;
  public final calibrationRevision:Int;
  public final jointCount:Int;
  public final linkCount:Int;
  public final frameCount:Int;
  public final joints:Array<RobotRuntimeJointBlueprint> = [];
  /** Fastest specified drive position loop, Hz. */
  public function fastestPositionLoopRate():Float {
    var rate = 0.0;
    for (joint in joints) rate = Math.max(rate, joint.positionLoopRate);
    return rate;
  }
  /** Conservative integration interval from servo gain and reflected inertia, seconds. */
  public function servoStabilityInterval():Float {
    var interval = Math.POSITIVE_INFINITY;
    for (joint in joints) if (joint.servoStiffness > 0 && joint.reflectedInertia > 0)
      interval = Math.min(interval, 0.5 * Math.sqrt(joint.reflectedInertia / joint.servoStiffness));
    return interval;
  }
  public final couplings:Array<RobotRuntimeJointCouplingBlueprint> = [];
  public final sensors:Array<RobotRuntimeSensorBlueprint> = [];
  public final channels:Array<ProcessChannelDeclaration> = [];
  /** Pneumatic process drives are bound to channels, never to trajectory axes. */
  public final pneumaticDrives:Array<RobotRuntimePneumaticDriveBlueprint> = [];
  /** Velocity process drives are bound to analog channels, never to trajectory axes. */
  public final velocityDrives:Array<RobotRuntimeVelocityDriveBlueprint> = [];
  public final links:Array<RobotRuntimeLinkBlueprint> = [];
  /**
   * Sets a tool's channels up with the safe values and stop policy the tool owns (`ToolChannels`). A channel the robot's
   * setup declared already must say the same, or the tool is refused: a torch's arc cannot be left to keep its output
   * through a stop because the setup declared it so.
   */
  public function addTool(tool:ToolChannels):Void {
    if (tool == null) throw "A tool is required";
    for (declaration in tool.declarations()) addChannel(declaration);
  }

  /** Adds a declaration once, requiring every owner to agree on its typed safe value and stop policy. */
  public function addChannel(declaration:ProcessChannelDeclaration):Void {
    if (declaration == null) throw "A process channel declaration is required";
    for (channel in channels) if (channel.id == declaration.id) {
      if (channel.keepOnStop != declaration.keepOnStop ||
          !RobotRuntimeBlueprint.sameValue(channel.safeValue, declaration.safeValue))
        throw 'Channel "${declaration.id}" has conflicting safe values or stop policies';
      return;
    }
    channels.push(declaration);
  }

  static function sameValue(a:ProcessEventValue, b:ProcessEventValue):Bool
    return switch [a, b] {
      case [Digital(x), Digital(y)]: x == y;
      case [Analog(x), Analog(y)]: x == y;
      case [Process(c, x), Process(d, y)]: c == d && x == y;
      case _: false;
    };

  public var collisionApproximation:Int = RobotKitRuntimeConstants.RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
  /** MuJoCo self-collision is enabled unless this opt-out is set false. */
  public var selfCollision:Bool = true;
  /** Primitive link collision shapes, in link order; Simulation collides through them. */
  public final linkCollisionShapes:Array<RobotRuntimeLinkShape> = [];
  /** Explicit contacts between linkCollisionShapes entries, by index. */
  public final contactPairs:Array<RobotRuntimeContactPair> = [];
  /**
   * How far an observed position may pass a joint limit before the runtime
   * faults; zero keeps limits exact. Legged robots resting on compliant stops
   * need a small tolerance.
   */
  public var observedLimitTolerance:Float = 0.0;
  /** True makes the root link a free six-DOF body in Simulation. */
  public var floatingBase:Bool = false;
  /** Zero uses two owner periods. */
  public var commitLeadNs:haxe.Int64 = haxe.Int64.ofInt(0);
  /** Zero uses the runtime's 10 ms owner period. */
  public var ownerPeriodNs:haxe.Int64 = haxe.Int64.ofInt(0);
  /** Zero uses the deployed 2 ms device per-frame processing allowance. */
  public var serialProcessingAllowanceNs:haxe.Int64 = haxe.Int64.ofInt(0);
  /** Per-joint SI-unit following-error bounds; zero leaves the check disabled. */
  public final followingErrorBounds:Array<Float>;
  /** Compiled user-layer roles; null for manually assembled native blueprints. */
  public final configuration:Null<RobotRuntimeConfiguration>;

  public function sensorLayout():Array<RobotRuntimeSensorBlueprint> {
    if (sensors.length > 0) return sensors.copy();
    var children = [for (joint in joints) joint.childLink];
    var root = 0;
    for (i in 0...linkCount) if (children.indexOf(i) < 0) { root = i; break; }
    return RobotRuntimeSensorBlueprint.defaults(root, "base_link");
  }

  /** Sensors the native runtime samples; a robot with only external sensors keeps the defaults. */
  public function nativeSensorLayout():Array<RobotRuntimeSensorBlueprint> {
    var native = [for (sensor in sensorLayout()) if (!sensor.external) sensor];
    if (native.length > 0) return native;
    var children = [for (joint in joints) joint.childLink];
    var root = 0;
    for (i in 0...linkCount) if (children.indexOf(i) < 0) { root = i; break; }
    return RobotRuntimeSensorBlueprint.defaults(root, "base_link");
  }

  /** Authored sensors whose observations are published from outside the native runtime. */
  public function externalSensorLayout():Array<RobotRuntimeSensorBlueprint>
    return [for (sensor in sensors) if (sensor.external) sensor];

  public function new(revision:Int, jointCount:Int, linkCount:Int,
      ?frameCount:Int = 0, ?identity:RobotRuntimeIdentity,
      ?configuration:RobotRuntimeConfiguration, ?calibrationRevision:Int = 0) {
    if (revision < 0 || jointCount < 0 || jointCount > RobotKitRuntimeConstants.RK_MAX_JOINTS ||
        linkCount < 1 || linkCount > RobotKitRuntimeConstants.RK_MAX_LINKS || frameCount < 0 ||
        calibrationRevision < 0)
      throw "Invalid RobotKit runtime blueprint";
    this.identity = identity;
    this.revision = revision;
    this.calibrationRevision = calibrationRevision;
    this.jointCount = jointCount;
    followingErrorBounds = [for (_ in 0...jointCount) 0.0];
    this.linkCount = linkCount;
    this.frameCount = frameCount;
    for (_ in 0...linkCount)
      links.push(new RobotRuntimeLinkBlueprint(1.0, [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]));
    this.configuration = configuration;
  }

  public function addJoint(value:RobotRuntimeJointBlueprint):Void {
    if (joints.length >= jointCount)
      throw "RobotKit runtime blueprint has too many joints";
    joints.push(value);
  }

  /** Finds a compiled sensor by its stable model ID, independent of array order. */
  public function sensorById(id:String):Null<RobotRuntimeSensorBlueprint> {
    if (id == null || id.length == 0) return null;
    if (identity != null) {
      var index = identity.sensorIndex(id);
      if (index >= 0 && index < sensors.length && sensors[index].id == id)
        return sensors[index];
    }
    for (sensor in sensors) if (sensor.id == id) return sensor;
    for (sensor in sensorLayout()) if (sensor.id == id) return sensor;
    return null;
  }

  @:allow(RobotRuntime, Simulation)
  function nativeValue():rk_robot_runtime_blueprint {
    if (joints.length != jointCount)
      throw "RobotKit runtime blueprint is missing joints";
    var value = new rk_robot_runtime_blueprint();
    value.set_struct_size(rk_robot_runtime_blueprint.size());
    value.set_revision(haxe.Int64.ofInt(revision));
    value.set_calibration_revision(haxe.Int64.ofInt(calibrationRevision));
    value.set_commit_lead_ns(commitLeadNs);
    if (haxe.Int64.compare(ownerPeriodNs, haxe.Int64.ofInt(0)) < 0)
      throw "Invalid runtime owner period";
    value.set_owner_period_ns(ownerPeriodNs);
    if (haxe.Int64.compare(serialProcessingAllowanceNs, haxe.Int64.ofInt(0)) < 0)
      throw "Invalid serial processing allowance";
    value.set_serial_processing_allowance_ns(serialProcessingAllowanceNs);
    if (channels.length > RobotKitRuntimeConstants.RK_MAX_PROCESS_CHANNELS)
      throw "Too many runtime process channels";
    value.set_channel_count(channels.length);
    for (index in 0...channels.length) {
      var channel = channels[index];
      for (earlier in 0...index)
        if (channels[earlier].id == channel.id) throw "Duplicate runtime process channel";
      var nativeChannel = new rk_channel_declaration();
      for (i in 0...channel.id.length)
        nativeChannel.set_id(i, channel.id.charCodeAt(i));
      var safe = ProcessEventCodec.encode(channel.safeValue);
      nativeChannel.set_kind(safe.get_kind());
      nativeChannel.set_safe_value(safe);
      nativeChannel.set_stop_policy(channel.keepOnStop ? RobotKitRuntimeConstants.RK_CHANNEL_KEEP_ON_STOP
        : RobotKitRuntimeConstants.RK_CHANNEL_SAFE_ON_STOP);
      value.set_channels(index, nativeChannel);
    }
    if (pneumaticDrives.length > RobotKitRuntimeConstants.RK_MAX_JOINTS)
      throw "Too many process-driven joints";
    var processJointIds = new Map<Int, Bool>();
    for (drive in pneumaticDrives) {
      if (drive.joint < 0 || drive.joint >= jointCount)
        throw 'Pneumatic process drive references unknown joint ${drive.joint}';
      if (processJointIds.exists(drive.joint))
        throw 'Multiple process drives reference joint ${drive.joint}';
      processJointIds.set(drive.joint, true);
      var channelA = false, channelB = drive.channelB == null;
      for (channel in channels) {
        if (channel.id == drive.channelA && channel.safeValue == Digital(false)) channelA = true;
        if (drive.channelB != null && channel.id == drive.channelB && channel.safeValue == Digital(false)) channelB = true;
      }
      if (!channelA || !channelB)
        throw 'Pneumatic drive on joint ${drive.joint} needs declared digital coil channels';
      value.set_process_joint(drive.joint, 1);
    }
    if (velocityDrives.length > RobotKitRuntimeConstants.RK_MAX_JOINTS)
      throw "Too many process-driven joints";
    for (drive in velocityDrives) {
      if (drive.joint < 0 || drive.joint >= jointCount)
        throw 'Velocity process drive references unknown joint ${drive.joint}';
      if (processJointIds.exists(drive.joint))
        throw 'Multiple process drives reference joint ${drive.joint}';
      processJointIds.set(drive.joint, true);
      if (!(drive.radiansPerSpeedUnit > 0.0) || !Math.isFinite(drive.radiansPerSpeedUnit) ||
          !(drive.maxRate > 0.0) || !Math.isFinite(drive.maxRate) ||
          !Math.isFinite(drive.maxEffort) || drive.maxEffort < 0.0)
        throw 'Velocity process drive on joint ${drive.joint} has invalid ratings';
      var speedChannel = false, directionChannel = false;
      for (channel in channels) {
        if (channel.id == drive.speedChannel && channel.safeValue == Analog(0.0)) speedChannel = true;
        if (channel.id == drive.directionChannel && channel.safeValue == Analog(0.0)) directionChannel = true;
      }
      if (!speedChannel || !directionChannel)
        throw 'Velocity drive on joint ${drive.joint} needs declared analog speed and direction channels';
      value.set_process_joint(drive.joint, 1);
    }
    for (joint in 0...jointCount) {
      var bound = followingErrorBounds[joint];
      if (!Math.isFinite(bound) || bound < 0.0)
        throw "Invalid runtime following-error bound";
      if (joint < RobotKitRuntimeConstants.RK_MAX_TRAJECTORY_JOINTS)
        value.set_following_error_bound(joint, bound);
    }
    value.set_joint_count(jointCount);
    if (couplings.length > RobotKitRuntimeConstants.RK_MAX_JOINT_COUPLINGS)
      throw "Too many runtime joint couplings";
    value.set_coupling_count(couplings.length);
    for (index in 0...couplings.length) {
      value.set_couplings(index, couplings[index].nativeValue());
      value.set_coupling_stiffness(index, couplings[index].stiffness);
    }
    value.set_link_count(linkCount);
    value.set_frame_count(frameCount);
    value.set_collision_approximation(collisionApproximation);
    value.set_self_collision(selfCollision ? 1 : 2);
    value.set_floating_base(floatingBase ? 1 : 0);
    if (!Math.isFinite(observedLimitTolerance) || observedLimitTolerance < 0.0)
      throw "Observed limit tolerance must be finite and non-negative";
    value.set_observed_limit_tolerance(observedLimitTolerance);
    var layout = nativeSensorLayout();
    if (layout.length > RobotKitRuntimeConstants.RK_MAX_SENSORS) throw "Too many sensors";
    value.set_sensor_count(layout.length);
    for (i in 0...RobotKitRuntimeConstants.RK_MAX_SENSORS) {
      value.set_sensor_joint(i, -1);
      value.set_sensor_window_lower(i, 0.0);
      value.set_sensor_window_upper(i, 0.0);
      value.set_sensor_hysteresis(i, 0.0);
    }
    for (i in 0...layout.length) {
      var sensor = layout[i];
      value.set_sensors(i, sensor.nativeValue());
      value.set_sensor_joint(i, sensor.joint);
      value.set_sensor_window_lower(i, sensor.windowLower);
      value.set_sensor_window_upper(i, sensor.windowUpper);
      value.set_sensor_hysteresis(i, sensor.hysteresis);
    }
    for (index in 0...joints.length) {
      var joint = joints[index];
      value.set_joints(index, joint.nativeValue());
      if (!Math.isFinite(joint.overtravel) || joint.overtravel < 0.0)
        throw "Joint overtravel must be finite and non-negative";
      value.set_joint_overtravel(index, joint.overtravel);
      var dynamics = new rk_robot_joint_dynamics();
      dynamics.set_armature(joint.armature);
      dynamics.set_damping(joint.damping);
      dynamics.set_friction_loss(joint.frictionLoss);
      dynamics.set_limit_time_constant(joint.limitTimeConstant);
      dynamics.set_limit_damping_ratio(joint.limitDampingRatio);
      for (term in 0...5) dynamics.set_limit_impedance(term, joint.limitImpedance[term]);
      value.set_joint_dynamics(index, dynamics);
      if (!Math.isFinite(joint.servoStiffness) || joint.servoStiffness < 0.0 ||
          !Math.isFinite(joint.servoDamping) || joint.servoDamping < 0.0)
        throw "Joint servo gains must be finite and non-negative";
      var servo = new rk_robot_joint_servo();
      servo.set_stiffness(joint.servoStiffness);
      servo.set_damping(joint.servoDamping);
      value.set_joint_servo(index, servo);
      value.set_servo_reflected_inertia(index, joint.reflectedInertia);
    }
    if (links.length != linkCount) throw "RobotKit runtime blueprint is missing link physical properties";
    for (index in 0...links.length) value.set_links(index, links[index].nativeValue());
    return value;
  }

}

/** One primitive collision shape on the link at a runtime link index. */
class RobotRuntimeLinkShape {
  public final link:Int;
  public final shape:robotkit.model.CollisionShape;

  public function new(link:Int, shape:robotkit.model.CollisionShape) {
    this.link = link;
    this.shape = shape;
  }
}

/** An explicit contact between two entries of linkCollisionShapes. */
class RobotRuntimeContactPair {
  public final shapeA:Int;
  public final shapeB:Int;
  public final surface:robotkit.model.CollisionShape.ContactSurface;

  public function new(shapeA:Int, shapeB:Int, surface:robotkit.model.CollisionShape.ContactSurface) {
    this.shapeA = shapeA;
    this.shapeB = shapeB;
    this.surface = surface;
  }
}
