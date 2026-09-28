package robotkit.runtime;

import RobotKitRuntime;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventCodec;

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
  public final couplings:Array<RobotRuntimeJointCouplingBlueprint> = [];
  public final sensors:Array<RobotRuntimeSensorBlueprint> = [];
  public final channels:Array<ProcessChannelDeclaration> = [];
  public final links:Array<RobotRuntimeLinkBlueprint> = [];
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
      value.set_channels(index, nativeChannel);
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
    for (index in 0...couplings.length)
      value.set_couplings(index, couplings[index].nativeValue());
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
    for (i in 0...layout.length) value.set_sensors(i, layout[i].nativeValue());
    for (index in 0...joints.length) {
      var joint = joints[index];
      value.set_joints(index, joint.nativeValue());
      var dynamics = new rk_robot_joint_dynamics();
      dynamics.set_armature(joint.armature);
      dynamics.set_damping(joint.damping);
      dynamics.set_friction_loss(joint.frictionLoss);
      dynamics.set_limit_time_constant(joint.limitTimeConstant);
      dynamics.set_limit_damping_ratio(joint.limitDampingRatio);
      for (term in 0...5) dynamics.set_limit_impedance(term, joint.limitImpedance[term]);
      value.set_joint_dynamics(index, dynamics);
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
