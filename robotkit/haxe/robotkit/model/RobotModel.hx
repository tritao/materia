package robotkit.model;

import robotkit.model.Transmission;

/** Editable static definition of a robot's links, joints, and sensors. */
class RobotModel {
  public static inline var CURRENT_VERSION:Int = 6;
  public final schemaVersion:Int = CURRENT_VERSION;
  public final name:String;
  public final links:Array<Link> = [];
  public final joints:Array<Joint> = [];
  /** Multiple actuators may address one joint through independent transmissions. */
  public final actuators:Array<Actuator> = [];
  /** Mechanical joint-to-joint relations, independent of actuator transmissions. */
  public final couplings:Array<JointCoupling> = [];
  public final sensors:Array<Sensor> = [];
  /** Encoders on joints; they are read from joint positions rather than compiled into the runtime. */
  public final encoders:Array<Encoder> = [];
  /** Explicit contacts between link collision shapes. */
  public final contactPairs:Array<ContactPair> = [];
  public final frames:Array<Frame> = [];
  public var collisionApproximation:CollisionApproximation = CollisionApproximation.BoundsBox;
  /**
   * True when the root link is a free six-DOF body, as for a legged or
   * humanoid robot, rather than a base fixed to the world. This replaces a
   * floating joint: runtime joints stay one-DOF.
   */
  public var floatingBase:Bool = false;
  /** Semantic mobile roles authored against stable joint IDs. */
  public var mobileBase:Null<RobotMobileConfiguration> = null;
  /** Semantic fork roles authored against stable joint IDs. */
  public var forkMechanism:Null<RobotForkConfiguration> = null;

  public function addFrame(frame:Frame):Frame {
    frames.push(frame);
    return frame;
  }

  public function new(name:String) {
    this.name = name;
  }

  public function addLink(link:Link):Link {
    links.push(link);
    return link;
  }

  public function addJoint(joint:Joint):Joint {
    joints.push(joint);
    return joint;
  }

  public function addActuator(actuator:Actuator):Actuator {
    actuators.push(actuator);
    return actuator;
  }

  public function addEncoder(encoder:Encoder):Encoder {
    encoders.push(encoder);
    return encoder;
  }

  /**
   * The encoder that reads `actuator`: the one it names, or, for a servo saved before encoders were
   * sensors, one made from the servo drive's own count (incremental, on the actuator's joint).
   */
  public function encoderFor(actuator:Actuator):Null<Encoder> {
    if (actuator.encoder != "") {
      for (encoder in encoders) if (encoder.id == actuator.encoder) return encoder;
      return null;
    }
    var drive = actuator.drive;
    if (drive == null || !Std.isOfType(drive, ActuatorDrive.ServoDrive)) return null;
    var counts = cast(drive, ActuatorDrive.ServoDrive).encoderCounts;
    if (!(counts > 0.0)) return null;
    return switch actuator.transmission {
      case SimpleTransmission(jointId, _, _): Encoder.perRevolution(actuator.id + ".encoder", jointId, EncoderKind.Incremental, counts);
    };
  }

  public function addCoupling(coupling:JointCoupling):JointCoupling {
    couplings.push(coupling);
    return coupling;
  }

  /**
   * Joint `id`'s limits, tightened by every joint that moves with it through couplings and by
   * the actuators that drive them, in `id`'s units. A lead screw's speed limit caps the axis it
   * turns with; a motor caps it at its rate through the ratios. For a sliding joint driven by
   * actuators, acceleration is also capped at their summed force, through each coupling's
   * efficiency, over the mass the joint carries plus every coupled joint's turning inertia (rotor
   * armature included) seen through the ratio. Gravity and friction are left out. Zero still
   * means unlimited. With `steady` loads, the force left for acceleration is what the motors give
   * less their drag, the axis's rail friction and its weight (see `SteadyLoads`).
   */
  public function coupledLimits(id:JointId, ?steady:SteadyLoads):JointLimits {
    var joint = [for (candidate in joints) if (candidate.id == id) candidate];
    if (joint.length != 1) throw 'Robot model has no joint "$id"';
    var own = joint[0].limits;
    var limits = new JointLimits(own.lower, own.upper, own.velocity, own.effort, own.maxAcceleration);
    limits.overtravel = own.overtravel;
    function tighten(current:Float, bound:Float):Float
      return bound <= 0.0 ? current : current <= 0.0 ? bound : Math.min(current, bound);
    // Each joint reached so far, how far it moves per unit of joint `id`, and the efficiency of
    // the couplings between them.
    var reached:Array<String> = [id];
    var scales:Array<Float> = [1.0];
    var efficiencies:Array<Float> = [1.0];
    var next = 0;
    while (next < reached.length) {
      var leader = reached[next], leaderScale = scales[next], leaderEfficiency = efficiencies[next];
      next++;
      for (coupling in couplings) if (coupling.leader == leader && reached.indexOf(coupling.follower) < 0) {
        var scale = Math.abs(coupling.ratio) * leaderScale;
        reached.push(coupling.follower);
        scales.push(scale);
        efficiencies.push(leaderEfficiency * coupling.efficiency);
        for (follower in joints) if (follower.id == coupling.follower) {
          limits.velocity = tighten(limits.velocity, follower.limits.velocity / scale);
          limits.maxAcceleration = tighten(limits.maxAcceleration, follower.limits.maxAcceleration / scale);
        }
      }
    }
    var force = 0.0, driven = false;
    for (actuator in actuators) switch actuator.transmission {
      case SimpleTransmission(target, ratio, _):
        var index = reached.indexOf(target);
        if (index < 0) continue;
        // An actuator coordinate moves |ratio| per unit of its joint, so |ratio| * scale per unit of `id`.
        var gearing = Math.abs(ratio) * scales[index];
        limits.velocity = tighten(limits.velocity, actuator.planningRate() / gearing);
        force += efficiencies[index] * actuator.planningEffort() * gearing;
        driven = true;
    }
    if (driven && joint[0].type == JointType.Prismatic && force > 0) {
      // A drive that cannot carry the steady loads has nothing left to accelerate with.
      if (steady != null) force = Math.max(steadyForce(joint[0], force, steady), 1e-9);
      var inertia = carriedMass(joint[0]) + joint[0].armature;
      for (index in 1...reached.length) for (follower in joints) if (follower.id == reached[index])
        inertia += efficiencies[index] * turningInertia(follower) * scales[index] * scales[index];
      limits.maxAcceleration = tighten(limits.maxAcceleration, force / inertia);
    }
    return limits;
  }

  /**
   * The force a sliding axis's drive has left for accelerating, N: `force` less each motor's drag
   * through its ratio, the axis's rail friction and its weight along its direction (the worst
   * way, as it has to accelerate both up and down).
   */
  function steadyForce(joint:Joint, force:Float, steady:SteadyLoads):Float {
    var load = DriveLoads.forAxis(this, joint.id, steady);
    var left = force;
    if (load != null) {
      for (motor in load.motors)
        left -= motor.efficiency * motor.drag * Math.abs(motor.ratio);
      left -= Math.abs(load.gravityForce) + load.friction;
    }
    return left;
  }

  /** Mass of the link a joint moves and of everything mounted on it, in kg. */
  @:allow(robotkit.model.DriveLoads)
  function carriedMass(joint:Joint):Float {
    var mass = 0.0, frontier = [joint.child], seen:Array<Link> = [];
    while (frontier.length > 0) {
      var link = frontier.pop();
      if (link == null || seen.indexOf(link) >= 0) continue;
      seen.push(link);
      mass += link.mass;
      for (candidate in joints) if (candidate.parent == link) frontier.push(candidate.child);
    }
    return mass;
  }

  /**
   * What a joint moves per unit of its own acceleration: a revolute joint's child link inertia
   * about the joint axis, plus its armature; a sliding joint's carried mass.
   */
  @:allow(robotkit.model.DriveLoads)
  function turningInertia(joint:Joint):Float {
    if (joint.type == JointType.Prismatic) return carriedMass(joint) + joint.armature;
    var link = joint.child, q = joint.childFrameRotation;
    // The joint axis in the child link's frame: rotate by the joint frame's quaternion.
    var x = q[0], y = q[1], z = q[2], w = q[3], v = joint.axis;
    var tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    var a = [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
    var length = Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (!(length > 0)) return joint.armature;
    for (i in 0...3) a[i] /= length;
    var t = link.inertiaTensor, about = 0.0;
    for (i in 0...3) for (j in 0...3) about += a[i] * t[i * 3 + j] * a[j];
    // Parallel axis: the centre of mass's distance from the axis line through the joint.
    var d = [for (i in 0...3) link.centerOfMass[i] - joint.childFramePosition[i]];
    var along = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
    var squared = d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - along * along;
    return about + link.mass * Math.max(0.0, squared) + joint.armature;
  }

  public function addSensor(sensor:Sensor):Sensor {
    sensors.push(sensor);
    return sensor;
  }

  /**
   * Performs the lightweight model checks used by editors.
   *
   * Runtime creation must use `RobotRuntimeCompiler.validate()` or `compile()`;
   * that pass also checks backend support and graph topology and returns
   * structured diagnostics.
   */
  public function validate():Array<String> {
    var errors:Array<String> = [];
    if (name.length == 0) errors.push("robot name is empty");
    if (links.length == 0) errors.push("robot has no links");
    if (links.length > 1024) errors.push("robot exceeds the native link limit");
    if (joints.length > 512) errors.push("robot exceeds the native joint limit");
    for (joint in joints) {
      var limitError = joint.limits.validate();
      if (limitError != null) errors.push('joint ${joint.name}: $limitError');
    }
    var ids = new Map<String, Bool>();
    for (actuator in actuators) {
      if (actuator == null) {
        errors.push("robot has a null actuator");
        continue;
      }
      if (ids.exists(actuator.id)) errors.push('duplicate actuator ID ${actuator.id}');
      ids.set(actuator.id, true);
      if (!Math.isFinite(actuator.maxEffort) || actuator.maxEffort < 0.0 ||
          !Math.isFinite(actuator.maxRate) || actuator.maxRate < 0.0)
        errors.push('actuator ${actuator.id} has invalid limits');
      if (actuator.transmission == null) {
        errors.push('actuator ${actuator.id} has no transmission');
        continue;
      }
      switch actuator.transmission {
        case SimpleTransmission(jointId, ratio, offset):
          var found = false;
          for (joint in joints) if (joint.id == jointId) found = true;
          if (!found) errors.push('actuator ${actuator.id} references unknown joint $jointId');
          if (!Math.isFinite(ratio) || ratio == 0.0 || !Math.isFinite(offset))
            errors.push('actuator ${actuator.id} has an invalid transmission');
      }
    }
    var encoderIds = new Map<String, Bool>();
    for (encoder in encoders) {
      if (encoder == null) { errors.push("robot has a null encoder"); continue; }
      if (encoderIds.exists(encoder.id)) errors.push('duplicate encoder ID ${encoder.id}');
      encoderIds.set(encoder.id, true);
      var onJoint = false;
      for (joint in joints) if (joint.id == encoder.joint) onJoint = true;
      if (!onJoint) errors.push('encoder ${encoder.id} references unknown joint ${encoder.joint}');
    }
    for (actuator in actuators)
      if (actuator != null && actuator.encoder != "" && !encoderIds.exists(actuator.encoder))
        errors.push('actuator ${actuator.id} references unknown encoder ${actuator.encoder}');
    var couplingIds = new Map<String, Bool>();
    var followers = new Map<String, Bool>();
    for (coupling in couplings) {
      if (coupling == null) { errors.push("robot has a null joint coupling"); continue; }
      if (couplingIds.exists(coupling.id)) errors.push('duplicate joint coupling ID ${coupling.id}');
      couplingIds.set(coupling.id, true);
      if (followers.exists(coupling.follower))
        errors.push('joint ${coupling.follower} has multiple coupling leaders');
      followers.set(coupling.follower, true);
      var hasLeader = false, hasFollower = false;
      for (joint in joints) {
        if (joint.id == coupling.leader) hasLeader = true;
        if (joint.id == coupling.follower) hasFollower = true;
      }
      if (!hasLeader || !hasFollower)
        errors.push('joint coupling ${coupling.id} references an unknown joint');
    }
    return errors;
  }
}
