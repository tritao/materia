package robotkit.model;

/** One motor's place in the drive of an axis: how the axis's motion and force reach its rotor. */
class MotorLoad {
  public final actuator:Actuator;
  /** The joint the motor's transmission drives. */
  public final joint:JointId;
  /** Motor coordinate (rotor radians for a stepper or servo) per unit of the axis, signed. */
  public final ratio:Float;
  /** Share of power the couplings between the axis and the motor pass on. */
  public final efficiency:Float;
  /** Inertia that turns with the motor alone: its rotor and the body it is on, kg m². */
  public final rotorInertia:Float;
  /** Constant torque the couplings' drag adds at the motor while it turns, N m. */
  public final drag:Float;
  /** Share of the axis's force this motor carries, from what each could deliver. */
  public var share:Float = 1.0;
  public var assumed:Array<String> = [];

  public function new(actuator:Actuator, joint:JointId, ratio:Float, efficiency:Float, rotorInertia:Float, drag:Float) {
    this.actuator = actuator;
    this.joint = joint;
    this.ratio = ratio;
    this.efficiency = efficiency;
    this.rotorInertia = rotorInertia;
    this.drag = drag;
  }
}

/**
 * What an axis (a joint no other joint leads) asks of its motors: the mass and inertia it moves,
 * the steady loads on it (gravity, friction) and how stiff and loose its drive is. Built by
 * `DriveLoads.of`; the plan check turns it into torque along a plan.
 */
class AxisLoad {
  public final axis:JointId;
  /** True for a sliding axis, whose force is in N and acceleration in m/s². */
  public final sliding:Bool;
  /**
   * What accelerating the axis moves, in kg (kg m² for a turning axis): the mass the joint carries
   * plus the turning inertia of every coupled joint that is not a motor (idlers and the like),
   * reflected through the couplings.
   */
  public final mass:Float;
  /** Force needed to hold the axis against gravity along its positive direction, N; zero when it is not sliding. */
  public final gravityForce:Float;
  /** True when a turning joint between the base and the axis makes its direction in the world unknown, so gravity is a worst case. */
  public final gravityWorstCase:Bool;
  public final motors:Array<MotorLoad> = [];
  /** Stiffness of the axis's drive against force on the axis, N/m; 0 when rigid. */
  public var stiffness:Float = 0.0;
  /** Lost motion on reversal at the axis, in its units, summed along the drive. */
  public var backlash:Float = 0.0;
  /** Assumptions along the axis's motor paths, with duplicates removed. */
  public var assumed:Array<String> = [];
  /** Running friction of the axis, in N: the joint's own dry friction, or the assumed rail drag of a sliding axis. */
  public final friction:Float;

  public function new(axis:JointId, sliding:Bool, mass:Float, gravityForce:Float, gravityWorstCase:Bool, friction:Float) {
    this.axis = axis;
    this.sliding = sliding;
    this.mass = mass;
    this.gravityForce = gravityForce;
    this.gravityWorstCase = gravityWorstCase;
    this.friction = friction;
  }

  /**
   * Force the whole drive must put on the axis, N: inertia, gravity and the resisting forces
   * `resisting` (friction, cutting), which oppose `velocity`.
   */
  public function force(velocity:Float, acceleration:Float, resisting:Float):Float {
    var held = gravityForce;
    var push = mass * acceleration + held;
    var direction = velocity > 1e-12 ? 1.0 : velocity < -1e-12 ? -1.0 : (acceleration >= 0.0 ? 1.0 : -1.0);
    // With no known direction for gravity, take it in the sense that asks most of the drive.
    if (gravityWorstCase) push = mass * acceleration + (push >= 0.0 ? Math.abs(held) : -Math.abs(held));
    return push + direction * resisting;
  }

  /**
   * Torque `motor` must give at its rotor, N m (signed like its own motion): its share of the axis
   * force through the ratio and the couplings' efficiency, its rotor's inertia times its
   * acceleration, and drag. A load that drives the motor (braking) passes through the couplings'
   * efficiency the other way.
   */
  public function motorTorque(motor:MotorLoad, velocity:Float, acceleration:Float, resisting:Float):Float {
    var speed = motor.ratio * velocity;
    var drag = speed > 1e-12 ? motor.drag : speed < -1e-12 ? -motor.drag : 0.0;
    return motorTorqueWithoutDrag(motor, velocity, acceleration, resisting) + drag;
  }

  /**
   * As `motorTorque`, less drag. A motor that serves several axes (CoreXY) takes the sum of this
   * over its axes, and one drag in the direction of its total motion.
   */
  public function motorTorqueWithoutDrag(motor:MotorLoad, velocity:Float, acceleration:Float, resisting:Float):Float {
    var load = force(velocity, acceleration, resisting);
    var perUnit = motor.share * load / motor.ratio;
    var motoring = load * velocity >= 0.0;
    var transmitted = motoring ? perUnit / motor.efficiency : perUnit * motor.efficiency;
    return motor.rotorInertia * motor.ratio * acceleration + transmitted;
  }
}

/**
 * The axes of a model that actuators drive, through couplings or directly, with what each asks of
 * them. A joint is an axis when no coupling leads to it as a follower. Joint frames are taken in
 * a world whose +Z is up; an axis whose direction depends on a turning joint is treated as worst
 * case for gravity.
 */
class DriveLoads {
  public static inline final GRAVITY = 9.80665;

  /** The load on axis `id`, or null when it is not an axis or no actuator drives it. */
  public static function forAxis(model:RobotModel, id:JointId, ?steady:SteadyLoads):Null<AxisLoad> {
    for (joint in model.joints) if (joint.id == id) {
      for (coupling in model.couplings) if (coupling.follower == id) return null;
      return axisLoad(model, joint, steady == null ? new SteadyLoads() : steady);
    }
    return null;
  }

  public static function of(model:RobotModel, ?steady:SteadyLoads):Array<AxisLoad> {
    var loads = steady == null ? new SteadyLoads() : steady;
    var followers = new Map<String, Bool>();
    for (coupling in model.couplings) followers.set(coupling.follower, true);
    var result:Array<AxisLoad> = [];
    for (joint in model.joints) {
      if (followers.exists(joint.id)) continue;
      var load = axisLoad(model, joint, loads);
      if (load != null) result.push(load);
    }
    return result;
  }

  static function axisLoad(model:RobotModel, axis:Joint, steady:SteadyLoads):Null<AxisLoad> {
    var reached:Array<Joint> = [axis];
    var ratios:Array<Float> = [1.0];
    var efficiencies:Array<Float> = [1.0];
    // Compliance at the axis, axis units per N, and backlash in axis units, accumulated along the path.
    var compliance:Array<Float> = [0.0];
    var backlash:Array<Float> = [0.0];
    var drag:Array<Float> = [0.0];
    var rigid:Array<Bool> = [true];
    var combined:Array<Bool> = [false];
    var assumed:Array<Array<String>> = [[]];
    var next = 0;
    while (next < reached.length) {
      var leader = reached[next], leaderRatio = ratios[next];
      var at = next++;
      for (coupling in model.couplings) if (coupling.leader == leader.id) {
        var follower:Null<Joint> = null;
        for (candidate in model.joints) if (candidate.id == coupling.follower) follower = candidate;
        if (follower == null || reached.indexOf(follower) >= 0) continue;
        var ratio = coupling.ratio * leaderRatio;
        reached.push(follower);
        var labels = assumed[at].copy();
        for (label in coupling.assumed) if (labels.indexOf(label) < 0) labels.push(label);
        assumed.push(labels);
        ratios.push(ratio);
        efficiencies.push(efficiencies[at] * coupling.efficiency);
        // Stiffness is at the coupling's leader, which moves |leaderRatio| per unit of the axis.
        var leaderScale = Math.abs(leaderRatio);
        var soft = coupling.stiffness > 0.0;
        compliance.push(compliance[at] + (soft ? 1.0 / (coupling.stiffness * leaderScale * leaderScale) : 0.0));
        rigid.push(rigid[at] && !soft);
        var terms = 0;
        for (term in model.couplings) if (term.follower == coupling.follower) terms++;
        combined.push(combined[at] || terms > 1);
        backlash.push(backlash[at] + coupling.backlash / leaderScale);
        // Drag is in the follower's units; further down the chain it reaches the motor through the ratios.
        drag.push(drag[at] * Math.abs(coupling.ratio) + coupling.drag);
      }
    }
    var motors:Array<MotorLoad> = [];
    var motorJoints:Array<Joint> = [];
    var total = 0.0;
    for (actuator in model.actuators) switch actuator.transmission {
      case SimpleTransmission(target, transmissionRatio, _):
        var index = -1;
        for (candidate in 0...reached.length) if (reached[candidate].id == target) index = candidate;
        if (index < 0) continue;
        var motorRatio = transmissionRatio * ratios[index];
        if (motorRatio == 0.0) continue;
        var motor = new MotorLoad(actuator, target, motorRatio, efficiencies[index] * actuator.efficiency,
          model.turningInertia(reached[index]), drag[index] * Math.abs(transmissionRatio));
        motor.assumed = assumed[index].copy();
        for (label in actuator.assumed) if (motor.assumed.indexOf(label) < 0) motor.assumed.push(label);
        motors.push(motor);
        motorJoints.push(reached[index]);
        total += efficiencies[index] * actuator.efficiency * actuator.planningEffort() * Math.abs(motorRatio);
    }
    if (motors.length == 0) return null;
    for (motor in motors)
      motor.share = total > 0.0 ? motor.efficiency * motor.actuator.planningEffort() * Math.abs(motor.ratio) / total : 1.0 / motors.length;
    var sliding = axis.type == JointType.Prismatic;
    var mass = sliding ? model.carriedMass(axis) + axis.armature : model.turningInertia(axis);
    // Coupled joints that no motor turns are inertia on the axis, through their own couplings.
    for (index in 1...reached.length) if (motorJoints.indexOf(reached[index]) < 0)
      mass += efficiencies[index] * model.turningInertia(reached[index]) * ratios[index] * ratios[index];
    var gravityForce = 0.0, worst = false;
    if (sliding) {
      var along = gravityAlong(model, axis);
      var carried = model.carriedMass(axis);
      if (along == null) {
        worst = true;
        gravityForce = carried * steady.gravity;
      } else gravityForce = carried * steady.gravity * along;
    }
    var load = new AxisLoad(axis.id, sliding, mass, gravityForce, worst, steady.friction(axis));
    for (motor in motors) {
      load.motors.push(motor);
      for (label in motor.assumed) if (load.assumed.indexOf(label) < 0) load.assumed.push(label);
    }
    load.assumed.sort(Reflect.compare);
    // Motors in parallel share the deflection, so their stiffnesses add; one rigid drive makes the
    // axis rigid, whatever softer ones do. Backlash takes the loosest drive.
    var stiffness = 0.0, loose = 0.0, anyRigid = false;
    var seriesCompliance = 0.0, sharedCoordinates = false;
    for (motor in motors) {
      var index = -1;
      for (candidate in 0...reached.length) if (reached[candidate].id == motor.joint) index = candidate;
      sharedCoordinates = sharedCoordinates || combined[index];
      seriesCompliance += motor.share * motor.share * compliance[index];
      if (rigid[index]) anyRigid = true;
      else stiffness += 1.0 / compliance[index];
      loose = Math.max(loose, backlash[index]);
    }
    // A summed motor coordinate (CoreXY) depends on every motor path. Reflect each
    // path's compliance by the square of its force share (strain energy). With equal
    // shares, K_axis = 4 / (1/K_A + 1/K_B); a rigid belt cannot mask the other belt.
    load.stiffness = sharedCoordinates ? (seriesCompliance > 0.0 ? 1.0 / seriesCompliance : 0.0)
      : (anyRigid ? 0.0 : stiffness);
    load.backlash = loose;
    return load;
  }

  /**
   * The axis's direction along world +Z (the cosine), when every joint above it keeps its
   * orientation (fixed or sliding); null when a turning joint makes it unknown.
   */
  static function gravityAlong(model:RobotModel, axis:Joint):Null<Float> {
    var path:Array<Joint> = [];
    var link = axis.parent;
    var guard = 0;
    while (guard++ < 4096) {
      var above:Null<Joint> = null;
      for (candidate in model.joints) if (candidate.child == link) above = candidate;
      if (above == null) break;
      path.push(above);
      link = above.parent;
    }
    // Orientation of the parent link in the world, from the root: each joint turns the frame by the
    // parent frame's rotation then the inverse of the child frame's rotation.
    var rotation = [0.0, 0.0, 0.0, 1.0];
    var index = path.length - 1;
    while (index >= 0) {
      var joint = path[index--];
      if (joint.type != JointType.Prismatic && joint.type != JointType.Fixed) return null;
      rotation = multiply(multiply(rotation, joint.parentFrameRotation), conjugate(joint.childFrameRotation));
    }
    var direction = rotate(multiply(rotation, axis.parentFrameRotation), axis.axis);
    var length = Math.sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    return length > 0.0 ? direction[2] / length : null;
  }

  static function multiply(a:Array<Float>, b:Array<Float>):Array<Float>
    return [a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
      a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
      a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
      a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];

  static function conjugate(q:Array<Float>):Array<Float> return [-q[0], -q[1], -q[2], q[3]];

  static function rotate(q:Array<Float>, v:Array<Float>):Array<Float> {
    var x = q[0], y = q[1], z = q[2], w = q[3];
    var tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
  }
}
