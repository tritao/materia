package robotkit.world;

/** Immutable target for one joint inside an atomic RobotCommand batch. */
class JointTarget {
  /** Maximum targets accepted by the fixed-size RobotRuntime ABI. */
  public static inline final MAX_BATCH_SIZE:Int = 64;

  public final joint:Int;
  public final mode:JointTargetMode;
  public final target:Float;
  /** Servo terms, used only in Servo mode, in the joint's SI units. */
  public var servoVelocity(default, null):Float = 0.0;
  public var stiffness(default, null):Float = 0.0;
  public var damping(default, null):Float = 0.0;
  public var feedforward(default, null):Float = 0.0;

  public function new(joint:Int, mode:JointTargetMode, target:Float) {
    if (joint < 0 || joint >= MAX_BATCH_SIZE)
      throw 'Invalid joint target index $joint';
    if (mode == null)
      throw "Joint target mode is required";
    if (!Math.isFinite(target))
      throw "Joint target must be finite";
    this.joint = joint;
    this.mode = mode;
    this.target = target;
  }

  public static function position(joint:Int, target:Float):JointTarget
    return new JointTarget(joint, JointTargetMode.Position, target);

  public static function velocity(joint:Int, target:Float):JointTarget
    return new JointTarget(joint, JointTargetMode.Velocity, target);

  public static function effort(joint:Int, target:Float):JointTarget
    return new JointTarget(joint, JointTargetMode.Effort, target);

  /**
   * A joint servo: effort = stiffness * (position - q) + damping *
   * (velocity - qdot) + feedforward, applied every physics step and clamped to
   * the joint's effort limit.
   */
  public static function servo(joint:Int, position:Float, velocity:Float, stiffness:Float,
      damping:Float, feedforward:Float):JointTarget {
    if (!Math.isFinite(velocity) || !Math.isFinite(stiffness) || !Math.isFinite(damping) ||
        !Math.isFinite(feedforward) || stiffness < 0.0 || damping < 0.0)
      throw "Servo terms must be finite, with non-negative stiffness and damping";
    var result = new JointTarget(joint, JointTargetMode.Servo, position);
    result.servoVelocity = velocity;
    result.stiffness = stiffness;
    result.damping = damping;
    result.feedforward = feedforward;
    return result;
  }

  public function copy():JointTarget
    return mode == JointTargetMode.Servo
      ? servo(joint, target, servoVelocity, stiffness, damping, feedforward)
      : new JointTarget(joint, mode, target);

  /** Validates and copies a complete batch so callers cannot mutate it mid-submit. */
  public static function copyBatch(values:Array<JointTarget>):Array<JointTarget> {
    if (values == null || values.length == 0 || values.length > MAX_BATCH_SIZE)
      throw 'Joint target batch must contain 1..$MAX_BATCH_SIZE targets';
    var seen = new Map<Int, Bool>();
    var result:Array<JointTarget> = [];
    for (value in values) {
      if (value == null)
        throw "Joint target batch cannot contain null values";
      if (seen.exists(value.joint))
        throw 'Joint target batch contains joint ${value.joint} more than once';
      seen.set(value.joint, true);
      result.push(value.copy());
    }
    return result;
  }
}
