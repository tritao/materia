package robotkit.model;

class Joint {
  public final id:JointId;
  public var name:String;
  /** Assembly include that owns this joint; null for non-assembly models. */
  public var includePath:Null<String> = null;
  public final type:JointType;
  public final parent:Link;
  public final child:Link;
  public var limits:JointLimits;
  /** Mechanical inputs retained when `limits` becomes the compiled effective result. */
  public var mechanicalLimits:Null<JointLimits>;
  public var parentFramePosition:Array<Float> = [0.0, 0.0, 0.0];
  public var parentFrameRotation:Array<Float> = [0.0, 0.0, 0.0, 1.0];
  public var childFramePosition:Array<Float> = [0.0, 0.0, 0.0];
  public var childFrameRotation:Array<Float> = [0.0, 0.0, 0.0, 1.0];
  public var axis:Array<Float> = [0.0, 0.0, 1.0];
  /** Reflected rotor inertia: kg m^2, or kg for a prismatic joint. */
  public var armature:Float = 0.0;
  /** Viscous effort per unit joint velocity. */
  public var damping:Float = 0.0;
  /** Dry friction effort. */
  public var frictionLoss:Float = 0.0;
  /**
   * How soft the joint's limits are, like a soft contact: time constant (s)
   * and damping ratio, and MuJoCo's five-term impedance curve. Zeros keep the
   * simulator's default.
   */
  public var limitTimeConstant:Float = 0.0;
  public var limitDampingRatio:Float = 0.0;
  public var limitImpedance:Array<Float> = [0.0, 0.0, 0.0, 0.0, 0.0];

  public function new(name:String, type:JointType, parent:Link, child:Link, ?id:JointId) {
    // Authors can use the initial name once; imports pass the stored ID.
    this.id = id == null ? name : id;
    this.name = name;
    this.type = type;
    this.parent = parent;
    this.child = child;
    this.limits = new JointLimits();
  }
}
