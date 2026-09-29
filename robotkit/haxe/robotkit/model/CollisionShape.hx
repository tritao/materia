package robotkit.model;

/** Primitive collision geometry. Sizes are in metres; lengths run along local Z. */
enum CollisionPrimitive {
  Box(halfX:Float, halfY:Float, halfZ:Float);
  Sphere(radius:Float);
  /** A capsule's half-length covers only its straight part, excluding the caps. */
  Capsule(radius:Float, halfLength:Float);
  Cylinder(radius:Float, halfLength:Float);
}

/**
 * How a collision shape touches others. Zero fields keep the simulator's
 * default. friction is sliding, torsional and rolling; frictionDimensions is 1
 * (frictionless), 3 (sliding), 4 (with torsional) or 6 (with rolling);
 * contactTimeConstant (s) and contactDampingRatio set how stiff and how damped
 * the soft contact is.
 */
class ContactSurface {
  public var friction:Array<Float>;
  public var frictionDimensions:Int;
  public var contactTimeConstant:Float;
  public var contactDampingRatio:Float;

  public function new(?friction:Array<Float>, ?frictionDimensions:Int = 0,
      ?contactTimeConstant:Float = 0.0, ?contactDampingRatio:Float = 0.0) {
    this.friction = friction == null ? [0.0, 0.0, 0.0] : friction;
    this.frictionDimensions = frictionDimensions;
    this.contactTimeConstant = contactTimeConstant;
    this.contactDampingRatio = contactDampingRatio;
  }

  public function validate():Null<String> {
    if (friction == null || friction.length != 3) return "contact friction needs three values";
    for (value in friction)
      if (!Math.isFinite(value) || value < 0.0) return "contact friction must be finite and non-negative";
    if (frictionDimensions != 0 && frictionDimensions != 1 && frictionDimensions != 3 &&
        frictionDimensions != 4 && frictionDimensions != 6)
      return "friction dimensions must be 0, 1, 3, 4 or 6";
    if (!Math.isFinite(contactTimeConstant) || contactTimeConstant < 0.0 ||
        !Math.isFinite(contactDampingRatio) || contactDampingRatio < 0.0)
      return "contact time constant and damping ratio must be finite and non-negative";
    return null;
  }
}

/** Which contacts a collision shape takes part in. */
enum ShapeContact {
  /** Collides like the rest of the robot: with the environment and, unless disabled, other links. */
  Layers;
  /** Collides only through the robot's contact pairs. */
  PairsOnly;
  /** Collides through contact pairs and with every environment object, using its own surface. */
  PairsAndEnvironment;
}

/** One primitive collision shape, posed in its link's frame. */
class CollisionShape {
  public var primitive:CollisionPrimitive;
  public var position:Array<Float>;
  /** Unit quaternion in x, y, z, w order. */
  public var rotation:Array<Float>;
  /** Contact surface; null keeps the simulator's defaults. */
  public var surface:Null<ContactSurface> = null;
  public var contact:ShapeContact = ShapeContact.Layers;

  public function new(primitive:CollisionPrimitive, ?position:Array<Float>,
      ?rotation:Array<Float>) {
    if (primitive == null) throw "Collision shape primitive is required";
    this.primitive = primitive;
    this.position = position == null ? [0.0, 0.0, 0.0] : position;
    this.rotation = rotation == null ? [0.0, 0.0, 0.0, 1.0] : rotation;
  }

  /** Returns why the shape is invalid, or null when it is valid. */
  public function validate():Null<String> {
    var sizes = switch primitive {
      case Box(x, y, z): [x, y, z];
      case Sphere(radius): [radius];
      case Capsule(radius, half), Cylinder(radius, half): [radius, half];
    };
    for (size in sizes)
      if (!Math.isFinite(size) || size <= 0.0) return "collision shape sizes must be finite and positive";
    if (position == null || position.length != 3) return "collision shape position needs three values";
    for (value in position) if (!Math.isFinite(value)) return "collision shape position must be finite";
    if (rotation == null || rotation.length != 4) return "collision shape rotation needs four values";
    var norm = 0.0;
    for (value in rotation) {
      if (!Math.isFinite(value)) return "collision shape rotation must be finite";
      norm += value * value;
    }
    if (Math.abs(norm - 1.0) > 1e-6) return "collision shape rotation must be a unit quaternion";
    var surfaceValue = surface;
    return surfaceValue == null ? null : surfaceValue.validate();
  }
}
