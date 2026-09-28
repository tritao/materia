package robotkit.model;

/** Primitive collision geometry. Sizes are in metres; lengths run along local Z. */
enum CollisionPrimitive {
  Box(halfX:Float, halfY:Float, halfZ:Float);
  Sphere(radius:Float);
  /** A capsule's half-length covers only its straight part, excluding the caps. */
  Capsule(radius:Float, halfLength:Float);
  Cylinder(radius:Float, halfLength:Float);
}

/** One primitive collision shape, posed in its link's frame. */
class CollisionShape {
  public var primitive:CollisionPrimitive;
  public var position:Array<Float>;
  /** Unit quaternion in x, y, z, w order. */
  public var rotation:Array<Float>;

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
    return null;
  }
}
