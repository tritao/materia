package robotkit.model;

import collisionkit.CollisionGeometry;


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

/** One collision shape, posed in its link's frame. */
class CollisionShape {
  public var geometry:CollisionGeometry;
  public var inflation:Float = 0.0;
  /** Source identity, such as a CAD occurrence id. */
  public var name:String = "";
  public var position:Array<Float>;
  /** Unit quaternion in x, y, z, w order. */
  public var rotation:Array<Float>;
  /** Contact surface; null keeps the simulator's defaults. */
  public var surface:Null<ContactSurface> = null;
  public var contact:ShapeContact = ShapeContact.Layers;

  public function new(geometry:CollisionGeometry, ?position:Array<Float>,
      ?rotation:Array<Float>) {
    if (geometry == null) throw "Collision shape geometry is required";
    this.geometry = geometry;
    this.position = position == null ? [0.0, 0.0, 0.0] : position;
    this.rotation = rotation == null ? [0.0, 0.0, 0.0, 1.0] : rotation;
  }

  /** Convex vertices in the link frame, including this shape's pose. */
  public function convexVertices():Array<Float> {
    var points = switch geometry { case Convex(value): value; case _: throw "Shape is not convex"; };
    var result:Array<Float> = [];
    var qx = rotation[0], qy = rotation[1], qz = rotation[2], qw = rotation[3];
    for (i in 0...Std.int(points.length / 3)) {
      var x = points[3*i], y = points[3*i+1], z = points[3*i+2];
      var tx = 2*(qy*z-qz*y), ty = 2*(qz*x-qx*z), tz = 2*(qx*y-qy*x);
      result.push(position[0]+x+qw*tx+qy*tz-qz*ty);
      result.push(position[1]+y+qw*ty+qz*tx-qx*tz);
      result.push(position[2]+z+qw*tz+qx*ty-qy*tx);
    }
    return result;
  }

  /** Returns why the shape is invalid, or null when it is valid. */
  public function validate():Null<String> {
    if (!Math.isFinite(inflation) || inflation < 0.0) return "collision inflation must be finite and non-negative";
    if (geometry == null) return "collision geometry is required";
    var sizes = switch geometry {
      case Box(x, y, z): [x, y, z];
      case Sphere(radius): [radius];
      case Capsule(radius, half), Cylinder(radius, half): [radius, half];
      case Convex(points):
        if (points == null || points.length < 12 || points.length % 3 != 0) return "convex collision needs at least four XYZ points";
        for (value in points) if (!Math.isFinite(value)) return "convex collision points must be finite";
        [];
      case Mesh(_, _), HeightField(_, _, _, _, _), HalfSpace(_, _, _, _):
        return "scene geometry is not accepted on a link";
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
