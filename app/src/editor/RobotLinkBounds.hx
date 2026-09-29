package app.editor;

import nativekit.sim.SimPose;
import robotkit.model.CollisionShape;
import robotkit.model.CollisionShape.CollisionPrimitive;
import robotkit.model.RobotModel;

/** Bounding spheres for a robot model's authored link collision shapes. */
class RobotLinkBounds {
  public static function shapes(model:RobotModel):Array<{link:Int, shape:Int, offset:Array<Float>, radius:Float}> {
    var result = [];
    for (link in 0...model.links.length)
      for (shape in 0...model.links[link].collisionShapes.length) {
        var collision = model.links[link].collisionShapes[shape];
        result.push({link:link, shape:shape, offset:collision.position.copy(),
          radius:radius(collision)});
      }
    return result;
  }

  public static function radius(shape:CollisionShape):Float
    return switch shape.primitive {
      case Sphere(value): value;
      case Capsule(value, half), Cylinder(value, half): value + half;
      case Box(x, y, z): Math.sqrt(x * x + y * y + z * z);
    };

  /** Applies a link's world rotation to a link-local collision centre. */
  public static function worldCenter(position:Array<Float>, rotation:Array<Float>, offset:Array<Float>):SimPose {
    var x = offset[0], y = offset[1], z = offset[2];
    var qx = rotation[0], qy = rotation[1], qz = rotation[2], qw = rotation[3];
    var tx = 2 * (qy * z - qz * y), ty = 2 * (qz * x - qx * z),
      tz = 2 * (qx * y - qy * x);
    return new SimPose(position[0] + x + qw * tx + qy * tz - qz * ty,
      position[1] + y + qw * ty + qz * tx - qx * tz,
      position[2] + z + qw * tz + qx * ty - qy * tx);
  }
}
