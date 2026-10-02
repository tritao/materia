package robotkit.tool;

/** Where a body is in the world: a position and an xyzw rotation, as a link pose is. */
typedef WeldBodyPose = {position:Array<Float>, rotation:Array<Float>};

/**
 * Grounded work made of convex solids, each carried by a body whose pose is read when asked, so the work
 * follows what carries it (a fixture on a moving base, a workpiece on a positioner).
 */
class GroundedWork implements WeldWork {
  final solids:Array<ConvexSolid> = [];
  final poses:Array<Void -> WeldBodyPose> = [];

  public function new() {}

  /** Adds a solid whose vertices are in the frame of the body `pose` reports. */
  public function add(solid:ConvexSolid, pose:Void -> WeldBodyPose):GroundedWork {
    if (solid == null || pose == null) throw "Grounded work needs a solid and its pose";
    solids.push(solid);
    poses.push(pose);
    return this;
  }

  /** A solid fixed in the world. */
  public function addFixed(solid:ConvexSolid):GroundedWork {
    var pose:WeldBodyPose = {position: [0.0, 0.0, 0.0], rotation: [0.0, 0.0, 0.0, 1.0]};
    return add(solid, () -> pose);
  }

  public function count():Int return solids.length;

  public function distance(x:Float, y:Float, z:Float):Float {
    var result = Math.POSITIVE_INFINITY;
    for (index in 0...solids.length) {
      var local = toLocal(poses[index](), x, y, z);
      result = Math.min(result, solids[index].distance(local[0], local[1], local[2]));
    }
    return result;
  }

  public function ray(ox:Float, oy:Float, oz:Float, dx:Float, dy:Float, dz:Float, range:Float):Float {
    var result = Math.POSITIVE_INFINITY;
    for (index in 0...solids.length) {
      var pose = poses[index]();
      var origin = toLocal(pose, ox, oy, oz);
      var direction = turnBack(pose.rotation, dx, dy, dz);
      result = Math.min(result, solids[index].ray(origin[0], origin[1], origin[2], direction[0], direction[1], direction[2], range));
    }
    return result;
  }

  /** The world point in the body's frame. */
  static function toLocal(pose:WeldBodyPose, x:Float, y:Float, z:Float):Array<Float>
    return turnBack(pose.rotation, x - pose.position[0], y - pose.position[1], z - pose.position[2]);

  /** `v` turned by the inverse of the xyzw quaternion `q`. */
  static function turnBack(q:Array<Float>, vx:Float, vy:Float, vz:Float):Array<Float> {
    var x = -q[0], y = -q[1], z = -q[2], w = q[3];
    var tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
    return [vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx];
  }
}
