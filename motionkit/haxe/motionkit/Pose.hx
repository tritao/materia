package motionkit;

import motionkit.path.PathPoint;

/** Root-package convenience constructors for Cartesian motion targets. */
class Pose {
  public static function xyz(x:Float, y:Float, z:Float):PathPoint
    return new PathPoint(x, y, z);
}
