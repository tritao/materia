package app;

import CadKit;
import cadkit.modeling.Vector;

/** Axis-aligned box proxy derived from a CAD body's actual local bounds. */
class CadCollisionBounds {
  public final center:Vector;
  public final halfExtents:Vector;

  public function new(center:Vector, halfExtents:Vector) {
    if (center == null || halfExtents == null || halfExtents.x <= 0 ||
        halfExtents.y <= 0 || halfExtents.z <= 0)
      throw "CAD collision bounds must have positive half extents";
    this.center = center;
    this.halfExtents = halfExtents;
  }

  public static function fromKernelBounds(bounds:Bounds, unitScale:Float,
      offsetX:Float = 0, offsetY:Float = 0, offsetZ:Float = 0):CadCollisionBounds {
    if (bounds == null || !Math.isFinite(unitScale) || unitScale <= 0)
      throw "CAD collision bounds require finite bounds and scale";
    var minimum = bounds.get_min();
    var maximum = bounds.get_max();
    return new CadCollisionBounds(
      new Vector((minimum.get_x() + maximum.get_x()) * 0.5 * unitScale + offsetX,
        (minimum.get_y() + maximum.get_y()) * 0.5 * unitScale + offsetY,
        (minimum.get_z() + maximum.get_z()) * 0.5 * unitScale + offsetZ),
      new Vector((maximum.get_x() - minimum.get_x()) * 0.5 * unitScale,
        (maximum.get_y() - minimum.get_y()) * 0.5 * unitScale,
        (maximum.get_z() - minimum.get_z()) * 0.5 * unitScale));
  }
}
