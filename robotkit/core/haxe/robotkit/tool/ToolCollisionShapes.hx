package robotkit.tool;

import robotkit.spatial.Vec3;

/** Bounds of tool collision geometry in the flange frame. */
class ToolCollisionShapes {
  public static function bounds(shape:ToolCollisionShape):{centre:Vec3, halfExtents:Vec3} {
    var minimum = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
    var maximum = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
    switch shape {
      case NoCollision: throw "Tool has no collision geometry";
      case Box(half, centre):
        if (half == null || half.x <= 0 || half.y <= 0 || half.z <= 0)
          throw "Tool box needs positive half extents";
        var c = centre == null ? Vec3.zero() : centre;
        minimum = [c.x - half.x, c.y - half.y, c.z - half.z];
        maximum = [c.x + half.x, c.y + half.y, c.z + half.z];
      case Cylinder(radius, height):
        if (!Math.isFinite(radius) || radius <= 0 || !Math.isFinite(height) || height <= 0)
          throw "Tool cylinder needs positive dimensions";
        minimum = [-radius, -radius, -height * 0.5];
        maximum = [radius, radius, height * 0.5];
      case Hulls(pieces, padding):
        if (pieces == null || pieces.length == 0 || !Math.isFinite(padding) || padding < 0)
          throw "Tool hulls need pieces and nonnegative padding";
        for (piece in pieces) {
          if (piece == null || piece.length < 12 || piece.length > 192 || piece.length % 3 != 0)
            throw "Tool hull piece needs 4–64 vertices";
          for (index in 0...piece.length) {
            var value = piece[index];
            if (!Math.isFinite(value)) throw "Tool hull vertex must be finite";
            var axis = index % 3;
            minimum[axis] = Math.min(minimum[axis], value - padding);
            maximum[axis] = Math.max(maximum[axis], value + padding);
          }
        }
    }
    return {centre: new Vec3((minimum[0] + maximum[0]) * 0.5,
      (minimum[1] + maximum[1]) * 0.5, (minimum[2] + maximum[2]) * 0.5),
      halfExtents: new Vec3((maximum[0] - minimum[0]) * 0.5,
        (maximum[1] - minimum[1]) * 0.5, (maximum[2] - minimum[2]) * 0.5)};
  }
}
