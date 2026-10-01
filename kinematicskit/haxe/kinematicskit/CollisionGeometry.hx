package kinematicskit;

/**
 * A collision shape, in its body's frame after the offset it is added with
 * (COLLISION.md CL-D1). Lengths are in the model's unit; axial shapes run
 * along local z.
 */
enum CollisionGeometry {
  Box(halfX:Float, halfY:Float, halfZ:Float);
  Sphere(radius:Float);
  /** A segment of length 2·halfLength along z, swept by a sphere. */
  Capsule(radius:Float, halfLength:Float);
  Cylinder(radius:Float, halfLength:Float);
  /** The solid side of a plane: n·p ≤ offset. */
  HalfSpace(nx:Float, ny:Float, nz:Float, offset:Float);
  /** The convex hull of points (x, y, z each; four or more, not coplanar). Pass the hull's vertices. */
  Convex(points:Array<Float>);
  /** A triangle mesh: vertices x, y, z each, three indices per triangle. */
  Mesh(vertices:Array<Float>, indices:Array<Int>);
  /**
   * A grid of `rows` x (heights.length / rows) heights, row-major, centred on
   * the offset over xSize by ySize; columns run along +x, rows along -y. A
   * solid down to `minHeight`.
   */
  HeightField(xSize:Float, ySize:Float, heights:Array<Float>, rows:Int, minHeight:Float);
}
