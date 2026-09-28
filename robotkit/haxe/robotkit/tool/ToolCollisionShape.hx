package robotkit.tool;

import robotkit.spatial.Vec3;

/**
 * Simple tool collision approximation in the flange frame. A Box centre
 * defaults to the flange origin when omitted.
 * `model.CollisionApproximation`
 * is a link-geometry derivation policy, not a shape; tools need an actual
 * box/cylinder value instead.
 */
enum ToolCollisionShape {
  NoCollision;
  Box(halfExtents:Vec3, ?centre:Vec3);
  Cylinder(radius:Float, height:Float);
  /** Convex pieces in the flange frame, in metres. */
  Hulls(pieces:Array<Array<Float>>, padding:Float);
}
