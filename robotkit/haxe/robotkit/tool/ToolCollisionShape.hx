package robotkit.tool;

import robotkit.spatial.Vec3;

/**
 * Tool collision geometry in the flange frame. A Box centre defaults to the
 * flange origin when omitted. Hull vertices and padding use metres.
 * `model.CollisionApproximation`
 * is a link-geometry derivation policy, not a shape; tools need an actual
 * shape value instead.
 */
enum ToolCollisionShape {
  NoCollision;
  Box(halfExtents:Vec3, ?centre:Vec3);
  Cylinder(radius:Float, height:Float);
  /** Convex pieces in the flange frame, in metres. */
  Hulls(pieces:Array<Array<Float>>, padding:Float);
}
