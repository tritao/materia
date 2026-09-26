package robotkit.tool;

import robotkit.spatial.Vec3;

/**
 * Simple tool collision approximation, expressed in the tool's own frame
 * (see `Tool.flangeTTcp`'s parent, the flange). `model.CollisionApproximation`
 * is a link-geometry derivation policy, not a shape; tools need an actual
 * box/cylinder value instead.
 */
enum ToolCollisionShape {
  NoCollision;
  Box(halfExtents:Vec3);
  Cylinder(radius:Float, height:Float);
}
