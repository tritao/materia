package robotkit.tool;

import robotkit.spatial.Vec3;

/**
 * Simple tool collision approximation, centred on the flange frame. Box and
 * Cylinder have no offset, so displaced geometry needs a conservative shape.
 * `model.CollisionApproximation`
 * is a link-geometry derivation policy, not a shape; tools need an actual
 * box/cylinder value instead.
 */
enum ToolCollisionShape {
  NoCollision;
  Box(halfExtents:Vec3);
  Cylinder(radius:Float, height:Float);
}
