package robotkit.model;

import robotkit.model.CollisionShape;

/**
 * An explicit contact between two collision shapes on different links, named
 * by link ID and index into that link's collisionShapes, with its own surface.
 * It applies whatever the shapes' own contact modes say.
 */
class ContactPair {
  public var linkA:LinkId;
  public var shapeA:Int;
  public var linkB:LinkId;
  public var shapeB:Int;
  public var surface:ContactSurface;

  public function new(linkA:LinkId, shapeA:Int, linkB:LinkId, shapeB:Int, ?surface:ContactSurface) {
    this.linkA = linkA;
    this.shapeA = shapeA;
    this.linkB = linkB;
    this.shapeB = shapeB;
    this.surface = surface == null ? new ContactSurface() : surface;
  }
}
