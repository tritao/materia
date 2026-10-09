package collisionkit;

/**
 * A `CollisionDescription` built into a world: the world's id of every
 * described body and object, how many pairs overlapped at their
 * articulation's reference, and the layout errors: pairs colliding at the
 * reference between an articulation and anything outside it (CL-D3).
 */
class CollisionBuild {
  public final world:CollisionWorld;
  public final bodies:Array<Int>;
  public final objects:Array<Int>;
  public final overlapsAtReference:Int;
  public final layoutErrors:Array<CollisionPair>;

  public function new(world:CollisionWorld, bodies:Array<Int>, objects:Array<Int>, overlapsAtReference:Int,
      layoutErrors:Array<CollisionPair>) {
    this.world = world;
    this.bodies = bodies;
    this.objects = objects;
    this.overlapsAtReference = overlapsAtReference;
    this.layoutErrors = layoutErrors;
  }

  /** The described object a world object came from, or -1. */
  public function objectOf(worldObject:Int):Int return objects.indexOf(worldObject);

  /** The described body a world body came from, or -1 (the world). */
  public function bodyOf(worldBody:Int):Int return worldBody < 0 ? -1 : bodies.indexOf(worldBody);
}
