package kinematicskit;

/**
 * Collision objects attached to the bodies of a `KinematicModel` (or to the
 * world, body -1) and posed together from a `KinematicState`
 * (COLLISION.md CL-D1, CL-D2). Object ids are never reused.
 *
 * Only `Checked` pairs are queried (`CollisionPairStatus` says why others
 * are not). A checked pair whose shapes cannot be compared makes queries
 * throw: a check is never silently skipped.
 *
 * `NativeCollisionWorld` (kinematicskit-native, on coal) implements it.
 * Without native code nothing does yet: callers report collision as
 * unchecked, never as passed.
 */
interface CollisionWorld {
  /** Adds a shape on `body` (-1: the world) at `offset` in the body's frame; returns its id. */
  function add(body:Int, offset:Transform, geometry:CollisionGeometry):Int;
  /** Replaces a height field's heights (the same grid), e.g. after digging. */
  function setHeights(object:Int, heights:Array<Float>):Void;
  /** Moves an object onto `body` (-1: the world) at `offset`: a grasped or placed part. */
  function attach(object:Int, body:Int, offset:Transform):Void;
  function remove(object:Int):Void;
  function setPairRule(a:Int, b:Int, rule:CollisionPairRule):Void;
  function pairStatus(a:Int, b:Int):CollisionPairStatus;
  /**
   * Marks the pairs colliding in `state` (the group's reference
   * configuration) as `OverlapsAtReference`, replacing earlier marks, and
   * leaves the world posed there. Returns how many.
   */
  function allowOverlapping(state:KinematicState):Int;
  /** Poses every object for `state`. */
  function update(state:KinematicState):Void;
  /** Checked pairs closer than `margin` (0: touching or penetrating), in id order. */
  function colliding(margin:Float):Array<CollisionPair>;
  /** Checked pairs closer than `queryDistance`, closest first. */
  function distances(queryDistance:Float):Array<CollisionDistance>;
  function dispose():Void;
}
