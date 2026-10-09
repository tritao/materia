package collisionkit;

/**
 * Collision objects attached to bodies the caller registers and poses
 * (COLLISION.md CL-D1, CL-D2). The world knows bodies, not models: one
 * world holds a whole cell (several robots and mechanisms, each the
 * caller's own model, and the environment). Body -1 is the world itself.
 * Bodies are numbered from 0 in the order they are added; object ids are
 * never reused.
 *
 * Only `Checked` pairs are queried (`CollisionPairStatus` says why others
 * are not). A checked pair whose shapes cannot be compared makes queries
 * throw: a check is never silently skipped.
 *
 * `collisionkit.native.NativeCollisionWorld` (collisionkit-native, on coal)
 * implements it. Without a backend nothing does: callers report collision
 * as unchecked, never as passed.
 */
interface CollisionWorld {
  /** Adds a body in `group` (CL-D4's pair classes, from 0), at the identity and not static; returns its number. */
  function addBody(group:Int):Int;
  function bodyCount():Int;
  function setBodyGroup(body:Int, group:Int):Void;
  /** A static body is fixed in this cell: pairs between static bodies are never checked. */
  function setBodyStatic(body:Int, fixed:Bool):Void;
  function setBodyPose(body:Int, pose:CollisionPose):Void;
  /** Poses bodies `first`, `first + 1`, ... from seven numbers each (x, y, z, qx, qy, qz, qw). */
  function setBodyPoses(first:Int, poses:Array<Float>):Void;

  /** Adds a shape on `body` (-1: the world) at `offset` in the body's frame; returns its id. */
  function add(body:Int, offset:CollisionPose, geometry:CollisionGeometry):Int;
  /** Inflates a primitive or convex object by `radius`: every distance to it shrinks by that much. */
  function setInflation(object:Int, radius:Float):Void;
  /** Replaces a height field's heights (the same grid, none below its minimum), e.g. after digging. */
  function setHeights(object:Int, heights:Array<Float>):Void;
  /** Moves an object onto `body` (-1: the world) at `offset`: a grasped or placed part. It follows the new body's rules. */
  function attach(object:Int, body:Int, offset:CollisionPose):Void;
  function remove(object:Int):Void;

  /** A rule for every object pair between two bodies; `reason` is what an allow rule records. */
  function setBodyRule(a:Int, b:Int, rule:CollisionPairRule, reason:CollisionPairStatus):Void;
  /** A rule for one object pair (process contact); it overrides body rules. */
  function setObjectRule(a:Int, b:Int, rule:CollisionPairRule, reason:CollisionPairStatus):Void;
  function pairStatus(a:Int, b:Int):CollisionPairStatus;
  /**
   * Over `bodies` (one articulation, posed at its reference configuration),
   * marks the object pairs colliding now as `OverlapsAtReference`, replacing
   * earlier marks among them. Pairs reaching outside the set stay checked:
   * an overlap with the environment is a layout error. Returns how many.
   */
  function allowOverlapping(bodies:Array<Int>):Int;

  /** Checked pairs closer than `margin` (0: touching or penetrating), in id order. */
  function colliding(margin:Float):Array<CollisionPair>;
  /** Checked pairs closer than `queryDistance`, closest first. */
  function distances(queryDistance:Float):Array<CollisionDistance>;
  /** The distances of chosen pairs, whatever their status, each as given (`a` first). */
  function pairDistances(pairs:Array<CollisionPair>):Array<CollisionDistance>;
  /**
   * The first checked pair, in id order, closer than its group margin plus
   * both bodies' `inflation` (one per body; null for none), or null.
   */
  function violation(margins:CollisionMargins, ?inflation:Array<Float>):Null<CollisionViolation>;
  /** The closest checked pair, clear or not, with its group margin as `required`; null when none is checked. */
  function closest(margins:CollisionMargins):Null<CollisionViolation>;
  /**
   * Batched violations: `poses` holds sets of every body's pose (seven
   * numbers per body), `inflation` one number per body per set (or null).
   * The first set with a violation reports it, with `set` its index; null
   * when every set is clear. The world is left posed at the last set checked.
   */
  function firstViolation(poses:Array<Float>, margins:CollisionMargins, ?inflation:Array<Float>):Null<CollisionViolation>;
  function dispose():Void;
}
