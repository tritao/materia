package collisionkit;

/** A declared rule for a pair of bodies or of objects. */
enum abstract CollisionPairRule(Int) to Int {
  /** No declared rule: the pair's status follows the defaults. */
  var Default = 0;
  /** Not checked, for a reason (`CollisionPairStatus`). */
  var Allow = 1;
  /** Checked even where the defaults would not check it. */
  var Check = 2;
}
