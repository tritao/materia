package kinematicskit;

/** A declared rule for one pair of collision objects. */
enum abstract CollisionPairRule(Int) to Int {
  /** No declared rule: the pair's status follows the defaults. */
  var Default = 0;
  var Allow = 1;
  /** Checked even where the defaults would not check it. */
  var Check = 2;
}
