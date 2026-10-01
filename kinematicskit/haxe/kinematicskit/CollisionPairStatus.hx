package kinematicskit;

/** Whether a pair of collision objects is checked, and why not (COLLISION.md CL-D3). */
enum abstract CollisionPairStatus(Int) to Int {
  var Checked = 0;
  /** Both on the world. */
  var Static = 1;
  /** Their bodies never move relative to each other (one body, or joined through fixed joints). */
  var Rigid = 2;
  /** Their bodies are one movable joint apart. */
  var Adjacent = 3;
  /** They overlapped at the reference configuration (`CollisionWorld.allowOverlapping`). */
  var OverlapsAtReference = 4;
  /** Allowed by a declared rule. */
  var Allowed = 5;
  /** Would be checked, but the shapes cannot be compared: queries fail until it is allowed or removed. */
  var Unsupported = 6;
}
