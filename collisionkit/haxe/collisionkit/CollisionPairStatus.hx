package collisionkit;

/**
 * Whether a pair of collision objects is checked, and why not
 * (COLLISION.md CL-D3). An allow rule records one of the reasons here.
 */
enum abstract CollisionPairStatus(Int) to Int {
  var Checked = 0;
  /** Both bodies are static (fixed in this cell; the world is). */
  var Static = 1;
  /** They never move relative to each other: one body, or bodies joined only through fixed joints. */
  var Rigid = 2;
  /** Their bodies are one movable joint apart. */
  var Adjacent = 3;
  /** They overlapped at their articulation's reference configuration (`CollisionWorld.allowOverlapping`). */
  var OverlapsAtReference = 4;
  /** Allowed by a declared rule. */
  var Allowed = 5;
  /** Would be checked, but the shapes cannot be compared: queries fail until it is allowed or removed. */
  var Unsupported = 6;
  /** Joined by a closure of the caller's model. */
  var Closure = 7;
  /** Process contact (a torch on its seam), allowed per window (CL-D9). */
  var ProcessContact = 8;
  /** Meant to touch: RobotKit's contact pairs (CL-D10). */
  var DeclaredContact = 9;
}
