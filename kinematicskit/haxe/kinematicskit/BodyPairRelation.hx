package kinematicskit;

/** Why two bodies of one model need no collision check (COLLISION.md CL-D3). */
enum abstract BodyPairRelation(Int) to Int {
  /** They never move relative to each other: joined only through fixed joints. */
  var Rigid = 0;
  /** One movable joint apart, through fixed joints on either side. */
  var Adjacent = 1;
  /** Joined by a closure (KK-D4): the two sides meet by design. */
  var Closure = 2;
}
