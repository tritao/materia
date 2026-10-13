package collisionkit;

/** Cell margin classes; exact process-interface margins are sparse object-pair overrides. */
enum abstract CollisionMarginClass(Int) to Int {
  var Air = 0;
  var Installed = 1;
  var Tool = 2;
  var Work = 3;
  var Held = 4;
}
