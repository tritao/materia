package robotkit.runtime;

/** The owner of the geometry opposite the queried robot link. */
enum abstract RobotContactOtherKind(Int) from Int to Int {
  var World = 0;
  var Object = 1;
  var RobotLink = 2;
}
