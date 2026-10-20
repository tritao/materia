package camkit;

/** Relative to a clockwise spindle. Outside climb is CW; inside climb is CCW. */
enum abstract CamCutDirection(String) to String {
  var Climb = "climb";
  var Conventional = "conventional";
}
