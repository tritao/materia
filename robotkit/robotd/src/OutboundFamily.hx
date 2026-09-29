package robotd;

/** RKF1 outbound traffic classes; future families add a value here. */
enum abstract OutboundFamily(String) from String to String {
  var Essential = "essential";
  var Sensor = "sensor";
  var Camera = "camera";
  var Observation = "observation";
}
