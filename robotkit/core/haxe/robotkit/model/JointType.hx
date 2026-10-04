package robotkit.model;

enum abstract JointType(String) from String to String {
  var Fixed = "fixed";
  var Revolute = "revolute";
  var Continuous = "continuous";
  var Prismatic = "prismatic";
  var Floating = "floating";
}
