package kinematicskit;

/** How a compiled joint moves its child: not at all, about its axis, or along it. */
enum abstract JointKind(Int) {
  var Fixed = 0;
  var Revolute = 1;
  var Prismatic = 2;
}
