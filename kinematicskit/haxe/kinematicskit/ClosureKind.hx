package kinematicskit;

/**
 * What a loop-closure constraint holds between its two frames: the whole
 * pose (Fixed), a shared point and axis (Revolute), or a shared axis line
 * with free sliding along it (Prismatic).
 */
enum abstract ClosureKind(Int) {
  var Fixed = 0;
  var Revolute = 1;
  var Prismatic = 2;
}
