package kinematicskit;

/**
 * What a loop-closure constraint holds between its two frames: the whole
 * pose (Fixed), a shared point and axis (Revolute), a shared axis line
 * with free sliding along it (Prismatic), a shared point only (Spherical),
 * a shared axis line with free sliding and turning (Cylindrical), or B's
 * origin on A's plane with parallel normals (Planar; the axis is A's normal).
 * Prismatic is Cylindrical plus a row holding the twist about the axis.
 * Planar's B origin sits `value` along A's normal. Parallel, Perpendicular,
 * Distance and Angle exist for assembly mates.
 */
enum abstract ClosureKind(Int) {
  var Fixed = 0;
  var Revolute = 1;
  var Prismatic = 2;
  var Spherical = 3;
  var Cylindrical = 4;
  var Planar = 5;
  /** Axes parallel (either direction): the two axis rows. */
  var Parallel = 6;
  /** Axes perpendicular: a·b = 0. */
  var Perpendicular = 7;
  /** Origins `value` apart: |pB − pA| = value. */
  var Distance = 8;
  /** `value` radians between the axes: a·b = cos(value). */
  var Angle = 9;
}
