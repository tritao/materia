package motionkit.path;

/** Geometric path primitive parameterized by travelled arc length. */
interface PathPrimitive {
  function length():Float;
  function pointAt(distance:Float):PathPoint;
  /** Unit tangent in increasing path-distance direction. */
  function tangentAt(distance:Float):Array<Float>;
  /** Signed planar curvature; zero denotes a straight primitive. */
  function curvatureAt(distance:Float):Float;
}
