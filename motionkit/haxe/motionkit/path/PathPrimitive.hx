package motionkit.path;

/** Geometric path primitive parameterized by travelled arc length. */
interface PathPrimitive {
  function length():Float;
  function pointAt(distance:Float):PathPoint;
}
