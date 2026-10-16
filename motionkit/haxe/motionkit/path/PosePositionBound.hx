package motionkit.path;

/** Optional continuous position bound for a Cartesian quintic interpolating part of this primitive.
 * Controls are three rows of six Bernstein ordinates on [from,to]. Infinity means no proof.
 */
interface PosePositionBound {
  function positionErrorBound(control:Array<Array<Float>>,from:Float,to:Float):Float;
}
