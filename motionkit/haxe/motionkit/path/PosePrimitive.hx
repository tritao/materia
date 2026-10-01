package motionkit.path;

interface PosePrimitive {
  function length():Float;
  function waypointAt(distance:Float):PoseWaypoint;
  function startWaypoint():PoseWaypoint;
  function endWaypoint():PoseWaypoint;
  function speedLimit():Float;
  function orientationPolicy():OrientationPolicy;
  /** Derivatives of the pose with respect to distance along this primitive. */
  function derivativesAt(distance:Float):PoseDerivatives;
}
