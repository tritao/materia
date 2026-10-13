package motionkit.trajectory;

/** Read-only timed joint motion. The caller retains ownership of its native storage. */
interface TimedJointTrajectory {
  function duration():Float;
  function evaluate(timeSeconds:Float):TrajectoryState;
}
