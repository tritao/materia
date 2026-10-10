package motionkit.planner;

/**
 * Plans a collision-free path in joint space between two configurations
 * (COLLISION.md CL-D7, CL7). The path is a list of waypoints, start and goal
 * included, whose straight edges have been checked; a timed path must follow
 * them (rest to rest with phase synchronization).
 */
interface MotionPlanner {
  function plan(start:Array<Float>, goal:Array<Float>):MotionPlan;
}
