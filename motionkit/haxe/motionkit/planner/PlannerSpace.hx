package motionkit.planner;

/**
 * The configuration space a `MotionPlanner` searches: joint bounds, which
 * configurations are valid, and which straight edges are clear. Edge checks
 * come in batches, so a space backed by native collision checks makes one
 * call for many edges (COLLISION.md CL-D7).
 */
interface PlannerSpace {
  function dimension():Int;
  function lower():Array<Float>;
  function upper():Array<Float>;
  /** Why `q` is not valid (what collides, named), or null when it is. */
  function invalid(q:Array<Float>):Null<String>;
  /**
   * Whether each straight edge is clear, conservatively (an edge it cannot
   * prove clear counts as blocked). `edges` holds `count` edges, each its
   * start then its end (`dimension()` values each). Ends are known valid.
   */
  function edgesClear(edges:Array<Float>, count:Int):Array<Bool>;
}
