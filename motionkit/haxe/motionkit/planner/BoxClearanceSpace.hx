package motionkit.planner;

/**
 * A planner space that can also prove whole axis-aligned boxes of configurations clear, so
 * motion that leaves the straight edges (a curve through a waypoint) can be certified by
 * covering it with boxes.
 */
interface BoxClearanceSpace extends PlannerSpace {
  /**
   * Whether every configuration within each box is clear, conservatively (a box it cannot
   * prove clear counts as blocked). `centres` and `halfWidths` hold `count` boxes of
   * `dimension()` values each; a box spans centre ± half width in every coordinate.
   */
  function boxesClear(centres:Array<Float>, halfWidths:Array<Float>, count:Int):Array<Bool>;
}
