package motionkit.planner;

/** A planned path, or why there is none, with what planning cost. */
class MotionPlan {
  /** Waypoints, start and goal included; null when planning failed. */
  public final waypoints:Null<Array<Array<Float>>>;
  /** Why planning failed ("" when it did not): the start or goal in collision, named, or no path found. */
  public final failure:String;
  public final iterations:Int;
  public final nodes:Int;
  /** Edges checked, and the time spent in the space's checks and in planning overall. */
  public final edgeChecks:Int;
  public final checkSeconds:Float;
  public final totalSeconds:Float;

  public function new(waypoints:Null<Array<Array<Float>>>, failure:String, iterations:Int, nodes:Int, edgeChecks:Int,
      checkSeconds:Float, totalSeconds:Float) {
    this.waypoints = waypoints;
    this.failure = failure;
    this.iterations = iterations;
    this.nodes = nodes;
    this.edgeChecks = edgeChecks;
    this.checkSeconds = checkSeconds;
    this.totalSeconds = totalSeconds;
  }

  public function found():Bool return waypoints != null;

  /** The path's length in joint space (Euclidean over the waypoints). */
  public function length():Float {
    var path = waypoints;
    if (path == null) return Math.POSITIVE_INFINITY;
    var sum = 0.0;
    for (i in 1...path.length) sum += PlannerTree.distance(path[i - 1], path[i]);
    return sum;
  }
}
