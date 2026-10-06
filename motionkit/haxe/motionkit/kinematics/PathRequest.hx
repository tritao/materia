package motionkit.kinematics;

import motionkit.path.OrientationPolicy;

/**
 * What `KinematicsSolver.solvePath` is asked for: one joint configuration
 * per sample of a Cartesian path (`poses` at `distances`), starting at
 * `startQ`, no joint moving more than `maxJump` between neighbouring
 * samples. A search weighs each joint's motion by `1 / velocity` (its speed
 * limit) and keeps at most `maxCandidates` configurations per sample.
 */
class PathRequest {
  public final distances:Array<Float>;
  public final poses:Array<Pose3>;
  public final startQ:Array<Float>;
  public final tolerance:IkTolerance;
  public final maxJump:Array<Float>;
  public final velocity:Array<Float>;
  public final maxCandidates:Int;
  public final drives:Null<PathDriveLimits>;
  /** One hard orientation policy per sample. */
  public final freedoms:Array<OrientationPolicy>;

  public function new(distances:Array<Float>, poses:Array<Pose3>, startQ:Array<Float>, tolerance:IkTolerance,
      maxJump:Array<Float>, velocity:Array<Float>, ?maxCandidates:Int = 32, ?freedoms:Array<OrientationPolicy>,?drives:PathDriveLimits) {
    if (distances == null || poses == null || distances.length != poses.length || distances.length == 0 ||
        startQ == null || tolerance == null || maxJump == null || velocity == null ||
        maxJump.length != startQ.length || velocity.length != startQ.length || maxCandidates < 1)
      throw "A path request needs aligned samples, a start, a tolerance and per-joint limits";
    this.distances = distances;
    this.poses = poses;
    this.startQ = startQ.copy();
    this.tolerance = tolerance;
    this.maxJump = maxJump.copy();
    this.velocity = velocity.copy();
    this.maxCandidates = maxCandidates;
    if(drives!=null && (drives.acceleration.length!=startQ.length || drives.feed.length!=poses.length))
      throw "Drive refinement limits must match path samples and compiled joints";
    this.drives=drives;
    if (freedoms != null && freedoms.length != poses.length)
      throw "Path freedoms must align with pose samples";
    this.freedoms = freedoms == null ? [for (_ in poses) OrientationPolicy.Interpolated] : freedoms.copy();
    for (freedom in this.freedoms) if (freedom == null) throw "Path freedom cannot be null";
  }

  /**
   * Each sample solved from the previous one: `startQ`, then `solvePose`
   * seeded by the last answer. A sample that does not solve, and every one
   * after it, is null (the caller reports where the path became unreachable).
   */
  public function followPointByPoint(solver:KinematicsSolver):Array<Null<Array<Float>>> {
    var result:Array<Null<Array<Float>>> = [startQ.copy()];
    var previous:Null<Array<Float>> = startQ;
    for (index in 1...poses.length) {
      var seed:Null<Array<Float>> = previous;
      var solved = seed == null ? null : solver.solvePose(poses[index], seed, tolerance, freedoms[index]);
      result.push(solved);
      previous = solved;
    }
    return result;
  }
}
