package robotkit.world;

import haxe.Int64;

/** One joint-space polynomial segment, in seconds from its start knot. */
class TrajectorySegment {
  public final timeFromStartNs:Int64;
  public final durationNs:Int64;
  public final degree:Int;
  public final jointCount:Int;
  public final coefficients:Array<Array<Float>>;

  public function new(timeFromStartNs:Int64, durationNs:Int64,
      coefficients:Array<Array<Float>>) {
    if (timeFromStartNs == null || Int64.compare(timeFromStartNs, Int64.ofInt(0)) < 0 ||
        durationNs == null || Int64.compare(durationNs, Int64.ofInt(0)) <= 0)
      throw "Trajectory segment needs a nonnegative start and positive duration";
    if (coefficients == null || coefficients.length < 1 || coefficients.length > 64 ||
        coefficients[0] == null || coefficients[0].length < 1 ||
        coefficients[0].length > 6)
      throw "Trajectory segment coefficients are invalid";
    this.timeFromStartNs = timeFromStartNs;
    this.durationNs = durationNs;
    degree = coefficients[0].length - 1;
    jointCount = coefficients.length;
    this.coefficients = [];
    for (joint in coefficients) {
      if (joint == null || joint.length != degree + 1)
        throw "Trajectory segment joints must share one degree";
      for (value in joint)
        if (!Math.isFinite(value)) throw "Trajectory coefficients must be finite";
      this.coefficients.push(joint.copy());
    }
  }

  public function copy():TrajectorySegment
    return new TrajectorySegment(timeFromStartNs, durationNs, coefficients);
}
