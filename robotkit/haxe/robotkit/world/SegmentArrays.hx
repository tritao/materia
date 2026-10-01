package robotkit.world;

import haxe.Int64;
import runtime.memory.NativeSpan;

/**
  A run of polynomial segments held as arrays in native memory, such as a
  MotionKit plan's, for an in-process runtime to read in place. Segment `i`
  starts `starts[i] - starts[0]` after the run's start, lasts `durations[i]`,
  and has degree `degrees[i]`. The arrays' joints are source joints: source
  joint `j` drives robot joint `jointMap[j]`, with its coefficient of power `p`
  at `coefficients[(i * jointMap.length + j) * STRIDE + p]`; a robot joint no
  source joint drives holds its `heldPositions` value.

  The spans are borrowed: they stay readable while their owner is open, so a
  submission carrying them is consumed while its plan is alive. `segments()`
  copies them into `TrajectorySegment`s for robots that need managed values.
**/
class SegmentArrays {
  /** Coefficients stored per joint and segment: degree zero through five. */
  public static inline final STRIDE = 6;

  public final starts:NativeSpan<Int64>;
  public final durations:NativeSpan<Int64>;
  public final degrees:NativeSpan<Int>;
  public final coefficients:NativeSpan<Float>;
  public final jointMap:Array<Int>;
  public final heldPositions:Array<Float>;

  public function new(starts:NativeSpan<Int64>, durations:NativeSpan<Int64>, degrees:NativeSpan<Int>,
      coefficients:NativeSpan<Float>, jointMap:Array<Int>, heldPositions:Array<Float>) {
    var count = starts.length();
    if (count < 1 || durations.length() != count || degrees.length() != count)
      throw "Segment arrays need equal, nonempty start, duration and degree arrays";
    if (jointMap == null || jointMap.length < 1 || heldPositions == null ||
        jointMap.length > heldPositions.length || heldPositions.length > 64)
      throw "Segment arrays need a joint map onto the robot's joints";
    if (coefficients.length() != count * jointMap.length * STRIDE)
      throw "Segment arrays need STRIDE coefficients per segment and source joint";
    var driven = [for (_ in heldPositions) false];
    for (joint in jointMap) {
      if (joint < 0 || joint >= heldPositions.length || driven[joint])
        throw "Segment arrays map each source joint to a distinct robot joint";
      driven[joint] = true;
    }
    for (position in heldPositions)
      if (!Math.isFinite(position)) throw "Segment arrays need finite held positions";
    this.starts = starts;
    this.durations = durations;
    this.degrees = degrees;
    this.coefficients = coefficients;
    this.jointMap = jointMap.copy();
    this.heldPositions = heldPositions.copy();
  }

  public function count():Int return starts.length();

  /** The robot joint count the segments drive. */
  public function robotJointCount():Int return heldPositions.length;

  /** Segments `[first, last)`, reading the same memory. */
  public function slice(first:Int, last:Int):SegmentArrays {
    var joints = jointMap.length * STRIDE;
    return new SegmentArrays(starts.slice(first, last - first), durations.slice(first, last - first),
      degrees.slice(first, last - first), coefficients.slice(first * joints, (last - first) * joints),
      jointMap, heldPositions);
  }

  /** Robot joint `joint`'s coefficient of `power` in segment `index`. */
  public function coefficient(index:Int, joint:Int, power:Int):Float {
    var source = jointMap.indexOf(joint);
    if (source < 0) return power == 0 ? heldPositions[joint] : 0.0;
    return coefficients.get((index * jointMap.length + source) * STRIDE + power);
  }

  /** The segments as managed values over every robot joint, timed from the first one's start. */
  public function segments():Array<TrajectorySegment> {
    var origin = starts.get(0), result:Array<TrajectorySegment> = [];
    for (index in 0...count()) {
      var degree = degrees.get(index);
      result.push(new TrajectorySegment(Int64.sub(starts.get(index), origin), durations.get(index),
        [for (joint in 0...heldPositions.length) [for (power in 0...degree + 1) coefficient(index, joint, power)]]));
    }
    return result;
  }
}
