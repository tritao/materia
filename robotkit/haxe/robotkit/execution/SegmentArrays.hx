package robotkit.execution;

import robotkit.core.CoupledJoint;

import haxe.Int64;
import runtime.memory.NativeSpan;

/**
  A run of polynomial segments held as arrays in native memory, such as a
  MotionKit plan's, for an in-process runtime to read in place. Segment `i`
  starts `starts[i] - starts[0]` after the run's start, lasts `durations[i]`,
  and has degree `degrees[i]`. The arrays' joints are source joints: source
  joint `j` drives robot joint `jointMap[j]`, with its coefficient of power `p`
  at `coefficients[(i * jointMap.length + j) * STRIDE + p]`. A robot joint no
  source joint drives follows its leaders when `couplings` couple it to some
  (the sum of their terms), as the runtime does, and otherwise holds its `heldPositions` value.

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
  public final couplings:Array<CoupledJoint>;
  /** For each robot joint no source joint drives, the couplings it sums (none when it follows nothing). */
  final followed:Array<Array<CoupledJoint>>;

  public function new(starts:NativeSpan<Int64>, durations:NativeSpan<Int64>, degrees:NativeSpan<Int>,
      coefficients:NativeSpan<Float>, jointMap:Array<Int>, heldPositions:Array<Float>,
      ?couplings:Array<CoupledJoint>) {
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
    followed = [for (_ in heldPositions) []];
    if (couplings != null) for (coupling in couplings) {
      if (coupling.follower >= heldPositions.length || coupling.leader >= heldPositions.length)
        throw "Segment arrays couple a joint the robot does not have";
      if (!driven[coupling.follower]) followed[coupling.follower].push(coupling);
    }
    for (start in 0...followed.length) {
      // Each chain of followers must reach a driven or held joint: depth first, 1 on the path, 2 done.
      var state = [for (_ in followed) 0];
      var stack = [start], cursor = [0];
      state[start] = 1;
      while (stack.length > 0) {
        var joint = stack[stack.length - 1], at = cursor[cursor.length - 1];
        if (at >= followed[joint].length) {
          state[joint] = 2;
          stack.pop();
          cursor.pop();
          continue;
        }
        cursor[cursor.length - 1] = at + 1;
        var leader = followed[joint][at].leader;
        if (state[leader] == 1) throw "Segment arrays have a cycle of couplings";
        if (state[leader] == 2) continue;
        state[leader] = 1;
        stack.push(leader);
        cursor.push(0);
      }
    }
    this.starts = starts;
    this.durations = durations;
    this.degrees = degrees;
    this.coefficients = coefficients;
    this.jointMap = jointMap.copy();
    this.heldPositions = heldPositions.copy();
    this.couplings = couplings == null ? [] : couplings.copy();
  }

  public function count():Int return starts.length();

  /** The robot joint count the segments drive. */
  public function robotJointCount():Int return heldPositions.length;

  /** Segments `[first, last)`, reading the same memory. */
  public function slice(first:Int, last:Int):SegmentArrays {
    var joints = jointMap.length * STRIDE;
    return new SegmentArrays(starts.slice(first, last - first), durations.slice(first, last - first),
      degrees.slice(first, last - first), coefficients.slice(first * joints, (last - first) * joints),
      jointMap, heldPositions, couplings);
  }

  /** Robot joint `joint`'s coefficient of `power` in segment `index`. */
  public function coefficient(index:Int, joint:Int, power:Int):Float {
    var source = jointMap.indexOf(joint);
    if (source < 0) {
      var terms = followed[joint];
      if (terms.length == 0) return power == 0 ? heldPositions[joint] : 0.0;
      var sum = 0.0;
      for (coupling in terms)
        sum += coupling.ratio * coefficient(index, coupling.leader, power) + (power == 0 ? coupling.offset : 0.0);
      return sum;
    }
    return coefficients.get((index * jointMap.length + source) * STRIDE + power);
  }

  /**
    Source joints' `values` over every robot joint: each at its robot joint, a
    coupled joint's from its leader (with the coupling's offset when the values
    are `positions`, not their derivatives), and `rest` for every other joint.
  **/
  public function robotValues(values:Array<Float>, rest:Array<Float>, positions:Bool):Array<Float> {
    if (values.length != jointMap.length || rest.length != heldPositions.length)
      throw "Segment arrays map source values onto every robot joint";
    var result = rest.copy();
    for (source in 0...jointMap.length) result[jointMap[source]] = values[source];
    function value(joint:Int):Float {
      var terms = followed[joint];
      if (terms.length == 0) return result[joint];
      var sum = 0.0;
      for (coupling in terms) sum += coupling.ratio * value(coupling.leader) + (positions ? coupling.offset : 0.0);
      return sum;
    }
    return [for (joint in 0...result.length) value(joint)];
  }

  /**
    Source joints' tolerances over every robot joint: as `robotValues`, but a
    coupled joint's is its leaders' scaled by the sizes of the ratios and summed, so it
    allows the same error, and `rest` for every other joint.
  **/
  public function robotTolerances(values:Array<Float>, rest:Array<Float>):Array<Float> {
    if (values.length != jointMap.length || rest.length != heldPositions.length)
      throw "Segment arrays map source tolerances onto every robot joint";
    var result = rest.copy();
    for (source in 0...jointMap.length) result[jointMap[source]] = values[source];
    function tolerance(joint:Int):Float {
      var terms = followed[joint];
      if (terms.length == 0) return result[joint];
      var sum = 0.0;
      for (coupling in terms) sum += Math.abs(coupling.ratio) * tolerance(coupling.leader);
      return sum;
    }
    return [for (joint in 0...result.length) tolerance(joint)];
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
