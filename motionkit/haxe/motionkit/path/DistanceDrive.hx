package motionkit.path;

/** A stretch of a path along which the driven joint turns at a nominal rate per metre of path. */
typedef DriveInterval = {
  var start:Float;
  var end:Float;
  var rate:Float;
};

/**
 * A joint driven by distance along a path rather than by the pose: an extruder's gear turning so much per metre of
 * bead. Between nominal intervals the rate is not a step but a ramp, so the joint's velocity stays continuous and its
 * acceleration finite, and the ramps are shaped so that the filament fed over every run equals the nominal feed
 * exactly:
 *  - a run that begins at rest (after a travel, or at the start of its plan) climbs 0 -> 1.5 rate -> rate over two ramp
 *    lengths (area 2 rate ramp, as the step it replaces);
 *  - a run that ends does the same backwards;
 *  - a change of rate in the middle of a run is a linear ramp centred on the change.
 * The profile is piecewise linear in the rate, so the joint's position is piecewise quadratic between the stations
 * and a quintic Hermite fit through the stations is exact. Timing sees the joint's speed and acceleration limits
 * like any other joint's.
 */
class DistanceDrive {
  /** The joint index driven. */
  public final joint:Int;
  /** The nominal intervals, in order, rebased so that the path starts at 0. */
  public var intervals(get,never):Array<DriveInterval>;
  final nominalIntervals:Array<DriveInterval>;
  public final pathLength:Float;
  /** The ramp length the profile aims for, metres. */
  public final ramp:Float;
  final distances:Array<Float> = [];
  final rates:Array<Float> = [];
  /** The joint's position change from the start at each station. */
  final positions:Array<Float> = [];

  public function new(joint:Int, intervals:Array<DriveInterval>, pathLength:Float, ramp:Float) {
    if (joint < 0 || !Math.isFinite(pathLength) || pathLength <= 0.0 || !Math.isFinite(ramp) || ramp <= 0.0)
      throw "A distance drive needs a joint, a positive path length and a positive ramp";
    this.joint = joint; this.pathLength = pathLength; this.ramp = ramp;
    // Clean the intervals: ordered, inside the path, nonzero, with equal neighbours merged.
    var cleaned:Array<DriveInterval> = [];
    var previousEnd = 0.0;
    for (interval in intervals) {
      if (!Math.isFinite(interval.start) || !Math.isFinite(interval.end) || !Math.isFinite(interval.rate) ||
          interval.start < previousEnd - 1e-12 || interval.end > pathLength + 1e-9 || interval.end < interval.start)
        throw "Distance-drive intervals must be ordered, finite and inside the path";
      previousEnd = interval.end;
      var start = Math.max(0.0, interval.start), end = Math.min(pathLength, interval.end);
      if (end - start <= 1e-12 || interval.rate == 0.0) continue;
      var last = cleaned.length == 0 ? null : cleaned[cleaned.length - 1];
      if (last != null && Math.abs(last.end - start) <= 1e-9 && last.rate == interval.rate) last.end = end;
      else cleaned.push({start: start, end: end, rate: interval.rate});
    }
    this.nominalIntervals = cleaned;
    build();
  }

  function get_intervals():Array<DriveInterval>
    return [for (interval in nominalIntervals) {start:interval.start,end:interval.end,rate:interval.rate}];

  /** The drive restricted to [from, to] of this path and rebased to start at 0, re-shaped so it starts and ends at rest; null if it drives nothing there. */
  public function slice(from:Float, to:Float):Null<DistanceDrive> {
    var kept:Array<DriveInterval> = [];
    var low=0,high=nominalIntervals.length;
    while (low<high) {
      var middle=low+((high-low)>>1);
      if (nominalIntervals[middle].end<=from) low=middle+1; else high=middle;
    }
    var index=low;
    while (index<nominalIntervals.length && nominalIntervals[index].start<to) {
      var interval=nominalIntervals[index++];
      var start = Math.max(interval.start, from), end = Math.min(interval.end, to);
      if (end - start > 1e-12) kept.push({start: start - from, end: end - from, rate: interval.rate});
    }
    return kept.length == 0 ? null : new DistanceDrive(joint, kept, to - from, ramp);
  }

  /** The distances at which the rate changes slope: a path must be sampled at them. */
  public function stations():Array<Float> return distances.copy();
  /** The rate is piecewise affine, so its absolute maximum occurs at a station, including startup overshoot. */
  public function maximumRate():Float {
    var bound=0.0;for(rate in rates)bound=Math.max(bound,Math.abs(rate));return bound;
  }

  /** The joint's position change from the path's start, at `distance`. */
  public function offsetAt(distance:Float):Float {
    var at = locate(distance);
    if (at < 0) return 0.0;
    if (at >= distances.length - 1) return positions[positions.length - 1];
    var span = distances[at + 1] - distances[at], t = distance - distances[at];
    var slope = span <= 0.0 ? 0.0 : (rates[at + 1] - rates[at]) / span;
    return positions[at] + rates[at] * t + 0.5 * slope * t * t;
  }

  /** d(position)/d(distance); `leaving` picks the side at a station (the rate is continuous, so it matters not). */
  public function rateAt(distance:Float, ?leaving:Bool = true):Float {
    var at = locate(distance);
    if (at < 0 || at >= distances.length - 1) return at < 0 ? 0.0 : rates[rates.length - 1];
    var span = distances[at + 1] - distances[at];
    return span <= 0.0 ? rates[at] : rates[at] + (rates[at + 1] - rates[at]) * (distance - distances[at]) / span;
  }

  /** d(rate)/d(distance): the slope of the ramp leaving (`leaving`) or arriving at `distance`. */
  public function curvatureAt(distance:Float, leaving:Bool):Float {
    var at = locate(distance);
    if (at < 0) return 0.0;
    // A station belongs to the span after it when leaving and the span before it when arriving.
    if (!leaving && at > 0 && Math.abs(distance - distances[at]) <= 1e-12) at--;
    if (at >= distances.length - 1) return 0.0;
    var span = distances[at + 1] - distances[at];
    return span <= 0.0 ? 0.0 : (rates[at + 1] - rates[at]) / span;
  }

  /** The nominal position change over the whole path: what the rate profile integrates to. */
  public function total():Float return positions[positions.length - 1];

  /** The span containing `distance` (the span after a station at a station), or -1 before the first. */
  function locate(distance:Float):Int {
    if (distances.length == 0 || distance < distances[0] - 1e-12) return -1;
    var low = 0, high = distances.length - 1;
    while (low < high) {
      var middle = (low + high + 1) >> 1;
      if (distances[middle] <= distance + 1e-12) low = middle; else high = middle - 1;
    }
    return low;
  }

  function build():Void {
    var points:Array<{s:Float, rate:Float}> = [{s: 0.0, rate: 0.0}];
    function put(s:Float, rate:Float):Void {
      var last = points[points.length - 1];
      if (s < last.s - 1e-12) throw "Distance-drive ramps overlap";
      if (Math.abs(s - last.s) <= 1e-12) last.rate = rate; else points.push({s: s, rate: rate});
    }
    // Runs: consecutive intervals that touch.
    var i = 0;
    while (i < nominalIntervals.length) {
      var j = i;
      while (j + 1 < nominalIntervals.length && Math.abs(nominalIntervals[j + 1].start - nominalIntervals[j].end) <= 1e-9) j++;
      var begin = nominalIntervals[i].start, end = nominalIntervals[j].end;
      // The ramp a run can carry: its start-up and wind-down take two ramps each, and a change inside it one.
      var shortest = end - begin;
      for (k in i...(j + 1)) shortest = Math.min(shortest, nominalIntervals[k].end - nominalIntervals[k].start);
      var ell = Math.min(ramp, Math.min((end - begin) / 4.0, shortest / 2.5));
      put(begin, 0.0);
      put(begin + ell, 1.5 * nominalIntervals[i].rate);
      put(begin + 2.0 * ell, nominalIntervals[i].rate);
      for (k in i...j) {
        var at = nominalIntervals[k].end;
        put(at - ell / 2.0, nominalIntervals[k].rate);
        put(at + ell / 2.0, nominalIntervals[k + 1].rate);
      }
      var rate = nominalIntervals[j].rate;
      put(end - 2.0 * ell, rate);
      put(end - ell, 1.5 * rate);
      put(end, 0.0);
      i = j + 1;
    }
    put(pathLength, 0.0);
    positions.push(0.0);
    for (k in 0...points.length) {
      distances.push(points[k].s); rates.push(points[k].rate);
      if (k > 0) positions.push(positions[k - 1] + 0.5 * (rates[k - 1] + rates[k]) * (distances[k] - distances[k - 1]));
    }
  }
}
