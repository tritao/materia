package motionkit.planner;

import haxe.Int64;
import motionkit.trajectory.Trajectory;

private typedef TimedRange = {
  var timed:TimedPath;
  var begin:Int64;
  var end:Int64;
  var firstDistance:Float;
  var lastDistance:Float;
  var firstStage:Int;
}

/** One owner's retained clock laws. Streaming keeps only filter/event context;
 * a whole-path request drains the same laws and joins their polynomial output.
 */
class TimedPathSeries {
  final ranges:Array<TimedRange> = [];
  var end = Int64.ofInt(0);
  var disposed = false;

  public function new() {}
  public function durationTicks():Int64 return end;
  public function firstDistance():Float { live(); return ranges[0].firstDistance; }
  public function lastDistance():Float { live(); return ranges[ranges.length-1].lastDistance; }

  /** Takes ownership; the caller may borrow the result until it retires. */
  public function append(timed:TimedPath, firstDistance:Float, lastDistance:Float, firstStage:Int):Void {
    if (disposed || timed == null || !Math.isFinite(firstDistance) || !Math.isFinite(lastDistance) ||
        lastDistance <= firstDistance || firstStage < 0) throw "Invalid retained timing range";
    var next = Int64.add(end,timed.trajectory.durationNanoseconds());
    if (Int64.compare(next,end) <= 0) throw "Retained timing duration is empty or overflows";
    ranges.push({timed:timed,begin:end,end:next,firstDistance:firstDistance,lastDistance:lastDistance,firstStage:firstStage});
    end = next;
  }

  public function distanceToTime(distance:Float):Float {
    live();
    var low=0, high=ranges.length-1;
    while (low < high) { var middle=(low+high)>>1;
      if (distance <= ranges[middle].lastDistance) high=middle; else low=middle+1; }
    var range=ranges[low];
    return Int64.toFloat(range.begin)*1e-9+range.timed.distanceToTime(distance);
  }

  /** One bulk inverse call per retained law, preserving arbitrary caller order. */
  public function timesToDistances(seconds:Array<Float>):Array<Float> {
    live();
    var groups:Array<Array<Float>> = [for (_ in ranges) []];
    var indices:Array<Array<Int>> = [for (_ in ranges) []];
    for (i in 0...seconds.length) {
      if (!Math.isFinite(seconds[i])) throw "Timing inverse needs finite clock values";
      var low=0, high=ranges.length-1;
      while (low < high) { var middle=(low+high)>>1;
        if (seconds[i] <= Int64.toFloat(ranges[middle].end)*1e-9) high=middle; else low=middle+1; }
      groups[low].push(seconds[i]-Int64.toFloat(ranges[low].begin)*1e-9); indices[low].push(i);
    }
    var distances=[for (_ in seconds) 0.0];
    for (range in 0...ranges.length) if (groups[range].length > 0) {
      var mapped=ranges[range].timed.timesToDistances(groups[range]);
      for (i in 0...mapped.length) distances[indices[range][i]]=mapped[i];
    }
    return distances;
  }

  public function releaseBefore(tick:Int64):Void {
    live();
    while (ranges.length > 1 && Int64.compare(ranges[0].end,tick) < 0) {
      var range=ranges.shift();range.timed.releaseDistanceMap();range.timed.trajectory.dispose();
    }
  }

  /** Transfers the joined output; its distance-law lifetime becomes the result's. */
  public function finish():TimedPath {
    live();
    if (ranges.length == 1) {
      var result=ranges.shift().timed;disposed=true;return result;
    }
    var joined=Trajectory.concatenate([for (range in ranges) range.timed.trajectory]);
    try {
      var result=new TimedPath(joined,distanceToTime,[],dispose,timesToDistances,() -> {
        var bindings:Array<BindingConstraint> = [];
        for (range in ranges) for (binding in range.timed.bindingConstraints)
          bindings.push(new BindingConstraint(range.firstStage+binding.spanIndex,binding.jointIndex,binding.kind,binding.limit));
        return bindings;
      });
      for (range in ranges) range.timed.trajectory.dispose();
      return result;
    } catch (error:Dynamic) { joined.dispose();throw error; }
  }

  public function dispose():Void {
    if (disposed) return;
    disposed=true;
    for (range in ranges) { range.timed.releaseDistanceMap();range.timed.trajectory.dispose(); }
    ranges.splice(0,ranges.length);
  }
  function live():Void { if (disposed || ranges.length == 0) throw "Timing series has no retained law"; }
}
