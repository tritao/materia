package processkit;

import motionkit.event.EventValue;
import motionkit.event.HoldPolicy;
import motionkit.event.PathEvent;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.robot.CompiledProgram;

/** Constant quantity per authored metre, scheduled against the validated motion's clock. */
class ProcessRateSchedule {
  /**
   * Each held rate deposits exactly the requested quantity over its timed interval.
   * Distances belong to the whole FollowPath operation; times belong to one native section.
   * Coalescing intervals bounds the event count while preserving their total quantity.
   */
  public static function section(channel:String, quantity:Float, distances:Array<Float>,
      times:Array<Float>, ?budget:Int = 240):Array<PathEvent> {
    if (!(quantity > 0.0) || !Math.isFinite(quantity) || budget < 1 ||
        distances == null || times == null || distances.length != times.length || distances.length < 2)
      throw "A process rate schedule needs quantity and a timed path section";
    for (i in 0...distances.length) {
      if (!Math.isFinite(distances[i]) || !Math.isFinite(times[i]) || distances[i] < 0.0 || times[i] < 0.0)
        throw "A process rate schedule needs finite nonnegative distances and times";
      if (i > 0 && (distances[i] <= distances[i - 1] || times[i] <= times[i - 1]))
        throw "A process rate schedule needs increasing distances and times";
    }
    var events:Array<PathEvent> = [];
    var stride = Std.int(Math.ceil((distances.length - 1) / budget));
    var first = 0;
    while (first < distances.length - 1) {
      var last = Std.int(Math.min(first + stride, distances.length - 1));
      var rate = quantity * (distances[last] - distances[first]) / (times[last] - times[first]);
      events.push(new PathEvent(distances[first], channel, EventValue.Analog(rate), 0.0,
        HoldPolicy.RestoreOnResume));
      first = last;
    }
    return events;
  }

  /** Schedule a continuously active process; other channels and the engagement stay intact. */
  public static function apply(program:MotionProgram, compiled:CompiledProgram, op:Int,
      channel:String, quantity:Float, endRate:Float):MotionProgram {
    var scheduled:Array<PathEvent> = [];
    for (block in compiled.blocks) for (i in 0...block.plans.length)
      if (block.opIndices[i] == op) scheduled = scheduled.concat(section(channel, quantity,
        block.pathDistances[i], block.pathTimes[i]));
    if (scheduled.length == 0) throw "The process path has no validated timing";
    var ops = program.ops.copy();
    switch ops[op] {
      case FollowPath(path, frame, feed, events):
        for (event in events) {
          if (event.channel != channel) scheduled.push(event);
          else if (event.distance > 0.0 && event.distance < path.length())
            throw "A continuous process rate schedule cannot replace internal process transitions";
        }
        scheduled.push(new PathEvent(path.length(), channel, EventValue.Analog(endRate)));
        scheduled.sort(function(a, b) return a.distance < b.distance ? -1 : a.distance > b.distance ? 1 : 0);
        ops[op] = MotionOp.FollowPath(path, frame, feed, scheduled);
      case _: throw "A process rate schedule requires a FollowPath operation";
    }
    return new MotionProgram(ops);
  }
}
