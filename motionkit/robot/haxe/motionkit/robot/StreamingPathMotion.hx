package motionkit.robot;

import haxe.Int64;
import motionkit.event.PathEvent;
import motionkit.event.TimedEvent;
import motionkit.path.PosePath;
import motionkit.planner.PathTimingStream;
import motionkit.planner.TimedPath;
import motionkit.planner.TimedPathSeries;
import motionkit.robot.ProgramCompiler.PendingMotion;
import motionkit.robot.ProgramCompiler.PreparedProgramPath;
import motionkit.robot.TimedMotionBuffer.TimedMotionChunk;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.Trajectory;

/** One worker's unpublished raw timing and continuous filter context. A step either
 * prepares one timing window or offers one independently certified execution chunk.
 */
class StreamingPathMotion {
  final timing:PathTimingStream;
  final buffer:TimedMotionBuffer;
  final path:PosePath;
  final prepared:PreparedProgramPath;
  final index:Int;
  final feed:Float;
  final affine:Bool;
  final offset:Float;
  final authored:Array<PathEvent>;
  final mergeBudget:Null<PathMergeBudget>;
  final assigned:Array<Bool>;
  final events:Array<TimedEvent> = [];
  final laws = new TimedPathSeries();
  var end:Int64 = Int64.ofInt(0);
  var sourceFinished = false;
  var chunk:Null<TimedMotionChunk>;
  var startQ:Array<Float>;
  var startV:Array<Float>;
  var startA:Array<Float>;
  var disposed = false;
  public var offeredFinal(default,null) = false;

  public function new(timing:PathTimingStream, buffer:TimedMotionBuffer, path:PosePath,
      prepared:PreparedProgramPath, startQ:Array<Float>, index:Int, feed:Float,
      offset:Float, authored:Array<PathEvent>, affine:Bool, ?mergeBudget:PathMergeBudget) {
    this.timing = timing; this.buffer = buffer; this.path = path; this.prepared = prepared;
    this.startQ = startQ.copy(); startV = [for (_ in startQ) 0.0]; startA = startV.copy();
    this.index = index; this.feed = feed; this.offset = offset; this.affine = affine;
    this.mergeBudget=mergeBudget;
    this.authored = authored.copy(); assigned = [for (_ in authored) false];
  }

  /** Null means this bounded step has not produced publishable execution yet. */
  public function step():Null<PendingMotion> {
    if (disposed || chunk != null) throw "Streaming motion cannot advance with an outstanding chunk";
    var profiling=Sys.getEnv("PROCESS_PATH_PROFILE")=="1";
    var began=profiling ? Sys.cpuTime() : 0.0;
    var mark=function(stage:String) {
      if (profiling) { var now=Sys.cpuTime();Sys.println("PROCESS_PATH_STAGE "+haxe.Json.stringify({stage:stage,cpuSeconds:now-began}));began=now; }
    };
    var candidate = buffer.preview();
    mark("stream_filter_preview");
    if (candidate != null) { var offered=offer(candidate);mark("stream_offer");return offered; }
    if (sourceFinished) throw "Final streaming input produced no execution chunk";
    switch timing.step() {
      case Waiting: return null;
      case Done: throw "Timing stream ended before its final result";
      case Ready(preview):
        var raw:Null<TimedPath> = null;
        try {
          raw = preview.commitTiming();
          mark("stream_timing_commit");
          var next = Int64.add(end,raw.trajectory.durationNanoseconds());
          var timed = raw;
          laws.append(timed,preview.startDistance,preview.endDistance,preview.firstStage);raw = null;
          sourceFinished = preview.windowIndex+1 == preview.totalWindows;
          // Final source readiness owns the authored endpoint event even if
          // integer-clock integration lands a few ULPs before that endpoint.
          // The unchanged native inverse law handles final rest quantization.
          for (i in 0...authored.length) if (!assigned[i] && (sourceFinished || authored[i].distance <= preview.endDistance)) {
            var event = authored[i];
            var tick = Int64.add(end,Trajectory.nanoseconds(timed.distanceToTime(event.distance)));
            tick = Int64.sub(tick,Trajectory.nanoseconds(event.leadSeconds));
            if (Int64.compare(tick,Int64.ofInt(0)) < 0) tick = Int64.ofInt(0);
            tick = Int64.add(tick,Trajectory.nanoseconds(buffer.lagSeconds));
            if (Int64.compare(tick,buffer.publishedThrough()) < 0) throw "Streaming event arrived after publication";
            events.push(new TimedEvent(tick,event.channel,event.value,event.holdPolicy));
            assigned[i] = true;
          }
          if (sourceFinished) for (done in assigned) if (!done) throw "Streaming event lies beyond the timed path";
          mark("stream_laws_events");
          buffer.append(timed.trajectory,sourceFinished);
          mark("stream_buffer_append");
          timed.trajectory.dispose(); end = next;
          preview.dispose();
          return null;
        } catch (error:Dynamic) {
          preview.dispose();
          if (raw != null) { raw.releaseDistanceMap(); raw.trajectory.dispose(); }
          throw error;
        }
    }
  }

  function offer(candidate:TimedMotionChunk):PendingMotion {
    chunk = candidate;
    var begin = Int64.toFloat(candidate.startNs)*1e-9, finish = Int64.toFloat(candidate.endNs)*1e-9;
    var duration = candidate.trajectory.durationSeconds(), lag = buffer.lagSeconds;
    var distances:Array<Float> = [], times:Array<Float> = [];
    // Explicit geometry samples and midpoints keep their exact distance-law times.
    for (i in 0...prepared.distances.length*2-1) {
      var at = i >> 1;
      var distance = i%2 == 0 ? prepared.distances[at] : (prepared.distances[at]+prepared.distances[at+1])*0.5;
      if (distance < laws.firstDistance() || distance > laws.lastDistance() && !sourceFinished) continue;
      var time = laws.distanceToTime(distance)+lag;
      if (time >= begin && time <= finish) { distances.push(distance); times.push(time-begin); }
    }
    var steps = affine ? 1 : Std.int(Math.max(1,Math.ceil(duration/0.001)));
    var clocks = [for (i in 0...steps+1) Math.min(Int64.toFloat(end)*1e-9,Math.max(0.0,begin+duration*i/steps-lag))];
    var sampled = [for (distance in laws.timesToDistances(clocks)) Math.max(0.0,Math.min(path.length(),distance))];
    if (candidate.finalChunk) sampled[sampled.length-1] = path.length();
    var selected:Array<TimedEvent> = [];
    for (event in events) if (Int64.compare(event.timeNs,candidate.startNs) >= 0 &&
        (Int64.compare(event.timeNs,candidate.endNs) < 0 || candidate.finalChunk && Int64.compare(event.timeNs,candidate.endNs) == 0))
      selected.push(new TimedEvent(Int64.sub(event.timeNs,candidate.startNs),event.channel,event.value,event.holdPolicy));
    var state = candidate.trajectory.evaluate(duration);
    var pending = new PendingMotion(index,startQ,state.positions,candidate.trajectory,selected,path,
      distances.copy(),times.copy(),null,0.0,sampled,distances,times);
    pending.startVelocity = startV.copy(); pending.startAcceleration = startA.copy();
    pending.continues = !candidate.finalChunk;
    pending.filtered = true; pending.filteredShaped = candidate.shaped;
    pending.distanceOffset = offset; pending.endDistance = sampled[sampled.length-1];
    pending.geometryMergeBudget=mergeBudget;
    pending.geometryCurve = prepared.jointPath; pending.geometryRefined = prepared.refined;
    pending.feed = feed;
    pending.accept = accept; pending.abandon = dispose;
    offeredFinal = candidate.finalChunk;
    return pending;
  }

  function accept(plan:ExecutionPlan):Void {
    var current = chunk;
    if (current == null) throw "Streaming chunk has already been retired";
    var state = plan.evaluate(plan.durationSeconds);
    buffer.retireCertified(current);
    startQ = state.positions; startV = state.velocities; startA = state.accelerations;
    current.dispose(); chunk = null;
    var keep = buffer.retainedFrom();
    laws.releaseBefore(keep);
    if (offeredFinal) dispose();
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true; timing.dispose(); buffer.dispose();
    if (chunk != null) chunk.dispose(); chunk = null;
    laws.dispose();
  }
}
