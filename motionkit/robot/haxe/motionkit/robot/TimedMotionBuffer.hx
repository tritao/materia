package motionkit.robot;

import haxe.Int64;
import motionkit.robot.TimedMotionTransforms.MotionFilterSettings;
import motionkit.trajectory.ShaperImpulses;
import motionkit.trajectory.Trajectory;

private class BufferRevision { public function new() {} }

/** Independently owned output and speculative retained context. */
@:allow(motionkit.robot.TimedMotionBuffer)
class TimedMotionChunk {
  public final trajectory:Trajectory;
  public final startNs:Int64;
  public final endNs:Int64;
  public final finalChunk:Bool;
  public final shaped:Bool;
  final revision:BufferRevision;
  final retainedFrom:Int64;
  var nextBuffer:Null<Trajectory>;
  var disposed = false;

  function new(trajectory:Trajectory, start:Int64, end:Int64, finalChunk:Bool, shaped:Bool,
      revision:BufferRevision, retainedFrom:Int64, nextBuffer:Null<Trajectory>) {
    this.trajectory = trajectory; startNs = start; endNs = end;
    this.finalChunk = finalChunk; this.shaped = shaped; this.revision = revision;
    this.retainedFrom = retainedFrom; this.nextBuffer = nextBuffer;
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    trajectory.dispose();
    if (nextBuffer != null) nextBuffer.dispose();
    nextBuffer = null;
  }
}

/** Worker-owned continuous filter context. Accepting raw timing here never
 * publishes execution. Retire a preview only after the caller certifies its
 * transformed motion, joining state and events. No accepted output is retimed.
 */
class TimedMotionBuffer {
  final pressureAdvance:Float;
  final pressureWindow:Float;
  final drivenJoint:Null<Int>;
  final impulses:Null<Array<ShaperImpulses>>;
  final projection:Null<CouplingProjection>;
  final pastNs:Int64;
  final futureNs:Int64;
  final taperNs:Int64;
  final tailNs:Int64;
  final maximumNs:Null<Int64>;
  public final lagSeconds:Float;
  var buffer:Null<Trajectory>;
  var origin:Int64 = Int64.ofInt(0);
  var end:Int64 = Int64.ofInt(0);
  var published:Int64 = Int64.ofInt(0);
  var finalInput = false;
  var disposed = false;
  var done = false;
  var revision:Null<BufferRevision>;

  public function new(settings:MotionFilterSettings, projection:Null<CouplingProjection>, drivenJoint:Null<Int>,
      ?maximumChunkSeconds:Float, eventLeadSeconds:Float = 0.0) {
    if (settings == null || !Math.isFinite(settings.pressureAdvance) || settings.pressureAdvance < 0 ||
        !Math.isFinite(settings.pressureAdvanceWindow) || settings.pressureAdvanceWindow <= 0 ||
        !Math.isFinite(settings.shapingDelay) || settings.shapingDelay < 0 ||
        !Math.isFinite(eventLeadSeconds) || eventLeadSeconds < 0)
      throw "Motion filter settings must be finite with a positive pressure window";
    this.projection = projection; this.drivenJoint = drivenJoint;
    pressureAdvance = settings.pressureAdvance; pressureWindow = settings.pressureAdvanceWindow;
    var authored = settings.shaping;
    impulses = authored == null ? null : [for (joint in authored) new ShaperImpulses(joint.delays,joint.amplitudes)];
    lagSeconds = impulses == null ? 0.0 : settings.shapingDelay;
    var tail = 0.0;
    if (impulses != null) for (joint in impulses) tail = Math.max(tail,joint.tail());
    tailNs = Trajectory.nanoseconds(tail);
    var half = Trajectory.nanoseconds(pressureWindow*0.5);
    if (Int64.compare(half,Int64.ofInt(1)) < 0) half = Int64.ofInt(1);
    taperNs = drivenJoint != null && pressureAdvance > 0 ? Int64.add(half,half) : Int64.ofInt(0);
    // A yet-unknown true end can introduce a full-window pressure taper.
    // Half-window averaging alone is insufficient to prove a stable prefix.
    pastNs = Int64.add(tailNs,taperNs);
    // Event lead and shaper lag are quantized separately by the event scheduler.
    // Rounding their Float difference can undercount the required future by a tick.
    var eventFuture = Int64.sub(Trajectory.nanoseconds(eventLeadSeconds),Trajectory.nanoseconds(lagSeconds));
    if (Int64.compare(eventFuture,Int64.ofInt(0)) < 0) eventFuture = Int64.ofInt(0);
    futureNs = Int64.compare(eventFuture,taperNs) > 0 ? eventFuture : taperNs;
    maximumNs = maximumChunkSeconds == null ? null : Trajectory.nanoseconds(maximumChunkSeconds);
    if (maximumNs != null && Int64.compare(maximumNs,Int64.ofInt(0)) <= 0)
      throw "A publication budget must contain an integer clock tick";
  }

  public function publishedThrough():Int64 return published;
  public function retainedFrom():Int64 return origin;
  public function finished():Bool return done;

  /** Copies raw polynomials; their timing/geometry certificate belongs to the caller. */
  public function append(raw:Trajectory, finalRange:Bool):Void {
    ensureLive();
    if (finalInput) throw "Cannot append after the final filter input";
    if (raw == null) throw "Motion filter input is required";
    var duration = raw.durationNanoseconds(), nextEnd = Int64.add(end,duration);
    if (Int64.compare(duration,Int64.ofInt(0)) <= 0 || Int64.compare(nextEnd,end) <= 0 ||
        Int64.compare(Int64.add(nextEnd,tailNs),nextEnd) < 0)
      throw "Motion filter input duration is empty or overflows the clock";
    var previous = buffer;
    var copied = previous == null ? raw.sliceTicks(Int64.ofInt(0),duration) : Trajectory.concatenate([previous,raw]);
    buffer = copied; end = nextEnd; finalInput = finalRange; revision = null;
    if (previous != null) previous.dispose();
  }

  public function preview():Null<TimedMotionChunk> {
    ensureLive();
    revision = null;
    var source = buffer;
    if (source == null || done) return null;
    var available = finalInput ? Int64.add(end,tailNs) : Int64.sub(end,futureNs);
    if (Int64.compare(available,published) <= 0) return null;
    var limit = maximumNs, wanted = available;
    if (limit != null && Int64.compare(limit,Int64.sub(available,published)) < 0) wanted = Int64.add(published,limit);
    var current = new BufferRevision();
    var owned:Array<Trajectory> = [];
    var output:Null<Trajectory> = null, retained:Null<Trajectory> = null;
    try {
      var filtered = source;
      if (drivenJoint != null && pressureAdvance > 0) {
        filtered = filtered.advanceJointContext(drivenJoint,pressureAdvance,pressureWindow,origin,finalInput);
        owned.push(filtered);
      }
      if (projection != null) { filtered = projection.trajectory(filtered); owned.push(filtered); }
      if (impulses != null) {
        filtered = filtered.shape([for (joint in impulses) joint.delays],[for (joint in impulses) joint.amplitudes]);
        owned.push(filtered);
        if (projection != null) { filtered = projection.trajectory(filtered); owned.push(filtered); }
      }
      var complete = finalInput && Int64.compare(wanted,Int64.add(end,tailNs)) == 0;
      var cut = Int64.sub(wanted,origin);
      if (!complete) {
        var interior = filtered.interiorTimeAtOrBefore(cut);
        if (interior == null || Int64.compare(interior,Int64.sub(published,origin)) <= 0) {
          for (part in owned) part.dispose();
          return null;
        }
        cut = interior; wanted = Int64.add(origin,cut);
      }
      output = filtered.sliceTicks(Int64.sub(published,origin),cut);
      var keep = Int64.sub(wanted,pastNs);
      if (Int64.compare(keep,origin) < 0 || Int64.compare(keep,taperNs) < 0) keep = origin;
      if (complete) keep = end;
      else retained = source.sliceTicks(Int64.sub(keep,origin),source.durationNanoseconds());
      var result = new TimedMotionChunk(output,published,wanted,complete,impulses != null,current,keep,retained);
      revision = current;
      for (part in owned) part.dispose();
      return result;
    } catch (error:Dynamic) {
      for (part in owned) part.dispose();
      if (output != null) output.dispose();
      if (retained != null) retained.dispose();
      throw error;
    }
  }

  /** The caller has certified output and owns its execution plan independently. */
  public function retireCertified(chunk:TimedMotionChunk):Void {
    ensureLive();
    if (chunk == null || chunk.disposed || revision == null || chunk.revision != revision ||
        Int64.compare(chunk.startNs,published) != 0) throw "Motion filter preview is stale or foreign";
    var previous = buffer;
    buffer = chunk.nextBuffer; chunk.nextBuffer = null;
    origin = chunk.retainedFrom; published = chunk.endNs; done = chunk.finalChunk;
    revision = null;
    if (previous != null) previous.dispose();
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true; revision = null;
    if (buffer != null) buffer.dispose();
    buffer = null;
  }
  function ensureLive():Void { if (disposed) throw "Motion filter buffer has been disposed"; }
}
