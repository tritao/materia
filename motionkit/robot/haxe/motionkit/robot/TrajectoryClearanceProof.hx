package motionkit.robot;

import motionkit.trajectory.Trajectory;
import robotkit.manipulation.ClearanceChange;
import robotkit.manipulation.ClearanceScene;
import robotkit.manipulation.ClearanceViolation;
import robotkit.manipulation.ClearanceWorld;

/** A change to the cell at a time of one compiled motion (seconds from its start; COLLISION.md CL-D9). */
class ClearanceEvent {
  /** The program op whose motion it happens in. */
  public final op:Int;
  public final timeSeconds:Float;
  public final change:ClearanceChange;

  public function new(op:Int, timeSeconds:Float, change:ClearanceChange) {
    if (!Math.isFinite(timeSeconds) || timeSeconds < 0) throw "A clearance event needs a time from the motion's start";
    this.op = op;
    this.timeSeconds = timeSeconds;
    this.change = change;
  }
}

/**
 * A conservative clearance check of a timed trajectory (COLLISION.md CL-D5,
 * CL4b), for joint moves and generated entry, exit and hold moves, whose
 * timed joint path need not be straight.
 *
 * Over a time interval of half-length h around t, joint j moves at most
 * `velocity[j] * h` from q(t), since the validated plan keeps every joint
 * within its velocity limit; the world's displacement bounds turn those
 * joint ranges into per-body inflation. The interval is clear when q(t) is
 * clear with the inflation; otherwise it is bisected, down to `depthLimit`.
 * At each midpoint the configuration is also checked as it is: a violation
 * there is a real one, found between the sampled check's samples.
 *
 * Changes (`ClearanceEvent`) are applied in time order through the world's
 * `ClearanceScene`, and no interval crosses one.
 *
 * Contact: with a contact policy over configurations (`contactAt`) the
 * bound needs the whole interval to be in the contact zone, which only a
 * neighbourhood policy (`neighbourhood`, given the TCP's displacement bound)
 * can say; without one the bound uses the stricter margins, and the
 * midpoint check uses `contactAt`.
 */
class TrajectoryClearanceProof {
  /** True when the bound closed every interval. */
  public var closed(default, null):Bool = true;
  /** A real violation, found at `failureTime`. */
  public var failure(default, null):Null<ClearanceViolation> = null;
  public var failureTime(default, null):Float = 0.0;
  /** The first interval the bound could not close (bisection at its depth limit). */
  public var openStart(default, null):Float = 0.0;
  public var openEnd(default, null):Float = 0.0;
  public var queries(default, null):Int = 0;
  /** The closest pair at a few times along the motion (informational), and when. */
  public var closest(default, null):Null<ClearanceViolation> = null;
  public var closestTime(default, null):Float = 0.0;

  final world:ClearanceWorld;
  final trajectory:Trajectory;
  final velocity:Array<Float>;
  final contact:Bool;
  final contactAt:Null<Array<Float>->Bool>;
  final neighbourhood:Null<(Array<Float>, Float)->Bool>;
  final depthLimit:Int;

  function new(world:ClearanceWorld, trajectory:Trajectory, velocity:Array<Float>, contact:Bool,
      contactAt:Null<Array<Float>->Bool>, neighbourhood:Null<(Array<Float>, Float)->Bool>, depthLimit:Int) {
    this.world = world;
    this.trajectory = trajectory;
    this.velocity = velocity;
    this.contact = contact;
    this.contactAt = contactAt;
    this.neighbourhood = neighbourhood;
    this.depthLimit = depthLimit;
  }

  /**
   * Checks `trajectory` against `world`. `velocity` bounds each joint's speed
   * over the whole motion; `events` (this motion's, in any order) need a
   * world that is a `ClearanceScene`; `closestSamples` closest-pair queries
   * are spread over the motion.
   */
  public static function run(world:ClearanceWorld, trajectory:Trajectory, velocity:Array<Float>, contact:Bool,
      ?contactAt:Array<Float>->Bool, ?neighbourhood:(Array<Float>, Float)->Bool, ?events:Array<ClearanceEvent>,
      depthLimit:Int = 16, closestSamples:Int = 8):TrajectoryClearanceProof {
    if (world == null || trajectory == null || velocity == null || depthLimit < 0)
      throw "A trajectory clearance proof needs a world, a motion, joint speed bounds and a depth limit";
    var proof = new TrajectoryClearanceProof(world, trajectory, velocity, contact, contactAt, neighbourhood, depthLimit);
    var changes = events == null ? [] : events.copy();
    changes.sort((x, y) -> x.timeSeconds < y.timeSeconds ? -1 : x.timeSeconds > y.timeSeconds ? 1 : 0);
    var scene:Null<ClearanceScene> = null;
    if (changes.length > 0) {
      if (!Std.isOfType(world, ClearanceScene)) throw "Clearance events need a world that replays changes (a ClearanceScene)";
      scene = cast world;
    }
    var duration = trajectory.durationSeconds();
    var bounds = [0.0];
    for (event in changes) if (event.timeSeconds > bounds[bounds.length - 1] && event.timeSeconds < duration)
      bounds.push(event.timeSeconds);
    bounds.push(duration);
    var next = 0;
    for (k in 0...bounds.length - 1) {
      var start = bounds[k], end = bounds[k + 1];
      // Changes at or before this interval's start happen first, with the arm where it then is.
      while (next < changes.length && changes[next].timeSeconds <= start + 1e-12) {
        var at = trajectory.evaluate(Math.min(changes[next].timeSeconds, duration)).positions;
        if (scene != null) scene.apply(changes[next].change, at);
        next++;
      }
      if (end > start) {
        proof.piece(start, end, 0);
        if (proof.failure != null) break;
        proof.sampleClosest(start, end, Std.int(Math.max(1, Math.round(closestSamples * (end - start) / Math.max(duration, 1e-12)))));
      } else {
        proof.check(start);
        if (proof.failure != null) break;
      }
    }
    // Changes at the very end still happen, so the scene carries on to the next motion.
    while (next < changes.length) {
      if (scene != null) scene.apply(changes[next].change, trajectory.evaluate(duration).positions);
      next++;
    }
    return proof;
  }

  function contactIn(q:Array<Float>):Bool return contactAt == null ? contact : contactAt(q);

  /** The configuration at `t` as it is; records a real violation. */
  function check(t:Float):Void {
    var q = trajectory.evaluate(t).positions;
    queries++;
    var found = world.violation(q, contactIn(q));
    if (found != null && failure == null) {
      failure = found;
      failureTime = t;
    }
  }

  function piece(t0:Float, t1:Float, depth:Int):Void {
    if (failure != null) return;
    var t = 0.5 * (t0 + t1), half = 0.5 * (t1 - t0);
    var q = trajectory.evaluate(t).positions;
    var errors:Array<Float> = [];
    var bounded = true;
    for (j in 0...q.length) {
      var speed = j < velocity.length ? velocity[j] : Math.POSITIVE_INFINITY;
      if (!Math.isFinite(speed) || speed < 0) bounded = false;
      errors.push(bounded ? speed * half * (1 + 1e-9) + 1e-12 : 0.0);
    }
    if (bounded) {
      var bound = contact;
      if (contactAt != null) {
        var guard = neighbourhood;
        bound = guard != null && guard(q, world.tcpDisplacementBound(q, errors));
      }
      queries++;
      if (world.violation(q, bound, null, world.displacementBounds(q, errors)) == null) return;
    }
    check(t);
    if (failure != null) return;
    if (!bounded || depth >= depthLimit) {
      if (closed) {
        openStart = t0;
        openEnd = t1;
      }
      closed = false;
      return;
    }
    piece(t0, t, depth + 1);
    piece(t, t1, depth + 1);
  }

  function sampleClosest(t0:Float, t1:Float, count:Int):Void {
    for (i in 0...count) {
      var t = t0 + (t1 - t0) * (i + 0.5) / count;
      var q = trajectory.evaluate(t).positions;
      var found = world.closest(q, contactIn(q));
      if (found != null && (closest == null || found.distance < closest.distance)) {
        closest = found;
        closestTime = t;
      }
    }
  }
}
