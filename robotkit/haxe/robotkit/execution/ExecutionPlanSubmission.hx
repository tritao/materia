package robotkit.execution;

import robotkit.core.ImmutableFloatArray;

import haxe.Int64;
import RobotKitRuntime;

/** Immutable plan metadata and bounded polynomial payload for a runtime session. */
class ExecutionPlanSubmission {
  public final planId:Int64;
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  public final requiredCapabilities:Int;
  public final startPosition:ImmutableFloatArray;
  public final startVelocity:ImmutableFloatArray;
  public final startAcceleration:ImmutableFloatArray;
  public final positionTolerances:ImmutableFloatArray;
  public final velocityTolerances:ImmutableFloatArray;
  public final accelerationTolerances:ImmutableFloatArray;
  public final endsAtRest:Bool;
  public final jerkUnchecked:Bool;
  /** The payload, copied out of `arrays` on first use when the plan came as arrays. */
  public var segments(get, never):Array<TrajectorySegment>;
  /** The payload as native arrays an in-process runtime reads in place, or null. */
  public final arrays:Null<SegmentArrays>;
  var storedSegments:Null<Array<TrajectorySegment>>;
  public final events:Array<ProcessTimedEvent>;
  public final replaceAfterPlanId:Int64;
  public final replaceAfterTimeNs:Int64;

  public function new(planId:Int64, modelRevision:Int64, calibrationRevision:Int64,
      requiredCapabilities:Int, startPosition:Array<Float>, startVelocity:Array<Float>,
      startAcceleration:Array<Float>, segments:Null<Array<TrajectorySegment>>,
      ?replaceAfterPlanId:Int64, ?replaceAfterTimeNs:Int64,
      ?positionTolerances:Array<Float>, ?velocityTolerances:Array<Float>,
      ?accelerationTolerances:Array<Float>, ?endsAtRest:Bool = true,
      ?events:Array<ProcessTimedEvent>, ?jerkUnchecked:Bool = false, ?arrays:SegmentArrays) {
    var payloadJoints = arrays != null ? arrays.robotJointCount() :
      segments == null || segments.length == 0 || segments[0] == null ? -1 : segments[0].jointCount;
    if (planId == null || Int64.compare(planId, Int64.ofInt(0)) <= 0 ||
        startPosition == null || startVelocity == null || startAcceleration == null ||
        (arrays == null) == (segments == null) ||
        startPosition.length != startVelocity.length ||
        startPosition.length != startAcceleration.length ||
        payloadJoints != startPosition.length)
      throw "Invalid execution plan submission";
    for (state in [startPosition, startVelocity, startAcceleration])
      for (value in state) if (!Math.isFinite(value)) throw "Invalid execution plan start state";
    var count = arrays != null ? arrays.count() : segments == null ? 0 : segments.length;
    if (count < 1 || count > RobotKitRuntimeConstants.RK_MAX_TRAJECTORY_QUEUE_POINTS)
      throw "Invalid execution plan segment count";
    var expected = Int64.ofInt(0);
    if (arrays != null) {
      var origin = arrays.starts.get(0);
      for (index in 0...count) {
        var duration = arrays.durations.get(index);
        if (Int64.compare(Int64.sub(arrays.starts.get(index), origin), expected) != 0 ||
            Int64.compare(duration, Int64.ofInt(0)) <= 0 ||
            arrays.degrees.get(index) < 0 || arrays.degrees.get(index) > SegmentArrays.STRIDE - 1)
          throw "Execution plan segments must be contiguous";
        var end = Int64.add(expected, duration);
        if (Int64.compare(end, expected) <= 0) throw "Invalid execution plan duration";
        expected = end;
      }
    } else {
      for (segment in segments) {
        if (segment == null || segment.jointCount != payloadJoints ||
            Int64.compare(segment.timeFromStartNs, expected) != 0)
          throw "Execution plan segments must be contiguous with one joint count";
        var end = Int64.add(expected, segment.durationNs);
        if (Int64.compare(end, expected) <= 0) throw "Invalid execution plan duration";
        expected = end;
      }
    }
    this.planId = planId;
    this.modelRevision = modelRevision;
    this.calibrationRevision = calibrationRevision;
    this.events = events == null ? [] : events.copy();
    var previous = Int64.ofInt(0);
    for (event in this.events) {
      if (event == null || Int64.compare(event.timeNs, previous) < 0 ||
          Int64.compare(event.timeNs, expected) > 0)
        throw "Plan events must be sorted and within the plan duration";
      previous = event.timeNs;
    }
    this.requiredCapabilities = requiredCapabilities |
      (this.events.length > 0 ? RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_EVENTS : 0);
    this.startPosition = new ImmutableFloatArray(startPosition);
    this.startVelocity = new ImmutableFloatArray(startVelocity);
    this.startAcceleration = new ImmutableFloatArray(startAcceleration);
    var joints = startPosition.length;
    var pTol = positionTolerances == null ? [for (_ in 0...joints) 0.0] : positionTolerances;
    var vTol = velocityTolerances == null ? [for (_ in 0...joints) 0.0] : velocityTolerances;
    var aTol = accelerationTolerances == null ? [for (_ in 0...joints) 0.0] : accelerationTolerances;
    if (pTol.length != joints || vTol.length != joints || aTol.length != joints)
      throw "Execution plan tolerance count mismatch";
    for (joint in 0...joints)
      if (!Math.isFinite(pTol[joint]) || pTol[joint] < 0.0 ||
          !Math.isFinite(vTol[joint]) || vTol[joint] < 0.0 ||
          !Math.isFinite(aTol[joint]) || aTol[joint] < 0.0)
        throw "Invalid execution plan start tolerance";
    this.positionTolerances = new ImmutableFloatArray(pTol);
    this.velocityTolerances = new ImmutableFloatArray(vTol);
    this.accelerationTolerances = new ImmutableFloatArray(aTol);
    this.endsAtRest = endsAtRest;
    this.jerkUnchecked = jerkUnchecked;
    this.arrays = arrays;
    storedSegments = segments == null ? null : [for (segment in segments) segment.copy()];
    this.replaceAfterPlanId = replaceAfterPlanId == null ? Int64.ofInt(0) : replaceAfterPlanId;
    this.replaceAfterTimeNs = replaceAfterTimeNs == null ? Int64.ofInt(0) : replaceAfterTimeNs;
  }

  /**
    A plan whose payload is native segment arrays, read in place by an
    in-process runtime and copied out for any other robot.
  **/
  public static function ofArrays(planId:Int64, modelRevision:Int64, calibrationRevision:Int64,
      requiredCapabilities:Int, startPosition:Array<Float>, startVelocity:Array<Float>,
      startAcceleration:Array<Float>, arrays:SegmentArrays, positionTolerances:Array<Float>,
      velocityTolerances:Array<Float>, accelerationTolerances:Array<Float>, endsAtRest:Bool,
      events:Array<ProcessTimedEvent>, jerkUnchecked:Bool):ExecutionPlanSubmission
    return new ExecutionPlanSubmission(planId, modelRevision, calibrationRevision,
      requiredCapabilities, startPosition, startVelocity, startAcceleration, null, null, null,
      positionTolerances, velocityTolerances, accelerationTolerances, endsAtRest, events,
      jerkUnchecked, arrays);

  function get_segments():Array<TrajectorySegment> {
    var stored = storedSegments;
    if (stored != null) return stored;
    var source = arrays;
    if (source == null) throw "Execution plan submission has no payload";
    var copied = source.segments();
    storedSegments = copied;
    return copied;
  }

  public function copy():ExecutionPlanSubmission
    return new ExecutionPlanSubmission(planId, modelRevision, calibrationRevision,
      requiredCapabilities, startPosition.toArray(), startVelocity.toArray(),
      startAcceleration.toArray(), arrays == null ? segments : null, replaceAfterPlanId,
      replaceAfterTimeNs, positionTolerances.toArray(), velocityTolerances.toArray(),
      accelerationTolerances.toArray(), endsAtRest, events, jerkUnchecked, arrays);
}
