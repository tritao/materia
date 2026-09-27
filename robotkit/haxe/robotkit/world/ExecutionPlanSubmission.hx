package robotkit.world;

import haxe.Int64;

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
  public final segments:Array<TrajectorySegment>;
  public final replaceAfterPlanId:Int64;
  public final replaceAfterTimeNs:Int64;

  public function new(planId:Int64, modelRevision:Int64, calibrationRevision:Int64,
      requiredCapabilities:Int, startPosition:Array<Float>, startVelocity:Array<Float>,
      startAcceleration:Array<Float>, segments:Array<TrajectorySegment>,
      ?replaceAfterPlanId:Int64, ?replaceAfterTimeNs:Int64,
      ?positionTolerances:Array<Float>, ?velocityTolerances:Array<Float>,
      ?accelerationTolerances:Array<Float>) {
    if (planId == null || Int64.compare(planId, Int64.ofInt(0)) <= 0 ||
        startPosition == null || startVelocity == null || startAcceleration == null ||
        segments == null || segments.length == 0 ||
        startPosition.length != startVelocity.length ||
        startPosition.length != startAcceleration.length ||
        segments[0].jointCount != startPosition.length)
      throw "Invalid execution plan submission";
    this.planId = planId;
    this.modelRevision = modelRevision;
    this.calibrationRevision = calibrationRevision;
    this.requiredCapabilities = requiredCapabilities;
    this.startPosition = new ImmutableFloatArray(startPosition);
    this.startVelocity = new ImmutableFloatArray(startVelocity);
    this.startAcceleration = new ImmutableFloatArray(startAcceleration);
    var count = startPosition.length;
    var pTol = positionTolerances == null ? [for (_ in 0...count) 0.0] : positionTolerances;
    var vTol = velocityTolerances == null ? [for (_ in 0...count) 0.0] : velocityTolerances;
    var aTol = accelerationTolerances == null ? [for (_ in 0...count) 0.0] : accelerationTolerances;
    if (pTol.length != count || vTol.length != count || aTol.length != count)
      throw "Execution plan tolerance count mismatch";
    for (joint in 0...count)
      if (!Math.isFinite(pTol[joint]) || pTol[joint] < 0.0 ||
          !Math.isFinite(vTol[joint]) || vTol[joint] < 0.0 ||
          !Math.isFinite(aTol[joint]) || aTol[joint] < 0.0)
        throw "Invalid execution plan start tolerance";
    this.positionTolerances = new ImmutableFloatArray(pTol);
    this.velocityTolerances = new ImmutableFloatArray(vTol);
    this.accelerationTolerances = new ImmutableFloatArray(aTol);
    this.segments = [for (segment in segments) segment.copy()];
    this.replaceAfterPlanId = replaceAfterPlanId == null ? Int64.ofInt(0) : replaceAfterPlanId;
    this.replaceAfterTimeNs = replaceAfterTimeNs == null ? Int64.ofInt(0) : replaceAfterTimeNs;
  }

  public function copy():ExecutionPlanSubmission
    return new ExecutionPlanSubmission(planId, modelRevision, calibrationRevision,
      requiredCapabilities, startPosition.toArray(), startVelocity.toArray(),
      startAcceleration.toArray(), segments, replaceAfterPlanId, replaceAfterTimeNs,
      positionTolerances.toArray(), velocityTolerances.toArray(),
      accelerationTolerances.toArray());
}
