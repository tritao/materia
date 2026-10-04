package robotkit.protocol;

import RobotKitRuntime;

import haxe.Int64;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.TrajectorySegment;

/** Versioned wire shape of an execution plan, including replacement metadata. */
@:wire
class PlanSubmission {
  @:id(1) public var robotId:Int64;
  @:id(2) public var planId:Int64;
  @:id(3) public var modelRevision:Int64;
  @:id(4) public var calibrationRevision:Int64;
  @:id(5) public var requiredCapabilities:Int;
  @:id(6) public var startPosition:Array<Float>;
  @:id(7) public var startVelocity:Array<Float>;
  @:id(8) public var startAcceleration:Array<Float>;
  @:id(9) public var positionTolerances:Array<Float>;
  @:id(10) public var velocityTolerances:Array<Float>;
  @:id(11) public var accelerationTolerances:Array<Float>;
  @:id(12) public var endsAtRest:Bool;
  @:id(13) public var segments:Array<PlanSegment>;
  @:id(14) public var replaceAfterPlanId:Int64;
  @:id(15) public var replaceAfterTimeNs:Int64;
  @:id(16) public var jerkUnchecked:Bool;

  @:optional @:id(17) public var controlAcceleration:Null<Array<Float>>;

  public function new(?robotId:Int64, ?planId:Int64, ?modelRevision:Int64,
      ?calibrationRevision:Int64, ?requiredCapabilities:Int = 0,
      ?startPosition:Array<Float>, ?startVelocity:Array<Float>,
      ?startAcceleration:Array<Float>, ?positionTolerances:Array<Float>,
      ?velocityTolerances:Array<Float>, ?accelerationTolerances:Array<Float>,
      ?endsAtRest:Bool = true, ?segments:Array<PlanSegment>,
      ?replaceAfterPlanId:Int64, ?replaceAfterTimeNs:Int64,
      ?jerkUnchecked:Bool = false, ?controlAcceleration:Array<Float>) {
    this.robotId = robotId == null ? Int64.ofInt(0) : robotId;
    this.planId = planId == null ? Int64.ofInt(0) : planId;
    this.modelRevision = modelRevision == null ? Int64.ofInt(0) : modelRevision;
    this.calibrationRevision = calibrationRevision == null ? Int64.ofInt(0) : calibrationRevision;
    this.requiredCapabilities = requiredCapabilities;
    this.startPosition = startPosition == null ? [] : startPosition;
    this.startVelocity = startVelocity == null ? [] : startVelocity;
    this.startAcceleration = startAcceleration == null ? [] : startAcceleration;
    this.positionTolerances = positionTolerances == null ? [] : positionTolerances;
    this.velocityTolerances = velocityTolerances == null ? [] : velocityTolerances;
    this.accelerationTolerances = accelerationTolerances == null ? [] : accelerationTolerances;
    this.endsAtRest = endsAtRest;
    this.jerkUnchecked = jerkUnchecked;
    this.controlAcceleration = controlAcceleration;
    this.segments = segments == null ? [] : segments;
    this.replaceAfterPlanId = replaceAfterPlanId == null ? Int64.ofInt(0) : replaceAfterPlanId;
    this.replaceAfterTimeNs = replaceAfterTimeNs == null ? Int64.ofInt(0) : replaceAfterTimeNs;
  }

  public static function fromWorld(robotId:Int64, plan:ExecutionPlanSubmission):PlanSubmission {
    var segments = [for (segment in plan.segments) {
      var flat = [for (joint in segment.coefficients) for (coefficient in joint) coefficient];
      new PlanSegment(segment.timeFromStartNs, segment.durationNs,
        segment.jointCount, segment.degree, flat);
    }];
    return new PlanSubmission(robotId, plan.planId, plan.modelRevision,
      plan.calibrationRevision, plan.requiredCapabilities,
      plan.startPosition.toArray(), plan.startVelocity.toArray(),
      plan.startAcceleration.toArray(), plan.positionTolerances.toArray(),
      plan.velocityTolerances.toArray(), plan.accelerationTolerances.toArray(),
      plan.endsAtRest, segments, plan.replaceAfterPlanId, plan.replaceAfterTimeNs,
      plan.jerkUnchecked, plan.statedControlAcceleration());
  }

  public function toWorld():ExecutionPlanSubmission {
    if (segments == null || segments.length == 0 ||
        segments.length > RobotKitRuntimeConstants.RK_MAX_TRAJECTORY_QUEUE_POINTS)
      throw "Plan segment count is invalid";
    var converted:Array<TrajectorySegment> = [];
    for (segment in segments) {
      if (segment == null || segment.jointCount < 1 || segment.jointCount > 64 ||
          segment.degree < 0 || segment.degree > 5 || segment.coefficients == null ||
          segment.coefficients.length != segment.jointCount * (segment.degree + 1))
        throw "Plan segment coefficients are invalid";
      var coefficients:Array<Array<Float>> = [];
      for (joint in 0...segment.jointCount) {
        var row:Array<Float> = [];
        for (degree in 0...(segment.degree + 1))
          row.push(segment.coefficients[joint * (segment.degree + 1) + degree]);
        coefficients.push(row);
      }
      converted.push(new TrajectorySegment(segment.timeFromStartNs,
        segment.durationNs, coefficients));
    }
    return new ExecutionPlanSubmission(planId, modelRevision, calibrationRevision,
      requiredCapabilities, startPosition, startVelocity, startAcceleration,
      converted, replaceAfterPlanId, replaceAfterTimeNs,
      positionTolerances, velocityTolerances, accelerationTolerances, endsAtRest,
      null, jerkUnchecked, null, controlAcceleration);
  }
}
