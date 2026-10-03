package motionkit.robot;

import haxe.Int64;
import robotkit.manipulation.Manipulator;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.TrajectorySegment;

/** Settings for servoing through streamed plan chunks (see `ServoSession`). */
class ServoPlanOptions {
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  /**
   * How much motion to keep queued ahead of execution. It must cover the
   * gap between updates plus the link (a device commits segments that far
   * ahead), and it is the servo's latency and how long a stalled host lets
   * the arm run on before the queue runs dry.
   */
  public final leadSeconds:Float;

  public function new(modelRevision:Int64, calibrationRevision:Int64, ?leadSeconds:Float = 0.04) {
    if (!(leadSeconds > 0.0) || !Math.isFinite(leadSeconds)) throw "Servo plan lead must be positive";
    this.modelRevision = modelRevision;
    this.calibrationRevision = calibrationRevision;
    this.leadSeconds = leadSeconds;
  }
}

/**
 * The plan stream `ServoSession` keeps on robots that execute plans rather
 * than joint targets (the virtual device), and the host's copy of its end.
 *
 * Each chunk is one control period appended at the end of the queue: every
 * arm joint ramps at constant acceleration from its velocity there to the
 * servo's next velocity (other joints hold still), so chunks join with
 * continuous position and velocity and a step in acceleration (they are
 * jerk-unchecked). Chunks do not end at rest while servoing; a braking
 * stream's last chunk does. Should the host stall, the queue runs dry while
 * moving, and the runtime or device brakes each joint at its limit.
 */
class ServoPlan {
  static var nextId:Int64 = Int64.ofInt(0x40000000);

  public final options:ServoPlanOptions;
  final jointCount:Int;
  final indices:Array<Int>;
  /** A hair under each arm joint's acceleration limit, so ramps cannot exceed it by rounding. */
  public final accelerationLimits:Array<Float>;
  /** True while the queue holds this stream's chunks. */
  public var streaming(default, null) = false;
  /** True once the stream's last chunk ended at rest. */
  public var endedAtRest(default, null) = false;
  /** End of the queue on the plan clock, and every joint's state there. */
  public var endNs(default, null):Int64 = Int64.ofInt(0);
  public final endPosition:Array<Float>;
  public final endVelocity:Array<Float>;
  public final endAcceleration:Array<Float>;

  public function new(options:ServoPlanOptions, manipulator:Manipulator, jointCount:Int) {
    if (options == null) throw "Servo plans need options";
    this.options = options;
    this.jointCount = jointCount;
    indices = manipulator.jointIndices();
    accelerationLimits = [];
    for (k in 0...indices.length) {
      var limit = manipulator.group.limitsOf(k).maxAcceleration;
      if (limit == null || !(limit > 0.0)) throw 'Servo plans need an acceleration limit on arm joint $k';
      accelerationLimits.push(0.999 * limit);
    }
    endPosition = [for (_ in 0...jointCount) 0.0];
    endVelocity = [for (_ in 0...jointCount) 0.0];
    endAcceleration = [for (_ in 0...jointCount) 0.0];
  }

  /** Starts a new stream from rest at `positions` (every joint): the robot's setpoint, where a new plan anchors. */
  public function begin(positions:Array<Float>):Void {
    for (j in 0...jointCount) {
      endPosition[j] = positions[j];
      endVelocity[j] = 0.0;
      endAcceleration[j] = 0.0;
    }
    endNs = Int64.ofInt(0);
    streaming = false;
    endedAtRest = false;
  }

  /** The arm joints' state at the end of the queue, in arm order. */
  public function armPosition():Array<Float> return [for (index in indices) endPosition[index]];
  public function armVelocity():Array<Float> return [for (index in indices) endVelocity[index]];

  /**
   * The next chunk: `durationNs` from the end of the queue, ramping the arm
   * joints to `velocity` (arm order). The first chunk of a stream starts a
   * new plan; later ones append to it.
   */
  public function chunk(velocity:Array<Float>, durationNs:Int64, endsAtRest:Bool):ExecutionPlanSubmission {
    var target = [for (_ in 0...jointCount) 0.0];
    for (k in 0...indices.length) target[indices[k]] = velocity[k];
    var t = Int64.toFloat(durationNs) * 1e-9;
    var coefficients = [for (j in 0...jointCount) [endPosition[j], endVelocity[j], 0.5 * (target[j] - endVelocity[j]) / t]];
    var accelerationTolerance = [for (j in 0...jointCount) Math.abs(2.0 * coefficients[j][2] - endAcceleration[j]) + 1e-5];
    var tight = [for (_ in 0...jointCount) 1e-6];
    nextId = nextId + Int64.ofInt(1);
    return new ExecutionPlanSubmission(nextId, options.modelRevision, options.calibrationRevision,
      RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE, endPosition.copy(), endVelocity.copy(),
      endAcceleration.copy(), [new TrajectorySegment(Int64.ofInt(0), durationNs, coefficients)], null, null,
      tight, tight, accelerationTolerance, endsAtRest, null, true);
  }

  /** Records a chunk the robot accepted: the queue now ends where it does. */
  public function accept(plan:ExecutionPlanSubmission):Void {
    var segment = plan.segments[0];
    var t = Int64.toFloat(segment.durationNs) * 1e-9;
    for (j in 0...jointCount) {
      var c = segment.coefficients[j];
      endPosition[j] = c[0] + (c[1] + c[2] * t) * t;
      endVelocity[j] = c[1] + 2.0 * c[2] * t;
      endAcceleration[j] = 2.0 * c[2];
    }
    if (plan.endsAtRest) for (j in 0...jointCount) endVelocity[j] = 0.0;
    endNs = endNs + segment.durationNs;
    streaming = true;
    endedAtRest = plan.endsAtRest;
  }

  /** Forgets the stream: its queue drained (at rest, or run dry and braked by the robot). */
  public function clear():Void {
    streaming = false;
    endedAtRest = false;
  }
}
