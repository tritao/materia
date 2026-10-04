package robotkit.runtime;

import robotkit.core.ExecutionCapabilities;
import robotkit.core.Robot;
import robotkit.core.RobotCapabilities;
import robotkit.core.RobotCommand;
import robotkit.core.RobotDescription;
import robotkit.core.RobotEvent;
import robotkit.core.RobotEventRing;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSensorFrames;
import robotkit.core.RobotSnapshot;
import robotkit.core.RobotStatus;
import robotkit.core.SensorFrame;
import robotkit.core.StopMode;
import robotkit.world.RobotWorld;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.runtime.RobotRuntime;

/** Shared RobotWorld adapter for runtimes owned either by simulation or a device. */
class RuntimeRobotAdapter implements Robot {
  public final logicalId:RobotId;

  final runtime:RobotRuntime;
  final robotDescription:RobotDescription;
  final robotCapabilities:RobotCapabilities;
  final ownsRuntime:Bool;
  final faultMessage:String;
  var changeListener:Null < RobotId -> Void > = null;
  var commandSequence:Int = 0;
  var observedSequence:Int64 = Int64.ofInt(-1);
  var observedReceipt:Int64 = Int64.ofInt(-1);
  var currentSensors:Array<SensorFrame> = [];
  final eventRing = new RobotEventRing();
  final sensorStreams = new robotkit.streams.SensorStreams();
  var closed:Bool = false;

  public function new(id:RobotId, runtime:RobotRuntime, name:String,
      links:Array<String>, joints:Array<String>, ?ownsRuntime:Bool = false,
      ?startRuntime:Bool = false, ?faultMessage:String = "robot runtime fault",
      ?executionPolicy:ExecutionCapabilities) {
    if (id == null || id.length == 0)
      throw "RuntimeRobotAdapter requires a non-empty logical ID";
    if (runtime == null)
      throw "RuntimeRobotAdapter requires a runtime";
    logicalId = id;
    this.runtime = runtime;
    this.ownsRuntime = ownsRuntime;
    this.faultMessage = faultMessage;
    robotDescription = new RobotDescription(id, name, links, joints, runtime.channels, runtime.couplings);
    var actual = runtime.capabilities(id);
    var policy = executionPolicy;
    var execution = actual.execution;
    if (policy != null) {
      if ((policy.plans && !execution.plans) ||
          policy.maximumPolynomialDegree > execution.maximumPolynomialDegree ||
          policy.maximumJoints > execution.maximumJoints || policy.maximumSegments > execution.maximumSegments ||
          (policy.timedEvents && !execution.timedEvents) ||
          (policy.replacementBoundaries && !execution.replacementBoundaries) ||
          (policy.holdResume && !execution.holdResume))
        throw "Execution policy exceeds endpoint capabilities";
      execution = policy.plans ? new ExecutionCapabilities(true, policy.maximumPolynomialDegree,
        policy.maximumJoints, policy.maximumSegments, policy.timedEvents,
        policy.replacementBoundaries, policy.holdResume, actual.execution.polynomialLimits)
        : ExecutionCapabilities.unavailable();
    }
    robotCapabilities = new RobotCapabilities(id, actual.jointCount, actual.controlModes,
      execution, actual.timing, actual.streams);
    if (startRuntime) {
      try runtime.start() catch (error:Dynamic) {
        if (ownsRuntime) runtime.dispose();
        throw error;
      }
    }
  }

  public function id():RobotId return logicalId;

  public function status():RobotStatus {
    if (closed) return Disconnected;
    var value = runtime.snapshot();
    return value.endpoint == RobotKitRuntimeConstants.RK_ENDPOINT_FAULT ||
      value.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ? Fault : Ready;
  }

  public function description():RobotDescription return robotDescription;
  public function capabilities():RobotCapabilities return robotCapabilities;

  public function snapshot():RobotSnapshot {
    ensureOpen();
    var value = runtime.snapshot();
    observe(value);
    return new RobotSnapshot(logicalId, value.sequence, value.sourceTimestampNs,
      value.q.toArray(), value.dq.toArray(), value.effort.toArray(), value.mode,
      value.faultCode, value.receivedTimestampNs, currentSensors,
      runtime.sourceClockId, "robotkit.monotonic", value.safety,
      value.trajectoryQueueDepth, value.trajectoryActive,
      value.trajectoryTimeNs, value.trajectoryDurationNs,
      value.trajectoryTag, value.trajectoryTagTimeNs,
      value.sessionState, value.activePlanId,
      value.committedUntilNs, value.queueEndTimeNs, value.setpoint.toArray(), sensorStreams.sequences());
  }

  public function fault():Null<RobotFault> {
    if (closed) return null;
    var value = runtime.snapshot();
    if (value.faultCode == 0 && value.safety != RobotKitRuntimeConstants.RK_SAFETY_FAULT)
      return null;
    if (value.faultCode == RobotKitRuntimeConstants.RK_FAULT_TRAJECTORY_UNDERFLOW)
      return new RobotFault(logicalId, value.faultCode, "trajectory_underflow", false);
    if (value.faultCode == RobotKitRuntimeConstants.RK_FAULT_LIMIT_SWITCH)
      return new RobotFault(logicalId, value.faultCode, "limit_switch", true);
    if (value.faultCode == RobotKitRuntimeConstants.RK_FAULT_RAMP_LIMIT)
      return new RobotFault(logicalId, value.faultCode, "ramp_limit", true);
    return new RobotFault(logicalId, value.faultCode, faultMessage, true);
  }

  public function events(afterOrdinal:Int64, max:Int):Array<RobotEvent>
    return eventRing.events(afterOrdinal, max);

  public function publishObservation(value:robotkit.streams.ImageDetectionObservation):RobotEvent {
    sensorStreams.publish(robotkit.streams.SensorStreamSample.inference(value));
    return eventRing.publish(value);
  }

  public function submit(command:RobotCommand):Void {
    ensureOpen();
    switch command {
      case JointTargets(targets, expiryNs):
        // In-process, the deadline is on the snapshot's source clock, which the runtime maps itself.
        commandSequence++;
        runtime.submitTargets(targets, commandSequence, null,
          expiryNs == null || Int64.compare(expiryNs, Int64.ofInt(0)) == 0 ? null : expiryNs);
      case ExecutionPlan(plan):
        if (!robotCapabilities.execution.plans)
          throw "Runtime endpoint does not support execution plans";
        var contract = robotCapabilities.execution;
        var payload = plan.arrays;
        var count = payload == null ? plan.segments.length : payload.count();
        if (plan.startPosition.length > contract.maximumJoints || count > contract.maximumSegments ||
            (plan.events.length > 0 && !contract.timedEvents) ||
            (Int64.compare(plan.replaceAfterPlanId, Int64.ofInt(0)) != 0 && !contract.replacementBoundaries))
          throw "Plan exceeds endpoint execution capabilities";
        for (index in 0...count) {
          var degree = payload == null ? plan.segments[index].degree : payload.degrees.get(index);
          if (degree > contract.maximumPolynomialDegree)
            throw "Plan exceeds endpoint polynomial degree";
        }
        commandSequence++;
        runtime.submitPlan(plan, commandSequence);
      case Hold:
        if (!robotCapabilities.execution.holdResume) throw "Endpoint cannot hold a plan";
        commandSequence++;
        runtime.submitHold(commandSequence);
      case Resume:
        if (!robotCapabilities.execution.holdResume) throw "Endpoint cannot resume a plan";
        commandSequence++;
        runtime.submitResume(commandSequence);
      case Abort:
        commandSequence++;
        runtime.submitAbort(commandSequence);
    }
  }

  public function stop(mode:StopMode):Void {
    ensureOpen();
    commandSequence++;
    runtime.submitStop(commandSequence, mode == StopMode.Emergency);
  }

  public function resetSafety():Void {
    ensureOpen();
    commandSequence++;
    runtime.resetSafety(commandSequence);
  }

  public function streams():robotkit.streams.SensorStreams {
    ensureOpen();
    observe(runtime.snapshot());
    return sensorStreams;
  }

  public function setChangeListener(listener:Null < RobotId -> Void >):Void {
    changeListener = listener;
  }

  public function close():Void {
    if (closed) return;
    closed = true;
    changeListener = null;
    if (ownsRuntime) runtime.dispose();
  }

  function observe(value:robotkit.runtime.RobotSnapshot):Void {
    // Externally published sensors (cameras, GNSS) update without a native
    // state change, so compare the sensor frames as well as the sequence.
    var sensors = RobotSensorFrames.fromRuntimeSnapshot(value);
    if (value.sequence == observedSequence && value.receivedTimestampNs == observedReceipt &&
        sameSensors(currentSensors, sensors))
      return;
    observedSequence = value.sequence;
    observedReceipt = value.receivedTimestampNs;
    currentSensors = sensors;
    sensorStreams.publishFrames(sensors);
    var listener = changeListener;
    if (listener != null) listener(logicalId);
  }

  static function sameSensors(left:Array<SensorFrame>, right:Array<SensorFrame>):Bool {
    if (left == null || left.length != right.length) return false;
    for (index in 0...left.length) {
      var a = left[index], b = right[index];
      if (a.sensorId != b.sensorId || a.sequence != b.sequence ||
          a.sourceTimestampNs != b.sourceTimestampNs ||
          a.receivedTimestampNs != b.receivedTimestampNs)
        return false;
    }
    return true;
  }

  function ensureOpen():Void {
    if (closed) throw "RuntimeRobotAdapter has been closed";
  }
}
