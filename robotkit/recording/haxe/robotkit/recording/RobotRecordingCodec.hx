package robotkit.recording;

import robotkit.core.JointTarget;
import robotkit.core.RobotCommand;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;
import robotkit.core.SensorFrame;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.execution.FiredProcessEvent;
import robotkit.execution.ProcessEventValue;
import robotkit.execution.ProcessTimedEvent;
import robotkit.execution.TrajectorySegment;
import robotkit.streams.CameraImage;
import robotkit.world.RobotWorldEvent;
import robotkit.world.WorldSnapshot;

import haxe.Int64;
import robotkit.protocol.RecordingCommandMsg;
import robotkit.protocol.RecordingFaultMsg;
import robotkit.protocol.RecordingImageMsg;
import robotkit.protocol.RecordingJointTargetMsg;
import robotkit.protocol.RecordingPlanMsg;
import robotkit.protocol.RecordingProcessEventMsg;
import robotkit.protocol.RecordingProcessValueMsg;
import robotkit.protocol.RecordingSegmentMsg;
import robotkit.protocol.RecordingSensorMsg;
import robotkit.protocol.RecordingSnapshotMsg;
import robotkit.protocol.RecordingTimedEventMsg;
import robotkit.protocol.RecordingWorldEventMsg;
import robotkit.protocol.RecordingWorldMsg;

/** Typed conversions between world values and v8 MessagePack payloads. */
class RobotRecordingCodec {
  public static inline final VERSION:Int = 8;

  public static function command(value:RobotCommand, robotId:RobotId):RecordingCommandMsg {
    var result = new RecordingCommandMsg();
    result.robotId = robotId;
    result.targets = [];
    result.expiryNs = Int64.ofInt(0);
    result.plan = null;
    switch value {
      case JointTargets(targets, expiryNs):
        result.kind = 1;
        result.targets = [for (target in targets) jointTarget(target)];
        result.expiryNs = expiryNs == null ? Int64.ofInt(0) : expiryNs;
      case ExecutionPlan(plan): result.kind = 3; result.plan = planMsg(plan);
      case Hold: result.kind = 4;
      case Resume: result.kind = 5;
      case Abort: result.kind = 6;
    }
    return result;
  }

  public static function readCommand(msg:RecordingCommandMsg):RobotCommand return switch msg.kind {
    case 1: JointTargets([for (target in msg.targets) readJointTarget(target)],
      Int64.compare(msg.expiryNs, Int64.ofInt(0)) == 0 ? null : msg.expiryNs);
    case 3: ExecutionPlan(readPlan(msg.plan));
    case 4: Hold;
    case 5: Resume;
    case 6: Abort;
    case _: throw "Unsupported recorded command kind";
  };

  static function jointTarget(value:JointTarget):RecordingJointTargetMsg {
    var msg = new RecordingJointTargetMsg();
    msg.joint = value.joint;
    msg.mode = switch value.mode {
      case Position: 1; case Velocity: 2; case Effort: 3; case Servo: 4;
    };
    msg.target = value.target;
    msg.velocity = value.servoVelocity;
    msg.stiffness = value.stiffness;
    msg.damping = value.damping;
    msg.feedforward = value.feedforward;
    return msg;
  }
  static function readJointTarget(msg:RecordingJointTargetMsg):JointTarget return switch msg.mode {
    case 1: JointTarget.position(msg.joint, msg.target);
    case 2: JointTarget.velocity(msg.joint, msg.target);
    case 3: JointTarget.effort(msg.joint, msg.target);
    case 4: JointTarget.servo(msg.joint, msg.target, msg.velocity,
      msg.stiffness, msg.damping, msg.feedforward);
    case _: throw "Unsupported recorded joint target mode";
  };
  static function segmentMsg(value:TrajectorySegment):RecordingSegmentMsg {
    var msg = new RecordingSegmentMsg();
    msg.timeFromStartNs = value.timeFromStartNs;
    msg.durationNs = value.durationNs;
    msg.coefficients = [for (row in value.coefficients) row.copy()];
    return msg;
  }
  static function readSegment(msg:RecordingSegmentMsg):TrajectorySegment
    return new TrajectorySegment(msg.timeFromStartNs, msg.durationNs, msg.coefficients);

  static function planMsg(value:ExecutionPlanSubmission):RecordingPlanMsg {
    var msg = new RecordingPlanMsg();
    msg.planId = value.planId;
    msg.modelRevision = value.modelRevision;
    msg.calibrationRevision = value.calibrationRevision;
    msg.requiredCapabilities = value.requiredCapabilities;
    msg.startPosition = value.startPosition.toArray();
    msg.startVelocity = value.startVelocity.toArray();
    msg.startAcceleration = value.startAcceleration.toArray();
    msg.positionTolerances = value.positionTolerances.toArray();
    msg.velocityTolerances = value.velocityTolerances.toArray();
    msg.accelerationTolerances = value.accelerationTolerances.toArray();
    msg.endsAtRest = value.endsAtRest;
    msg.jerkUnchecked = value.jerkUnchecked;
    msg.segments = [for (segment in value.segments) segmentMsg(segment)];
    msg.events = [for (event in value.events) timedEvent(event)];
    msg.replaceAfterPlanId = value.replaceAfterPlanId;
    msg.replaceAfterTimeNs = value.replaceAfterTimeNs;
    return msg;
  }
  static function readPlan(msg:RecordingPlanMsg):ExecutionPlanSubmission {
    finiteArray(msg.startPosition, "plan start position");
    finiteArray(msg.startVelocity, "plan start velocity");
    finiteArray(msg.startAcceleration, "plan start acceleration");
    finiteArray(msg.positionTolerances, "plan position tolerances");
    finiteArray(msg.velocityTolerances, "plan velocity tolerances");
    finiteArray(msg.accelerationTolerances, "plan acceleration tolerances");
    return new ExecutionPlanSubmission(msg.planId, msg.modelRevision,
      msg.calibrationRevision, msg.requiredCapabilities, msg.startPosition,
      msg.startVelocity, msg.startAcceleration,
      [for (segment in msg.segments) readSegment(segment)], msg.replaceAfterPlanId,
      msg.replaceAfterTimeNs, msg.positionTolerances, msg.velocityTolerances,
      msg.accelerationTolerances, msg.endsAtRest,
      [for (event in msg.events) readTimedEvent(event)], msg.jerkUnchecked);
  }
  static function timedEvent(value:ProcessTimedEvent):RecordingTimedEventMsg {
    var msg = new RecordingTimedEventMsg();
    msg.timeNs = value.timeNs;
    msg.channel = value.channel;
    msg.value = processValue(value.value);
    msg.holdPolicy = switch value.holdPolicy {
      case Keep: 1; case SafeWhileHeld: 2; case RestoreOnResume: 3;
    };
    return msg;
  }
  static function readTimedEvent(msg:RecordingTimedEventMsg):ProcessTimedEvent
    return new ProcessTimedEvent(msg.timeNs, msg.channel, readProcessValue(msg.value),
      switch msg.holdPolicy {
        case 1: Keep; case 2: SafeWhileHeld; case 3: RestoreOnResume;
        case _: throw "Unsupported recorded process hold policy";
      });
  static function processValue(value:ProcessEventValue):RecordingProcessValueMsg {
    var msg = new RecordingProcessValueMsg();
    msg.digital = false; msg.analog = 0.0; msg.command = ""; msg.argument = 0.0;
    switch value {
      case Digital(enabled): msg.kind = 1; msg.digital = enabled;
      case Analog(number): msg.kind = 2; msg.analog = number;
      case Process(command, argument): msg.kind = 3; msg.command = command; msg.argument = argument;
    }
    return msg;
  }
  static function readProcessValue(msg:RecordingProcessValueMsg):ProcessEventValue
    return switch msg.kind {
      case 1: Digital(msg.digital);
      case 2: Analog(msg.analog);
      case 3: Process(msg.command, msg.argument);
      case _: throw "Unsupported recorded process value";
    };

  public static function sensor(value:SensorFrame, robotId:RobotId):RecordingSensorMsg {
    var msg = new RecordingSensorMsg();
    msg.robotId = robotId;
    msg.sensorId = value.sensorId;
    msg.kind = value.kind;
    msg.frameId = value.frameId;
    msg.sequence = value.sequence;
    msg.sourceTimestampNs = value.sourceTimestampNs;
    msg.receivedTimestampNs = value.receivedTimestampNs;
    msg.sourceClockId = value.sourceClockId;
    msg.receivedClockId = value.receivedClockId;
    msg.values = value.values.toArray();
    msg.linkId = value.linkId;
    msg.mountPosition = value.mountPosition.toArray();
    msg.mountRotation = value.mountRotation.toArray();
    msg.image = null;
    if (value.image != null) {
      var image = new RecordingImageMsg();
      image.width = value.image.width;
      image.height = value.image.height;
      image.encoding = value.image.encoding;
      image.pixels = value.image.bytes();
      msg.image = image;
    }
    return msg;
  }
  public static function readSensor(msg:RecordingSensorMsg):SensorFrame {
    finiteArray(msg.values, "sensor values");
    if (msg.mountPosition == null || msg.mountPosition.length != 3 ||
        msg.mountRotation == null || msg.mountRotation.length != 4)
      throw "Recorded sensor mount dimensions are invalid";
    finiteArray(msg.mountPosition, "sensor mount position");
    finiteArray(msg.mountRotation, "sensor mount rotation");
    var norm = 0.0;
    for (value in msg.mountRotation) norm += value * value;
    if (Math.abs(norm - 1.0) > 0.000001)
      throw "Recorded sensor mount rotation is not a unit quaternion";
    return new SensorFrame(msg.sensorId, msg.kind, msg.frameId, msg.sequence,
      msg.sourceTimestampNs, msg.values, msg.receivedTimestampNs, msg.linkId,
      msg.mountPosition, msg.mountRotation, msg.sourceClockId, msg.receivedClockId,
      msg.image == null ? null : new CameraImage(msg.image.width, msg.image.height,
        msg.image.encoding, msg.image.pixels));
  }

  public static function snapshot(value:RobotSnapshot):RecordingSnapshotMsg {
    var msg = new RecordingSnapshotMsg();
    msg.id = value.id;
    msg.sourceSequence = value.sourceSequence;
    msg.sourceTimestampNs = value.sourceTimestampNs;
    msg.receivedTimestampNs = value.receivedTimestampNs;
    msg.sourceClockId = value.sourceClockId;
    msg.receivedClockId = value.receivedClockId;
    msg.positions = value.positions.toArray();
    msg.velocities = value.velocities.toArray();
    msg.efforts = value.efforts.toArray();
    msg.streamSequences = [for (stream in value.streamSequences) new robotkit.protocol.StreamSequenceMsg(stream.streamId,stream.kind,stream.sequence,stream.sourceClockId)];
    msg.sensors = [for (sensor in value.sensors.toArray()) RobotRecordingCodec.sensor(sensor, value.id)];
    msg.mode = value.mode;
    msg.faultCode = value.faultCode;
    msg.safety = value.safety;
    msg.trajectoryQueueDepth = value.trajectoryQueueDepth;
    msg.trajectoryActive = value.trajectoryActive;
    msg.trajectoryTimeNs = value.trajectoryTimeNs;
    msg.trajectoryDurationNs = value.trajectoryDurationNs;
    msg.trajectoryTag = value.trajectoryTag;
    msg.trajectoryTagTimeNs = value.trajectoryTagTimeNs;
    msg.sessionState = value.sessionState;
    msg.activePlanId = value.activePlanId;
    msg.committedUntilNs = value.committedUntilNs;
    msg.queueEndTimeNs = value.queueEndTimeNs;
    return msg;
  }
  public static function readSnapshot(msg:RecordingSnapshotMsg):RobotSnapshot {
    finiteArray(msg.positions, "snapshot positions");
    finiteArray(msg.velocities, "snapshot velocities");
    finiteArray(msg.efforts, "snapshot efforts");
    return new RobotSnapshot(msg.id, msg.sourceSequence, msg.sourceTimestampNs,
      msg.positions, msg.velocities, msg.efforts, msg.mode, msg.faultCode,
      msg.receivedTimestampNs, [for (sensor in msg.sensors) readSensor(sensor)],
      msg.sourceClockId, msg.receivedClockId, msg.safety, msg.trajectoryQueueDepth,
      msg.trajectoryActive, msg.trajectoryTimeNs, msg.trajectoryDurationNs,
      msg.trajectoryTag, msg.trajectoryTagTimeNs, msg.sessionState, msg.activePlanId,
      msg.committedUntilNs, msg.queueEndTimeNs, null, robotkit.protocol.StreamSequenceMsg.values(msg.streamSequences));
  }

  static function finiteArray(values:Array<Float>, label:String):Void {
    if (values == null) throw 'Recorded $label is missing';
    for (value in values) if (!Math.isFinite(value)) throw 'Recorded $label is not finite';
  }

  public static function fault(value:RobotFault):RecordingFaultMsg {
    var msg = new RecordingFaultMsg();
    msg.id = value.id; msg.code = value.code; msg.message = value.message; msg.fatal = value.fatal;
    return msg;
  }
  public static function readFault(msg:RecordingFaultMsg):RobotFault
    return new RobotFault(msg.id, msg.code, msg.message, msg.fatal);

  public static function world(value:WorldSnapshot):RecordingWorldMsg {
    var msg = new RecordingWorldMsg();
    msg.sequence = value.sequence;
    msg.topologyRevision = value.topologyRevision;
    msg.sourceTimestampNs = value.sourceTimestampNs;
    msg.receivedTimestampNs = value.receivedTimestampNs;
    msg.robots = [for (robot in value.robots()) snapshot(robot)];
    return msg;
  }
  public static function readWorld(msg:RecordingWorldMsg):WorldSnapshot {
    var robots = new Map<RobotId, RobotSnapshot>();
    for (snapshot in msg.robots) {
      var value = readSnapshot(snapshot);
      robots.set(value.id, value);
    }
    return new WorldSnapshot(msg.sequence, msg.topologyRevision,
      msg.sourceTimestampNs, robots, msg.receivedTimestampNs);
  }
  public static function worldEvent(value:RobotWorldEvent):RecordingWorldEventMsg {
    var msg = new RecordingWorldEventMsg();
    switch value {
      case RobotAttached(id): msg.kind = 1; msg.robotId = id;
      case RobotDetached(id): msg.kind = 2; msg.robotId = id;
      case RobotChanged(id): msg.kind = 3; msg.robotId = id;
    }
    return msg;
  }
  public static function readWorldEvent(msg:RecordingWorldEventMsg):RobotWorldEvent
    return switch msg.kind {
      case 1: RobotAttached(msg.robotId);
      case 2: RobotDetached(msg.robotId);
      case 3: RobotChanged(msg.robotId);
      case _: throw "Unsupported recorded world event";
    };
  public static function processEvent(value:FiredProcessEvent, robotId:RobotId):RecordingProcessEventMsg {
    var msg = new RecordingProcessEventMsg();
    msg.robotId = robotId;
    msg.planId = value.planId;
    msg.channel = value.channel;
    msg.value = processValue(value.value);
    msg.scheduledTimeNs = value.scheduledTimeNs;
    msg.appliedOwnerTimeNs = value.appliedOwnerTimeNs;
    msg.cause = value.cause;
    return msg;
  }
  public static function readProcessEvent(msg:RecordingProcessEventMsg):FiredProcessEvent
    return new FiredProcessEvent(msg.planId, msg.channel, readProcessValue(msg.value),
      msg.scheduledTimeNs, msg.appliedOwnerTimeNs, msg.cause);
}
