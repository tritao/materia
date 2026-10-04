package robotkit.protocol;
import haxe.Int64;
import trajectorykit.validation.ValidationGuarantee;
/** One conversion for local and remote capability records; unknown modes are rejected. */
class CapabilityCodec {
  public static function encode(value:robotkit.world.RobotCapabilities, robotId:Int64):RobotCapabilities {
    var result = new RobotCapabilities();
    result.robotId = robotId;
    result.jointCount = value.jointCount;
    result.controlModes = [for (mode in value.controlModes) switch mode {
      case Position: "position"; case Velocity: "velocity"; case Effort: "effort"; case Servo: "servo";
    }];
    result.streams = value.streams;
    var source = value.execution;
    var execution = new ExecutionCapabilitiesMsg();
    execution.plans = source.plans;
    execution.maximumPolynomialDegree = source.maximumPolynomialDegree;
    execution.maximumJoints = source.maximumJoints;
    execution.maximumSegments = source.maximumSegments;
    execution.timedEvents = source.timedEvents;
    execution.replacementBoundaries = source.replacementBoundaries;
    execution.holdResume = source.holdResume;
    execution.polynomialLimits = label(source.polynomialLimits);
    execution.samplingResolutionNs = resolution(source.polynomialLimits);
    result.execution = execution;
    var timing = new TimingCapabilitiesMsg();
    timing.deadlines = value.timing.deadlines;
    timing.clockMapping = value.timing.clockMapping;
    timing.prediction = label(value.timing.prediction);
    timing.samplingResolutionNs = resolution(value.timing.prediction);
    result.timing = timing;
    return result;
  }
  public static function decode(value:RobotCapabilities, id:robotkit.world.RobotId):robotkit.world.RobotCapabilities {
    if (value == null || value.controlModes == null || value.execution == null ||
        value.timing == null || value.streams == null) throw "Invalid robot capabilities";
    var modes:Array<robotkit.world.JointTargetMode> = [for (mode in value.controlModes) switch mode {
      case "position": Position; case "velocity": Velocity; case "effort": Effort; case "servo": Servo;
      case _: throw "Invalid robot capabilities";
    }];
    var e = value.execution;
    var t = value.timing;
    return new robotkit.world.RobotCapabilities(id, value.jointCount, modes,
      new robotkit.world.ExecutionCapabilities(e.plans, e.maximumPolynomialDegree, e.maximumJoints,
        e.maximumSegments, e.timedEvents, e.replacementBoundaries, e.holdResume,
        guarantee(e.polynomialLimits, e.samplingResolutionNs)),
      new robotkit.world.TimingCapabilities(t.deadlines, t.clockMapping,
        guarantee(t.prediction, t.samplingResolutionNs)), value.streams);
  }
  static function label(value:ValidationGuarantee):String return switch value {
    case Proven: "Proven"; case Sampled(_): "Sampled"; case Unchecked: "Unchecked"; case Failed: "Failed";
  };
  static function resolution(value:ValidationGuarantee):Int64 return switch value {
    case Sampled(ns): ns; case _: Int64.ofInt(0);
  };
  static function guarantee(label:String, ns:Int64):ValidationGuarantee return switch label {
    case "Proven": Proven;
    case "Sampled":
      if (Int64.compare(ns, Int64.ofInt(0)) <= 0) throw "Invalid robot capabilities";
      Sampled(ns);
    case "Unchecked": Unchecked; case "Failed": Failed;
    case _: throw "Invalid robot capabilities";
  };
}
