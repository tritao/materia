package robotkit.runtime;

import robotkit.model.Actuator;
import robotkit.model.ActuatorDrive.StepperDrive;
import robotkit.model.Encoder;
import robotkit.model.EncoderReading;
import robotkit.model.RobotModel;
import robotkit.model.Transmission;

/** What an encoder says is wrong: a motor that is not where it was told to be. */
enum abstract EncoderFindingKind(String) from String to String {
  /** A stepper's encoder is further from the commanded position than its bound: the motor lost steps. */
  var LostSteps = "lost-steps";
  /** A servo's (or any other motor's) encoder is further from the command than its bound. */
  var FollowingError = "following-error";
}

/** One fault an encoder found, latched at the first observation past the bound. */
class EncoderFinding {
  public final kind:EncoderFindingKind;
  public final encoder:String;
  /** The joint the encoder reads. */
  public final joint:String;
  /** The motor's actuator, or empty when no motor is on the joint. */
  public final motor:String;
  /** Encoder position minus commanded position, in joint units. */
  public final error:Float;
  public final bound:Float;
  /** Full steps the error stands for when the motor is a stepper, else 0. */
  public final steps:Float;
  public final timeSeconds:Float;

  public function new(kind:EncoderFindingKind, encoder:String, joint:String, motor:String, error:Float, bound:Float, steps:Float,
      timeSeconds:Float) {
    this.kind = kind;
    this.encoder = encoder;
    this.joint = joint;
    this.motor = motor;
    this.error = error;
    this.bound = bound;
    this.steps = steps;
    this.timeSeconds = timeSeconds;
  }

  public function describe():String {
    var where = 'encoder $encoder on $joint (${Math.round(timeSeconds * 1000.0) / 1000.0} s)';
    return kind == EncoderFindingKind.LostSteps
      ? '$where: motor $motor lost about ${Math.round(steps * 10.0) / 10.0} steps (${format(error)} off its command, bound ${format(bound)})'
      : '$where: ${motor == "" ? "joint" : "motor " + motor} is ${format(error)} off its command, bound ${format(bound)}';
  }

  static function format(value:Float):String return Std.string(Math.round(value * 1e6) / 1e6);

  public function toString():String return '$kind: ' + describe();
}

/**
 * Reads a robot's encoders from its measured joint positions and compares them with where the joints were
 * commanded to be. Monitoring only: it names what it finds and never commands anything.
 *
 * - A **motor-side** encoder (one on a joint a motor drives directly) sees the rotor, so it sees what
 *   the command hides: a stepper's lost steps, a servo's following error. Past `bound` it latches a
 *   finding, `LostSteps` for a stepper and `FollowingError` otherwise. It does not see backlash or
 *   belt stretch, which sit between the rotor and the load.
 * - A **load-side** encoder (a linear scale, an encoder on a driven pulley) sees where the load is, so
 *   its error against the command is the **path error** the machine really has, the observed
 *   counterpart of the plan check's accuracy prediction (`pathError`). It faults nothing.
 *
 * `bounds` gives a joint's bound in joint units (the runtime blueprint's `following_error_bound`);
 * where it has none or zero, a stepper's is `STEPPER_BOUND_STEPS` full steps of its rotor and anything
 * else's is `DEFAULT_BOUND_COUNTS` encoder counts, both assumptions. A device that counts edges in
 * hardware feeds `EncoderReading` its own counts and calls `evaluate`.
 */
class EncoderMonitor {
  /** A stepper may be this many full steps from its command before it is called lost. */
  public static inline final STEPPER_BOUND_STEPS = 2.0;
  /** Counts a non-stepper motor may be from its command. */
  public static inline final DEFAULT_BOUND_COUNTS = 8.0;

  public final model:RobotModel;
  public final readings:Array<EncoderReading> = [];
  /** Findings in the order they were made, one per encoder. */
  public final findings:Array<EncoderFinding> = [];
  final jointIndex:Array<Int> = [];
  final motors:Array<Null<Actuator>> = [];
  final bound:Array<Float> = [];
  final lastError:Array<Float> = [];
  final peakError:Array<Float> = [];
  final squares:Array<Float> = [];
  final samples:Array<Int> = [];
  final faulted:Array<Bool> = [];

  /** `positions` are the joints' positions the encoders power up at; `bounds` maps joint ids to bounds in joint units. */
  public function new(model:RobotModel, positions:Array<Float>, ?bounds:Map<String, Float>) {
    this.model = model;
    for (encoder in model.encoders) {
      var index = -1;
      for (joint in 0...model.joints.length) if (model.joints[joint].id == encoder.joint) index = joint;
      if (index < 0 || index >= positions.length) throw 'Encoder ${encoder.id} reads a joint the robot does not have';
      jointIndex.push(index);
      readings.push(new EncoderReading(encoder, positions[index]));
      var motor = motorOn(encoder.joint);
      motors.push(motor);
      var given = bounds == null ? null : bounds.get(encoder.joint);
      bound.push(given != null && given > 0.0 ? given : defaultBound(encoder, motor));
      lastError.push(0.0);
      peakError.push(0.0);
      squares.push(0.0);
      samples.push(0);
      faulted.push(false);
    }
  }

  /** The actuator that drives `joint` directly, if one does. */
  function motorOn(joint:String):Null<Actuator> {
    for (actuator in model.actuators) switch actuator.transmission {
      case SimpleTransmission(jointId, _, _): if (jointId == joint) return actuator;
    }
    return null;
  }

  function defaultBound(encoder:Encoder, motor:Null<Actuator>):Float {
    if (motor != null) {
      var drive = motor.drive;
      if (drive != null && Std.isOfType(drive, StepperDrive)) {
        // A step of the rotor is 2 pi / full steps radians of the actuator, which the transmission scales to the joint.
        var ratio = switch motor.transmission { case SimpleTransmission(_, ratio, _): Math.abs(ratio); }
        return STEPPER_BOUND_STEPS * 2.0 * Math.PI / cast(drive, StepperDrive).fullStepsPerRevolution / ratio;
      }
    }
    return DEFAULT_BOUND_COUNTS * encoder.resolution();
  }

  /** Whether the encoder is on a joint a motor drives (motor-side) rather than a load joint. */
  public function motorSide(index:Int):Bool return motors[index] != null;

  /** Reads every encoder from the measured joint positions and checks it against the commanded ones. */
  public function observe(positions:Array<Float>, commanded:Array<Float>, timeSeconds:Float):Void {
    for (index in 0...readings.length) readings[index].sample(positions[jointIndex[index]]);
    evaluate(commanded, timeSeconds);
  }

  /** Checks the readings as they are, such as counts a device reported, against the commanded joint positions. */
  public function evaluate(commanded:Array<Float>, timeSeconds:Float):Void {
    for (index in 0...readings.length) {
      var error = readings[index].position() - commanded[jointIndex[index]];
      lastError[index] = error;
      peakError[index] = Math.max(peakError[index], Math.abs(error));
      squares[index] += error * error;
      samples[index]++;
      var motor = motors[index];
      if (motor == null || faulted[index] || Math.abs(error) <= bound[index]) continue;
      faulted[index] = true;
      var encoder = readings[index].encoder;
      var stepper = motor.drive != null && Std.isOfType(motor.drive, StepperDrive);
      var steps = 0.0;
      if (stepper) {
        var ratio = switch motor.transmission { case SimpleTransmission(_, ratio, _): Math.abs(ratio); }
        steps = Math.abs(error) * ratio / (2.0 * Math.PI / cast(motor.drive, StepperDrive).fullStepsPerRevolution);
      }
      findings.push(new EncoderFinding(stepper ? EncoderFindingKind.LostSteps : EncoderFindingKind.FollowingError, encoder.id,
        encoder.joint, motor.id, error, bound[index], steps, timeSeconds));
    }
  }

  /** Whether any motor-side encoder has faulted. */
  public function faults():Bool return findings.length > 0;

  /** The path error a load-side (or any) encoder has measured so far: its latest, worst and RMS error against the command, in joint units. */
  public function pathError(encoderId:String):{last:Float, peak:Float, rms:Float} {
    for (index in 0...readings.length) if (readings[index].encoder.id == encoderId)
      return {last: lastError[index], peak: peakError[index], rms: samples[index] == 0 ? 0.0 : Math.sqrt(squares[index] / samples[index])};
    throw 'Unknown encoder $encoderId';
  }

  /** Forgets what was measured and the faults, and powers the encoders up again at `positions`, as a homing does. */
  public function reset(positions:Array<Float>):Void {
    while (findings.length > 0) findings.pop();
    for (index in 0...readings.length) {
      readings[index] = new EncoderReading(readings[index].encoder, positions[jointIndex[index]]);
      lastError[index] = 0.0;
      peakError[index] = 0.0;
      squares[index] = 0.0;
      samples[index] = 0;
      faulted[index] = false;
    }
  }
}
