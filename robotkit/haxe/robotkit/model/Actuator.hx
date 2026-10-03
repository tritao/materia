package robotkit.model;

import robotkit.model.ActuatorDrive;
import robotkit.model.Transmission;

class Actuator {
  public final id:String;
  /** Assumed engineering inputs carried from the model's source. */
  public var assumed:Array<String> = [];
  /** Driver facts are optional for models saved before amplifier parts. */
  public var microsteps:Null<Int>;
  public var maxStepRate:Null<Float>;
  /** Limits are in the actuator's effort and coordinate units, not joint units. */
  public var maxEffort:Float;
  public var maxRate:Float;
  public var transmission:Transmission;
  /**
   * Default servo gains in actuator units, such as an MJCF position
   * actuator's kp and kv: effort per unit of position and velocity error.
   * Zero means none is authored.
   */
  public var servoStiffness:Float = 0.0;
  public var servoDamping:Float = 0.0;
  /** Position-loop frequency supplied by the driver, Hz; zero when unspecified. */
  public var positionLoopRate:Float = 0.0;
  /**
   * What kind of motor this is and what it can deliver, or null for a bare effort and rate. For a
   * stepper or a servo, `maxEffort` and `maxRate` are the torque and speed a planner may rely
   * on (a stepper's usable share of its holding torque, a servo's peak torque and maximum speed),
   * and the drive carries the rest: the torque-speed curve the plan check compares against.
   */
  public var drive:Null<ActuatorDrive> = null;
  /**
   * The id of the `Encoder` that reads this motor, or empty for none. A servo's feedback comes from it
   * (the servo drive's own `encoderCounts` is what models saved before encoders were sensors recorded).
   */
  public var encoder:String = "";
  /**
   * Share of the motor's power its transmission delivers to the joint, such as a gearbox's 0.9; 1 is
   * lossless. The transmission's ratio is the gearbox ratio: the actuator turns `ratio` times for one turn
   * of the joint, so the joint gets the motor's torque times the ratio times this, at the motor's speed over
   * the ratio.
   */
  public var efficiency:Float = 1.0;
  /**
   * Full steps in one turn of a stepper motor's rotor, 0 when this is not a stepper. A stepper's
   * actuator coordinate is the rotor angle in radians; microstepping is a property of the driver
   * wiring, so the deployment adds it. Setting it makes the actuator a stepper known only by its
   * steps, unless it already is one.
   */
  public var fullStepsPerRevolution(get, set):Float;

  function get_fullStepsPerRevolution():Float {
    var current = drive;
    return current != null && Std.isOfType(current, StepperDrive) ? cast(current, StepperDrive).fullStepsPerRevolution : 0.0;
  }

  function set_fullStepsPerRevolution(value:Float):Float {
    var current = drive;
    if (value > 0.0) {
      if (current == null || !Std.isOfType(current, StepperDrive) ||
          cast(current, StepperDrive).fullStepsPerRevolution != value)
        drive = current != null && Std.isOfType(current, StepperDrive) ?
          new StepperDrive(value, current.rotorInertia, cast(current, StepperDrive).holdingTorque, current.curve) :
          StepperDrive.stepsOnly(value);
    } else if (current != null && Std.isOfType(current, StepperDrive)) drive = null;
    return value;
  }

  /**
   * The torque a planner may rely on at any speed. A stepper's `maxEffort` is its usable share of
   * holding torque; a servo's is its peak torque, and the drive's peak when none is set.
   */
  public function planningEffort():Float {
    var current = drive;
    if (current == null || !Std.isOfType(current, ServoDrive)) return maxEffort;
    return maxEffort > 0.0 ? Math.min(maxEffort, current.peakTorque()) : current.peakTorque();
  }

  /** The speed a planner may rely on, in actuator units: a servo falls back to its maximum speed. */
  public function planningRate():Float {
    var current = drive;
    if (current == null || !Std.isOfType(current, ServoDrive)) return maxRate;
    return maxRate > 0.0 ? Math.min(maxRate, current.maxSpeed()) : current.maxSpeed();
  }

  /** The torque-speed curve the plan check holds this actuator to: its drive's, or flat at its effort and rate. */
  public function torqueCurve():TorqueSpeedCurve {
    var current = drive;
    if (current != null && (!Std.isOfType(current, StepperDrive) || cast(current, StepperDrive).hasTorqueData()))
      return current.curve;
    return TorqueSpeedCurve.flat(maxEffort, maxRate > 0.0 ? maxRate : 1e9);
  }

  public function new(id:String, maxEffort:Float, maxRate:Float,
      transmission:Transmission) {
    if (id == null || StringTools.trim(id).length == 0)
      throw "Actuator ID must be non-empty";
    if (!Math.isFinite(maxEffort) || maxEffort < 0.0 ||
        !Math.isFinite(maxRate) || maxRate < 0.0)
      throw "Actuator limits must be finite and non-negative";
    if (transmission == null) throw "Actuator transmission is required";
    switch transmission {
      case SimpleTransmission(jointId, ratio, offset):
        if (jointId == null || StringTools.trim(jointId).length == 0 ||
            !Math.isFinite(ratio) || ratio == 0.0 || !Math.isFinite(offset))
          throw "Simple transmission needs a joint ID, nonzero finite ratio and finite offset";
    }
    this.id = id;
    this.maxEffort = maxEffort;
    this.maxRate = maxRate;
    this.transmission = transmission;
  }
}
