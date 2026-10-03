package robotkit.model;

import robotkit.model.EngineeringAssumptions.QuantityAssumption;

import robotkit.model.ActuatorDrive;
import robotkit.model.Transmission;

class Actuator {
  public var assumptions:Array<QuantityAssumption> = [];
  public final id:String;
  /** Assumed engineering inputs carried from the model's source. */
  public var assumed:Array<String> = [];
  /** Driver settings and input ceiling, required for stepper actuators. */
  public var microsteps:Null<Int>;
  public var maxStepRate:Null<Float>;
  /** The bound controller, when its clock further limits the driver. */
  public var speedLimiter:String = "";
  /** Limits are in the actuator's effort and coordinate units, not joint units. */
  public var maxEffort:Null<Float>;
  public var maxRate:Null<Float>;
  public var transmission:Transmission;
  /**
   * Default servo gains in actuator units, such as an MJCF position
   * actuator's kp and kv: effort per unit of position and velocity error.
   * Zero means none is authored.
   */
  public var servoStiffness:Float = 0.0;
  public var servoDamping:Float = 0.0;
  /**
   * What kind of motor this is and what it can deliver, or null for a bare effort and rate. For a
   * stepper or a servo, `maxEffort` and `maxRate` are the torque and speed a planner may rely
   * on (a stepper's usable share of its holding torque, a servo's peak torque and maximum speed),
   * and the drive carries the rest: the torque-speed curve the plan check compares against.
   */
  public var drive:Null<ActuatorDrive> = null;
  /**
   * The id of the `Encoder` that reads this motor, or empty for none. A servo's feedback comes from it
   * (the drive also states its feedback resolution).
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
  public function planningEffort():Null<Float> {
    var current = drive, effort = maxEffort;
    if (current == null || !Std.isOfType(current, ServoDrive)) return effort;
    return effort == null ? current.peakTorque() : Math.min(effort, current.peakTorque());
  }

  /** Missing motor speed stays missing until a stated drive or driver supplies a cap. */
  public function planningRate():Null<Float> {
    var current = drive, rate = maxRate;
    if (current != null && Std.isOfType(current, ServoDrive))
      rate = rate == null ? current.maxSpeed() : Math.min(rate, current.maxSpeed());
    var setting = microsteps, inputRate = maxStepRate;
    if (fullStepsPerRevolution > 0 && setting != null && inputRate != null) {
      var ceiling = inputRate * 2 * Math.PI / (fullStepsPerRevolution * setting);
      rate = rate == null ? ceiling : Math.min(rate, ceiling);
    }
    return rate;
  }

  public function requireRate():Float {
    var value = planningRate();
    if (value == null) throw 'Actuator "$id" has no planning speed';
    return value;
  }

  public function requireEffort():Float {
    var value = planningEffort();
    if (value == null) throw 'Actuator "$id" has no planning effort';
    return value;
  }

  /** The hardware ceiling that is active at the motor's planning rate. */
  public function rateLimiter():String {
    if (speedLimiter != "") return speedLimiter;
    var input = maxStepRate, setting = microsteps;
    if (input == null || setting == null || fullStepsPerRevolution <= 0) return "";
    var ceiling = input * 2 * Math.PI / (fullStepsPerRevolution * setting);
    var current = drive, motorRate = maxRate;
    if (current != null && Std.isOfType(current, ServoDrive))
      motorRate = motorRate == null ? current.maxSpeed() : Math.min(motorRate, current.maxSpeed());
    return motorRate == null || ceiling <= motorRate ? "driver step input" : "";
  }

  /** The torque-speed curve the plan check holds this actuator to: its drive's, or flat at its effort and rate. */
  public function torqueCurve():TorqueSpeedCurve {
    var current = drive;
    if (current != null && (!Std.isOfType(current, StepperDrive) || cast(current, StepperDrive).hasTorqueData()))
      return current.curve;
    return TorqueSpeedCurve.flat(requireEffort(), requireRate());
  }

  public function new(id:String, maxEffort:Null<Float>, maxRate:Null<Float>,
      transmission:Transmission) {
    if (id == null || StringTools.trim(id).length == 0)
      throw "Actuator ID must be non-empty";
    if ((maxEffort != null && (!Math.isFinite(maxEffort) || maxEffort < 0.0)) ||
        (maxRate != null && (!Math.isFinite(maxRate) || maxRate < 0.0)))
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
