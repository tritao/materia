package robotkit.model;

/**
 * What kind of motor an actuator is, and what it can deliver. Steppers and servos fail
 * differently: a stepper that is asked for more torque than its pull-out curve gives loses steps,
 * a servo that is asked for more than its peak torque falls behind or trips. Drive-level
 * behaviour only; current loops and thermal mass are left out.
 */
class ActuatorDrive {
  /** Inertia of the rotor, kg m². */
  public final rotorInertia:Float;
  /** Torque against speed, in the actuator's coordinate units: the limit the plan check works to. */
  public final curve:TorqueSpeedCurve;

  function new(rotorInertia:Float, curve:TorqueSpeedCurve) {
    if (!Math.isFinite(rotorInertia) || rotorInertia < 0.0) throw "Rotor inertia must be finite and non-negative";
    if (curve == null) throw "A drive needs a torque-speed curve";
    this.rotorInertia = rotorInertia;
    this.curve = curve;
  }

  /** The name saved with the drive: "stepper" or "servo". */
  public function kind():String throw "ActuatorDrive is abstract";

  /** The largest torque the drive can give at any speed. */
  public function peakTorque():Float return curve.peakTorque();

  public function maxSpeed():Float return curve.maxSpeed();
}

/**
 * A stepper: full steps in one turn, holding torque and a pull-out curve of the torque it keeps
 * while stepping at each speed. Microstepping is a property of the driver wiring, so the
 * deployment adds it.
 */
class StepperDrive extends ActuatorDrive {
  public final fullStepsPerRevolution:Float;
  public final holdingTorque:Float;

  public function new(fullStepsPerRevolution:Float, rotorInertia:Float, holdingTorque:Float,
      pullOut:TorqueSpeedCurve) {
    super(rotorInertia, pullOut);
    if (!Math.isFinite(fullStepsPerRevolution) || fullStepsPerRevolution <= 0.0)
      throw "A stepper needs a positive number of full steps per revolution";
    if (!Math.isFinite(holdingTorque) || holdingTorque < 0.0) throw "Holding torque must be finite and non-negative";
    this.fullStepsPerRevolution = fullStepsPerRevolution;
    this.holdingTorque = holdingTorque;
  }

  /** A stepper known only by its steps, as in models saved before ratings were recorded. */
  public static function stepsOnly(fullStepsPerRevolution:Float):StepperDrive
    return new StepperDrive(fullStepsPerRevolution, 0.0, 0.0, TorqueSpeedCurve.flat(0.0, 1.0));

  /** Whether this drive knows its torque, or only its steps. */
  public function hasTorqueData():Bool return holdingTorque > 0.0;

  override public function kind():String return "stepper";
}

/**
 * A servo: rated (continuous) and peak torque, rated and maximum speed, rotor inertia, encoder
 * counts per revolution and its torque-speed envelope. Its gains, when authored, are the
 * actuator's `servoStiffness` and `servoDamping`.
 */
class ServoDrive extends ActuatorDrive {
  public final ratedTorque:Float;
  public final peakTorqueValue:Float;
  public final ratedSpeed:Float;
  public final maxSpeedValue:Float;
  public final encoderCounts:Float;

  public function new(ratedTorque:Float, peakTorque:Float, ratedSpeed:Float, maxSpeed:Float,
      rotorInertia:Float, encoderCounts:Float, ?envelope:TorqueSpeedCurve) {
    super(rotorInertia, envelope == null ? ServoDrive.defaultEnvelope(ratedTorque, peakTorque, ratedSpeed, maxSpeed) : envelope);
    if (!(ratedTorque > 0.0) || !(peakTorque >= ratedTorque) || !(ratedSpeed > 0.0) || !(maxSpeed >= ratedSpeed) ||
        !Math.isFinite(peakTorque) || !Math.isFinite(maxSpeed))
      throw "A servo needs 0 < rated torque <= peak torque and 0 < rated speed <= max speed";
    if (!Math.isFinite(encoderCounts) || encoderCounts < 0.0) throw "Encoder counts must be finite and non-negative";
    this.ratedTorque = ratedTorque;
    this.peakTorqueValue = peakTorque;
    this.ratedSpeed = ratedSpeed;
    this.maxSpeedValue = maxSpeed;
    this.encoderCounts = encoderCounts;
  }

  /**
   * Peak torque up to the rated speed, falling in a straight line to the rated torque at the
   * maximum speed: the usual shape of a servo's intermittent-duty envelope.
   */
  public static function defaultEnvelope(ratedTorque:Float, peakTorque:Float, ratedSpeed:Float,
      maxSpeed:Float):TorqueSpeedCurve {
    if (maxSpeed <= ratedSpeed) return TorqueSpeedCurve.flat(peakTorque, maxSpeed);
    return new TorqueSpeedCurve([0.0, ratedSpeed, maxSpeed], [peakTorque, peakTorque, ratedTorque]);
  }

  /**
   * Stiffness a simulation gives this servo when none is authored: its peak torque for a hundredth
   * of a radian of error, N m per rad. An assumption for a stiff positioning servo, not a datasheet value.
   */
  public function defaultStiffness():Float return peakTorqueValue / 0.01;

  /** Damping that goes with `stiffness` when none is authored: a 10 ms damping time, an assumption. */
  public static function defaultDamping(stiffness:Float):Float return stiffness * 0.01;

  override public function kind():String return "servo";
  override public function peakTorque():Float return peakTorqueValue;
  override public function maxSpeed():Float return maxSpeedValue;
}
