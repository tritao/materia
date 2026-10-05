package robotkit.model;

/**
 * What powers an actuator, and what it can deliver. Steppers and servos fail
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

  /** The name saved with the drive. */
  public function kind():String throw "ActuatorDrive is abstract";

  /** The largest torque the drive can give at any speed. */
  public function peakTorque():Float return curve.peakTorque();

  public function maxSpeed():Float return curve.maxSpeed();

  /** Stepper fields when this drive has steps; null for other drives. */
  public function stepper():Null<StepperDrive> return null;

  /** Fluid cylinder fields when this drive belongs to a process joint. */
  public function cylinder():Null<CylinderDrive> return null;

  /** Hydraulic phase fields when this drive uses flow control. */
  public function hydraulic():Null<HydraulicDrive> return null;

  /** Pressure-effort pneumatic fields; null for other drives. */
  public function pneumatic():Null<PneumaticDrive> return null;

  /** Servo fields when this drive has a position loop; null for other drives. */
  public function servo():Null<ServoDrive> return null;
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

  /** A stepper with a stated step count and no torque ratings. */
  public static function stepsOnly(fullStepsPerRevolution:Float):StepperDrive
    return new StepperDrive(fullStepsPerRevolution, 0.0, 0.0, TorqueSpeedCurve.flat(0.0, 1.0));

  /** Whether this drive knows its torque, or only its steps. */
  public function hasTorqueData():Bool return holdingTorque > 0.0;

  override public function kind():String return "stepper";
  override public function stepper():Null<StepperDrive> return this;
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
  override public function servo():Null<ServoDrive> return this;
}

/** Shared double-acting cylinder dimensions, pressure forces and process channels in SI units. */
class CylinderDrive extends ActuatorDrive {
  public final bore:Float;
  public final rod:Float;
  public final stroke:Float;
  public final ratedSpeed:Float;
  public final pressurePa:Float;
  public final channelA:String;
  public final channelB:Null<String>;
  public final normallyToA:Bool;
  public final extendSign:Float;

  function new(bore:Float, rod:Float, stroke:Float, ratedSpeed:Float, pressurePa:Float,
      channelA:String, channelB:Null<String>, normallyToA:Bool, extendSign:Float) {
    super(0.0, TorqueSpeedCurve.flat(0.0, ratedSpeed));
    if (!(bore > rod && rod > 0 && stroke > 0 && ratedSpeed > 0 && pressurePa >= 0) ||
        !Math.isFinite(bore + rod + stroke + ratedSpeed + pressurePa) ||
        channelA == null || channelA.length == 0 || (channelB != null && channelB.length == 0) ||
        (extendSign != 1.0 && extendSign != -1.0))
      throw "Invalid cylinder drive";
    this.bore = bore; this.rod = rod; this.stroke = stroke; this.ratedSpeed = ratedSpeed;
    this.pressurePa = pressurePa; this.channelA = channelA; this.channelB = channelB;
    this.normallyToA = normallyToA; this.extendSign = extendSign;
    if (!Math.isFinite(extensionForce() + retractionForce())) throw "Cylinder pressure-area force overflows";
  }

  public function extensionForce():Float return pressurePa * Math.PI * bore * bore / 4.0;
  public function retractionForce():Float return pressurePa * Math.PI * (bore * bore - rod * rod) / 4.0;
  public function extensionSpeed(extension:Float):Float return ratedSpeed;
  public function returningSpeed():Float return ratedSpeed;
  override public function cylinder():Null<CylinderDrive> return this;
  override public function peakTorque():Float return extensionForce();
  override public function maxSpeed():Float return ratedSpeed;
}

/** Pressure effort with a spring-return or latched directional valve. */
class PneumaticDrive extends CylinderDrive {
  public function new(bore:Float, rod:Float, stroke:Float, ratedSpeed:Float, pressurePa:Float,
      channelA:String, channelB:Null<String>, normallyToA:Bool, extendSign:Float) {
    super(bore, rod, stroke, ratedSpeed, pressurePa, channelA, channelB, normallyToA, extendSign);
  }
  /** A critically damped end stop: static deflection uses one quarter of the cushion's allowance. */
  public function limitTimeConstant(movingMass:Float, cushionCompression:Float):Float {
    if (!(movingMass > 0.0) || !(cushionCompression > 0.0) || !(extensionForce() > 0.0) ||
        !Math.isFinite(movingMass + cushionCompression))
      throw "Pneumatic limit needs positive mass and cushion compression";
    return Math.sqrt(movingMass * cushionCompression / extensionForce()) / 2.0;
  }
  override public function pneumatic():Null<PneumaticDrive> return this;
  override public function kind():String return "pneumatic";
}

/** Flow-limited hydraulic cylinder. Coincident coil states close the valve centre. */
class HydraulicDrive extends CylinderDrive {
  public final approachSpeed:Float;
  public final workingSpeed:Float;
  public final returnSpeed:Float;
  public final changePoint:Float;
  /** Joint coordinate at which the cylinder is retracted, in metres. */
  public final retractedPosition:Float;

  public function new(bore:Float, rod:Float, stroke:Float, pressurePa:Float,
      approachSpeed:Float, workingSpeed:Float, returnSpeed:Float, changePoint:Float,
      channelA:String, channelB:String, extendSign:Float, retractedPosition:Float = 0.0) {
    super(bore, rod, stroke, Math.max(approachSpeed, Math.max(workingSpeed, returnSpeed)),
      pressurePa, channelA, channelB, false, extendSign);
    if (!(approachSpeed > 0.0 && workingSpeed > 0.0 && returnSpeed > 0.0) ||
        changePoint < 0.0 || changePoint > stroke || channelB == null || channelA == channelB ||
        !Math.isFinite(approachSpeed + workingSpeed + returnSpeed + changePoint + retractedPosition))
      throw "Invalid hydraulic phase speeds, change point or valve channels";
    this.approachSpeed = approachSpeed; this.workingSpeed = workingSpeed;
    this.returnSpeed = returnSpeed; this.changePoint = changePoint;
    this.retractedPosition = retractedPosition;
  }
  override public function extensionSpeed(extension:Float):Float {
    if (!Math.isFinite(extension)) throw "Hydraulic extension must be finite";
    return extension < changePoint ? approachSpeed : workingSpeed;
  }
  override public function returningSpeed():Float return returnSpeed;
  override public function hydraulic():Null<HydraulicDrive> return this;
  override public function kind():String return "hydraulic";
}
