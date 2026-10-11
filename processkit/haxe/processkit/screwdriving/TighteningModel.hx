package processkit.screwdriving;

import robotkit.tool.ScrewdriverContract;

/** Geometry and joint response are explicit simulation inputs, never inferred from a torque table. */
typedef TighteningJoint = {
  var leadMm:Float;
  var runDownMm:Float;
  var leadInMm:Float;
  var maximumAxisAngle:Float;
  var springRate:Float;
  var prevailingTorque:Float;
  var stiffnessNmPerRad:Float;
  var stripTorque:Float;
};
/** One sampled screw/axis state. Angle is unwrapped tightening rotation from cycle start. */
typedef TighteningInput = {
  var screwPresent:Bool;
  var vacuum:Bool;
  var start:Bool;
  var angle:Float;
  var strokeMm:Float;
  var radialErrorMm:Float;
  var axialErrorMm:Float;
  var axisAngle:Float;
  /** A damaged lead-in blocks advancement; its measured torque rises with rotation. */
  var damagedLeadIn:Bool;
};

/** Deterministic torque/angle process, independent of the arm and physics backend. */
class TighteningModel {
  public final program:TighteningProgram;
  public final joint:TighteningJoint;
  public var reading(default, null):ScrewdriverReading;
  public var advanceMm(default, null):Float = 0;
  var snugAngle = 0.0;
  var contactRunDownMm = 0.0;
  var engaged = false;
  var hadScrew = false;
  var started = false;
  var lastAngle = 0.0;
  var recontact = false;

  public function new(program:TighteningProgram, joint:TighteningJoint) {
    if (program == null || joint == null) throw "Tightening model needs a program and joint";
    for (value in [joint.leadMm, joint.leadInMm, joint.maximumAxisAngle, joint.springRate,
        joint.stiffnessNmPerRad, joint.stripTorque])
      if (!Math.isFinite(value) || value <= 0) throw "Tightening joint needs finite positive geometry and response";
    if (!Math.isFinite(joint.runDownMm) || joint.runDownMm < 0 || !Math.isFinite(joint.prevailingTorque) || joint.prevailingTorque < 0)
      throw "Tightening joint has invalid run-down or prevailing torque";
    this.program = program;
    this.joint = {leadMm: joint.leadMm, runDownMm: joint.runDownMm, leadInMm: joint.leadInMm,
      maximumAxisAngle: joint.maximumAxisAngle, springRate: joint.springRate, prevailingTorque: joint.prevailingTorque,
      stiffnessNmPerRad: joint.stiffnessNmPerRad, stripTorque: joint.stripTorque};
    reset();
  }
  public function reset():Void {
    contactRunDownMm = joint.runDownMm;
    advanceMm = 0; snugAngle = 0; lastAngle = 0; engaged = false; hadScrew = false; started = false;
    recontact = false;
    reading = ScrewdriverSensor.idle();
  }
  /** Continue a successfully seated joint without losing its calibrated travel or elastic rotation. */
  public function retighten(next:TighteningProgram, qualified:TighteningJoint):TighteningModel {
    if (reading.result != ScrewdriverContract.OK || !engaged || !started || advanceMm != contactRunDownMm)
      throw "Retightening needs a successfully seated screw";
    var resumed = new TighteningModel(next,qualified);
    var other = resumed.joint;
    if (joint.leadMm != other.leadMm || joint.runDownMm != other.runDownMm || joint.leadInMm != other.leadInMm ||
        joint.maximumAxisAngle != other.maximumAxisAngle || joint.springRate != other.springRate ||
        joint.prevailingTorque != other.prevailingTorque || joint.stiffnessNmPerRad != other.stiffnessNmPerRad ||
        joint.stripTorque != other.stripTorque)
      throw "Retightening cannot change the engaged joint's geometry or response";
    if (next.stopAtSeat) throw "Retightening needs a final program, not another head-seat stop";
    if (next.settings.targetTorque <= reading.torqueNm || next.settings.maximumRunDownAngle < snugAngle)
      throw "Retightening needs a higher target and the original run-down allowance";
    resumed.contactRunDownMm = contactRunDownMm;
    resumed.advanceMm = advanceMm; resumed.snugAngle = snugAngle; resumed.lastAngle = reading.angle;
    resumed.engaged = true; resumed.hadScrew = true; resumed.started = true; resumed.recontact = true;
    resumed.reading = {torqueNm:reading.torqueNm,angle:reading.angle,speed:0,strokeMm:0,
      phase:ScrewdriverContract.FINAL,result:ScrewdriverContract.NONE,vacuum:false};
    return resumed;
  }
  public function speed():Float return reading.result != ScrewdriverContract.NONE || !started ? 0 :
    (reading.phase == ScrewdriverContract.FINAL || reading.phase == ScrewdriverContract.SNUG) ? program.settings.finalSpeed : program.settings.runDownSpeed;

  /** `speed` is the requested rate; the simulation sensor replaces it with measured shaft speed. */
  public function step(input:TighteningInput):ScrewdriverReading {
    if (input == null) throw "Tightening model needs a sampled screw state";
    for (value in [input.angle, input.strokeMm, input.radialErrorMm, input.axialErrorMm, input.axisAngle])
      if (!Math.isFinite(value)) throw "Tightening sample must be finite";
    if (input.strokeMm < 0 || input.radialErrorMm < 0 || input.axisAngle < 0) throw "Tightening sample has negative geometry";
    reading.strokeMm = input.strokeMm; reading.vacuum = input.vacuum;
    reading.speed = 0;
    if (reading.result != ScrewdriverContract.NONE) return copy();
    if (!input.start) return copy();
    if (input.angle < lastAngle) throw "Tightening angle must be monotonic; reverse is a separate operation";
    var delta = input.angle - lastAngle; lastAngle = input.angle;
    reading.angle = input.angle;
    if (!started) {
      started = true;
      if (!input.screwPresent) return finish(ScrewdriverContract.NO_SCREW);
      hadScrew = true;
      reading.phase = ScrewdriverContract.ENGAGE;
    }
    if (hadScrew && (!input.screwPresent || !input.vacuum)) return finish(ScrewdriverContract.SCREW_LOST);
    var settings = program.settings;
    var expectedRunDownAngle = runDownAngle();
    var force = joint.springRate * input.strokeMm;
    var minimumForce = joint.springRate * settings.minimumStroke;
    var maximumForce = joint.springRate * settings.maximumStroke;
    if (engaged && (force < minimumForce - 1e-9 || force > maximumForce + 1e-9)) return finish(ScrewdriverContract.NO_START);
    if (recontact) {
      if (input.radialErrorMm > joint.leadInMm || input.axisAngle > joint.maximumAxisAngle)
        return finish(ScrewdriverContract.NO_START);
      recontact = false;
    }
    if (!engaged) {
      if (input.radialErrorMm > joint.leadInMm || Math.abs(input.axialErrorMm) > joint.leadInMm || input.axisAngle > joint.maximumAxisAngle || force < minimumForce - 1e-9 || force > maximumForce + 1e-9) {
        reading.torqueNm = 0;
        if (input.angle >= settings.maximumRunDownAngle) {
          reading.angle = settings.maximumRunDownAngle;
          return finish(ScrewdriverContract.NO_START);
        }
        reading.speed = settings.runDownSpeed; return copy();
      }
      if (input.damagedLeadIn) {
        var crossAngle = Math.max(0,(settings.runDownTorqueLimit-joint.prevailingTorque)/joint.stiffnessNmPerRad);
        if (crossAngle <= settings.maximumRunDownAngle && input.angle >= crossAngle) {
          reading.angle = crossAngle;
          reading.torqueNm = Math.max(joint.prevailingTorque,settings.runDownTorqueLimit);
          return finish(crossAngle < expectedRunDownAngle ? ScrewdriverContract.CROSS_THREAD : ScrewdriverContract.NO_START);
        }
        reading.angle = Math.min(input.angle,settings.maximumRunDownAngle);
        reading.torqueNm = joint.prevailingTorque + reading.angle * joint.stiffnessNmPerRad;
        if (input.angle >= settings.maximumRunDownAngle) return finish(ScrewdriverContract.NO_START);
        reading.speed = settings.runDownSpeed; return copy();
      }
      // A screw caught within the lead-in can start above or below the nominal
      // entry. Its seat stays fixed; that measured offset changes the travel to it.
      contactRunDownMm = Math.max(0,joint.runDownMm-input.axialErrorMm);
      engaged = true;
      expectedRunDownAngle = runDownAngle();
      // Rotation spent slipping before engagement cannot advance the screw.
      snugAngle = input.angle - delta + expectedRunDownAngle;
    }
    reading.phase = ScrewdriverContract.RUN_DOWN;
    reading.torqueNm = joint.prevailingTorque;
    if (reading.torqueNm > settings.runDownTorqueLimit) {
      reading.angle = snugAngle-expectedRunDownAngle;
      return finish(ScrewdriverContract.EARLY_HIGH_TORQUE);
    }
    var stopAngle = Math.min(input.angle,Math.min(snugAngle,settings.maximumRunDownAngle));
    advanceMm = stopAngle >= snugAngle ? contactRunDownMm :
      Math.max(0,(stopAngle-(snugAngle-expectedRunDownAngle))*joint.leadMm/(2*Math.PI));
    if (snugAngle > settings.maximumRunDownAngle && input.angle >= settings.maximumRunDownAngle) {
      reading.angle = settings.maximumRunDownAngle;
      return finish(ScrewdriverContract.RUN_DOWN_TOO_LONG);
    }
    if (input.angle < snugAngle) {
      reading.speed = settings.runDownSpeed; return copy();
    }
    if (program.stopAtSeat) {
      reading.angle = snugAngle;
      reading.phase = ScrewdriverContract.SNUG;
      return finish(ScrewdriverContract.OK);
    }
    var finalAngle = Math.max(0, input.angle - snugAngle);
    reading.phase = finalAngle == 0 ? ScrewdriverContract.SNUG : ScrewdriverContract.FINAL;
    var targetAngle = (settings.targetTorque - joint.prevailingTorque) / joint.stiffnessNmPerRad;
    var stripAngle = Math.max(0,(joint.stripTorque - joint.prevailingTorque) / joint.stiffnessNmPerRad);
    // Resolve the first threshold crossed, so a large sample cannot skip a window or a stop.
    if (stripAngle <= targetAngle && stripAngle <= settings.maximumAngle && finalAngle >= stripAngle) {
      reading.angle = snugAngle + stripAngle; reading.torqueNm = 0;
      return finish(ScrewdriverContract.STRIPPED);
    }
    if (targetAngle <= settings.maximumAngle && finalAngle >= targetAngle) {
      reading.angle = snugAngle + targetAngle; reading.torqueNm = settings.targetTorque;
      return finish(targetAngle < settings.minimumAngle ? ScrewdriverContract.ANGLE_OUT_OF_WINDOW : ScrewdriverContract.OK);
    }
    if (finalAngle >= settings.maximumAngle) {
      reading.angle = snugAngle + settings.maximumAngle;
      reading.torqueNm = joint.prevailingTorque + settings.maximumAngle*joint.stiffnessNmPerRad;
      return finish(ScrewdriverContract.ANGLE_OUT_OF_WINDOW);
    }
    reading.torqueNm = joint.prevailingTorque + finalAngle*joint.stiffnessNmPerRad;
    reading.speed = settings.finalSpeed; return copy();
  }
  /** Reversing first unloads the final elastic angle, then backs out by the thread lead. */
  public function advanceAtAngle(angle:Float):Float {
    if (!Math.isFinite(angle) || angle < 0) throw "Screw reversal needs a finite nonnegative angle";
    if (!engaged) return 0;
    var engagementAngle = snugAngle - runDownAngle();
    return Math.min(advanceMm,Math.max(0,(angle-engagementAngle)*joint.leadMm/(2*Math.PI)));
  }
  function runDownAngle():Float return contactRunDownMm*2*Math.PI/joint.leadMm;
  function finish(result:Int):ScrewdriverReading {
    reading.result = result; reading.phase = ScrewdriverContract.COMPLETE; reading.speed = 0; return copy();
  }
  function copy():ScrewdriverReading return {torqueNm: reading.torqueNm, angle: reading.angle, speed: reading.speed,
    strokeMm: reading.strokeMm, phase: reading.phase, result: reading.result, vacuum: reading.vacuum};
}
