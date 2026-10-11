package processkit.screwdriving;

/** Explicit joint qualification; speeds are rad/s, angles rad, torque Nm, stroke mm. */
typedef TighteningSettings = {
  var runDownSpeed:Float;
  var finalSpeed:Float;
  var targetTorque:Float;
  var minimumAngle:Float;
  var maximumAngle:Float;
  var runDownTorqueLimit:Float;
  var finalTorqueLimit:Float;
  var maximumRunDownAngle:Float;
  var minimumStroke:Float;
  var maximumStroke:Float;
};

/** A qualified program. A torque table alone cannot supply its angle or stroke windows. */
class TighteningProgram {
  public final settings:TighteningSettings;
  public function new(settings:TighteningSettings) {
    if (settings == null) throw "Tightening program needs qualified settings";
    for (value in [settings.runDownSpeed, settings.finalSpeed, settings.targetTorque,
        settings.maximumAngle, settings.runDownTorqueLimit, settings.finalTorqueLimit,
        settings.maximumRunDownAngle, settings.maximumStroke])
      if (!Math.isFinite(value) || value <= 0) throw "Tightening program needs finite positive settings";
    if (!Math.isFinite(settings.minimumAngle) || settings.minimumAngle < 0 ||
        !Math.isFinite(settings.minimumStroke) || settings.minimumStroke < 0) throw "Tightening minima must be finite and nonnegative";
    if (settings.minimumAngle >= settings.maximumAngle || settings.minimumStroke > settings.maximumStroke ||
        settings.runDownTorqueLimit >= settings.targetTorque || settings.finalTorqueLimit < settings.targetTorque)
      throw "Tightening program has inconsistent windows or torque limits";
    this.settings = {runDownSpeed: settings.runDownSpeed, finalSpeed: settings.finalSpeed,
      targetTorque: settings.targetTorque, minimumAngle: settings.minimumAngle, maximumAngle: settings.maximumAngle,
      runDownTorqueLimit: settings.runDownTorqueLimit, finalTorqueLimit: settings.finalTorqueLimit,
      maximumRunDownAngle: settings.maximumRunDownAngle, minimumStroke: settings.minimumStroke, maximumStroke: settings.maximumStroke};
  }
}
