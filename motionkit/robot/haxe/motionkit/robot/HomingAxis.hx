package motionkit.robot;

import robotkit.model.JointSwitch;

/** Physical limits and home switches for one independent coordinate, in SI units. */
class HomingAxis {
  public final id:String;
  public final joint:Int;
  public final switches:Array<JointSwitch>;
  public final acceleration:Float;
  public final seekSpeed:Float;
  public final latchSpeed:Float;
  public final releaseDistance:Float;
  public final maximumTravel:Float;
  public final home:Float;
  public final positionTolerance:Float;
  public final timestep:Float;

  public function new(id:String, joint:Int, switches:Array<JointSwitch>, velocity:Float,
      acceleration:Float, lower:Float, upper:Float, overtravel:Float, home:Float, timestep:Float) {
    if (id == null || id.length == 0 || joint < 0 || switches == null || switches.length == 0)
      throw "Homing axis requires an ID, joint and physical home switches";
    for (value in [velocity, acceleration, overtravel, timestep])
      if (!Math.isFinite(value) || value <= 0) throw "Homing drive limits and timing must be finite and positive";
    if (!Math.isFinite(lower) || !Math.isFinite(upper) || upper <= lower ||
        !Math.isFinite(home) || home < lower || home > upper) throw "Homing axis has invalid travel/home";
    this.id = id; this.joint = joint; this.switches = switches.copy();
    this.acceleration = acceleration; this.home = home;
    if (switches[0] == null) throw "Homing has a null home switch";
    this.timestep = timestep;
    var side = switches[0].side;
    var margin = overtravel, release = 0.0, precision = Math.POSITIVE_INFINITY;
    for (contact in switches) {
      if (contact == null || contact.role != "home" || contact.joint != switches[0].joint || contact.side != side)
        throw "One homing coordinate needs home switches on the same joint and side";
      var beforeStop = side < 0 ? contact.trip - (lower - overtravel) : upper + overtravel - contact.trip;
      margin = Math.min(margin, beforeStop - contact.repeatability);
      release = Math.max(release, contact.hysteresis + 2 * contact.repeatability);
      if (contact.repeatability > 0) precision = Math.min(precision, contact.repeatability);
    }
    if (!(margin > 0)) throw "Home switch leaves no braking distance before the end stop";
    if (!Math.isFinite(precision)) precision = 1e-6;
    seekSpeed = Math.min(velocity * 0.25, Math.sqrt(2 * acceleration * margin) * 0.5);
    latchSpeed = Math.min(seekSpeed * 0.1, precision / timestep * 0.25);
    releaseDistance = Math.max(release, latchSpeed * timestep * 4);
    maximumTravel = upper - lower + 2 * overtravel;
    if (!Math.isFinite(maximumTravel) || !Math.isFinite(seekSpeed) || seekSpeed <= 0 ||
        !Math.isFinite(latchSpeed) || latchSpeed <= 0) throw "Homing derived travel/speeds must be finite and positive";
    positionTolerance = precision;
  }
}
