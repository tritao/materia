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
  public final backoffSpeed:Float;
  public final capturedApproachSpeed:Float;
  public final dynamics:HomingDynamics;
  public final releaseSearchDistance:Float;
  public final releaseDistance:Float;
  public final maximumTravel:Float;
  public final home:Float;
  public final positionTolerance:Float;
  public final timestep:Float;
  public final lowerTravel:Float;
  public final upperTravel:Float;

  public function new(id:String, joint:Int, switches:Array<JointSwitch>, velocity:Float,
      acceleration:Float, lower:Float, upper:Float, overtravel:Float, home:Float, timestep:Float, ?dynamics:HomingDynamics, ?maximumCaptureOverrun:Float) {
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
    lowerTravel = lower - overtravel; upperTravel = upper + overtravel;
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
    // Two owner periods for command admission and one for a fresh switch observation.
    // The jerk limit is an explicit planning policy until a drive supplies a tighter limit.
    this.dynamics = dynamics == null ? new HomingDynamics(acceleration, acceleration / timestep, 3 * timestep, timestep) : dynamics;
    if (this.dynamics.acceleration > acceleration || this.dynamics.timestep != timestep)
      throw "Homing dynamics must respect the physical drive and owner period";
    seekSpeed = this.dynamics.speedForRoom(margin, velocity);
    latchSpeed = Math.min(seekSpeed, precision / timestep * 0.25);
    releaseDistance = Math.max(release, precision * 4);
    // Finite backoff plans decelerate at their endpoint; they need no release-triggered stop.
    backoffSpeed = seekSpeed;
    // One coordinate may use the available end-stop room. Independent side holds
    // additionally need an authored racking budget; otherwise retain sampled speed.
    if (maximumCaptureOverrun != null && (!Math.isFinite(maximumCaptureOverrun) || maximumCaptureOverrun <= 0))
      throw "Captured-edge overrun budget must be finite and positive";
    var captureRoom = maximumCaptureOverrun == null ? margin : Math.min(margin, maximumCaptureOverrun - 2 * precision);
    capturedApproachSpeed = switches.length > 1 && maximumCaptureOverrun == null ? latchSpeed :
      captureRoom > 0 ? this.dynamics.speedForRoom(captureRoom, seekSpeed) : latchSpeed;
    // A sampled closing observation may already be past the switch by one owner interval.
    releaseSearchDistance = releaseDistance + release + seekSpeed * timestep;
    maximumTravel = upper - lower + 2 * overtravel;
    if (!Math.isFinite(maximumTravel) || !Math.isFinite(seekSpeed) || seekSpeed <= 0 ||
        !Math.isFinite(latchSpeed) || latchSpeed <= 0 ||
        !Math.isFinite(backoffSpeed) || backoffSpeed <= 0 ||
        !Math.isFinite(capturedApproachSpeed) || capturedApproachSpeed <= 0) throw "Homing derived travel/speeds must be finite and positive";
    positionTolerance = precision;
  }
}
