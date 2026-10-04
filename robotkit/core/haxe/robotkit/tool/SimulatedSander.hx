package robotkit.tool;

import haxe.Int64;

/** In-memory `Sander` that records every commanded state change with its timestamp. */
class SimulatedSander implements Sander {
  public final history:Array<SanderEvent> = [];

  var speedState = 0.0;
  var contactForceState = 0.0;

  public function new() {}

  public function setSpeed(rpm:Float, timestampNs:Int64):Void {
    if (!Math.isFinite(rpm) || rpm < 0.0) throw "Sander speed must be finite and non-negative";
    speedState = rpm;
    record(timestampNs);
  }

  public function speed():Float return speedState;

  public function setContactForce(newtons:Float, timestampNs:Int64):Void {
    if (!Math.isFinite(newtons) || newtons < 0.0)
      throw "Sander contact force must be finite and non-negative";
    contactForceState = newtons;
    record(timestampNs);
  }

  public function contactForce():Float return contactForceState;

  function record(timestampNs:Int64):Void
    history.push(new SanderEvent(speedState, contactForceState, timestampNs));
}

/** One recorded `SimulatedSander` state change. */
class SanderEvent {
  public final speed:Float;
  public final contactForce:Float;
  public final timestampNs:Int64;

  public function new(speed:Float, contactForce:Float, timestampNs:Int64) {
    this.speed = speed;
    this.contactForce = contactForce;
    this.timestampNs = timestampNs;
  }
}
