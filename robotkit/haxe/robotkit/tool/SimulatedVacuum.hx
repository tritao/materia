package robotkit.tool;

import haxe.Int64;

/** In-memory vacuum pickup. Without contact physics, enabling assumes a pickup. */
class SimulatedVacuum implements Vacuum {
  public final history:Array<VacuumEvent> = [];

  var enabledState = false;
  var holdingState = false;

  public function new() {}

  public function enable(timestampNs:Int64):Void {
    enabledState = true;
    holdingState = true;
    record(timestampNs);
  }

  public function disable(timestampNs:Int64):Void {
    enabledState = false;
    holdingState = false;
    record(timestampNs);
  }

  public function isEnabled():Bool return enabledState;
  public function isHolding():Bool return holdingState;

  function record(timestampNs:Int64):Void
    history.push(new VacuumEvent(enabledState, holdingState, timestampNs));
}

/** One recorded vacuum state change. */
class VacuumEvent {
  public final enabled:Bool;
  public final holding:Bool;
  public final timestampNs:Int64;

  public function new(enabled:Bool, holding:Bool, timestampNs:Int64) {
    this.enabled = enabled;
    this.holding = holding;
    this.timestampNs = timestampNs;
  }
}
