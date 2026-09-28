package robotkit.tool;

import haxe.Int64;

/** In-memory vacuum pickup driven by measured vacuum magnitude. */
class SimulatedVacuum implements Vacuum {
  public final history:Array<VacuumEvent> = [];
  public final observations:Array<VacuumObservation> = [];
  public final holdThresholdKpa:Float;

  var enabledState = false;
  var holdingState = false;
  var measuredVacuumKpa = 0.0;

  public function new(holdThresholdKpa:Float = 40.0) {
    if (!Math.isFinite(holdThresholdKpa) || holdThresholdKpa <= 0.0)
      throw "Vacuum hold threshold must be finite and positive";
    this.holdThresholdKpa = holdThresholdKpa;
  }

  public function enable(timestampNs:Int64):Void {
    enabledState = true;
    holdingState = false;
    measuredVacuumKpa = 0.0;
    record(timestampNs);
  }

  public function disable(timestampNs:Int64):Void {
    enabledState = false;
    holdingState = false;
    measuredVacuumKpa = 0.0;
    record(timestampNs);
  }

  /** Report measured vacuum magnitude in kPa below ambient pressure. */
  public function observeVacuumKpa(value:Float, timestampNs:Int64):Void {
    if (!Math.isFinite(value) || value < 0.0)
      throw "Measured vacuum must be finite and non-negative";
    measuredVacuumKpa = value;
    holdingState = enabledState && value >= holdThresholdKpa;
    observations.push(new VacuumObservation(value, holdingState, timestampNs));
  }

  public function isEnabled():Bool return enabledState;
  public function isHolding():Bool return holdingState;
  public function vacuumKpa():Float return measuredVacuumKpa;

  function record(timestampNs:Int64):Void
    history.push(new VacuumEvent(enabledState, holdingState, timestampNs));
}

/** One pressure observation and the resulting holding state. */
class VacuumObservation {
  public final vacuumKpa:Float;
  public final holding:Bool;
  public final timestampNs:Int64;

  public function new(vacuumKpa:Float, holding:Bool, timestampNs:Int64) {
    this.vacuumKpa = vacuumKpa;
    this.holding = holding;
    this.timestampNs = timestampNs;
  }
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
