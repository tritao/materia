package robotkit.tool;

import haxe.Int64;

/** In-memory `SurfaceTool` that records every commanded state change with its timestamp. */
class SimulatedSurfaceTool implements SurfaceTool {
  public final history:Array<SurfaceToolEvent> = [];

  var enabledState = false;
  var standoffState = 0.0;

  public function new() {}

  public function enable(timestampNs:Int64):Void {
    enabledState = true;
    record(timestampNs);
  }

  public function disable(timestampNs:Int64):Void {
    enabledState = false;
    record(timestampNs);
  }

  public function isEnabled():Bool return enabledState;

  public function setStandoff(standoffMeters:Float, timestampNs:Int64):Void {
    if (!Math.isFinite(standoffMeters) || standoffMeters < 0.0)
      throw "Surface tool standoff must be finite and non-negative";
    standoffState = standoffMeters;
    record(timestampNs);
  }

  public function standoff():Float return standoffState;

  function record(timestampNs:Int64):Void
    history.push(new SurfaceToolEvent(enabledState, standoffState, timestampNs));
}

/** One recorded `SimulatedSurfaceTool` state change. */
class SurfaceToolEvent {
  public final enabled:Bool;
  public final standoff:Float;
  public final timestampNs:Int64;

  public function new(enabled:Bool, standoff:Float, timestampNs:Int64) {
    this.enabled = enabled;
    this.standoff = standoff;
    this.timestampNs = timestampNs;
  }
}
