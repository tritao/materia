package robotkit.tool;

import haxe.Int64;

/** In-memory `Sprayer` that records every commanded state change with its timestamp. */
class SimulatedSprayer implements Sprayer {
  public final history:Array<SprayerEvent> = [];

  var flowState = 0.0;
  var pressureState = 0.0;

  public function new() {}

  public function setFlow(litersPerMinute:Float, timestampNs:Int64):Void {
    if (!Math.isFinite(litersPerMinute) || litersPerMinute < 0.0)
      throw "Sprayer flow must be finite and non-negative";
    flowState = litersPerMinute;
    record(timestampNs);
  }

  public function flow():Float return flowState;

  public function setPressure(bar:Float, timestampNs:Int64):Void {
    if (!Math.isFinite(bar) || bar < 0.0) throw "Sprayer pressure must be finite and non-negative";
    pressureState = bar;
    record(timestampNs);
  }

  public function pressure():Float return pressureState;

  function record(timestampNs:Int64):Void
    history.push(new SprayerEvent(flowState, pressureState, timestampNs));
}

/** One recorded `SimulatedSprayer` state change. */
class SprayerEvent {
  public final flow:Float;
  public final pressure:Float;
  public final timestampNs:Int64;

  public function new(flow:Float, pressure:Float, timestampNs:Int64) {
    this.flow = flow;
    this.pressure = pressure;
    this.timestampNs = timestampNs;
  }
}
