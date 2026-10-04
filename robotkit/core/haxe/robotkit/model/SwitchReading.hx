package robotkit.model;

/** Deterministic mechanical switch state with a distinct release threshold. */
class SwitchReading {
  public final source:JointSwitch;
  public var active(default, null):Bool = false;
  public var changed(default, null):Bool = false;
  /** Exact mechanical edge coordinate; null until an edge is observed. */
  public var edgePosition(default, null):Null<Float> = null;
  public var closingEdges(default, null):Int = 0;
  var state:Int;
  var variation:Float;
  var rearm:Bool = false;

  public function new(source:JointSwitch) {
    if (source == null) throw "Switch reading requires a physical switch";
    this.source = source;
    state = source.seed;
    variation = nextVariation();
  }

  /** Samples actual position, including mechanical slip, in the joint's SI units. */
  public function sample(position:Float):Bool {
    if (!Math.isFinite(position)) throw "Switch position must be finite";
    changed = false;
    var distance = source.side * (position - source.trip);
    // Choose the next repeatability error only after fully leaving the uncertainty
    // band, so repeated stationary samples cannot produce random chatter.
    if (rearm && distance < -source.hysteresis - source.repeatability) {
      variation = nextVariation();
      rearm = false;
    }
    if (!active && distance >= variation) {
      active = true; changed = true; closingEdges++;
      edgePosition = source.trip + source.side * variation;
    } else if (active && distance < variation - source.hysteresis) {
      active = false; changed = true; rearm = true;
      edgePosition = source.trip + source.side * (variation - source.hysteresis);
    }
    return active;
  }

  function nextVariation():Float {
    // Integer arithmetic gives a reproducible bounded sequence for an authored seed.
    state = (state * 1664525 + 1013904223) & 0x7fffffff;
    return source.repeatability * (2.0 * (state / 2147483647.0) - 1.0);
  }
}
