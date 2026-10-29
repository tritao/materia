package materia.project;

import haxe.io.Bytes;

/** The model's joint positions as a live display's driver reads its controls: in display units (degrees or millimetres). */
interface LiveInputs {
  /** The joint's position, or 0 when the model has no such joint. */
  public function position(joint:String):Float;

  /** Which of a clicking joint's positions it is at (0 for the first); 0 for a joint without detents. */
  public function detentIndex(joint:String):Int;

  /** True when a momentary button's joint is in the lower half of its travel. */
  public function pressed(joint:String):Bool;
}

/** What a model says about a display: the driver that fills it, the profile it serves and which joint feeds each of its signals. */
class LiveDisplayConfig {
  public final source:String;
  public final profile:String;
  /** The joint (an assembly joint id) that feeds each signal the driver reads, by the driver's signal name. */
  public final bindings:Map<String, String>;

  public function new(source:String, profile:String, bindings:Map<String, String>) {
    this.source = source;
    this.profile = profile;
    this.bindings = bindings;
  }

  public function joint(signal:String):String {
    var joint = bindings.get(signal);
    return joint == null ? "" : joint;
  }
}

/** What fills a display in the scene while a simulation runs: a picture that changes. */
interface LiveDisplaySource {
  public function width():Int;
  public function height():Int;

  /**
   * RGBA8, `width() * height() * 4` bytes, when the picture changed since the last call, else null. The editor calls this every
   * loop iteration, so a source whose picture changes with time (a dimming display, labels that arrive over time) is found out
   * when it changes. `nowMs` is the editor clock;
   * `inputs` are the model's joints, which the source reads through the bindings of its `LiveDisplayConfig`.
   */
  public function advance(nowMs:Int, inputs:LiveInputs):Null<Bytes>;
}

/** The drivers the editor can attach to a display, by the source name a model gives it (`SurfaceDetail.Display`). */
class LiveDisplays {
  static final drivers:Map<String, LiveDisplayConfig -> LiveDisplaySource> = [];

  /** Makes `source` available; a package that knows how to drive a kind of display calls this once. */
  public static function register(source:String, make:LiveDisplayConfig -> LiveDisplaySource):Void {
    drivers.set(source, make);
  }

  /** The driver for a display, or null when none is registered for its source. */
  public static function create(config:LiveDisplayConfig):Null<LiveDisplaySource> {
    var make = drivers.get(config.source);
    return make == null ? null : make(config);
  }
}
