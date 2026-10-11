package processkit.moonraker;

/** One heater as Moonraker reports it, degrees Celsius. */
typedef MoonrakerHeater = {temperature:Float, target:Float};

/**
 * What a Klipper machine reports through Moonraker (`printer/objects/query`, or the `printer.objects.subscribe`
 * updates over the WebSocket): where the toolhead is, the heaters and the state of the print. Lengths are the machine's,
 * millimetres.
 *
 * The toolhead's position comes from `motion_report.live_position`, where the nozzle is now. `toolhead.position` is the
 * end of the last move queued, which runs ahead of the nozzle by everything Klipper has planned but not yet stepped (a
 * fraction of a second to seconds); it is kept apart as `queuedPosition` and a twin must not follow it.
 */
class MoonrakerStatus {
  /** `standby`, `printing`, `paused`, `complete`, `cancelled` or `error`. */
  public final state:String;
  public final filename:String;
  /** Fraction of the file printed, 0 to 1. */
  public final progress:Float;
  /** The nozzle's x, y and z and the extruder's position now (`motion_report.live_position`). */
  public final position:Array<Float>;
  /** The toolhead's speed and the extruder's, millimetres per second (`live_velocity`, `live_extruder_velocity`). */
  public final velocity:Float;
  public final extruderVelocity:Float;
  /** Where the last queued move ends (`toolhead.position`): ahead of `position`. */
  public final queuedPosition:Array<Float>;
  public final extruder:MoonrakerHeater;
  public final bed:MoonrakerHeater;
  public final message:String;

  public function new(state:String, filename:String, progress:Float, position:Array<Float>, velocity:Float, extruderVelocity:Float,
      queuedPosition:Array<Float>, extruder:MoonrakerHeater, bed:MoonrakerHeater, message:String) {
    this.state = state; this.filename = filename; this.progress = progress; this.position = position;
    this.velocity = velocity; this.extruderVelocity = extruderVelocity; this.queuedPosition = queuedPosition;
    this.extruder = extruder; this.bed = bed; this.message = message;
  }

  /** The objects and fields that `fromObjects` reads, as a subscription (`null` fields would mean all of an object's). */
  public static function subscription():Dynamic
    return {motion_report: ["live_position", "live_velocity", "live_extruder_velocity"], toolhead: ["position"],
      extruder: ["temperature", "target"], heater_bed: ["temperature", "target"],
      print_stats: ["state", "filename", "message"], virtual_sdcard: ["progress"]};

  /** The same asked of an HTTP `printer/objects/query`. */
  public static inline var QUERY:String = "/printer/objects/query?motion_report=live_position,live_velocity,live_extruder_velocity" +
    "&toolhead=position&extruder=temperature,target&heater_bed=temperature,target&print_stats=state,filename,message" +
    "&virtual_sdcard=progress";

  static function number(source:Dynamic, name:String):Float {
    var value:Dynamic = source == null ? null : Reflect.field(source, name);
    if (value == null) return 0.0;
    if (!Std.isOfType(value, Float) && !Std.isOfType(value, Int)) throw 'Moonraker field "$name" is not a number';
    return value;
  }

  static function text(source:Dynamic, name:String):String {
    var value:Dynamic = source == null ? null : Reflect.field(source, name);
    return value == null ? "" : Std.string(value);
  }

  static function coordinates(source:Dynamic, name:String):Array<Float> {
    var raw:Dynamic = source == null ? null : Reflect.field(source, name);
    var result = [0.0, 0.0, 0.0, 0.0];
    if (raw == null) return result;
    if (!Std.isOfType(raw, Array)) throw 'Moonraker field "$name" is not a list';
    var list:Array<Dynamic> = cast raw;
    for (index in 0...Std.int(Math.min(4, list.length))) {
      if (!Std.isOfType(list[index], Float) && !Std.isOfType(list[index], Int)) throw 'Moonraker field "$name" holds a non-number';
      result[index] = list[index];
    }
    return result;
  }

  /** Reads the `status` object of a query reply or the merged state of a subscription; what the machine left out reads as zero or empty. */
  public static function fromObjects(status:Dynamic):MoonrakerStatus {
    var report:Dynamic = Reflect.field(status, "motion_report"), toolhead:Dynamic = Reflect.field(status, "toolhead");
    var extruder:Dynamic = Reflect.field(status, "extruder"), bed:Dynamic = Reflect.field(status, "heater_bed");
    var stats:Dynamic = Reflect.field(status, "print_stats");
    return new MoonrakerStatus(text(stats, "state"), text(stats, "filename"), number(Reflect.field(status, "virtual_sdcard"), "progress"),
      coordinates(report, "live_position"), number(report, "live_velocity"), number(report, "live_extruder_velocity"),
      coordinates(toolhead, "position"),
      {temperature: number(extruder, "temperature"), target: number(extruder, "target")},
      {temperature: number(bed, "temperature"), target: number(bed, "target")}, text(stats, "message"));
  }

  /** Reads a reply to `QUERY`; a malformed reply is refused. */
  public static function parse(reply:Dynamic):MoonrakerStatus {
    var result:Dynamic = Reflect.field(reply, "result");
    var status:Dynamic = result == null ? null : Reflect.field(result, "status");
    if (status == null) throw "Moonraker status reply has no result.status";
    return fromObjects(status);
  }
}
