package machinekit.operator;

/**
 * What the operator panel and the jog pendant need to know about one machine. The pendant itself
 * is identical on every machine: its selector positions are printed numbers and this profile gives
 * them meaning. Validation runs in the constructor, so no invalid profile exists.
 */
class OperatorProfile {
  public static inline final CURRENT_VERSION:Int = 1;
  /** Positions 1 to 11 of the pendant's axis switch. Position 0 is OFF. */
  public static inline final PENDANT_POSITIONS:Int = 11;
  public static inline final KEYS:Int = 3;

  public final name:String;
  public final axes:Array<OperatorAxis>;
  /** Axis id at pendant position i + 1, or null when that position is unused. */
  public final pendantAxes:Array<Null<String>>;
  public final keys:Array<OperatorKey>;
  public final modes:Array<OperatorMode>;
  public final hasSpindle:Bool;
  /** True when the machine has a powered door the operator can open and close from the pendant. */
  public final hasDoor:Bool;
  public final jogRules:Array<JogRule>;

  public function new(
    name:String,
    axes:Array<OperatorAxis>,
    pendantAxes:Array<Null<String>>,
    keys:Array<OperatorKey>,
    modes:Array<OperatorMode>,
    hasSpindle:Bool,
    hasDoor:Bool,
    jogRules:Array<JogRule>
  ) {
    requireId(name, "profile name");
    if (axes == null || axes.length == 0) throw "A profile needs at least one axis";
    var ids:Array<String> = [];
    for (axis in axes) {
      if (axis == null || ids.contains(axis.id)) throw "Axis ids must be unique";
      ids.push(axis.id);
    }
    if (pendantAxes == null || pendantAxes.length != PENDANT_POSITIONS) throw "The pendant axis map needs 11 positions";
    var mapped:Array<String> = [];
    for (id in pendantAxes) {
      if (id == null) continue;
      if (!ids.contains(id)) throw 'Pendant position names unknown axis $id';
      if (mapped.contains(id)) throw 'Axis $id appears twice on the pendant';
      mapped.push(id);
    }
    if (keys == null || keys.length != KEYS) throw "A profile needs exactly 3 function keys";
    for (key in keys) {
      if (key == null) throw "Missing function key";
      if (key.action == DoorToggle && !hasDoor) throw "A door key needs a door";
      if (key.action == SpindleJog && !hasSpindle) throw "A spindle key needs a spindle";
    }
    requireUnique(modes, "mode");
    if (mapped.length > 0 && !modes.contains(Jog)) throw "A pendant with axes needs the Jog mode";
    for (rule in jogRules) {
      if (rule == null) throw "Missing jog rule";
      if (!ids.contains(rule.axis)) throw 'Jog rule names unknown axis ${rule.axis}';
    }
    this.name = name;
    this.axes = axes.copy();
    this.pendantAxes = pendantAxes.copy();
    this.keys = keys.copy();
    this.modes = modes.copy();
    this.hasSpindle = hasSpindle;
    this.hasDoor = hasDoor;
    this.jogRules = jogRules.copy();
  }

  public function axis(id:String):OperatorAxis {
    for (a in axes) if (a.id == id) return a;
    throw 'Unknown axis $id';
  }

  /** Label the pendant's OLED shows for a switch position 1 to 11, or "" when unused. */
  public function pendantLabel(position:Int):String {
    if (position < 1 || position > PENDANT_POSITIONS) throw "Pendant position out of range";
    var id = pendantAxes[position - 1];
    return id == null ? "" : axis(id).label;
  }

  public static function requireId(value:Dynamic, what:String):Void {
    if (!Std.isOfType(value, String) || (value:String).length == 0) throw 'Invalid $what';
    for (i in 0...(value:String).length) {
      var c = (value:String).charCodeAt(i);
      var ok = (c >= 97 && c <= 122) || (c >= 48 && c <= 57) || c == 45 || c == 46 || c == 95;
      if (!ok) throw 'Invalid $what: lowercase letters, digits, - . _ only';
    }
  }

  /** Printable ASCII only, since the pendant OLED has one small font. */
  public static function requireLabel(value:Dynamic, max:Int, what:String):Void {
    if (!Std.isOfType(value,
      String) || (value:String).length == 0 || (value:String).length > max) throw 'Invalid $what';
    for (i in 0...(value:String).length) {
      var c = (value:String).charCodeAt(i);
      if (c < 32 || c > 126) throw 'Invalid $what: printable ASCII only';
    }
  }

  static function requireUnique<T>(values:Array<T>, what:String):Void {
    if (values == null || values.length == 0) throw 'A profile needs at least one $what';
    for (i in 0...values.length) for (j in i + 1...values.length) if (values[i] == values[j]) throw 'Duplicate $what';
  }
}
