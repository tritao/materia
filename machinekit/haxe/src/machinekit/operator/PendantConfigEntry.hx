package machinekit.operator;

/** One text the cabinet controller sends so the pendant can label itself from the profile. */
class PendantConfigEntry {
  public static inline final AXIS:Int = 0;
  public static inline final KEY:Int = 1;
  public static inline final TEXT_BYTES:Int = 12;
  public final kind:Int;
  /** Axis: switch position 1 to 11. Key: 0 to 2 for F1 to F3. */
  public final index:Int;
  /** Empty clears the label. */
  public final text:String;
  public function new(kind:Int, index:Int, text:String) {
    if (kind == AXIS) {
      if (index < 1 || index > OperatorProfile.PENDANT_POSITIONS) throw "Config axis position out of range";
    } else if (kind == KEY) {
      if (index < 0 || index >= OperatorProfile.KEYS) throw "Config key out of range";
    } else throw "Unknown config kind";
    if (text != "") OperatorProfile.requireLabel(text, kind == AXIS ? OperatorAxis.MAX_LABEL : OperatorKey.MAX_LABEL, "config text");
    this.kind = kind;
    this.index = index;
    this.text = text;
  }
}
