package machinekit.operator;

/** One of the three profile keys beside the fixed Start, Hold and Stop keys. */
class OperatorKey {
  public static inline final MAX_LABEL:Int = 6;
  public final action:KeyAction;
  public final label:String;
  public function new(action:KeyAction, label:String) {
    if (action == Unused) {
      if (label != "") throw "An unused key has no label";
    } else OperatorProfile.requireLabel(label, MAX_LABEL, "key label");
    this.action = action;
    this.label = label;
  }
}
