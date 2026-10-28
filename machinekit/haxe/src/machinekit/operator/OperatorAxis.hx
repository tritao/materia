package machinekit.operator;

/** One controllable axis or joint, named by the stable id the motion layer uses. */
class OperatorAxis {
  public static inline final MAX_LABEL:Int = 12;
  public final id:String;
  public final label:String;
  public function new(id:String, label:String) {
    OperatorProfile.requireId(id, "axis id");
    OperatorProfile.requireLabel(label, MAX_LABEL, "axis label");
    this.id = id;
    this.label = label;
  }
}
