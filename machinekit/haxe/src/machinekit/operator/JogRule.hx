package machinekit.operator;

/** The pendant refuses to jog `axis` while the machine condition holds. */
class JogRule {
  public final axis:String;
  public final blockedWhen:OperatorCondition;
  public final reason:String;
  public function new(axis:String, blockedWhen:OperatorCondition, reason:String) {
    OperatorProfile.requireId(axis, "jog rule axis");
    OperatorProfile.requireLabel(reason, 21, "jog rule reason");
    this.axis = axis;
    this.blockedWhen = blockedWhen;
    this.reason = reason;
  }
}
