package machinekit.operator;

/** The pendant refuses to jog `axis` while the named service condition holds. */
class JogRule {
  public final axis:String;
  public final blockedWhen:String;
  public final reason:String;
  public function new(axis:String, blockedWhen:String, reason:String) {
    OperatorProfile.requireId(axis, "jog rule axis");
    OperatorProfile.requireId(blockedWhen, "jog rule condition");
    OperatorProfile.requireLabel(reason, 21, "jog rule reason");
    this.axis = axis;
    this.blockedWhen = blockedWhen;
    this.reason = reason;
  }
}
