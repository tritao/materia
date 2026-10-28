package machinekit.operator;

import haxe.io.Bytes;
import machinekit.operator.wire.PendantConfig;
import machinekit.operator.wire.PendantProfile;
import machinekit.operator.wire.PendantReasonText;

/**
 * Turns an OperatorProfile into the integer form the cabinet controller runs. An axis becomes its
 * index in `profile.axes`, a service condition becomes its index in `profile.conditions` (a bit of
 * the active-condition mask the host sends), and a jog rule's reason text becomes an id into a
 * table of status lines. Units: `stepUnits` is the distance per wheel count for x1, x10 and x100
 * in the controller's integer position units.
 */
class PendantCompiler {
  public static inline final UNUSED:Int = 255;
  public static inline final MAX_RULES:Int = 8;
  public static inline final MAX_CONDITIONS:Int = 32;
  static inline final KIND_AXIS:Int = 0;
  static inline final KIND_KEY:Int = 1;

  public static function compile(p:OperatorProfile, stepUnits:Array<Int>, timeoutMs:Int):CompiledPendant {
    if (p == null) throw "A profile is required";
    if (stepUnits == null || stepUnits.length != 3) throw "Three step sizes are required";
    for (u in stepUnits) if (u <= 0) throw "Step sizes must be positive";
    if (timeoutMs < 1 || timeoutMs > 1000) throw "Timeout must be 1 to 1000 ms";
    if (p.axes.length >= UNUSED) throw "Too many axes for a pendant profile";
    if (p.conditions.length > MAX_CONDITIONS) throw "Too many service conditions";
    if (p.jogRules.length > MAX_RULES) throw "Too many jog rules";

    var record = new PendantProfile();
    var indexOf = function(id:String):Int {
      for (i in 0...p.axes.length) if (p.axes[i].id == id) return i;
      throw 'Unknown axis $id';
    };
    for (i in 0...OperatorProfile.PENDANT_POSITIONS) {
      var id = p.pendantAxes[i];
      record.axes[i] = id == null ? UNUSED : indexOf(id);
    }
    var reasons:Array<String> = [];
    record.rule_count = p.jogRules.length;
    for (i in 0...p.jogRules.length) {
      var rule = p.jogRules[i];
      var reason = reasons.indexOf(rule.reason);
      if (reason < 0) {
        reason = reasons.length;
        reasons.push(rule.reason);
      }
      if (reason >= MAX_RULES) throw "Too many distinct jog refusal texts";
      record.rule_axis[i] = indexOf(rule.axis);
      record.rule_condition[i] = p.conditions.indexOf(rule.blockedWhen);
      record.rule_reason[i] = reason;
    }
    record.has_spindle = p.hasSpindle ? 1 : 0;
    record.spindle_key = UNUSED;
    for (i in 0...p.keys.length) if (p.keys[i].action == SpindleJog) record.spindle_key = i;
    for (i in 0...3) record.step_units[i] = stepUnits[i];
    record.timeout_ms = timeoutMs;

    var texts:Array<Bytes> = [];
    for (i in 0...reasons.length) {
      var text = new PendantReasonText();
      text.index = i;
      fill(text.text, reasons[i]);
      texts.push(text.encode());
    }
    var labels:Array<Bytes> = [];
    for (position in 1...OperatorProfile.PENDANT_POSITIONS + 1) {
      var entry = new PendantConfig();
      entry.kind = KIND_AXIS;
      entry.index = position;
      fill(entry.text, p.pendantLabel(position));
      labels.push(entry.encode());
    }
    for (i in 0...OperatorProfile.KEYS) {
      var entry = new PendantConfig();
      entry.kind = KIND_KEY;
      entry.index = i;
      fill(entry.text, p.keys[i].label);
      labels.push(entry.encode());
    }
    return new CompiledPendant(record.encode(), texts, labels);
  }

  static function fill(target:Array<Int>, text:String):Void {
    for (i in 0...text.length) target[i] = text.charCodeAt(i);
  }
}
