package machinekit.operator.wire;

import haxe.io.Bytes;

/** Generated from operator_link/schema/operator.wire.idl. Do not edit by hand. */
class PendantProfile {
  public static inline final SIZE:Int = 54;
  public var axes:Array<Int> = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
  public var rule_count:Int = 0;
  public var rule_axis:Array<Int> = [0, 0, 0, 0, 0, 0, 0, 0];
  public var rule_condition:Array<Int> = [0, 0, 0, 0, 0, 0, 0, 0];
  public var rule_reason:Array<Int> = [0, 0, 0, 0, 0, 0, 0, 0];
  public var has_spindle:Int = 0;
  public var spindle_key:Int = 0;
  public var step_units:Array<Int> = [0, 0, 0];
  public var timeout_ms:Int = 0;
  public var host_timeout_ms:Int = 0;
  public function new() {
  }

  /** Constraints declared in the schema; checked on encode and decode. */
  public function check():Void {
    if (axes.length != 11) throw "PendantProfile.axes has the wrong length";
    if (rule_axis.length != 8) throw "PendantProfile.rule_axis has the wrong length";
    if (rule_condition.length != 8) throw "PendantProfile.rule_condition has the wrong length";
    if (rule_reason.length != 8) throw "PendantProfile.rule_reason has the wrong length";
    if (step_units.length != 3) throw "PendantProfile.step_units has the wrong length";
    if (rule_count < 0 || rule_count > 8) throw "PendantProfile.rule_count violates its constraint";
    for (v in rule_condition) if (v < 0 || v > 3) throw "PendantProfile.rule_condition violates its constraint";
    for (v in rule_reason) if (v < 0 || v > 7) throw "PendantProfile.rule_reason violates its constraint";
    if (has_spindle < 0 || has_spindle > 1) throw "PendantProfile.has_spindle violates its constraint";
    if (timeout_ms < 1 || timeout_ms > 1000) throw "PendantProfile.timeout_ms violates its constraint";
    if (host_timeout_ms < 1 || host_timeout_ms > 5000) throw "PendantProfile.host_timeout_ms violates its constraint";
  }

  public function encode():Bytes {
    check();
    var out = Bytes.alloc(SIZE);
    var offset = 0;
    for (i in 0...11) {
      out.set(offset + 0, (axes[i] >> 0) & 0xFF);
      offset += 1;
    }
    out.set(offset + 0, (rule_count >> 0) & 0xFF);
    offset += 1;
    for (i in 0...8) {
      out.set(offset + 0, (rule_axis[i] >> 0) & 0xFF);
      offset += 1;
    }
    for (i in 0...8) {
      out.set(offset + 0, (rule_condition[i] >> 0) & 0xFF);
      offset += 1;
    }
    for (i in 0...8) {
      out.set(offset + 0, (rule_reason[i] >> 0) & 0xFF);
      offset += 1;
    }
    out.set(offset + 0, (has_spindle >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (spindle_key >> 0) & 0xFF);
    offset += 1;
    for (i in 0...3) {
      out.set(offset + 0, (step_units[i] >> 0) & 0xFF);
      out.set(offset + 1, (step_units[i] >> 8) & 0xFF);
      out.set(offset + 2, (step_units[i] >> 16) & 0xFF);
      out.set(offset + 3, (step_units[i] >> 24) & 0xFF);
      offset += 4;
    }
    out.set(offset + 0, (timeout_ms >> 0) & 0xFF);
    out.set(offset + 1, (timeout_ms >> 8) & 0xFF);
    offset += 2;
    out.set(offset + 0, (host_timeout_ms >> 0) & 0xFF);
    out.set(offset + 1, (host_timeout_ms >> 8) & 0xFF);
    offset += 2;
    return out;
  }

  public static function decode(input:Bytes):PendantProfile {
    if (input == null || input.length != SIZE) throw "PendantProfile must be 54 bytes";
    var value = new PendantProfile();
    var offset = 0;
    for (i in 0...11) {
      var axesRaw = 0;
      axesRaw |= input.get(offset + 0) << 0;
      value.axes[i] = axesRaw;
      offset += 1;
    }
    var rule_countRaw = 0;
    rule_countRaw |= input.get(offset + 0) << 0;
    value.rule_count = rule_countRaw;
    offset += 1;
    for (i in 0...8) {
      var rule_axisRaw = 0;
      rule_axisRaw |= input.get(offset + 0) << 0;
      value.rule_axis[i] = rule_axisRaw;
      offset += 1;
    }
    for (i in 0...8) {
      var rule_conditionRaw = 0;
      rule_conditionRaw |= input.get(offset + 0) << 0;
      value.rule_condition[i] = rule_conditionRaw;
      offset += 1;
    }
    for (i in 0...8) {
      var rule_reasonRaw = 0;
      rule_reasonRaw |= input.get(offset + 0) << 0;
      value.rule_reason[i] = rule_reasonRaw;
      offset += 1;
    }
    var has_spindleRaw = 0;
    has_spindleRaw |= input.get(offset + 0) << 0;
    value.has_spindle = has_spindleRaw;
    offset += 1;
    var spindle_keyRaw = 0;
    spindle_keyRaw |= input.get(offset + 0) << 0;
    value.spindle_key = spindle_keyRaw;
    offset += 1;
    for (i in 0...3) {
      var step_unitsRaw = 0;
      step_unitsRaw |= input.get(offset + 0) << 0;
      step_unitsRaw |= input.get(offset + 1) << 8;
      step_unitsRaw |= input.get(offset + 2) << 16;
      step_unitsRaw |= input.get(offset + 3) << 24;
      value.step_units[i] = step_unitsRaw;
      offset += 4;
    }
    var timeout_msRaw = 0;
    timeout_msRaw |= input.get(offset + 0) << 0;
    timeout_msRaw |= input.get(offset + 1) << 8;
    value.timeout_ms = timeout_msRaw;
    offset += 2;
    var host_timeout_msRaw = 0;
    host_timeout_msRaw |= input.get(offset + 0) << 0;
    host_timeout_msRaw |= input.get(offset + 1) << 8;
    value.host_timeout_ms = host_timeout_msRaw;
    offset += 2;
    value.check();
    return value;
  }
}
