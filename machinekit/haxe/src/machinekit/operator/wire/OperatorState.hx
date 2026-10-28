package machinekit.operator.wire;

import haxe.io.Bytes;

/** Generated from pendant_wire/schema/pendant.wire.idl. Do not edit by hand. */
class OperatorState {
  public static inline final SIZE:Int = 4;
  public var sequence:Int = 0;
  public var jog_allowed:Int = 0;
  public var reserved:Int = 0;
  public function new() {}

  /** Constraints declared in the schema; checked on encode and decode. */
  public function check():Void {
    if (jog_allowed < 0 || jog_allowed > 1) throw "OperatorState.jog_allowed violates its constraint";
    if (reserved != 0) throw "OperatorState.reserved violates its constraint";
  }

  public function encode():Bytes {
    check();
    var out = Bytes.alloc(SIZE);
    var offset = 0;
    out.set(offset + 0, (sequence >> 0) & 0xFF);
    out.set(offset + 1, (sequence >> 8) & 0xFF);
    offset += 2;
    out.set(offset + 0, (jog_allowed >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (reserved >> 0) & 0xFF);
    offset += 1;
    return out;
  }

  public static function decode(input:Bytes):OperatorState {
    if (input == null || input.length != SIZE) throw "OperatorState must be 4 bytes";
    var value = new OperatorState();
    var offset = 0;
    var sequenceRaw = 0;
    sequenceRaw |= input.get(offset + 0) << 0;
    sequenceRaw |= input.get(offset + 1) << 8;
    value.sequence = sequenceRaw;
    offset += 2;
    var jog_allowedRaw = 0;
    jog_allowedRaw |= input.get(offset + 0) << 0;
    value.jog_allowed = jog_allowedRaw;
    offset += 1;
    var reservedRaw = 0;
    reservedRaw |= input.get(offset + 0) << 0;
    value.reserved = reservedRaw;
    offset += 1;
    value.check();
    return value;
  }
}
