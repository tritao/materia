package machinekit.operator.wire;

import haxe.io.Bytes;

/** Generated from operator_link/schema/operator.wire.idl. Do not edit by hand. */
class PendantState {
  public static inline final SIZE:Int = 8;
  public var sequence:Int = 0;
  public var wheel_delta:Int = 0;
  public var position:Int = 0;
  public var step:Int = 0;
  public var buttons:Int = 0;
  public var flags:Int = 0;
  public function new() {
  }

  /** Constraints declared in the schema; checked on encode and decode. */
  public function check():Void {
    if (position < 0 || position > 11) throw "PendantState.position violates its constraint";
    if (step < 0 || step > 2) throw "PendantState.step violates its constraint";
    if ((buttons & ~63) != 0) throw "PendantState.buttons violates its constraint";
    if ((flags & ~3) != 0) throw "PendantState.flags violates its constraint";
  }

  public function encode():Bytes {
    check();
    var out = Bytes.alloc(SIZE);
    var offset = 0;
    out.set(offset + 0, (sequence >> 0) & 0xFF);
    out.set(offset + 1, (sequence >> 8) & 0xFF);
    offset += 2;
    out.set(offset + 0, (wheel_delta >> 0) & 0xFF);
    out.set(offset + 1, (wheel_delta >> 8) & 0xFF);
    offset += 2;
    out.set(offset + 0, (position >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (step >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (buttons >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (flags >> 0) & 0xFF);
    offset += 1;
    return out;
  }

  public static function decode(input:Bytes):PendantState {
    if (input == null || input.length != SIZE) throw "PendantState must be 8 bytes";
    var value = new PendantState();
    var offset = 0;
    var sequenceRaw = 0;
    sequenceRaw |= input.get(offset + 0) << 0;
    sequenceRaw |= input.get(offset + 1) << 8;
    value.sequence = sequenceRaw;
    offset += 2;
    var wheel_deltaRaw = 0;
    wheel_deltaRaw |= input.get(offset + 0) << 0;
    wheel_deltaRaw |= input.get(offset + 1) << 8;
    if (wheel_deltaRaw >= 32768) wheel_deltaRaw -= 65536;
    value.wheel_delta = wheel_deltaRaw;
    offset += 2;
    var positionRaw = 0;
    positionRaw |= input.get(offset + 0) << 0;
    value.position = positionRaw;
    offset += 1;
    var stepRaw = 0;
    stepRaw |= input.get(offset + 0) << 0;
    value.step = stepRaw;
    offset += 1;
    var buttonsRaw = 0;
    buttonsRaw |= input.get(offset + 0) << 0;
    value.buttons = buttonsRaw;
    offset += 1;
    var flagsRaw = 0;
    flagsRaw |= input.get(offset + 0) << 0;
    value.flags = flagsRaw;
    offset += 1;
    value.check();
    return value;
  }
}
