package machinekit.operator.wire;

import haxe.io.Bytes;

/** Generated from pendant_wire/schema/pendant.wire.idl. Do not edit by hand. */
class PendantEvent {
  public static inline final SIZE:Int = 12;
  public var sequence:Int = 0;
  public var axis:Int = 0;
  public var flags:Int = 0;
  public var distance:Int = 0;
  public var pressed:Int = 0;
  public var refusal:Int = 0;
  public var reason:Int = 0;
  public var reserved:Int = 0;
  public function new() {}

  /** Constraints declared in the schema; checked on encode and decode. */
  public function check():Void {
    if ((flags & ~3) != 0) throw "PendantEvent.flags violates its constraint";
    if ((pressed & ~63) != 0) throw "PendantEvent.pressed violates its constraint";
    if (!(refusal == 0 || refusal == 1 || refusal == 2 || refusal == 3 || refusal == 4 || refusal == 5 || refusal == 6)) throw "PendantEvent.refusal violates its constraint";
    if (reason < 0 || reason > 7) throw "PendantEvent.reason violates its constraint";
    if (reserved != 0) throw "PendantEvent.reserved violates its constraint";
  }

  public function encode():Bytes {
    check();
    var out = Bytes.alloc(SIZE);
    var offset = 0;
    out.set(offset + 0, (sequence >> 0) & 0xFF);
    out.set(offset + 1, (sequence >> 8) & 0xFF);
    offset += 2;
    out.set(offset + 0, (axis >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (flags >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (distance >> 0) & 0xFF);
    out.set(offset + 1, (distance >> 8) & 0xFF);
    out.set(offset + 2, (distance >> 16) & 0xFF);
    out.set(offset + 3, (distance >> 24) & 0xFF);
    offset += 4;
    out.set(offset + 0, (pressed >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (refusal >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (reason >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (reserved >> 0) & 0xFF);
    offset += 1;
    return out;
  }

  public static function decode(input:Bytes):PendantEvent {
    if (input == null || input.length != SIZE) throw "PendantEvent must be 12 bytes";
    var value = new PendantEvent();
    var offset = 0;
    var sequenceRaw = 0;
    sequenceRaw |= input.get(offset + 0) << 0;
    sequenceRaw |= input.get(offset + 1) << 8;
    value.sequence = sequenceRaw;
    offset += 2;
    var axisRaw = 0;
    axisRaw |= input.get(offset + 0) << 0;
    value.axis = axisRaw;
    offset += 1;
    var flagsRaw = 0;
    flagsRaw |= input.get(offset + 0) << 0;
    value.flags = flagsRaw;
    offset += 1;
    var distanceRaw = 0;
    distanceRaw |= input.get(offset + 0) << 0;
    distanceRaw |= input.get(offset + 1) << 8;
    distanceRaw |= input.get(offset + 2) << 16;
    distanceRaw |= input.get(offset + 3) << 24;
    value.distance = distanceRaw;
    offset += 4;
    var pressedRaw = 0;
    pressedRaw |= input.get(offset + 0) << 0;
    value.pressed = pressedRaw;
    offset += 1;
    var refusalRaw = 0;
    refusalRaw |= input.get(offset + 0) << 0;
    value.refusal = refusalRaw;
    offset += 1;
    var reasonRaw = 0;
    reasonRaw |= input.get(offset + 0) << 0;
    value.reason = reasonRaw;
    offset += 1;
    var reservedRaw = 0;
    reservedRaw |= input.get(offset + 0) << 0;
    value.reserved = reservedRaw;
    offset += 1;
    value.check();
    return value;
  }
}
