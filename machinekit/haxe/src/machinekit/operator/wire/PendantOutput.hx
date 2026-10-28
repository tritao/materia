package machinekit.operator.wire;

import haxe.io.Bytes;

/** Generated from pendant_wire/schema/pendant.wire.idl. Do not edit by hand. */
class PendantOutput {
  public static inline final SIZE:Int = 24;
  public var leds:Int = 0;
  public var lease:Int = 0;
  public var text:Array<Int> = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
  public var reserved:Int = 0;
  public function new() {}

  /** Constraints declared in the schema; checked on encode and decode. */
  public function check():Void {
    if (text.length != 21) throw "PendantOutput.text has the wrong length";
    if ((leds & ~63) != 0) throw "PendantOutput.leds violates its constraint";
    if (!(lease == 0 || lease == 1 || lease == 2)) throw "PendantOutput.lease violates its constraint";
    var textEnd = 0;
    while (textEnd < text.length && text[textEnd] != 0) textEnd++;
    for (i in 0...text.length) if (i < textEnd ? (text[i] < 32 || text[i] > 126) : text[i] != 0) throw "PendantOutput.text violates its constraint";
    if (reserved != 0) throw "PendantOutput.reserved violates its constraint";
  }

  public function encode():Bytes {
    check();
    var out = Bytes.alloc(SIZE);
    var offset = 0;
    out.set(offset + 0, (leds >> 0) & 0xFF);
    offset += 1;
    out.set(offset + 0, (lease >> 0) & 0xFF);
    offset += 1;
    for (i in 0...21) {
      out.set(offset + 0, (text[i] >> 0) & 0xFF);
      offset += 1;
    }
    out.set(offset + 0, (reserved >> 0) & 0xFF);
    offset += 1;
    return out;
  }

  public static function decode(input:Bytes):PendantOutput {
    if (input == null || input.length != SIZE) throw "PendantOutput must be 24 bytes";
    var value = new PendantOutput();
    var offset = 0;
    var ledsRaw = 0;
    ledsRaw |= input.get(offset + 0) << 0;
    value.leds = ledsRaw;
    offset += 1;
    var leaseRaw = 0;
    leaseRaw |= input.get(offset + 0) << 0;
    value.lease = leaseRaw;
    offset += 1;
    for (i in 0...21) {
      var textRaw = 0;
      textRaw |= input.get(offset + 0) << 0;
      value.text[i] = textRaw;
      offset += 1;
    }
    var reservedRaw = 0;
    reservedRaw |= input.get(offset + 0) << 0;
    value.reserved = reservedRaw;
    offset += 1;
    value.check();
    return value;
  }
}
