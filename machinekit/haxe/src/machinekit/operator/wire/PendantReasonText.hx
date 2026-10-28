package machinekit.operator.wire;

import haxe.io.Bytes;

/** Generated from operator_link/schema/operator.wire.idl. Do not edit by hand. */
class PendantReasonText {
  public static inline final SIZE:Int = 24;
  public var index:Int = 0;
  public var text:Array<Int> = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
  public var reserved:Array<Int> = [0, 0];
  public function new() {
  }

  /** Constraints declared in the schema; checked on encode and decode. */
  public function check():Void {
    if (text.length != 21) throw "PendantReasonText.text has the wrong length";
    if (reserved.length != 2) throw "PendantReasonText.reserved has the wrong length";
    if (index < 0 || index > 7) throw "PendantReasonText.index violates its constraint";
    var textEnd = 0;
    while (textEnd < text.length && text[textEnd] != 0) textEnd++;
    for (i in 0...text.length) if (i < textEnd ? (text[i] < 32
      || text[i] > 126) : text[i] != 0) throw "PendantReasonText.text violates its constraint";
    for (v in reserved) if (v != 0) throw "PendantReasonText.reserved violates its constraint";
  }

  public function encode():Bytes {
    check();
    var out = Bytes.alloc(SIZE);
    var offset = 0;
    out.set(offset + 0, (index >> 0) & 0xFF);
    offset += 1;
    for (i in 0...21) {
      out.set(offset + 0, (text[i] >> 0) & 0xFF);
      offset += 1;
    }
    for (i in 0...2) {
      out.set(offset + 0, (reserved[i] >> 0) & 0xFF);
      offset += 1;
    }
    return out;
  }

  public static function decode(input:Bytes):PendantReasonText {
    if (input == null || input.length != SIZE) throw "PendantReasonText must be 24 bytes";
    var value = new PendantReasonText();
    var offset = 0;
    var indexRaw = 0;
    indexRaw |= input.get(offset + 0) << 0;
    value.index = indexRaw;
    offset += 1;
    for (i in 0...21) {
      var textRaw = 0;
      textRaw |= input.get(offset + 0) << 0;
      value.text[i] = textRaw;
      offset += 1;
    }
    for (i in 0...2) {
      var reservedRaw = 0;
      reservedRaw |= input.get(offset + 0) << 0;
      value.reserved[i] = reservedRaw;
      offset += 1;
    }
    value.check();
    return value;
  }
}
